// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_cccb.h"
#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_drv.h"
#include "pvr_gem.h"
#include "pvr_job.h"
#include "pvr_rogue_fwif.h"
#include "pvr_rogue_fwif_common.h"
#include "pvr_rogue_fwif_resetframework.h"

#include <drm/drm_auth.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/xarray.h>

/* TODO: placeholder */
#define MAX_DEADLINE_MS 30000

#define CLEANUP_SLEEP_TIME_MS 20

#define CTX_COMPUTE_CCCB_SIZE_LOG2 15
#define CTX_FRAG_CCCB_SIZE_LOG2 15
#define CTX_GEOM_CCCB_SIZE_LOG2 15
#define CTX_TRANSFER_CCCB_SIZE_LOG2 15

static struct pvr_context *
queue_to_ctx(struct pvr_context_queue *queue)
{
	switch (queue->fence_ctx->type) {
	case PVR_CONTEXT_QUEUE_TYPE_GEOMETRY:
		return &container_of(queue, struct pvr_context_render, ctx_geom.queue)->base;

	case PVR_CONTEXT_QUEUE_TYPE_FRAGMENT:
		return &container_of(queue, struct pvr_context_render, ctx_frag.queue)->base;

	case PVR_CONTEXT_QUEUE_TYPE_COMPUTE:
		return &container_of(queue, struct pvr_context_compute, queue)->base;

	case PVR_CONTEXT_QUEUE_TYPE_TRANSFER:
		return &container_of(queue, struct pvr_context_transfer, queue)->base;

	default:
		return NULL;
	}
}

static void
pvr_context_queue_cancel_pending_jobs(struct pvr_context_queue *queue)
{
	struct pvr_job *job, *tmp_job;
	LIST_HEAD(cancel_jobs);

	spin_lock(&queue->jobs.lock);
	list_for_each_entry_safe(job, tmp_job, &queue->jobs.pending, node)
		list_move_tail(&job->node, &cancel_jobs);
	spin_unlock(&queue->jobs.lock);

	list_for_each_entry_safe(job, tmp_job, &cancel_jobs, node) {
		list_del(&job->node);
		dma_fence_set_error(job->done_fence, -ECANCELED);
		dma_fence_signal(job->done_fence);
		pvr_job_put(job);
	}
}

static void
pvr_context_queue_cancel_inflight_jobs(struct pvr_context_queue *queue)
{
	/* Signal in_flight job fences. We keep the jobs around to retain the
	 * context until the FW object backing this context is idle and ready
	 * for cleanup.
	 */
	while (true) {
		struct dma_fence *done_fence = NULL;
		struct pvr_job *job;
		unsigned long flags;

		spin_lock(&queue->jobs.lock);
		list_for_each_entry_reverse(job, &queue->jobs.in_flight, node) {
			/* Grab the fence, so it doesn't disappear after we released the lock. */
			if (!dma_fence_is_signaled(job->done_fence)) {
				done_fence = dma_fence_get(job->done_fence);
				break;
			}
		}
		spin_unlock(&queue->jobs.lock);

		if (!done_fence)
			break;

		spin_lock_irqsave(done_fence->lock, flags);

		/* pvr_context_queue_collect_done_jobs() might have signaled the fence
		 * when we get there, hence the is_signaled() check.:
		 */
		if (!dma_fence_is_signaled_locked(done_fence)) {
			dma_fence_set_error(done_fence, -ECANCELED);
			dma_fence_signal_locked(done_fence);
		}
		spin_unlock_irqrestore(done_fence->lock, flags);

		dma_fence_put(done_fence);
	}
}

static void
pvr_context_cancel_inflight_jobs(struct pvr_context *ctx)
{
	switch (ctx->type) {
	case DRM_PVR_CTX_TYPE_RENDER:
		pvr_context_queue_cancel_inflight_jobs(&to_pvr_context_render(ctx)->ctx_geom.queue);
		pvr_context_queue_cancel_inflight_jobs(&to_pvr_context_render(ctx)->ctx_frag.queue);
		break;

	case DRM_PVR_CTX_TYPE_COMPUTE:
		pvr_context_queue_cancel_inflight_jobs(&to_pvr_context_compute(ctx)->queue);
		break;

	case DRM_PVR_CTX_TYPE_TRANSFER_FRAG:
		pvr_context_queue_cancel_inflight_jobs(&to_pvr_context_transfer_frag(ctx)->queue);
		break;

	default:
		break;
	}
}

static int
pvr_context_queue_fw_cleanup(struct pvr_context_queue *queue)
{
	struct pvr_context *ctx = queue_to_ctx(queue);
	struct pvr_fw_object *fw_obj;
	u32 fw_obj_offset;

	switch (queue->fence_ctx->type) {
	case PVR_CONTEXT_QUEUE_TYPE_GEOMETRY:
		fw_obj = to_pvr_context_render(ctx)->fw_obj;
		fw_obj_offset = offsetof(struct rogue_fwif_fwrendercontext, geom_context);
		break;

	case PVR_CONTEXT_QUEUE_TYPE_FRAGMENT:
		fw_obj = to_pvr_context_render(ctx)->fw_obj;
		fw_obj_offset = offsetof(struct rogue_fwif_fwrendercontext, frag_context);
		break;

	case PVR_CONTEXT_QUEUE_TYPE_COMPUTE:
		fw_obj = to_pvr_context_compute(ctx)->fw_obj;
		fw_obj_offset = offsetof(struct rogue_fwif_fwcomputecontext, cdm_context);
		break;

	case PVR_CONTEXT_QUEUE_TYPE_TRANSFER:
		fw_obj = to_pvr_context_transfer_frag(ctx)->fw_obj;
		fw_obj_offset = offsetof(struct rogue_fwif_fwtransfercontext, tq_context);
		break;

	default:
		return -EINVAL;
	}

	return pvr_fw_structure_cleanup(ctx->pvr_dev,
					ROGUE_FWIF_CLEANUP_FWCOMMONCONTEXT,
					fw_obj, fw_obj_offset);
}

static int
pvr_context_fw_cleanup(struct pvr_context *ctx)
{
	struct pvr_context_render *render_ctx;
	int err;

	switch (ctx->type) {
	case DRM_PVR_CTX_TYPE_RENDER:
		render_ctx = to_pvr_context_render(ctx);

		err = pvr_context_queue_fw_cleanup(&render_ctx->ctx_geom.queue);
		if (err)
			pvr_context_queue_fw_cleanup(&render_ctx->ctx_frag.queue);
		else
			err = pvr_context_queue_fw_cleanup(&render_ctx->ctx_frag.queue);

		return err;

	case DRM_PVR_CTX_TYPE_COMPUTE:
		return pvr_context_queue_fw_cleanup(&to_pvr_context_compute(ctx)->queue);

	case DRM_PVR_CTX_TYPE_TRANSFER_FRAG:
		return pvr_context_queue_fw_cleanup(&to_pvr_context_transfer_frag(ctx)->queue);

	default:
		return -EINVAL;
	}
}

/**
 * pvr_context_queue_check_pending_jobs() - Check if any pending job can be submitted
 * @queue: Queue to check.
 *
 * Iterates over pending jobs, submitting ready jobs and stopping at the first non-ready one.
 * A job is ready when:
 * - all its dependencies are signaled
 * - the job commands fit in the CCCB
 */
static void
pvr_context_queue_check_pending_jobs(struct pvr_context_queue *queue)
{
	struct pvr_context *ctx = queue_to_ctx(queue);
	LIST_HEAD(cancel_jobs);

	while (!atomic_read(&ctx->destroyed)) {
		struct pvr_job *job;

		spin_lock(&queue->jobs.lock);
		job = list_first_entry_or_null(&queue->jobs.pending,
					       struct pvr_job, node);
		pvr_job_get(job);
		spin_unlock(&queue->jobs.lock);

		if (!job || !pvr_job_non_native_deps_done(job)) {
			pvr_job_put(job);
			break;
		}

		pvr_job_evict_signaled_native_deps(job);

		if (pvr_job_fits_in_cccb(job) &&
		    !pvr_job_wait_first_non_signaled_native_dep(job)) {
			pvr_job_put(job);
			break;
		}

		WARN_ON(pvr_job_fits_in_cccb(job));
		pvr_job_submit(job);
		pvr_job_put(job);
	}

	if (atomic_read(&ctx->destroyed))
		pvr_context_queue_cancel_pending_jobs(queue);
}

/**
 * pvr_context_job_pending_worker() - Worker called when a pending_job event is received
 * @work: Work struct.
 *
 * Check all queues bound to the context embedding the job_pending_work object, and
 * submit ready jobs if any.
 */
static void
pvr_context_job_pending_worker(struct work_struct *work)
{
	struct pvr_context *ctx = container_of(work, struct pvr_context, job_pending_work);

	switch (ctx->type) {
	case DRM_PVR_CTX_TYPE_RENDER:
		pvr_context_queue_check_pending_jobs(&to_pvr_context_render(ctx)->ctx_geom.queue);
		pvr_context_queue_check_pending_jobs(&to_pvr_context_render(ctx)->ctx_frag.queue);
		break;

	case DRM_PVR_CTX_TYPE_COMPUTE:
		pvr_context_queue_check_pending_jobs(&to_pvr_context_compute(ctx)->queue);
		break;

	case DRM_PVR_CTX_TYPE_TRANSFER_FRAG:
		pvr_context_queue_check_pending_jobs(&to_pvr_context_transfer_frag(ctx)->queue);
		break;

	default:
		break;
	}

	/* Release the reference that was taking in pvr_context_pending_job_event(). */
	pvr_context_put(ctx);
}

/**
 * pvr_context_pending_job_event() - Wrapper to queue a job_pending event
 * @ctx: Context to queue the event on.
 *
 * Queues the job_pending_work attached to the context.
 */
void pvr_context_pending_job_event(struct pvr_context *ctx)
{
	/* Grab a reference, and release it if the work was already queued. */
	pvr_context_get(ctx);
	if (!queue_work(ctx->pvr_dev->irq_wq, &ctx->job_pending_work))
		pvr_context_put(ctx);
}

static int
pvr_init_context_common(struct pvr_device *pvr_dev, struct pvr_file *pvr_file,
			struct pvr_context *ctx, int type,
			enum pvr_context_priority priority,
			struct drm_pvr_ioctl_create_context_args *args,
			u32 id)
{
	ctx->type = type;
	ctx->pvr_dev = pvr_dev;
	ctx->vm_ctx = pvr_vm_context_lookup(pvr_file, args->vm_context_handle);
	if (!ctx->vm_ctx)
		return -EINVAL;

	ctx->flags = args->flags;
	ctx->priority = priority;

	ctx->ctx_id = id;

	strscpy(ctx->process_name, current->comm, sizeof(ctx->process_name));

	kref_init(&ctx->ref_count);
	INIT_LIST_HEAD(&ctx->active_node);
	INIT_WORK(&ctx->job_pending_work, pvr_context_job_pending_worker);

	return 0;
}

static void
pvr_fini_context_common(struct pvr_device *pvr_dev, struct pvr_context *ctx)
{
	pvr_vm_context_put(ctx->vm_ctx);
}

/**
 * pvr_context_queue_fence_ctx_release() - Release callback for a queue fence context
 * @kref: The kref object being released.
 */
static void
pvr_context_queue_fence_ctx_release(struct kref *kref)
{
	struct pvr_context_queue_fence_ctx *ctx;

	ctx =  container_of(kref, struct pvr_context_queue_fence_ctx, refcount);
	pvr_fw_object_vunmap(ctx->timeline_ufo.fw_obj, false);
	pvr_fw_object_release(ctx->timeline_ufo.fw_obj);
	kfree(ctx);
	module_put(THIS_MODULE);
}

/**
 * pvr_context_queue_fence_ctx_create() - Create a queue fence context
 * @pvr_dev: The PowerVR device used to create this queue fence context.
 * @queue: The queue to create the fence context for.
 * @type: The type of queue being created.
 *
 * Return:
 *  * 0 on success, or
 *  * -ENOMEM if we fail to allocate memory or fail to acquire a module ref.
 */
static int
pvr_context_queue_fence_ctx_create(struct pvr_device *pvr_dev,
				   struct pvr_context_queue *queue,
				   enum pvr_context_queue_type type)
{
	struct pvr_context_queue_fence_ctx *ctx;
	void *cpu_map;
	int err;

	if (WARN_ON(!try_module_get(THIS_MODULE)))
		return -ENOENT;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		module_put(THIS_MODULE);
		return -ENOMEM;
	}

	cpu_map = pvr_gem_create_and_map_fw_object(pvr_dev, sizeof(*ctx->timeline_ufo.value),
						   PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
						   DRM_PVR_BO_CREATE_ZEROED,
						   &ctx->timeline_ufo.fw_obj);
	if (IS_ERR(cpu_map)) {
		err = PTR_ERR(cpu_map);
		goto err_free_ctx;
	}

	ctx->timeline_ufo.value = cpu_map;
	ctx->type = type;
	ctx->id = dma_fence_context_alloc(1);
	kref_init(&ctx->refcount);
	spin_lock_init(&ctx->lock);
	queue->fence_ctx = ctx;
	return 0;

err_free_ctx:
	kfree(ctx);
	return err;
}

static const char *
pvr_context_queue_fence_get_driver_name(struct dma_fence *f)
{
	return PVR_DRIVER_NAME;
}

static const char *
pvr_context_queue_fence_get_timeline_name(struct dma_fence *f)
{
	struct pvr_context_queue_fence *fence = to_pvr_context_queue_fence(f);

	switch (fence->ctx->type) {
	case PVR_CONTEXT_QUEUE_TYPE_GEOMETRY:
		return "geometry";

	case PVR_CONTEXT_QUEUE_TYPE_FRAGMENT:
		return "fragment";

	case PVR_CONTEXT_QUEUE_TYPE_COMPUTE:
		return "compute";

	case PVR_CONTEXT_QUEUE_TYPE_TRANSFER:
		return "transfer";

	default:
		return "invalid";
	}
}

static void pvr_context_queue_fence_release(struct dma_fence *f)
{
	struct pvr_context_queue_fence *fence = to_pvr_context_queue_fence(f);

	kref_put(&fence->ctx->refcount, pvr_context_queue_fence_ctx_release);
	dma_fence_free(f);
}

const struct dma_fence_ops pvr_context_queue_fence_ops = {
	.get_driver_name = pvr_context_queue_fence_get_driver_name,
	.get_timeline_name = pvr_context_queue_fence_get_timeline_name,
	.release = pvr_context_queue_fence_release,
};

struct pvr_context_queue_fence *
to_pvr_context_queue_fence(struct dma_fence *f)
{
	if (f && f->ops == &pvr_context_queue_fence_ops)
		return container_of(f, struct pvr_context_queue_fence, base);

	return NULL;
}

struct pvr_context_queue_fence_ctx *
pvr_context_queue_fence_ctx_from_fence(struct dma_fence *f)
{
	static struct pvr_context_queue_fence *fence;

	if (f->ops != &pvr_context_queue_fence_ops)
		return NULL;

	fence = to_pvr_context_queue_fence(f);
	return fence->ctx;
}

/**
 * pvr_context_queue_fence_create() - Create a queue fence object
 * @queue: The queue to create the fence on.
 *
 * Any job object should have a fence created with
 * pvr_context_queue_fence_create() attached to it. This fence will be signaled
 * when the job is done, or when something failed.
 *
 * Return:
 *  * A valid dma_fence pointer on success, or
 *  * ERR_PTR(-ENOMEM) if we fail to allocate memory.
 */
struct dma_fence *
pvr_context_queue_fence_create(struct pvr_context_queue *queue)
{
	struct pvr_context_queue_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	kref_get(&queue->fence_ctx->refcount);
	fence->ctx = queue->fence_ctx;
	dma_fence_init(&fence->base, &pvr_context_queue_fence_ops, &fence->ctx->lock,
		       fence->ctx->id, atomic_inc_return(&fence->ctx->seqno));

	return &fence->base;
}

/**
 * pvr_context_queue_init() - Initialize a context queue
 * @pvr_dev: A PowerVR device.
 * @queue: The queue object to initialize.
 * @type: The type of jobs taken by this queue.
 *
 * Return:
 *  * 0 on success, or
 *  * a negative error code when something failed.
 */
static int
pvr_context_queue_init(struct pvr_device *pvr_dev,
		       struct pvr_context_queue *queue,
		       enum pvr_context_queue_type type)
{
	int err;

	INIT_LIST_HEAD(&queue->jobs.pending);
	INIT_LIST_HEAD(&queue->jobs.in_flight);
	spin_lock_init(&queue->jobs.lock);

	err = pvr_context_queue_fence_ctx_create(pvr_dev, queue, type);
	if (err)
		return err;

	return 0;
}

/**
 * pvr_context_queue_fini() - Cleanup a context queue
 * @queue: The queue object to cleanup.
 */
static void
pvr_context_queue_fini(struct pvr_context_queue *queue)
{
	WARN_ON(!list_empty(&queue->jobs.pending));
	WARN_ON(!list_empty(&queue->jobs.in_flight));
	kref_put(&queue->fence_ctx->refcount, pvr_context_queue_fence_ctx_release);
}

/**
 * pvr_init_geom_context() - Initialise a geometry context
 * @pvr_file: Pointer to pvr_file structure.
 * @ctx_render: Pointer to parent render context.
 * @args: Arguments from userspace.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by pvr_gem_create_and_map_fw_object().
 */
static int
pvr_init_geom_context(struct pvr_file *pvr_file,
		      struct pvr_context_render *ctx_render,
		      struct drm_pvr_ioctl_create_context_args *args)
{
	struct pvr_device *pvr_dev = ctx_render->base.pvr_dev;
	struct pvr_context_geom *ctx_geom = &ctx_render->ctx_geom;
	struct rogue_fwif_geom_ctx_state *geom_ctx_state_fw;
	int err;

	err = pvr_cccb_init(pvr_dev, &ctx_geom->cccb, CTX_GEOM_CCCB_SIZE_LOG2, "geometry");
	if (err)
		goto err_out;

	geom_ctx_state_fw = pvr_gem_create_and_map_fw_object(pvr_dev, sizeof(*geom_ctx_state_fw),
							     PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							     DRM_PVR_BO_CREATE_ZEROED,
							     &ctx_geom->ctx_state_obj);
	if (IS_ERR(geom_ctx_state_fw)) {
		err = PTR_ERR(geom_ctx_state_fw);
		goto err_cccb_fini;
	}

	err = pvr_context_queue_init(pvr_dev, &ctx_geom->queue, PVR_CONTEXT_QUEUE_TYPE_GEOMETRY);
	if (err)
		goto err_release_ctx_state;

	geom_ctx_state_fw->geom_core[0].geom_reg_vdm_call_stack_pointer = args->callstack_addr;

	pvr_fw_object_vunmap(ctx_geom->ctx_state_obj, true);

	return 0;

err_release_ctx_state:
	pvr_fw_object_vunmap(ctx_geom->ctx_state_obj, false);
	pvr_fw_object_release(ctx_geom->ctx_state_obj);

err_cccb_fini:
	pvr_cccb_fini(&ctx_geom->cccb);

err_out:
	return err;
}

/**
 * pvr_fini_geom_context() - Clean up a geometry context
 * @ctx_render: Pointer to parent render context.
 */
static void
pvr_fini_geom_context(struct pvr_context_render *ctx_render)
{
	struct pvr_context_geom *ctx_geom = &ctx_render->ctx_geom;

	pvr_context_queue_fini(&ctx_geom->queue);
	pvr_fw_object_release(ctx_geom->ctx_state_obj);

	pvr_cccb_fini(&ctx_geom->cccb);
}

/**
 * pvr_init_frag_context() - Initialise a fragment context
 * @pvr_file: Pointer to pvr_file structure.
 * @ctx_render: Pointer to parent render context.
 * @args: Arguments from userspace.
 *
 * Return:
 *  * 0 on success.
 */
static int
pvr_init_frag_context(struct pvr_file *pvr_file,
		      struct pvr_context_render *ctx_render,
		      struct drm_pvr_ioctl_create_context_args *args)
{
	struct pvr_device *pvr_dev = ctx_render->base.pvr_dev;
	struct pvr_context_frag *ctx_frag = &ctx_render->ctx_frag;
	u32 num_isp_store_registers;
	size_t frag_ctx_state_size;
	int err;

	err = pvr_cccb_init(pvr_dev, &ctx_frag->cccb, CTX_FRAG_CCCB_SIZE_LOG2, "fragment");
	if (err)
		goto err_out;

	if (PVR_HAS_FEATURE(pvr_dev, xe_memory_hierarchy)) {
		WARN_ON(PVR_FEATURE_VALUE(pvr_dev, num_raster_pipes, &num_isp_store_registers));

		if (PVR_HAS_FEATURE(pvr_dev, gpu_multicore_support)) {
			u32 xpu_max_slaves;

			WARN_ON(PVR_FEATURE_VALUE(pvr_dev, xpu_max_slaves, &xpu_max_slaves));

			num_isp_store_registers *= (1 + xpu_max_slaves);
		}
	} else {
		WARN_ON(PVR_FEATURE_VALUE(pvr_dev, num_isp_ipp_pipes, &num_isp_store_registers));
	}

	frag_ctx_state_size = sizeof(struct rogue_fwif_frag_ctx_state) + num_isp_store_registers *
			      sizeof(((struct rogue_fwif_frag_ctx_state *)0)->frag_reg_isp_store[0]);

	err = pvr_gem_create_fw_object(pvr_dev, frag_ctx_state_size,
				       PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
				       DRM_PVR_BO_CREATE_ZEROED, &ctx_frag->ctx_state_obj);
	if (err)
		goto err_cccb_fini;

	err = pvr_context_queue_init(pvr_dev, &ctx_frag->queue, PVR_CONTEXT_QUEUE_TYPE_FRAGMENT);
	if (err)
		goto err_release_ctx_state;

	return 0;

err_release_ctx_state:
	pvr_fw_object_release(ctx_frag->ctx_state_obj);

err_cccb_fini:
	pvr_cccb_fini(&ctx_frag->cccb);

err_out:
	return err;
}

/**
 * pvr_fini_frag_context() - Clean up a fragment context
 * @ctx_render: Pointer to parent render context.
 */
static void
pvr_fini_frag_context(struct pvr_context_render *ctx_render)
{
	struct pvr_context_frag *ctx_frag = &ctx_render->ctx_frag;

	pvr_context_queue_fini(&ctx_frag->queue);
	pvr_fw_object_release(ctx_frag->ctx_state_obj);

	pvr_cccb_fini(&ctx_frag->cccb);
}

static int
remap_priority(struct pvr_file *pvr_file, s32 uapi_priority,
	       enum pvr_context_priority *priority_out)
{
	switch (uapi_priority) {
	case DRM_PVR_CTX_PRIORITY_LOW:
		*priority_out = PVR_CTX_PRIORITY_LOW;
		break;
	case DRM_PVR_CTX_PRIORITY_NORMAL:
		*priority_out = PVR_CTX_PRIORITY_MEDIUM;
		break;
	case DRM_PVR_CTX_PRIORITY_HIGH:
		if (!capable(CAP_SYS_NICE) && !drm_is_current_master(from_pvr_file(pvr_file)))
			return -EACCES;
		*priority_out = PVR_CTX_PRIORITY_HIGH;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * pvr_init_fw_common_context() - Initialise an FW-side common context structure
 * @ctx: Pointer to context.
 * @cctx_fw: Pointer to FW common context structure.
 * @dm_type: Data master type.
 * @priority: Context priority.
 * @max_deadline_ms: Maximum deadline for work on this context.
 * @cctx_id: Common context ID.
 * @ctx_state_obj: FW object representing context state.
 * @cccb: Client CCB for this context.
 */
static void
pvr_init_fw_common_context(struct pvr_context *ctx,
			   struct rogue_fwif_fwcommoncontext *cctx_fw,
			   u32 dm_type, u32 priority, u32 max_deadline_ms,
			   u32 cctx_id, struct pvr_fw_object *ctx_state_obj,
			   struct pvr_cccb *cccb)
{
	struct pvr_fw_object *fw_mem_ctx_obj = pvr_vm_get_fw_mem_context(ctx->vm_ctx);

	cctx_fw->ccbctl_fw_addr = cccb->ctrl_fw_addr;
	cctx_fw->ccb_fw_addr = cccb->cccb_fw_addr;

	cctx_fw->dm = dm_type;
	cctx_fw->priority = ctx->priority;
	cctx_fw->priority_seq_num = 0;
	cctx_fw->max_deadline_ms = max_deadline_ms;
	cctx_fw->pid = task_tgid_nr(current);
	cctx_fw->server_common_context_id = cctx_id;

	pvr_gem_get_fw_addr(fw_mem_ctx_obj, &cctx_fw->fw_mem_context_fw_addr);

	pvr_gem_get_fw_addr(ctx_state_obj, &cctx_fw->context_state_addr);
}

static void
pvr_fini_fw_common_context(struct pvr_context *ctx)
{
}

/**
 * pvr_init_fw_render_context() - Initialise an FW-side render context structure
 * @ctx_render: Pointer to parent render context.
 * @args: Context creation arguments from userspace.
 *
 * Return:
 *  * 0 on success.
 */
static int
pvr_init_fw_render_context(struct pvr_context_render *ctx_render,
			   struct drm_pvr_ioctl_create_context_args *args)
{
	struct rogue_fwif_static_rendercontext_state *static_rendercontext_state;
	struct rogue_fwif_fwrendercontext *fw_render_context;
	int err;

	if (args->static_context_state_len != sizeof(*static_rendercontext_state)) {
		err = -EINVAL;
		goto err_out;
	}

	fw_render_context = pvr_gem_create_and_map_fw_object(ctx_render->base.pvr_dev,
							     sizeof(*fw_render_context),
							     PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							     DRM_PVR_BO_CREATE_ZEROED,
							     &ctx_render->fw_obj);
	if (IS_ERR(fw_render_context)) {
		err = PTR_ERR(fw_render_context);
		goto err_out;
	}

	static_rendercontext_state = &fw_render_context->static_render_context_state;

	/* Copy static render context state from userspace. */
	if (copy_from_user(static_rendercontext_state, u64_to_user_ptr(args->static_context_state),
			   sizeof(*static_rendercontext_state))) {
		err = -EFAULT;
		goto err_destroy_gem_object;
	}

	pvr_init_fw_common_context(&ctx_render->base, &fw_render_context->geom_context,
				   PVR_FWIF_DM_GEOM, args->priority, MAX_DEADLINE_MS,
				   ctx_render->base.ctx_id, ctx_render->ctx_geom.ctx_state_obj,
				   &ctx_render->ctx_geom.cccb);

	pvr_init_fw_common_context(&ctx_render->base, &fw_render_context->frag_context,
				   PVR_FWIF_DM_FRAG, args->priority, MAX_DEADLINE_MS,
				   ctx_render->base.ctx_id, ctx_render->ctx_frag.ctx_state_obj,
				   &ctx_render->ctx_frag.cccb);

	pvr_fw_object_vunmap(ctx_render->fw_obj, true);
	return 0;

err_destroy_gem_object:
	pvr_fw_object_vunmap(ctx_render->fw_obj, true);
	pvr_fw_object_release(ctx_render->fw_obj);

err_out:
	return err;
}

/**
 * pvr_fini_fw_render_context() - Clean up an FW-side render context structure
 * @ctx_render: Pointer to parent render context.
 */
static void
pvr_fini_fw_render_context(struct pvr_context_render *ctx_render)
{
	struct pvr_context *ctx = from_pvr_context_render(ctx_render);

	pvr_fini_fw_common_context(ctx);
	pvr_fini_fw_common_context(ctx);

	pvr_fw_object_release(ctx_render->fw_obj);
}

/**
 * pvr_init_compute_context() - Initialise a compute context structure
 * @pvr_file: Pointer to pvr_file structure.
 * @ctx_compute: Pointer to parent compute context.
 * @args: Context creation arguments from userspace.
 *
 * Return:
 *  * 0 on success.
 */
static int
pvr_init_compute_context(struct pvr_file *pvr_file, struct pvr_context_compute *ctx_compute,
			 struct drm_pvr_ioctl_create_context_args *args)
{
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct rogue_fwif_cdm_registers_cswitch *ctxswitch_regs;
	struct rogue_fwif_fwcomputecontext *fw_compute_context;
	int err;

	if (args->static_context_state_len != sizeof(*ctxswitch_regs)) {
		err = -EINVAL;
		goto err_out;
	}

	err = pvr_cccb_init(pvr_dev, &ctx_compute->cccb, CTX_COMPUTE_CCCB_SIZE_LOG2, "compute");
	if (err)
		goto err_out;

	err = pvr_gem_create_fw_object(pvr_dev, sizeof(struct rogue_fwif_compute_ctx_state),
				       PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
				       DRM_PVR_BO_CREATE_ZEROED, &ctx_compute->ctx_state_obj);
	if (err)
		goto err_cccb_fini;

	fw_compute_context = pvr_gem_create_and_map_fw_object(ctx_compute->base.pvr_dev,
							      sizeof(*fw_compute_context),
							      PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							      DRM_PVR_BO_CREATE_ZEROED,
							      &ctx_compute->fw_obj);
	if (IS_ERR(fw_compute_context)) {
		err = PTR_ERR(fw_compute_context);
		goto err_destroy_ctx_state_obj;
	}

	ctxswitch_regs =
		&fw_compute_context->static_compute_context_state.ctxswitch_regs;

	/* Copy static compute context state from userspace. */
	if (copy_from_user(ctxswitch_regs,
			   u64_to_user_ptr(args->static_context_state),
			   sizeof(*ctxswitch_regs))) {
		err = -EFAULT;
		goto err_destroy_gem_object;
	}

	err = pvr_context_queue_init(pvr_dev, &ctx_compute->queue, PVR_CONTEXT_QUEUE_TYPE_COMPUTE);
	if (err)
		goto err_destroy_gem_object;

	pvr_init_fw_common_context(&ctx_compute->base, &fw_compute_context->cdm_context,
				   PVR_FWIF_DM_CDM, args->priority, MAX_DEADLINE_MS,
				   ctx_compute->base.ctx_id, ctx_compute->ctx_state_obj,
				   &ctx_compute->cccb);

	pvr_fw_object_vunmap(ctx_compute->fw_obj, true);
	return 0;

err_destroy_gem_object:
	pvr_fw_object_vunmap(ctx_compute->fw_obj, true);
	pvr_fw_object_release(ctx_compute->fw_obj);

err_destroy_ctx_state_obj:
	pvr_fw_object_release(ctx_compute->ctx_state_obj);

err_cccb_fini:
	pvr_cccb_fini(&ctx_compute->cccb);

err_out:
	return err;
}

/**
 * pvr_fini_compute_context() - Clean up a compute context structure
 * @ctx_compute: Pointer to compute context.
 */
static void
pvr_fini_compute_context(struct pvr_context_compute *ctx_compute)
{
	struct pvr_context *ctx = from_pvr_context_compute(ctx_compute);

	pvr_fini_fw_common_context(ctx);
	pvr_context_queue_fini(&ctx_compute->queue);
	pvr_fw_object_release(ctx_compute->fw_obj);
	pvr_fw_object_release(ctx_compute->ctx_state_obj);
	pvr_cccb_fini(&ctx_compute->cccb);
}

/**
 * pvr_init_transfer_context() - Initialise a transfer context structure
 * @pvr_file: Pointer to pvr_file structure.
 * @ctx_transfer: Pointer to parent transfer context.
 * @args: Context creation arguments from userspace.
 *
 * Return:
 *  * 0 on success.
 */
static int
pvr_init_transfer_context(struct pvr_file *pvr_file, struct pvr_context_transfer *ctx_transfer,
			  struct drm_pvr_ioctl_create_context_args *args)
{
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct rogue_fwif_fwtransfercontext *fw_transfer_context;
	size_t transfer_ctx_state_size;
	u32 num_isp_store_registers;
	int err;

	err = pvr_cccb_init(pvr_dev, &ctx_transfer->cccb, CTX_TRANSFER_CCCB_SIZE_LOG2,
			    "transfer_frag");
	if (err)
		goto err_out;

	if (PVR_HAS_FEATURE(pvr_dev, xe_memory_hierarchy))
		num_isp_store_registers = 1;
	else
		WARN_ON(PVR_FEATURE_VALUE(pvr_dev, num_isp_ipp_pipes, &num_isp_store_registers));

	transfer_ctx_state_size = sizeof(struct rogue_fwif_frag_ctx_state) +
				  num_isp_store_registers *
				  sizeof(((struct rogue_fwif_frag_ctx_state *)0)->frag_reg_isp_store[0]);

	err = pvr_gem_create_fw_object(pvr_dev, transfer_ctx_state_size,
				       PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
				       DRM_PVR_BO_CREATE_ZEROED, &ctx_transfer->ctx_state_obj);
	if (err)
		goto err_cccb_fini;

	fw_transfer_context = pvr_gem_create_and_map_fw_object(ctx_transfer->base.pvr_dev,
							       sizeof(*fw_transfer_context),
							       PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							       DRM_PVR_BO_CREATE_ZEROED,
							       &ctx_transfer->fw_obj);
	if (IS_ERR(fw_transfer_context)) {
		err = PTR_ERR(fw_transfer_context);
		goto err_destroy_ctx_state_obj;
	}

	err = pvr_context_queue_init(pvr_dev, &ctx_transfer->queue,
				     PVR_CONTEXT_QUEUE_TYPE_TRANSFER);
	if (err)
		goto err_destroy_ctx_state_obj;

	pvr_init_fw_common_context(&ctx_transfer->base, &fw_transfer_context->tq_context,
				   PVR_FWIF_DM_FRAG, args->priority, MAX_DEADLINE_MS,
				   ctx_transfer->base.ctx_id, ctx_transfer->ctx_state_obj,
				   &ctx_transfer->cccb);

	pvr_fw_object_vunmap(ctx_transfer->fw_obj, true);
	return 0;

err_destroy_ctx_state_obj:
	pvr_fw_object_release(ctx_transfer->ctx_state_obj);

err_cccb_fini:
	pvr_cccb_fini(&ctx_transfer->cccb);

err_out:
	return err;
}

/**
 * pvr_fini_transfer_context() - Clean up a transfer context structure
 * @ctx_transfer: Pointer to transfer context.
 */
static void
pvr_fini_transfer_context(struct pvr_context_transfer *ctx_transfer)
{
	struct pvr_context *ctx = from_pvr_context_transfer(ctx_transfer);

	pvr_fini_fw_common_context(ctx);
	pvr_context_queue_fini(&ctx_transfer->queue);
	pvr_fw_object_release(ctx_transfer->fw_obj);
	pvr_fw_object_release(ctx_transfer->ctx_state_obj);
	pvr_cccb_fini(&ctx_transfer->cccb);
}

/**
 * pvr_create_render_context() - Create a combination geometry/fragment render
 *                               context and return a handle
 * @pvr_file: Pointer to pvr_file structure.
 * @args: Creation arguments from userspace.
 * @id: FW context ID.
 *
 * The context is initialised with refcount of 1.
 *
 * Return:
 *  * Context pointer on success, or
 *  * -%ENOMEM on out-of-memory, or
 *  * Any error returned by xa_alloc().
 */
struct pvr_context *
pvr_create_render_context(struct pvr_file *pvr_file,
			  struct drm_pvr_ioctl_create_context_args *args,
			  u32 id)
{
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct pvr_context_render *ctx_render;
	enum pvr_context_priority priority;
	int err;

	if (!args->static_context_state) {
		err = -EINVAL;
		goto err_out;
	}

	err = remap_priority(pvr_file, args->priority, &priority);
	if (err)
		goto err_out;

	ctx_render = kzalloc(sizeof(*ctx_render), GFP_KERNEL);
	if (!ctx_render) {
		err = -ENOMEM;
		goto err_out;
	}

	err = pvr_init_context_common(pvr_dev, pvr_file,
				      from_pvr_context_render(ctx_render),
				      DRM_PVR_CTX_TYPE_RENDER, priority, args, id);
	if (err < 0)
		goto err_free;

	err = pvr_init_geom_context(pvr_file, ctx_render, args);
	if (err < 0)
		goto err_destroy_common_context;

	err = pvr_init_frag_context(pvr_file, ctx_render, args);
	if (err < 0)
		goto err_destroy_geom_context;

	err = pvr_init_fw_render_context(ctx_render, args);
	if (err < 0)
		goto err_destroy_frag_context;

	return from_pvr_context_render(ctx_render);

err_destroy_frag_context:
	pvr_fini_frag_context(ctx_render);

err_destroy_geom_context:
	pvr_fini_geom_context(ctx_render);

err_destroy_common_context:
	pvr_fini_context_common(pvr_dev, from_pvr_context_render(ctx_render));

err_free:
	kfree(ctx_render);

err_out:
	return ERR_PTR(err);
}

/**
 * pvr_create_compute_context() - Create a compute context and return a handle
 * @pvr_file: Pointer to pvr_file structure.
 * @args: Creation arguments from userspace.
 * @id: FW context ID.
 *
 * The context is initialised with refcount of 1.
 *
 * Return:
 *  * Context pointer on success, or
 *  * -%ENOMEM on out-of-memory, or
 *  * Any error returned by xa_alloc().
 */
struct pvr_context *
pvr_create_compute_context(struct pvr_file *pvr_file,
			   struct drm_pvr_ioctl_create_context_args *args,
			   u32 id)
{
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct pvr_context_compute *ctx_compute;
	enum pvr_context_priority priority;
	int err;

	if (!args->static_context_state || args->callstack_addr) {
		err = -EINVAL;
		goto err_out;
	}

	err = remap_priority(pvr_file, args->priority, &priority);
	if (err)
		goto err_out;

	ctx_compute = kzalloc(sizeof(*ctx_compute), GFP_KERNEL);
	if (!ctx_compute) {
		err = -ENOMEM;
		goto err_out;
	}

	err = pvr_init_context_common(pvr_dev, pvr_file,
				      from_pvr_context_compute(ctx_compute),
				      DRM_PVR_CTX_TYPE_COMPUTE, priority, args, id);
	if (err < 0)
		goto err_free;

	err = pvr_init_compute_context(pvr_file, ctx_compute, args);
	if (err < 0)
		goto err_destroy_common_context;

	return from_pvr_context_compute(ctx_compute);

err_destroy_common_context:
	pvr_fini_context_common(pvr_dev, from_pvr_context_compute(ctx_compute));

err_free:
	kfree(ctx_compute);

err_out:
	return ERR_PTR(err);
}

/**
 * pvr_create_transfer_context() - Create a transfer context and return a handle
 * @pvr_file: Pointer to pvr_file structure.
 * @args: Creation arguments from userspace.
 * @id: FW context ID.
 *
 * The context is initialised with refcount of 1.
 *
 * Return:
 *  * Context pointer on success, or
 *  * -%ENOMEM on out-of-memory, or
 *  * Any error returned by xa_alloc().
 */
struct pvr_context *
pvr_create_transfer_context(struct pvr_file *pvr_file,
			    struct drm_pvr_ioctl_create_context_args *args,
			    u32 id)
{
	struct pvr_device *pvr_dev = pvr_file->pvr_dev;
	struct pvr_context_transfer *ctx_transfer;
	enum pvr_context_priority priority;
	int err;

	if (args->callstack_addr || args->static_context_state) {
		err = -EINVAL;
		goto err_out;
	}

	err = remap_priority(pvr_file, args->priority, &priority);
	if (err)
		goto err_out;

	ctx_transfer = kzalloc(sizeof(*ctx_transfer), GFP_KERNEL);
	if (!ctx_transfer) {
		err = -ENOMEM;
		goto err_out;
	}

	err = pvr_init_context_common(pvr_dev, pvr_file,
				      from_pvr_context_transfer(ctx_transfer),
				      args->type, priority, args, id);
	if (err < 0)
		goto err_free;

	err = pvr_init_transfer_context(pvr_file, ctx_transfer, args);
	if (err < 0)
		goto err_destroy_common_context;

	return from_pvr_context_transfer(ctx_transfer);

err_destroy_common_context:
	pvr_fini_context_common(pvr_dev, from_pvr_context_transfer(ctx_transfer));

err_free:
	kfree(ctx_transfer);

err_out:
	return ERR_PTR(err);
}

static void
pvr_release_context(struct kref *ref_count)
{
	struct pvr_context *ctx =
		container_of(ref_count, struct pvr_context, ref_count);
	struct pvr_device *pvr_dev = ctx->pvr_dev;

	WARN_ON(pvr_context_fw_cleanup(ctx));

	if (WARN_ON(!list_empty(&ctx->active_node))) {
		spin_lock(&pvr_dev->active_contexts.lock);
		list_del_init(&ctx->active_node);
		spin_unlock(&pvr_dev->active_contexts.lock);
	}

	xa_erase(&pvr_dev->ctx_ids, ctx->ctx_id);

	if (ctx->type == DRM_PVR_CTX_TYPE_RENDER) {
		struct pvr_context_render *ctx_render = to_pvr_context_render(ctx);

		pvr_fini_fw_render_context(ctx_render);

		/* Destroy owned geometry & fragment contexts. */
		pvr_fini_frag_context(ctx_render);
		pvr_fini_geom_context(ctx_render);
	} else if (ctx->type == DRM_PVR_CTX_TYPE_COMPUTE) {
		struct pvr_context_compute *ctx_compute = to_pvr_context_compute(ctx);

		pvr_fini_compute_context(ctx_compute);
	} else if (ctx->type == DRM_PVR_CTX_TYPE_TRANSFER_FRAG) {
		struct pvr_context_transfer *ctx_transfer = to_pvr_context_transfer_frag(ctx);

		pvr_fini_transfer_context(ctx_transfer);
	}

	pvr_fini_context_common(ctx->pvr_dev, ctx);

	kfree(ctx);
}

/**
 * pvr_context_put() - Release reference on context
 * @ctx: Target context.
 */
void
pvr_context_put(struct pvr_context *ctx)
{
	if (ctx)
		kref_put(&ctx->ref_count, pvr_release_context);
}

/**
 * pvr_context_destroy() - Destroy context
 * @pvr_file: Pointer to pvr_file structure.
 * @handle: Userspace context handle.
 *
 * Removes context from context list and drops initial reference. Context will
 * then be destroyed once all outstanding references are dropped.
 *
 * Return:
 *  * 0 on success, or
 *  * -%EINVAL if context not in context list.
 */
int
pvr_context_destroy(struct pvr_file *pvr_file, u32 handle)
{
	struct pvr_context *ctx = xa_erase(&pvr_file->ctx_handles, handle);

	if (!ctx)
		return -EINVAL;

	/* Flag as destroyed so no jobs are submitted. */
	atomic_set(&ctx->destroyed, 1);

	/* Cancel in-flight jobs. This is not actually cancelling the jobs, just
	 * signaling done fences attached to them.
	 */
	pvr_context_cancel_inflight_jobs(ctx);

	/* Trigger pending job processing to cancel all pending jobs. */
	pvr_context_pending_job_event(ctx);

	/* Release the reference held by the handle set. */
	pvr_context_put(ctx);

	return 0;
}

/**
 * pvr_destroy_contexts_for_file: Destroy any contexts associated with the given file
 * @pvr_file: Pointer to pvr_file structure.
 *
 * Removes all contexts associated with @pvr_file from the device context list and drops initial
 * references. Contexts will then be destroyed once all outstanding references are dropped.
 */
void pvr_destroy_contexts_for_file(struct pvr_file *pvr_file)
{
	struct pvr_context *ctx;
	unsigned long handle;

	xa_for_each(&pvr_file->ctx_handles, handle, ctx)
		pvr_context_destroy(pvr_file, handle);
}

/**
 * pvr_context_queue_collect_done_jobs() - Collect all done jobs and add them to the list
 * @queue: Queue to collect done jobs on.
 * @done_jobs: List to queue these done jobs to.
 *
 * Collect all jobs whose sequence number is below the current timeline UFO sequence.
 */
static void
pvr_context_queue_collect_done_jobs(struct pvr_context_queue *queue, struct list_head *done_jobs)
{
	struct pvr_job *job, *tmp_job;
	u32 cur_seqno;

	spin_lock(&queue->jobs.lock);
	cur_seqno = *queue->fence_ctx->timeline_ufo.value;
	list_for_each_entry_safe(job, tmp_job, &queue->jobs.in_flight, node) {
		if (cur_seqno < job->done_fence->seqno)
			break;

		list_move_tail(&job->node, done_jobs);
	}
	spin_unlock(&queue->jobs.lock);
}

/**
 * pvr_context_queue_collect_done_jobs() - Collect all done jobs and add them to the list
 * @context: Context to collect done jobs on.
 * @done_jobs: List to queue these done jobs to.
 *
 * Collect all jobs on all queues belonging to this context.
 */
static void
pvr_context_collect_done_jobs(struct pvr_context *ctx, struct list_head *done_jobs)
{
	switch (ctx->type) {
	case DRM_PVR_CTX_TYPE_RENDER:
		pvr_context_queue_collect_done_jobs(&to_pvr_context_render(ctx)->ctx_geom.queue,
						    done_jobs);
		pvr_context_queue_collect_done_jobs(&to_pvr_context_render(ctx)->ctx_frag.queue,
						    done_jobs);
		break;

	case DRM_PVR_CTX_TYPE_COMPUTE:
		pvr_context_queue_collect_done_jobs(&to_pvr_context_compute(ctx)->queue,
						    done_jobs);
		break;

	case DRM_PVR_CTX_TYPE_TRANSFER_FRAG:
		pvr_context_queue_collect_done_jobs(&to_pvr_context_transfer_frag(ctx)->queue,
						    done_jobs);
		break;

	default:
		break;
	}
}

/**
 * pvr_context_process_worker() - Called to process context updates when a FW interrupt is received
 * @work: Work struct embedded in the context object.
 *
 * Right now, we simply iterate over all active contexts, collect done jobs and signal the
 * fences attached to them. If more context processing is needed at some point, it can be done
 * here as well.
 */
static void
pvr_context_process_worker(struct work_struct *work)
{
	struct pvr_device *pvr_dev = container_of(work, struct pvr_device, context_work);
	struct pvr_context *ctx, *tmp_ctx;
	struct pvr_job *job, *tmp_job;
	LIST_HEAD(done_jobs);

	spin_lock(&pvr_dev->active_contexts.lock);
	list_for_each_entry_safe(ctx, tmp_ctx, &pvr_dev->active_contexts.list, active_node) {
		pvr_context_collect_done_jobs(ctx, &done_jobs);
		if (!pvr_context_has_in_flight_jobs(ctx))
			list_del_init(&ctx->active_node);
	}
	spin_unlock(&pvr_dev->active_contexts.lock);

	list_for_each_entry_safe(job, tmp_job, &done_jobs, node) {
		list_del(&job->node);
		dma_fence_signal(job->done_fence);
		pvr_job_put(job);
	}
}

/**
 * pvr_context_has_in_flight_jobs() - Check if a context has in-flight jobs
 * @ctx: Context to check.
 *
 * Check if a context has in-flight jobs. Must be called with the
 * active_contexts.lock held, since any modification to the in_flight list
 * is protected by this lock.
 *
 * Returns:
 *  * %true if the context has in-flight jobs, or
 *  * %false otherwise.
 */
bool pvr_context_has_in_flight_jobs(struct pvr_context *ctx)
{
	lockdep_assert_held(&ctx->pvr_dev->active_contexts.lock);

	switch (ctx->type) {
	case DRM_PVR_CTX_TYPE_RENDER:
		return !list_empty(&to_pvr_context_render(ctx)->ctx_geom.queue.jobs.in_flight) ||
		       !list_empty(&to_pvr_context_render(ctx)->ctx_frag.queue.jobs.in_flight);

	case DRM_PVR_CTX_TYPE_COMPUTE:
		return !list_empty(&to_pvr_context_compute(ctx)->queue.jobs.in_flight);

	case DRM_PVR_CTX_TYPE_TRANSFER_FRAG:
		return !list_empty(&to_pvr_context_transfer_frag(ctx)->queue.jobs.in_flight);

	default:
		return false;
	}
}

/**
 * pvr_context_device_init() - Context-related device initialization
 * @pvr_dev: Device object being initialized.
 *
 * Initializes the active_contexts data and the context_work. More context-related
 * initialization can be added here if needed.
 */
void pvr_context_device_init(struct pvr_device *pvr_dev)
{
	spin_lock_init(&pvr_dev->active_contexts.lock);
	INIT_LIST_HEAD(&pvr_dev->active_contexts.list);
	INIT_WORK(&pvr_dev->context_work, pvr_context_process_worker);
}
