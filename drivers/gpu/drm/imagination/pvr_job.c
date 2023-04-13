// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_drv.h"
#include "pvr_gem.h"
#include "pvr_hwrt.h"
#include "pvr_job.h"
#include "pvr_rogue_fwif.h"
#include "pvr_rogue_fwif_client.h"
#include "pvr_stream.h"
#include "pvr_stream_defs.h"

#include <drm/drm_gem.h>
#include <drm/drm_syncobj.h>
#include <linux/dma-fence.h>
#include <linux/dma-fence-array.h>
#include <linux/dma-fence-unwrap.h>
#include <linux/dma-resv.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/ww_mutex.h>
#include <linux/xarray.h>
#include <uapi/drm/pvr_drm.h>

static void release_fences(struct xarray *in_fences)
{
	struct dma_fence *fence;
	unsigned long id;

	xa_for_each(in_fences, id, fence)
		dma_fence_put(fence);

	xa_destroy(in_fences);
}

static void pvr_job_release(struct kref *kref)
{
	struct pvr_job *job = container_of(kref, struct pvr_job, ref_count);

	xa_erase(&job->pvr_dev->job_ids, job->id);

	pvr_hwrt_data_put(job->hwrt);
	pvr_context_put(job->ctx);

	if (job->deps.cur) {
		dma_fence_remove_callback(job->deps.cur, &job->deps.cb);
		dma_fence_put(job->deps.cur);
	}

	release_fences(&job->deps.non_native);
	release_fences(&job->deps.native);

	dma_fence_put(job->done_fence);
	kfree(job->cmd);
	kfree(job);
}

static void pvr_job_dep_cb(struct dma_fence *fence, struct dma_fence_cb *cb)
{
	struct pvr_job *job = container_of(cb, struct pvr_job, deps.cb);
	struct pvr_context *ctx = job->ctx;

	pvr_context_pending_job_event(ctx);
}

bool pvr_job_non_native_deps_done(struct pvr_job *job)
{
	/* The current fence is a native fence, meaning all non-native deps are done. */
	if (job->deps.cur && to_pvr_context_queue_fence(job->deps.cur))
		return true;

	/* No more non-native deps to wait on. */
	if (!job->deps.cur && xa_empty(&job->deps.non_native))
		return true;

	/* Current non-native fence is still unsignaled. */
	if (job->deps.cur && !dma_fence_is_signaled(job->deps.cur))
		return false;

	dma_fence_put(job->deps.cur);
	job->deps.cur = NULL;

	while (!xa_empty(&job->deps.non_native)) {
		struct dma_fence *next_dep;
		int ret;

		next_dep = xa_erase(&job->deps.non_native, job->deps.next_index++);
		ret = dma_fence_add_callback(next_dep, &job->deps.cb, pvr_job_dep_cb);
		if (!ret) {
			job->deps.cur = next_dep;
			break;
		}

		WARN_ON(ret != -ENOENT);
		dma_fence_put(next_dep);
	}

	/* Reset the index so it can be used to iterate over the native array. */
	if (xa_empty(&job->deps.non_native))
		job->deps.next_index = 0;

	return !job->deps.cur;
}

/**
 * pvr_job_evict_signaled_native_deps() - Evict signaled native deps
 * @job: Job to operate on.
 */
void pvr_job_evict_signaled_native_deps(struct pvr_job *job)
{
	struct dma_fence *fence = NULL;
	unsigned long index;

	if (job->deps.cur && dma_fence_is_signaled(job->deps.cur)) {
		if (!WARN_ON(!to_pvr_context_queue_fence(job->deps.cur)))
			job->deps.native_count--;

		dma_fence_put(job->deps.cur);
		job->deps.cur = NULL;
	}

	xa_for_each_start(&job->deps.native, index, fence, job->deps.next_index) {
		if (dma_fence_is_signaled(fence)) {
			xa_erase(&job->deps.native, index);
			dma_fence_put(fence);
			job->deps.native_count--;
		}
	}
}

/**
 * pvr_job_wait_first_non_signaled_native_dep() - Register a fence callback on the first
 *						  non-signaled native dep
 * @job: Job to operate on.
 *
 * Returns:
 *  * 0 on success,
 *  * or -ENOENT if there's no fence to wait on.
 */
int pvr_job_wait_first_non_signaled_native_dep(struct pvr_job *job)
{
	struct dma_fence *fence;
	unsigned long index;

	if (job->deps.cur)
		return 0;

	xa_for_each_start(&job->deps.native, index, fence, job->deps.next_index) {
		int err;

		xa_erase(&job->deps.native, index);
		err = dma_fence_add_callback(fence, &job->deps.cb, pvr_job_dep_cb);
		if (!err) {
			job->deps.cur = fence;
			job->deps.next_index = index + 1;
			return 0;
		}

		WARN_ON(err != -ENOENT);
		job->deps.native_count--;
		dma_fence_put(fence);
	}

	return -ENOENT;
}

struct pvr_cccb *
get_cccb(struct pvr_job *job)
{
	switch (job->type) {
	case DRM_PVR_JOB_TYPE_GEOMETRY:
		return &container_of(job->ctx, struct pvr_context_render, base)->ctx_geom.cccb;

	case DRM_PVR_JOB_TYPE_FRAGMENT:
		return &container_of(job->ctx, struct pvr_context_render, base)->ctx_frag.cccb;

	case DRM_PVR_JOB_TYPE_COMPUTE:
		return &container_of(job->ctx, struct pvr_context_compute, base)->cccb;

	case DRM_PVR_JOB_TYPE_TRANSFER_FRAG:
		return &container_of(job->ctx, struct pvr_context_transfer, base)->cccb;

	default:
		return NULL;
	}
}

struct pvr_context_queue *
get_ctx_queue(struct pvr_job *job)
{
	switch (job->type) {
	case DRM_PVR_JOB_TYPE_GEOMETRY:
		return &container_of(job->ctx, struct pvr_context_render, base)->ctx_geom.queue;

	case DRM_PVR_JOB_TYPE_FRAGMENT:
		return &container_of(job->ctx, struct pvr_context_render, base)->ctx_frag.queue;

	case DRM_PVR_JOB_TYPE_COMPUTE:
		return &container_of(job->ctx, struct pvr_context_compute, base)->queue;

	case DRM_PVR_JOB_TYPE_TRANSFER_FRAG:
		return &container_of(job->ctx, struct pvr_context_transfer, base)->queue;

	default:
		return NULL;
	}
}

static u32 get_ctx_fw_addr(struct pvr_job *job)
{
	struct pvr_fw_object *ctx_fw_obj = NULL;
	u32 ctx_fw_addr;

	switch (job->type) {
	case DRM_PVR_JOB_TYPE_GEOMETRY:
	case DRM_PVR_JOB_TYPE_FRAGMENT:
		ctx_fw_obj = container_of(job->ctx, struct pvr_context_render, base)->fw_obj;
		break;

	case DRM_PVR_JOB_TYPE_COMPUTE:
		ctx_fw_obj = container_of(job->ctx, struct pvr_context_compute, base)->fw_obj;
		break;

	case DRM_PVR_JOB_TYPE_TRANSFER_FRAG:
		ctx_fw_obj = container_of(job->ctx, struct pvr_context_transfer, base)->fw_obj;
		break;

	default:
		WARN_ON(1);
		return 0;
	}

	pvr_gem_get_fw_addr(ctx_fw_obj, &ctx_fw_addr);

	if (job->type == DRM_PVR_JOB_TYPE_FRAGMENT)
		ctx_fw_addr += offsetof(struct rogue_fwif_fwrendercontext, frag_context);

	return ctx_fw_addr;
}

/**
 * pvr_job_fits_in_cccb() - Check if a job fits in CCCB
 * @job: Job to check.
 *
 * Returns:
 *  * 0 on success,
 *  * or -E2BIG if the CCCB is too small to ever hold the commands for this job,
 *  * or -ENOMEM if the CCCB doesn't have enough memory to hold the commands for
 *    this job at the moment.
 */
int pvr_job_fits_in_cccb(struct pvr_job *job)
{
	/* One UFO for job done signaling, and one per remaining native fence. */
	u32 ufo_op_count = 1 + job->deps.native_count;

	u32 size = (pvr_cccb_get_size_of_cmd_with_hdr(sizeof(struct rogue_fwif_ufo)) *
		    ufo_op_count) +
		   pvr_cccb_get_size_of_cmd_with_hdr(job->cmd_len);
	struct pvr_cccb *cccb = get_cccb(job);

	return pvr_cccb_check_command_space(cccb, size);
}

void pvr_job_submit(struct pvr_job *job)
{
	struct pvr_context_queue *queue = get_ctx_queue(job);
	struct pvr_device *pvr_dev = job->pvr_dev;
	struct pvr_context_queue_fence *qfence;
	struct pvr_cccb *cccb = get_cccb(job);
	struct rogue_fwif_ufo queue_ufo;
	u32 ctx_fw_addr = get_ctx_fw_addr(job);
	struct dma_fence *fence;
	unsigned long index;
	int err;

	if (WARN_ON(!queue || !cccb))
		return;

	pvr_cccb_lock(cccb);

	spin_lock(&pvr_dev->active_contexts.lock);
	if (list_empty(&job->ctx->active_node))
		list_add_tail(&job->ctx->active_node, &pvr_dev->active_contexts.list);
	spin_lock(&queue->jobs.lock);
	list_move_tail(&job->node, &queue->jobs.in_flight);
	spin_unlock(&queue->jobs.lock);
	spin_unlock(&pvr_dev->active_contexts.lock);

	qfence = to_pvr_context_queue_fence(job->deps.cur);
	WARN_ON(job->deps.cur && !qfence);
	if (qfence) {
		pvr_gem_get_fw_addr(qfence->ctx->timeline_ufo.fw_obj, &queue_ufo.addr);
		queue_ufo.value = job->deps.cur->seqno;
		err = pvr_cccb_write_command_with_header(cccb, ROGUE_FWIF_CCB_CMD_TYPE_FENCE_PR,
							 sizeof(queue_ufo), &queue_ufo, 0, 0);
		if (WARN_ON(err))
			goto err_cccb_unlock_rollback;
	}

	xa_for_each(&job->deps.native, index, fence) {
		qfence = to_pvr_context_queue_fence(fence);
		if (WARN_ON(!qfence))
			continue;

		pvr_gem_get_fw_addr(qfence->ctx->timeline_ufo.fw_obj, &queue_ufo.addr);
		queue_ufo.value = fence->seqno;
		err = pvr_cccb_write_command_with_header(cccb, ROGUE_FWIF_CCB_CMD_TYPE_FENCE_PR,
							 sizeof(queue_ufo), &queue_ufo, 0, 0);
		if (WARN_ON(err))
			goto err_cccb_unlock_rollback;
	}

	/* Submit job to FW */
	err = pvr_cccb_write_command_with_header(cccb, job->fw_ccb_cmd_type, job->cmd_len, job->cmd,
						 job->id, job->id);
	if (WARN_ON(err))
		goto err_cccb_unlock_rollback;

	pvr_gem_get_fw_addr(queue->fence_ctx->timeline_ufo.fw_obj, &queue_ufo.addr);
	queue_ufo.value = job->done_fence->seqno;
	err = pvr_cccb_write_command_with_header(cccb, ROGUE_FWIF_CCB_CMD_TYPE_UPDATE,
						 sizeof(queue_ufo), &queue_ufo, 0, 0);
	if (WARN_ON(err))
		goto err_cccb_unlock_rollback;

	err = pvr_cccb_unlock_send_kccb_kick(pvr_dev, cccb, ctx_fw_addr, job->hwrt);
	if (WARN_ON(err))
		goto err_cccb_unlock_rollback;

	return;

err_cccb_unlock_rollback:
	spin_lock(&pvr_dev->active_contexts.lock);
	spin_lock(&queue->jobs.lock);
	list_move(&job->node, &queue->jobs.pending);
	spin_unlock(&queue->jobs.lock);
	if (!pvr_context_has_in_flight_jobs(job->ctx))
		list_del_init(&job->ctx->active_node);
	spin_unlock(&pvr_dev->active_contexts.lock);
	pvr_cccb_unlock_rollback(cccb);
}

static void pvr_job_push(struct pvr_job *job)
{
	struct pvr_context_queue *queue = get_ctx_queue(job);

	pvr_job_get(job);

	spin_lock(&queue->jobs.lock);
	list_add_tail(&job->node, &queue->jobs.pending);
	spin_unlock(&queue->jobs.lock);

	pvr_context_pending_job_event(job->ctx);
}

/**
 * pvr_job_put() - Release reference on job
 * @job: Target job.
 */
void
pvr_job_put(struct pvr_job *job)
{
	if (job)
		kref_put(&job->ref_count, pvr_job_release);
}

static int
pvr_check_sync_op(const struct drm_pvr_sync_op *sync_op)
{
	u8 handle_type;

	if (sync_op->flags & ~DRM_PVR_SYNC_OP_FLAGS_MASK)
		return -EINVAL;

	handle_type = sync_op->flags & DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_MASK;
	if (handle_type != DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_SYNCOBJ &&
	    handle_type != DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_TIMELINE_SYNCOBJ)
		return -EINVAL;

	if (handle_type == DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_SYNCOBJ &&
	    sync_op->value != 0)
		return -EINVAL;

	return 0;
}

/**
 * struct pvr_sync_signal - Object encoding a syncobj signal operation
 *
 * The job submission logic collects all signal operations in an array of
 * pvr_sync_signal objects. This array also serves as a cache to get the
 * latest dma_fence when multiple jobs are submitted at once, and one job
 * signals a syncobj point that's later waited on by a subsequent job.
 */
struct pvr_sync_signal {
	/** @handle: Handle of the syncobj to signal. */
	u32 handle;

	/** @point: Point to signal in the syncobj.
	 *
	 * Only relevant for timeline syncobjs.
	 */
	u64 point;

	/** @syncobj: Syncobj retrieved from the handle. */
	struct drm_syncobj *syncobj;

	/**
	 * @chain: Chain object used to link the new fence with the
	 *	   existing timeline syncobj.
	 *
	 * Should be zero when manipulating a regular syncobj.
	 */
	struct dma_fence_chain *chain;

	/**
	 * @fence: New fence object to attach to the syncobj.
	 *
	 * This pointer starts with the currently fence bound to
	 * the <handle,point> pair.
	 */
	struct dma_fence *fence;
};

static void
pvr_sync_signal_free(struct pvr_sync_signal *sig_sync)
{
	if (!sig_sync)
		return;

	drm_syncobj_put(sig_sync->syncobj);
	dma_fence_chain_free(sig_sync->chain);
	dma_fence_put(sig_sync->fence);
	kfree(sig_sync);
}

static void
pvr_sync_signal_array_cleanup(struct xarray *array)
{
	struct pvr_sync_signal *sig_sync;
	unsigned long i;

	xa_for_each(array, i, sig_sync)
		pvr_sync_signal_free(sig_sync);

	xa_destroy(array);
}

static struct pvr_sync_signal *
pvr_sync_signal_array_add(struct xarray *array, struct drm_file *file, u32 handle, u64 point)
{
	struct pvr_sync_signal *sig_sync;
	int err;
	u32 id;

	sig_sync = kzalloc(sizeof(*sig_sync), GFP_KERNEL);
	if (!sig_sync)
		return ERR_PTR(-ENOMEM);

	sig_sync->handle = handle;
	sig_sync->point = point;

	if (point > 0) {
		sig_sync->chain = dma_fence_chain_alloc();
		if (!sig_sync->chain) {
			err = -ENOMEM;
			goto err_free_sig_sync;
		}
	}

	sig_sync->syncobj = drm_syncobj_find(file, handle);
	if (!sig_sync->syncobj) {
		err = -EINVAL;
		goto err_free_sig_sync;
	}

	/* Retrieve the current fence attached to that point. It's
	 * perfectly fine to get a NULL fence here, it just means there's
	 * no fence attached to that point yet.
	 */
	drm_syncobj_find_fence(file, handle, point, 0, &sig_sync->fence);

	err = xa_alloc(array, &id, sig_sync, xa_limit_32b, GFP_KERNEL);
	if (err)
		goto err_free_sig_sync;

	return sig_sync;

err_free_sig_sync:
	pvr_sync_signal_free(sig_sync);
	return ERR_PTR(err);
}

static struct pvr_sync_signal *
pvr_sync_signal_array_search(struct xarray *array, u32 handle, u64 point)
{
	struct pvr_sync_signal *sig_sync;
	unsigned long i;

	xa_for_each(array, i, sig_sync) {
		if (handle == sig_sync->handle && point == sig_sync->point)
			return sig_sync;
	}

	return NULL;
}

static struct pvr_sync_signal *
pvr_sync_signal_array_get(struct xarray *array, struct drm_file *file, u32 handle, u64 point)
{
	struct pvr_sync_signal *sig_sync;

	sig_sync = pvr_sync_signal_array_search(array, handle, point);
	if (sig_sync)
		return sig_sync;

	return pvr_sync_signal_array_add(array, file, handle, point);
}

static int
pvr_sync_signal_array_collect_ops(struct xarray *array,
				  struct drm_file *file,
				  u32 sync_op_count,
				  struct drm_pvr_sync_op *sync_ops)
{
	for (u32 i = 0; i < sync_op_count; i++) {
		struct pvr_sync_signal *sig_sync;
		int ret;

		if (!(sync_ops[i].flags & DRM_PVR_SYNC_OP_FLAG_SIGNAL))
			continue;

		ret = pvr_check_sync_op(&sync_ops[i]);
		if (ret)
			return ret;

		sig_sync = pvr_sync_signal_array_get(array, file,
						     sync_ops[i].handle,
						     sync_ops[i].value);
		if (IS_ERR(sig_sync))
			return PTR_ERR(sig_sync);
	}

	return 0;
}

static int
pvr_sync_signal_array_update_fences(struct xarray *array,
				    u32 sync_op_count,
				    const struct drm_pvr_sync_op *sync_ops,
				    struct dma_fence *done_fence)
{
	for (u32 i = 0; i < sync_op_count; i++) {
		struct dma_fence *old_fence;
		struct pvr_sync_signal *sig_sync;

		if (!(sync_ops[i].flags & DRM_PVR_SYNC_OP_FLAG_SIGNAL))
			continue;

		sig_sync = pvr_sync_signal_array_search(array, sync_ops[i].handle,
							sync_ops[i].value);
		if (WARN_ON(!sig_sync))
			return -EINVAL;

		old_fence = sig_sync->fence;
		sig_sync->fence = dma_fence_get(done_fence);
		dma_fence_put(old_fence);

		if (!sig_sync->fence)
			return -EINVAL;
	}

	return 0;
}

static void
pvr_sync_signal_array_push_fences(struct xarray *array)
{
	struct pvr_sync_signal *sig_sync;
	unsigned long i;

	xa_for_each(array, i, sig_sync) {
		if (sig_sync->chain) {
			drm_syncobj_add_point(sig_sync->syncobj, sig_sync->chain,
					      sig_sync->fence, sig_sync->point);
			sig_sync->chain = NULL;
		} else {
			drm_syncobj_replace_fence(sig_sync->syncobj, sig_sync->fence);
		}
	}
}

/**
 * fence_array_add() - Adds the fence to an array of fences to be waited on,
 *                     deduplicating fences from the same context.
 * @fence_array: array of dma_fence * for the job to block on.
 * @fence: the dma_fence to add to the list of dependencies.
 *
 * This functions consumes the reference for @fence both on success and error
 * cases.
 *
 * Returns:
 *  * 0 on success, or an error on failing to expand the array.
 */
static int
fence_array_add(struct xarray *fence_array, struct dma_fence *fence)
{
	struct dma_fence *entry;
	unsigned long index;
	u32 id = 0;
	int ret;

	if (!fence)
		return 0;

	/* Deduplicate if we already depend on a fence from the same context.
	 * This lets the size of the array of deps scale with the number of
	 * engines involved, rather than the number of BOs.
	 */
	xa_for_each(fence_array, index, entry) {
		if (entry->context != fence->context)
			continue;

		if (dma_fence_is_later(fence, entry)) {
			dma_fence_put(entry);
			xa_store(fence_array, index, fence, GFP_KERNEL);
		} else {
			dma_fence_put(fence);
		}
		return 0;
	}

	ret = xa_alloc(fence_array, &id, fence, xa_limit_32b, GFP_KERNEL);
	if (ret != 0)
		dma_fence_put(fence);

	return ret;
}

static int
pvr_job_add_deps(struct pvr_file *pvr_file, struct pvr_job *job,
		 u32 sync_op_count, const struct drm_pvr_sync_op *sync_ops,
		 struct xarray *signal_array)
{
	struct dma_fence *fence;
	unsigned long index;
	int err = 0;

	if (!sync_op_count)
		return 0;

	for (u32 i = 0; i < sync_op_count; i++) {
		struct dma_fence *unwrapped_fence;
		struct pvr_sync_signal *sig_sync;
		struct dma_fence_unwrap iter;
		u32 native_fence_count = 0;

		if (sync_ops[i].flags & DRM_PVR_SYNC_OP_FLAG_SIGNAL)
			continue;

		err = pvr_check_sync_op(&sync_ops[i]);
		if (err)
			return err;

		sig_sync = pvr_sync_signal_array_search(signal_array, sync_ops[i].handle,
							sync_ops[i].value);
		if (sig_sync) {
			if (WARN_ON(!sig_sync->fence))
				return -EINVAL;

			fence = dma_fence_get(sig_sync->fence);
		} else {
			err = drm_syncobj_find_fence(from_pvr_file(pvr_file), sync_ops[i].handle,
						     sync_ops[i].value, 0, &fence);
			if (err)
				return err;
		}

		dma_fence_unwrap_for_each(unwrapped_fence, &iter, fence) {
			if (to_pvr_context_queue_fence(unwrapped_fence))
				native_fence_count++;
		}

		if (!native_fence_count) {
			/* No need to unwrap the fence if it's fully non-native. */
			err = fence_array_add(&job->deps.non_native, fence);
			if (err)
				return err;
		} else {
			dma_fence_unwrap_for_each(unwrapped_fence, &iter, fence) {
				/* There's no dma_fence_unwrap_stop() helper cleaning up the refs
				 * owned by dma_fence_unwrap(), so let's just iterate over all
				 * entries without doing anything when something failed.
				 */
				if (err)
					continue;

				dma_fence_get(unwrapped_fence);
				if (to_pvr_context_queue_fence(unwrapped_fence)) {
					err = fence_array_add(&job->deps.native, unwrapped_fence);
				} else {
					err = fence_array_add(&job->deps.non_native,
							      unwrapped_fence);
				}
				dma_fence_put(unwrapped_fence);
			}

			if (err)
				return err;
		}
	}

	xa_for_each(&job->deps.native, index, fence)
		job->deps.native_count++;

	return 0;
}

static int pvr_fw_cmd_init(struct pvr_device *pvr_dev, struct pvr_job *job,
			   const struct pvr_stream_cmd_defs *stream_def,
			   u64 stream_userptr, u32 stream_len)
{
	void *stream;
	int err;

	stream = kzalloc(stream_len, GFP_KERNEL);
	if (!stream) {
		err = -ENOMEM;
		goto err_out;
	}

	if (copy_from_user(stream, u64_to_user_ptr(stream_userptr), stream_len)) {
		err = -EFAULT;
		goto err_free_stream;
	}

	err = pvr_stream_process(pvr_dev, stream_def, stream, stream_len, job);

err_free_stream:
	kfree(stream);

err_out:
	return err;
}

static u32
convert_geom_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_GEOM_CMD_FIRST)
		out_flags |= ROGUE_GEOM_FLAGS_FIRSTKICK;
	if (in_flags & DRM_PVR_SUBMIT_JOB_GEOM_CMD_LAST)
		out_flags |= ROGUE_GEOM_FLAGS_LASTKICK;
	if (in_flags & DRM_PVR_SUBMIT_JOB_GEOM_CMD_SINGLE_CORE)
		out_flags |= ROGUE_GEOM_FLAGS_SINGLE_CORE;

	return out_flags;
}

static u32
convert_frag_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_SINGLE_CORE)
		out_flags |= ROGUE_FRAG_FLAGS_SINGLE_CORE;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_DEPTHBUFFER)
		out_flags |= ROGUE_FRAG_FLAGS_DEPTHBUFFER;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_STENCILBUFFER)
		out_flags |= ROGUE_FRAG_FLAGS_STENCILBUFFER;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_PREVENT_CDM_OVERLAP)
		out_flags |= ROGUE_FRAG_FLAGS_PREVENT_CDM_OVERLAP;
	if (in_flags & DRM_PVR_SUBMIT_JOB_FRAG_CMD_GET_VIS_RESULTS)
		out_flags |= ROGUE_FRAG_FLAGS_GET_VIS_RESULTS;

	return out_flags;
}

static int
pvr_geom_job_fw_cmd_init(struct pvr_job *job,
			 struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_geom *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_GEOM_CMD_FLAGS_MASK)
		return -EINVAL;

	if (!to_pvr_context_render(job->ctx))
		return -EINVAL;

	if (!job->hwrt)
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_GEOM;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_geom_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->cmd_shared.cmn.frame_num = 0;
	cmd->flags = convert_geom_flags(args->flags);
	pvr_gem_get_fw_addr(job->hwrt->fw_obj, &cmd->cmd_shared.hwrt_data_fw_addr);
	return 0;
}

static int
pvr_frag_job_fw_cmd_init(struct pvr_job *job,
			 struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_frag *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_FRAG_CMD_FLAGS_MASK)
		return -EINVAL;

	if (!to_pvr_context_render(job->ctx))
		return -EINVAL;

	if (!job->hwrt)
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_FRAG;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_frag_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->cmd_shared.cmn.frame_num = 0;
	cmd->flags = convert_frag_flags(args->flags);
	pvr_gem_get_fw_addr(job->hwrt->fw_obj, &cmd->cmd_shared.hwrt_data_fw_addr);
	return 0;
}

static u32
convert_compute_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_PREVENT_ALL_OVERLAP)
		out_flags |= ROGUE_COMPUTE_FLAG_PREVENT_ALL_OVERLAP;
	if (in_flags & DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_SINGLE_CORE)
		out_flags |= ROGUE_COMPUTE_FLAG_SINGLE_CORE;

	return out_flags;
}

static int
pvr_compute_job_fw_cmd_init(struct pvr_job *job,
			    struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_compute *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_FLAGS_MASK)
		return -EINVAL;

	if (!to_pvr_context_compute(job->ctx))
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_CDM;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_compute_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->common.frame_num = 0;
	cmd->flags = convert_compute_flags(args->flags);
	return 0;
}

static u32
convert_transfer_flags(u32 in_flags)
{
	u32 out_flags = 0;

	if (in_flags & DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_SINGLE_CORE)
		out_flags |= ROGUE_TRANSFER_FLAGS_SINGLE_CORE;

	return out_flags;
}

static int
pvr_transfer_job_fw_cmd_init(struct pvr_job *job,
			     struct drm_pvr_job *args)
{
	struct rogue_fwif_cmd_transfer *cmd;
	int err;

	if (args->flags & ~DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_FLAGS_MASK)
		return -EINVAL;

	if (!to_pvr_context_transfer_frag(job->ctx))
		return -EINVAL;

	job->fw_ccb_cmd_type = ROGUE_FWIF_CCB_CMD_TYPE_TQ_3D;
	err = pvr_fw_cmd_init(job->pvr_dev, job, &pvr_cmd_transfer_stream,
			      args->cmd_stream, args->cmd_stream_len);
	if (err)
		return err;

	cmd = job->cmd;
	cmd->common.frame_num = 0;
	cmd->flags = convert_transfer_flags(args->flags);
	return 0;
}

static int
pvr_job_fw_cmd_init(struct pvr_job *job,
		    struct drm_pvr_job *args)
{
	switch (args->type) {
	case DRM_PVR_JOB_TYPE_GEOMETRY:
		return pvr_geom_job_fw_cmd_init(job, args);

	case DRM_PVR_JOB_TYPE_FRAGMENT:
		return pvr_frag_job_fw_cmd_init(job, args);

	case DRM_PVR_JOB_TYPE_COMPUTE:
		return pvr_compute_job_fw_cmd_init(job, args);

	case DRM_PVR_JOB_TYPE_TRANSFER_FRAG:
		return pvr_transfer_job_fw_cmd_init(job, args);

	default:
		return -EINVAL;
	}
}

static struct pvr_job *
pvr_create_job(struct pvr_device *pvr_dev,
	       struct pvr_file *pvr_file,
	       struct drm_pvr_job *args,
	       struct xarray *signal_array)
{
	struct drm_pvr_sync_op *sync_ops = NULL;
	struct pvr_context_queue *queue;
	struct pvr_job *job = NULL;
	int err;

	if (!args->cmd_stream || !args->cmd_stream_len)
		return ERR_PTR(-EINVAL);

	if (args->type != DRM_PVR_JOB_TYPE_GEOMETRY &&
	    args->type != DRM_PVR_JOB_TYPE_FRAGMENT &&
	    (args->hwrt.set_handle || args->hwrt.data_index))
		return ERR_PTR(-EINVAL);

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job)
		return ERR_PTR(-ENOMEM);

	xa_init_flags(&job->deps.non_native, XA_FLAGS_ALLOC);
	xa_init_flags(&job->deps.native, XA_FLAGS_ALLOC);
	kref_init(&job->ref_count);
	job->pvr_dev = pvr_dev;
	job->type = args->type;

	err = xa_alloc(&pvr_dev->job_ids, &job->id, job, xa_limit_32b, GFP_KERNEL);
	if (err)
		goto err_put_job;

	err = PVR_UOBJ_GET_ARRAY(sync_ops, &args->sync_ops);
	if (err)
		goto err_put_job;

	err = pvr_sync_signal_array_collect_ops(signal_array, from_pvr_file(pvr_file),
						args->sync_ops.count, sync_ops);
	if (err)
		goto err_put_job;

	job->ctx = pvr_context_lookup(pvr_file, args->context_handle);
	if (!job->ctx) {
		err = -EINVAL;
		goto err_put_job;
	}

	if (args->hwrt.set_handle) {
		job->hwrt = pvr_hwrt_data_lookup(pvr_file, args->hwrt.set_handle,
						 args->hwrt.data_index);
		if (!job->hwrt) {
			err = -EINVAL;
			goto err_put_job;
		}
	}

	err = pvr_job_fw_cmd_init(job, args);
	if (err)
		goto err_put_job;

	/* Check if the job will ever fit in the CCCB. */
	err = pvr_job_fits_in_cccb(job);
	if (err == -E2BIG)
		goto err_put_job;

	err = pvr_job_add_deps(pvr_file, job, args->sync_ops.count, sync_ops, signal_array);
	if (err)
		goto err_put_job;

	queue = get_ctx_queue(job);
	if (!queue) {
		err = -EINVAL;
		goto err_put_job;
	}

	job->done_fence = pvr_context_queue_fence_create(queue);
	if (IS_ERR(job->done_fence)) {
		err = PTR_ERR(job->done_fence);
		job->done_fence = NULL;
		goto err_put_job;
	}

	err = pvr_sync_signal_array_update_fences(signal_array,
						  args->sync_ops.count, sync_ops,
						  job->done_fence);
	if (err)
		goto err_put_job;

	kvfree(sync_ops);
	return job;

err_put_job:
	kvfree(sync_ops);
	pvr_job_put(job);
	return ERR_PTR(err);
}

/**
 * pvr_submit_jobs() - Submit jobs to the GPU
 * @pvr_dev: Target PowerVR device.
 * @pvr_file: Pointer to PowerVR file structure.
 * @args: IOCTL arguments.
 *
 * This initial implementation is entirely synchronous; on return the GPU will
 * be idle. This will not be the case for future implementations.
 *
 * Returns:
 *  * 0 on success,
 *  * -%EFAULT if arguments can not be copied from user space,
 *  * -%EINVAL on invalid arguments, or
 *  * Any other error.
 */
int
pvr_submit_jobs(struct pvr_device *pvr_dev,
		struct pvr_file *pvr_file,
		struct drm_pvr_ioctl_submit_jobs_args *args)
{
	struct drm_pvr_job *jobs_args = NULL;
	DEFINE_XARRAY_ALLOC(signal_array);
	struct pvr_job **jobs = NULL;
	int err;

	if (!args->jobs.count)
		return -EINVAL;

	err = PVR_UOBJ_GET_ARRAY(jobs_args, &args->jobs);
	if (err)
		return err;

	jobs = kvmalloc_array(args->jobs.count, sizeof(*jobs), GFP_KERNEL | __GFP_ZERO);
	if (!jobs) {
		err = -ENOMEM;
		goto out_free_jobs_args;
	}

	for (u32 i = 0; i < args->jobs.count; i++) {
		jobs[i] = pvr_create_job(pvr_dev, pvr_file, &jobs_args[i], &signal_array);
		if (IS_ERR(jobs[i])) {
			err = PTR_ERR(jobs[i]);
			jobs[i] = NULL;
			goto out_free_jobs;
		}
	}

	for (u32 i = 0; i < args->jobs.count; i++)
		pvr_job_push(jobs[i]);

	pvr_sync_signal_array_push_fences(&signal_array);
	err = 0;

out_free_jobs:
	for (u32 i = 0; i < args->jobs.count; i++)
		pvr_job_put(jobs[i]);

	kvfree(jobs);

out_free_jobs_args:
	pvr_sync_signal_array_cleanup(&signal_array);
	return err;
}
