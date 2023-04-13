/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_CONTEXT_H__
#define __PVR_CONTEXT_H__

#include <linux/compiler_attributes.h>
#include <linux/dma-fence.h>
#include <linux/kref.h>
#include <linux/types.h>
#include <linux/xarray.h>
#include <uapi/drm/pvr_drm.h>

#include "pvr_cccb.h"
#include "pvr_device.h"

/* Forward declaration from pvr_gem.h. */
struct pvr_fw_object;

enum pvr_context_queue_type {
	PVR_CONTEXT_QUEUE_TYPE_GEOMETRY,
	PVR_CONTEXT_QUEUE_TYPE_FRAGMENT,
	PVR_CONTEXT_QUEUE_TYPE_COMPUTE,
	PVR_CONTEXT_QUEUE_TYPE_TRANSFER,
};

/**
 * struct pvr_context_queue_fence_ctx - Queue fence context
 *
 * Used to implement dma_fence_ops. The fence context and the queue are kept
 * separate even though there's a 1:1 mapping between them. We do that so
 * we can release the queue and its resource even if some external users
 * have references to dma_fence objects allocated here. In that case, all fences
 * should have been signaled by the driver before the queue is destroyed, and the
 * fence context will fade away as soon as all references to queue fences allocated
 * by this context are released.
 */
struct pvr_context_queue_fence_ctx {
	/**
	 * @refcount: Tracks the number of active users of this fence context.
	 *
	 * The queue owns a ref, as well as any dma_fence object allocated from this
	 * context. This allows us to get a module reference and avoids any module
	 * removal while external users still own a ref to the fence, which would
	 * cause access to freed memory since the dma_fence_ops are part of this
	 * module, even though the fence object is allocated dynamically.
	 */
	struct kref refcount;

	/** @type: Queue type. Used to implement get_timeline_name(). */
	enum pvr_context_queue_type type;

	/** @id: Context ID allocated with dma_fence_context_alloc(). */
	u64 id;

	/** @seqno: Sequence number incremented each time a fence is created. */
	atomic_t seqno;

	/** @lock: Lock used to synchronize access to fences allocated by this context. */
	spinlock_t lock;

	/** @timeline_ufo: Timeline UFO for the context queue. */
	struct {
		 /** @fw_obj: FW object representing the UFO value. */
		struct pvr_fw_object *fw_obj;

		/** @value: CPU mapping of the UFO value. */
		u32 *value;
	} timeline_ufo;
};

/**
 * struct pvr_context_queue_fence() - Queue fence object
 *
 * Allocated anytime a job is created. This fence object will be signaled when the
 * underlying timeline UFO reaches the sequence number specified in this object. No
 * direct access to the UFO object here, we do software signaling instead, which
 * simplifies things a bit.
 *
 * If there's any need to add native UFO support, we can extend the
 * pvr_context_queue_fence_ctx object to point to the queue, so we get access to
 * the UFO, and can queue UFO waits directly at the CCCB level, thus killing some
 * CPU waits done in pvr_job_deps_done(). Note that we will still need
 * pvr_job_deps_done() to deal with non-UFO fences, so not sure it is worth the
 * extra complexity.
 */
struct pvr_context_queue_fence {
	/** @base: Base dma_fence object. */
	struct dma_fence base;

	/** @ctx: Fence context that created this fence object. */
	struct pvr_context_queue_fence_ctx *ctx;
};

struct pvr_context_queue_fence *to_pvr_context_queue_fence(struct dma_fence *f);

/**
 * struct pvr_context_queue - Queue object.
 *
 * This is where we keep track of jobs that are or will be submitted to the CCCB.
 * We also have a timeline UFO per queue to determine when jobs are done.
 */
struct pvr_context_queue {
	/** @fence_ctx: Internal fence context object. */
	struct pvr_context_queue_fence_ctx *fence_ctx;

	/** @jobs: List of jobs queued to this context queue and the lock protecting it. */
	struct {
		/** @lock: Lock protecting the in_fight and pending lists. */
		spinlock_t lock;

		/** @in_flight: List of in-flight jobs, waiting for signaling. */
		struct list_head in_flight;

		/**
		 * @pendings: List of jobs pending jobs.
		 *
		 * Jobs in there are waiting for their dependencies to be signaled or for
		 * some space in the CCCB to queue their commands.
		 */
		struct list_head pending;
	} jobs;
};

/**
 * struct pvr_context_geom - Geometry render context data
 */
struct pvr_context_geom {
	/**
	 * @ctx_state_obj: FW object representing context register state.
	 */
	struct pvr_fw_object *ctx_state_obj;

	/** @queue: Geometry queue. */
	struct pvr_context_queue queue;

	/** @cccb: Client Circular Command Buffer. */
	struct pvr_cccb cccb;
};

/**
 * struct pvr_context_frag - Fragment render context data
 */
struct pvr_context_frag {
	/**
	 * @ctx_state_obj: FW object representing context register state.
	 */
	struct pvr_fw_object *ctx_state_obj;

	/** @queue: Fragment queue. */
	struct pvr_context_queue queue;

	/** @cccb: Client Circular Command Buffer. */
	struct pvr_cccb cccb;
};

enum pvr_context_priority {
	PVR_CTX_PRIORITY_LOW = 0,
	PVR_CTX_PRIORITY_MEDIUM,
	PVR_CTX_PRIORITY_HIGH,
};

/**
 * struct pvr_context - Context data
 */
struct pvr_context {
	/** @ref_count: Refcount for context. */
	struct kref ref_count;

	/** @pvr_dev: Pointer to owning device. */
	struct pvr_device *pvr_dev;

	/** @vm_ctx: Pointer to associated VM context. */
	struct pvr_vm_context *vm_ctx;

	/** @type: Type of context. */
	enum drm_pvr_ctx_type type;

	/** @flags: Context flags. */
	u32 flags;

	/** @priority: Context priority*/
	enum pvr_context_priority priority;

	/** @ctx_id: FW context ID. */
	u32 ctx_id;

	char process_name[PVR_COREDUMP_PROCESS_NAME_LEN];

	/** @active_node: Used to queue the context to the active context list. */
	struct list_head active_node;

	/** @job_pending: Job pending worker, used to evalulate job dependencies. */
	struct work_struct job_pending_work;

	/** @destroyed: True when the context has been destroyed. */
	atomic_t destroyed;
};

/**
 * struct pvr_context_render - Render context data
 */
struct pvr_context_render {
	/** @base: Base context structure. */
	struct pvr_context base;

	/** @ctx_geom: Geometry context data. */
	struct pvr_context_geom ctx_geom;

	/** @ctx_frag: Fragment context data. */
	struct pvr_context_frag ctx_frag;

	/** @fw_obj: FW object representing FW-side context data. */
	struct pvr_fw_object *fw_obj;
};

/**
 * struct pvr_context_compute - Compute context data
 */
struct pvr_context_compute {
	/** @base: Base context structure. */
	struct pvr_context base;

	/** @fw_obj: FW object representing FW-side context data. */
	struct pvr_fw_object *fw_obj;

	/** @queue: Compute queue. */
	struct pvr_context_queue queue;

	/**
	 * @ctx_state_obj: FW object representing context register state.
	 */
	struct pvr_fw_object *ctx_state_obj;

	/** @cccb: Client Circular Command Buffer. */
	struct pvr_cccb cccb;
};

/**
 * struct pvr_context_transfer - Transfer context data
 */
struct pvr_context_transfer {
	/** @base: Base context structure. */
	struct pvr_context base;

	/** @fw_obj: FW object representing FW-side context data. */
	struct pvr_fw_object *fw_obj;

	/** @queue: Transfer queue. */
	struct pvr_context_queue queue;

	/**
	 * @ctx_state_obj: FW object representing context register state.
	 */
	struct pvr_fw_object *ctx_state_obj;

	/** @cccb: Client Circular Command Buffer. */
	struct pvr_cccb cccb;
};

struct pvr_context *
pvr_create_render_context(struct pvr_file *pvr_file,
			  struct drm_pvr_ioctl_create_context_args *args,
			  u32 handle);
struct pvr_context *
pvr_create_compute_context(struct pvr_file *pvr_file,
			   struct drm_pvr_ioctl_create_context_args *args,
			   u32 handle);
struct pvr_context *
pvr_create_transfer_context(struct pvr_file *pvr_file,
			    struct drm_pvr_ioctl_create_context_args *args,
			    u32 handle);

static __always_inline struct pvr_context *
from_pvr_context_render(struct pvr_context_render *ctx_render)
{
	return &ctx_render->base;
};

static __always_inline struct pvr_context_render *
to_pvr_context_render(struct pvr_context *ctx)
{
	if (ctx->type != DRM_PVR_CTX_TYPE_RENDER)
		return NULL;

	return container_of(ctx, struct pvr_context_render, base);
}

static __always_inline struct pvr_context *
from_pvr_context_compute(struct pvr_context_compute *ctx_context)
{
	return &ctx_context->base;
};

static __always_inline struct pvr_context_compute *
to_pvr_context_compute(struct pvr_context *ctx)
{
	if (ctx->type != DRM_PVR_CTX_TYPE_COMPUTE)
		return NULL;

	return container_of(ctx, struct pvr_context_compute, base);
}

static __always_inline struct pvr_context *
from_pvr_context_transfer(struct pvr_context_transfer *ctx_context)
{
	return &ctx_context->base;
};

static __always_inline struct pvr_context_transfer *
to_pvr_context_transfer_frag(struct pvr_context *ctx)
{
	if (ctx->type != DRM_PVR_CTX_TYPE_TRANSFER_FRAG)
		return NULL;

	return container_of(ctx, struct pvr_context_transfer, base);
}

/**
 * pvr_context_get() - Take additional reference on context.
 * @ctx: Context pointer.
 *
 * Call pvr_context_put() to release.
 *
 * Returns:
 *  * The requested context on success, or
 *  * %NULL if no context pointer passed.
 */
static __always_inline struct pvr_context *
pvr_context_get(struct pvr_context *ctx)
{
	if (ctx)
		kref_get(&ctx->ref_count);

	return ctx;
}

/**
 * pvr_context_lookup() - Lookup context pointer from handle and file.
 * @pvr_file: Pointer to pvr_file structure.
 * @handle: Context handle.
 *
 * Takes reference on context. Call pvr_context_put() to release.
 *
 * Return:
 *  * The requested context on success, or
 *  * %NULL on failure (context does not exist, or does not belong to @pvr_file).
 */
static __always_inline struct pvr_context *
pvr_context_lookup(struct pvr_file *pvr_file, u32 handle)
{
	struct pvr_context *ctx;

	/* Take the array lock to protect against context removal.  */
	xa_lock(&pvr_file->ctx_handles);
	ctx = pvr_context_get(xa_load(&pvr_file->ctx_handles, handle));
	xa_unlock(&pvr_file->ctx_handles);

	return ctx;
}

/**
 * pvr_context_lookup_id() - Lookup context pointer from ID.
 * @pvr_dev: Device pointer.
 * @id: FW context ID.
 *
 * Takes reference on context. Call pvr_context_put() to release.
 *
 * Return:
 *  * The requested context on success, or
 *  * %NULL on failure (context does not exist).
 */
static __always_inline struct pvr_context *
pvr_context_lookup_id(struct pvr_device *pvr_dev, u32 id)
{
	struct pvr_context *ctx;

	/* Take the array lock to protect against context removal.  */
	xa_lock(&pvr_dev->ctx_ids);

	/* Contexts are removed from the ctx_ids set in the context release path,
	 * meaning the ref_count reached zero before they get removed. We need
	 * to make sure we're not trying to acquire a context that's being
	 * destroyed.
	 */
	ctx = xa_load(&pvr_dev->ctx_ids, id);
	if (!kref_get_unless_zero(&ctx->ref_count))
		ctx = NULL;

	xa_unlock(&pvr_dev->ctx_ids);

	return ctx;
}

struct pvr_context_queue_fence_ctx *
pvr_context_queue_fence_ctx_from_fence(struct dma_fence *fence);

void pvr_context_put(struct pvr_context *ctx);

int pvr_context_destroy(struct pvr_file *pvr_file, u32 handle);

void pvr_destroy_contexts_for_file(struct pvr_file *pvr_file);

bool pvr_context_has_in_flight_jobs(struct pvr_context *ctx);

struct dma_fence *pvr_context_queue_fence_create(struct pvr_context_queue *queue);

void pvr_context_pending_job_event(struct pvr_context *ctx);

void pvr_context_device_init(struct pvr_device *pvr_dev);

#endif /* __PVR_CONTEXT_H__ */
