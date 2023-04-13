/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_JOB_H__
#define __PVR_JOB_H__

#include <uapi/drm/pvr_drm.h>

#include <linux/kref.h>
#include <linux/types.h>

#include <drm/drm_gem.h>

/* Forward declaration from "pvr_context.h". */
struct pvr_context;

/* Forward declarations from "pvr_device.h". */
struct pvr_device;
struct pvr_file;

/* Forward declarations from "pvr_hwrt.h". */
struct pvr_hwrt_data;

struct pvr_job {
	/** @ref_count: Refcount for job. */
	struct kref ref_count;

	/** @type: Type of job. */
	enum drm_pvr_job_type type;

	/** @id: Job ID number. */
	u32 id;

	/** @node: List node used to add a job to a context queue. */
	struct list_head node;

	/** @done_fence: Fence to signal when the job is done. */
	struct dma_fence *done_fence;

	/** @deps: Dependency tracking data. */
	struct {
		/** @cb: dma_fence callback used to get informed when a dependency is signaled. */
		struct dma_fence_cb cb;

		/** @cur: Current dependency we're waiting on. */
		struct dma_fence *cur;

		/** @next_index: Index of the next dependency to process. */
		unsigned long next_index;

		/** @non_native: Array containing remaining non-native dependencies to wait on. */
		struct xarray non_native;

		/** @native_count: Number of native dependencies. */
		unsigned long native_count;

		/** @native: Array containing remaining native dependencies to wait on. */
		struct xarray native;
	} deps;

	/** @pvr_dev: Device pointer. */
	struct pvr_device *pvr_dev;

	/** @ctx: Pointer to owning context. */
	struct pvr_context *ctx;

	/** @cmd: Command data. Format depends on @type. */
	void *cmd;

	/** @cmd_len: Length of command data, in bytes. */
	u32 cmd_len;

	/**
	 * @fw_ccb_cmd_type: Firmware CCB command type. Must be one of %ROGUE_FWIF_CCB_CMD_TYPE_*.
	 */
	u32 fw_ccb_cmd_type;

	/** @hwrt: HWRT object. Will be NULL for compute and transfer jobs. */
	struct pvr_hwrt_data *hwrt;
};

/**
 * pvr_job_get() - Take additional reference on job.
 * @job: Job pointer.
 *
 * Call pvr_job_put() to release.
 *
 * Returns:
 *  * The requested job on success, or
 *  * %NULL if no job pointer passed.
 */
static __always_inline struct pvr_job *
pvr_job_get(struct pvr_job *job)
{
	if (job)
		kref_get(&job->ref_count);

	return job;
}

void pvr_job_put(struct pvr_job *job);

void pvr_job_evict_signaled_native_deps(struct pvr_job *job);

int pvr_job_wait_first_non_signaled_native_dep(struct pvr_job *job);

bool pvr_job_non_native_deps_done(struct pvr_job *job);

int pvr_job_fits_in_cccb(struct pvr_job *job);

void pvr_job_submit(struct pvr_job *job);

int pvr_submit_jobs(struct pvr_device *pvr_dev, struct pvr_file *pvr_file,
		    struct drm_pvr_ioctl_submit_jobs_args *args);

#endif /* __PVR_JOB_H__ */
