/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2024-2025 Tomeu Vizoso <tomeu@tomeuvizoso.net> */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#ifndef __THAMES_JOB_H__
#define __THAMES_JOB_H__

#include <drm/drm_drv.h>
#include <drm/gpu_scheduler.h>

#include "thames_core.h"
#include "thames_drv.h"

struct thames_job {
	struct drm_sched_job base;

	struct thames_device *tdev;
	struct thames_file_priv *file_priv;

	u32 job_id;
	u32 ipc_sequence;

	struct drm_gem_object *kernel;
	size_t kernel_size;

	struct drm_gem_object *params;
	size_t params_size;

	struct drm_gem_object **in_bos;
	u32 in_bo_count;

	struct drm_gem_object **out_bos;
	u32 out_bo_count;

	/* Fence to be signaled by drm-sched once its done with the job */
	struct dma_fence *inference_done_fence;

	/* Fence to be signaled by rpmsg handler when the job is complete. */
	struct dma_fence *done_fence;

	struct kref refcount;
};

int thames_ioctl_submit(struct drm_device *dev, void *data, struct drm_file *file);

int thames_job_init(struct thames_core *core);
void thames_job_fini(struct thames_core *core);
int thames_job_open(struct thames_file_priv *thames_priv);
void thames_job_close(struct thames_file_priv *thames_priv);

#endif
