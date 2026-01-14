/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#ifndef __THAMES_DEVICE_H__
#define __THAMES_DEVICE_H__

#include <drm/drm_device.h>
#include <drm/drm_mm.h>
#include <linux/clk.h>
#include <linux/container_of.h>
#include <linux/idr.h>
#include <linux/platform_device.h>

#include "thames_core.h"

#define MAX_CORES 8

struct thames_device {
	struct drm_device ddev;

	struct mutex sched_lock;

	struct thames_core cores[MAX_CORES];
	unsigned int num_cores;

	struct ida bo_ida;
	struct ida ctx_ida;
	struct ida job_ida;
	struct ida ipc_seq_ida;

	struct drm_mm mm;
	struct mutex mm_lock;

	u64 iova_start;
	u64 iova_size;
};

struct thames_device *thames_device_init(struct platform_device *pdev,
					 const struct drm_driver *thames_drm_driver, u64 iova_start,
					 u64 iova_size);
void thames_device_fini(struct thames_device *rdev);

#define to_thames_device(drm_dev) \
	((struct thames_device *)(container_of((drm_dev), struct thames_device, ddev)))

#endif /* __THAMES_DEVICE_H__ */
