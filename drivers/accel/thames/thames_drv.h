/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#ifndef __THAMES_DRV_H__
#define __THAMES_DRV_H__

#include <drm/drm_mm.h>
#include <drm/gpu_scheduler.h>

#include "thames_device.h"

struct thames_file_priv {
	struct thames_device *tdev;

	struct drm_sched_entity sched_entity;

	u32 context_id;
	bool context_valid;
};

#endif
