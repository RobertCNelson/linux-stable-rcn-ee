/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_POWER_H__
#define __PVR_POWER_H__

#include "pvr_device.h"

#include <linux/mutex.h>

void pvr_power_init(struct pvr_device *pvr_dev);
int pvr_power_set_state(struct pvr_device *pvr_dev, enum pvr_power_state new_state);
void pvr_power_check_idle(struct pvr_device *pvr_dev);

/**
 * pvr_power_lock() - Take device power lock
 * @pvr_dev: Target PowerVR device.
 *
 * This must be held before attempting to change power state.
 */
static __always_inline void
pvr_power_lock(struct pvr_device *pvr_dev)
{
	mutex_lock(&pvr_dev->power_lock);
}

/**
 * pvr_power_unlock() - Release device power lock
 * @pvr_dev: Target PowerVR device.
 */
static __always_inline void
pvr_power_unlock(struct pvr_device *pvr_dev)
{
	mutex_unlock(&pvr_dev->power_lock);
}

#endif /* __PVR_POWER_H__ */
