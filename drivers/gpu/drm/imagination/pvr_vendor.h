/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_VENDOR_H__
#define __PVR_VENDOR_H__

/* Forward declaration from "pvr_device.h". */
struct pvr_device;

/**
 * struct pvr_vendor_callbacks - Vendor-specific callbacks
 */
struct pvr_vendor_callbacks {
	/**
	 * @init: Initialise vendor-specific functionality
	 * @pvr_dev: Target PowerVR device.
	 *
	 * This callback should initialise any vendor-specific data in @pvr_dev->vendor.data, and
	 * retrieve any required resources. It should not attempt to power on the GPU or interact
	 * with the GPU hardware in any way.
	 *
	 * This callback is optional.
	 *
	 * Returns:
	 * * 0 on success, or
	 * * Any negative error (see individual implementations for details).
	 */
	int (*init)(struct pvr_device *pvr_dev);

	/**
	 * @fini: Close vendor-specific functionality
	 * @pvr_dev: Target PowerVR device.
	 *
	 * This callback is optional.
	 */
	void (*fini)(struct pvr_device *pvr_dev);

	/**
	 * @power_enable: Enable GPU power
	 * @pvr_dev: Target PowerVR device.
	 *
	 * On function entry the GPU clocks in device tree will be enabled, but the GPU will
	 * otherwise be unpowered.
	 *
	 * On function exit the GPU will be fully powered and the register file will be accessible.
	 *
	 * This callback is optional.
	 *
	 * Returns:
	 *  * 0 on success, or
	 *  * -%EINVAL if the GPU was already powered, or
	 *  * -%EBUSY if the GPU fails to power on.
	 */
	int (*power_enable)(struct pvr_device *pvr_dev);

	/**
	 * @power_disable: Disable GPU power
	 * @pvr_dev: Target PowerVR device.
	 *
	 * On function entry the GPU will be fully powered.
	 *
	 * On function exit the GPU will be unpowered. This function will not affect the clocks
	 * defined in device tree; the driver will disable these later.
	 *
	 * This callback is optional.
	 *
	 * Returns:
	 *  * 0 on success, or
	 *  * -%EINVAL if the GPU was already unpowered, or
	 *  * -%EBUSY if the GPU fails to power off.
	 */
	int (*power_disable)(struct pvr_device *pvr_dev);
};

extern const struct pvr_vendor_callbacks pvr_mt8173_callbacks;

#endif /* __PVR_VENDOR_H__ */
