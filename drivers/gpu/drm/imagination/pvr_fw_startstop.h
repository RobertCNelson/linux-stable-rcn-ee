/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_FW_STARTSTOP_H__
#define __PVR_FW_STARTSTOP_H__

/* Forward declaration from pvr_device.h. */
struct pvr_device;

int pvr_fw_start(struct pvr_device *pvr_dev);
int pvr_fw_stop(struct pvr_device *pvr_dev);

#endif /* __PVR_FW_STARTSTOP_H__ */
