/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_FW_META_H__
#define __PVR_FW_META_H__

#include <linux/types.h>

/* Forward declaration from pvr_device.h */
struct pvr_device;

int pvr_meta_cr_read32(struct pvr_device *pvr_dev, u32 reg_addr, u32 *reg_value_out);

#endif /* __PVR_FW_META_H__ */
