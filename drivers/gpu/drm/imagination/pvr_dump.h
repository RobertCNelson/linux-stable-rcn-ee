/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_DUMP_H__
#define __PVR_DUMP_H__

/* Forward declaration from pvr_device.h. */
struct pvr_device;

/* Forward declaration from pvr_rogue_fwif.h. */
struct rogue_fwif_fwccb_cmd_context_reset_data;

void
pvr_context_reset_notification(struct pvr_device *pvr_dev,
			       struct rogue_fwif_fwccb_cmd_context_reset_data *data);

#endif /* __PVR_DUMP_H__ */
