/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_CCCB_H__
#define __PVR_CCCB_H__

#include "pvr_rogue_fwif.h"
#include "pvr_rogue_fwif_shared.h"

#include <linux/mutex.h>
#include <linux/types.h>

/* Forward declaration from pvr_device.h. */
struct pvr_device;

/* Forward declaration from pvr_gem.h. */
struct pvr_fw_object;

/* Forward declaration from pvr_hwrt.h. */
struct pvr_hwrt_data;

struct pvr_cccb {
	/** @ctrl_obj: FW object representing CCCB control structure. */
	struct pvr_fw_object *ctrl_obj;

	/** @ccb_obj: FW object representing CCCB. */
	struct pvr_fw_object *cccb_obj;

	/** @lock: Mutex protecting @ctrl and @cccb. */
	struct mutex lock;

	/**
	 * @ctrl: Kernel mapping of CCCB control structure. @lock must be held
	 *        when accessing.
	 */
	struct rogue_fwif_cccb_ctl *ctrl;

	/** @cccb: Kernel mapping of CCCB. @lock must be held when accessing.*/
	u8 *cccb;

	/** @ctrl_fw_addr: FW virtual address of CCCB control structure. */
	u32 ctrl_fw_addr;
	/** @ccb_fw_addr: FW virtual address of CCCB. */
	u32 cccb_fw_addr;

	/** @size: Size of CCCB in bytes. */
	size_t size;

	/** @write_offset: CCCB write offset. */
	u32 write_offset;

	/** @wrap_mask: CCCB wrap mask. */
	u32 wrap_mask;

	/** @old_write_offset: CCCB write offset, sampled at CCCB lock time. */
	u32 old_write_offset;
};

int pvr_cccb_init(struct pvr_device *pvr_dev, struct pvr_cccb *cccb,
		  u32 size_log2, const char *name);
void pvr_cccb_fini(struct pvr_cccb *cccb);

int pvr_cccb_write_command(struct pvr_cccb *pvr_cccb, void *cmd_data,
			   size_t size);
int
pvr_cccb_write_command_with_header(struct pvr_cccb *pvr_cccb, u32 cmd_type, u32 cmd_size,
				   void *cmd_data, u32 ext_job_ref, u32 int_job_ref);
int pvr_cccb_wait_for_idle(struct pvr_cccb *pvr_cccb, u32 timeout);
int pvr_cccb_unlock_send_kccb_kick(struct pvr_device *pvr_dev,
				   struct pvr_cccb *pvr_cccb, u32 cctx_fw_addr,
				   struct pvr_hwrt_data *hwrt);
int pvr_cccb_check_command_space(struct pvr_cccb *pvr_cccb, size_t size);

/**
 * pvr_cccb_lock() - Lock a client CCB for writing
 * @pvr_cccb: Target client CCB.
 */
static __always_inline void
pvr_cccb_lock(struct pvr_cccb *pvr_cccb)
{
	mutex_lock(&pvr_cccb->lock);

	pvr_cccb->old_write_offset = pvr_cccb->write_offset;
}

/**
 * pvr_cccb_unlock_rollback() - Unlock a client CCB and rollback any written
 *                              commands
 * @pvr_cccb: Target client CCB.
 */
static __always_inline void
pvr_cccb_unlock_rollback(struct pvr_cccb *pvr_cccb)
{
	lockdep_assert_held(&pvr_cccb->lock);

	pvr_cccb->write_offset = pvr_cccb->old_write_offset;
	mutex_unlock(&pvr_cccb->lock);
}

/**
 * pvr_cccb_get_size_of_cmd_with_hdr() - Get the size of a command and its header.
 * @cmd_size: Command size.
 *
 * Returns the size of the command and its header.
 */
static __always_inline u32
pvr_cccb_get_size_of_cmd_with_hdr(u32 cmd_size)
{
	return sizeof(struct rogue_fwif_ccb_cmd_header) + cmd_size;
}

#endif /* __PVR_CCCB_H__ */
