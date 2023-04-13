// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_ccb.h"
#include "pvr_cccb.h"
#include "pvr_device.h"
#include "pvr_gem.h"
#include "pvr_hwrt.h"

#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/types.h>

#define PADDING_COMMAND_SIZE sizeof(struct rogue_fwif_ccb_cmd_header)

static __always_inline u32
get_ccb_space(u32 w_off, u32 r_off, u32 ccb_size)
{
	return (((r_off) - (w_off)) + ((ccb_size) - 1)) & ((ccb_size) - 1);
}

/**
 * pvr_cccb_init() - Initialise a Client CCB
 * @pvr_dev: Device pointer.
 * @pvr_cccb: Pointer to Client CCB structure to initialise.
 * @size_log2: Log2 size of Client CCB in bytes.
 * @name: Name of owner of Client CCB. Used for fence context.
 *
 * Return:
 *  * Zero on success, or
 *  * Any error code returned by pvr_gem_create_and_map_fw_object().
 */
int
pvr_cccb_init(struct pvr_device *pvr_dev, struct pvr_cccb *pvr_cccb,
	      u32 size_log2, const char *name)
{
	size_t size = 1 << size_log2;
	int err;

	mutex_init(&pvr_cccb->lock);

	/*
	 * Map CCCB and control structure as uncached, so we don't have to flush
	 * CPU cache repeatedly when polling for space.
	 */
	pvr_cccb->ctrl = pvr_gem_create_and_map_fw_object(pvr_dev, sizeof(*pvr_cccb->ctrl),
							  PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							  &pvr_cccb->ctrl_obj);
	if (IS_ERR(pvr_cccb->ctrl)) {
		err = PTR_ERR(pvr_cccb->ctrl);
		goto err_out;
	}

	pvr_cccb->cccb = pvr_gem_create_and_map_fw_object(pvr_dev, size,
							  PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							  DRM_PVR_BO_CREATE_ZEROED,
							  &pvr_cccb->cccb_obj);
	if (IS_ERR(pvr_cccb->cccb)) {
		err = PTR_ERR(pvr_cccb->cccb);
		goto err_free_ctrl;
	}

	pvr_gem_get_fw_addr(pvr_cccb->ctrl_obj, &pvr_cccb->ctrl_fw_addr);
	pvr_gem_get_fw_addr(pvr_cccb->cccb_obj, &pvr_cccb->cccb_fw_addr);

	WRITE_ONCE(pvr_cccb->ctrl->write_offset, 0);
	WRITE_ONCE(pvr_cccb->ctrl->read_offset, 0);
	WRITE_ONCE(pvr_cccb->ctrl->dep_offset, 0);
	WRITE_ONCE(pvr_cccb->ctrl->wrap_mask, size - 1);
	pvr_cccb->size = size;
	pvr_cccb->write_offset = 0;
	pvr_cccb->wrap_mask = size - 1;

	return 0;

err_free_ctrl:
	pvr_fw_object_vunmap(pvr_cccb->ctrl_obj, false);
	pvr_fw_object_release(pvr_cccb->ctrl_obj);

err_out:
	return err;
}

/**
 * pvr_cccb_fini() - Release Client CCB structure
 * @pvr_cccb: Client CCB to release.
 */
void
pvr_cccb_fini(struct pvr_cccb *pvr_cccb)
{
	pvr_fw_object_vunmap(pvr_cccb->cccb_obj, false);
	pvr_fw_object_release(pvr_cccb->cccb_obj);

	pvr_fw_object_vunmap(pvr_cccb->ctrl_obj, false);
	pvr_fw_object_release(pvr_cccb->ctrl_obj);
}

static void
build_padding_command(void *cmd_ptr, u32 remaining)
{
	struct rogue_fwif_ccb_cmd_header *cmd = cmd_ptr;

	WRITE_ONCE(cmd->cmd_type, ROGUE_FWIF_CCB_CMD_TYPE_PADDING);
	WRITE_ONCE(cmd->cmd_size, remaining - sizeof(*cmd));
}

/**
 * pvr_cccb_check_command_space_locked() - Check if a command sequence fits in the CCCB
 * @pvr_cccb: Target Client CCB.
 * @size: Size of the command sequence.
 *
 * Caller must hold @pvr_cccb->lock.
 *
 * Returns:
 *  * Zero on success, or
 *  * -ENOMEM if insufficient space is currently available in the CCCB, or
 *  * -E2BIG if the command will never fit in the CCCB.
 */
static int pvr_cccb_check_command_space_locked(struct pvr_cccb *pvr_cccb, size_t size)
{
	struct rogue_fwif_cccb_ctl *ctrl = pvr_cccb->ctrl;
	u32 read_offset = READ_ONCE(ctrl->read_offset);
	u32 remaining = pvr_cccb->size - pvr_cccb->write_offset;
	u32 required_size = size;

	lockdep_assert_held(&pvr_cccb->lock);

	/*
	 * Always ensure we have enough room for a padding command at the end of
	 * the CCCB.
	 */
	required_size += PADDING_COMMAND_SIZE;

	if (required_size > pvr_cccb->size)
		return -E2BIG;

	if (remaining < required_size) {
		/*
		 * Command would need to wrap, so we need to pad the remainder
		 * of the CCCB.
		 */
		required_size += remaining;
	}

	if (get_ccb_space(pvr_cccb->write_offset, read_offset, pvr_cccb->size) < required_size)
		return -ENOMEM;

	return 0;
}

/**
 * pvr_cccb_check_command_space_locked() - Check if a command sequence fits in the CCCB
 * @pvr_cccb: Target Client CCB.
 * @size: Size of the command sequence.
 *
 * Takes the @pvr_cccb->lock and call pvr_cccb_check_command_space_locked().
 *
 * Returns:
 *  * Zero on success, or
 *  * -ENOMEM if insufficient space is currently available in the CCCB.
 */
int pvr_cccb_check_command_space(struct pvr_cccb *pvr_cccb, size_t size)
{
	int ret;

	mutex_lock(&pvr_cccb->lock);
	ret = pvr_cccb_check_command_space_locked(pvr_cccb, size);
	mutex_unlock(&pvr_cccb->lock);

	return ret;
}

/**
 * pvr_cccb_acquire_command_space_locked() - Acquire space in a Client CCB
 * @pvr_cccb: Target Client CCB.
 * @size: Size of allocation, in bytes.
 * @out_ptr: Pointer to location to store CPU pointer to acquired space.
 * @new_write_offset: Pointer to location to store new CCB write offset.
 *
 * Caller must hold @pvr_cccb->lock, and if this function succeeds then it must
 * be held until after pvr_release_cccb_space() is called.
 *
 * Returns:
 *  * Zero on success, or
 *  * -EAGAIN if insufficient space is currently available in the CCCB.
 */
static int
pvr_cccb_acquire_command_space_locked(struct pvr_cccb *pvr_cccb, size_t size,
				      void **out_ptr, u32 *new_write_offset)
{
	struct rogue_fwif_cccb_ctl *ctrl = pvr_cccb->ctrl;
	u32 read_offset = READ_ONCE(ctrl->read_offset);
	u32 remaining = pvr_cccb->size - pvr_cccb->write_offset;
	u32 required_size = size;
	bool padding_required = false;

	lockdep_assert_held(&pvr_cccb->lock);

	/*
	 * Always ensure we have enough room for a padding command at the end of
	 * the CCCB.
	 */
	required_size += PADDING_COMMAND_SIZE;

	if (remaining < required_size) {
		/*
		 * Command would need to wrap, so we need to pad the remainder
		 * of the CCCB.
		 */
		required_size += remaining;
		padding_required = true;
	}

	if (get_ccb_space(pvr_cccb->write_offset, read_offset, pvr_cccb->size) <
	    required_size)
		return -EAGAIN;

	if (padding_required) {
		/* Add padding command */
		build_padding_command(&pvr_cccb->cccb[pvr_cccb->write_offset], remaining);
		pvr_cccb->write_offset = 0;
	}

	*out_ptr = &pvr_cccb->cccb[pvr_cccb->write_offset];
	*new_write_offset = pvr_cccb->write_offset + size;

	return 0;
}

/**
 * pvr_cccb_write_command() - Write a command to a Client CCB
 * @pvr_cccb: Target Client CCB.
 * @cmd_data: Pointer to command to write.
 * @size: Size of command in bytes.
 *
 * Caller must have locked the Client CCB with pvr_cccb_lock().
 *
 * Returns:
 *  * Zero on success, or
 *  * -EAGAIN if insufficient space is currently available in the CCCB.
 */
int
pvr_cccb_write_command(struct pvr_cccb *pvr_cccb, void *cmd_data, size_t size)
{
	void *cccb_ptr;
	u32 new_write_offset;
	int err;

	lockdep_assert_held(&pvr_cccb->lock);

	err = pvr_cccb_acquire_command_space_locked(pvr_cccb, size, &cccb_ptr,
						    &new_write_offset);
	if (err)
		return err;

	memcpy(cccb_ptr, cmd_data, size);
	pvr_cccb->write_offset = new_write_offset;

	return 0;
}

/**
 * pvr_cccb_write_command_with_header() - Write a command + command header to a
 *                                        Client CCB
 * @pvr_cccb: Target Client CCB.
 * @cmd_type: Client CCB command type. Must be one of %ROGUE_FWIF_CCB_CMD_TYPE_*.
 * @cmd_size: Size of command in bytes.
 * @cmd_data: Pointer to command to write.
 * @ext_job_ref: External job reference.
 * @int_job_ref: Internal job reference.
 *
 * Caller must have locked the Client CCB with pvr_cccb_lock().
 *
 * Returns:
 *  * Zero on success, or
 *  * -EAGAIN if insufficient space is currently available in the CCCB.
 */
int
pvr_cccb_write_command_with_header(struct pvr_cccb *pvr_cccb, u32 cmd_type, u32 cmd_size,
				   void *cmd_data, u32 ext_job_ref, u32 int_job_ref)
{
	struct rogue_fwif_ccb_cmd_header cmd_header;
	u8 *cccb_ptr;
	u32 new_write_offset;
	const size_t size = sizeof(cmd_header) + cmd_size;
	int err;

	lockdep_assert_held(&pvr_cccb->lock);

	cmd_header.cmd_type = cmd_type;
	cmd_header.cmd_size = cmd_size;
	cmd_header.ext_job_ref = ext_job_ref;
	cmd_header.int_job_ref = int_job_ref;

	err = pvr_cccb_acquire_command_space_locked(pvr_cccb, size, (void **)&cccb_ptr,
						    &new_write_offset);
	if (err)
		return err;

	memcpy(cccb_ptr, &cmd_header, sizeof(cmd_header));
	memcpy(cccb_ptr + sizeof(cmd_header), cmd_data, cmd_size);
	pvr_cccb->write_offset = new_write_offset;

	return 0;
}

/**
 * pvr_cccb_wait_for_idle: Wait for Client CCB to go idle
 * @pvr_cccb: Client CCB to wait on.
 * @timeout: Timeout length (in jiffies).
 *
 * Returns:
 *  * Zero on success, or
 *  * -EBUSY on timeout.
 */
int
pvr_cccb_wait_for_idle(struct pvr_cccb *pvr_cccb, u32 timeout)
{
	struct rogue_fwif_cccb_ctl *ctrl = pvr_cccb->ctrl;
	unsigned long end_jiffies = jiffies + timeout;

	while (!time_after(jiffies, end_jiffies)) {
		if (READ_ONCE(ctrl->read_offset) == READ_ONCE(ctrl->write_offset))
			return 0;
		usleep_range(100, 1000);
	}

	return -EBUSY;
}

/**
 * pvr_cccb_unlock_send_kccb_kick: Unlock Client CCB and send KCCB kick to
 *                                 trigger command processing
 * @pvr_dev: Device pointer.
 * @pvr_cccb: Pointer to CCCB to process.
 * @cctx_fw_addr: FW virtual address for context owning this Client CCB.
 * @hwrt: HWRT data set associated with this kick. May be %NULL.
 *
 * Caller must have locked the Client CCB with pvr_cccb_lock().
 *
 * If this function is successful, then the Client CCB will be unlocked. On
 * error, the Client CCB will still be locked, and it is the caller's
 * responsibility to unlock it with pvr_cccb_unlock_rollback().
 *
 * Return :
 *  * Zero on success, or
 *  * Any error returned by pvr_kccb_send_cmd().
 */
int
pvr_cccb_unlock_send_kccb_kick(struct pvr_device *pvr_dev,
			       struct pvr_cccb *pvr_cccb, u32 cctx_fw_addr,
			       struct pvr_hwrt_data *hwrt)
{
	struct rogue_fwif_kccb_cmd cmd_kick;
	struct rogue_fwif_kccb_cmd_kick_data *cmd_kick_data =
		&cmd_kick.cmd_data.cmd_kick_data;
	u32 *cleanup_ctl;
	int err;

	lockdep_assert_held(&pvr_cccb->lock);

	cmd_kick.cmd_type = ROGUE_FWIF_KCCB_CMD_KICK;
	cmd_kick_data->context_fw_addr = cctx_fw_addr;
	cmd_kick_data->client_woff_update = pvr_cccb->write_offset;
	cmd_kick_data->client_wrap_mask_update = pvr_cccb->wrap_mask;

	cmd_kick_data->num_cleanup_ctl = 0;
	cleanup_ctl = cmd_kick_data->cleanup_ctl_fw_addr;
	if (hwrt) {
		pvr_gem_get_fw_addr_offset(hwrt->fw_obj,
					   offsetof(struct rogue_fwif_hwrtdata, cleanup_state),
					   cleanup_ctl);
		cmd_kick_data->num_cleanup_ctl++;
		cleanup_ctl++;
	}
	cmd_kick_data->work_est_cmd_header_offset = 0;

	err = pvr_kccb_send_cmd(pvr_dev, &cmd_kick, NULL);
	if (err)
		goto err_out;

	mutex_unlock(&pvr_cccb->lock);

	return 0;

err_out:
	return err;
}
