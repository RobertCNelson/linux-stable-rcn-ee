// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_ccb.h"
#include "pvr_device.h"
#include "pvr_dump.h"
#include "pvr_free_list.h"
#include "pvr_fw.h"
#include "pvr_gem.h"
#include "pvr_power.h"

#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define ACQUIRE_SLOT_TIMEOUT (1 * HZ) /* 1s */

/**
 * pvr_ccb_init() - Initialise a CCB
 * @pvr_dev: Device pointer.
 * @pvr_ccb: Pointer to CCB structure to initialise.
 * @num_cmds_log2: Log2 of number of commands in this CCB.
 * @cmd_size: Command size for this CCB.
 *
 * Return:
 *  * Zero on success, or
 *  * Any error code returned by pvr_gem_create_and_map_fw_object().
 */
static int
pvr_ccb_init(struct pvr_device *pvr_dev, struct pvr_ccb *pvr_ccb,
	     u32 num_cmds_log2, size_t cmd_size)
{
	u32 num_cmds = 1 << num_cmds_log2;
	u32 ccb_size = num_cmds * cmd_size;
	int err;

	mutex_init(&pvr_ccb->lock);

	/*
	 * Map CCB and control structure as uncached, so we don't have to flush
	 * CPU cache repeatedly when polling for space.
	 */
	pvr_ccb->ctrl = pvr_gem_create_and_map_fw_object(pvr_dev, sizeof(*pvr_ccb->ctrl),
							 PVR_BO_FW_FLAGS_DEVICE_UNCACHED,
							 &pvr_ccb->ctrl_obj);
	if (IS_ERR(pvr_ccb->ctrl)) {
		err = PTR_ERR(pvr_ccb->ctrl);
		goto err_out;
	}

	pvr_ccb->ccb = pvr_gem_create_and_map_fw_object(pvr_dev, ccb_size,
							PVR_BO_FW_FLAGS_DEVICE_UNCACHED |
							DRM_PVR_BO_CREATE_ZEROED,
							&pvr_ccb->ccb_obj);
	if (IS_ERR(pvr_ccb->ccb)) {
		err = PTR_ERR(pvr_ccb->ccb);
		goto err_free_ctrl;
	}

	pvr_gem_get_fw_addr(pvr_ccb->ctrl_obj, &pvr_ccb->ctrl_fw_addr);
	pvr_gem_get_fw_addr(pvr_ccb->ccb_obj, &pvr_ccb->ccb_fw_addr);

	pvr_ccb->ctrl->write_offset = 0;
	pvr_ccb->ctrl->read_offset = 0;
	pvr_ccb->ctrl->wrap_mask = num_cmds - 1;
	pvr_ccb->ctrl->cmd_size = cmd_size;

	return 0;

err_free_ctrl:
	pvr_fw_object_vunmap(pvr_ccb->ctrl_obj, false);
	pvr_fw_object_release(pvr_ccb->ctrl_obj);

err_out:
	return err;
}

/**
 * pvr_ccb_fini() - Release CCB structure
 * @pvr_ccb: CCB to release.
 */
void
pvr_ccb_fini(struct pvr_ccb *pvr_ccb)
{
	pvr_fw_object_vunmap(pvr_ccb->ccb_obj, false);
	pvr_fw_object_release(pvr_ccb->ccb_obj);

	pvr_fw_object_vunmap(pvr_ccb->ctrl_obj, false);
	pvr_fw_object_release(pvr_ccb->ctrl_obj);
}

/**
 * pvr_ccb_slot_available_locked() - Test whether any slots are available in CCB
 * @pvr_ccb: CCB to test.
 * @write_offset: Address to store number of next available slot. May be %NULL.
 *
 * Caller must hold @pvr_ccb->lock.
 *
 * Return:
 *  * %true if a slot is available, or
 *  * %false if no slot is available.
 */
static __always_inline bool
pvr_ccb_slot_available_locked(struct pvr_ccb *pvr_ccb, u32 *write_offset)
{
	struct rogue_fwif_ccb_ctl *ctrl = pvr_ccb->ctrl;
	u32 next_write_offset = (ctrl->write_offset + 1) & ctrl->wrap_mask;

	lockdep_assert_held(&pvr_ccb->lock);

	if (ctrl->read_offset != next_write_offset) {
		if (write_offset)
			*write_offset = next_write_offset;
		return true;
	}

	return false;
}

/**
 * pvr_ccb_acquire_slot_locked() - Acquire slot in CCB
 * @pvr_ccb: CCB to acquire slot in.
 * @write_offset: Address to store acquired slot number.
 *
 * Caller must hold @pvr_ccb->lock.
 *
 * Return:
 *  * Zero on success, or
 *  * -EBUSY if function times out waiting for a slot.
 */
static int
pvr_ccb_acquire_slot_locked(struct pvr_ccb *pvr_ccb, u32 *write_offset)
{
	unsigned long start_timestamp = jiffies;

	lockdep_assert_held(&pvr_ccb->lock);

	while ((jiffies - start_timestamp) < ACQUIRE_SLOT_TIMEOUT) {
		if (pvr_ccb_slot_available_locked(pvr_ccb, write_offset))
			return 0;
		usleep_range(1, 50);
	}

	return -EBUSY;
}

static void
process_fwccb_command(struct pvr_device *pvr_dev, struct rogue_fwif_fwccb_cmd *cmd)
{
	switch (cmd->cmd_type) {
	case ROGUE_FWIF_FWCCB_CMD_REQUEST_GPU_RESTART:
		pvr_power_lock(pvr_dev);

		/* Stop FW. */
		WARN_ON(pvr_power_set_state(pvr_dev, PVR_POWER_STATE_OFF));

		/* Clear the FW faulted flags. */
		pvr_dev->fw_dev.fwif_sysdata->hwr_state_flags &= ~(ROGUE_FWIF_HWR_FW_FAULT |
								  ROGUE_FWIF_HWR_RESTART_REQUESTED);

		/* Start FW again. */
		WARN_ON(pvr_power_set_state(pvr_dev, PVR_POWER_STATE_ON));

		pvr_power_unlock(pvr_dev);
		break;

	case ROGUE_FWIF_FWCCB_CMD_FREELISTS_RECONSTRUCTION: {
		struct rogue_fwif_fwccb_cmd_freelists_reconstruction_data *data =
			&cmd->cmd_data.cmd_freelists_reconstruction;
		struct rogue_fwif_kccb_cmd resp_cmd;
		struct rogue_fwif_freelists_reconstruction_data *resp_data =
			&resp_cmd.cmd_data.free_lists_reconstruction_data;
		u32 i;

		for (i = 0; i < data->freelist_count; i++)
			pvr_free_list_reconstruct(pvr_dev, data->freelist_ids[i]);

		resp_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_FREELISTS_RECONSTRUCTION_UPDATE;
		resp_cmd.kccb_flags = 0;
		resp_data->freelist_count = data->freelist_count;

		memcpy(resp_data->freelist_ids, data->freelist_ids,
		       data->freelist_count * sizeof(resp_data->freelist_ids[0]));

		WARN_ON(pvr_kccb_send_cmd(pvr_dev, &resp_cmd, NULL));
		break;
	}

	case ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_NOTIFICATION:
		pvr_context_reset_notification(pvr_dev,
					       &cmd->cmd_data.cmd_context_reset_notification);
		break;

	default:
		drm_info(from_pvr_device(pvr_dev), "Received unknown FWCCB command %x\n",
			 cmd->cmd_type);
		break;
	}
}

/**
 * pvr_fwccb_process_worker() - Process any pending FWCCB commands
 * @work: Work item.
 *
 * For this initial implementation, FWCCB commands will be printed to the console but otherwise not
 * processed.
 */
static void
pvr_fwccb_process_worker(struct work_struct *work)
{
	struct pvr_device *pvr_dev = container_of(work, struct pvr_device, fwccb_work);
	struct rogue_fwif_fwccb_cmd *fwccb = pvr_dev->fwccb.ccb;
	struct rogue_fwif_ccb_ctl *ctrl = pvr_dev->fwccb.ctrl;

	mutex_lock(&pvr_dev->fwccb.lock);

	while (ctrl->read_offset != ctrl->write_offset) {
		struct rogue_fwif_fwccb_cmd cmd = fwccb[ctrl->read_offset];

		ctrl->read_offset = (ctrl->read_offset + 1) & ctrl->wrap_mask;

		/* Drop FWCCB lock while we process command. */
		mutex_unlock(&pvr_dev->fwccb.lock);

		process_fwccb_command(pvr_dev, &cmd);

		mutex_lock(&pvr_dev->fwccb.lock);
	}

	mutex_unlock(&pvr_dev->fwccb.lock);
}

/**
 * pvr_kccb_send_cmd_power_locked() - Send command to the KCCB, with the power lock held
 * @pvr_dev: Device pointer.
 * @cmd: Command to sent.
 * @kccb_slot: Address to store the KCCB slot for this command. May be %NULL.
 *
 * Returns:
 *  * Zero on success, or
 *  * -EBUSY if timeout while waiting for a free KCCB slot.
 */
int
pvr_kccb_send_cmd_power_locked(struct pvr_device *pvr_dev, struct rogue_fwif_kccb_cmd *cmd,
			       u32 *kccb_slot)
{
	struct pvr_ccb *pvr_ccb = &pvr_dev->kccb;
	struct rogue_fwif_kccb_cmd *kccb = pvr_ccb->ccb;
	struct rogue_fwif_ccb_ctl *ctrl = pvr_ccb->ctrl;
	u32 old_write_offset;
	u32 new_write_offset;
	int err;

	lockdep_assert_held(&pvr_dev->power_lock);
	WARN_ON(pvr_dev->power_state != PVR_POWER_STATE_ON);

	mutex_lock(&pvr_ccb->lock);

	old_write_offset = ctrl->write_offset;

	err = pvr_ccb_acquire_slot_locked(pvr_ccb, &new_write_offset);
	if (err)
		goto err_unlock;

	memcpy(&kccb[old_write_offset], cmd,
	       sizeof(struct rogue_fwif_kccb_cmd));
	if (kccb_slot) {
		*kccb_slot = old_write_offset;
		/* Clear return status for this slot. */
		WRITE_ONCE(pvr_dev->kccb_rtn[old_write_offset],
			   ROGUE_FWIF_KCCB_RTN_SLOT_NO_RESPONSE);
	}
	mb(); /* memory barrier */
	ctrl->write_offset = new_write_offset;

	mutex_unlock(&pvr_ccb->lock);

	/* Kick MTS */
	pvr_fw_mts_schedule(pvr_dev,
			    PVR_FWIF_DM_GP & ~ROGUE_CR_MTS_SCHEDULE_DM_CLRMSK);

	return 0;

err_unlock:
	mutex_unlock(&pvr_ccb->lock);

	return err;
}

/**
 * pvr_kccb_send_cmd() - Send command to the KCCB
 * @pvr_dev: Device pointer.
 * @cmd: Command to sent.
 * @kccb_slot: Address to store the KCCB slot for this command. May be %NULL.
 *
 * Returns:
 *  * Zero on success, or
 *  * -EBUSY if timeout while waiting for a free KCCB slot.
 */
int
pvr_kccb_send_cmd(struct pvr_device *pvr_dev, struct rogue_fwif_kccb_cmd *cmd,
		  u32 *kccb_slot)
{
	int err;

	pvr_power_lock(pvr_dev);

	err = pvr_power_set_state(pvr_dev, PVR_POWER_STATE_ON);
	if (err)
		goto err_power_unlock;

	err = pvr_kccb_send_cmd_power_locked(pvr_dev, cmd, kccb_slot);

err_power_unlock:
	pvr_power_unlock(pvr_dev);

	return err;
}

/**
 * pvr_kccb_wait_for_completion() - Wait for a KCCB command to complete
 * @pvr_dev: Device pointer.
 * @slot_nr: KCCB slot to wait on.
 * @timeout: Timeout length (in jiffies).
 * @rtn_out: Location to store KCCB command result. May be %NULL.
 *
 * Returns:
 *  * Zero on success, or
 *  * -ETIMEDOUT on timeout.
 */
int
pvr_kccb_wait_for_completion(struct pvr_device *pvr_dev, u32 slot_nr,
			     u32 timeout, u32 *rtn_out)
{
	int ret = wait_event_timeout(pvr_dev->kccb_rtn_q, READ_ONCE(pvr_dev->kccb_rtn[slot_nr]) &
				     ROGUE_FWIF_KCCB_RTN_SLOT_CMD_EXECUTED, timeout);

	if (ret && rtn_out)
		*rtn_out = READ_ONCE(pvr_dev->kccb_rtn[slot_nr]);

	return ret ? 0 : -ETIMEDOUT;
}

/**
 * pvr_kccb_init() - Initialise device KCCB
 * @pvr_dev: Target PowerVR device
 *
 * Returns:
 *  * 0 on success, or
 *  * Any error returned by pvr_ccb_init().
 */
int
pvr_kccb_init(struct pvr_device *pvr_dev)
{
	return pvr_ccb_init(pvr_dev, &pvr_dev->kccb,
			    ROGUE_FWIF_KCCB_NUMCMDS_LOG2_DEFAULT,
			    sizeof(struct rogue_fwif_kccb_cmd));
}

/**
 * pvr_fwccb_init() - Initialise device FWCCB
 * @pvr_dev: Target PowerVR device
 *
 * Returns:
 *  * 0 on success, or
 *  * Any error returned by pvr_ccb_init().
 */
int
pvr_fwccb_init(struct pvr_device *pvr_dev)
{
	INIT_WORK(&pvr_dev->fwccb_work, pvr_fwccb_process_worker);

	return pvr_ccb_init(pvr_dev, &pvr_dev->fwccb,
			    ROGUE_FWIF_FWCCB_NUMCMDS_LOG2,
			    sizeof(struct rogue_fwif_fwccb_cmd));
}
