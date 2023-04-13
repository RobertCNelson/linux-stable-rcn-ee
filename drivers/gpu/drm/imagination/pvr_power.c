// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_fw.h"
#include "pvr_fw_startstop.h"
#include "pvr_power.h"
#include "pvr_rogue_fwif.h"

#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define POWER_SYNC_TIMEOUT_US (1000000) /* 1s */

#define POWER_IDLE_DELAY_JIFFIES (1)

static int
pvr_power_send_command(struct pvr_device *pvr_dev, struct rogue_fwif_kccb_cmd *pow_cmd)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	u32 slot_nr;
	u32 value;
	int err;

	WRITE_ONCE(*fw_dev->power_sync, 0);

	err = pvr_kccb_send_cmd_power_locked(pvr_dev, pow_cmd, &slot_nr);
	if (err)
		goto err_out;

	/* Wait for FW to acknowledge. */
	err = readl_poll_timeout(pvr_dev->fw_dev.power_sync, value, value != 0, 100,
				 POWER_SYNC_TIMEOUT_US);
	if (err)
		goto err_out;

	return 0;

err_out:
	return err;
}

static int
pvr_power_request_idle(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd pow_cmd;

	/* Send FORCED_IDLE request to FW. */
	pow_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_POW;
	pow_cmd.cmd_data.pow_data.pow_type = ROGUE_FWIF_POW_FORCED_IDLE_REQ;
	pow_cmd.cmd_data.pow_data.power_req_data.pow_request_type = ROGUE_FWIF_POWER_FORCE_IDLE;

	return pvr_power_send_command(pvr_dev, &pow_cmd);
}

static int
pvr_power_request_pwr_off(struct pvr_device *pvr_dev)
{
	struct rogue_fwif_kccb_cmd pow_cmd;

	/* Send POW_OFF request to firmware. */
	pow_cmd.cmd_type = ROGUE_FWIF_KCCB_CMD_POW;
	pow_cmd.cmd_data.pow_data.pow_type = ROGUE_FWIF_POW_OFF_REQ;
	pow_cmd.cmd_data.pow_data.power_req_data.forced = true;

	return pvr_power_send_command(pvr_dev, &pow_cmd);
}

/**
 * pvr_power_set_state() - Change GPU power state
 * @pvr_dev: Target PowerVR device.
 * @new_state: Desired power state.
 *
 * If the GPU is already in the desired power state then this function is a no-op.
 *
 * Caller must hold the device's power lock.
 *
 * Returns:
 *  * 0 on success, or
 *  * Any appropriate error on failure.
 */
int
pvr_power_set_state(struct pvr_device *pvr_dev, enum pvr_power_state new_state)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct device *dev = drm_dev->dev;
	int err;

	lockdep_assert_held(&pvr_dev->power_lock);

	cancel_delayed_work(&pvr_dev->delayed_idle_work);

	if (pvr_dev->power_state == new_state) {
		err = 0;
		goto err_out;
	}

	switch (new_state) {
	case PVR_POWER_STATE_OFF:
		/* Force idle */
		err = pvr_power_request_idle(pvr_dev);
		if (err)
			goto err_out;

		err = pvr_power_request_pwr_off(pvr_dev);
		if (err)
			goto err_out;

		err = pvr_fw_stop(pvr_dev);
		if (err)
			goto err_out;

		pm_runtime_put_sync_suspend(dev);
		break;

	case PVR_POWER_STATE_ON:
		err = pm_runtime_resume_and_get(dev);
		if (err)
			goto err_out;

		/* Restart FW */
		err = pvr_fw_start(pvr_dev);
		if (err)
			goto err_out;

		err = pvr_wait_for_fw_boot(pvr_dev);
		if (err) {
			drm_err(drm_dev, "Firmware failed to boot\n");
			goto err_out;
		}
		break;

	default:
		WARN_ON(true);
		err = -EINVAL;
		goto err_out;
	}

	/* Set power state */
	pvr_dev->power_state = new_state;

	return 0;

err_out:
	return err;
}

static void
pvr_delayed_idle_worker(struct work_struct *work)
{
	struct pvr_device *pvr_dev = container_of(work, struct pvr_device,
						  delayed_idle_work.work);

	mutex_lock(&pvr_dev->power_lock);

	if (pvr_dev->fw_dev.fwif_sysdata->pow_state == ROGUE_FWIF_POW_IDLE)
		pvr_power_set_state(pvr_dev, PVR_POWER_STATE_OFF);

	mutex_unlock(&pvr_dev->power_lock);
}

/**
 * pvr_power_check_idle() - Check for GPU idle, and schedule power off if required
 * @pvr_dev: Target PowerVR device
 *
 * The actual power off is performed by a delayed work item. This implements hysteresis.
 */
void
pvr_power_check_idle(struct pvr_device *pvr_dev)
{
	enum rogue_fwif_pow_state pow_state = READ_ONCE(pvr_dev->fw_dev.fwif_sysdata->pow_state);

	if (pow_state == ROGUE_FWIF_POW_IDLE &&
	    !delayed_work_pending(&pvr_dev->delayed_idle_work)) {
		queue_delayed_work(pvr_dev->irq_wq, &pvr_dev->delayed_idle_work,
				   POWER_IDLE_DELAY_JIFFIES);
	} else if (pow_state != ROGUE_FWIF_POW_IDLE &&
		   delayed_work_pending(&pvr_dev->delayed_idle_work)) {
		cancel_delayed_work(&pvr_dev->delayed_idle_work);
	}
}

/**
 * pvr_power_init() - Initialise power management for device
 * @pvr_dev: Target PowerVR device.
 */
void
pvr_power_init(struct pvr_device *pvr_dev)
{
	mutex_init(&pvr_dev->power_lock);
	pvr_dev->power_state = PVR_POWER_STATE_OFF;
	INIT_DELAYED_WORK(&pvr_dev->delayed_idle_work, pvr_delayed_idle_worker);
}
