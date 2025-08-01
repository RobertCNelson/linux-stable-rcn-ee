// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2012-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/phy/phy.h>
#include <drm/drm_print.h>

#include "dp_reg.h"
#include "dp_aux.h"

enum msm_dp_aux_err {
	DP_AUX_ERR_NONE,
	DP_AUX_ERR_ADDR,
	DP_AUX_ERR_TOUT,
	DP_AUX_ERR_NACK,
	DP_AUX_ERR_DEFER,
	DP_AUX_ERR_NACK_DEFER,
	DP_AUX_ERR_PHY,
};

struct msm_dp_aux_private {
	struct device *dev;
	void __iomem *aux_base;

	struct phy *phy;

	struct mutex mutex;
	struct completion comp;

	enum msm_dp_aux_err aux_error_num;
	u32 retry_cnt;
	bool cmd_busy;
	bool native;
	bool read;
	bool no_send_addr;
	bool no_send_stop;
	bool initted;
	bool is_edp;
	bool enable_xfers;
	u32 offset;
	u32 segment;

	struct drm_dp_aux msm_dp_aux;
};

static inline u32 msm_dp_read_aux(struct msm_dp_aux_private *aux, u32 offset)
{
	return readl_relaxed(aux->aux_base + offset);
}

static inline void msm_dp_write_aux(struct msm_dp_aux_private *aux,
				u32 offset, u32 data)
{
	/*
	 * To make sure aux reg writes happens before any other operation,
	 * this function uses writel() instread of writel_relaxed()
	 */
	writel(data, aux->aux_base + offset);
}

static void msm_dp_aux_clear_hw_interrupts(struct msm_dp_aux_private *aux)
{
	msm_dp_read_aux(aux, REG_DP_PHY_AUX_INTERRUPT_STATUS);
	msm_dp_write_aux(aux, REG_DP_PHY_AUX_INTERRUPT_CLEAR, 0x1f);
	msm_dp_write_aux(aux, REG_DP_PHY_AUX_INTERRUPT_CLEAR, 0x9f);
	msm_dp_write_aux(aux, REG_DP_PHY_AUX_INTERRUPT_CLEAR, 0);
}

/*
 * NOTE: resetting AUX controller will also clear any pending HPD related interrupts
 */
static void msm_dp_aux_reset(struct msm_dp_aux_private *aux)
{
	u32 aux_ctrl;

	aux_ctrl = msm_dp_read_aux(aux, REG_DP_AUX_CTRL);

	aux_ctrl |= DP_AUX_CTRL_RESET;
	msm_dp_write_aux(aux, REG_DP_AUX_CTRL, aux_ctrl);
	usleep_range(1000, 1100); /* h/w recommended delay */

	aux_ctrl &= ~DP_AUX_CTRL_RESET;
	msm_dp_write_aux(aux, REG_DP_AUX_CTRL, aux_ctrl);
}

static void msm_dp_aux_enable(struct msm_dp_aux_private *aux)
{
	u32 aux_ctrl;

	aux_ctrl = msm_dp_read_aux(aux, REG_DP_AUX_CTRL);

	msm_dp_write_aux(aux, REG_DP_TIMEOUT_COUNT, 0xffff);
	msm_dp_write_aux(aux, REG_DP_AUX_LIMITS, 0xffff);

	aux_ctrl |= DP_AUX_CTRL_ENABLE;
	msm_dp_write_aux(aux, REG_DP_AUX_CTRL, aux_ctrl);
}

static void msm_dp_aux_disable(struct msm_dp_aux_private *aux)
{
	u32 aux_ctrl;

	aux_ctrl = msm_dp_read_aux(aux, REG_DP_AUX_CTRL);
	aux_ctrl &= ~DP_AUX_CTRL_ENABLE;
	msm_dp_write_aux(aux, REG_DP_AUX_CTRL, aux_ctrl);
}

static int msm_dp_aux_wait_for_hpd_connect_state(struct msm_dp_aux_private *aux,
					     unsigned long wait_us)
{
	u32 state;

	/* poll for hpd connected status every 2ms and timeout after wait_us */
	return readl_poll_timeout(aux->aux_base +
				  REG_DP_DP_HPD_INT_STATUS,
				  state, state & DP_DP_HPD_STATE_STATUS_CONNECTED,
				  min(wait_us, 2000), wait_us);
}

#define MAX_AUX_RETRIES			5

static ssize_t msm_dp_aux_write(struct msm_dp_aux_private *aux,
			struct drm_dp_aux_msg *msg)
{
	u8 data[4];
	u32 reg;
	ssize_t len;
	u8 *msgdata = msg->buffer;
	int const AUX_CMD_FIFO_LEN = 128;
	int i = 0;

	if (aux->read)
		len = 0;
	else
		len = msg->size;

	/*
	 * cmd fifo only has depth of 144 bytes
	 * limit buf length to 128 bytes here
	 */
	if (len > AUX_CMD_FIFO_LEN - 4) {
		DRM_ERROR("buf size greater than allowed size of 128 bytes\n");
		return -EINVAL;
	}

	/* Pack cmd and write to HW */
	data[0] = (msg->address >> 16) & 0xf;	/* addr[19:16] */
	if (aux->read)
		data[0] |=  BIT(4);		/* R/W */

	data[1] = msg->address >> 8;		/* addr[15:8] */
	data[2] = msg->address;			/* addr[7:0] */
	data[3] = msg->size - 1;		/* len[7:0] */

	for (i = 0; i < len + 4; i++) {
		reg = (i < 4) ? data[i] : msgdata[i - 4];
		reg <<= DP_AUX_DATA_OFFSET;
		reg &= DP_AUX_DATA_MASK;
		reg |= DP_AUX_DATA_WRITE;
		/* index = 0, write */
		if (i == 0)
			reg |= DP_AUX_DATA_INDEX_WRITE;
		msm_dp_write_aux(aux, REG_DP_AUX_DATA, reg);
	}

	msm_dp_write_aux(aux, REG_DP_AUX_TRANS_CTRL, 0);
	msm_dp_aux_clear_hw_interrupts(aux);

	reg = 0; /* Transaction number == 1 */
	if (!aux->native) { /* i2c */
		reg |= DP_AUX_TRANS_CTRL_I2C;

		if (aux->no_send_addr)
			reg |= DP_AUX_TRANS_CTRL_NO_SEND_ADDR;

		if (aux->no_send_stop)
			reg |= DP_AUX_TRANS_CTRL_NO_SEND_STOP;
	}

	reg |= DP_AUX_TRANS_CTRL_GO;
	msm_dp_write_aux(aux, REG_DP_AUX_TRANS_CTRL, reg);

	return len;
}

static ssize_t msm_dp_aux_cmd_fifo_tx(struct msm_dp_aux_private *aux,
			      struct drm_dp_aux_msg *msg)
{
	ssize_t ret;
	unsigned long time_left;

	reinit_completion(&aux->comp);

	ret = msm_dp_aux_write(aux, msg);
	if (ret < 0)
		return ret;

	time_left = wait_for_completion_timeout(&aux->comp,
						msecs_to_jiffies(250));
	if (!time_left)
		return -ETIMEDOUT;

	return ret;
}

static ssize_t msm_dp_aux_cmd_fifo_rx(struct msm_dp_aux_private *aux,
		struct drm_dp_aux_msg *msg)
{
	u32 data;
	u8 *dp;
	u32 i, actual_i;
	u32 len = msg->size;

	data = msm_dp_read_aux(aux, REG_DP_AUX_TRANS_CTRL);
	data &= ~DP_AUX_TRANS_CTRL_GO;
	msm_dp_write_aux(aux, REG_DP_AUX_TRANS_CTRL, data);

	data = DP_AUX_DATA_INDEX_WRITE; /* INDEX_WRITE */
	data |= DP_AUX_DATA_READ;  /* read */

	msm_dp_write_aux(aux, REG_DP_AUX_DATA, data);

	dp = msg->buffer;

	/* discard first byte */
	data = msm_dp_read_aux(aux, REG_DP_AUX_DATA);

	for (i = 0; i < len; i++) {
		data = msm_dp_read_aux(aux, REG_DP_AUX_DATA);
		*dp++ = (u8)((data >> DP_AUX_DATA_OFFSET) & 0xff);

		actual_i = (data >> DP_AUX_DATA_INDEX_OFFSET) & 0xFF;
		if (i != actual_i)
			break;
	}

	return i;
}

static void msm_dp_aux_update_offset_and_segment(struct msm_dp_aux_private *aux,
					     struct drm_dp_aux_msg *input_msg)
{
	u32 edid_address = 0x50;
	u32 segment_address = 0x30;
	bool i2c_read = input_msg->request &
		(DP_AUX_I2C_READ & DP_AUX_NATIVE_READ);
	u8 *data;

	if (aux->native || i2c_read || ((input_msg->address != edid_address) &&
		(input_msg->address != segment_address)))
		return;


	data = input_msg->buffer;
	if (input_msg->address == segment_address)
		aux->segment = *data;
	else
		aux->offset = *data;
}

/**
 * msm_dp_aux_transfer_helper() - helper function for EDID read transactions
 *
 * @aux: DP AUX private structure
 * @input_msg: input message from DRM upstream APIs
 * @send_seg: send the segment to sink
 *
 * return: void
 *
 * This helper function is used to fix EDID reads for non-compliant
 * sinks that do not handle the i2c middle-of-transaction flag correctly.
 */
static void msm_dp_aux_transfer_helper(struct msm_dp_aux_private *aux,
				   struct drm_dp_aux_msg *input_msg,
				   bool send_seg)
{
	struct drm_dp_aux_msg helper_msg;
	u32 message_size = 0x10;
	u32 segment_address = 0x30;
	u32 const edid_block_length = 0x80;
	bool i2c_mot = input_msg->request & DP_AUX_I2C_MOT;
	bool i2c_read = input_msg->request &
		(DP_AUX_I2C_READ & DP_AUX_NATIVE_READ);

	if (!i2c_mot || !i2c_read || (input_msg->size == 0))
		return;

	/*
	 * Sending the segment value and EDID offset will be performed
	 * from the DRM upstream EDID driver for each block. Avoid
	 * duplicate AUX transactions related to this while reading the
	 * first 16 bytes of each block.
	 */
	if (!(aux->offset % edid_block_length) || !send_seg)
		goto end;

	aux->read = false;
	aux->cmd_busy = true;
	aux->no_send_addr = true;
	aux->no_send_stop = true;

	/*
	 * Send the segment address for every i2c read in which the
	 * middle-of-tranaction flag is set. This is required to support EDID
	 * reads of more than 2 blocks as the segment address is reset to 0
	 * since we are overriding the middle-of-transaction flag for read
	 * transactions.
	 */

	if (aux->segment) {
		memset(&helper_msg, 0, sizeof(helper_msg));
		helper_msg.address = segment_address;
		helper_msg.buffer = &aux->segment;
		helper_msg.size = 1;
		msm_dp_aux_cmd_fifo_tx(aux, &helper_msg);
	}

	/*
	 * Send the offset address for every i2c read in which the
	 * middle-of-transaction flag is set. This will ensure that the sink
	 * will update its read pointer and return the correct portion of the
	 * EDID buffer in the subsequent i2c read trasntion triggered in the
	 * native AUX transfer function.
	 */
	memset(&helper_msg, 0, sizeof(helper_msg));
	helper_msg.address = input_msg->address;
	helper_msg.buffer = &aux->offset;
	helper_msg.size = 1;
	msm_dp_aux_cmd_fifo_tx(aux, &helper_msg);

end:
	aux->offset += message_size;
	if (aux->offset == 0x80 || aux->offset == 0x100)
		aux->segment = 0x0; /* reset segment at end of block */
}

/*
 * This function does the real job to process an AUX transaction.
 * It will call aux_reset() function to reset the AUX channel,
 * if the waiting is timeout.
 */
static ssize_t msm_dp_aux_transfer(struct drm_dp_aux *msm_dp_aux,
			       struct drm_dp_aux_msg *msg)
{
	ssize_t ret;
	int const aux_cmd_native_max = 16;
	int const aux_cmd_i2c_max = 128;
	struct msm_dp_aux_private *aux;

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	aux->native = msg->request & (DP_AUX_NATIVE_WRITE & DP_AUX_NATIVE_READ);

	/* Ignore address only message */
	if (msg->size == 0 || !msg->buffer) {
		msg->reply = aux->native ?
			DP_AUX_NATIVE_REPLY_ACK : DP_AUX_I2C_REPLY_ACK;
		return msg->size;
	}

	/* msg sanity check */
	if ((aux->native && msg->size > aux_cmd_native_max) ||
	    msg->size > aux_cmd_i2c_max) {
		DRM_ERROR("%s: invalid msg: size(%zu), request(%x)\n",
			__func__, msg->size, msg->request);
		return -EINVAL;
	}

	ret = pm_runtime_resume_and_get(msm_dp_aux->dev);
	if (ret)
		return  ret;

	mutex_lock(&aux->mutex);
	if (!aux->initted) {
		ret = -EIO;
		goto exit;
	}

	/*
	 * If we're using DP and an external display isn't connected then the
	 * transfer won't succeed. Return right away. If we don't do this we
	 * can end up with long timeouts if someone tries to access the DP AUX
	 * character device when no DP device is connected.
	 */
	if (!aux->is_edp && !aux->enable_xfers) {
		ret = -ENXIO;
		goto exit;
	}

	msm_dp_aux_update_offset_and_segment(aux, msg);
	msm_dp_aux_transfer_helper(aux, msg, true);

	aux->read = msg->request & (DP_AUX_I2C_READ & DP_AUX_NATIVE_READ);
	aux->cmd_busy = true;

	if (aux->read) {
		aux->no_send_addr = true;
		aux->no_send_stop = false;
	} else {
		aux->no_send_addr = true;
		aux->no_send_stop = true;
	}

	ret = msm_dp_aux_cmd_fifo_tx(aux, msg);
	if (ret < 0) {
		if (aux->native) {
			aux->retry_cnt++;
			if (!(aux->retry_cnt % MAX_AUX_RETRIES))
				phy_calibrate(aux->phy);
		}
		/* reset aux if link is in connected state */
		if (msm_dp_aux_is_link_connected(msm_dp_aux))
			msm_dp_aux_reset(aux);
	} else {
		aux->retry_cnt = 0;
		switch (aux->aux_error_num) {
		case DP_AUX_ERR_NONE:
			if (aux->read)
				ret = msm_dp_aux_cmd_fifo_rx(aux, msg);
			msg->reply = aux->native ? DP_AUX_NATIVE_REPLY_ACK : DP_AUX_I2C_REPLY_ACK;
			break;
		case DP_AUX_ERR_DEFER:
			msg->reply = aux->native ? DP_AUX_NATIVE_REPLY_DEFER : DP_AUX_I2C_REPLY_DEFER;
			break;
		case DP_AUX_ERR_PHY:
		case DP_AUX_ERR_ADDR:
		case DP_AUX_ERR_NACK:
		case DP_AUX_ERR_NACK_DEFER:
			msg->reply = aux->native ? DP_AUX_NATIVE_REPLY_NACK : DP_AUX_I2C_REPLY_NACK;
			break;
		case DP_AUX_ERR_TOUT:
			ret = -ETIMEDOUT;
			break;
		}
	}

	aux->cmd_busy = false;

exit:
	mutex_unlock(&aux->mutex);
	pm_runtime_put_sync(msm_dp_aux->dev);

	return ret;
}

irqreturn_t msm_dp_aux_isr(struct drm_dp_aux *msm_dp_aux, u32 isr)
{
	struct msm_dp_aux_private *aux;

	if (!msm_dp_aux) {
		DRM_ERROR("invalid input\n");
		return IRQ_NONE;
	}

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	if (!aux->cmd_busy) {
		DRM_ERROR("Unexpected DP AUX IRQ %#010x when not busy\n", isr);
		return IRQ_NONE;
	}

	/*
	 * The logic below assumes only one error bit is set (other than "done"
	 * which can apparently be set at the same time as some of the other
	 * bits). Warn if more than one get set so we know we need to improve
	 * the logic.
	 */
	if (hweight32(isr & ~DP_INTR_AUX_XFER_DONE) > 1)
		DRM_WARN("Some DP AUX interrupts unhandled: %#010x\n", isr);

	if (isr & DP_INTR_AUX_ERROR) {
		aux->aux_error_num = DP_AUX_ERR_PHY;
		msm_dp_aux_clear_hw_interrupts(aux);
	} else if (isr & DP_INTR_NACK_DEFER) {
		aux->aux_error_num = DP_AUX_ERR_NACK_DEFER;
	} else if (isr & DP_INTR_WRONG_ADDR) {
		aux->aux_error_num = DP_AUX_ERR_ADDR;
	} else if (isr & DP_INTR_TIMEOUT) {
		aux->aux_error_num = DP_AUX_ERR_TOUT;
	} else if (!aux->native && (isr & DP_INTR_I2C_NACK)) {
		aux->aux_error_num = DP_AUX_ERR_NACK;
	} else if (!aux->native && (isr & DP_INTR_I2C_DEFER)) {
		if (isr & DP_INTR_AUX_XFER_DONE)
			aux->aux_error_num = DP_AUX_ERR_NACK;
		else
			aux->aux_error_num = DP_AUX_ERR_DEFER;
	} else if (isr & DP_INTR_AUX_XFER_DONE) {
		aux->aux_error_num = DP_AUX_ERR_NONE;
	} else {
		DRM_WARN("Unexpected interrupt: %#010x\n", isr);
		return IRQ_NONE;
	}

	complete(&aux->comp);

	return IRQ_HANDLED;
}

void msm_dp_aux_enable_xfers(struct drm_dp_aux *msm_dp_aux, bool enabled)
{
	struct msm_dp_aux_private *aux;

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	aux->enable_xfers = enabled;
}

void msm_dp_aux_reconfig(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux;

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	phy_calibrate(aux->phy);
	msm_dp_aux_reset(aux);
}

void msm_dp_aux_init(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux;

	if (!msm_dp_aux) {
		DRM_ERROR("invalid input\n");
		return;
	}

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	mutex_lock(&aux->mutex);

	msm_dp_aux_enable(aux);
	aux->retry_cnt = 0;
	aux->initted = true;

	mutex_unlock(&aux->mutex);
}

void msm_dp_aux_deinit(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux;

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	mutex_lock(&aux->mutex);

	aux->initted = false;
	msm_dp_aux_disable(aux);

	mutex_unlock(&aux->mutex);
}

int msm_dp_aux_register(struct drm_dp_aux *msm_dp_aux)
{
	int ret;

	if (!msm_dp_aux) {
		DRM_ERROR("invalid input\n");
		return -EINVAL;
	}

	ret = drm_dp_aux_register(msm_dp_aux);
	if (ret) {
		DRM_ERROR("%s: failed to register drm aux: %d\n", __func__,
				ret);
		return ret;
	}

	return 0;
}

void msm_dp_aux_unregister(struct drm_dp_aux *msm_dp_aux)
{
	drm_dp_aux_unregister(msm_dp_aux);
}

static int msm_dp_wait_hpd_asserted(struct drm_dp_aux *msm_dp_aux,
				 unsigned long wait_us)
{
	int ret;
	struct msm_dp_aux_private *aux;

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	ret = pm_runtime_resume_and_get(aux->dev);
	if (ret)
		return ret;

	ret = msm_dp_aux_wait_for_hpd_connect_state(aux, wait_us);
	pm_runtime_put_sync(aux->dev);

	return ret;
}

void msm_dp_aux_hpd_enable(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux =
		container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	u32 reg;

	/* Configure REFTIMER and enable it */
	reg = msm_dp_read_aux(aux, REG_DP_DP_HPD_REFTIMER);
	reg |= DP_DP_HPD_REFTIMER_ENABLE;
	msm_dp_write_aux(aux, REG_DP_DP_HPD_REFTIMER, reg);

	/* Enable HPD */
	msm_dp_write_aux(aux, REG_DP_DP_HPD_CTRL, DP_DP_HPD_CTRL_HPD_EN);
}

void msm_dp_aux_hpd_disable(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux =
		container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	u32 reg;

	reg = msm_dp_read_aux(aux, REG_DP_DP_HPD_REFTIMER);
	reg &= ~DP_DP_HPD_REFTIMER_ENABLE;
	msm_dp_write_aux(aux, REG_DP_DP_HPD_REFTIMER, reg);

	msm_dp_write_aux(aux, REG_DP_DP_HPD_CTRL, 0);
}

void msm_dp_aux_hpd_intr_enable(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux =
		container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	u32 reg;

	reg = msm_dp_read_aux(aux, REG_DP_DP_HPD_INT_MASK);
	reg |= DP_DP_HPD_INT_MASK;
	msm_dp_write_aux(aux, REG_DP_DP_HPD_INT_MASK,
		     reg & DP_DP_HPD_INT_MASK);
}

void msm_dp_aux_hpd_intr_disable(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux =
		container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	u32 reg;

	reg = msm_dp_read_aux(aux, REG_DP_DP_HPD_INT_MASK);
	reg &= ~DP_DP_HPD_INT_MASK;
	msm_dp_write_aux(aux, REG_DP_DP_HPD_INT_MASK,
		     reg & DP_DP_HPD_INT_MASK);
}

u32 msm_dp_aux_get_hpd_intr_status(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux =
		container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	int isr, mask;

	isr = msm_dp_read_aux(aux, REG_DP_DP_HPD_INT_STATUS);
	msm_dp_write_aux(aux, REG_DP_DP_HPD_INT_ACK,
				 (isr & DP_DP_HPD_INT_MASK));
	mask = msm_dp_read_aux(aux, REG_DP_DP_HPD_INT_MASK);

	/*
	 * We only want to return interrupts that are unmasked to the caller.
	 * However, the interrupt status field also contains other
	 * informational bits about the HPD state status, so we only mask
	 * out the part of the register that tells us about which interrupts
	 * are pending.
	 */
	return isr & (mask | ~DP_DP_HPD_INT_MASK);
}

u32 msm_dp_aux_is_link_connected(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux =
		container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);
	u32 status;

	status = msm_dp_read_aux(aux, REG_DP_DP_HPD_INT_STATUS);
	status >>= DP_DP_HPD_STATE_STATUS_BITS_SHIFT;
	status &= DP_DP_HPD_STATE_STATUS_BITS_MASK;

	return status;
}

struct drm_dp_aux *msm_dp_aux_get(struct device *dev,
			      struct phy *phy,
			      bool is_edp,
			      void __iomem *aux_base)
{
	struct msm_dp_aux_private *aux;

	aux = devm_kzalloc(dev, sizeof(*aux), GFP_KERNEL);
	if (!aux)
		return ERR_PTR(-ENOMEM);

	init_completion(&aux->comp);
	aux->cmd_busy = false;
	aux->is_edp = is_edp;
	mutex_init(&aux->mutex);

	aux->dev = dev;
	aux->phy = phy;
	aux->retry_cnt = 0;
	aux->aux_base = aux_base;

	/*
	 * Use the drm_dp_aux_init() to use the aux adapter
	 * before registering AUX with the DRM device so that
	 * msm eDP panel can be detected by generic_dep_panel_probe().
	 */
	aux->msm_dp_aux.name = "dpu_dp_aux";
	aux->msm_dp_aux.dev = dev;
	aux->msm_dp_aux.transfer = msm_dp_aux_transfer;
	aux->msm_dp_aux.wait_hpd_asserted = msm_dp_wait_hpd_asserted;
	drm_dp_aux_init(&aux->msm_dp_aux);

	return &aux->msm_dp_aux;
}

void msm_dp_aux_put(struct drm_dp_aux *msm_dp_aux)
{
	struct msm_dp_aux_private *aux;

	if (!msm_dp_aux)
		return;

	aux = container_of(msm_dp_aux, struct msm_dp_aux_private, msm_dp_aux);

	mutex_destroy(&aux->mutex);

	devm_kfree(aux->dev, aux);
}
