// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/rpmsg.h>

#include "thames_rpmsg.h"
#include "thames_core.h"
#include "thames_device.h"
#include "thames_ipc.h"

#define THAMES_PING_TEST_PATTERN 0xDEADBEEF
#define THAMES_PING_TIMEOUT_MS 5000

static int thames_rpmsg_callback(struct rpmsg_device *rpdev, void *data, int len, void *priv,
				 u32 src)
{
	struct thames_msg_hdr *hdr = (struct thames_msg_hdr *)data;
	struct thames_core *core = priv;

	dev_dbg(&rpdev->dev, "Received response on core %d with length %d\n", core->index, len);

	if (len < sizeof(struct thames_msg_hdr)) {
		dev_err(&rpdev->dev, "Received message too short: %d bytes", len);
		return -EINVAL;
	}

	switch (hdr->type) {
	case THAMES_MSG_PING_RESPONSE: {
		struct thames_msg_response *response = (struct thames_msg_response *)data;

		dev_dbg(&rpdev->dev,
			"Received PING response: status=%u, data=0x%x, expected_data=0x%x, seq=%u, expected_seq=%u\n",
			response->status, response->data, core->rpmsg_ctx.ping_test.expected_data,
			hdr->seq, core->rpmsg_ctx.ping_test.sequence);

		if (hdr->seq != core->rpmsg_ctx.ping_test.sequence) {
			dev_err(&rpdev->dev,
				"PING response sequence mismatch: got %u, expected %u\n", hdr->seq,
				core->rpmsg_ctx.ping_test.sequence);
			ida_free(&core->tdev->ipc_seq_ida, hdr->seq);
			return -EINVAL;
		}

		if (response->data != core->rpmsg_ctx.ping_test.expected_data) {
			dev_err(&rpdev->dev,
				"PING response data mismatch: got 0x%x, expected 0x%x\n",
				response->data, core->rpmsg_ctx.ping_test.expected_data);
			core->rpmsg_ctx.ping_test.success = false;
			complete(&core->rpmsg_ctx.ping_test.completion);
			ida_free(&core->tdev->ipc_seq_ida, hdr->seq);
			return -EINVAL;
		}

		core->rpmsg_ctx.ping_test.success = (response->status == THAMES_RESP_SUCCESS);
		complete(&core->rpmsg_ctx.ping_test.completion);

		ida_free(&core->tdev->ipc_seq_ida, hdr->seq);

		break;
	}

	case THAMES_MSG_CONTEXT_OP_RESPONSE:
		ida_free(&core->tdev->ipc_seq_ida, hdr->seq);
		break;

	case THAMES_MSG_BO_OP_RESPONSE:
		ida_free(&core->tdev->ipc_seq_ida, hdr->seq);
		break;

	default:
		dev_warn(&rpdev->dev, "Unknown message type: %u\n", hdr->type);
		break;
	}

	return 0;
}

static int thames_rpmsg_send_raw(struct thames_core *core, const void *data, size_t len)
{
	if (!core->rpmsg_ctx.endpoint) {
		dev_err(core->dev, "RPMSG endpoint not available");
		return -ENODEV;
	}

	return rpmsg_send(core->rpmsg_ctx.endpoint, (void *)data, len);
}

int thames_rpmsg_init(struct thames_core *core)
{
	struct rpmsg_device *rpdev = core->rpdev;
	struct rpmsg_channel_info chinfo = {};

	strscpy(chinfo.name, rpdev->id.name, sizeof(chinfo.name));
	chinfo.src = RPMSG_ADDR_ANY; /* Let rpmsg assign an address */
	chinfo.dst = RPMSG_ADDR_ANY;

	core->rpmsg_ctx.endpoint = rpmsg_create_ept(rpdev, thames_rpmsg_callback, core, chinfo);
	if (!core->rpmsg_ctx.endpoint) {
		dev_err(core->dev, "Failed to create RPMSG endpoint for core %d", core->index);
		return -ENODEV;
	}

	return 0;
}

void thames_rpmsg_fini(struct thames_core *core)
{
	if (core->rpmsg_ctx.endpoint) {
		rpmsg_destroy_ept(core->rpmsg_ctx.endpoint);
		core->rpmsg_ctx.endpoint = NULL;
	}
}

int thames_rpmsg_send_ping(struct thames_core *core, u32 ping_data, u32 *sequence)
{
	struct thames_msg_ping ping_msg = {};

	ping_msg.hdr.type = THAMES_MSG_PING;
	ping_msg.hdr.seq = ida_alloc(&core->tdev->ipc_seq_ida, GFP_KERNEL);
	ping_msg.hdr.len = sizeof(ping_msg);
	ping_msg.hdr.reserved = 0;
	ping_msg.ping_data = ping_data;

	*sequence = ping_msg.hdr.seq;

	return thames_rpmsg_send_raw(core, &ping_msg, sizeof(ping_msg));
}

int thames_rpmsg_send_create_context(struct thames_core *core, u32 context_id)
{
	struct thames_msg_context_op msg = {};

	msg.hdr.type = THAMES_MSG_CONTEXT_OP;
	msg.hdr.seq = ida_alloc(&core->tdev->ipc_seq_ida, GFP_KERNEL);
	msg.hdr.len = sizeof(msg);
	msg.op = THAMES_CONTEXT_CREATE;
	msg.context_id = context_id;

	return thames_rpmsg_send_raw(core, &msg, sizeof(msg));
}

int thames_rpmsg_send_destroy_context(struct thames_core *core, u32 context_id)
{
	struct thames_msg_context_op msg = {};

	msg.hdr.type = THAMES_MSG_CONTEXT_OP;
	msg.hdr.seq = ida_alloc(&core->tdev->ipc_seq_ida, GFP_KERNEL);
	msg.hdr.len = sizeof(msg);
	msg.op = THAMES_CONTEXT_DESTROY;
	msg.context_id = context_id;

	return thames_rpmsg_send_raw(core, &msg, sizeof(msg));
}

int thames_rpmsg_send_map_bo(struct thames_core *core, u32 context_id, u32 bo_id, u64 vaddr,
			     u64 paddr, u64 size)
{
	struct thames_msg_bo_op msg = {};

	msg.hdr.type = THAMES_MSG_BO_OP;
	msg.hdr.seq = ida_alloc(&core->tdev->ipc_seq_ida, GFP_KERNEL);
	msg.hdr.len = sizeof(msg);
	msg.op = THAMES_BO_MAP;
	msg.context_id = context_id;
	msg.bo_id = bo_id;
	msg.vaddr = vaddr;
	msg.paddr = paddr;
	msg.size = size;

	return thames_rpmsg_send_raw(core, &msg, sizeof(msg));
}

int thames_rpmsg_send_unmap_bo(struct thames_core *core, u32 context_id, u32 bo_id)
{
	struct thames_msg_bo_op msg = {};

	msg.hdr.type = THAMES_MSG_BO_OP;
	msg.hdr.seq = ida_alloc(&core->tdev->ipc_seq_ida, GFP_KERNEL);
	msg.hdr.len = sizeof(msg);
	msg.op = THAMES_BO_UNMAP;
	msg.context_id = context_id;
	msg.bo_id = bo_id;
	msg.vaddr = 0;
	msg.paddr = 0;
	msg.size = 0;

	return thames_rpmsg_send_raw(core, &msg, sizeof(msg));
}

int thames_rpmsg_ping_test(struct thames_core *core)
{
	const u32 test_data = THAMES_PING_TEST_PATTERN;
	int ret;
	unsigned long timeout;

	core->rpmsg_ctx.ping_test.expected_data = test_data;
	core->rpmsg_ctx.ping_test.success = false;
	init_completion(&core->rpmsg_ctx.ping_test.completion);

	ret = thames_rpmsg_send_ping(core, test_data, &core->rpmsg_ctx.ping_test.sequence);
	if (ret) {
		dev_err(core->dev, "Failed to send PING message to core %d: %d", core->index, ret);
		return ret;
	}

	timeout = msecs_to_jiffies(THAMES_PING_TIMEOUT_MS);
	ret = wait_for_completion_timeout(&core->rpmsg_ctx.ping_test.completion, timeout);
	if (ret == 0) {
		dev_err(core->dev, "PING test timed out - DSP core %d not responding", core->index);
		return -ETIMEDOUT;
	}

	if (!core->rpmsg_ctx.ping_test.success) {
		dev_err(core->dev, "PING test failed - incorrect PONG response from DSP core %d",
			core->index);
		return -EIO;
	}

	return 0;
}
