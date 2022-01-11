// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip PolarFire SoC (MPFS) hardware random driver
 *
 * Copyright (c) 2020 Microchip Corporation. All rights reserved.
 *
 * Author:
 *
 */

#include <linux/module.h>
#include <linux/hw_random.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <soc/microchip/mpfs.h>

#define RNG_RESP_BYTES 32U
#define CMD_OPCODE 0x21
#define CMD_DATA_SIZE 0U
#define RESP_SIZE (RNG_RESP_BYTES / 4U)
#define CMD_DATA NULL
#define MBOX_OFFSET 0U
#define RESP_OFFSET 0U
#define check_excess_response(nw, mw) (nw > mw ? sizeof(u32) * (nw - mw) : 0U)

static struct mpfs_rng_priv {
	struct mpfs_sys_controller *sys_controller;
	struct hwrng ops;
};

static int mpfs_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	struct mpfs_rng_priv *rng_priv = (struct mpfs_rng_priv *)rng->priv;

	u32 max_words = max / sizeof(u32);
	u32 num_words_rx = 0;
	u32 num_requests;
	u32 response_msg[RESP_SIZE];
	u16 i;
	u16 copy_size_bytes;
	int ret;

	struct mpfs_mss_response response = {
		.resp_status = 0U,
		.resp_msg = (u32 *)response_msg,
		.resp_size = RNG_RESP_BYTES
	};
	struct mpfs_mss_msg msg = { .cmd_opcode = CMD_OPCODE,
				    .cmd_data_size = CMD_DATA_SIZE,
				    .response = &response,
				    .cmd_data = CMD_DATA,
				    .mbox_offset = MBOX_OFFSET,
				    .resp_offset = RESP_OFFSET };

	if (!max)
		return 0;

	num_requests = 1 + (max_words - 1) / RESP_SIZE;

	for (i = 0; i < num_requests; i++) {
		ret = mpfs_blocking_transaction(rng_priv->sys_controller, &msg);
		if (ret)
			return ret;

		num_words_rx += RESP_SIZE;
		copy_size_bytes = RNG_RESP_BYTES - check_excess_response(num_words_rx, max_words);

		memcpy((u32 *)buf + i * RESP_SIZE, response_msg, copy_size_bytes);
	}

	return num_words_rx * sizeof(u32);
}

static int mpfs_rng_number_probe(struct platform_device *pdev)
{
	struct device_node *sys_controller_np;
	struct device *dev = &pdev->dev;
	struct mpfs_rng_priv *rng_priv;
	int ret;

	rng_priv = devm_kzalloc(dev, sizeof(*rng_priv), GFP_KERNEL);
	if (!rng_priv)
		return -ENOMEM;

	sys_controller_np = of_parse_phandle(pdev->dev.of_node, "syscontroller", 0);
	if (!sys_controller_np) {
		dev_err(&pdev->dev,
			"Failed to find mpfs system controller node\n");
		return -ENODEV;
	}

	rng_priv->sys_controller = mpfs_sys_controller_get(sys_controller_np);
	of_node_put(sys_controller_np);
	if (!rng_priv->sys_controller)
		return -EPROBE_DEFER;

	rng_priv->ops.priv = (unsigned long)rng_priv;
	rng_priv->ops.read = mpfs_rng_read;
	rng_priv->ops.name = pdev->name;

	platform_set_drvdata(pdev, rng_priv);

	ret = devm_hwrng_register(&pdev->dev, &rng_priv->ops);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register HW RNG\n");
		return ret;
	}

	dev_info(&pdev->dev, "Successfully registered HW RNG\n");

	return 0;
}

static const struct of_device_id mpfs_rng_number_of_match[] = {
	{
		.compatible = "microchip,polarfire-soc-rng",
	},
	{
		.compatible = "microchip,mpfs-rng",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_rng_number_of_match);

static struct platform_driver mpfs_rng_number_driver = {
	.driver = {
		.name = "mpfs-rng",
		.of_match_table = mpfs_rng_number_of_match,
	},
	.probe = mpfs_rng_number_probe,
};
module_platform_driver(mpfs_rng_number_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("mpfs mailbox client driver");
