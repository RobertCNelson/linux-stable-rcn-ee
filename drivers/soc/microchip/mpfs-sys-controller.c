// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip PolarFire SoC (MPFS) system controller driver
 *
 * Copyright (c) 2020 Microchip Corporation. All rights reserved.
 *
 * Author: Conor Dooley <conor.dooley@microchip.com>
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/mailbox_client.h>
#include <linux/platform_device.h>
#include <soc/microchip/mpfs.h>

static DEFINE_MUTEX(transaction_lock);

struct mpfs_sys_controller {
	struct mbox_client client;
	struct mbox_chan *chan;
	struct completion c;
	u32 enabled;
};

int mpfs_blocking_transaction(struct mpfs_sys_controller *mpfs_client, void *msg)
{
	int ret;

	mutex_lock_interruptible(&transaction_lock);

	reinit_completion(&mpfs_client->c);

	ret = mbox_send_message(mpfs_client->chan, msg);
	if (ret >= 0) {
		if (wait_for_completion_timeout(&mpfs_client->c, HZ)) {
			ret = 0;
		} else {
			ret = -ETIMEDOUT;
			dev_warn(mpfs_client->client.dev, "MPFS sys controller transaction timeout\n");
		}
	} else {
		dev_err(mpfs_client->client.dev,
			"mpfs sys controller transaction returned %d\n", ret);
	}

	mutex_unlock(&transaction_lock);

	return ret;
}
EXPORT_SYMBOL(mpfs_blocking_transaction);

static void rx_callback(struct mbox_client *client, void *msg)
{
	struct mpfs_sys_controller *mpfs_client =
		container_of(client, struct mpfs_sys_controller, client);

	complete(&mpfs_client->c);
}

static int mpfs_sys_controller_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mpfs_sys_controller *mpfs_client;

	mpfs_client = devm_kzalloc(dev, sizeof(*mpfs_client), GFP_KERNEL);
	if (!mpfs_client)
		return -ENOMEM;

	mpfs_client->client.dev = dev;
	mpfs_client->client.rx_callback = rx_callback;
	mpfs_client->client.tx_block = 1U;

	mpfs_client->chan = mbox_request_channel(&mpfs_client->client, 0);
	if (IS_ERR(mpfs_client->chan))
		return dev_err_probe(dev, PTR_ERR(mpfs_client->chan),
				     "Failed to get mbox channel\n");

	init_completion(&mpfs_client->c);

	platform_set_drvdata(pdev, mpfs_client);

	dev_info(&pdev->dev, "Registered MPFS system controller driver\n");

	return 0;
}

struct mpfs_sys_controller *
mpfs_sys_controller_get(struct device_node *mss_node)
{
	struct platform_device *pdev = of_find_device_by_node(mss_node);

	if (!pdev)
		return NULL;

	return platform_get_drvdata(pdev);
}
EXPORT_SYMBOL(mpfs_sys_controller_get);

static const struct of_device_id mpfs_sys_controller_of_match[] = {
	{
		.compatible = "microchip,polarfire-soc-sys-controller",
	},
	{
		.compatible = "microchip,mpfs-sys-controller",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_sys_controller_of_match);

static struct platform_driver mpfs_sys_controller_driver = {
	.driver = {
		.name = "mpfs-sys-controller",
		.of_match_table = mpfs_sys_controller_of_match,
	},
	.probe = mpfs_sys_controller_probe,
};
module_platform_driver(mpfs_sys_controller_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("MPFS system controller driver");
