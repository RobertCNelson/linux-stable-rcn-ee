// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip PFSoC check device cert driver
 *
 * Copyright (c) 2020 Microchip Corporation. All rights reserved.
 *
 * Author:
 *
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/of_platform.h>
#include <soc/microchip/mpfs.h>

#define MPFS_CERT_RESP_SIZE_BYTES 1024U

#define CMD_OPCODE 0x3U
#define CMD_DATA_SIZE 0U
#define CMD_DATA NULL
#define MBOX_OFFSET 0U
#define RESP_OFFSET 0U

static DEFINE_MUTEX(mpfs_device_cert_mutex);

struct mpfs_device_cert_priv {
	struct mpfs_sys_controller *sys_controller;
};

struct mpfs_device_cert_priv *device_cert_priv;

static ssize_t mpfs_device_cert_read(struct file *filp, char __user *userbuf,
				size_t len, loff_t *f_pos)
{
	u8 response_msg[MPFS_CERT_RESP_SIZE_BYTES];
	u8 buffer[2 * MPFS_CERT_RESP_SIZE_BYTES + 3];
	u8 *bufferp = buffer;
	u32 i;

	struct mpfs_mss_response response = {
		.resp_status = 0U,
		.resp_msg = (u32 *)response_msg,
		.resp_size = MPFS_CERT_RESP_SIZE_BYTES
	};
	struct mpfs_mss_msg msg = { .cmd_opcode = CMD_OPCODE,
				    .cmd_data_size = CMD_DATA_SIZE,
				    .response = &response,
				    .cmd_data = CMD_DATA,
				    .mbox_offset = MBOX_OFFSET,
				    .resp_offset = RESP_OFFSET };

	int ret = mpfs_blocking_transaction(device_cert_priv->sys_controller, &msg);
	if (ret)
		return -EIO;
	bufferp += sprintf(bufferp, "%02x ", response.resp_status);
	for (i = 0; i < MPFS_CERT_RESP_SIZE_BYTES; i++)
		bufferp += sprintf(bufferp, "%02x", response_msg[i]);

	return simple_read_from_buffer(userbuf, len, f_pos, buffer,
				       2 * MPFS_CERT_RESP_SIZE_BYTES + 3);
}

static int mpfs_device_cert_open(struct inode *inode, struct file *filp)
{
	if (!mutex_trylock(&mpfs_device_cert_mutex)) {
		pr_debug("Device Busy\n");
		return -EBUSY;
	}
	return 0;
}

static int mpfs_device_cert_release(struct inode *inode, struct file *filp)
{
	mutex_unlock(&mpfs_device_cert_mutex);
	return 0;
}

static const struct file_operations mpfs_device_cert_fops = {
	.owner = THIS_MODULE,
	.read = mpfs_device_cert_read,
	.open = mpfs_device_cert_open,
	.release = mpfs_device_cert_release
};

static struct miscdevice mpfs_device_cert_dev = { .minor = MISC_DYNAMIC_MINOR,
					     .name = "mpfs_device_cert_num",
					     .fops = &mpfs_device_cert_fops };

static int mpfs_device_cert_probe(struct platform_device *pdev)
{
	struct device_node *sys_controller_np;
	struct device *dev = &pdev->dev;

	device_cert_priv = devm_kzalloc(dev, sizeof(*device_cert_priv), GFP_KERNEL);
	if (!device_cert_priv)
		return -ENOMEM;

	sys_controller_np =
		of_parse_phandle(pdev->dev.of_node, "syscontroller", 0);
	if (!sys_controller_np) {
		dev_err(&pdev->dev,
			"Failed to find mpfs system controller node\n");
		return -ENODEV;
	}

	device_cert_priv->sys_controller =
		mpfs_sys_controller_get(sys_controller_np);
	of_node_put(sys_controller_np);
	if (!device_cert_priv->sys_controller)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, device_cert_priv);
	misc_register(&mpfs_device_cert_dev);
	dev_info(&pdev->dev,
		 "Successfully registered mpfs device cert driver\n");

	return 0;
}

static const struct of_device_id mpfs_device_cert_of_match[] = {
	{
		.compatible = "microchip,polarfire-soc-device-cert",
	},
	{
		.compatible = "microchip,mpfs-device-cert",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_device_cert_of_match);

static struct platform_driver mpfs_device_cert_driver = {
	.driver = {
	.name = "mpfs-device-cert",
	.of_match_table = mpfs_device_cert_of_match,
	},
	.probe = mpfs_device_cert_probe,
};
module_platform_driver(mpfs_device_cert_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("PFSoC device cert driver");
