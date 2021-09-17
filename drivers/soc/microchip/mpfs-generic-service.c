// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip MPFS generic_service driver
 *
 * Copyright (c) 2020 Microchip Corporation. All rights reserved.
 *
 * Author: Conor Dooley
 *
 */

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/of_platform.h>
#include <soc/microchip/mpfs.h>

static DEFINE_MUTEX(mpfs_generic_service_mutex);

struct mpfs_generic_service_priv {
	struct mpfs_sys_controller *sys_controller;
	u8 *buffer;
	size_t len;
	u8 cmd_opcode;
	u16 cmd_size_bytes;
	u16 resp_size_bytes;
	u16 mbox_offset;
	u16 resp_offset;
};

struct mpfs_generic_service_priv *generic_service_priv;

static ssize_t mpfs_generic_service_write(struct file *filep, const char __user *userbuf,
				    size_t len, loff_t *offset)
{
	u8 *cmd_data;

	if (len < 9U || len > 4096U)
		return -1;

	cmd_data = kmalloc(len, GFP_KERNEL);
	if (!cmd_data)
		return -ENOMEM;

	int ret = copy_from_user(cmd_data, userbuf, len);
	if (ret)
		return -ret;

	generic_service_priv->buffer = cmd_data+9;
	generic_service_priv->len = len - 9;

	generic_service_priv->cmd_opcode = *(cmd_data);
	generic_service_priv->cmd_size_bytes = (u16)(*(cmd_data + 1)) + (u16)(*(cmd_data + 2) << 8);
	generic_service_priv->resp_size_bytes =
		ALIGN((u16)(*(cmd_data + 3)) + ((u16)(*(cmd_data + 4) << 8)), (4U));
	generic_service_priv->mbox_offset = (u16)(*(cmd_data + 5)) + ((u16)(*(cmd_data + 6) << 8));
	generic_service_priv->resp_offset = (u16)(*(cmd_data + 7)) + ((u16)(*(cmd_data + 8) << 8));

	return len;
}

static ssize_t mpfs_generic_service_read(struct file *filp, char __user *userbuf,
				     size_t len, loff_t *f_pos)
{
	u8 response_msg[generic_service_priv->resp_size_bytes];
	u8 buffer[2 * generic_service_priv->resp_size_bytes + 3];
	u8 *cmd_data = generic_service_priv->buffer;
	u8 *bufferp = buffer;
	u32 i;

	struct mpfs_mss_response response = {
		.resp_status = 0U,
		.resp_msg = (u32 *)response_msg,
		.resp_size = generic_service_priv->resp_size_bytes
	};

	struct mpfs_mss_msg msg = {
		.cmd_opcode = generic_service_priv->cmd_opcode,
		.cmd_data_size = generic_service_priv->cmd_size_bytes,
		.response = &response,
		.cmd_data = cmd_data,
		.mbox_offset = generic_service_priv->mbox_offset,
		.resp_offset = generic_service_priv->resp_offset
	};

	if (!generic_service_priv->buffer || generic_service_priv->len != generic_service_priv->cmd_size_bytes)
		return -EFAULT;

	int ret = mpfs_blocking_transaction(generic_service_priv->sys_controller, &msg);
	kfree(generic_service_priv->buffer);

	if (ret)
		return -EIO;
	bufferp += sprintf(bufferp, "%02x ", response.resp_status);
	for (i = 0; i < generic_service_priv->resp_size_bytes; i++)
		bufferp += sprintf(bufferp, "%02x", response_msg[i]);

	return simple_read_from_buffer(
		userbuf, len, f_pos, buffer,
		2 * generic_service_priv->resp_size_bytes + 3);
}

static int mpfs_generic_service_open(struct inode *inode, struct file *filp)
{
	if (!mutex_trylock(&mpfs_generic_service_mutex)) {
		pr_debug("Device Busy\n");
		return -EBUSY;
	}
	return 0;
}

static int mpfs_generic_service_release(struct inode *inode, struct file *filp)
{
	mutex_unlock(&mpfs_generic_service_mutex);
	return 0;
}

static const struct file_operations mpfs_generic_service_fops = {
	.owner = THIS_MODULE,
	.read = mpfs_generic_service_read,
	.open = mpfs_generic_service_open,
	.write = mpfs_generic_service_write,
	.release = mpfs_generic_service_release
};

static struct miscdevice mpfs_generic_service_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mpfs_generic_service",
	.fops = &mpfs_generic_service_fops
};

static int mpfs_generic_service_probe(struct platform_device *pdev)
{
	struct device_node *sys_controller_np;
	struct device *dev = &pdev->dev;

	generic_service_priv =
		devm_kzalloc(dev, sizeof(*generic_service_priv), GFP_KERNEL);
	if (!generic_service_priv)
		return -ENOMEM;
	
	sys_controller_np =
		of_parse_phandle(pdev->dev.of_node, "syscontroller", 0);
	if (!sys_controller_np) {
		dev_err(&pdev->dev,
			"Failed to find mpfs system controller node\n");
		return -ENODEV;
	}

	generic_service_priv->sys_controller =
		mpfs_sys_controller_get(sys_controller_np);
	of_node_put(sys_controller_np);
	if (!generic_service_priv->sys_controller)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, generic_service_priv);
	misc_register(&mpfs_generic_service_dev);
	dev_info(&pdev->dev,
		 "Successfully registered mpfs generic_service driver\n");

	return 0;
}

static const struct of_device_id mpfs_generic_service_of_match[] = {
	{
		.compatible = "microchip,polarfire-soc-generic-service",
	},
	{
		.compatible = "microchip,mpfs-generic-service",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_generic_service_of_match);

static struct platform_driver mpfs_generic_service_driver = {
	.driver = {
	.name = "mpfs-generic-service",
	.of_match_table = mpfs_generic_service_of_match,
	},
	.probe = mpfs_generic_service_probe,
};
module_platform_driver(mpfs_generic_service_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("PFSoC generic_service driver");
