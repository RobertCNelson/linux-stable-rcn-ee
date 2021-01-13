// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip PFSoC check signature driver
 *
 * Copyright (c) 2020 Microchip Corporation. All rights reserved.
 *
 * Author:
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

#define MPFS_SIG_RESP_SIZE_BYTES 104U //96U raw format, DER is 104
#define MPFS_SIG_CMD_SIZE_BYTES 48U

#define CMD_OPCODE 0x1AU //0x19 raw format, DER is 0x1A
#define CMD_DATA NULL
#define MBOX_OFFSET 0U
#define RESP_OFFSET ALIGN((MPFS_SIG_CMD_SIZE_BYTES), (4U))

static DEFINE_MUTEX(mpfs_signature_mutex);

struct mpfs_signature_priv {
	struct mpfs_sys_controller *sys_controller;
	u8 *buffer; //of 48?
	size_t len;
};

struct mpfs_signature_priv *signature_priv;

static ssize_t mpfs_signature_write(struct file *filep, const char __user *userbuf,
				    size_t len, loff_t *offset)
{
	u16 len_trun = len > MPFS_SIG_CMD_SIZE_BYTES ? MPFS_SIG_CMD_SIZE_BYTES : len;
	u8 *cmd_data = signature_priv->buffer;

	cmd_data = kmalloc(MPFS_SIG_CMD_SIZE_BYTES, GFP_KERNEL);
	if (!cmd_data)
		return -ENOMEM;

	int ret = copy_from_user(cmd_data, userbuf, len_trun); //TODO ret val check
	if (ret)
		return -ret;
	signature_priv->buffer = cmd_data;
	signature_priv->len = len_trun;

	return len;
}

static ssize_t mpfs_signature_read(struct file *filp, char __user *userbuf,
				     size_t len, loff_t *f_pos)
{
	u8 response_msg[MPFS_SIG_RESP_SIZE_BYTES];
	u8 buffer[2 * MPFS_SIG_RESP_SIZE_BYTES + 3];
	u8 *cmd_data = signature_priv->buffer;
	u8 *bufferp = buffer;
	u32 i;

	struct mpfs_mss_response response = {
		.resp_status = 0U,
		.resp_msg = (u32 *)response_msg,
		.resp_size = MPFS_SIG_RESP_SIZE_BYTES
	};
	struct mpfs_mss_msg msg = { .cmd_opcode = CMD_OPCODE,
				    .cmd_data_size = MPFS_SIG_CMD_SIZE_BYTES,
				    .response = &response,
				    .cmd_data = cmd_data,
				    .mbox_offset = MBOX_OFFSET,
				    .resp_offset = RESP_OFFSET };

	if (!signature_priv->buffer || signature_priv->len != MPFS_SIG_CMD_SIZE_BYTES)
		return -EFAULT;

	int ret = mpfs_blocking_transaction(signature_priv->sys_controller, &msg);
	kfree(signature_priv->buffer);

	if (ret)
		return -EIO;
	bufferp += sprintf(bufferp, "%02x ", response.resp_status);
	for (i = 0; i < MPFS_SIG_RESP_SIZE_BYTES; i++)
		bufferp += sprintf(bufferp, "%02x", response_msg[i]);

	return simple_read_from_buffer(userbuf, len, f_pos, buffer,
				       2 * MPFS_SIG_RESP_SIZE_BYTES + 3);
}

static int mpfs_signature_open(struct inode *inode, struct file *filp)
{
	if (!mutex_trylock(&mpfs_signature_mutex)) {
		pr_debug("Device Busy\n");
		return -EBUSY;
	}
	return 0;
}

static int mpfs_signature_release(struct inode *inode, struct file *filp)
{
	mutex_unlock(&mpfs_signature_mutex);
	return 0;
}

static const struct file_operations mpfs_signature_fops = {
	.owner = THIS_MODULE,
	.read = mpfs_signature_read,
	.open = mpfs_signature_open,
	.write = mpfs_signature_write,
	.release = mpfs_signature_release
};

static struct miscdevice mpfs_signature_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mpfs_signature",
	.fops = &mpfs_signature_fops
};

static int mpfs_signature_probe(struct platform_device *pdev)
{
	struct device_node *sys_controller_np;
	struct device *dev = &pdev->dev;

	signature_priv =
		devm_kzalloc(dev, sizeof(*signature_priv), GFP_KERNEL);
	if (!signature_priv)
		return -ENOMEM;
	
	sys_controller_np =
		of_parse_phandle(pdev->dev.of_node, "syscontroller", 0);
	if (!sys_controller_np) {
		dev_err(&pdev->dev,
			"Failed to find mpfs system controller node\n");
		return -ENODEV;
	}

	signature_priv->sys_controller =
		mpfs_sys_controller_get(sys_controller_np);
	of_node_put(sys_controller_np);
	if (!signature_priv->sys_controller)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, signature_priv);
	misc_register(&mpfs_signature_dev);
	dev_info(&pdev->dev,
		 "Successfully registered mpfs signature driver\n");

	return 0;
}

static const struct of_device_id mpfs_signature_of_match[] = {
	{
		.compatible = "microchip,polarfire-soc-signature",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_signature_of_match);

static struct platform_driver mpfs_signature_driver = {
	.driver = {
	.name = "mpfs-signature",
	.of_match_table = mpfs_signature_of_match,
	},
	.probe = mpfs_signature_probe,
};
module_platform_driver(mpfs_signature_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("PFSoC signature driver");
