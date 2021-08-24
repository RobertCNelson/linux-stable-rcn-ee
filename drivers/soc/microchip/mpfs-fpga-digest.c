// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip PFSoC fpga digest driver
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

#define MPFS_DIGEST_CHECK_FABRIC_OFFSET        (0U)
#define MPFS_DIGEST_CHECK_CC_OFFSET            (1U)
#define MPFS_DIGEST_CHECK_SNVM_OFFSET          (2U)
#define MPFS_DIGEST_CHECK_UL_OFFSET            (3U)
#define MPFS_DIGEST_CHECK_UKDIGEST0_OFFSET     (4U)
#define MPFS_DIGEST_CHECK_UKDIGEST1_OFFSET     (5U)
#define MPFS_DIGEST_CHECK_UKDIGEST2_OFFSET     (6U)
#define MPFS_DIGEST_CHECK_UKDIGEST3_OFFSET     (7U)
#define MPFS_DIGEST_CHECK_UKDIGEST4_OFFSET     (8U)
#define MPFS_DIGEST_CHECK_UKDIGEST5_OFFSET     (9U)
#define MPFS_DIGEST_CHECK_UKDIGEST6_OFFSET     (10U)
#define MPFS_DIGEST_CHECK_UPERM_OFFSET         (11U)
#define MPFS_DIGEST_CHECK_SYS_OFFSET           (12U)
#define MPFS_DIGEST_CHECK_UKDIGEST7_OFFSET     (13U)
#define MPFS_DIGEST_CHECK_ENVM_OFFSET          (14U)
#define MPFS_DIGEST_CHECK_UKDIGEST8_OFFSET     (15U)
#define MPFS_DIGEST_CHECK_UKDIGEST9_OFFSET     (16U)
#define MPFS_DIGEST_CHECK_UKDIGEST10_OFFSET (17U)
#define MPFS_DIGEST_RESP_SIZE_BYTES 576U
#define MPFS_DIGEST_SECTION_RESP_SIZE_BYTES 32U

#define CMD_OPCODE 0x04U
#define CMD_DATA_SIZE 0U
#define CMD_DATA NULL
#define MBOX_OFFSET 0U
#define RESP_OFSET 0U
#define CMD_DATA_SIZE 0U

static DEFINE_MUTEX(mpfs_digest_mutex);

struct mpfs_digest_priv {
	struct mpfs_sys_controller *sys_controller;
};

struct mpfs_digest_priv *digest_priv;

static ssize_t mpfs_digest_read(struct file *filp, char __user *userbuf,
				 size_t len, loff_t *f_pos)
{
	u8 response_msg[MPFS_DIGEST_RESP_SIZE_BYTES];
	u16 buffer_length = 2 * MPFS_DIGEST_RESP_SIZE_BYTES + MPFS_DIGEST_RESP_SIZE_BYTES/32;
	u8 buffer[buffer_length];
	u8 *bufferp = buffer;
	u32 i;

	struct mpfs_mss_response response = {
		.resp_status = 0U,
		.resp_msg = (u32 *)response_msg,
		.resp_size = MPFS_DIGEST_RESP_SIZE_BYTES
	};
	struct mpfs_mss_msg msg = {
		.cmd_opcode = CMD_OPCODE,
		.cmd_data_size = CMD_DATA_SIZE,
		.response = &response,
		.cmd_data = CMD_DATA,
		.mbox_offset = MBOX_OFFSET,
		.resp_offset = RESP_OFSET
		};

	int ret = mpfs_blocking_transaction(digest_priv->sys_controller, &msg);
	if (ret)
		return -EIO;

	for (i = 0; i < MPFS_DIGEST_RESP_SIZE_BYTES; i++) {
		if (i % 32 == 0 && i != 0)
			bufferp += sprintf(bufferp, "\r\n");

		bufferp += sprintf(bufferp, "%02x", response_msg[i]);
	}

	return simple_read_from_buffer(userbuf, len, f_pos, buffer, buffer_length);
}

static int mpfs_digest_open(struct inode *inode, struct file *filp)
{
	if (!mutex_trylock(&mpfs_digest_mutex)) {
		pr_debug("Device Busy\n");
		return -EBUSY;
	}
	return 0;
}

static int mpfs_digest_release(struct inode *inode, struct file *filp)
{
	mutex_unlock(&mpfs_digest_mutex);
	return 0;
}

static const struct file_operations mpfs_digest_fops = {
	.owner		= THIS_MODULE,
	.read		= mpfs_digest_read,
	.open		= mpfs_digest_open,
	.release	= mpfs_digest_release
};

static struct miscdevice mpfs_digest_dev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "mpfs_fpga_digest",
	.fops		= &mpfs_digest_fops
};

static int mpfs_digest_probe(struct platform_device *pdev)
{
	struct device_node *sys_controller_np;
	struct device *dev = &pdev->dev;

	digest_priv = devm_kzalloc(dev, sizeof(*digest_priv), GFP_KERNEL);
	if (!digest_priv)
		return -ENOMEM;

	sys_controller_np =
		of_parse_phandle(pdev->dev.of_node, "syscontroller", 0);
	if (!sys_controller_np) {
		dev_err(&pdev->dev, "Failed to find mpfs system controller node\n");
		return -ENODEV;
	}

	digest_priv->sys_controller = mpfs_sys_controller_get(sys_controller_np);
	of_node_put(sys_controller_np);
	if (!digest_priv->sys_controller)
		return -EPROBE_DEFER;

	platform_set_drvdata(pdev, digest_priv);
	misc_register(&mpfs_digest_dev);
	dev_info(&pdev->dev, "Successfully registered mpfs fpga digest driver\n");

	return 0;
}

static const struct of_device_id mpfs_digest_of_match[] = {
	{
		.compatible = "microchip,polarfire-soc-digest",
	},
	{
		.compatible = "microchip,mpfs-digest",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_digest_of_match);

static struct platform_driver mpfs_digest_driver = {
	.driver = {
		.name = "mpfs-digest",
		.of_match_table = mpfs_digest_of_match,
	},
	.probe = mpfs_digest_probe,
};
module_platform_driver(mpfs_digest_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("PFSoC mailbox client driver");
