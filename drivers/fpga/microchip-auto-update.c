// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip Polarfire SoC "Auto Update" FPGA reprogramming.
 *
 * Documentation of this functionality is available in the "PolarFire® FPGA and
 * PolarFire SoC FPGA Programming" User Guide.
 *
 * Copyright (c) 2022-2023 Microchip Corporation. All rights reserved.
 *
 * Author: Conor Dooley <conor.dooley@microchip.com>
 */
#include <linux/debugfs.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/of_device.h>
#include <linux/sizes.h>

#include <soc/microchip/mpfs.h>

#define AUTO_UPDATE_DEFAULT_MBOX_OFFSET		0u
#define AUTO_UPDATE_DEFAULT_RESP_OFFSET		0u

#define AUTO_UPDATE_FEATURE_CMD_OPCODE		0x05u
#define AUTO_UPDATE_FEATURE_CMD_DATA_SIZE	0u
#define AUTO_UPDATE_FEATURE_RESP_SIZE		33u
#define AUTO_UPDATE_FEATURE_CMD_DATA		NULL
#define AUTO_UPDATE_FEATURE_ENABLED		BIT(5)

#define AUTO_UPDATE_AUTHENTICATE_CMD_OPCODE	0x22u
#define AUTO_UPDATE_AUTHENTICATE_CMD_DATA_SIZE	0u
#define AUTO_UPDATE_AUTHENTICATE_RESP_SIZE	1u
#define AUTO_UPDATE_AUTHENTICATE_CMD_DATA	NULL

#define AUTO_UPDATE_PROGRAM_CMD_OPCODE		0x46u
#define AUTO_UPDATE_PROGRAM_CMD_DATA_SIZE	0u
#define AUTO_UPDATE_PROGRAM_RESP_SIZE		1u
#define AUTO_UPDATE_PROGRAM_CMD_DATA		NULL

/*
 * SPI Flash layout example:
 * |------------------------------| 0x0000000
 * | 1 KiB                        |
 * | SPI "directories"            |
 * |------------------------------| 0x0000400
 * | 1 MiB                        |
 * | Reserved area                |
 * | Used for bitstream info      |
 * |------------------------------| 0x0100400
 * | 20 MiB                       |
 * | Golden Image                 |
 * |------------------------------| 0x1500400
 * | 20 MiB                       |
 * | Auto Update Image            |
 * |------------------------------| 0x2900400
 * | 20 MiB                       |
 * | Reserved for multi-image IAP |
 * | Unused for Auto Update       |
 * |------------------------------| 0x3D00400
 * | ? B                          |
 * | Unused                       |
 * |------------------------------| 0x?
 */
#define AUTO_UPDATE_DIRECTORY_BASE	0u
#define AUTO_UPDATE_DIRECTORY_WIDTH	4u
#define AUTO_UPDATE_GOLDEN_INDEX	0u
#define AUTO_UPDATE_UPGRADE_INDEX	1u
#define AUTO_UPDATE_BLANK_INDEX		2u
#define AUTO_UPDATE_GOLDEN_DIRECTORY	(AUTO_UPDATE_DIRECTORY_WIDTH * AUTO_UPDATE_GOLDEN_INDEX)
#define AUTO_UPDATE_UPGRADE_DIRECTORY	(AUTO_UPDATE_DIRECTORY_WIDTH * AUTO_UPDATE_UPGRADE_INDEX)
#define AUTO_UPDATE_BLANK_DIRECTORY	(AUTO_UPDATE_DIRECTORY_WIDTH * AUTO_UPDATE_BLANK_INDEX)
#define AUTO_UPDATE_DIRECTORY_SIZE	SZ_1K
#define AUTO_UPDATE_RESERVED_SIZE	SZ_1M
#define AUTO_UPDATE_BITSTREAM_BASE	(AUTO_UPDATE_DIRECTORY_SIZE + AUTO_UPDATE_RESERVED_SIZE)

struct mpfs_auto_update_config {
	u8 feature_response_size;
};

struct mpfs_auto_update_priv {
	struct mpfs_sys_controller *sys_controller;
	struct device *dev;
	struct fpga_region *region;
	struct mpfs_auto_update_config *config;
	struct mtd_info *flash;
	struct dentry *debugfs_dir;
};

static struct device *mpfs_auto_update_debug_dev;

static enum fpga_mgr_states mpfs_auto_update_state(struct fpga_manager *mgr)
{
	struct mpfs_auto_update_priv *priv = mgr->priv;
	struct mpfs_mss_response *response;
	struct mpfs_mss_msg *message;
	u32 *response_msg;
	int ret;
	enum fpga_mgr_states rc = FPGA_MGR_STATE_WRITE_INIT_ERR;

	response_msg = devm_kzalloc(priv->dev, AUTO_UPDATE_FEATURE_RESP_SIZE * sizeof(response_msg),
				    GFP_KERNEL);
	if (!response_msg)
		return FPGA_MGR_STATE_WRITE_INIT_ERR;

	response = devm_kzalloc(priv->dev, sizeof(struct mpfs_mss_response), GFP_KERNEL);
	if (!response) {
		rc = FPGA_MGR_STATE_WRITE_INIT_ERR;
		goto free_response_msg;
	}

	message = devm_kzalloc(priv->dev, sizeof(struct mpfs_mss_msg), GFP_KERNEL);
	if (!message) {
		rc = FPGA_MGR_STATE_WRITE_INIT_ERR;
		goto free_response;
	}

	/*
	 * To verify that Auto Update is possible, the "Query Security Service
	 * Request" is performed. Bit 5 of byte 1 is "UL_Auto Update" & if it is
	 * set, Auto Update is not possible.
	 * This service has no command data & does not overload mbox_offset.
	 * The size of the response varies between PolarFire & PolarFire SoC.
	 */
	response->resp_msg = response_msg;
	response->resp_size = AUTO_UPDATE_FEATURE_RESP_SIZE;
	message->cmd_opcode = AUTO_UPDATE_FEATURE_CMD_OPCODE;
	message->cmd_data_size = AUTO_UPDATE_FEATURE_CMD_DATA_SIZE;
	message->response = response;
	message->cmd_data = AUTO_UPDATE_FEATURE_CMD_DATA;
	message->mbox_offset = AUTO_UPDATE_DEFAULT_MBOX_OFFSET;
	message->resp_offset = AUTO_UPDATE_DEFAULT_RESP_OFFSET;

	ret = mpfs_blocking_transaction(priv->sys_controller, message);
	if (ret | response->resp_status) {
		rc = FPGA_MGR_STATE_UNKNOWN;
		goto free_message;
	}

	if (!(response_msg[1] & AUTO_UPDATE_FEATURE_ENABLED))
		rc = FPGA_MGR_STATE_OPERATING;

free_message:
	devm_kfree(priv->dev, message);
free_response:
	devm_kfree(priv->dev, response);
free_response_msg:
	devm_kfree(priv->dev, response_msg);

	return rc;
}

static int mpfs_auto_update_write_init(struct fpga_manager *mgr, struct fpga_image_info *info,
				       const char *buf, size_t count)
{
	/*
	 * Verifying the Golden Image is idealistic. It will be evaluated
	 * against the currently programmed image and thus may fail - due to
	 * either rollback protection (if its an older version than that in use)
	 * or if the version is the same as that of the in-use image.
	 * Extracting the information as to why a failure occurred is not
	 * currently possible due to limitations of the system controller
	 * driver. If those are fixed, verification of the Golden Image should
	 * be added here.
	 */
	return 0;
}

static int mpfs_auto_update_write(struct fpga_manager *mgr, const char *buf, size_t count)
{
	/*
	 * No parsing etc of the bitstream is required. The system controller
	 * will do all of that itself - including verifying that the bitstream
	 * is valid.
	 */
	struct mpfs_auto_update_priv *priv = mgr->priv;
	struct erase_info erase;
	char *buffer;
	loff_t directory_address = AUTO_UPDATE_UPGRADE_DIRECTORY;
	size_t bytes_written = 0, bytes_read = 0;
	size_t erase_size = AUTO_UPDATE_DIRECTORY_SIZE;
	size_t size_per_bitstream = 0;
	u32 image_address;
	int ret;

	priv->flash = mpfs_sys_controller_get_flash(priv->sys_controller);
	if (!priv->flash)
		return -EIO;

	erase_size = round_up(erase_size, (u64)priv->flash->erasesize);

	/*
	 * We need to calculate if we have enough space in the flash for the
	 * new image.
	 * First, chop off the first 1 KiB as it's reserved for the directory.
	 * The 1 MiB reserved for design info needs to be ignored also.
	 * All that remains is carved into 3 & rounded down to the erasesize.
	 * If this is smaller than the image size, we abort.
	 * There's also no need to consume more than 20 MiB per image.
	 */
	size_per_bitstream = priv->flash->size - SZ_1K - SZ_1M;
	size_per_bitstream = round_down(size_per_bitstream / 3, erase_size);
	if (size_per_bitstream > 20 * SZ_1M)
		size_per_bitstream = 20 * SZ_1M;

	if (size_per_bitstream < count) {
		dev_err(priv->dev,
			"flash device has insufficient capacity to store this bitstream\n");
		return -EINVAL;
	}

	image_address = AUTO_UPDATE_BITSTREAM_BASE + AUTO_UPDATE_UPGRADE_INDEX * size_per_bitstream;

	buffer = devm_kzalloc(priv->dev, erase_size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	erase.addr = AUTO_UPDATE_DIRECTORY_BASE;
	erase.len = erase_size;

	/*
	 * We need to write the "SPI DIRECTORY" to the first 1 KiB, telling
	 * the system controller where to find the actual bitstream. Since
	 * this is spi-nor, we have to read the first eraseblock, erase that
	 * portion of the flash, modify the data and then write it back.
	 */
	ret = mtd_read(priv->flash, AUTO_UPDATE_DIRECTORY_BASE, erase_size, &bytes_read,
		       (u_char *)buffer);
	if (ret)
		goto out;

	if (bytes_read != erase_size) {
		ret = -EIO;
		goto out;
	}

	if (*(u32 *)buffer == GENMASK(31, 0)) {
		dev_err(priv->dev, "No Golden Image programmed, aborting\n");
		ret = -EINVAL;
		goto out;
	}

	ret = mtd_erase(priv->flash, &erase);
	if (ret)
		goto out;

	/*
	 * Populate the image address and then zero out the next directory so
	 * that the system controller doesn't complain if in "Single Image"
	 * mode.
	 */
	memcpy(buffer + AUTO_UPDATE_UPGRADE_DIRECTORY, &image_address, AUTO_UPDATE_DIRECTORY_WIDTH);
	memset(buffer + AUTO_UPDATE_BLANK_DIRECTORY, 0x0, AUTO_UPDATE_DIRECTORY_WIDTH);

	dev_info(priv->dev, "Writing the image address (%x) to the flash directory (%llx)\n",
		 image_address, directory_address);

	ret = mtd_write(priv->flash, 0x0, erase_size, &bytes_written, (u_char *)buffer);
	if (ret)
		goto out;

	if (bytes_written != erase_size) {
		ret = -EIO;
		goto out;
	}

	/*
	 * Now the .spi image itself can be written to the flash. Preservation
	 * of contents here is not important here, unlike the spi "directory"
	 * which must be RMWed.
	 */
	erase.len = round_up(count, (size_t)priv->flash->erasesize);
	erase.addr = image_address;

	dev_info(priv->dev, "Erasing the flash at address (%x)\n", image_address);
	ret = mtd_erase(priv->flash, &erase);
	if (ret)
		goto out;

	dev_info(priv->dev, "Writing the image to the flash at address (%x)\n", image_address);
	ret = mtd_write(priv->flash, (loff_t)image_address, count, &bytes_written, buf);
	if (ret)
		goto out;

	if (bytes_written != count)
		ret = -EIO;

out:
	devm_kfree(priv->dev, buffer);
	return ret;
}

static int mpfs_auto_update_write_complete(struct fpga_manager *mgr, struct fpga_image_info *info)
{
	struct mpfs_auto_update_priv *priv = mgr->priv;
	struct mpfs_mss_response *response;
	struct mpfs_mss_msg *message;
	u32 *response_msg;
	int ret;

	response_msg = devm_kzalloc(priv->dev, AUTO_UPDATE_FEATURE_RESP_SIZE * sizeof(response_msg),
				    GFP_KERNEL);
	if (!response_msg)
		return -ENOMEM;

	response = devm_kzalloc(priv->dev, sizeof(struct mpfs_mss_response), GFP_KERNEL);
	if (!response) {
		ret = -ENOMEM;
		goto free_response_msg;
	}

	message = devm_kzalloc(priv->dev, sizeof(struct mpfs_mss_msg), GFP_KERNEL);
	if (!message) {
		ret = -ENOMEM;
		goto free_response;
	}

	/*
	 * The system controller can verify that an image in the flash is valid.
	 * Rather than duplicate the check in this driver, call the relevant
	 * service from the system controller instead.
	 * This service has no command data and no response data. It overloads
	 * mbox_offset with the image index in the flash's SPI directory where
	 * the bitstream is located.
	 */
	response->resp_msg = response_msg;
	response->resp_size = AUTO_UPDATE_AUTHENTICATE_RESP_SIZE;
	message->cmd_opcode = AUTO_UPDATE_AUTHENTICATE_CMD_OPCODE;
	message->cmd_data_size = AUTO_UPDATE_AUTHENTICATE_CMD_DATA_SIZE;
	message->response = response;
	message->cmd_data = AUTO_UPDATE_AUTHENTICATE_CMD_DATA;
	message->mbox_offset = AUTO_UPDATE_UPGRADE_INDEX;
	message->resp_offset = AUTO_UPDATE_DEFAULT_RESP_OFFSET;

	dev_info(priv->dev, "Running verification of Update Image\n");
	ret = mpfs_blocking_transaction(priv->sys_controller, message);
	if (ret | response->resp_status) {
		dev_warn(priv->dev, "Verification of Update Image failed!\n");
		ret = ret ? ret : -EBADMSG;
	} else {
		dev_info(priv->dev, "Verification of Update Image passed!\n");
	}

//	/*
//	 * If the validation has passed, initiate Auto Update.
//	 * This service has no command data and no response data. It overloads
//	 * mbox_offset with the image index in the flash's SPI directory where
//	 * the bitstream is located.
//	 * Once we attempt Auto Update either:
//	 * - it passes and the board reboots
//	 * - it fails and the board reboots to recover
//	 * - the system controller aborts and we exit "gracefully".
//	 *   "gracefully" since there is no interrupt produced & it just times
//	 *   out.
//	 */
//	response->resp_msg = response_msg;
//	response->resp_size = AUTO_UPDATE_PROGRAM_RESP_SIZE;
//	message->cmd_opcode = AUTO_UPDATE_PROGRAM_CMD_OPCODE;
//	message->cmd_data_size = AUTO_UPDATE_PROGRAM_CMD_DATA_SIZE;
//	message->response = response;
//	message->cmd_data = AUTO_UPDATE_PROGRAM_CMD_DATA;
//	message->mbox_offset = 0; //field is ignored
//	message->resp_offset = AUTO_UPDATE_DEFAULT_RESP_OFFSET;
//
//	dev_info(priv->dev, "Running Auto Update command\n");
//	ret = mpfs_blocking_transaction(priv->sys_controller, message);
//	if (ret && ret != -ETIMEDOUT)
//		goto out;
//
//	/* *remove this for auto update*
//	 * This return 0 is dead code. Either the Auto Update will fail, or it will pass
//	 * & the FPGA will be rebooted in which case mpfs_blocking_transaction()
//	 * will never return and Linux will die.
//	 */
//	return 0;

	devm_kfree(priv->dev, message);
free_response:
	devm_kfree(priv->dev, response);
free_response_msg:
	devm_kfree(priv->dev, response_msg);

	return ret;
}

static const struct fpga_manager_ops mpfs_auto_update_ops = {
	.state = mpfs_auto_update_state,
	.write_init = mpfs_auto_update_write_init,
	.write = mpfs_auto_update_write,
	.write_complete = mpfs_auto_update_write_complete,
};

static int mpfs_auto_update_run(struct device *dev)
{
	struct fpga_manager *mgr;
	struct fpga_image_info *info;
	int ret;

	mgr = fpga_mgr_get(dev);
	info = fpga_image_info_alloc(dev);

	info->firmware_name = devm_kstrdup(dev, "mpfs_bitstream.spi", GFP_KERNEL);

	ret = fpga_mgr_lock(mgr);
	if (ret)
		goto free_info;

	ret = fpga_mgr_load(mgr, info);
	if (ret) {
		dev_err(dev, "Failed to write the bitstream\n");
		goto unlock_mgr;
	}

unlock_mgr:
	fpga_mgr_unlock(mgr);
free_info:
	fpga_image_info_free(info);
	fpga_mgr_put(mgr);

	return ret;
}

static ssize_t mpfs_auto_update_exec(struct file *file, const char __user *data, size_t count,
				     loff_t *ppos)
{
	int ret;

	ret = mpfs_auto_update_run(mpfs_auto_update_debug_dev);
	if (ret)
		dev_err_probe(mpfs_auto_update_debug_dev, ret, "Auto Update failed");

	return count;
}

static const struct file_operations mpfs_auto_update_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mpfs_auto_update_exec
};

static int mpfs_auto_update_debugfs_setup(struct mpfs_auto_update_priv *priv)
{
	priv->debugfs_dir = debugfs_create_dir("fpga", NULL);

	if(IS_ERR(priv->debugfs_dir))
		return PTR_ERR(priv->debugfs_dir);

	debugfs_create_file("microchip_exec_update", 0200, priv->debugfs_dir, NULL,
			    &mpfs_auto_update_fops);

	return 0;
}

static int mpfs_auto_update_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mpfs_auto_update_priv *priv;
	struct fpga_manager *mgr;
	enum fpga_mgr_states state;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->sys_controller = mpfs_sys_controller_get(dev);
	if (IS_ERR(priv->sys_controller))
		return dev_err_probe(dev, PTR_ERR(priv->sys_controller),
				     "Could not register as a sub device of the system controller\n");

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	mgr = devm_fpga_mgr_register(dev, "Microchip MPFS Auto Update FPGA Manager",
				     &mpfs_auto_update_ops, priv);
	if (IS_ERR(mgr))
		return dev_err_probe(dev, PTR_ERR(mgr), "Could not register FPGA manager.\n");

	state = mpfs_auto_update_state(mgr);
	if (state != FPGA_MGR_STATE_OPERATING)
		return -EIO;

	ret = mpfs_auto_update_debugfs_setup(priv);
	if (ret && ret != -ENODEV)
		return ret;

	mpfs_auto_update_debug_dev = priv->dev;

	return 0;
}

static struct platform_driver mpfs_auto_update_driver = {
	.driver = {
		.name = "mpfs-auto-update",
	},
	.probe = mpfs_auto_update_probe,
};
module_platform_driver(mpfs_auto_update_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("PolarFire SoC Auto Update FPGA reprogramming");
