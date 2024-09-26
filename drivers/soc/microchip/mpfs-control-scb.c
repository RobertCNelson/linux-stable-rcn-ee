// SPDX-License-Identifier: GPL-2.0

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static const struct mfd_cell mpfs_control_scb_devs[] = {
	{ .name = "mpfs-tvs", },
};

static int mpfs_control_scb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = mfd_add_devices(dev, PLATFORM_DEVID_NONE, mpfs_control_scb_devs,
				   1, NULL, 0, NULL);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id mpfs_control_scb_of_match[] = {
	{.compatible = "microchip,mpfs-control-scb", },
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_control_scb_of_match);

static struct platform_driver mpfs_control_scb_driver = {
	.driver = {
		.name = "mpfs-control-scb",
		.of_match_table = mpfs_control_scb_of_match,
	},
	.probe = mpfs_control_scb_probe,
};
module_platform_driver(mpfs_control_scb_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("PolarFire SoC control scb driver");
