// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#include <drm/drm_accel.h>
#include <drm/drm_drv.h>
#include <drm/drm_gem.h>
#include <drm/drm_ioctl.h>
#include <uapi/drm/thames_accel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg.h>

#include "thames_drv.h"
#include "thames_core.h"
#include "thames_gem.h"
#include "thames_ipc.h"

static struct platform_device *drm_dev;
static struct thames_device *tdev;

static int thames_open(struct drm_device *dev, struct drm_file *file)
{
	struct thames_device *tdev = to_thames_device(dev);
	struct thames_file_priv *thames_priv;
	int ret;

	if (!try_module_get(THIS_MODULE))
		return -EINVAL;

	thames_priv = kzalloc(sizeof(*thames_priv), GFP_KERNEL);
	if (!thames_priv) {
		ret = -ENOMEM;
		goto err_put_mod;
	}

	thames_priv->tdev = tdev;

	file->driver_priv = thames_priv;

	return 0;

err_put_mod:
	module_put(THIS_MODULE);
	return ret;
}

static void thames_postclose(struct drm_device *dev, struct drm_file *file)
{
	struct thames_file_priv *thames_priv = file->driver_priv;

	kfree(thames_priv);
	module_put(THIS_MODULE);
}

static const struct drm_ioctl_desc thames_drm_driver_ioctls[] = {
#define THAMES_IOCTL(n, func) DRM_IOCTL_DEF_DRV(THAMES_##n, thames_ioctl_##func, 0)
	THAMES_IOCTL(BO_CREATE, bo_create),
	THAMES_IOCTL(BO_MMAP_OFFSET, bo_mmap_offset),
};

DEFINE_DRM_ACCEL_FOPS(thames_accel_driver_fops);

static const struct drm_driver thames_drm_driver = {
	.driver_features = DRIVER_COMPUTE_ACCEL | DRIVER_GEM,
	.open = thames_open,
	.postclose = thames_postclose,
	.gem_create_object = thames_gem_create_object,
	.ioctls = thames_drm_driver_ioctls,
	.num_ioctls = ARRAY_SIZE(thames_drm_driver_ioctls),
	.fops = &thames_accel_driver_fops,
	.name = "thames",
	.desc = "thames DRM",
};

static int thames_probe(struct rpmsg_device *rpdev)
{
	u64 iova_start, iova_size;
	unsigned int core;
	int err;

	if (!tdev) {
		err = thames_core_get_iova_range(rpdev, &iova_start, &iova_size);
		if (err)
			return err;

		tdev = thames_device_init(drm_dev, &thames_drm_driver, iova_start, iova_size);
		if (IS_ERR(tdev)) {
			dev_err(&rpdev->dev, "failed to initialize thames device\n");
			return PTR_ERR(tdev);
		}
	}

	core = tdev->num_cores;

	tdev->cores[core].tdev = tdev;
	tdev->cores[core].rpdev = rpdev;
	tdev->cores[core].dev = &rpdev->dev;
	tdev->cores[core].index = core;

	tdev->num_cores++;

	return thames_core_init(&tdev->cores[core]);
}

static void thames_remove(struct rpmsg_device *rpdev)
{
	unsigned int core;

	for (core = 0; core < tdev->num_cores; core++) {
		if (tdev->cores[core].rpdev == rpdev) {
			thames_core_fini(&tdev->cores[core]);
			tdev->num_cores--;
			break;
		}
	}

	if (!tdev->num_cores) {
		thames_device_fini(tdev);
		tdev = NULL;
	}
}

static const struct rpmsg_device_id thames_rpmsg_id_table[] = { { .name = THAMES_SERVICE_NAME },
								{} };

static struct rpmsg_driver thames_rpmsg_driver = {
	.drv = {
		.name = "thames",
		.owner = THIS_MODULE,
	},
	.id_table = thames_rpmsg_id_table,
	.probe = thames_probe,
	.remove = thames_remove,
};

static int __init thames_register(void)
{
	drm_dev = platform_device_register_simple("thames", -1, NULL, 0);
	if (IS_ERR(drm_dev))
		return PTR_ERR(drm_dev);

	return register_rpmsg_driver(&thames_rpmsg_driver);
}

static void __exit thames_unregister(void)
{
	unregister_rpmsg_driver(&thames_rpmsg_driver);

	platform_device_unregister(drm_dev);
}

module_init(thames_register);
module_exit(thames_unregister);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DRM driver for Texas Instrument's C7x accelerator cores");
MODULE_AUTHOR("Tomeu Vizoso");
MODULE_ALIAS("rpmsg:" THAMES_SERVICE_NAME);
