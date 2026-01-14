// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#include "linux/remoteproc.h"
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/rpmsg.h>

#include "thames_core.h"
#include "thames_device.h"
#include "thames_rpmsg.h"

/* Shift to convert bytes to megabytes (divide by 1048576) */
#define THAMES_BYTES_TO_MB_SHIFT 20

int thames_core_get_iova_range(struct rpmsg_device *rpdev, u64 *iova_start, u64 *iova_size)
{
	struct rproc *rproc;
	struct device_node *of_node;
	struct device_node *mem_node;
	struct resource mem_res;
	int err;

	if (!iova_start || !iova_size)
		return -EINVAL;

	rproc = rproc_get_by_child(&rpdev->dev);
	if (!rproc) {
		dev_err(&rpdev->dev, "Failed to get rproc device\n");
		return -ENODEV;
	}

	of_node = rproc->dev.parent->of_node;
	put_device(&rproc->dev);

	if (!of_node) {
		dev_err(&rpdev->dev, "No device tree node found on rproc parent\n");
		return -ENODEV;
	}

	/*
	 * Read the IOVA pool range from the device tree node.
	 * The third memory-region (index 2) defines the virtual address range.
	 * The first two regions are typically:
	 *   [0] = DMA memory region for remoteproc (physically contiguous)
	 *   [1] = Code/data memory region for remoteproc (physically contiguous)
	 *   [2] = Virtual address pool for BO mappings (firmware-managed MMU)
	 */
	mem_node = of_parse_phandle(of_node, "memory-region", 2);
	if (!mem_node) {
		dev_err(&rpdev->dev, "Missing third memory-region (DSP VA pool) in device tree\n");
		return -EINVAL;
	}

	err = of_address_to_resource(mem_node, 0, &mem_res);
	of_node_put(mem_node);
	if (err) {
		dev_err(&rpdev->dev, "Failed to get DSP VA pool range from memory-region[2]: %d\n",
			err);
		return err;
	}

	*iova_start = mem_res.start;
	*iova_size = resource_size(&mem_res);

	if (!*iova_size) {
		dev_err(&rpdev->dev, "Invalid DSP VA pool size: 0\n");
		return -EINVAL;
	}

	return 0;
}

static int thames_core_validate_iova_range(struct thames_core *core)
{
	struct thames_device *tdev = core->tdev;
	u64 iova_start, iova_size;
	int err;

	err = thames_core_get_iova_range(core->rpdev, &iova_start, &iova_size);
	if (err)
		return err;

	if (iova_start != tdev->iova_start || iova_size != tdev->iova_size) {
		dev_err(core->dev,
			"Core %d IOVA range mismatch! Expected 0x%llx-0x%llx, got 0x%llx-0x%llx\n",
			core->index, tdev->iova_start, tdev->iova_start + tdev->iova_size - 1,
			iova_start, iova_start + iova_size - 1);
		dev_err(core->dev,
			"All cores must have the same memory-region[2] (IOVA pool) in device tree\n");
		return -EINVAL;
	}

	return 0;
}

int thames_core_init(struct thames_core *core)
{
	int err = 0;

	err = thames_core_validate_iova_range(core);
	if (err)
		return err;

	err = thames_rpmsg_init(core);
	if (err)
		return err;

	err = thames_rpmsg_ping_test(core);
	if (err)
		return err;

	return 0;
}

void thames_core_fini(struct thames_core *core)
{
	thames_rpmsg_fini(core);
}

void thames_core_reset(struct thames_core *core)
{
	struct rpmsg_device *rpdev = core->rpdev;
	struct rproc *rproc;
	int ret;

	dev_warn(core->dev, "Resetting DSP core %d", core->index);

	if (!atomic_read(&core->reset.pending))
		dev_warn(core->dev, "Reset called without reset.pending set\n");

	rproc = rproc_get_by_child(&rpdev->dev);
	if (!rproc) {
		dev_err(core->dev, "Failed to get rproc for reset\n");
		return;
	}

	ret = rproc_shutdown(rproc);
	if (ret) {
		dev_err(&rproc->dev, "Failed to shut down DSP: %d\n", ret);
		goto put_rproc;
	}

	ret = rproc_boot(rproc);
	if (ret)
		dev_err(&rproc->dev, "Failed to boot DSP: %d\n", ret);

put_rproc:
	put_device(&rproc->dev);
}
