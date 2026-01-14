// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#include <drm/drm_drv.h>
#include <linux/array_size.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>
#include <linux/platform_device.h>

#include "thames_device.h"

/* Shift to convert bytes to megabytes (divide by 1048576) */
#define THAMES_BYTES_TO_MB_SHIFT 20

struct thames_device *thames_device_init(struct platform_device *pdev,

					 const struct drm_driver *thames_drm_driver, u64 iova_start,
					 u64 iova_size)
{
	struct device *dev = &pdev->dev;
	struct thames_device *tdev;
	struct drm_device *ddev;
	int err;

	tdev = devm_drm_dev_alloc(dev, thames_drm_driver, struct thames_device, ddev);
	if (IS_ERR(tdev))
		return tdev;

	tdev->num_cores = 0;
	ddev = &tdev->ddev;
	dev_set_drvdata(dev, tdev);

	dma_set_max_seg_size(dev, UINT_MAX);

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (err)
		return ERR_PTR(err);

	err = devm_mutex_init(dev, &tdev->sched_lock);
	if (err)
		return ERR_PTR(-ENOMEM);

	ida_init(&tdev->bo_ida);
	ida_init(&tdev->ctx_ida);
	ida_init(&tdev->job_ida);
	ida_init(&tdev->ipc_seq_ida);

	/*
	 * Initialize shared virtual address space for all DSP cores.
	 *
	 * IMPORTANT: This driver does NOT use Linux IOMMU. The TI C7x DSP cores
	 * have their own MMUs that are managed entirely by the DSP firmware.
	 * The VA space is shared across all cores - userspace receives VAs that
	 * work on all cores. Each core's firmware programs its own MMU to map
	 * the same VA to the same PA.
	 *
	 * The Linux driver's role is only to:
	 * 1. Allocate non-overlapping virtual addresses from a safe range
	 * 2. Provide physical addresses to each DSP firmware via IPC
	 * 3. Let each firmware program its own MMU to map VA -> PA
	 */
	if (!iova_size) {
		dev_err(dev, "Invalid DSP VA pool size: 0\n");
		return ERR_PTR(-EINVAL);
	}

	tdev->iova_start = iova_start;
	tdev->iova_size = iova_size;

	drm_mm_init(&tdev->mm, iova_start, iova_size);
	err = devm_mutex_init(dev, &tdev->mm_lock);
	if (err)
		return ERR_PTR(-ENOMEM);

	err = drm_dev_register(ddev, 0);
	if (err)
		return ERR_PTR(err);

	return tdev;
}

void thames_device_fini(struct thames_device *tdev)
{
	WARN_ON(tdev->num_cores > 0);

	ida_destroy(&tdev->bo_ida);
	ida_destroy(&tdev->ctx_ida);
	ida_destroy(&tdev->job_ida);
	ida_destroy(&tdev->ipc_seq_ida);
	drm_mm_takedown(&tdev->mm);
	drm_dev_unregister(&tdev->ddev);
}
