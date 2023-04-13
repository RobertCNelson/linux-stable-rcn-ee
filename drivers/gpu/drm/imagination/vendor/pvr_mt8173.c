// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */
/* Parts copyright (c) 2014 MediaTek Inc. */
#include "pvr_device.h"
#include "pvr_vendor.h"

#include <drm/drm_print.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* Taken from img-rogue/mt8173/mt8173_mfgsys.c in ChromeOS kernel */
#define REG_MFG_AXI BIT(0)
#define REG_MFG_MEM BIT(1)
#define REG_MFG_G3D BIT(2)
#define REG_MFG_26M BIT(3)
#define REG_MFG_ALL (REG_MFG_AXI | REG_MFG_MEM | REG_MFG_G3D | REG_MFG_26M)

#define REG_MFG_CG_STA 0x00
#define REG_MFG_CG_SET 0x04
#define REG_MFG_CG_CLR 0x08

struct pvr_mt8173_data {
	void __iomem *regs;
};

static int
mt8173_init(struct pvr_device *pvr_dev)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct platform_device *plat_dev = to_platform_device(drm_dev->dev);
	struct pvr_mt8173_data *mt8173_data;
	void __iomem *regs;
	int err;

	mt8173_data = kzalloc(sizeof(*mt8173_data), GFP_KERNEL);
	if (!mt8173_data) {
		err = -ENOMEM;
		goto err_out;
	}

	regs = devm_platform_ioremap_resource(plat_dev, 1);
	if (IS_ERR(regs)) {
		err = PTR_ERR(regs);
		drm_err(drm_dev,
			"failed to ioremap mt8173 gpu registers (err=%d)\n",
			err);
		goto err_free;
	}

	mt8173_data->regs = regs;

	pvr_dev->vendor.data = mt8173_data;

	return 0;

err_free:
	kfree(mt8173_data);

err_out:
	return err;
}

static void
mt8173_fini(struct pvr_device *pvr_dev)
{
	struct pvr_mt8173_data *mt8173_data = pvr_dev->vendor.data;

	kfree(mt8173_data);
	pvr_dev->vendor.data = NULL;
}

static void
mtk_mfg_enable_hw_apm(struct pvr_mt8173_data *mt8173_data)
{
	/* Taken from img-rogue/mt8173/mt8173_mfgsys.c in ChromeOS kernel */
	writel(0x003c3d4d, mt8173_data->regs + 0x24);
	writel(0x4d45440b, mt8173_data->regs + 0x28);
	writel(0x7a710184, mt8173_data->regs + 0xe0);
	writel(0x835f6856, mt8173_data->regs + 0xe4);
	writel(0x002b0234, mt8173_data->regs + 0xe8);
	writel(0x80000000, mt8173_data->regs + 0xec);
	writel(0x08000000, mt8173_data->regs + 0xa0);
}

static void
mtk_mfg_disable_hw_apm(struct pvr_mt8173_data *mt8173_data)
{
	/* Taken from img-rogue/mt8173/mt8173_mfgsys.c in ChromeOS kernel */
	writel(0x00, mt8173_data->regs + 0xec);
}

static int
mt8173_power_enable(struct pvr_device *pvr_dev)
{
	struct pvr_mt8173_data *mt8173_data = pvr_dev->vendor.data;

	/* Taken from img-rogue/mt8173/mt8173_mfgsys.c in ChromeOS kernel */
	writel(REG_MFG_ALL, mt8173_data->regs + REG_MFG_CG_CLR);
	mtk_mfg_enable_hw_apm(mt8173_data);

	return 0;
}

static int
mt8173_power_disable(struct pvr_device *pvr_dev)
{
	struct pvr_mt8173_data *mt8173_data = pvr_dev->vendor.data;

	mtk_mfg_disable_hw_apm(mt8173_data);

	return 0;
}

const struct pvr_vendor_callbacks pvr_mt8173_callbacks = {
	.init = mt8173_init,
	.fini = mt8173_fini,
	.power_enable = mt8173_power_enable,
	.power_disable = mt8173_power_disable,
};
