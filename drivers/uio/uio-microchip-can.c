/*
 * MSS CAN UIO driver (uio_mss_can)
 *
 * This driver exports interrupts and MSS CAN register space
 * to user space for applications interacting with MSS CAN
 *
 * Copyright (C) 2018-19 Microchip Incorporated - http://www.microchip.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/genalloc.h>

#define DRV_NAME "mss-can-uio"
#define DRV_VERSION "0.1"

#define CAN_INT_ENABLE (4)
#define CAN_INT_STATUS (0)

#define MAX_MSS_CAN_EVT	1

struct uio_mss_can_dev {
	struct uio_info *info;
	struct clk *mss_can_clk;
	void __iomem *mss_can_io_vaddr;
	int irq;
	unsigned int pintc_base;
};

static irqreturn_t mss_can_handler(int irq, struct uio_info *info)
{
	struct uio_mss_can_dev *dev_info = info->priv;
	int val;
	void __iomem *base = dev_info->mss_can_io_vaddr + dev_info->pintc_base;
	void __iomem *intren_reg = base + CAN_INT_ENABLE;
	void __iomem *intrstat_reg = base + CAN_INT_STATUS;

	val = ioread32(intren_reg);
	/* Is interrupt enabled and active ? */
	if (!(val & 0xffff) && (ioread32(intrstat_reg) & 0xffff))
		return IRQ_NONE;
	return IRQ_HANDLED;
}

static void mss_can_cleanup(struct device *dev,
		struct uio_mss_can_dev *dev_info)
{
	int cnt;
	struct uio_info *p = dev_info->info;

	for (cnt = 0; cnt < MAX_MSS_CAN_EVT; cnt++, p++) {
		uio_unregister_device(p);
		kfree(p->name);
	}
	iounmap(dev_info->mss_can_io_vaddr);
	kfree(dev_info->info);
	clk_disable(dev_info->mss_can_clk);
	clk_put(dev_info->mss_can_clk);
	kfree(dev_info);
}

static int mss_can_probe(struct platform_device *pdev)
{
	struct uio_info *p;
	struct uio_mss_can_dev *dev_info;
	struct resource *regs_mss_can_io;
	struct device *dev = &pdev->dev;
	int ret = -ENODEV, cnt = 0, len;
	/* struct uio_mss_can_pdata *pdata = dev_get_platdata(dev); TODO */

	dev_info(dev, "Running Probe\n");

	dev_info = kzalloc(sizeof(struct uio_mss_can_dev), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	dev_info->info = kzalloc(sizeof(*p) * MAX_MSS_CAN_EVT, GFP_KERNEL);
	if (!dev_info->info) {
		kfree(dev_info);
		return -ENOMEM;
	}

	/* Power on PRU in case its not done as part of boot-loader */
	dev_info->mss_can_clk = devm_clk_get(dev, NULL);
	if ((!dev_info->mss_can_clk) || (IS_ERR(dev_info->mss_can_clk))) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(dev_info->mss_can_clk);
		kfree(dev_info->info);
		kfree(dev_info);
		return ret;
	} else {
		ret = clk_prepare_enable(dev_info->mss_can_clk);
		if (ret) {
			dev_err(dev, "Failed to enable clock\n");
			clk_put(dev_info->mss_can_clk);
			kfree(dev_info->info);
			kfree(dev_info);
			return ret;
		}
	}
	devm_add_action_or_reset(dev, (void (*) (void *))clk_disable_unprepare, dev_info->mss_can_clk);

	regs_mss_can_io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_mss_can_io) {
		dev_err(dev, "No MSS CAN I/O resource specified\n");
		goto out_free;
	}

	if (!regs_mss_can_io->start) {
		dev_err(dev, "Invalid memory resource\n");
		goto out_free;
	}

	len = resource_size(regs_mss_can_io);
	dev_info->mss_can_io_vaddr = ioremap(regs_mss_can_io->start, len);
	if (!dev_info->mss_can_io_vaddr) {
		dev_err(dev, "Can't remap MSS CAN I/O  address range\n");
		goto out_free;
	}

	dev_info->irq = platform_get_irq(pdev, 0);

	p = dev_info->info;

	p->mem[0].addr = regs_mss_can_io->start;
	p->mem[0].size = resource_size(regs_mss_can_io);
	p->mem[0].memtype = UIO_MEM_PHYS;

	p->mem[1].size = 0;

	p->name = kasprintf(GFP_KERNEL, "mss_can%d", cnt);
	p->version = DRV_VERSION;

	/* Register MSS CAN IRQ lines */
	p->irq = dev_info->irq;
	p->irq_flags = IRQF_SHARED;
	p->handler = mss_can_handler;
	p->priv = dev_info;

	ret = uio_register_device(dev, p);
	if (ret < 0)
		goto out_free;

	platform_set_drvdata(pdev, dev_info);
	return 0;

out_free:
	mss_can_cleanup(dev, dev_info);
	return ret;
}

static int mss_can_remove(struct platform_device *dev)
{
	struct uio_mss_can_dev *dev_info = platform_get_drvdata(dev);

	mss_can_cleanup(&dev->dev, dev_info);
	return 0;
}

#define MICROCHIP_CAN_PM_OPS (NULL)

#if defined(CONFIG_OF)
static const struct of_device_id mss_can_dt_ids[] = {
	{ .compatible = "microchip,mpfs-can-uio" },
	{ /*sentinel */ }
};
#endif

static struct platform_driver mss_can_driver = {
	.probe = mss_can_probe,
	.remove = mss_can_remove,
	.driver = {
		.name = DRV_NAME,
		.pm = MICROCHIP_CAN_PM_OPS,
		.of_match_table = of_match_ptr(mss_can_dt_ids),
		.owner = THIS_MODULE,
		   },
};

module_platform_driver(mss_can_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Daire McNamara <daire.mcnamara@microchip.com>");
