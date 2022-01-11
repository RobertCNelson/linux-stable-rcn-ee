/*
 * MSS DMA UIO driver (uio_mss_dma)
 *
 * This driver exports interrupts and MSS DMA register space
 * to user space for applications interacting with MSS DMA
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

#define DRV_NAME "mss-dma-uio"
#define DRV_VERSION "0.1"

#define MAX_MSS_DMA_EVT	1
#define DMA_INT_CLR  0x18
#define DMA_INT_STA  0x10
#define DMA_INT_OCCURED 0x01

struct uio_mss_dma_dev {
	struct uio_info *info;
	struct clk *mss_dma_clk;
	void __iomem *mss_dma_io_vaddr;
	int irq;
	unsigned int pintc_base;
};

static irqreturn_t mss_dma_handler(int irq, struct uio_info *info)
{
	struct uio_mss_dma_dev *dev_info = info->priv;
	void __iomem *base = dev_info->mss_dma_io_vaddr;
	void __iomem *int_clear = base + (DMA_INT_CLR);
	void __iomem *int_sta = base + (DMA_INT_STA);

	if((ioread32(int_sta) & DMA_INT_OCCURED) == DMA_INT_OCCURED)
	{
		iowrite32(DMA_INT_OCCURED, int_clear);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void mss_dma_cleanup(struct device *dev,
		struct uio_mss_dma_dev *dev_info)
{
	int cnt;
	struct uio_info *p = dev_info->info;

	for (cnt = 0; cnt < MAX_MSS_DMA_EVT; cnt++, p++) {
		uio_unregister_device(p);
		kfree(p->name);
	}
	iounmap(dev_info->mss_dma_io_vaddr);
	kfree(dev_info->info);

	kfree(dev_info);
}

static int mss_dma_probe(struct platform_device *pdev)
{
	struct uio_info *p;
	struct uio_mss_dma_dev *dev_info;
	struct resource *regs_mss_dma_io;
	struct device *dev = &pdev->dev;
	int ret = -ENODEV, cnt = 0, len;
	/* struct uio_mss_dma_pdata *pdata = dev_get_platdata(dev); TODO */

	dev_info(dev, "Running Probe\n");

	dev_info = kzalloc(sizeof(struct uio_mss_dma_dev), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	dev_info->info = kzalloc(sizeof(*p) * MAX_MSS_DMA_EVT, GFP_KERNEL);
	if (!dev_info->info) {
		kfree(dev_info);
		return -ENOMEM;
	}

	regs_mss_dma_io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_mss_dma_io) {
		dev_err(dev, "No MSS DMA I/O resource specified\n");
		goto out_free;
	}

	if (!regs_mss_dma_io->start) {
		dev_err(dev, "Invalid memory resource\n");
		goto out_free;
	}

	len = resource_size(regs_mss_dma_io);
	dev_info->mss_dma_io_vaddr = ioremap(regs_mss_dma_io->start, len);
	if (!dev_info->mss_dma_io_vaddr) {
		dev_err(dev, "Can't remap MSS DMA I/O  address range\n");
		goto out_free;
	}

	dev_info->irq = platform_get_irq(pdev, 0);

	p = dev_info->info;

	p->mem[0].addr = regs_mss_dma_io->start;
	p->mem[0].size = resource_size(regs_mss_dma_io);
	p->mem[0].memtype = UIO_MEM_PHYS;

	p->mem[1].size = 0;

	p->name = kasprintf(GFP_KERNEL, "fpga_dma%d", cnt);
	p->version = DRV_VERSION;

	/* Register MSS DMA IRQ lines */
	p->irq = dev_info->irq;
	p->irq_flags = IRQF_SHARED;
	p->handler = mss_dma_handler;
	p->priv = dev_info;

	ret = uio_register_device(dev, p);
	if (ret < 0)
		goto out_free;

	platform_set_drvdata(pdev, dev_info);
	return 0;

out_free:
	mss_dma_cleanup(dev, dev_info);
	return ret;
}

static int mss_dma_remove(struct platform_device *dev)
{
	struct uio_mss_dma_dev *dev_info = platform_get_drvdata(dev);

	mss_dma_cleanup(&dev->dev, dev_info);
	return 0;
}

#define MICROSEMI_DMA_PM_OPS (NULL)

#if defined(CONFIG_OF)
static const struct of_device_id mss_dma_dt_ids[] = {
	{ .compatible = "microchip,mpfs-fpga-dma-uio" },
	{ /*sentinel */ }
};
#endif

static struct platform_driver mss_dma_driver = {
	.probe = mss_dma_probe,
	.remove = mss_dma_remove,
	.driver = {
		.name = DRV_NAME,
		.pm = MICROSEMI_DMA_PM_OPS,
		.of_match_table = of_match_ptr(mss_dma_dt_ids),
		.owner = THIS_MODULE,
		   },
};

module_platform_driver(mss_dma_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Vattipalli Praveen <praveen.kumar@microchip.com>");
