// SPDX-License-Identifier: GPL-2.0
/*
 * This driver exports interrupts and PDMA register space
 * to user space for applications interacting with PDMA
 *
 * Copyright (C) 2021 Microchip Incorporated - http://www.microchip.com/
 *
 * Author: Daire McNamara <daire.mcnamara@microchip.com>
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/genalloc.h>

#define DRV_NAME "pdma-uio"
#define DRV_VERSION "0.1"

/* Register Offset */
#define PDMA_CTRL					0x000
#define PDMA_XFER_TYPE					0x004
#define PDMA_XFER_SIZE					0x008
#define PDMA_DST_ADDR					0x010
#define PDMA_SRC_ADDR					0x018
#define PDMA_ACT_TYPE					0x104 /* Read-only */
#define PDMA_REMAINING_BYTE				0x108 /* Read-only */
#define PDMA_CUR_DST_ADDR				0x110 /* Read-only*/
#define PDMA_CUR_SRC_ADDR				0x118 /* Read-only*/

/* CTRL */

#define PDMA_DONE_STATUS_MASK	GENMASK(30, 30)
#define PDMA_ERR_STATUS_MASK	GENMASK(31, 31)

#define PDMA_NR_CH	(4)

#define PDMA_CHAN_OFFSET				0x1000

#define PDMA_REG_BASE(ch)	(dev_info->base + (PDMA_CHAN_OFFSET * (ch)))

struct pdma_regs {
	/* read-write regs */
	void __iomem *ctrl;		/* 4 bytes */

	void __iomem *xfer_type;	/* 4 bytes */
	void __iomem *xfer_size;	/* 8 bytes */
	void __iomem *dst_addr;		/* 8 bytes */
	void __iomem *src_addr;		/* 8 bytes */

	/* read-only */
	void __iomem *act_type;		/* 4 bytes */
	void __iomem *residue;		/* 8 bytes */
	void __iomem *cur_dst_addr;	/* 8 bytes */
	void __iomem *cur_src_addr;	/* 8 bytes */
};

struct uio_pdma_chan {
	int txirq;
	int errirq;
	struct pdma_regs regs;
};

struct uio_pdma {
	struct uio_info *uio_info[PDMA_NR_CH * 2];
	void __iomem *base;
	struct uio_pdma_chan chans[PDMA_NR_CH];
	unsigned int pintc_base;
};

static irqreturn_t uio_pdma_done_isr(int irq, struct uio_info *uio_info)
{
	struct uio_pdma_chan *chan = uio_info->priv;
	struct pdma_regs *regs = &chan->regs;
	u32 val;

	val = readl(regs->ctrl) & PDMA_DONE_STATUS_MASK;
	writel(val & ~PDMA_DONE_STATUS_MASK, regs->ctrl);

	return IRQ_HANDLED;
}

static irqreturn_t uio_pdma_err_isr(int irq, struct uio_info *uio_info)
{
	struct uio_pdma_chan *chan = uio_info->priv;
	struct pdma_regs *regs = &chan->regs;
	u32 val;

	val = readl(regs->ctrl) & PDMA_ERR_STATUS_MASK;
	writel(val & ~PDMA_ERR_STATUS_MASK, regs->ctrl);

	return IRQ_HANDLED;
}

static int uio_pdma_irq_init(struct platform_device *pdev,
			     struct uio_pdma *dev_info)
{
	int irq, i;
	struct uio_pdma_chan *chan;

	for (i = 0; i < PDMA_NR_CH; i++) {
		chan = &dev_info->chans[i];

		irq = platform_get_irq(pdev, i * 2);
		if (irq < 0) {
			dev_err(&pdev->dev, "ch(%d) Can't get done irq.\n", i);
			return -EINVAL;
		}

		chan->txirq = irq;

		irq = platform_get_irq(pdev, (i * 2) + 1);
		if (irq < 0) {
			dev_err(&pdev->dev, "ch(%d) Can't get err irq.\n", i);
			return -EINVAL;
		}

		chan->errirq = irq;

		chan->regs.ctrl =
			PDMA_REG_BASE(i) + PDMA_CTRL;
		chan->regs.xfer_type =
			PDMA_REG_BASE(i) + PDMA_XFER_TYPE;
		chan->regs.xfer_size =
			PDMA_REG_BASE(i) + PDMA_XFER_SIZE;
		chan->regs.dst_addr =
			PDMA_REG_BASE(i) + PDMA_DST_ADDR;
		chan->regs.src_addr =
			PDMA_REG_BASE(i) + PDMA_SRC_ADDR;
		chan->regs.act_type =
			PDMA_REG_BASE(i) + PDMA_ACT_TYPE;
		chan->regs.residue =
			PDMA_REG_BASE(i) + PDMA_REMAINING_BYTE;
		chan->regs.cur_dst_addr =
			PDMA_REG_BASE(i) + PDMA_CUR_DST_ADDR;
		chan->regs.cur_src_addr =
			PDMA_REG_BASE(i) + PDMA_CUR_SRC_ADDR;
	}

	return 0;
}

static void pdma_cleanup(struct device *dev, struct uio_pdma *dev_info)
{
	int cnt;
	struct uio_info *uio_info;

	for (cnt = 0; cnt < PDMA_NR_CH * 2; cnt++) {
		uio_info = dev_info->uio_info[cnt];
		uio_unregister_device(uio_info);
		kfree(uio_info->name);
		kfree(uio_info);
	}
	iounmap(dev_info->base);
	kfree(dev_info);
}

static int pdma_probe(struct platform_device *pdev)
{
	struct uio_info *uio_info;
	struct uio_pdma *dev_info;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int ret = -ENODEV, cnt = 0, len;
	int i;

	dev_info(dev, "Running Probe\n");

	dev_info = kzalloc(sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	for (i = 0; i < PDMA_NR_CH * 2; i++) {
		dev_info->uio_info[i] = kzalloc(sizeof(*dev_info->uio_info[i]),
						GFP_KERNEL);

		if (!dev_info->uio_info[i]) {
			for (cnt = 0; cnt < i; cnt++)
				kfree(dev_info->uio_info[cnt]);
			kfree(dev_info);
			return -ENOMEM;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "No PDMA I/O resource specified\n");
		goto out_free;
	}

	len = resource_size(res);
	dev_info->base = ioremap(res->start, len);
	if (!dev_info->base) {
		dev_err(dev, "Can't remap PDMA I/O  address range\n");
		goto out_free;
	}

	ret = uio_pdma_irq_init(pdev, dev_info);
	if (ret) {
		dev_err(dev, "Can't parse IRQs for PDMA\n");
		goto out_free;
	}

	for (cnt = 0; cnt < PDMA_NR_CH; cnt++) {
		uio_info = dev_info->uio_info[cnt * 2];

		uio_info->mem[0].addr = res->start;
		uio_info->mem[0].size = len;
		uio_info->mem[0].memtype = UIO_MEM_PHYS;

		uio_info->mem[1].size = 0;

		uio_info->name = kasprintf(GFP_KERNEL, "pdma%d", cnt);
		uio_info->version = DRV_VERSION;

		/* Register PDMA txdone IRQ line */
		uio_info->irq = dev_info->chans[cnt].txirq;
		uio_info->irq_flags = IRQF_SHARED;
		uio_info->handler = uio_pdma_done_isr;
		uio_info->priv = &dev_info->chans[cnt];

		ret = uio_register_device(dev, uio_info);
		if (ret < 0)
			goto out_free;

		uio_info = dev_info->uio_info[(cnt * 2) + 1];

		uio_info->mem[0].addr = res->start;
		uio_info->mem[0].size = len;
		uio_info->mem[0].memtype = UIO_MEM_PHYS;

		uio_info->mem[1].size = 0;

		uio_info->name = kasprintf(GFP_KERNEL, "pdmaerr%d", cnt);
		uio_info->version = DRV_VERSION;

		/* Register PDMA err IRQ line */
		uio_info->irq = dev_info->chans[cnt].errirq;
		uio_info->irq_flags = IRQF_SHARED;
		uio_info->handler = uio_pdma_err_isr;
		uio_info->priv = &dev_info->chans[cnt];

		ret = uio_register_device(dev, uio_info);
		if (ret < 0)
			goto out_free;
	}

	platform_set_drvdata(pdev, dev_info);

	dev_info(dev, "Registered 8 devices\n");

	return 0;

out_free:
	pdma_cleanup(dev, dev_info);
	return ret;
}

static int pdma_remove(struct platform_device *dev)
{
	struct uio_pdma *dev_info = platform_get_drvdata(dev);

	pdma_cleanup(&dev->dev, dev_info);
	return 0;
}

#define MICROCHIP_PDMA_PM_OPS (NULL)

#if defined(CONFIG_OF)
static const struct of_device_id pdma_dt_ids[] = {
	{ .compatible = "microchip,mpfs-pdma-uio" },
	{ /*sentinel */ }
};
#endif

static struct platform_driver pdma_driver = {
	.probe = pdma_probe,
	.remove = pdma_remove,
	.driver = {
		.name = DRV_NAME,
		.pm = MICROCHIP_PDMA_PM_OPS,
		.of_match_table = of_match_ptr(pdma_dt_ids),
		.owner = THIS_MODULE,
		   },
};

module_platform_driver(pdma_driver);

MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Daire McNamara <daire.mcnamara@microchip.com>");
