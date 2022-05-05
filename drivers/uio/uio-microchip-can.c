// SPDX-License-Identifier: GPL-2.0
/*
 * This driver exports interrupts and CAN register space
 * to user space for applications interacting with PDMA
 *
 * Copyright (C) 2018 - 2021 Microchip Incorporated - http://www.microchip.com/
 *
 * Author: Daire McNamara <daire.mcnamara@microchip.com>
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uio_driver.h>

#define DRV_NAME "can-uio"
#define DRV_VERSION "0.1"

#define CAN_INT_ENABLE (4)
#define CAN_INT_STATUS (0)

#define MAX_CAN_EVT	1

struct uio_can_dev {
	struct uio_info *uio_info;
	struct clk *clk;
	void __iomem *base;
	int irq;
};

static irqreturn_t can_handler(int irq, struct uio_info *uio_info)
{
	struct uio_can_dev *dev_info = uio_info->priv;
	void __iomem *base = dev_info->base;
	int val;

	val = ioread32(base + CAN_INT_STATUS);

	/* clear anything that was active */
	iowrite32(val, base + CAN_INT_STATUS);

	/* is interrupt enabled and active ? */
	if (!(val & 0xffff) && (ioread32(base + CAN_INT_ENABLE) & 0xffff))
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int can_probe(struct platform_device *pdev)
{
	struct uio_info *uio_info;
	struct uio_can_dev *dev_info;
	struct resource *res;
	struct device *dev = &pdev->dev;
	int ret = -ENODEV, cnt = 0, len;

	dev_info = kzalloc(sizeof(*dev_info), GFP_KERNEL);
	if (!dev_info)
		return -ENOMEM;

	dev_info->uio_info = kzalloc(sizeof(*dev_info->uio_info) * MAX_CAN_EVT, GFP_KERNEL);
	if (!dev_info->uio_info) {
		ret = -ENOMEM;
		goto out_free_dev;
	}

	/* power on PRU in case its not done as part of boot-loader */
	dev_info->clk = devm_clk_get(dev, NULL);
	if (!dev_info->clk || (IS_ERR(dev_info->clk))) {
		dev_err(dev, "failed to get clock\n");
		ret = PTR_ERR(dev_info->clk);
		goto out_free_uio;
	}

	ret = clk_prepare_enable(dev_info->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		goto out_free_uio;
	}

	ret = devm_add_action_or_reset(dev, (void (*) (void *))clk_disable_unprepare,
				       dev_info->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		goto out_free_uio;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no CAN I/O resource specified\n");
		ret = -ENODEV;
		goto out_free_uio;
	}

	len = resource_size(res);

	dev_info->base = ioremap(res->start, len);
	if (!dev_info->base) {
		dev_err(dev, "failed to remap CAN I/O  address range\n");
		ret = -ENODEV;
		goto out_free_uio;
	}

	dev_info->irq = platform_get_irq(pdev, 0);
	if (dev_info->irq < 0) {
		dev_err(dev, "failed to get irq.\n");
		ret = -ENODEV;
		goto out_iounmap;
	}

	uio_info = dev_info->uio_info;

	uio_info->mem[0].addr = res->start;
	uio_info->mem[0].size = resource_size(res);
	uio_info->mem[0].memtype = UIO_MEM_PHYS;

	uio_info->mem[1].size = 0;

	uio_info->name = dev->of_node->full_name;
	uio_info->version = DRV_VERSION;

	/* register CAN IRQ lines */
	uio_info->irq = dev_info->irq;
	uio_info->irq_flags = IRQF_SHARED;
	uio_info->handler = can_handler;
	uio_info->priv = dev_info;

	ret = uio_register_device(dev, uio_info);
	if (ret < 0) {
		dev_err(dev, "failed to register CAN device\n");
		goto out_unregister;
	}

	platform_set_drvdata(pdev, dev_info);

	dev_info(dev, "registered device as %s\n", uio_info->name);

	return 0;

out_unregister:
	for (cnt = 0; cnt < MAX_CAN_EVT; cnt++, uio_info++) {
		uio_unregister_device(uio_info);
		kfree(uio_info->name);
	}
out_iounmap:
	iounmap(dev_info->base);
out_free_uio:
	kfree(dev_info->uio_info);
out_free_dev:
	kfree(dev_info);

	return ret;
}

static int can_remove(struct platform_device *dev)
{
	struct uio_can_dev *dev_info = platform_get_drvdata(dev);
	struct uio_info *uio_info = dev_info->uio_info;
	int cnt;

	for (cnt = 0; cnt < MAX_CAN_EVT; cnt++, uio_info++) {
		uio_unregister_device(uio_info);
		kfree(uio_info->name);
	}
	iounmap(dev_info->base);
	kfree(dev_info->uio_info);
	kfree(dev_info);

	return 0;
}

#define MICROCHIP_CAN_PM_OPS (NULL)

#if defined(CONFIG_OF)
static const struct of_device_id can_dt_ids[] = {
	{ .compatible = "microchip,mpfs-can" },
	{ /*sentinel */ }
};
#endif

static struct platform_driver can_driver = {
	.probe = can_probe,
	.remove = can_remove,
	.driver = {
		.name = DRV_NAME,
		.pm = MICROCHIP_CAN_PM_OPS,
		.of_match_table = of_match_ptr(can_dt_ids),
		.owner = THIS_MODULE,
		   },
};

module_platform_driver(can_driver);

MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Daire McNamara <daire.mcnamara@microchip.com>");
