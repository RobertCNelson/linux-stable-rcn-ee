// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include "pruss.h"

static int pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *np;
	struct pruss *pruss;
	struct resource res;
	int ret, i, index;
	const char *mem_names[PRUSS_MEM_MAX] = { "dram0", "dram1", "shrdram2" };

	if (!node) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	pruss = devm_kzalloc(dev, sizeof(*pruss), GFP_KERNEL);
	if (!pruss)
		return -ENOMEM;

	pruss->dev = dev;

	np = of_get_child_by_name(node, "memories");
	if (!np)
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		index = of_property_match_string(np, "reg-names", mem_names[i]);
		if (index < 0) {
			of_node_put(np);
			return index;
		}

		if (of_address_to_resource(np, index, &res)) {
			of_node_put(np);
			return -EINVAL;
		}

		pruss->mem_regions[i].va = devm_ioremap(dev, res.start,
							resource_size(&res));
		if (!pruss->mem_regions[i].va) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			of_node_put(np);
			return -ENOMEM;
		}
		pruss->mem_regions[i].pa = res.start;
		pruss->mem_regions[i].size = resource_size(&res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%zx va %p\n",
			mem_names[i], &pruss->mem_regions[i].pa,
			pruss->mem_regions[i].size, pruss->mem_regions[i].va);
	}
	of_node_put(np);

	platform_set_drvdata(pdev, pruss);

	dev_info(&pdev->dev, "creating PRU cores and other child platform devices\n");
	ret = of_platform_populate(node, NULL, NULL, &pdev->dev);
	if (ret)
		dev_err(dev, "of_platform_populate failed\n");

	return ret;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(dev, "remove PRU cores and other child platform devices\n");
	of_platform_depopulate(dev);

	return 0;
}

static const struct of_device_id pruss_of_match[] = {
	{ .compatible = "ti,am3356-pruss", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_of_match);

static struct platform_driver pruss_driver = {
	.driver = {
		.name = "pruss",
		.of_match_table = pruss_of_match,
	},
	.probe  = pruss_probe,
	.remove = pruss_remove,
};
module_platform_driver(pruss_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Subsystem Driver");
MODULE_LICENSE("GPL v2");
