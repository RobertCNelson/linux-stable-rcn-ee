/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pruss.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>

/**
 * struct pruss - PRUSS parent structure
 * @dev: pruss device pointer
 * @cfg: regmap for config region
 * @mem_regions: data for each of the PRUSS memory regions
 * @mem_in_use: to indicate if memory resource is in use
 * @no_shared_ram: indicate that shared RAM is absent
 * @lock: mutex to serialize access to resources
 */
struct pruss {
	struct device *dev;
	struct regmap *cfg;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	bool no_shared_ram;
	struct mutex lock; /* PRU resource lock */
};

/**
 * pruss_get() - get the pruss for a given PRU remoteproc
 * @rproc: remoteproc handle of a PRU instance
 *
 * Finds the parent pruss device for a PRU given the @rproc handle of the
 * PRU remote processor. This function increments the pruss device's refcount,
 * so always use pruss_put() to decrement it back once pruss isn't needed
 * anymore.
 *
 * Returns the pruss handle on success, and an ERR_PTR on failure using one
 * of the following error values
 *    -EINVAL if invalid parameter
 *    -ENODEV if PRU device or PRUSS device is not found
 */
struct pruss *pruss_get(struct rproc *rproc)
{
	struct pruss *pruss;
	struct device *dev;
	struct platform_device *ppdev;

	if (IS_ERR(rproc))
		return ERR_PTR(-EINVAL);

	dev = &rproc->dev;
	if (!dev->parent)
		return ERR_PTR(-ENODEV);

	/* rudimentary check to make sure rproc handle is for a PRU */
	if (!strstr(dev_name(dev->parent), "pru"))
		return ERR_PTR(-ENODEV);

	ppdev = to_platform_device(dev->parent->parent);
	pruss = platform_get_drvdata(ppdev);
	if (pruss)
		get_device(pruss->dev);

	return pruss ? pruss : ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(pruss_get);

/**
 * pruss_put() - decrement pruss device's usecount
 * @pruss: pruss handle
 *
 * Complimentary function for pruss_get(). Needs to be called
 * after the PRUSS is used, and only if the pruss_get() succeeds.
 */
void pruss_put(struct pruss *pruss)
{
	if (IS_ERR(pruss))
		return;

	put_device(pruss->dev);
}
EXPORT_SYMBOL_GPL(pruss_put);

/**
 * pruss_request_mem_region() - request a memory resource
 * @pruss: the pruss instance
 * @mem_id: the memory resource id
 * @region: pointer to memory region structure to be filled in
 *
 * This function allows a client driver to request a memory resource,
 * and if successful, will let the client driver own the particular
 * memory region until released using the pruss_release_mem_region()
 * API.
 *
 * Returns the memory region if requested resource is available, an
 * error otherwise
 */
int pruss_request_mem_region(struct pruss *pruss, enum pruss_mem mem_id,
			     struct pruss_mem_region *region)
{
	if (IS_ERR(pruss) || !region)
		return -EINVAL;

	if (mem_id >= PRUSS_MEM_MAX)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	if (pruss->mem_in_use[mem_id]) {
		mutex_unlock(&pruss->lock);
		return -EBUSY;
	}

	*region = pruss->mem_regions[mem_id];
	pruss->mem_in_use[mem_id] = region;

	mutex_unlock(&pruss->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_request_mem_region);

/**
 * pruss_release_mem_region() - release a memory resource
 * @pruss: the pruss instance
 * @region: the memory region to release
 *
 * This function is the complimentary function to
 * pruss_request_mem_region(), and allows the client drivers to
 * release back a memory resource.
 *
 * Returns 0 on success, an error code otherwise
 */
int pruss_release_mem_region(struct pruss *pruss,
			     struct pruss_mem_region *region)
{
	int id;

	if (IS_ERR(pruss) || !region)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	/* find out the memory region being released */
	for (id = 0; id < PRUSS_MEM_MAX; id++) {
		if (pruss->mem_in_use[id] == region)
			break;
	}

	if (id == PRUSS_MEM_MAX) {
		mutex_unlock(&pruss->lock);
		return -EINVAL;
	}

	pruss->mem_in_use[id] = NULL;

	mutex_unlock(&pruss->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_release_mem_region);

/**
 * pruss_cfg_read() - read a PRUSS CFG register
 * @pruss: the pruss instance handle
 * @reg: register offset within the CFG sub-module
 * @val: pointer to return the value in
 *
 * Reads a given register within CFG module of PRUSS
 * and returns it through the passed-in @val pointer
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_cfg_read(struct pruss *pruss, unsigned int reg, unsigned int *val)
{
	if (IS_ERR(pruss))
		return -EINVAL;

	return regmap_read(pruss->cfg, reg, val);
}
EXPORT_SYMBOL_GPL(pruss_cfg_read);

/**
 * pruss_cfg_update() - update a PRUSS CFG register
 * @pruss: the pruss instance handle
 * @reg: register offset within the CFG sub-module
 * @mask: bit mask to use for programming the @val
 * @val: value to write
 *
 * Updates a given register within CFG sub-module of PRUSS
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_cfg_update(struct pruss *pruss, unsigned int reg,
		     unsigned int mask, unsigned int val)
{
	if (IS_ERR(pruss))
		return -EINVAL;

	return regmap_update_bits(pruss->cfg, reg, mask, val);
}
EXPORT_SYMBOL_GPL(pruss_cfg_update);

/**
 * struct pruss_match_private_data - private data to handle multiple instances
 * @device_name: device name of the PRUSS instance
 * @priv_data: PRUSS driver private data for this PRUSS instance
 */
struct pruss_match_private_data {
	const char *device_name;
	const struct pruss_private_data *priv_data;
};

static const
struct pruss_private_data *pruss_get_private_data(struct platform_device *pdev)
{
	const struct pruss_match_private_data *data;

	if (!of_device_is_compatible(pdev->dev.of_node, "ti,am4376-pruss"))
		return NULL;

	data = of_device_get_match_data(&pdev->dev);
	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->priv_data;
	}

	return ERR_PTR(-ENODEV);
}

static int pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *np;
	struct pruss *pruss;
	struct resource *res;
	int ret, i;
	const struct pruss_private_data *data;
	const char *mem_names[PRUSS_MEM_MAX] = { "dram0", "dram1", "shrdram2" };

	if (!node) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	data = pruss_get_private_data(pdev);
	if (IS_ERR(data)) {
		dev_err(dev, "missing private data\n");
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
	mutex_init(&pruss->lock);

	pruss->no_shared_ram = of_property_read_bool(node, "no-shared-ram");

	np = of_get_child_by_name(node, "cfg");
	if (!np)
		return -ENODEV;

	pruss->cfg = syscon_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(pruss->cfg))
		return -ENODEV;

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		if (pruss->no_shared_ram && !strcmp(mem_names[i], "shrdram2"))
			continue;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pruss->mem_regions[i].va = devm_ioremap_resource(dev, res);
		if (!pruss->mem_regions[i].va) {
			dev_err(dev, "failed to get resource: %s\n",
				mem_names[i]);
			return -ENODEV;
		}
		pruss->mem_regions[i].pa = res->start;
		pruss->mem_regions[i].size = resource_size(res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%zx va %p\n",
			mem_names[i], &pruss->mem_regions[i].pa,
			pruss->mem_regions[i].size, pruss->mem_regions[i].va);
	}

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
	{ .compatible = "ti,am4376-pruss", },
	{ .compatible = "ti,am5728-pruss", },
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
