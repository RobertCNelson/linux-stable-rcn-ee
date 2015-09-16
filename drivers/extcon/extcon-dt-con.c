/*
 *  drivers/extcon/extcon-dt-conn.c
 *
 *  Device Tree Overlay based connector driver.
 *
 * Copyright (C) 2015 Konsulko Group.
 * Author: Pantelis Antoniou <pantelis.antoniou@konsulko.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/extcon.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "extcon-dt-con.h"

static const unsigned int dtcon_cable[] = {
	EXTCON_NONE,
};

static int dtcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *npn;
	struct dtcon_data *dtcd;
	struct dtcon_pin *dtcp;
	const __be32 *regp;
	int regsz, regcells, sz, count, len, i, j, err;
	char *s, *e;
	u32 num;

	if (!np) {
		dev_err(dev, "No device node\n");
		return -ENODEV;
	}

	dtcd = devm_kzalloc(dev, sizeof(*dtcd), GFP_KERNEL);
	if (!dtcd) {
		dev_err(dev, "Failed to allocated data structure\n");
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&dtcd->pin_list);
	INIT_LIST_HEAD(&dtcd->function_list);

	dtcd->pdev = pdev;
	dtcd->edev = devm_extcon_dev_allocate(dev, dtcon_cable);
	if (IS_ERR(dtcd->edev)) {
		dev_err(dev, "Failed to allocate extcon device\n");
		err = PTR_ERR(dtcd->edev);
		return err;
	}

	err = devm_extcon_dev_register(dev, dtcd->edev);
	if (err < 0) {
		dev_err(dev, "Failed to register extcon device\n");
		return err;
	}

	platform_set_drvdata(pdev, dtcd);

	dtcd->connector = of_get_child_by_name(np, "connector");
	dtcd->functions = of_get_child_by_name(np, "functions");
	dtcd->plugged = of_get_child_by_name(np, "plugged");

	err = -EINVAL;

	if (!dtcd->connector || !dtcd->functions || !dtcd->plugged) {
		dev_err(dev, "Bad OF configuration\n");
		goto out_err;
	}

	/* if no property return 1 */
	err = of_property_read_u32(dtcd->connector, "#address-cells", &num);
	if (err < 0)
		num = 1;
	dtcd->connector_address_cells = num;

	/* if no property return 0 */
	err = of_property_read_u32(dtcd->connector, "#size-cells", &num);
	if (err < 0)
		num = 0;
	dtcd->connector_size_cells = num;
	if (dtcd->connector_size_cells != 0) {
		dev_err(dev, "Only #size-cells = <0>; supported for now\n");
		err = -EINVAL;
		goto out_err;
	}

	regcells = dtcd->connector_address_cells;
	regsz = sizeof(u32) * regcells;
	for_each_child_of_node(dtcd->connector, npn) {
		regp = of_get_property(npn, "reg", &len);
		if (!regp || (len % regsz)) {
			dev_err(dev, "Bad connector pin @%s\n", npn->name);
			of_node_put(npn);
			goto out_err;
		}

		dtcp = devm_kzalloc(dev, sizeof(*dtcp), GFP_KERNEL);
		if (!dtcp) {
			dev_err(dev, "Failed to allocate pin node\n");
			err = -ENOMEM;
			of_node_put(npn);
			goto out_err;
		}
		/* keep it and add it to the list */
		dtcp->np = of_node_get(npn);

		/* convert to number of reg ranges */
		count = len / regsz;
		/* max reg is ,<4294967295> */
		sz = strlen(",<4294967295>") * count * regcells + 2;
		dtcp->regstr = devm_kmalloc(dev, sz, GFP_KERNEL);
		if (!dtcp->regstr) {
			dev_err(dev, "Failed to allocate reg string\n");
			err = -ENOMEM;
			of_node_put(npn);
			goto out_err;
		}
		s = dtcp->regstr;
		e = dtcp->regstr + sz;
		for (i = 0; i < count; i++, regp += regcells) {
			if (i > 0)
				*s++ = ',';
			*s++ = '<';
			for (j = 0; j < regcells; j++) {
				len = scnprintf(s, e - s, "%u",
						be32_to_cpu(regp[j]));
				s += len;
				if (j + 1 < regcells)
					*s++ = ' ';
			}
			*s++ = '>';
		}
		*s = '\0';

		list_add_tail(&dtcp->node, &dtcd->pin_list);
	}

	/* populate anything in plugged */
	of_platform_default_populate(dev->of_node, NULL, dev);

	dev_info(dev, "OK\n");

	return 0;

out_err:
	if (dtcd) {
		list_for_each_entry_reverse(dtcp, &dtcd->pin_list, node)
			of_node_put(dtcp->np);
		of_node_put(dtcd->connector);
		of_node_put(dtcd->functions);
		of_node_put(dtcd->plugged);
	}

	/* no need to manually deallocate, it's all devm */
	return err;
}

static int dtcon_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dtcon_data *dtcd = platform_get_drvdata(pdev);
	struct dtcon_pin *dtcp;

	dev_info(dev, "removing\n");

	list_for_each_entry_reverse(dtcp, &dtcd->pin_list, node)
		of_node_put(dtcp->np);
	of_node_put(dtcd->connector);
	of_node_put(dtcd->functions);
	of_node_put(dtcd->plugged);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dtcon_resume(struct device *dev)
{
	struct dtcon_data *dtcd;

	dtcd = dev_get_drvdata(dev);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(dtcon_pm_ops, NULL, dtcon_resume);

static const struct of_device_id dtcon_of_match[] = {
	{
		.compatible = "extcon,dt-con",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, dtcon_of_match);

static struct platform_driver dtcon_driver = {
	.probe		= dtcon_probe,
	.remove		= dtcon_remove,
	.driver		= {
		.name	= "extcon-dt-con",
		.pm	= &dtcon_pm_ops,
		.of_match_table = of_match_ptr(dtcon_of_match),
	},
};

module_platform_driver(dtcon_driver);

/* get the dtcon data from a platform device that's a descendant */
struct dtcon_data *dtcon_data_from_platform_device(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dtcon_data *dtcd;
	struct device *parent;

	/* we need to find the connector which must be a parent */
	for (parent = dev->parent; parent; parent = parent->parent) {
		if (parent->of_node &&
			of_device_is_compatible(parent->of_node,
				dtcon_of_match[0].compatible)) {
			break;
		}
	}

	if (!parent) {
		dev_err(dev, "Device is not a child of a connector\n");
		return NULL;
	}

	/* get pointer to connector data */
	dtcd = platform_get_drvdata(to_platform_device(parent));
	if (!dtcd) {
		dev_err(dev, "Missing connector data\n");
		return NULL;
	}

	return dtcd;
}
EXPORT_SYMBOL(dtcon_data_from_platform_device);

struct dtcon_pin *dtcon_pin_lookup(struct dtcon_data *dtcd,
		const void *regp, int regsz)
{
	struct dtcon_pin *dtcp = NULL;
	const void *value;
	int i, len;

	if (!dtcd || !regp)
		return NULL;

	/* must be the same size as the address cells */
	if (regsz != dtcd->connector_address_cells * sizeof(u32))
		return NULL;

	/* no size cells supported */
	if (dtcd->connector_size_cells != 0)
		return NULL;

	/* find pin with the given reg */
	list_for_each_entry(dtcp, &dtcd->pin_list, node) {
		value = of_get_property(dtcp->np, "reg", &len);

		/* check that it exists and is a multiple */
		if (!value || (len % regsz) != 0)
			continue;

		/* iterate over all */
		for (i = 0; i < len; i += regsz, value += regsz) {
			if (memcmp(value, regp, regsz) == 0)
				goto found;
		}
	}

	return NULL;

found:
	return dtcp;
}
EXPORT_SYMBOL(dtcon_pin_lookup);

struct dtcon_pin *dtcon_pin_lookup_by_node(struct dtcon_data *dtcd,
		struct device_node *np)
{
	struct dtcon_pin *dtcp = NULL;

	if (!dtcd)
		return NULL;

	/* release all pins of this proxy */
	list_for_each_entry(dtcp, &dtcd->pin_list, node) {
		if (dtcp->np == np)
			return dtcp;
	}

	return NULL;
}
EXPORT_SYMBOL(dtcon_pin_lookup_by_node);

struct dtcon_pin *dtcon_pin_lookup_by_phandle(struct dtcon_data *dtcd,
		phandle phandle)
{
	struct device_node *np = of_find_node_by_phandle(phandle);
	struct dtcon_pin *dtcp;

	if (!np)
		return NULL;

	dtcp = dtcon_pin_lookup_by_node(dtcd, np);
	of_node_put(np);
	return dtcp;
}
EXPORT_SYMBOL(dtcon_pin_lookup_by_phandle);

struct dtcon_pin *dtcon_proxy_pin_request(struct dtcon_proxy *proxy,
		const void *regp, int regsz, unsigned int flags)
{
	struct dtcon_data *dtcd;
	struct dtcon_pin *dtcp;
	struct device *dev;

	if (!proxy || !regp)
		return ERR_PTR(-EINVAL);

	dtcd = proxy->dtcf->dtcd;
	dev = &dtcd->pdev->dev;
	dtcp = dtcon_pin_lookup(dtcd, regp, regsz);
	if (!dtcp)
		return ERR_PTR(-ENXIO);

	dtcp->proxy = proxy;
	dtcp->data = NULL;
	list_add_tail(&dtcp->proxy_node, &proxy->proxy_pin_list);

	return dtcp;
}
EXPORT_SYMBOL(dtcon_proxy_pin_request);

int dtcon_proxy_pin_release(struct dtcon_proxy *proxy, struct dtcon_pin *dtcp)
{
	if (!proxy || !dtcp || dtcp->proxy != proxy)
		return -EINVAL;

	list_del(&dtcp->proxy_node);
	dtcp->proxy = NULL;
	dtcp->data = NULL;

	/* proxy device data release */
	of_node_put(dtcp->param);
	of_node_put(dtcp->pinctrl);
	dtcp->param = NULL;
	dtcp->pinctrl = NULL;
	return 0;
}
EXPORT_SYMBOL(dtcon_proxy_pin_release);

/* dtcon function methods */
struct dtcon_function *dtcon_function_lookup(struct dtcon_data *dtcd,
		const char *kind)
{
	struct dtcon_function *dtcf;

	/* lookup function by kind */
	list_for_each_entry(dtcf, &dtcd->function_list, node) {
		if (strcmp(dtcf->kind, kind) == 0)
			return dtcf;
	}

	return NULL;
}
EXPORT_SYMBOL(dtcon_function_lookup);

/* proxy device methods */
struct dtcon_proxy *dtcon_proxy_create(struct platform_device *pdev,
		const char *kind,
		int (*func_init)(struct dtcon_function *dtcf))
{
	struct device *dev = &pdev->dev;
	struct dtcon_data *dtcd;
	struct dtcon_proxy *proxy;
	struct dtcon_function *dtcf;
	int err;

	/* get connector data from a parent */
	dtcd = dtcon_data_from_platform_device(pdev);
	if (!dtcd) {
		err = -ENOMEM;
		goto err_out;
	}

	proxy = kzalloc(sizeof(*proxy), GFP_KERNEL);
	if (!proxy) {
		dev_err(dev, "Failed to allocate proxy data\n");
		err = -ENOMEM;
		goto err_out;
	}

	/* lookup function */
	dtcf = dtcon_function_lookup(dtcd, kind);
	if (dtcf == NULL) {
		dtcf = kzalloc(sizeof(*dtcf), GFP_KERNEL);
		if (!dtcf) {
			dev_err(dev, "Failed to allocate proxy data\n");
			err = -ENOMEM;
			goto err_free_proxy;
		}
		dtcf->dtcd = dtcd;
		dtcf->kind = kind;
		dtcf->np = of_get_child_by_name(dtcd->functions, kind);
		INIT_LIST_HEAD(&dtcf->proxy_list);

		if (func_init) {
			err = (*func_init)(dtcf);
			if (err) {
				dev_err(dev, "function init failed\n");
				kfree(dtcf);
				goto err_free_proxy;
			}
		}
		list_add_tail(&dtcf->node, &dtcd->function_list);
	}

	proxy->pdev = pdev;
	proxy->dtcf = dtcf;

	INIT_LIST_HEAD(&proxy->proxy_pin_list);

	list_add_tail(&proxy->node, &dtcf->proxy_list);

	return proxy;

err_free_proxy:
	kfree(proxy);
err_out:
	return ERR_PTR(err);
}
EXPORT_SYMBOL(dtcon_proxy_create);

void dtcon_proxy_destroy(struct dtcon_proxy *proxy,
		void (*func_fini)(struct dtcon_function *dtcf))
{
	struct dtcon_data *dtcd;
	struct dtcon_function *dtcf;
	struct dtcon_pin *dtcp, *dtcpn;

	if (!proxy || !proxy->dtcf)
		return;

	dtcf = proxy->dtcf;
	dtcd = dtcf->dtcd;

	list_del(&proxy->node);

	/* last proxy; remove function */
	if (list_empty(&dtcf->proxy_list)) {
		list_del(&dtcf->node);
		if (func_fini)
			(*func_fini)(dtcf);
		of_node_put(dtcf->np);
		kfree(dtcf);
	}

	/* release all pins of this proxy */
	list_for_each_entry_safe_reverse(dtcp, dtcpn,
			&proxy->proxy_pin_list, proxy_node)
		dtcon_proxy_pin_release(proxy, dtcp);

	kfree(proxy);
}
EXPORT_SYMBOL(dtcon_proxy_destroy);

MODULE_AUTHOR("Pantelis Antoniou <pantelis.antoniou@konsulko.com>");
MODULE_DESCRIPTION("DT connector extcon driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:extcon-dt-con");
