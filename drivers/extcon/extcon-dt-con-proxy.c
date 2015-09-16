/*
 * Copyright (C) 2016 Konsulko Group
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

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>

#include "extcon-dt-con.h"

static const struct of_device_id dtcon_proxy_of_match[] = {
	{ .compatible = "dtcon-uart", .data = "uart" },
	{ .compatible = "dtcon-i2c",  .data = "i2c"  },
	{ .compatible = "dtcon-spi",  .data = "spi"  },
	{},
};
MODULE_DEVICE_TABLE(of, dtcon_proxy_of_match);

struct dtcon_proxy_data {
	struct platform_device *pdev;
	struct dtcon_proxy *proxy;
	struct of_changeset ocs;
	struct device_node *paramsnp;
	int param_cells;
};

static int dtcon_proxy_parse_params(struct dtcon_proxy_data *dtcpd)
{
	struct device *dev = &dtcpd->pdev->dev;
	struct dtcon_proxy *proxy = dtcpd->proxy;
	struct dtcon_function *dtcf = proxy->dtcf;
	struct device_node *np;
	struct dtcon_pin *dtcp;
	const void *value;
	int err, len;
	u32 num;

	dtcpd->paramsnp = of_get_child_by_name(dtcf->np, "params");

	/* no parameters? no problem */
	if (!dtcpd->paramsnp)
		return -EINVAL;

	/* get number of param cells */
	if (of_property_read_u32(dtcpd->paramsnp, "#param-cells", &num))
		num = 2;

	dtcpd->param_cells = num;

	/* iterate over list of parameters */
	err = 0;
	for_each_child_of_node(dtcpd->paramsnp, np) {
		value = of_get_property(dev->of_node, np->name, &len);
		if (!value) {
			/* ignore non-required parameter */
			if (!of_property_read_bool(np, "required"))
				continue;
			/* missing required parameter */
			dev_err(dev, "%s: Missing required parameter %s\n",
					dtcf->kind, np->name);
err_out:
			of_node_put(np);
			err = -EINVAL;
			goto out;
		}

		/* we have a parameter; instantiate it */
		dev_dbg(dev, "%s: param %s\n", dtcf->kind,
				np->name);

		/* connector-pin parameter <connector reg address> */
		if (of_property_read_bool(np, "connector-pin")) {
			dtcp = dtcon_proxy_pin_request(proxy, value, len, 0);
			if (!dtcp) {
				dev_err(dev, "%s: failed to request %s's pin\n",
						dtcf->kind, np->name);
				goto err_out;
			}
			/* no need to keep dtcp around; it's on proxy's list */
			dev_info(dev, "%s: %s pin is @%s\n", dtcf->kind,
					np->name, dtcp->np->name);

			dtcp->param = of_node_get(np);
			/* need to iterate over the mux options for matches */
		}
	}
out:
	return err;
}

static int dtcon_proxy_grade_single(struct dtcon_proxy_data *dtcpd,
		struct device_node *fnp, struct device_node *devnp,
		struct device_node *muxnp)
{
	struct device *dev = &dtcpd->pdev->dev;
	struct dtcon_pin *dtcp;
	struct dtcon_proxy *proxy = dtcpd->proxy;
	int ret, paramsz, paramcells, i, count, len, match, score;
	const char *param, *gpio_prop;
	const __be32 *paramp;
	bool avail;
	u32 ph;

	if (!devnp)
		return 0;

	paramcells = dtcpd->param_cells;
	paramsz = paramcells * sizeof(u32);

	score = 0;
	list_for_each_entry(dtcp, &proxy->proxy_pin_list, proxy_node) {

		param = dtcp->param->name;
		paramp = of_get_property(muxnp, param, &len);

		/* no such parameter? */
		if (!paramp) {
			/* GPIO parameters only on disabled devices */
			if (of_device_is_available(devnp)) {
				dev_err(dev, "param %s with enabled device\n", param);
				score = 0;
				break;
			}

			ret = of_property_read_string(dtcp->param,
					"gpio-property", &gpio_prop);
			if (ret) {
				dev_err(dev, "no param %s\n", param);
				score = 0;
				break;
			}
			dtcp->proxy_gpio = true;
			/* gpio matches have a low score */
			score++;
			continue;
		}

		/* not-there? malformed? bail out */
		if ((len % paramsz))  {
			dev_err(dev, "malformed param %s\n", param);
			score = 0;
			break;
		}

		count = len / paramsz;

		match = 0;
		for (i = 0; i < count; i++, paramp += paramcells) {
			/* get phandle of connector pin */
			ph = be32_to_cpu(paramp[0]);
			if (ph != dtcp->np->phandle)
				continue;

			avail = of_device_is_available(devnp);
			/* when not preeneabled
			* -> device must be disabled
			* when preenabled
			* -> device must be enabled
			*/
			if (of_property_read_bool(muxnp, "pre-enabled"))
				match = avail;
			else
				match = !avail;

			if (match) {
				dtcp->proxy_gpio = false;
				/* pinmux changes only on disabled devices */
				if (!avail) {
					ph = be32_to_cpu(paramp[1]);
					dtcp->match_mux = ph;
				}
				score += (1 << 16);
				break;
			}
		}

		if (!match)
			return 0;
	}

	return score;
}

static int dtcon_proxy_instantiate(struct dtcon_proxy_data *dtcpd)
{
	struct device *dev = &dtcpd->pdev->dev;
	struct dtcon_proxy *proxy = dtcpd->proxy;
	struct dtcon_function *dtcf = proxy->dtcf;
	struct device_node *np, *fnp, *devnp, *muxnp, *best_devnp;
	struct dtcon_pin *dtcp;
	int err, len, count, score, best_score, arg_len;
	const void *value;
	const char *gpio_prop;
	const void *arg_value;
	void *data;
	__be32 *muxp;

	err = 0;

	/* iterate over the device targets */
	best_score = 0;
	best_devnp = NULL;
	for_each_child_of_node(dtcf->np, fnp) {
		devnp = of_parse_phandle(fnp, "device", 0);
		if (!devnp)
			continue;
		for_each_child_of_node(fnp, muxnp) {

			score = dtcon_proxy_grade_single(dtcpd, fnp,
							 devnp, muxnp);

			if (score <= best_score)
				continue;

			/* new best score */
			best_score = score;
			of_node_put(best_devnp);
			best_devnp = of_node_get(devnp);
			list_for_each_entry(dtcp, &proxy->proxy_pin_list,
					    proxy_node) {

				/* if it's a GPIO do not select it */
				if (dtcp->proxy_gpio)
					continue;

				of_node_put(dtcp->pinctrl);
				dtcp->pinctrl = of_find_node_by_phandle(
						dtcp->match_mux);
				dtcp->match_mux = 0;
			}
		}
		of_node_put(devnp);
	}

	if (best_score == 0) {
		dev_info(dev, "no matching proxy config\n");
		return -ENODEV;
	}

	devnp = best_devnp;
	best_devnp = NULL;

	dev_dbg(dev, "matches device %s\n", devnp->full_name);

	/* need to copy subdevices? */
	if (of_property_read_bool(dtcpd->paramsnp, "copy-subdevices")) {
		for_each_available_child_of_node(dev->of_node, np) {
			err = of_changeset_node_move(&dtcpd->ocs, np, devnp);
			if (err) {
				dev_err(dev, "Failed to move nodes\n");
				of_node_put(np);
				goto err_out;
			}
		}
	}

	/* changes only allowed to non-activated devices */
	if (!of_device_is_available(devnp)) {

		/* iterate over parameters */

		/* iterate over list of parameters */
		for_each_child_of_node(dtcpd->paramsnp, np) {
			if (!of_property_read_bool(np, "copy"))
				continue;

			value = of_get_property(dev->of_node, np->name, &len);
			if (!value) {
				if (!of_property_read_bool(np, "required"))
					continue;
				dev_err(dev, "Missing required property %s\n",
						np->name);
				of_node_put(np);
				err = -ENODEV;
				goto err_out;
			}

			of_changeset_update_property_copy(&dtcpd->ocs, devnp,
				np->name, value, len);

		}

		/* generate pinctrl for each pin of the connector */
		if (of_property_read_bool(dtcpd->paramsnp,
					"generate-pinctrl")) {
			/* count the pinctrl entries we're going to need */
			count = 0;
			list_for_each_entry(dtcp, &proxy->proxy_pin_list,
					proxy_node) {
				if (dtcp->pinctrl)
					count++;
			}

			if (count > 0) {
				muxp = kzalloc(sizeof(u32) * count, GFP_KERNEL);
				if (!muxp) {
					dev_err(dev,
						"Failed to allocate pinmux\n");
					err = -ENOMEM;
					goto err_out;
				}

				count = 0;
				list_for_each_entry(dtcp,
						&proxy->proxy_pin_list,
						proxy_node) {
					if (!dtcp->pinctrl)
						continue;
					muxp[count++] = cpu_to_be32(
							dtcp->pinctrl->phandle);
				}

				of_changeset_update_property_string(&dtcpd->ocs,
						devnp, "pinctrl-names",
						"default");
				of_changeset_update_property_copy(&dtcpd->ocs,
						devnp, "pinctrl-0", muxp,
						sizeof(u32) * count);

				kfree(muxp);
			}
		}

		/* fill in gpio properties if required */
		list_for_each_entry(dtcp, &proxy->proxy_pin_list, proxy_node) {
			if (!dtcp->proxy_gpio)
				continue;
			err = of_property_read_string(dtcp->param,
					"gpio-property", &gpio_prop);
			if (err) {
				dev_err(dev,
					"Failed to read gpio-property name %s\n",
					dtcp->param->name);
				goto err_out;
			}
			value = of_get_property(dtcp->np, "gpio", &len);
			if (!value) {
				dev_err(dev,
					"Failed on gpio name %s - %s\n",
					dtcp->param->name, dtcp->np->name);
				err = -EINVAL;
				goto err_out;
			}
			arg_value = of_get_property(dtcp->param,
					"gpio-args", &arg_len);
			if (!arg_value) {
				dev_err(dev,
					"Failed on gpio-args name %s - %s\n",
					dtcp->param->name, dtcp->param->name);
				err = -EINVAL;
				goto err_out;
			}
			data = kmalloc(len + arg_len, GFP_KERNEL);
			if (!data) {
				dev_err(dev,
					"Failed to allocate %s - %s\n",
					dtcp->param->name, dtcp->param->name);
				err = -ENOMEM;
				goto err_out;
			}
			memcpy(data, value, len);
			memcpy(data + len, arg_value, arg_len);
			of_changeset_add_property_copy(&dtcpd->ocs, devnp,
				gpio_prop, data, len + arg_len);
			kfree(data);
		}

		/* enabled target device and go */
		of_changeset_update_property_string(&dtcpd->ocs, devnp,
				"status", "okay");
	}

	of_changeset_apply(&dtcpd->ocs);

	of_node_put(devnp);
	of_node_put(muxnp);

	return 0;

err_out:
	of_changeset_destroy(&dtcpd->ocs);

	of_node_put(devnp);

	return err;
}

static int dtcon_proxy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dtcon_proxy *proxy;
	struct dtcon_function *dtcf;
	struct dtcon_proxy_data *dtcpd;
	const struct of_device_id *of_id;
	const char *function;
	int err;

	of_id = of_match_device(dtcon_proxy_of_match, dev);
	if (!of_id) {
		dev_err(dev, "Could not match device\n");
		return -ENODEV;
	}
	function = of_id->data;

	dtcpd = devm_kzalloc(dev, sizeof(*dtcpd), GFP_KERNEL);
	if (!dtcpd) {
		dev_err(dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}
	dtcpd->pdev = pdev;
	of_changeset_init(&dtcpd->ocs);

	proxy = dtcon_proxy_create(pdev, function, NULL);
	if (IS_ERR(proxy))
		return PTR_ERR(proxy);

	dtcpd->proxy = proxy;

	dtcf = proxy->dtcf;

	platform_set_drvdata(pdev, dtcpd);

	err = dtcon_proxy_parse_params(dtcpd);
	if (err) {
		dev_err(dev, "Failed to parse params\n");
		return err;
	}

	err = dtcon_proxy_instantiate(dtcpd);
	if (err) {
		dev_err(dev, "Failed to instantiate device\n");
		return err;
	}

	dev_info(dev, "OK\n");

	return 0;
}

static int dtcon_proxy_remove(struct platform_device *pdev)
{
	struct dtcon_proxy_data *dtcpd = platform_get_drvdata(pdev);

	of_changeset_revert(&dtcpd->ocs);
	of_changeset_destroy(&dtcpd->ocs);
	dtcon_proxy_destroy(dtcpd->proxy, NULL);
	of_node_put(dtcpd->paramsnp);
	return 0;
}

static struct platform_driver dtcon_proxy = {
	.probe = dtcon_proxy_probe,
	.remove = dtcon_proxy_remove,
	.driver = {
		.name = "dtcon-proxy",
		.owner = THIS_MODULE,
		.of_match_table = dtcon_proxy_of_match,
	}
};

module_platform_driver(dtcon_proxy);
MODULE_AUTHOR("Pantelis Antoniou <pantelis.antoniou@konsulko.com>");
MODULE_LICENSE("GPL");
