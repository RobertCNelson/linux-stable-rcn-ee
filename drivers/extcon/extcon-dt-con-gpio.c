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
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include "extcon-dt-con.h"

/* format: <gpio-phandle> <gpio-nr> <connector-pin-phandle> */
#define GPIO_CELLS_NUM	3

struct dtcon_gpio_func_data {
	struct dtcon_function *function;
	int gpio_base;
	int gpio_cells;
};

/* indexed by gpio number */
struct dtcon_gpio_pin_data {
	struct dtcon_pin *dtcp;
	struct device_node *chip_np;
	struct gpio_chip *chip;	/* when requested */
	int hwnum;
	char *label;
};

struct dtcon_gpio_data {
	struct platform_device *pdev;
	struct gpio_chip chip;
	struct dtcon_proxy *proxy;
	struct dtcon_gpio_pin_data *pin_data;
};
#define to_dtcon_gpio_data(x) container_of((x), struct dtcon_gpio_data, chip)

/* match when we got the same device node */
static int dtcon_find_gpiochip(struct gpio_chip *gc, void *data)
{
	return gc->of_node == data;
}

static struct gpio_chip *dtcon_gpio_get_gpiochip(struct dtcon_gpio_data *dtcg,
		unsigned offset)
{
	struct dtcon_gpio_pin_data *pin_data;
	struct dtcon_pin *dtcp;

	if (offset >= dtcg->chip.ngpio)
		return NULL;

	pin_data = &dtcg->pin_data[offset];
	dtcp = pin_data->dtcp;

	return gpiochip_find(pin_data->chip_np, dtcon_find_gpiochip);
}

/* methods */

static int dtcon_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	if (offset >= dtcg->chip.ngpio)
		return -EINVAL;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	err = gc->request(gc, pin_data->hwnum);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "%s(%u) - %d failed\n", __func__, offset, err);
		return err;
	}

	dev_dbg(dev, "%s(%u) -> %d\n", __func__, offset, err);

	return err;
}

static void dtcon_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;

	if (offset >= dtcg->chip.ngpio)
		return;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;

	if (gc->free)
		gc->free(gc, pin_data->hwnum);

	dev_dbg(dev, "%s(%u)\n", __func__, offset);

}

static int dtcon_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	/* if we're still in probe? return input */
	if (platform_get_drvdata(dtcg->pdev) != dtcg)
		return 1;

	BUG_ON(!dtcg);
	if (offset >= dtcg->chip.ngpio)
		return -EINVAL;
	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	if (!gc->get_direction)
		return -EINVAL;

	err = gc->get_direction(gc, pin_data->hwnum);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "%s(%u) - %d failed\n", __func__, offset, err);
		return err;
	}

	dev_dbg(dev, "%s(%u) -> %d\n", __func__, offset, err);

	return err;
}

static int dtcon_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	if (offset >= dtcg->chip.ngpio)
		return -EINVAL;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	if (!gc->direction_input)
		return -EINVAL;

	err = gc->direction_input(gc, pin_data->hwnum);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "%s(%u) - %d failed\n", __func__, offset, err);
		return err;
	}

	dev_dbg(dev, "%s(%u) -> %d\n", __func__, offset, err);

	return err;
}

static int dtcon_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	if (offset >= dtcg->chip.ngpio)
		return -EINVAL;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	if (!gc->get)
		return -EINVAL;

	err = gc->get(gc, pin_data->hwnum);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "%s(%u) - %d failed\n", __func__, offset, err);
		return err;
	}

	dev_dbg(dev, "%s(%u) -> %d\n", __func__, offset, err);

	return err;
}

static int dtcon_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
		int value)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	if (offset >= dtcg->chip.ngpio)
		return -EINVAL;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	if (!gc->direction_output)
		return -EINVAL;

	err = gc->direction_output(gc, pin_data->hwnum, value);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "%s(%u) - %d failed\n", __func__, offset, err);
		return err;
	}

	dev_dbg(dev, "%s(%u, %d) -> %d\n", __func__, offset, value, err);

	return err;
}

static int dtcon_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
		unsigned debounce)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	if (offset >= dtcg->chip.ngpio)
		return -EINVAL;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	if (!gc->set_debounce)
		return -EINVAL;

	err = gc->set_debounce(gc, pin_data->hwnum, debounce);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "%s(%u) - %d failed\n", __func__, offset, err);
		return err;
	}

	dev_dbg(dev, "%s(%u, %d) -> %d\n", __func__, offset, debounce, err);

	return err;
}

static void dtcon_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct dtcon_gpio_data *dtcg = to_dtcon_gpio_data(chip);
	struct device *dev = &dtcg->pdev->dev;
	struct dtcon_gpio_pin_data *pin_data;
	struct gpio_chip *gc;
	int err;

	if (offset >= dtcg->chip.ngpio)
		return;

	pin_data = &dtcg->pin_data[offset];
	gc = pin_data->chip;
	if (!gc->set)
		return;

	gc->set(gc, pin_data->hwnum, value);

	dev_dbg(dev, "%s(%u, %d) -> %d\n", __func__, offset, value, err);
}

/* gpio function methods */
static int dtcon_gpio_function_init(struct dtcon_function *dtcf)
{
	struct dtcon_data *dtcd = dtcf->dtcd;
	struct device *dev = &dtcd->pdev->dev;
	struct dtcon_gpio_func_data *dtcgf = NULL;
	int err;
	u32 num;

	if (!dtcf->np) {
		dev_err(dev, "No gpio function configuration node\n");
		return -EINVAL;
	}

	dtcgf = kzalloc(sizeof(*dtcgf), GFP_KERNEL);
	if (!dtcgf) {
		dev_err(dev, "No memory for function data\n");
		return -ENOMEM;
	}
	dtcf->data = dtcgf;

	err = of_property_read_u32(dtcf->np, "gpio-base", &num);
	if (err) {
		dev_err(dev, "No gpio-base in function configuration\n");
		err = -EINVAL;
		goto err_out;
	}

	dtcgf->gpio_base = num;

	/* format: <gpio-phandle> <gpio-nr> <connector-pin-phandle> */
	dtcgf->gpio_cells = GPIO_CELLS_NUM;

	return 0;

err_out:
	kfree(dtcgf);
	return err;
}

static void dtcon_gpio_function_fini(struct dtcon_function *dtcf)
{
	struct dtcon_gpio_func_data *dtcgf = dtcf->data;

	kfree(dtcgf);
}

static int dtcon_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct dtcon_gpio_data *dtcg;
	struct dtcon_data *dtcd;
	struct dtcon_function *dtcf;
	struct dtcon_gpio_func_data *dtcgf;
	struct dtcon_proxy *proxy;
	struct dtcon_pin *dtcp;
	struct dtcon_gpio_pin_data *pin_data;
	struct of_phandle_args args;
	const void *pin_regs;
	int i, err, count, regsz, index;

	if (!np) {
		dev_err(dev, "No OF configuration node\n");
		return -EINVAL;
	}

	count = of_property_count_elems_of_size(np, "pin-list", sizeof(u32));
	pin_regs = of_get_property(np, "pin-list", NULL);
	if (count <= 0 || !pin_regs) {
		dev_err(dev, "Invalid configuration\n");
		return -EINVAL;
	}

	dtcg = devm_kzalloc(dev, sizeof(*dtcg), GFP_KERNEL);
	if (!dtcg) {
		dev_err(dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}
	dtcg->pdev = pdev;

	proxy = dtcon_proxy_create(pdev, "gpio", dtcon_gpio_function_init);
	if (IS_ERR(proxy))
		return PTR_ERR(proxy);

	proxy->data = dtcg;
	dtcg->proxy = proxy;

	dtcf = dtcg->proxy->dtcf;
	dtcgf = dtcf->data;
	dtcd = dtcf->dtcd;

	/* the count must be a multiple of address cell numbers */
	if ((count % dtcd->connector_address_cells) != 0) {
		dev_err(dev, "Bad pin-list \n");
		err = -EINVAL;
		goto out_destroy_proxy;
	}

	count /= dtcd->connector_address_cells;

	dtcg->pin_data = devm_kzalloc(dev, sizeof(*dtcg->pin_data) * count,
			GFP_KERNEL);
	if (!dtcg->pin_data) {
		dev_err(dev, "Failed to allocate pin data\n");
		err = -ENOMEM;
		goto out_destroy_proxy;
	}

	dtcg->chip.request = dtcon_gpio_request;
	dtcg->chip.free = dtcon_gpio_free;
	dtcg->chip.get_direction = dtcon_gpio_get_direction;
	dtcg->chip.direction_input = dtcon_gpio_direction_input;
	dtcg->chip.get = dtcon_gpio_get;
	dtcg->chip.direction_output = dtcon_gpio_direction_output;
	dtcg->chip.set_debounce = dtcon_gpio_set_debounce;
	dtcg->chip.set = dtcon_gpio_set;
	dtcg->chip.label = np->name;
	dtcg->chip.base = dtcgf->gpio_base;
	dtcg->chip.ngpio = count;
	dtcg->chip.of_node = of_node_get(np);

	/* request all pins */
	regsz = sizeof(u32) * dtcd->connector_address_cells;
	for (i = 0; i < count; i++, pin_regs += regsz) {

		pin_data = &dtcg->pin_data[i];

		dtcp = dtcon_proxy_pin_request(proxy, pin_regs, regsz, 0);
		if (IS_ERR(dtcp)) {
			dev_err(dev, "could not request gpio #%d\n", i);
			err = PTR_ERR(dtcp);
			goto out_release_pin;
		}

		pin_data->dtcp = dtcp;

		err = of_parse_phandle_with_fixed_args(dtcp->np, "gpio", 1, 0, &args);
		if (err) {
			dev_err(dev, "could not #%d parse gpio property\n", i);
			goto out_release_pin;
		}
		pin_data->chip_np = args.np;
		pin_data->hwnum = args.args[0];

		pin_data->chip = dtcon_gpio_get_gpiochip(dtcg, i);
		if (!pin_data->chip) {
			dev_err(dev, "gpio #%d index #%d not found\n",
					i, index);
			/* the other device might not come up yet */
			err = -EPROBE_DEFER;
			goto out_release_pin;
		}
		pin_data->label = devm_kasprintf(dev, GFP_KERNEL,
				"%s:%d", np->name, i);
		if (!pin_data->label) {
			dev_err(dev, "gpio #%d index #%d label alloc\n",
					i, index);
			/* the other device might not come up yet */
			err = -ENOMEM;
			goto out_release_pin;
		}

		dev_err(dev, "gpio #%d -> %-8s @ %s\n",
				i, dtcp->regstr, dtcp->np->name);
	}

	err = gpiochip_add_data(&dtcg->chip, dtcg);
	if (err) {
		dev_err(dev, "Could not register gpio chip %d\n", err);
		goto out_release_pin;
	}

	/* advance base */
	dtcgf->gpio_base += count;

	platform_set_drvdata(pdev, dtcg);

	return 0;

out_release_pin:
	while (i-- >= 0) {
		pin_data = &dtcg->pin_data[i];
		of_node_put(pin_data->chip_np);
		dtcon_proxy_pin_release(proxy, pin_data->dtcp);
	}

out_destroy_proxy:
	dtcon_proxy_destroy(dtcg->proxy, dtcon_gpio_function_fini);
	return err;
}

static int dtcon_gpio_remove(struct platform_device *pdev)
{
	struct dtcon_gpio_data *dtcg = platform_get_drvdata(pdev);
	struct dtcon_proxy *proxy = dtcg->proxy;
	struct dtcon_gpio_pin_data *pin_data;
	int i;

	for (i = 0; i < dtcg->chip.ngpio; i++) {
		pin_data = &dtcg->pin_data[i];
		of_node_put(pin_data->chip_np);
		dtcon_proxy_pin_release(proxy, pin_data->dtcp);
	}

	gpiochip_remove(&dtcg->chip);
	dtcon_proxy_destroy(dtcg->proxy, dtcon_gpio_function_fini);
	return 0;
}

static const struct of_device_id dtcon_gpio_of_match[] = {
	{ .compatible = "dtcon-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, dtcon_gpio_of_match);

static struct platform_driver dtcon_gpio = {
	.probe = dtcon_gpio_probe,
	.remove = dtcon_gpio_remove,
	.driver = {
		.name = "dtcon-gpio",
		.owner = THIS_MODULE,
		.of_match_table = dtcon_gpio_of_match,
	}
};

module_platform_driver(dtcon_gpio);
MODULE_AUTHOR("Pantelis Antoniou <pantelis.antoniou@konsulko.com>");
MODULE_LICENSE("GPL");
