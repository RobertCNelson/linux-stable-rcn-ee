/*
 * pwrseq_compatible_sample.c	Sample power sequence handling for compatible
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Author: Peter Chen <peter.chen@nxp.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include <linux/power/pwrseq.h>

struct pwrseq_sample {
	struct pwrseq pwrseq;
	struct gpio_desc *gpiod_reset;
	struct clk *clks[PWRSEQ_MAX_CLKS];
	u32 duration_us;
};

#define to_generic_pwrseq(p) container_of(p, struct pwrseq_sample, pwrseq)

static void pwrseq_sample_free(struct pwrseq *pwrseq)
{
	pwrseq->used = false;
}

static void pwrseq_sample_put(struct pwrseq *pwrseq)
{
	struct pwrseq_sample *pwrseq_sam = to_generic_pwrseq(pwrseq);
	int clk;

	if (pwrseq_sam->gpiod_reset)
		gpiod_put(pwrseq_sam->gpiod_reset);

	for (clk = 0; clk < PWRSEQ_MAX_CLKS; clk++)
		clk_put(pwrseq_sam->clks[clk]);
}

static void pwrseq_sample_off(struct pwrseq *pwrseq)
{
	struct pwrseq_sample *pwrseq_sam = to_generic_pwrseq(pwrseq);
	int clk;

	for (clk = PWRSEQ_MAX_CLKS - 1; clk >= 0; clk--)
		clk_disable_unprepare(pwrseq_sam->clks[clk]);
}

static int pwrseq_sample_on(struct pwrseq *pwrseq)
{
	struct pwrseq_sample *pwrseq_sam = to_generic_pwrseq(pwrseq);
	int clk, ret = 0;
	struct gpio_desc *gpiod_reset = pwrseq_sam->gpiod_reset;

	for (clk = 0; clk < PWRSEQ_MAX_CLKS && pwrseq_sam->clks[clk]; clk++) {
		ret = clk_prepare_enable(pwrseq_sam->clks[clk]);
		if (ret) {
			pr_err("Can't enable clock, ret=%d\n", ret);
			goto err_disable_clks;
		}
	}

	if (gpiod_reset) {
		u32 duration_us = pwrseq_sam->duration_us;

		if (duration_us <= 10)
			udelay(10);
		else
			usleep_range(duration_us, duration_us + 100);
		gpiod_set_value(gpiod_reset, 0);
	}

	return ret;

err_disable_clks:
	while (--clk >= 0)
		clk_disable_unprepare(pwrseq_sam->clks[clk]);

	return ret;
}

static int pwrseq_sample_get(struct device_node *np, struct pwrseq *pwrseq)
{
	struct pwrseq_sample *pwrseq_sam = to_generic_pwrseq(pwrseq);
	enum of_gpio_flags flags;
	int reset_gpio, clk, ret = 0;

	for (clk = 0; clk < PWRSEQ_MAX_CLKS; clk++) {
		pwrseq_sam->clks[clk] = of_clk_get(np, clk);
		if (IS_ERR(pwrseq_sam->clks[clk])) {
			ret = PTR_ERR(pwrseq_sam->clks[clk]);
			if (ret != -ENOENT)
				goto err_put_clks;
			pwrseq_sam->clks[clk] = NULL;
			break;
		}
	}

	reset_gpio = of_get_named_gpio_flags(np, "reset-gpios", 0, &flags);
	if (gpio_is_valid(reset_gpio)) {
		unsigned long gpio_flags;

		if (flags & OF_GPIO_ACTIVE_LOW)
			gpio_flags = GPIOF_ACTIVE_LOW | GPIOF_OUT_INIT_LOW;
		else
			gpio_flags = GPIOF_OUT_INIT_HIGH;

		ret = gpio_request_one(reset_gpio, gpio_flags,
				"pwrseq-reset-gpios");
		if (ret)
			goto err_put_clks;

		pwrseq_sam->gpiod_reset = gpio_to_desc(reset_gpio);
		of_property_read_u32(np, "reset-duration-us",
				&pwrseq_sam->duration_us);
	} else {
		if (reset_gpio == -ENOENT)
			return 0;

		ret = reset_gpio;
		pr_err("Failed to get reset gpio on %s, err = %d\n",
				np->full_name, reset_gpio);
		goto err_put_clks;
	}

	return ret;

err_put_clks:
	while (--clk >= 0)
		clk_put(pwrseq_sam->clks[clk]);
	return ret;
}

static const struct of_device_id sample_id_table[] = {
	{ .compatible = "usb5e3,608",},
	{ .compatible = "usbb95,1708",},
	{ /* sentinel */ }
};

static int __init pwrseq_compatible_sample_register(void)
{
	struct pwrseq_sample *pwrseq_sam;
	int i;

	for (i = 0; i < CONFIG_PWRSEQ_SAMPLE_INSTANCE_NUMBER; i++) {
		pwrseq_sam = kzalloc(sizeof(*pwrseq_sam), GFP_KERNEL);
		if (!pwrseq_sam)
			return -ENOMEM;

		pwrseq_sam->pwrseq.pwrseq_of_match_table = sample_id_table;
		pwrseq_sam->pwrseq.get = pwrseq_sample_get;
		pwrseq_sam->pwrseq.on = pwrseq_sample_on;
		pwrseq_sam->pwrseq.off = pwrseq_sample_off;
		pwrseq_sam->pwrseq.put = pwrseq_sample_put;
		pwrseq_sam->pwrseq.free = pwrseq_sample_free;

		pwrseq_register(&pwrseq_sam->pwrseq);
	}

	return 0;
}
postcore_initcall(pwrseq_compatible_sample_register)
