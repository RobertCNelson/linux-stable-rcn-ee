/*
 * Copyright (C) 2021 Samin Guo <samin.guo@starfivetech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <asm-generic/io.h>
#include <linux/regmap.h>

#define SFCTEMP_Y100  23750L
#define SFCTEMP_Z100 409400L
#define SFCTEMP_K100   8110L

union sfctemp_reg {
	u32 v;
	struct {
		/* TempSensor reset. The RSTN can be de-asserted once the
		 * analog core has powered up. Trst(min 100ns)
		 * 0:reset  1:de-assert */
		u32 rstn   : 1;
		/* TempSensor analog core power down. The analog core will be
		 * power up when PD de-asserted after Tpu(min 50us) has
		 * elapsed. the RSTN should be held low until the analog core
		 * is powered up.
		 * 0:power up  1:power down */
		u32 pd     : 1;
		/* TempSensor start conversion enable.
		 * 0:disable  1:enable */
		u32 run    : 1;
		u32 rsvd_0 : 1;
		/* TempSensor calibration mode enable.
		 * 0:disable  1:enable */
		u32 cal    : 1;
		/* TempSensor signature enable. Generate a toggle value
		 * outputting on DOUT for test purpose.
		 * 0:disable  1:enable */
		u32 sgn    : 1;
		u32 rsvd_1 : 6;
		/* TempSensor test access control.
		 * 0000:normal 0001:Test1  0010:Test2  0011:Test3
		 * 0100:Test4  1000:Test8  1001:Test9 */
		u32 tm     : 4;
		/* TempSensor conversion value output.
		 * Temp(c)=DOUT/4094*Y-K */
		u32 dout   : 12;
		u32 rsvd_2 : 3;
		/* TempSensor digital test output. */
		u32 digo   : 1;
	} bits;
};

struct sfctemp {
	void __iomem *regs;
	u32 dout;
	bool enabled;
};

static irqreturn_t sfctemp_isr(int irq, void *data)
{
	struct sfctemp *sfctemp = data;
	union sfctemp_reg reg;

	reg.v = readl(sfctemp->regs);
	sfctemp->dout = reg.bits.dout;
	return IRQ_HANDLED;
}

static void sfctemp_power_up(struct sfctemp *sfctemp)
{
	union sfctemp_reg init;

	init.v = 0;
	init.bits.pd = 1;
	writel(init.v, sfctemp->regs);
	udelay(1);

	init.bits.pd = 0;
	writel(init.v, sfctemp->regs);
	/* wait t_pu(50us) + t_rst(100ns) */
	udelay(60);

	init.bits.rstn = 1;
	writel(init.v, sfctemp->regs);
	/* wait t_su(500ps) */
	udelay(1);

	init.bits.run = 1;
	writel(init.v, sfctemp->regs);
	/* wait 1st sample (8192 temp_sense clk: ~2MHz) */
	mdelay(10);
}

static void sfctemp_power_down(struct sfctemp *sfctemp)
{
	union sfctemp_reg init;

	init.v = readl(sfctemp->regs);
	init.bits.run = 0;
	writel(init.v, sfctemp->regs);
	udelay(1);

	init.bits.pd   = 1;
	init.bits.rstn = 0;
	writel(init.v, sfctemp->regs);
	udelay(1);
}

static int sfctemp_enable(struct sfctemp *sfctemp)
{
	if (sfctemp->enabled)
		return 0;

	sfctemp_power_up(sfctemp);
	sfctemp->enabled = true;
	return 0;
}

static int sfctemp_disable(struct sfctemp *sfctemp)
{
	if (!sfctemp->enabled)
		return 0;

	sfctemp_power_down(sfctemp);
	sfctemp->enabled = false;
	return 0;
}

static umode_t sfctemp_is_visible(const void *data, enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			return 0644;
		case hwmon_temp_input:
			return 0444;
		}
		return 0;
	default:
		return 0;
	}
}

static int sfctemp_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct sfctemp *sfctemp = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			*val = sfctemp->enabled;
			return 0;
		case hwmon_temp_input:
			if (!sfctemp->enabled)
				return -ENODATA;
			/* calculate temperature in milli Celcius */
			*val  = (1000 * SFCTEMP_Y100 * (long)sfctemp->dout) / SFCTEMP_Z100
				- 10 * SFCTEMP_K100;
			return 0;
		}
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static int sfctemp_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	struct sfctemp *sfctemp = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			if (val == 0)
				return sfctemp_disable(sfctemp);
			if (val == 1)
				return sfctemp_enable(sfctemp);
			break;
		}
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static const struct hwmon_channel_info *sfctemp_info[] = {
	HWMON_CHANNEL_INFO(chip, HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp, HWMON_T_ENABLE | HWMON_T_INPUT),
	NULL
};

static const struct hwmon_ops sfctemp_hwmon_ops = {
	.is_visible = sfctemp_is_visible,
	.read = sfctemp_read,
	.write = sfctemp_write,
};

static const struct hwmon_chip_info sfctemp_chip_info = {
	.ops = &sfctemp_hwmon_ops,
	.info = sfctemp_info,
};

static int sfctemp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *hwmon_dev;
	struct resource *mem;
	struct sfctemp *sfctemp;
	int ret;

	sfctemp = devm_kzalloc(dev, sizeof(*sfctemp), GFP_KERNEL);
	if (!sfctemp)
		return -ENOMEM;

	dev_set_drvdata(dev, sfctemp);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sfctemp->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(sfctemp->regs))
		return PTR_ERR(sfctemp->regs);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		return ret;

	ret = devm_request_irq(dev, ret, sfctemp_isr,
			       IRQF_SHARED, pdev->name, sfctemp);
	if (ret) {
		dev_err(dev, "request irq failed: %d\n", ret);
		return ret;
	}

	ret = sfctemp_enable(sfctemp);
	if (ret)
		return ret;

	hwmon_dev = hwmon_device_register_with_info(dev, pdev->name, sfctemp,
						    &sfctemp_chip_info, NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(dev, "%s: sensor '%s'\n", dev_name(hwmon_dev), pdev->name);
	return 0;
}

static int sfctemp_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sfctemp *sfctemp = dev_get_drvdata(dev);

	hwmon_device_unregister(dev);
	return sfctemp_disable(sfctemp);
}

static const struct of_device_id sfctemp_of_match[] = {
	{ .compatible = "sfc,tempsensor" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, sfctemp_of_match);

static struct platform_driver sfctemp_driver = {
	.driver = {
		.name = "sfctemp",
		.of_match_table = of_match_ptr(sfctemp_of_match),
	},
	.probe  = sfctemp_probe,
	.remove = sfctemp_remove,
};
module_platform_driver(sfctemp_driver);

MODULE_AUTHOR("Samin Guo");
MODULE_DESCRIPTION("Starfive JH7100 temperature sensor driver");
MODULE_LICENSE("GPL");
