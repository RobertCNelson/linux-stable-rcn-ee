// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Emil Renner Berthing <kernel@esmil.dk>
 * Copyright (C) 2021 Samin Guo <samin.guo@starfivetech.com>
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>

/*
 * TempSensor reset. The RSTN can be de-asserted once the analog core has
 * powered up. Trst(min 100ns)
 * 0:reset  1:de-assert
 */
#define SFCTEMP_RSTN	BIT(0)

/*
 * TempSensor analog core power down. The analog core will be powered up
 * Tpu(min 50us) after PD is de-asserted. RSTN should be held low until the
 * analog core is powered up.
 * 0:power up  1:power down
 */
#define SFCTEMP_PD	BIT(1)

/*
 * TempSensor start conversion enable.
 * 0:disable  1:enable
 */
#define SFCTEMP_RUN	BIT(2)

/*
 * TempSensor conversion value output.
 * Temp(C)=DOUT*Y/4094 - K
 */
#define SFCTEMP_DOUT_POS	16
#define SFCTEMP_DOUT_MSK	GENMASK(27, 16)

/* DOUT to Celcius conversion constants */
#define SFCTEMP_Y1000	237500L
#define SFCTEMP_Z	4094L
#define SFCTEMP_K1000	81100L

struct sfctemp {
	/* serialize access to hardware register and enabled below */
	struct mutex lock;
	struct completion conversion_done;
	void __iomem *regs;
	bool enabled;
};

static irqreturn_t sfctemp_isr(int irq, void *data)
{
	struct sfctemp *sfctemp = data;

	complete(&sfctemp->conversion_done);
	return IRQ_HANDLED;
}

static void sfctemp_power_up(struct sfctemp *sfctemp)
{
	/* make sure we're powered down first */
	writel(SFCTEMP_PD, sfctemp->regs);
	udelay(1);

	writel(0, sfctemp->regs);
	/* wait t_pu(50us) + t_rst(100ns) */
	usleep_range(60, 200);

	/* de-assert reset */
	writel(SFCTEMP_RSTN, sfctemp->regs);
	udelay(1); /* wait t_su(500ps) */
}

static void sfctemp_power_down(struct sfctemp *sfctemp)
{
	writel(SFCTEMP_PD, sfctemp->regs);
}

static void sfctemp_run_single(struct sfctemp *sfctemp)
{
	writel(SFCTEMP_RSTN | SFCTEMP_RUN, sfctemp->regs);
	udelay(1);
	writel(SFCTEMP_RSTN, sfctemp->regs);
}

static int sfctemp_enable(struct sfctemp *sfctemp)
{
	mutex_lock(&sfctemp->lock);
	if (sfctemp->enabled)
		goto done;

	sfctemp_power_up(sfctemp);
	sfctemp->enabled = true;
done:
	mutex_unlock(&sfctemp->lock);
	return 0;
}

static int sfctemp_disable(struct sfctemp *sfctemp)
{
	mutex_lock(&sfctemp->lock);
	if (!sfctemp->enabled)
		goto done;

	sfctemp_power_down(sfctemp);
	sfctemp->enabled = false;
done:
	mutex_unlock(&sfctemp->lock);
	return 0;
}

static void sfctemp_disable_action(void *data)
{
	sfctemp_disable(data);
}

static int sfctemp_convert(struct sfctemp *sfctemp, long *val)
{
	int ret;

	mutex_lock(&sfctemp->lock);
	if (!sfctemp->enabled) {
		ret = -ENODATA;
		goto out;
	}

	sfctemp_run_single(sfctemp);

	ret = wait_for_completion_interruptible_timeout(&sfctemp->conversion_done,
							msecs_to_jiffies(10));
	if (ret <= 0) {
		if (ret == 0)
			ret = -ETIMEDOUT;
		goto out;
	}

	/* calculate temperature in milli Celcius */
	*val = (long)((readl(sfctemp->regs) & SFCTEMP_DOUT_MSK) >> SFCTEMP_DOUT_POS)
		* SFCTEMP_Y1000 / SFCTEMP_Z - SFCTEMP_K1000;

	ret = 0;
out:
	mutex_unlock(&sfctemp->lock);
	return ret;
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
			return sfctemp_convert(sfctemp, val);
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
	mutex_init(&sfctemp->lock);
	init_completion(&sfctemp->conversion_done);

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

	ret = devm_add_action(dev, sfctemp_disable_action, sfctemp);
	if (ret)
		return ret;

	ret = sfctemp_enable(sfctemp);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, pdev->name, sfctemp,
							 &sfctemp_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct of_device_id sfctemp_of_match[] = {
	{ .compatible = "starfive,jh7100-temp" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, sfctemp_of_match);

static struct platform_driver sfctemp_driver = {
	.driver = {
		.name = "sfctemp",
		.of_match_table = of_match_ptr(sfctemp_of_match),
	},
	.probe  = sfctemp_probe,
};
module_platform_driver(sfctemp_driver);

MODULE_AUTHOR("Emil Renner Berthing");
MODULE_DESCRIPTION("StarFive JH7100 temperature sensor driver");
MODULE_LICENSE("GPL");
