// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip MPFS RTC driver
 *
 * Copyright (c) 2021 Microchip Corporation. All rights reserved.
 *
 * Author: Daire McNamara <daire.mcnamara@microchip.com>
 *
 */
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/rtc.h>
#include <linux/io.h>

/*
 * Device Specific Peripheral registers structures
 */
#define CONTROL_REG		0x00
#define  CONTROL_RUNNING_BIT		BIT(0)
#define  CONTROL_START_BIT		BIT(0)
#define  CONTROL_STOP_BIT		BIT(1)
#define  CONTROL_ALARM_ON_BIT		BIT(2)
#define  CONTROL_ALARM_OFF_BIT		BIT(3)
#define  CONTROL_RESET_BIT		BIT(4)
#define  CONTROL_UPLOAD_BIT		BIT(5)
#define  CONTROL_WAKEUP_CLR_BIT		BIT(8)
#define  CONTROL_WAKEUP_SET_BIT		BIT(9)
#define  CONTROL_UPDATED_BIT		BIT(10)
#define  MPFS_RTC_NUM_IRQS		2
#define MODE_REG		0x04
#define  MODE_CLOCK_BIT			BIT(0)
#define   MODE_CLOCK_CALENDAR		1
#define  MODE_WAKE_EN_BIT		BIT(1)
#define  MODE_WAKE_RESET_BIT		BIT(2)
#define  MODE_WAKE_CONTINUE_BIT		BIT(3)
#define PRESCALER_REG		0x08
#define ALARM_LOWER_REG		0x0c
#define ALARM_UPPER_REG		0x10
#define COMPARE_LOWER_REG	0x14
#define COMPARE_UPPER_REG	0x18
#define DATETIME_LOWER_REG	0x20
#define DATETIME_UPPER_REG	0x24
#define SECONDS_REG		0x30
#define MINUTES_REG		0x34
#define HOURS_REG		0x38
#define DAY_REG			0x3c
#define MONTH_REG		0x40
#define YEAR_REG		0x44
#define WEEKDAY_REG		0x48
#define WEEK_REG		0x4c

/*
 * Linux counts from 1970 but tm_year starts at 1900, MPFS RTC counts from 2000
 * Account for this difference
 */
#define YEAR_OFFSET		(100)

#define MAX_PRESCALER_COUNT	GENMASK(21, 0)
#define DEFAULT_PRESCALER	999999  /* (1Mhz / prescaler) -1 = 1Hz */

struct mpfs_rtc_dev {
	struct rtc_device *rtc;
	void __iomem *base;
	int wakeup_irq;
	u32 prescaler;
};

static void mpfs_rtc_stop(struct mpfs_rtc_dev *rtcdev)
{
	u32 ctrl;

	ctrl = readl(rtcdev->base + CONTROL_REG);
	ctrl &= ~(CONTROL_STOP_BIT | CONTROL_START_BIT);
	ctrl |= CONTROL_STOP_BIT;
	ctrl |= CONTROL_ALARM_OFF_BIT;
	writel(ctrl, rtcdev->base + CONTROL_REG);
}

static void mpfs_rtc_start(struct mpfs_rtc_dev *rtcdev)
{
	u32 ctrl;

	ctrl = readl(rtcdev->base + CONTROL_REG);
	ctrl &= ~(CONTROL_STOP_BIT | CONTROL_START_BIT);
	ctrl |= CONTROL_START_BIT;
	writel(ctrl, rtcdev->base + CONTROL_REG);
}

static void mpfs_rtc_clear_irq(struct mpfs_rtc_dev *rtcdev)
{
	u32 val = readl(rtcdev->base + CONTROL_REG);

	val &= ~(CONTROL_ALARM_ON_BIT | CONTROL_STOP_BIT);
	val |= CONTROL_ALARM_OFF_BIT;
	writel(val, rtcdev->base + CONTROL_REG);
	/*
	 * Ensure that the posted write to the CONTROL_REG register completed before
	 * returning from this function. Not doing this may result in the interrupt
	 * only being cleared some time after this function returns.
	 */
	(void)readl(rtcdev->base + CONTROL_REG);
}

static void mpfs_rtc_calendar_mode(struct mpfs_rtc_dev *rtcdev)
{
	u32 val;
	u32 rtc_running;

	val = readl(rtcdev->base + MODE_REG);
	val |= MODE_CLOCK_CALENDAR;

	rtc_running = readl(rtcdev->base + CONTROL_REG);
	if (rtc_running & CONTROL_RUNNING_BIT) {
		/* Stop the RTC in order to change the mode register content. */
		mpfs_rtc_stop(rtcdev);
		writel(val, rtcdev->base + MODE_REG);
		mpfs_rtc_start(rtcdev);
	} else {
		writel(val, rtcdev->base + MODE_REG);
	}
}

static int mpfs_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);

	tm->tm_sec = readl(rtcdev->base + SECONDS_REG);
	tm->tm_min = readl(rtcdev->base + MINUTES_REG);
	tm->tm_hour = readl(rtcdev->base + HOURS_REG);
	tm->tm_mday = readl(rtcdev->base + DAY_REG);
	tm->tm_mon   = readl(rtcdev->base + MONTH_REG) - 1;
	tm->tm_wday  = readl(rtcdev->base + WEEKDAY_REG) - 1;
	tm->tm_year  = readl(rtcdev->base + YEAR_REG) + YEAR_OFFSET;

	return 0;
}

static int mpfs_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);
	u32 val;
	u32 prog;

	writel(tm->tm_year - YEAR_OFFSET, rtcdev->base + YEAR_REG);
	writel(tm->tm_wday + 1, rtcdev->base + WEEKDAY_REG);
	writel(tm->tm_mon + 1, rtcdev->base + MONTH_REG);
	writel(tm->tm_mday, rtcdev->base + DAY_REG);
	writel(tm->tm_hour, rtcdev->base + HOURS_REG);
	writel(tm->tm_min, rtcdev->base + MINUTES_REG);
	writel(tm->tm_sec, rtcdev->base + SECONDS_REG);

	val = readl(rtcdev->base + CONTROL_REG);
	val &= ~CONTROL_STOP_BIT;
	val |= CONTROL_UPLOAD_BIT;
	writel(val, rtcdev->base + CONTROL_REG);

	do {
		prog = readl(rtcdev->base + CONTROL_REG);
		prog &= CONTROL_UPLOAD_BIT;
	} while (prog);

	mpfs_rtc_start(rtcdev);
	return 0;
}

static int mpfs_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);
	u32 mode = readl(rtcdev->base + MODE_REG);
	u32 val;

	if (mode & MODE_WAKE_EN_BIT)
		alrm->enabled = true;
	else
		alrm->enabled = false;

	val = readl(rtcdev->base + ALARM_LOWER_REG);
	alrm->time.tm_sec = (val & 0xff);
	val >>= 8;
	alrm->time.tm_min = (val & 0xff);
	val >>= 8;
	alrm->time.tm_hour = (val & 0xff);
	val >>= 8;
	alrm->time.tm_mday = (val & 0xff);

	val = readl(rtcdev->base + ALARM_UPPER_REG);
	alrm->time.tm_mon = (val & 0xff) - 1;
	val >>= 8;
	alrm->time.tm_year = (val & 0xff) + YEAR_OFFSET;
	val >>= 8;
	alrm->time.tm_wday = (val & 0xff) - 1;

	return 0;
}

static int mpfs_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);
	u32 val;
	u32 mode;
	u32 mask;
	u32 ctrl;

	/* Disable the alarm before updating */
	ctrl = readl(rtcdev->base + CONTROL_REG);
	ctrl |= CONTROL_ALARM_OFF_BIT;
	writel(ctrl, rtcdev->base + CONTROL_REG);

	/* Set alarm and compare lower registers. */
	val = alrm->time.tm_sec |
	      alrm->time.tm_min << 8 |
	      alrm->time.tm_hour << 16 |
	      alrm->time.tm_mday << 24;
	mask = 0xffffffff;

	writel(val, rtcdev->base + ALARM_LOWER_REG);
	writel(mask, rtcdev->base + COMPARE_LOWER_REG);

	val = (alrm->time.tm_mon + 1) |
	      (alrm->time.tm_year - YEAR_OFFSET) << 8;
	mask = 0x000ffff;

	writel(val, rtcdev->base + ALARM_UPPER_REG);
	writel(mask, rtcdev->base + COMPARE_UPPER_REG);

	/* Configure the RTC to enable the alarm. */
	if (alrm->enabled) {
		mode = MODE_CLOCK_CALENDAR | MODE_WAKE_EN_BIT | MODE_WAKE_CONTINUE_BIT;
		/* Enable the alarm */
		ctrl &= ~CONTROL_ALARM_OFF_BIT;
		ctrl |= CONTROL_ALARM_ON_BIT;
	}
	ctrl &= ~CONTROL_STOP_BIT;
	ctrl |= CONTROL_START_BIT;
	writel(ctrl, rtcdev->base + CONTROL_REG);
	writel(mode, rtcdev->base + MODE_REG);

	return 0;
}

static int mpfs_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);
	u32 ctrl;
	u32 mode;

	ctrl = readl(rtcdev->base + CONTROL_REG);
	mode = readl(rtcdev->base + MODE_REG);
	ctrl &= ~(CONTROL_ALARM_ON_BIT | CONTROL_ALARM_OFF_BIT | CONTROL_STOP_BIT);

	if (enabled)
		ctrl |= CONTROL_ALARM_ON_BIT;
	else
		ctrl |= CONTROL_ALARM_OFF_BIT;

	writel(ctrl, rtcdev->base + CONTROL_REG);

	return 0;
}

static void mpfs_rtc_set_prescaler(struct mpfs_rtc_dev *rtcdev, unsigned int prescaler)
{
	writel(prescaler, rtcdev->base + PRESCALER_REG);
}

static inline struct clk *mpfs_rtc_init_clk(struct device *dev)
{
	struct clk *clk;
	int ret;

	clk = devm_clk_get(dev, "rtc");
	if (IS_ERR(clk))
		return clk;

	ret = clk_prepare_enable(clk);
	if (ret)
		return ERR_PTR(ret);

	devm_add_action_or_reset(dev, (void (*) (void *))clk_disable_unprepare, clk);

	return clk;
}

static irqreturn_t mpfs_rtc_wakeup_irq_handler(int irq, void *d)
{
	struct mpfs_rtc_dev *rtcdev = d;
	unsigned long pending;

	pending = readl(rtcdev->base + CONTROL_REG);
	pending &= CONTROL_ALARM_ON_BIT;
	mpfs_rtc_clear_irq(rtcdev);

	/* its an alarm */
	rtc_update_irq(rtcdev->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops mpfs_rtc_ops = {
	.read_time		= mpfs_rtc_readtime,
	.set_time		= mpfs_rtc_settime,
	.read_alarm		= mpfs_rtc_readalarm,
	.set_alarm		= mpfs_rtc_setalarm,
	.alarm_irq_enable	= mpfs_rtc_alarm_irq_enable,
};

static void mpfs_rtc_init(struct mpfs_rtc_dev *rtcdev)
{
	u32 ctrl;

	mpfs_rtc_set_prescaler(rtcdev, rtcdev->prescaler);

	mpfs_rtc_stop(rtcdev);
	mpfs_rtc_calendar_mode(rtcdev);

	writel(0, rtcdev->base + ALARM_LOWER_REG);
	writel(0, rtcdev->base + ALARM_UPPER_REG);
	writel(0, rtcdev->base + COMPARE_LOWER_REG);
	writel(0, rtcdev->base + COMPARE_UPPER_REG);

	ctrl = readl(rtcdev->base + CONTROL_REG);
	ctrl &= ~(CONTROL_RESET_BIT | CONTROL_STOP_BIT | CONTROL_START_BIT);
	ctrl |= CONTROL_RESET_BIT | CONTROL_START_BIT;
	writel(ctrl, rtcdev->base + CONTROL_REG);
}

static int __init mpfs_rtc_probe(struct platform_device *pdev)
{
	struct mpfs_rtc_dev *rtcdev;
	struct clk *clk;
	int ret;

	rtcdev = devm_kzalloc(&pdev->dev, sizeof(struct mpfs_rtc_dev), GFP_KERNEL);
	if (!rtcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, rtcdev);

	rtcdev->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtcdev->rtc))
		return PTR_ERR(rtcdev->rtc);

	rtcdev->rtc->ops = &mpfs_rtc_ops;
	rtcdev->rtc->range_max = U32_MAX;

	clk = mpfs_rtc_init_clk(&pdev->dev);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rtcdev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rtcdev->base)) {
		dev_dbg(&pdev->dev, "invalid ioremap resources\n");
		return PTR_ERR(rtcdev->base);
	}

	rtcdev->wakeup_irq = platform_get_irq(pdev, 0);
	if (rtcdev->wakeup_irq <= 0) {
		dev_dbg(&pdev->dev, "could not get wakeup irq\n");
		return rtcdev->wakeup_irq;
	}
	ret = devm_request_irq(&pdev->dev, rtcdev->wakeup_irq, mpfs_rtc_wakeup_irq_handler, 0,
			       dev_name(&pdev->dev), rtcdev);
	if (ret) {
		dev_dbg(&pdev->dev, "could not request wakeup irq\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "prescaler", &rtcdev->prescaler);
	if (ret)
		rtcdev->prescaler = DEFAULT_PRESCALER;

	if (rtcdev->prescaler > MAX_PRESCALER_COUNT) {
		dev_dbg(&pdev->dev, "invalid prescaler %d\n", rtcdev->prescaler);
		return -EPERM;
	}

	mpfs_rtc_init(rtcdev);

	dev_info(&pdev->dev, "Microchip Polarfire SoC RTC\n");

	device_init_wakeup(&pdev->dev, 1);

	return devm_rtc_register_device(rtcdev->rtc);
}

static int mpfs_rtc_remove(struct platform_device *pdev)
{
	mpfs_rtc_alarm_irq_enable(&pdev->dev, 0);
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mpfs_rtc_suspend(struct device *dev)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);

	/* leave the alarm on as a wake source */
	if (device_may_wakeup(dev))
		enable_irq_wake(rtcdev->wakeup_irq);
	else
		mpfs_rtc_alarm_irq_enable(dev, 0);

	return 0;
}

static int mpfs_rtc_resume(struct device *dev)
{
	struct mpfs_rtc_dev *rtcdev = dev_get_drvdata(dev);

	/* alarms were left on as a wake source, turn them off */
	if (device_may_wakeup(dev))
		disable_irq_wake(rtcdev->wakeup_irq);
	else
		mpfs_rtc_alarm_irq_enable(dev, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mpfs_rtc_pm_ops, mpfs_rtc_suspend, mpfs_rtc_resume);

static const struct of_device_id mpfs_rtc_of_match[] = {
	{ .compatible = "microchip,mpfs-rtc" },
	{ }
};

MODULE_DEVICE_TABLE(of, mpfs_rtc_of_match);

static struct platform_driver mpfs_rtc_driver = {
	.probe = mpfs_rtc_probe,
	.remove = mpfs_rtc_remove,
	.driver	= {
		.name = "mpfs_rtc",
		.pm = &mpfs_rtc_pm_ops,
		.of_match_table = mpfs_rtc_of_match,
	},
};

module_platform_driver(mpfs_rtc_driver);

MODULE_DESCRIPTION("Real time clock for Microchip Polarfire SoC");
MODULE_AUTHOR("Daire McNamara <daire.mcnamara@microchip.com>");
MODULE_LICENSE("GPL v2");
