// SPDX-License-Identifier: (GPL-2.0)
/*
 * Microchip PolarFire SoC (MPFS) GPIO controller driver
 *
 * Copyright (c) 2018-2022 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Lewis Hanly <lewis.hanly@microchip.com>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define MPFS_GPIO_CTRL(i)		(0x4 * (i))
#define MAX_NUM_GPIO			32
#define MPFS_GPIO_EN_INT		3
#define MPFS_GPIO_EN_OUT_BUF		BIT(2)
#define MPFS_GPIO_EN_IN			BIT(1)
#define MPFS_GPIO_EN_OUT		BIT(0)

#define MPFS_GPIO_TYPE_INT_EDGE_BOTH	0x80
#define MPFS_GPIO_TYPE_INT_EDGE_NEG	0x60
#define MPFS_GPIO_TYPE_INT_EDGE_POS	0x40
#define MPFS_GPIO_TYPE_INT_LEVEL_LOW	0x20
#define MPFS_GPIO_TYPE_INT_LEVEL_HIGH	0x00
#define MPFS_GPIO_TYPE_INT_MASK		GENMASK(7, 5)
#define MPFS_IRQ_REG			0x80
#define MPFS_INP_REG			0x84
#define MPFS_OUTP_REG			0x88

struct mpfs_gpio_chip {
	void __iomem *base;
	struct clk *clk;
	raw_spinlock_t	lock;
	struct gpio_chip gc;
};

static void mpfs_gpio_assign_bit(void __iomem *addr, unsigned int bit_offset, bool value)
{
	unsigned long reg = readl(addr);

	__assign_bit(bit_offset, &reg, value);
	writel(reg, addr);
}

static int mpfs_gpio_direction_input(struct gpio_chip *gc, unsigned int gpio_index)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;
	unsigned long flags;

	raw_spin_lock_irqsave(&mpfs_gpio->lock, flags);

	gpio_cfg = readl(mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));
	gpio_cfg |= MPFS_GPIO_EN_IN;
	gpio_cfg &= ~(MPFS_GPIO_EN_OUT | MPFS_GPIO_EN_OUT_BUF);
	writel(gpio_cfg, mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));

	raw_spin_unlock_irqrestore(&mpfs_gpio->lock, flags);

	return 0;
}

static int mpfs_gpio_direction_output(struct gpio_chip *gc, unsigned int gpio_index, int value)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;
	unsigned long flags;

	raw_spin_lock_irqsave(&mpfs_gpio->lock, flags);

	gpio_cfg = readl(mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));
	gpio_cfg |= MPFS_GPIO_EN_OUT | MPFS_GPIO_EN_OUT_BUF;
	gpio_cfg &= ~MPFS_GPIO_EN_IN;
	writel(gpio_cfg, mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));

	mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_OUTP_REG, gpio_index, value);

	raw_spin_unlock_irqrestore(&mpfs_gpio->lock, flags);

	return 0;
}

static int mpfs_gpio_get_direction(struct gpio_chip *gc,
				   unsigned int gpio_index)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;

	gpio_cfg = readl(mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));
	if (gpio_cfg & MPFS_GPIO_EN_IN)
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static int mpfs_gpio_get(struct gpio_chip *gc,
			 unsigned int gpio_index)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);

	return !!(readl(mpfs_gpio->base + MPFS_INP_REG) & BIT(gpio_index));
}

static void mpfs_gpio_set(struct gpio_chip *gc, unsigned int gpio_index, int value)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	unsigned long flags;

	raw_spin_lock_irqsave(&mpfs_gpio->lock, flags);

	mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_OUTP_REG,
			     gpio_index, value);

	raw_spin_unlock_irqrestore(&mpfs_gpio->lock, flags);
}

static int mpfs_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	int gpio_index = irqd_to_hwirq(data);
	u32 interrupt_type;
	u32 gpio_cfg;
	unsigned long flags;

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		interrupt_type = MPFS_GPIO_TYPE_INT_EDGE_BOTH;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		interrupt_type = MPFS_GPIO_TYPE_INT_EDGE_NEG;
		break;
	case IRQ_TYPE_EDGE_RISING:
		interrupt_type = MPFS_GPIO_TYPE_INT_EDGE_POS;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		interrupt_type = MPFS_GPIO_TYPE_INT_LEVEL_HIGH;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		interrupt_type = MPFS_GPIO_TYPE_INT_LEVEL_LOW;
		break;
	}

	raw_spin_lock_irqsave(&mpfs_gpio->lock, flags);

	gpio_cfg = readl(mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));
	gpio_cfg &= ~MPFS_GPIO_TYPE_INT_MASK;
	gpio_cfg |= interrupt_type;
	writel(gpio_cfg, mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index));

	raw_spin_unlock_irqrestore(&mpfs_gpio->lock, flags);

	return 0;
}

static void mpfs_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	int gpio_index = irqd_to_hwirq(data);

	gpiochip_enable_irq(gc, gpio_index);
	mpfs_gpio_direction_input(gc, gpio_index);
	mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_IRQ_REG, gpio_index, 1);
	mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index),
			     MPFS_GPIO_EN_INT, 1);
}

static void mpfs_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	int gpio_index = irqd_to_hwirq(data);

	mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_IRQ_REG, gpio_index, 1);
	mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_GPIO_CTRL(gpio_index),
			     MPFS_GPIO_EN_INT, 0);
	gpiochip_disable_irq(gc, gpio_index);
}

static const struct irq_chip mpfs_gpio_irqchip = {
	.name = "mpfs",
	.irq_set_type = mpfs_gpio_irq_set_type,
	.irq_mask	= mpfs_gpio_irq_mask,
	.irq_unmask	= mpfs_gpio_irq_unmask,
	.flags = IRQCHIP_IMMUTABLE | IRQCHIP_MASK_ON_SUSPEND,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static void mpfs_gpio_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	struct mpfs_gpio_chip *mpfs_gpio =
		gpiochip_get_data(irq_desc_get_handler_data(desc));
	unsigned long status;
	int offset;

	chained_irq_enter(irqchip, desc);

	status = readl(mpfs_gpio->base + MPFS_IRQ_REG);
	for_each_set_bit(offset, &status, mpfs_gpio->gc.ngpio) {
		mpfs_gpio_assign_bit(mpfs_gpio->base + MPFS_IRQ_REG, offset, 1);
		generic_handle_irq(irq_find_mapping(mpfs_gpio->gc.irq.domain, offset));
	}

	chained_irq_exit(irqchip, desc);
}

static int mpfs_gpio_probe(struct platform_device *pdev)
{
	struct clk *clk;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct mpfs_gpio_chip *mpfs_gpio;
	struct gpio_irq_chip *girq;
	int i, ret, ngpios, nirqs;

	mpfs_gpio = devm_kzalloc(dev, sizeof(*mpfs_gpio), GFP_KERNEL);
	if (!mpfs_gpio)
		return -ENOMEM;

	mpfs_gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mpfs_gpio->base))
		return dev_err_probe(dev, PTR_ERR(mpfs_gpio->base), "failed to ioremap memory resource\n");

	clk = devm_clk_get(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "devm_clk_get failed\n");

	ret = clk_prepare_enable(clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clock\n");

	mpfs_gpio->clk = clk;

	ngpios = MAX_NUM_GPIO;
	device_property_read_u32(dev, "ngpios", &ngpios);
	if (ngpios > MAX_NUM_GPIO)
		ngpios = MAX_NUM_GPIO;

	raw_spin_lock_init(&mpfs_gpio->lock);
	mpfs_gpio->gc.direction_input = mpfs_gpio_direction_input;
	mpfs_gpio->gc.direction_output = mpfs_gpio_direction_output;
	mpfs_gpio->gc.get_direction = mpfs_gpio_get_direction;
	mpfs_gpio->gc.get = mpfs_gpio_get;
	mpfs_gpio->gc.set = mpfs_gpio_set;
	mpfs_gpio->gc.base = -1;
	mpfs_gpio->gc.ngpio = ngpios;
	mpfs_gpio->gc.label = dev_name(dev);
	mpfs_gpio->gc.parent = dev;
	mpfs_gpio->gc.owner = THIS_MODULE;

	nirqs = of_irq_count(node);
	if (nirqs > MAX_NUM_GPIO) {
		ret = -ENXIO;
		goto cleanup_clock;
	}
	girq = &mpfs_gpio->gc.irq;
	gpio_irq_chip_set_chip(girq, &mpfs_gpio_irqchip);
	girq->handler = handle_simple_irq;
	girq->parent_handler = mpfs_gpio_irq_handler;
	girq->default_type = IRQ_TYPE_NONE;
	girq->num_parents = nirqs;
	girq->parents = devm_kcalloc(&pdev->dev, nirqs,
				     sizeof(*girq->parents), GFP_KERNEL);
	if (!girq->parents) {
		ret = -ENOMEM;
		goto cleanup_clock;
	}
	for (i = 0; i < nirqs; i++)
		girq->parents[i] = platform_get_irq(pdev, i);

	ret = gpiochip_add_data(&mpfs_gpio->gc, mpfs_gpio);
	if (ret)
		goto cleanup_clock;

	platform_set_drvdata(pdev, mpfs_gpio);

	return 0;

cleanup_clock:
	clk_disable_unprepare(mpfs_gpio->clk);
	return ret;
}

static int mpfs_gpio_remove(struct platform_device *pdev)
{
	struct mpfs_gpio_chip *mpfs_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&mpfs_gpio->gc);
	clk_disable_unprepare(mpfs_gpio->clk);

	return 0;
}

static const struct of_device_id mpfs_of_ids[] = {
	{ .compatible = "microchip,mpfs-gpio", },
	{ /* end of list */ }
};

static struct platform_driver mpfs_gpio_driver = {
	.probe = mpfs_gpio_probe,
	.driver = {
		.name = "microchip,mpfs-gpio",
		.of_match_table = mpfs_of_ids,
	},
	.remove = mpfs_gpio_remove,
};
builtin_platform_driver(mpfs_gpio_driver);
