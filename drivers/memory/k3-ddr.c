// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI K3 DDR Controller Driver - Temperature Monitoring
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This driver monitors DDR temperature by reading LPDDR4 MR4 registers
 * and provides hwmon interface for temperature status notifications.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/devm-helpers.h>

/* J7 (DDRSS) Register offsets */
#define J7_INT_STAT				0x494	/* CTL_293 */
#define J7_INT_ACK				0x49C	/* CTL_295 */
#define J7_INT_MASK				0x4A4	/* CTL_297 */
#define J7_TEMP_REG0				0x288	/* CTL_162 */
#define J7_TEMP_REG1				0x28C	/* CTL_163 */

/* AM62 (DENALI) Register offsets */
#define AM62_INT_STAT_MASTER			0x538	/* CTL_334 */
#define AM62_INT_MASK_MASTER			0x53C	/* CTL_335 */
#define AM62_INT_STAT_MISC			0x554	/* CTL_341 */
#define AM62_INT_ACK_MISC			0x574	/* CTL_349 */
#define AM62_TEMP_REG0				0x2F8	/* CTL_190 */
#define AM62_TEMP_REG1				0x2FC	/* CTL_191 */

/* AM62A (DENALI variant) Register offsets */
#define AM62A_INT_STAT_MASTER			0x558	/* CTL_342 */
#define AM62A_INT_MASK_MASTER			0x55C	/* CTL_343 */
#define AM62A_INT_STAT_MISC			0x574	/* CTL_349 */
#define AM62A_INT_ACK_MISC			0x594	/* CTL_357 */
#define AM62A_TEMP_REG0				0x304	/* CTL_193 */
#define AM62A_TEMP_REG1				0x308	/* CTL_194 */

enum k3_ddr_fields {
	K3_DDR_INT_STAT_MASTER,
	K3_DDR_INT_MASK_MASTER,
	K3_DDR_INT_STAT_TUF,
	K3_DDR_INT_ACK_TUF,
	K3_DDR_TEMP_REG0_FIELD0,
	K3_DDR_TEMP_REG0_FIELD1,
	K3_DDR_TEMP_REG1_FIELD0,
	K3_DDR_TEMP_REG1_FIELD1,
	K3_DDR_MAX_FIELDS
};

static const struct reg_field j7_reg[] = {
	[K3_DDR_INT_STAT_MASTER]  = REG_FIELD(J7_INT_STAT, 28, 28),
	[K3_DDR_INT_MASK_MASTER]  = REG_FIELD(J7_INT_MASK, 28, 28),
	[K3_DDR_INT_STAT_TUF]	  = REG_FIELD(J7_INT_STAT, 28, 28),
	[K3_DDR_INT_ACK_TUF]	  = REG_FIELD(J7_INT_ACK, 28, 28),
	[K3_DDR_TEMP_REG0_FIELD0] = REG_FIELD(J7_TEMP_REG0, 8, 10),
	[K3_DDR_TEMP_REG0_FIELD1] = REG_FIELD(J7_TEMP_REG0, 16, 18),
	[K3_DDR_TEMP_REG1_FIELD0] = REG_FIELD(J7_TEMP_REG1, 0, 2),
	[K3_DDR_TEMP_REG1_FIELD1] = REG_FIELD(J7_TEMP_REG1, 8, 10),
};

static const struct reg_field am62_reg[] = {
	[K3_DDR_INT_STAT_MASTER]  = REG_FIELD(AM62_INT_STAT_MASTER, 7, 7),
	[K3_DDR_INT_MASK_MASTER]  = REG_FIELD(AM62_INT_MASK_MASTER, 7, 7),
	[K3_DDR_INT_STAT_TUF]	  = REG_FIELD(AM62_INT_STAT_MISC, 5, 5),
	[K3_DDR_INT_ACK_TUF]	  = REG_FIELD(AM62_INT_ACK_MISC, 5, 5),
	[K3_DDR_TEMP_REG0_FIELD0] = REG_FIELD(AM62_TEMP_REG0, 24, 26),
	[K3_DDR_TEMP_REG0_FIELD1] = REG_FIELD(AM62_TEMP_REG0, 28, 30),
	[K3_DDR_TEMP_REG1_FIELD0] = REG_FIELD(AM62_TEMP_REG1, 0, 2),
	[K3_DDR_TEMP_REG1_FIELD1] = REG_FIELD(AM62_TEMP_REG1, 4, 6),
};

static const struct reg_field am62a_reg[] = {
	[K3_DDR_INT_STAT_MASTER]  = REG_FIELD(AM62A_INT_STAT_MASTER, 7, 7),
	[K3_DDR_INT_MASK_MASTER]  = REG_FIELD(AM62A_INT_MASK_MASTER, 7, 7),
	[K3_DDR_INT_STAT_TUF]	  = REG_FIELD(AM62A_INT_STAT_MISC, 5, 5),
	[K3_DDR_INT_ACK_TUF]	  = REG_FIELD(AM62A_INT_ACK_MISC, 5, 5),
	[K3_DDR_TEMP_REG0_FIELD0] = REG_FIELD(AM62A_TEMP_REG0, 8, 10),
	[K3_DDR_TEMP_REG0_FIELD1] = REG_FIELD(AM62A_TEMP_REG0, 12, 14),
	[K3_DDR_TEMP_REG1_FIELD0] = REG_FIELD(AM62A_TEMP_REG1, 0, 2),
	[K3_DDR_TEMP_REG1_FIELD1] = REG_FIELD(AM62A_TEMP_REG1, 4, 6),
};

struct k3_ddr_cfg {
	const struct reg_field *cfg_fields;
	bool has_intr_group;
};

static const struct k3_ddr_cfg j7_cfg = {
	.cfg_fields	= j7_reg,
	.has_intr_group	= false
};

static const struct k3_ddr_cfg am62_cfg = {
	.cfg_fields	= am62_reg,
	.has_intr_group	= true
};

static const struct k3_ddr_cfg am62a_cfg = {
	.cfg_fields	= am62a_reg,
	.has_intr_group	= true
};

/**
 * struct k3_ddr_dev - K3 DDR device context
 * @dev:		Device pointer
 * @hwmon_dev:		Hwmon device pointer
 * @intr_work:		Work queue for interrupt handling
 * @cfg:		SoC-specific configuration
 * @regmap:		regmap pointer
 * @temp_val:		Current temperature value
 * @cfg_reg:		regmap fields for accessing reg with mask
 */
struct k3_ddr_dev {
	struct device *dev;
	struct device *hwmon_dev;
	struct work_struct intr_work;
	const struct k3_ddr_cfg *cfg;
	struct regmap *regmap;
	u8 temp_val;
	struct regmap_field *cfg_reg[K3_DDR_MAX_FIELDS];
};

/* Temperature status strings from LPDDR4 MR4 register */
static const char * const temp_status[] = {
	[0] = "low temperature\n",
	[1] = "4x refresh interval\n",
	[2] = "2x refresh interval\n",
	[3] = "1x refresh interval\n",
	[4] = "0.5x refresh interval\n",
	[5] = "0.25x refresh interval\n",
	[6] = "0.25x refresh interval with derating\n",
	[7] = "high temperature\n",
};

/* Regmap configuration */
static const struct regmap_config k3_ddr_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.fast_io	= true,
};

static void k3_ddr_temp_notify_work(struct work_struct *work)
{
	struct k3_ddr_dev *ddr = container_of(work, struct k3_ddr_dev,
					      intr_work);

	sysfs_notify(&ddr->hwmon_dev->kobj, NULL, "k3_ddr_temp_status");
	kobject_uevent(&ddr->hwmon_dev->kobj, KOBJ_CHANGE);
}

static irqreturn_t k3_ddr_irq_handler(int irq, void *data)
{
	struct k3_ddr_dev *ddr = data;
	const struct k3_ddr_cfg *cfg = ddr->cfg;
	int ret = 0;
	u32 stat_group = 0, stat_tuf = 0;
	u32 reg0_fld0, reg0_fld1, reg1_fld0, reg1_fld1;
	u8 temp0, temp1;

	/* Check interrupt group status (AM62/AM62A only) */
	if (cfg->has_intr_group) {
		ret = regmap_field_read(ddr->cfg_reg[K3_DDR_INT_STAT_MASTER], &stat_group);
		if (ret) {
			dev_err(ddr->dev, "Failed to read group status!\n");
			return IRQ_NONE;
		}
		if (!(stat_group))
			return IRQ_NONE;
	}

	/* Read TUF interrupt status */
	ret = regmap_field_read(ddr->cfg_reg[K3_DDR_INT_STAT_TUF], &stat_tuf);
	if (ret) {
		dev_err(ddr->dev, "Failed to read TUF status!\n");
		return IRQ_NONE;
	}
	if (!(stat_tuf))
		return IRQ_NONE;

	/* Read and extract temperature from both registers */
	ret = regmap_field_read(ddr->cfg_reg[K3_DDR_TEMP_REG0_FIELD0], &reg0_fld0);
	if (ret) {
		dev_err(ddr->dev, "Failed to read regmap field: %d:!\n", K3_DDR_TEMP_REG0_FIELD0);
		return IRQ_NONE;
	}
	ret = regmap_field_read(ddr->cfg_reg[K3_DDR_TEMP_REG0_FIELD1], &reg0_fld1);
	if (ret) {
		dev_err(ddr->dev, "Failed to read regmap field: %d:!\n", K3_DDR_TEMP_REG0_FIELD1);
		return IRQ_NONE;
	}
	ret = regmap_field_read(ddr->cfg_reg[K3_DDR_TEMP_REG1_FIELD0], &reg1_fld0);
	if (ret) {
		dev_err(ddr->dev, "Failed to read regmap field: %d:!\n", K3_DDR_TEMP_REG1_FIELD0);
		return IRQ_NONE;
	}
	ret = regmap_field_read(ddr->cfg_reg[K3_DDR_TEMP_REG1_FIELD1], &reg1_fld1);
	if (ret) {
		dev_err(ddr->dev, "Failed to read regmap field: %d:!\n", K3_DDR_TEMP_REG1_FIELD1);
		return IRQ_NONE;
	}

	/* Find the max temp */
	temp0 = max(reg0_fld0, reg0_fld1);
	temp1 = max(reg1_fld0, reg1_fld1);

	ddr->temp_val = max(temp0, temp1);

	/* Acknowledge interrupt */
	ret = regmap_field_write(ddr->cfg_reg[K3_DDR_INT_ACK_TUF], 1U);
	if (ret) {
		dev_err(ddr->dev, "Failed to write Ack TUF interrupt!\n");
		return IRQ_NONE;
	}

	/* Notify userspace */
	schedule_work(&ddr->intr_work);

	return IRQ_HANDLED;
}

static ssize_t k3_ddr_temp_status_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct k3_ddr_dev *ddr = dev_get_drvdata(dev);

	if (ddr->temp_val >= ARRAY_SIZE(temp_status))
		return sysfs_emit(buf, "unknown\n");

	return sysfs_emit(buf, "%s\n", temp_status[ddr->temp_val]);
}

static DEVICE_ATTR_RO(k3_ddr_temp_status);

static umode_t k3_ddr_is_visible(struct kobject *kobj,
				 struct attribute *attr, int index)
{
	return attr->mode;
}

static struct attribute *k3_ddr_attrs[] = {
	&dev_attr_k3_ddr_temp_status.attr,
	NULL
};

static const struct attribute_group k3_ddr_group = {
	.attrs = k3_ddr_attrs,
	.is_visible = k3_ddr_is_visible,
};

static const struct attribute_group *k3_ddr_groups[] = {
	&k3_ddr_group,
	NULL
};

static int k3_ddr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct k3_ddr_dev *ddr;
	struct device_node *child_np;
	struct resource res;
	int irq, ret;
	void __iomem *base;

	ddr = devm_kzalloc(dev, sizeof(*ddr), GFP_KERNEL);
	if (!ddr)
		return -ENOMEM;

	ddr->dev = dev;
	ddr->cfg = device_get_match_data(dev);
	if (!ddr->cfg)
		return dev_err_probe(dev, -ENODEV,
				     "No configuration data found\n");

	/* Find ddr child node */
	child_np = of_get_child_by_name(dev->of_node, "ddr");
	if (!child_np)
		return dev_err_probe(dev, -ENODEV,
				     "ddr child node not found\n");

	/* Get and map child registers */
	ret = of_address_to_resource(child_np, 0, &res);
	if (ret) {
		of_node_put(child_np);
		return dev_err_probe(dev, ret,
				     "Failed to get child register address\n");
	}

	base = devm_ioremap_resource(dev, &res);
	of_node_put(child_np);

	if (IS_ERR(base))
		return PTR_ERR(base);

	/* Initialize regmap */
	ddr->regmap = devm_regmap_init_mmio(dev,
					    base,
					    &k3_ddr_regmap_config);
	if (IS_ERR(ddr->regmap)) {
		dev_err(dev, "Failed to init regmap: %ld\n",
			PTR_ERR(ddr->regmap));
		return PTR_ERR(ddr->regmap);
	}

	ret = devm_regmap_field_bulk_alloc(dev, ddr->regmap, ddr->cfg_reg,
					   ddr->cfg->cfg_fields, K3_DDR_MAX_FIELDS);
	if (ret) {
		dev_err(dev, "Failed to create regmap fields!\n");
		return ret;
	}

	/* Enable/Unmask temperature interrupts */
	ret = regmap_field_write(ddr->cfg_reg[K3_DDR_INT_MASK_MASTER], 0U);
	if (ret) {
		dev_err(dev, "Failed to unmask interrupt!\n");
		return ret;
	}

	platform_set_drvdata(pdev, ddr);

	/* Register hwmon device */
	ddr->hwmon_dev = devm_hwmon_device_register_with_groups(dev, "k3_ddr",
								ddr,
								k3_ddr_groups);
	if (IS_ERR(ddr->hwmon_dev))
		return PTR_ERR(ddr->hwmon_dev);

	/* Initialize work queue before requesting IRQ */
	devm_work_autocancel(dev, &ddr->intr_work, k3_ddr_temp_notify_work);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* Request interrupt */
	ret = devm_request_irq(dev, irq, k3_ddr_irq_handler, IRQF_NO_SUSPEND,
			       dev_name(dev), ddr);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ %d\n",
				     irq);

	dev_info(dev, "K3 DDR temperature monitoring initialized\n");

	return 0;
}

static const struct of_device_id k3_ddr_of_match[] = {
	{ .compatible = "ti,j7-ddrss",   .data = &j7_cfg },
	{ .compatible = "ti,am62-ddrss", .data = &am62_cfg },
	{ .compatible = "ti,am62a-ddrss", .data = &am62a_cfg },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, k3_ddr_of_match);

static struct platform_driver k3_ddr_driver = {
	.probe		= k3_ddr_probe,
	.driver		= {
		.name		= "k3-ddr",
		.of_match_table	= k3_ddr_of_match,
	},
};
module_platform_driver(k3_ddr_driver);

MODULE_DESCRIPTION("TI K3 DDR Temperature Monitoring Driver");
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_LICENSE("GPL");
