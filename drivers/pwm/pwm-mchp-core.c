// SPDX-License-Identifier: GPL-2.0-only
/*
 * Microchip FPGA corePWM driver
 *
 * Copyright (c) 2021 Microchip Corporation. All rights reserved.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include <linux/math.h>

#define OUTPUT_MAPPED(mapped_outputs, n) (((mapped_outputs) >> (n)) & 0x1)
#define PREG_TO_VAL(PREG_TO_VAL) (PREG_TO_VAL + 1)

#define PRESCALE_REG 	0x00U
#define PERIOD_REG	0x04U
#define PWM_EN_LOW	0x08U
#define PWM_EN_HIGH	0x0CU
#define SYNC_UPD_REG	0xE4U
#define POSEDGE_OFFSET	0x10U
#define NEGEDGE_OFFSET	0x14U

struct mchp_core_pwm_registers {
	u32 posedge;
	u32 negedge;
	u32 period_steps;
	u32 prescale;
};

struct mchp_core_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	void __iomem *base;
	struct mchp_core_pwm_registers *regs;
	u16 mapped_outputs;
	u32 tmp_clk_rate;
};

static inline struct mchp_core_pwm_chip *
to_mchp_core_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct mchp_core_pwm_chip, chip);
}

static void mchp_core_pwm_enable(struct pwm_chip *chip,
				      struct pwm_device *pwm, bool enable)
{
	struct mchp_core_pwm_chip *mchp_core_pwm = to_mchp_core_pwm_chip(chip);
	u8 ch_enabled;
	u8 mask;
	u8 en_bit;
	u8 reg_offset = PWM_EN_LOW + (pwm->hwpwm / 8) * 0x4;
	en_bit = pwm->hwpwm > 7 ? pwm->hwpwm - 8 : pwm->hwpwm;

	//check this "maths"
	ch_enabled = (u8)(readl_relaxed(mchp_core_pwm->base + reg_offset));
	mask = ~(1 << en_bit);
	ch_enabled &= mask;
	ch_enabled |= (enable << en_bit);
	if (enable) {
		writel_relaxed((u32) ch_enabled, mchp_core_pwm->base + reg_offset);
	} else {
		writel_relaxed((u32) ch_enabled, mchp_core_pwm->base + reg_offset);
	}
}

static void
mchp_core_pwm_calculate_duty(struct pwm_chip *chip,
				 struct pwm_state *desired_state,
				 struct mchp_core_pwm_registers *regs)
{
	struct mchp_core_pwm_chip *mchp_core_pwm = to_mchp_core_pwm_chip(chip);
	u64 clk_period = NSEC_PER_SEC;
	u64 duty_steps;
	u64 local_duty;

	/* Calculate the clk period and then the duty cycle edges */
	//clk_get_rate(mchp_core_pwm->clk)
	do_div(clk_period, mchp_core_pwm->tmp_clk_rate);

	duty_steps = desired_state->duty_cycle * PREG_TO_VAL(regs->period_steps);
	do_div(duty_steps, (clk_period * PREG_TO_VAL(regs->period_steps)));
	if (desired_state->polarity == PWM_POLARITY_INVERSED) {
		regs->negedge = 0u;
		regs->posedge = duty_steps;
	}
	else {
		regs->posedge = 0u;
		regs->negedge = duty_steps;
	}
}

static void mchp_core_pwm_apply_duty(const u8 channel,
			     struct mchp_core_pwm_chip *pwm_chip,
			     struct mchp_core_pwm_registers *regs)
{
	u32 channel_offset = (channel)*0x8U;
	writel_relaxed(regs->posedge, pwm_chip->base + channel_offset + POSEDGE_OFFSET);
	writel_relaxed(regs->negedge, pwm_chip->base + channel_offset + NEGEDGE_OFFSET);
}
static void
mchp_core_pwm_apply_period(struct mchp_core_pwm_chip *pwm_chip,
			     struct mchp_core_pwm_registers *regs)
{
	writel_relaxed(regs->prescale, pwm_chip->base + PRESCALE_REG);
	writel_relaxed(regs->period_steps, pwm_chip->base + PERIOD_REG);
}

static int
mchp_core_pwm_calculate_base(struct pwm_chip *chip,
				 const struct pwm_state *desired_state,
				 u8 *period_steps_r, u8 *prescale_r)
{
	struct mchp_core_pwm_chip *mchp_core_pwm = to_mchp_core_pwm_chip(chip);
	u64 tmp = desired_state->period;
	u64 clk_period = NSEC_PER_SEC;
	int shift;

	/* Calculate the period cycles and prescale value */
	//clk_get_rate(mchp_core_pwm->clk)
	tmp *= mchp_core_pwm->tmp_clk_rate;
	do_div(tmp, NSEC_PER_SEC);

	if (tmp > 65535) {
		dev_err(chip->dev, "pres exceeds the maximum value\n");
		return -EINVAL;
	} else if (tmp <= 256) {
		*prescale_r = 0;
		*period_steps_r = tmp - 1;
	} else {
		*prescale_r = fls(tmp) - 8;
		*period_steps_r = (tmp >> *prescale_r) - 1;
	}

	return 0;
}

static int mchp_core_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
				   const struct pwm_state *desired_state)
{
	struct mchp_core_pwm_chip *mchp_core_pwm = to_mchp_core_pwm_chip(chip);
	struct pwm_state current_state;
	u8 period_steps_r, prescale_r;
	u32 pres, val;
	int ret;
	u8 channel = pwm->hwpwm;

	if (!OUTPUT_MAPPED(mchp_core_pwm->mapped_outputs, channel)) {
		dev_warn_once(chip->dev, "output is not mapped to io\n");
		return -EINVAL;
	}

	pwm_get_state(pwm, &current_state);

	if (desired_state->enabled) {
		if (current_state.enabled && current_state.period == desired_state->period &&
			current_state.polarity == desired_state->polarity) {
			mchp_core_pwm_calculate_duty(
				chip, desired_state, mchp_core_pwm->regs);
			mchp_core_pwm_apply_duty(channel, mchp_core_pwm,
						     mchp_core_pwm->regs);
		}
		else {

			ret = mchp_core_pwm_calculate_base(chip, desired_state, &period_steps_r, &prescale_r);
			if (ret) {
				dev_err(chip->dev,
					"failed to calculate baser\n");
				return ret;
			}

			mchp_core_pwm->regs->period_steps = period_steps_r;
			mchp_core_pwm->regs->prescale = prescale_r;

			mchp_core_pwm_calculate_duty(chip, desired_state,
							mchp_core_pwm->regs);
			mchp_core_pwm_apply_duty(channel, mchp_core_pwm, mchp_core_pwm->regs);
			mchp_core_pwm_apply_period(mchp_core_pwm,
						mchp_core_pwm->regs);
		}

		if (mchp_core_pwm->regs->posedge == mchp_core_pwm->regs->negedge) {
			mchp_core_pwm_enable(chip, pwm, false);
		} else {
			mchp_core_pwm_enable(chip, pwm, true);
		}

	} else if (!desired_state->enabled) {
		mchp_core_pwm_enable(chip, pwm, false);
	}

	return 0;
}

static void mchp_core_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct mchp_core_pwm_chip *mchp_core_pwm = to_mchp_core_pwm_chip(chip);
	u64 clk_period = NSEC_PER_SEC;
	u8 prescale, period_steps, duty_steps;
	// u8 sync_update; //????????????/
	u8 posedge, negedge;
	u16 ch_enabled;
	u32 channel_offset = (pwm->hwpwm) * 0x8U;
	u8 channel = pwm->hwpwm;

	if (!OUTPUT_MAPPED(mchp_core_pwm->mapped_outputs, channel)) {
		dev_warn_once(chip->dev, "output is not mapped to io\n");
		state->enabled = false;
		return;
	}

	ch_enabled = (u16)(
		(readl_relaxed(mchp_core_pwm->base + PWM_EN_HIGH) << 8) |
		readl_relaxed(mchp_core_pwm->base + PWM_EN_LOW));

	/* Calculate the period cycles and prescale value */
	do_div(clk_period, mchp_core_pwm->tmp_clk_rate);

	prescale = (u8)readl_relaxed(mchp_core_pwm->base +
					PRESCALE_REG);
	period_steps =
		(u8)readl_relaxed(mchp_core_pwm->base + PERIOD_REG);
	posedge = (u8)readl_relaxed(mchp_core_pwm->base +
					channel_offset + POSEDGE_OFFSET);
	negedge = (u8)readl_relaxed(mchp_core_pwm->base +
					channel_offset + NEGEDGE_OFFSET);
	// sync_update	= (u8 )readl_relaxed(mchp_core_pwm->base + PRESCALE_REG);

	duty_steps = abs((s8)posedge-(s8)negedge);
	state->polarity = negedge < posedge ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

	state->duty_cycle =  PREG_TO_VAL(prescale) * clk_period * duty_steps;
	state->period = PREG_TO_VAL(prescale) * clk_period * PREG_TO_VAL(period_steps);


	if (ch_enabled & 1 << (pwm->hwpwm)) {
		state->enabled = true;
	} else {
		state->enabled = false;
	}
}

static const struct pwm_ops mchp_core_pwm_ops = {
	.apply = mchp_core_pwm_apply,
	.get_state = mchp_core_pwm_get_state,
	.owner = THIS_MODULE,
};

static const struct of_device_id mchp_core_of_match[] = {
	{
		.compatible = "microchip,corepwm",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mchp_core_of_match);

static int mchp_core_pwm_probe(struct platform_device *pdev)
{
	struct mchp_core_pwm_chip *mchp_pwm;
	struct resource *regs;
	int ret;

	mchp_pwm = devm_kzalloc(&pdev->dev, sizeof(*mchp_pwm), GFP_KERNEL);
	if (!mchp_pwm)
		return -ENOMEM;

	mchp_pwm->regs = devm_kzalloc(&pdev->dev, sizeof(*regs), GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	mchp_pwm->base = devm_platform_get_and_ioremap_resource(pdev, 0, &regs);
	if (IS_ERR(mchp_pwm->base))
		return PTR_ERR(mchp_pwm->base);

	mchp_pwm->clk = devm_clk_get(&pdev->dev, "fic3");
	if (IS_ERR(mchp_pwm->clk))
		return PTR_ERR(mchp_pwm->clk);

	ret = clk_prepare(mchp_pwm->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare PWM clock\n");
		return ret;
	}

	/* temp */
	ret = device_property_read_u32(&pdev->dev, "clock-frequency", &mchp_pwm->tmp_clk_rate);
	if (ret) {
		dev_info(&pdev->dev, "default to 100kHz\n");
		mchp_pwm->tmp_clk_rate = 62500000; /* default clock rate */
	}
	printk("clk rate %lu\r\n", mchp_pwm->tmp_clk_rate);
	/* temp */

	mchp_pwm->chip.dev = &pdev->dev;
	mchp_pwm->chip.ops = &mchp_core_pwm_ops;
	mchp_pwm->chip.of_xlate = of_pwm_xlate_with_flags;
	mchp_pwm->chip.of_pwm_n_cells = 3;
	mchp_pwm->chip.base = -1;
	mchp_pwm->chip.npwm = 16;

	ret = pwmchip_add(&mchp_pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip %d\n", ret);
	}
	
	ret = of_property_read_u16(pdev->dev.of_node, "mchp,mapped-outputs",
				   &mchp_pwm->mapped_outputs);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to find mapped outputs\n");
		return -ENODEV;
	}

	writel_relaxed(0U, mchp_pwm->base + PWM_EN_LOW);
	writel_relaxed(0U, mchp_pwm->base + PWM_EN_HIGH);

	platform_set_drvdata(pdev, mchp_pwm);
	dev_info(&pdev->dev,
		 "Successfully registered mpfs pwm with outputs: %x\n",
		 mchp_pwm->mapped_outputs);

	return ret;
}

static int mchp_core_pwm_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mchp_core_pwm_driver = {
	.driver = {
		.name = "mchp-core-pwm",
		.of_match_table = mchp_core_of_match,
	},
	.probe = mchp_core_pwm_probe,
	.remove = mchp_core_pwm_remove,
};
module_platform_driver(mchp_core_pwm_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Conor Dooley <conor.dooley@microchip.com>");
MODULE_DESCRIPTION("microchip corePWM driver");
