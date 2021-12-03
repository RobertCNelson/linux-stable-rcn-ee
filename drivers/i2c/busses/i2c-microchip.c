// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip I2C controller driver
 *
 * Copyright (c) 2018 - 2021 Microchip Corporation. All rights reserved.
 *
 * Author: Daire McNamara <daire.mcnamara@microchip.com>
 */
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define MICROCHIP_I2C_TIMEOUT (msecs_to_jiffies(1000))

#define MPFS_I2C_CTRL				(0x00)
#define   CTRL_CR0				(0x00)
#define   CTRL_CR1				(0x01)
#define   CTRL_AA				(0x02)
#define   CTRL_SI				(0x03)
#define   CTRL_STO				(0x04)
#define   CTRL_STA				(0x05)
#define   CTRL_ENS1				(0x06)
#define   CTRL_CR2				(0x07)
#define MPFS_I2C_STATUS				(0x04)
#define   STATUS_BUS_ERROR			(0x00)
#define   STATUS_M_START_SENT			(0x08)
#define   STATUS_M_REPEATED_START_SENT		(0x10)
#define   STATUS_M_SLAW_ACK			(0x18)
#define   STATUS_M_SLAW_NACK			(0x20)
#define   STATUS_M_TX_DATA_ACK			(0x28)
#define   STATUS_M_TX_DATA_NACK			(0x30)
#define   STATUS_M_ARB_LOST			(0x38)
#define   STATUS_M_SLAR_ACK			(0x40)
#define   STATUS_M_SLAR_NACK			(0x48)
#define   STATUS_M_RX_DATA_ACKED		(0x50)
#define   STATUS_M_RX_DATA_NACKED		(0x58)
#define   STATUS_S_SLAW_ACKED			(0x60)
#define   STATUS_S_ARB_LOST_SLAW_ACKED		(0x68)
#define   STATUS_S_GENERAL_CALL_ACKED		(0x70)
#define   STATUS_S_ARB_LOST_GENERAL_CALL_ACKED	(0x78)
#define   STATUS_S_RX_DATA_ACKED		(0x80)
#define   STATUS_S_RX_DATA_NACKED		(0x88)
#define   STATUS_S_GENERAL_CALL_RX_DATA_ACKED	(0x90)
#define   STATUS_S_GENERAL_CALL_RX_DATA_NACKED	(0x98)
#define   STATUS_S_RX_STOP			(0xA0)
#define   STATUS_S_SLAR_ACKED			(0xA8)
#define   STATUS_S_ARB_LOST_SLAR_ACKED		(0xB0)
#define   STATUS_S_TX_DATA_ACK			(0xb8)
#define   STATUS_S_TX_DATA_NACK			(0xC0)
#define   STATUS_LAST_DATA_ACK			(0xC8)
#define   STATUS_M_SMB_MASTER_RESET		(0xD0)
#define   STATUS_S_SCL_LOW_TIMEOUT		(0xD8) /* 25 ms */
#define   STATUS_NO_STATE_INFO			(0xF8)
#define MPFS_I2C_DATA				(0x08)
#define   WRITE_BIT				(0)
#define   READ_BIT				BIT(0)
#define   SLAVE_ADDR_SHIFT			(1)
#define MPFS_I2C_SLAVE0_ADDR			(0x0c)
#define   GENERAL_CALL_BIT			BIT(0)
#define MPFS_I2C_SMBUS				(0x10)
#define   SMBALERT_INT_ENB			(0)
#define   SMBSUS_INT_ENB			(1)
#define   SMBUS_ENB				(2)
#define   SMBALERT_NI_STATUS			(3)
#define   SMBALERT_NO_CTRL			(4)
#define   SMBSUS_NI_STATUS			(5)
#define   SMBSUS_NO_CTRL			(6)
#define   SMBUS_RESET				(7)
#define MPFS_I2C_FREQ				(0x14)
#define MPFS_I2C_GLITCHREG			(0x18)
#define MPFS_I2C_SLAVE1_ADDR			(0x1c)

#define PCLK_DIV_256  ((0 << CTRL_CR0) | (0 << CTRL_CR1) | (0 << CTRL_CR2))
#define PCLK_DIV_224  ((1 << CTRL_CR0) | (0 << CTRL_CR1) | (0 << CTRL_CR2))
#define PCLK_DIV_192  ((0 << CTRL_CR0) | (1 << CTRL_CR1) | (0 << CTRL_CR2))
#define PCLK_DIV_160  ((1 << CTRL_CR0) | (1 << CTRL_CR1) | (0 << CTRL_CR2))
#define PCLK_DIV_960  ((0 << CTRL_CR0) | (0 << CTRL_CR1) | (1 << CTRL_CR2))
#define PCLK_DIV_120  ((1 << CTRL_CR0) | (0 << CTRL_CR1) | (1 << CTRL_CR2))
#define PCLK_DIV_60   ((0 << CTRL_CR0) | (1 << CTRL_CR1) | (1 << CTRL_CR2))
#define BCLK_DIV_8    ((1 << CTRL_CR0) | (1 << CTRL_CR1) | (1 << CTRL_CR2))
#define CLK_MASK      ((1 << CTRL_CR0) | (1 << CTRL_CR1) | (1 << CTRL_CR2))

/*
 * mpfs_i2c_dev - I2C device context
 * @base: pointer to register struct
 * @msg: pointer to current message
 * @msg_len: number of bytes transferred in msg
 * @msg_err: error code for completed message
 * @msg_complete: xfer completion object
 * @dev: device reference
 * @adapter: core i2c abstraction
 * @i2c_clk: clock reference for i2c input clock
 * @bus_clk_rate: current i2c bus clock rate
 * @buf: ptr to msg buffer for easier use.
 * @isr_status: cached copy of local ISR status.
 * @lock: spinlock for IRQ synchronization.
 */
struct mpfs_i2c_dev {
	void __iomem *base;
	size_t msg_len;
	int msg_err;
	struct completion msg_complete;
	struct device *dev;
	struct i2c_adapter adapter;
	struct clk *i2c_clk;
	u32 bus_clk_rate;
	u8 *buf;
	u8 addr;
	u32 msg_read;
	u32 isr_status;
	spinlock_t lock;	/* IRQ synchronization */
};

static void mpfs_i2c_int_clear(struct mpfs_i2c_dev *idev)
{
	u8 ctrl = readl(idev->base + MPFS_I2C_CTRL);

	ctrl &= ~(1 << CTRL_SI);
	writel(ctrl, idev->base + MPFS_I2C_CTRL);
}

static void mpfs_i2c_core_disable(struct mpfs_i2c_dev *idev)
{
	u8 ctrl = readl(idev->base + MPFS_I2C_CTRL);

	ctrl &= ~(1 << CTRL_ENS1);
	writel(ctrl, idev->base + MPFS_I2C_CTRL);
}

static void mpfs_i2c_core_enable(struct mpfs_i2c_dev *idev)
{
	u8 ctrl = readl(idev->base + MPFS_I2C_CTRL);

	ctrl |= (1 << CTRL_ENS1);
	writel(ctrl, idev->base + MPFS_I2C_CTRL);
}

static void mpfs_i2c_reset(struct mpfs_i2c_dev *idev)
{
	mpfs_i2c_core_disable(idev);
	mpfs_i2c_core_enable(idev);
}

static inline void mpfs_i2c_stop(struct mpfs_i2c_dev *idev)
{
	u8 ctrl = readl(idev->base + MPFS_I2C_CTRL);

	ctrl |= (1 << CTRL_STO);
	writel(ctrl, idev->base + MPFS_I2C_CTRL);
}

static inline int mpfs_generate_divisor(u32 rate, u8 *code)
{
	int ret = 0;

	if (rate >= 960)
		*code = PCLK_DIV_960;
	else if (rate >= 256)
		*code = PCLK_DIV_256;
	else if (rate >= 224)
		*code = PCLK_DIV_224;
	else if (rate >= 192)
		*code = PCLK_DIV_192;
	else if (rate >= 160)
		*code = PCLK_DIV_160;
	else if (rate >= 120)
		*code = PCLK_DIV_120;
	else if (rate >= 60)
		*code = PCLK_DIV_60;
	else if (rate >= 8)
		*code = BCLK_DIV_8;
	else
		ret = -1;

	return ret;
}

static int mpfs_i2c_init(struct mpfs_i2c_dev *idev)
{
	u32 clk_rate = clk_get_rate(idev->i2c_clk);
	u32 divisor = clk_rate / idev->bus_clk_rate;
	u8 clkval;
	int ret;
	u8 ctrl = readl(idev->base + MPFS_I2C_CTRL);

	ctrl &= ~CLK_MASK;

	ret = mpfs_generate_divisor(divisor, &clkval);

	if (ret)
		return -1;

	ctrl |= clkval;

	writel(ctrl, idev->base + MPFS_I2C_CTRL);

	ctrl = readl(idev->base + MPFS_I2C_CTRL);

	/* Reset controller */
	mpfs_i2c_reset(idev);

	return 0;
}

static void mpfs_i2c_transfer(struct mpfs_i2c_dev *idev, u32 data)
{
	if (idev->msg_len > 0)
		writel(data, idev->base + MPFS_I2C_DATA);
}

static void mpfs_i2c_empty_rx(struct mpfs_i2c_dev *idev)
{
	u8 ctrl;

	if (idev->msg_len > 0) {
		*idev->buf++ = readl(idev->base + MPFS_I2C_DATA);
		idev->msg_len--;
	}

	if (idev->msg_len == 0) {
		ctrl = readl(idev->base + MPFS_I2C_CTRL);
		ctrl &= ~(1 << CTRL_AA);
		writel(ctrl, idev->base + MPFS_I2C_CTRL);
	}
}

static int mpfs_i2c_fill_tx(struct mpfs_i2c_dev *idev)
{
	mpfs_i2c_transfer(idev, *idev->buf++);
	idev->msg_len--;

	return 0;
}

static irqreturn_t mpfs_i2c_handle_isr(int irq, void *_dev)
{
	bool read, finish = false;
	struct mpfs_i2c_dev *idev = _dev;
	u32 status = idev->isr_status;
	u8 ctrl;

	if (!idev->buf) {
		dev_warn(idev->dev, "unexpected interrupt\n");
		return IRQ_HANDLED;
	}

	read = idev->msg_read ? 1 : 0;

	switch (status) {
	case STATUS_M_START_SENT:
	case STATUS_M_REPEATED_START_SENT:
		ctrl = readl(idev->base + MPFS_I2C_CTRL);
		ctrl &= ~(1 << CTRL_STA);
		writel(idev->addr, idev->base + MPFS_I2C_DATA);
		writel(ctrl, idev->base + MPFS_I2C_CTRL);
		if (idev->msg_len <= 0)
			finish = true;
		break;
	case STATUS_M_ARB_LOST:
		/* handle Lost Arbitration */
		idev->msg_err = -EAGAIN;
		finish = true;
		break;
	case  STATUS_M_SLAW_ACK:
	case STATUS_M_TX_DATA_ACK:
		if (idev->msg_len > 0) {
			mpfs_i2c_fill_tx(idev);
		} else {
			/* On the last byte to be transmitted, send STOP */
			mpfs_i2c_stop(idev);
			finish = true;
		}
		break;
	case STATUS_M_TX_DATA_NACK:
	case STATUS_M_SLAR_NACK:
	case STATUS_M_SLAW_NACK:
		idev->msg_err = -ENXIO;
		mpfs_i2c_stop(idev);
		finish = true;
		break;
	case STATUS_M_SLAR_ACK:
		if (idev->msg_len > 1u)
		{
			ctrl = readl(idev->base + MPFS_I2C_CTRL);
			ctrl |= (1 << CTRL_AA);
			writel(ctrl, idev->base + MPFS_I2C_CTRL);
		}
		else if (1u == idev->msg_len)
		{
			ctrl = readl(idev->base + MPFS_I2C_CTRL);
			ctrl &= ~(1 << CTRL_AA);
			writel(ctrl, idev->base + MPFS_I2C_CTRL);
		}
		else {
			ctrl = readl(idev->base + MPFS_I2C_CTRL);
			ctrl |= (1 << CTRL_AA);
			writel(ctrl, idev->base + MPFS_I2C_CTRL);
			/* On the last byte to be transmitted, send STOP */
			mpfs_i2c_stop(idev);
			finish = true;
		}
		break;
	case STATUS_M_RX_DATA_ACKED:
		mpfs_i2c_empty_rx(idev);
		if (idev->msg_len >= 1) {
			ctrl = readl(idev->base + MPFS_I2C_CTRL);
			ctrl &= ~(1 << CTRL_AA);
			writel(ctrl, idev->base + MPFS_I2C_CTRL);
		}
		break;
	case STATUS_M_RX_DATA_NACKED:
		mpfs_i2c_empty_rx(idev);
		if (idev->msg_len == 0) {
			/* On the last byte to be transmitted, send STOP */
			mpfs_i2c_stop(idev);
			finish = true;
		}
		break;
	default:
		break;
	}

	if (finish) {
		complete(&idev->msg_complete);
	}

	return IRQ_HANDLED;
}

static irqreturn_t mpfs_i2c_isr(int irq, void *_dev)
{
	struct mpfs_i2c_dev *idev = _dev;
	irqreturn_t ret = IRQ_NONE;
	int si_bit = 0;

	si_bit = readl(idev->base + MPFS_I2C_CTRL);
	if (si_bit & (1 << CTRL_SI)) {
		idev->isr_status = readl(idev->base + MPFS_I2C_STATUS);
		ret = mpfs_i2c_handle_isr(irq, idev);
	}

	// Clear the si flag
	mpfs_i2c_int_clear(idev);

	return ret;
}

static int mpfs_i2c_xfer_msg(struct mpfs_i2c_dev *idev, struct i2c_msg *msg)
{
	u8 ctrl;
	unsigned long time_left;

	if (msg->len == 0)
		return -EINVAL;

	idev->addr = i2c_8bit_addr_from_msg(msg);
	idev->msg_len = msg->len;
	idev->buf = msg->buf;
	idev->msg_err = 0;
	idev->msg_read = (msg->flags & I2C_M_RD);

	reinit_completion(&idev->msg_complete);

	mpfs_i2c_core_enable(idev);

	ctrl = readl(idev->base + MPFS_I2C_CTRL);

	ctrl |= (1 << CTRL_STA);

	writel(ctrl, idev->base + MPFS_I2C_CTRL);

	time_left = wait_for_completion_timeout(&idev->msg_complete,
						MICROCHIP_I2C_TIMEOUT);

	if (time_left == 0)
		return -ETIMEDOUT;

	return idev->msg_err;
}

static int mpfs_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct mpfs_i2c_dev *idev = i2c_get_adapdata(adap);
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = mpfs_i2c_xfer_msg(idev, msgs++);
		if (ret)
			return ret;
	}

	return num;
}

static u32 mpfs_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm mpfs_i2c_algo = {
	.master_xfer = mpfs_i2c_xfer,
	.functionality = mpfs_i2c_func,
};

static int mpfs_i2c_probe(struct platform_device *pdev)
{
	struct mpfs_i2c_dev *idev = NULL;
	struct resource *res;
	int irq, ret;
	u32 val;

	idev = devm_kzalloc(&pdev->dev, sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	idev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(idev->base))
		return PTR_ERR(idev->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "missing interrupt resource\n");
		return irq;
	}

	idev->i2c_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(idev->i2c_clk)) {
		dev_err(&pdev->dev, "missing clock\n");
		return PTR_ERR(idev->i2c_clk);
	}

	idev->dev = &pdev->dev;
	init_completion(&idev->msg_complete);
	spin_lock_init(&idev->lock);

	val = device_property_read_u32(idev->dev, "clock-frequency",
				       &idev->bus_clk_rate);
	if (val) {
		dev_info(&pdev->dev, "default to 100kHz\n");
		idev->bus_clk_rate = 100000;	/* default clock rate */
	}

	if (idev->bus_clk_rate > 400000) {
		dev_err(&pdev->dev, "invalid clock-frequency %d\n",
			idev->bus_clk_rate);
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, irq, mpfs_i2c_isr,
			       IRQF_SHARED, pdev->name, idev);
	if (ret) {
		dev_err(&pdev->dev, "failed to claim irq %d\n", irq);
		return ret;
	}

	ret = clk_prepare_enable(idev->i2c_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		return ret;
	}

	ret = mpfs_i2c_init(idev);
	if (ret) {
		dev_err(&pdev->dev, "failed to program clock divider\n");
		return ret;
	}

	i2c_set_adapdata(&idev->adapter, idev);
	snprintf(idev->adapter.name, sizeof(idev->adapter.name),
		 "Microchip I2C hw bus");
	idev->adapter.owner = THIS_MODULE;
	idev->adapter.algo = &mpfs_i2c_algo;
	idev->adapter.dev.parent = &pdev->dev;
	idev->adapter.dev.of_node = pdev->dev.of_node;

	platform_set_drvdata(pdev, idev);

	ret = i2c_add_adapter(&idev->adapter);
	if (ret) {
		clk_disable_unprepare(idev->i2c_clk);
		return ret;
	}

	dev_info(&pdev->dev, "Microchip I2C Probe Complete\n");

	return 0;
}

static int mpfs_i2c_remove(struct platform_device *pdev)
{
	struct mpfs_i2c_dev *idev = platform_get_drvdata(pdev);

	clk_disable_unprepare(idev->i2c_clk);
	i2c_del_adapter(&idev->adapter);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id mpfs_i2c_of_match[] = {
	{ .compatible = "microchip,mpfs-i2c" },
	{ .compatible = "microchip,core-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, mpfs_i2c_of_match);

static struct platform_driver mpfs_i2c_driver = {
	.probe = mpfs_i2c_probe,
	.remove = mpfs_i2c_remove,
	.driver = {
		.name = "microchip-mpfs-i2c",
		.of_match_table = mpfs_i2c_of_match,
	},
};

module_platform_driver(mpfs_i2c_driver);

MODULE_DESCRIPTION("Microchip I2C bus driver");
MODULE_AUTHOR("Microchip Corporation");
MODULE_LICENSE("GPL v2");
