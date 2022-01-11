/*
 * SPI controller driver for Microsemi MSS SPI
 *
 * Copyright (c) 2018 Microsemi Corporation.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Based on spi_mss.c by Emcraft Systems
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/of.h>

/*
 * Maximum size we allow per xfer (limited with txrxdf_count register)
 */
#define MSS_SPI_MAX_LEN			(0xffff)

#define MSS_SPI_MAX_CS			(8)

#define MSS_SPI_FIFO_DEPTH		(32)

#define MSS_SPI_CLK_GEN_MAX		(255)
#define MSS_SPI_CLK_GEN_P2_MAX		(15)
#define MSS_SPI_CLK_GEN_MIN		(0)

#define MSS_SPI_CONTROL_ENABLE		(1 << 0)
#define MSS_SPI_CONTROL_MASTER		(1 << 1)
#define MSS_SPI_CONTROL_PROTO_MSK	(3 << 2)
#define MSS_SPI_CONTROL_PROTO_MOTO	(0 << 2)
#define MSS_SPI_CONTROL_RX_DATA_INT     (1 << 4)
#define MSS_SPI_CONTROL_TX_DATA_INT     (1 << 5)
#define MSS_SPI_CONTROL_RX_OVER_INT     (1 << 6)
#define MSS_SPI_CONTROL_TX_UNDER_INT    (1 << 7)
#define MSS_SPI_CONTROL_CNT_MSK		(0xffff << 8)
#define MSS_SPI_CONTROL_CNT_SHF		(8)
#define MSS_SPI_CONTROL_SPO		(1 << 24)
#define MSS_SPI_CONTROL_SPH		(1 << 25)
#define MSS_SPI_CONTROL_SPS		(1 << 26)
#define MSS_SPI_CONTROL_FRAMEURUN       (1 << 27)
#define MSS_SPI_CONTROL_CLKMODE		(1 << 28)
#define MSS_SPI_CONTROL_BIGFIFO		(1 << 29)
#define MSS_SPI_CONTROL_OENOFF		(1 << 30)
#define MSS_SPI_CONTROL_RESET		(1 << 31)

#define MSS_SPI_STATUS_ACTIVE			(1 << 14)
#define MSS_SPI_STATUS_SSEL			(1 << 13)
#define MSS_SPI_STATUS_FRAMESTART		(1 << 12)
#define MSS_SPI_STATUS_TXFIFO_EMPTY_NEXT_READ	(1 << 11)
#define MSS_SPI_STATUS_TXFIFO_EMPTY		(1 << 10)
#define MSS_SPI_STATUS_TXFIFO_FULL_NEXT_WRITE	(1 << 9)
#define MSS_SPI_STATUS_TXFIFO_FULL		(1 << 8)
#define MSS_SPI_STATUS_RXFIFO_EMPTY_NEXT_READ	(1 << 7)
#define MSS_SPI_STATUS_RXFIFO_EMPTY		(1 << 6)
#define MSS_SPI_STATUS_RXFIFO_FULL_NEXT_WRITE	(1 << 5)
#define MSS_SPI_STATUS_RXFIFO_FULL		(1 << 4)
#define MSS_SPI_STATUS_TX_UNDERRUN		(1 << 3)
#define MSS_SPI_STATUS_RX_OVERFLOW		(1 << 2)
#define MSS_SPI_STATUS_RXDAT_RCED		(1 << 1)
#define MSS_SPI_STATUS_TXDAT_SENT		(1 << 0)

#define MSS_SPI_INT_TXDONE		(1 << 0)
#define MSS_SPI_INT_RXRDY		(1 << 1)
#define MSS_SPI_INT_RX_CH_OVRFLW	(1 << 2)
#define MSS_SPI_INT_TX_CH_UNDRUN	(1 << 3)
#define MSS_SPI_INT_CMD			(1 << 4)
#define MSS_SPI_INT_SSEND		(1 << 5)

/*
 * Private data structure for an SPI slave
 */
struct mss_spi_dsc {
	void __iomem *regs;
	int irq;
	struct clk *clk;
	u8 clk_gen;
	u8 clk_mode;
	const u8 *tx_buf;
	u8 *rx_buf;
	int tx_len;
	int rx_len;
	int pending;
};

/*
 * Description of the the SmartFusion SPI hardware interfaces.
 * This is a 1-to-1 mapping of Actel's documentation onto a C structure.
 * Refer to SmartFusion Data Sheet for details.
 */
#define MSS_SPI_REG_CONTROL      (0x00)
#define MSS_SPI_REG_TXRXDF_SIZE  (0x04)
#define MSS_SPI_REG_STATUS       (0x08)
#define MSS_SPI_REG_INT_CLEAR    (0x0c)
#define MSS_SPI_REG_RX_DATA      (0x10)
#define MSS_SPI_REG_TX_DATA      (0x14)
#define MSS_SPI_REG_CLK_GEN      (0x18)
#define MSS_SPI_REG_SLAVE_SELECT (0x1c)
#define  MSS_SPI_SSEL_MASK	 (0xff)
#define  MSS_SPI_DIRECT	   (0x100)
#define  MSS_SPI_SSELOUT	  (0x200)
#define MSS_SPI_REG_MIS	  (0x20)
#define MSS_SPI_REG_RIS	  (0x24)
#define MSS_SPI_REG_CONTROL2     (0x28)
#define MSS_SPI_REG_COMMAND      (0x2c)
#define MSS_SPI_REG_PKTSIZE      (0x30)
#define MSS_SPI_REG_CMD_SIZE     (0x34)
#define MSS_SPI_REG_HWSTATUS     (0x38)
#define MSS_SPI_REG_STAT8	(0x3c)
#define MSS_SPI_REG_CTRL2	(0x48)
#define MSS_SPI_REG_FRAMESUP     (0x50)

static inline void mss_spi_hw_tfsz_set(struct mss_spi_dsc *s, int len);
static void mss_spi_enable_ints(struct mss_spi_dsc *s);

static inline u32 mss_spi_rd(struct mss_spi_dsc *s, unsigned int reg)
{
	return readl(s->regs + reg);
}

static inline void mss_spi_wr(struct mss_spi_dsc *s, unsigned int reg, u32 val)
{
	writel(val, s->regs + reg);
}

static inline void mss_spi_rd_fifo(struct mss_spi_dsc *s)
{
	u8 data;
	int i = 0;
	int count = min(s->rx_len, MSS_SPI_FIFO_DEPTH);

	/* read whatever is in the FIFO out, up to a maximum of
	 * rx_len and fifo depth
	 */
	while ((i < count) &&
		!(mss_spi_rd(s, MSS_SPI_REG_STATUS) &
		  MSS_SPI_STATUS_RXFIFO_EMPTY)) {
		data = mss_spi_rd(s, MSS_SPI_REG_RX_DATA);

		if (s->rx_buf)
		*s->rx_buf++ = data;
		i++;
	}
	s->rx_len -= i;
	s->pending -= i;
}

static inline void mss_spi_wr_fifo(struct mss_spi_dsc *s)
{
	u8 byte;
	int count;
	int i = 0;

	/* write a max of the tx len and the fifo depth in this burst */
	count = min(s->tx_len, MSS_SPI_FIFO_DEPTH);
	mss_spi_hw_tfsz_set(s, count);

	/* monitor the full next write bit */
	while ((i < count) &&
		(!(mss_spi_rd(s, MSS_SPI_REG_STATUS) &
			MSS_SPI_STATUS_TXFIFO_FULL))) {
		byte = s->tx_buf ? *s->tx_buf++ : 0xaa;
		mss_spi_wr(s, MSS_SPI_REG_TX_DATA, byte);
		i++;
	}

	s->tx_len -= i;
	s->pending += i;
}


static inline int mss_spi_hw_bt_set(struct mss_spi_dsc *s, int bt)
{
	int ret = 0;
	u32 control;
	/* disable the SPI controller. Writes to data frame size have
	 * no effect when the controller is enabled.
	 */
	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	/* set the new data frame size. */
	mss_spi_wr(s, MSS_SPI_REG_TXRXDF_SIZE, bt);

	/* re-enable the SPI controller */
	control |= MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	return ret;
}

static int mss_spi_hw_init(struct mss_spi_dsc *s)
{
	unsigned int ret = 0;
	u32 control = 0;

	/* Set master mode */
	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	control |= MSS_SPI_CONTROL_MASTER;

	/* using Motorola SPI mode */
	control &= ~MSS_SPI_CONTROL_PROTO_MSK;
	control |= MSS_SPI_CONTROL_PROTO_MOTO;

	mss_spi_hw_bt_set(s, 8);
	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	/*
	 * Set-up the controller so it doesn't remove
	 * Chip Select until the entire message has been transferred,
	 * even if at some points TX FIFO becomes empty.
	 *
	 * set fifo depth to 32 (for 8 bit xfer)
	 */
	control |= MSS_SPI_CONTROL_SPS | MSS_SPI_CONTROL_BIGFIFO;

	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	mss_spi_enable_ints(s);
	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);

	control &= ~MSS_SPI_CONTROL_RESET;
	control |= MSS_SPI_CONTROL_ENABLE;

	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	return ret;
}


/*
 * Set chip select
 * @s		slave
 * @cs		chip select: [0..7]->slave, otherwise->deselect all
 * @returns	0->good,!=0->bad
 */
static inline int mss_spi_hw_cs_set(struct mss_spi_dsc *s, int cs)
{
	unsigned int v = (cs >= 0 && cs <= 7) ? (1 << cs) : 0;
	u32 reg = mss_spi_rd(s, MSS_SPI_REG_SLAVE_SELECT);

	reg &= ~MSS_SPI_SSEL_MASK;
	reg |= v;

	mss_spi_wr(s, MSS_SPI_REG_SLAVE_SELECT, reg);

	return 0;
}

static inline void mss_spi_hw_cs_enable_direct_mode(struct mss_spi_dsc *s)
{
	u32 reg = mss_spi_rd(s, MSS_SPI_REG_SLAVE_SELECT);

	reg |= MSS_SPI_DIRECT;
	mss_spi_wr(s, MSS_SPI_REG_SLAVE_SELECT, reg);
}

static inline void mss_spi_hw_cs_disable_direct_mode(struct mss_spi_dsc *s)
{
	u32 reg = mss_spi_rd(s, MSS_SPI_REG_SLAVE_SELECT);

	reg &= ~MSS_SPI_DIRECT;
	mss_spi_wr(s, MSS_SPI_REG_SLAVE_SELECT, reg);
}

static inline void mss_spi_activate_cs(struct mss_spi_dsc *s)
{
	u32 reg = mss_spi_rd(s, MSS_SPI_REG_SLAVE_SELECT);

	reg |= MSS_SPI_SSELOUT;
	mss_spi_wr(s, MSS_SPI_REG_SLAVE_SELECT, reg);
}

static inline void mss_spi_deactivate_cs(struct mss_spi_dsc *s)
{
	u32 reg = mss_spi_rd(s, MSS_SPI_REG_SLAVE_SELECT);

	reg &= ~MSS_SPI_SSELOUT;
	mss_spi_wr(s, MSS_SPI_REG_SLAVE_SELECT, reg);
}

/*
 * Set controller clock rate
 * @s		slave
 * @returns	0->good,!=0->bad
 */
static inline int mss_spi_clk_gen_set(struct mss_spi_dsc *s)
{
	u32 control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	if (s->clk_mode)
		control |= MSS_SPI_CONTROL_CLKMODE;
	else
		control &= ~MSS_SPI_CONTROL_CLKMODE;

	mss_spi_wr(s, MSS_SPI_REG_CLK_GEN, s->clk_gen);
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control | MSS_SPI_CONTROL_ENABLE);
	return 0;
}

static void mss_spi_enable_ints(struct mss_spi_dsc *s)
{
	u32 control;
	u32 mask = MSS_SPI_CONTROL_RX_DATA_INT |
		   MSS_SPI_CONTROL_TX_DATA_INT |
		   MSS_SPI_CONTROL_RX_OVER_INT |
		   MSS_SPI_CONTROL_TX_UNDER_INT;

	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	control |= mask;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
	control |= MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
}

static void mss_spi_disable_ints(struct mss_spi_dsc *s)
{
	u32 control;
	u32 mask = MSS_SPI_CONTROL_RX_DATA_INT |
		   MSS_SPI_CONTROL_TX_DATA_INT |
		   MSS_SPI_CONTROL_RX_OVER_INT |
		   MSS_SPI_CONTROL_TX_UNDER_INT;

	mask = ~mask;
	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	control &= mask;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control | MSS_SPI_CONTROL_ENABLE);
}

/*
 * Set transfer length
 * @s		slave
 * @len		transfer size
 */
static inline void mss_spi_hw_tfsz_set(struct mss_spi_dsc *s, int len)
{
	u32 control;
	u16 lenpart;

	/*
	 * Disable the SPI controller. Writes to transfer length have
	 * no effect when the controller is enabled.
	 */
	control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);
	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	/*
	 * Set the new transfer size.
	 */

	/*
	 * write the lower 16 bits first
	 */
	lenpart = len & 0xffff;

	control &= ~MSS_SPI_CONTROL_CNT_MSK;
	control |= lenpart << MSS_SPI_CONTROL_CNT_SHF;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	/*
	 * write the upper 16 bits now
	 */
	lenpart = len & 0xffff0000;
	mss_spi_wr(s, MSS_SPI_REG_FRAMESUP, lenpart);

	/*
	 * Re-enable the SPI controller
	 */
	control |= MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
}

/*
 * Set SPI mode
 * @s		slave
 * @mode	mode
 * @returns	0->good;!=0->bad
 */
static inline int mss_spi_hw_mode_set(struct mss_spi_dsc *s, unsigned int mode)
{
	u32 control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);

	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	/* set the mode */
	if (mode & SPI_CPHA)
		control |= MSS_SPI_CONTROL_SPH;
	else
		control &= ~MSS_SPI_CONTROL_SPH;

	if (mode & SPI_CPOL)
		control |= MSS_SPI_CONTROL_SPO;
	else
		control &= ~MSS_SPI_CONTROL_SPO;

	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);

	control |= MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
	return 0;
}

/*
 * Shut down the SPI controller
 * @s		SPI slave
 */
static void mss_spi_hw_release(struct mss_spi_dsc *s)
{
	u32 control = mss_spi_rd(s, MSS_SPI_REG_CONTROL);

	/*
	 * Disable the SPI controller
	 */
	control &= ~MSS_SPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_SPI_REG_CONTROL, control);
}


/*
 * Interrupt handler
 */
static irqreturn_t mss_spi_interrupt(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct mss_spi_dsc *s = spi_master_get_devdata(master);
	irqreturn_t ret = IRQ_HANDLED;

	int intfield = mss_spi_rd(s, MSS_SPI_REG_MIS) & 0xf;

	/* Interrupt line may be shared and not for us at all */
	if (intfield == 0)
		return IRQ_NONE;

	/* clear the interrupt conditions */
	if (intfield & MSS_SPI_INT_TXDONE) {
		mss_spi_wr(s, MSS_SPI_REG_INT_CLEAR,
			MSS_SPI_INT_TXDONE);
		/* check if we have data to read */
		if (s->rx_len)
			mss_spi_rd_fifo(s);
		/* check if we have data to write */
		if (s->tx_len)
			mss_spi_wr_fifo(s);

		/* if rx_len is 0 then wake up completion */
		if (!s->rx_len)
			complete(&master->xfer_completion);
	}

	if (intfield & MSS_SPI_INT_RXRDY)
		mss_spi_wr(s, MSS_SPI_REG_INT_CLEAR, MSS_SPI_INT_RXRDY);

	if (intfield & MSS_SPI_INT_RX_CH_OVRFLW) {
		mss_spi_wr(s, MSS_SPI_REG_INT_CLEAR,
			MSS_SPI_INT_RX_CH_OVRFLW);
		complete(&master->xfer_completion);
		dev_err(&master->dev,
			"%s: RX OVERFLOW: rxlen: %d, txlen: %d\n", __func__,
			s->rx_len, s->tx_len);
	}

	if (intfield & MSS_SPI_INT_TX_CH_UNDRUN) {
		mss_spi_wr(s, MSS_SPI_REG_INT_CLEAR,
			MSS_SPI_INT_TX_CH_UNDRUN);
		complete(&master->xfer_completion);
		dev_err(&master->dev,
			"%s: TX UNDERFLOW: rxlen: %d, txlen: %d\n", __func__,
			s->rx_len, s->tx_len);
	}


	return ret;
}


static int mss_spi_transfer_one_irq(struct spi_master *master,
				    struct spi_device *spi,
				    struct spi_transfer *tfr)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);

	/* enable interrupts and fill in registers and fifos */
	if (s->tx_len)
		mss_spi_wr_fifo(s);

	return 1;
}

static int mss_spi_transfer_one(struct spi_master *master,
				struct spi_device *spi,
				struct spi_transfer *tfr)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);
	unsigned long spi_hz, clk_hz, clk_gen;

	/* set clock */
	clk_hz = clk_get_rate(s->clk);

	/* find suitable clock */
	spi_hz = min(tfr->speed_hz, master->max_speed_hz);

	if ((spi_hz == 0) || (spi_hz < master->min_speed_hz)) {
		dev_err(&spi->dev, "%s: %ld hz too slow\n",
			__func__, spi_hz);
		return -EINVAL;
	}

	/* calculate clock divider */
	if (spi_hz >= clk_hz / 2) {
		dev_err(&spi->dev,
			"%s: %ld hz too fast for system clock %ld Hz\n",
			__func__, spi_hz, clk_hz);
		return -EINVAL; /* spi rate too fast for chip */
	}

	clk_gen = DIV_ROUND_UP(clk_hz, 2 * spi_hz) - 1;
	if ((clk_gen > MSS_SPI_CLK_GEN_MAX) ||
	    (clk_gen <= MSS_SPI_CLK_GEN_MIN)) {
		/* use alternate mode */
		clk_gen = DIV_ROUND_UP(clk_hz, spi_hz);
		clk_gen = fls(clk_gen) - 1;
		if ((clk_gen > MSS_SPI_CLK_GEN_P2_MAX) ||
		    (clk_gen < MSS_SPI_CLK_GEN_MIN))
			return -EINVAL; /* give up */
		s->clk_gen = clk_gen;
		s->clk_mode = 0;
	} else {
		s->clk_gen = clk_gen;
		s->clk_mode = 1;
	}

	/* calculate spi_hz obtained */
	if (s->clk_mode)
		spi_hz = clk_hz / (2 * (s->clk_gen + 1));
	else
		spi_hz = clk_hz / (2 ^ (s->clk_gen + 1));

	if (mss_spi_clk_gen_set(s)) {
		dev_err(&spi->dev, "can't set clk divider\n");
		return -EINVAL;
	}

	/* set transmit buffers and length */
	s->tx_buf = tfr->tx_buf;
	s->rx_buf = tfr->rx_buf;
	s->tx_len = tfr->len;
	s->rx_len = tfr->len;
	s->pending = 0;

	mss_spi_hw_tfsz_set(s, (s->tx_len > MSS_SPI_FIFO_DEPTH)
			? MSS_SPI_FIFO_DEPTH : s->tx_len);

	/* run in interrupt mode */
	return mss_spi_transfer_one_irq(master, spi, tfr);
}


static int mss_spi_prepare_message(struct spi_master *master,
				   struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct mss_spi_dsc *s = spi_master_get_devdata(master);
	int ret = 0;

	/*
	 * set for this message: frame size, clock, slave select, mode
	 */
	if (mss_spi_hw_bt_set(s, 8)) {
		dev_err(&spi->dev, "unsupported frame size: %d\n",
			8);
		ret = -EINVAL;
		goto done;
	}

	if (mss_spi_hw_cs_set(s, spi->chip_select)) {
		dev_err(&spi->dev, "incorrect chip select: %d\n",
			spi->chip_select);
		ret = -EINVAL;
		goto done;
	}

	mss_spi_hw_cs_enable_direct_mode(s);
	mss_spi_activate_cs(s);

	if (mss_spi_hw_mode_set(s, spi->mode)) {
		dev_err(&spi->dev, "unsupported mode: %x\n", spi->mode);
		ret = -EINVAL;
		goto done;
	}

done:
	return ret;
}


static int mss_spi_unprepare_message(struct spi_master *master,
				     struct spi_message *msg)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);

	mss_spi_deactivate_cs(s);
	mss_spi_hw_cs_disable_direct_mode(s);

	return 0;
}

static void mss_spi_handle_err(struct spi_master *master,
				     struct spi_message *msg)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);
	mss_spi_deactivate_cs(s);
	mss_spi_hw_cs_disable_direct_mode(s);
}


/*
 * Instantiate a new instance of the SPI controller
 * @dev		SPI controller platform device
 * @returns	0->success, <0->error code
 */
static int mss_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master = NULL;
	struct mss_spi_dsc *s = NULL;
	struct resource *res;
	int ret = 0;
	int err;
	u32 val;

	/* allocate a spi master */
	master = spi_alloc_master(&pdev->dev, sizeof(*s));
	if (!master) {
		dev_err(&pdev->dev, "unable to allocate master for SPI controller\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);
	master->mode_bits = SPI_CPOL | SPI_CPHA;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->num_chipselect = -1;
	master->transfer_one = mss_spi_transfer_one;
	master->handle_err = mss_spi_handle_err;
	master->prepare_message = mss_spi_prepare_message;
	master->unprepare_message = mss_spi_unprepare_message;

	/* set up of_node so slaves can enumerate */
	master->dev.of_node = pdev->dev.of_node;

	/* pointer to the controller-specific data structure */
	s = spi_master_get_devdata(master);

	/*
	 * Get platform data
	 */

	/* get the register bases, and IRQ from the platform device */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no register base for SPI controller\n");
		ret = -ENXIO;
		goto error_release_nothing;
	}

	/* map in the controller registers */
	s->regs = ioremap(res->start, resource_size(res));
	if (!s->regs) {
		dev_err(&pdev->dev,
			"unable to map registers for SPI controller, base=%08llx\n",
			res->start);
		ret = -EINVAL;
		goto error_release_master;
	}

	/* get irq */
	s->irq = platform_get_irq(pdev, 0);
	if (s->irq <= 0) {
		dev_err(&pdev->dev,
			"invalid IRQ %d for SPI controller\n",
			s->irq);
		ret = -ENXIO;
		goto error_release_regs;
	}

	err = devm_request_irq(&pdev->dev, s->irq,
				mss_spi_interrupt,
				IRQF_SHARED,
				dev_name(&pdev->dev), master);
	if (err) {
		dev_err(&pdev->dev, "could not request irq: %d\n", err);
		goto error_release_regs;
	}

	s->clk = devm_clk_get(&pdev->dev, NULL);
	if ((!s->clk) || (IS_ERR(s->clk))) {
		err = PTR_ERR(s->clk);
		dev_err(&pdev->dev, "could not get clk: %d\n", err);
		goto error_release_regs;
	}

	err = clk_prepare_enable(s->clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		return err;
	}

	/* get master's max spi clock rate  from DT */
	err = of_property_read_u32(pdev->dev.of_node,
		"spi-max-frequency",
		&master->max_speed_hz);
	if (err)
		/* default to sensible clock */
		master->max_speed_hz = 140000000;

	/* get master's num-cs */
	err = of_property_read_u32(pdev->dev.of_node, "num-cs", &val);
	if (err)
		master->num_chipselect = MSS_SPI_MAX_CS;
	else
		master->num_chipselect = (u16)val;

	/* initialize the controller hardware */
	if (mss_spi_hw_init(s)) {
		dev_err(&pdev->dev,
			"unable to initialize hardware for SPI controller\n");
		ret = -ENXIO;
		goto error_release_irq;
	}

	/* register the SPI controller */
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register master for SPI controller\n");
		goto error_release_hardware;
	}

	/* if we are here, we are successful */
	dev_info(&pdev->dev, "Microsemi SPI Controller %d up\n",
		master->bus_num);

	goto done;

	/* error processing */
error_release_hardware:
	mss_spi_hw_release(s);
error_release_irq:
	free_irq(s->irq, s);
error_release_regs:
	iounmap(s->regs);
error_release_master:
	spi_master_put(master);
	platform_set_drvdata(pdev, NULL);
error_release_nothing:
done:
	return ret;
}

/*
 * Shutdown of an instance of the controller device
 * @dev		SPI controller platform device
 * @returns	0->success, <0->error code
 */
static int mss_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master  = platform_get_drvdata(pdev);
	struct mss_spi_dsc *s = spi_master_get_devdata(master);

	mss_spi_disable_ints(s);
	/* release kernel resources */
	spi_unregister_master(master);
	free_irq(s->irq, s);
	iounmap(s->regs);
	spi_master_put(master);
	clk_disable_unprepare(s->clk);
	platform_set_drvdata(pdev, NULL);

	/* shut the hardware down */
	mss_spi_hw_release(s);

	return 0;
}

#define MICROSEMI_SPI_PM_OPS (NULL)

/*
 * Platform driver data structure
 */


#if defined(CONFIG_OF)
static const struct of_device_id mss_spi_dt_ids[] = {
	{ .compatible = "microsemi,ms-pf-mss-spi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mss_spi_dt_ids);
#endif

static struct platform_driver mss_spi_driver = {
	.probe = mss_spi_probe,
	.driver = {
		.name = "microsemi-mss-spi",
		.pm = MICROSEMI_SPI_PM_OPS,
		.of_match_table = of_match_ptr(mss_spi_dt_ids),
		.owner = THIS_MODULE,
	},
	.remove = mss_spi_remove,
};
module_platform_driver(mss_spi_driver);
MODULE_AUTHOR("Microsemi Corporation");
MODULE_DESCRIPTION("Microsemi MSS SPI driver");
MODULE_LICENSE("GPL");
