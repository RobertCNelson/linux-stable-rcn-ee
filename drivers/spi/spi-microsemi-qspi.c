/*
 * SPI controller driver for Microsemi PolarFire QSPI
 *
 * Copyright (c) 2018 Microsemi Corporation.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
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
#define MSS_QSPI_MAX_LEN		(0xffff)

#define MSS_QSPI_MAX_CS			(1)

#define MSS_QSPI_FIFO_DEPTH		(16)

#define MSS_QSPI_CLK_GEN_MAX		(15)
#define MSS_QSPI_CLK_GEN_MIN		(1)

#define MSS_QSPI_CONTROL_ENABLE		(1 << 0)
#define MSS_QSPI_CONTROL_MASTER		(1 << 1)
#define MSS_QSPI_CONTROL_XIP		(1 << 2)
#define MSS_QSPI_CONTROL_XIPADDR	(1 << 3)
#define MSS_QSPI_CONTROL_CLKIDLE	(1 << 10)
#define MSS_QSPI_CONTROL_SAMPLE_MSK     (3 << 11)
#define MSS_QSPI_CONTROL_MODE0		(1 << 13)
#define MSS_QSPI_CONTROL_MODE12_MSK	(3 << 14)
#define MSS_QSPI_CONTROL_FLAGSX4	(1 << 16)
#define MSS_QSPI_CONTROL_CLKRATE_MSK	(0xf << 24)

#define MSS_QSPI_FRAMES_TOTALBYTES_MSK	(0xffff < 0)
#define MSS_QSPI_FRAMES_CMDBYTES_MSK	(0xff < 16)
#define MSS_QSPI_FRAMES_QSPI		(1 << 25)
#define MSS_QSPI_FRAMES_IDLE_MSK	(0xf << 26)
#define MSS_QSPI_FRAMES_FLAGBYTE	(1 << 30)
#define MSS_QSPI_FRAMES_FLAGWORD	(1 << 31)

#define MSS_QSPI_IEN_TXDONE		(1 << 0)
#define MSS_QSPI_IEN_RXDONE		(1 << 1)
#define MSS_QSPI_IEN_RXAVAILABLE	(1 << 2)
#define MSS_QSPI_IEN_TXAVAILABLE	(1 << 3)
#define MSS_QSPI_IEN_RXFIFOEMPTY	(1 << 4)
#define MSS_QSPI_IEN_TXFIFOFULL		(1 << 5)

#define MSS_QSPI_STATUS_TXDONE		(1 << 0)
#define MSS_QSPI_STATUS_RXDONE		(1 << 1)
#define MSS_QSPI_STATUS_RXAVAILABLE	(1 << 2)
#define MSS_QSPI_STATUS_TXAVAILABLE	(1 << 3)
#define MSS_QSPI_STATUS_RXFIFOEMPTY	(1 << 4)
#define MSS_QSPI_STATUS_TXFIFOFULL	(1 << 5)
#define MSS_QSPI_STATUS_READY		(1 << 7)
#define MSS_QSPI_STATUS_FLAGSX4		(1 << 8)

#define MSS_QSPI_DA_EN_SSEL		(1 << 0)
#define MSS_QSPI_DA_OP_SSEL		(1 << 1)
#define MSS_QSPI_DA_EN_SCLK		(1 << 2)
#define MSS_QSPI_DA_OP_SCLK		(1 << 3)
#define MSS_QSPI_DA_EN_SDO_MSK		(0xf << 4)
#define MSS_QSPI_DA_OP_SDO_MSK		(0xf << 8)
#define MSS_QSPI_DA_OP_SDATA_MSK	(0xf << 12)
#define MSS_QSPI_DA_IP_SDI_MSK		(0xf << 16)
#define MSS_QSPI_DA_IP_SCLK		(1 << 21)
#define MSS_QSPI_DA_IP_SSEL		(1 << 22)
#define MSS_QSPI_DA_IDLE		(1 << 23)

#define MSS_QSPI_RXDATA_MSK		(0xff << 0)

#define MSS_QSPI_TXDATA_MSK		(0xff << 0)



/*
 * Private data structure for an SPI slave
 */
struct mss_spi_dsc {
	void __iomem *regs;
	int irq;
	struct clk *clk;
	u8 clk_gen;
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
#define MSS_QSPI_REG_CONTROL		(0x00)
#define MSS_QSPI_REG_FRAMES		(0x04)
#define MSS_QSPI_REG_IEN		(0x0c)
#define MSS_QSPI_REG_STATUS		(0x10)
#define MSS_QSPI_REG_DIRECT_ACCESS	(0x14)
#define MSS_QSPI_REG_UPPER_ACCESS	(0x18)
#define MSS_QSPI_REG_RX_DATA		(0x40)
#define MSS_QSPI_REG_TX_DATA		(0x44)
#define MSS_QSPI_REG_X4_RX_DATA		(0x48)
#define MSS_QSPI_REG_X4_TX_DATA		(0x4c)


static inline void mss_spi_hw_tfsz_set(struct mss_spi_dsc *s, int len);
static void mss_spi_enable_ints(struct mss_spi_dsc *s,
	struct platform_device *pdev);

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
	u8 data = 0;
	int i = 0;
	int count = min(s->rx_len, MSS_QSPI_FIFO_DEPTH);

	/* read whatever is in the FIFO out, up to a maximum of
	 * rx_len and fifo depth
	 */
	while ((i < count) &&
		!(mss_spi_rd(s, MSS_QSPI_REG_STATUS) &
		  MSS_QSPI_STATUS_RXFIFOEMPTY)) {
		data = mss_spi_rd(s, MSS_QSPI_REG_RX_DATA);

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
	count = min(s->tx_len, MSS_QSPI_FIFO_DEPTH);
	mss_spi_hw_tfsz_set(s, count);

	/* monitor the full next write bit */
	while ((i < count) &&
		(!(mss_spi_rd(s, MSS_QSPI_REG_STATUS) &
			MSS_QSPI_STATUS_TXFIFOFULL))) {
		byte = s->tx_buf ? *s->tx_buf++ : 0xaa;
		mss_spi_wr(s, MSS_QSPI_REG_TX_DATA, byte);
		i++;
	}

	s->tx_len -= i;
	s->pending += i;
}


static int mss_spi_hw_init(struct mss_spi_dsc *s, struct platform_device *pdev)
{
	unsigned int ret = 0;
	u32 control = 0;

	/* Set master mode, full qspi mode */
	control |= MSS_QSPI_CONTROL_MASTER | MSS_QSPI_CONTROL_MODE0 |
		MSS_QSPI_CONTROL_MODE12_MSK;

	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	mss_spi_enable_ints(s, pdev);

	control = mss_spi_rd(s, MSS_QSPI_REG_CONTROL);

	control |= MSS_QSPI_CONTROL_ENABLE;

	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	return ret;
}


static inline void mss_spi_activate_cs(struct mss_spi_dsc *s)
{
	u32 reg = mss_spi_rd(s, MSS_QSPI_REG_DIRECT_ACCESS);

	reg = MSS_QSPI_DA_OP_SSEL | MSS_QSPI_DA_EN_SSEL;
	mss_spi_wr(s, MSS_QSPI_REG_DIRECT_ACCESS, reg);
}

static inline void mss_spi_deactivate_cs(struct mss_spi_dsc *s)
{
	u32 reg =  mss_spi_rd(s, MSS_QSPI_REG_DIRECT_ACCESS);

	reg &= ~(MSS_QSPI_DA_OP_SSEL | MSS_QSPI_DA_EN_SSEL);
	mss_spi_wr(s, MSS_QSPI_REG_DIRECT_ACCESS, reg);
}


/*
 * Set controller clock rate
 * @s		slave
 * @returns	0->good,!=0->bad
 */
static inline int mss_spi_clk_gen_set(struct mss_spi_dsc *s)
{
	u32 control = mss_spi_rd(s, MSS_QSPI_REG_CONTROL);

	control &= ~MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	control |= s->clk_gen << 24;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);
	control |= MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);
	return 0;
}

static void mss_spi_enable_ints(struct mss_spi_dsc *s,
	struct platform_device *pdev)
{
	u32 control;
	u32 mask = MSS_QSPI_IEN_TXDONE |
		   MSS_QSPI_IEN_RXDONE |
		   MSS_QSPI_IEN_RXAVAILABLE;

	control = mss_spi_rd(s, MSS_QSPI_REG_CONTROL);
	control &= ~MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	mss_spi_wr(s, MSS_QSPI_REG_IEN, mask);
	control |= MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);
}

static void mss_spi_disable_ints(struct mss_spi_dsc *s)
{
	u32 control;
	u32 ccache;
	u32 mask = 0;

	control = mss_spi_rd(s, MSS_QSPI_REG_CONTROL);
	ccache = control;
	control &= ~MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	mss_spi_wr(s, MSS_QSPI_REG_IEN, mask);
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, ccache);
}

/*
 * Set transfer length
 * @s		slave
 * @len		transfer size
 */
static inline void mss_spi_hw_tfsz_set(struct mss_spi_dsc *s, int len)
{
	u32 frames = 0;

	/*
	 * Disable the SPI controller. Writes to transfer length have
	 * no effect when the controller is enabled.
	 */
	len &= MSS_QSPI_FRAMES_TOTALBYTES_MSK;
	frames = len | MSS_QSPI_FRAMES_QSPI | MSS_QSPI_FRAMES_FLAGBYTE;

	mss_spi_wr(s, MSS_QSPI_REG_FRAMES, frames);
}

/*
 * Set SPI mode
 * @s		slave
 * @mode	mode
 * @returns	0->good;!=0->bad
 */
static inline int mss_spi_hw_mode_set(struct mss_spi_dsc *s, unsigned int mode)
{
	u32 control = mss_spi_rd(s, MSS_QSPI_REG_CONTROL);

	control &= ~MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	control &= ~MSS_QSPI_CONTROL_SAMPLE_MSK;
	control |= (0x10 << 11);

	if (mode & SPI_CPOL)
		control |= MSS_QSPI_CONTROL_CLKIDLE;
	else
		control &= ~MSS_QSPI_CONTROL_CLKIDLE;

	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

	control |= MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);
	return 0;
}

/*
 * Shut down the SPI controller
 * @s		SPI slave
 */
static void mss_spi_hw_release(struct mss_spi_dsc *s)
{
	u32 control = mss_spi_rd(s, MSS_QSPI_REG_CONTROL);

	mss_spi_disable_ints(s);

	/*
	 * Disable the SPI controller
	 */
	control &= ~MSS_QSPI_CONTROL_ENABLE;
	mss_spi_wr(s, MSS_QSPI_REG_CONTROL, control);

}


/*
 * Interrupt handler
 */
static irqreturn_t mss_spi_interrupt(int irq, void *dev_id)
{
	struct spi_master *master = dev_id;
	struct mss_spi_dsc *s = spi_master_get_devdata(master);
	irqreturn_t ret = IRQ_HANDLED;

	int intfield = mss_spi_rd(s, MSS_QSPI_REG_STATUS) & 0x3f;
	int ienfield = mss_spi_rd(s, MSS_QSPI_REG_IEN);

	/* Interrupt line may be shared and not for us at all */
	if ((intfield & ienfield) == 0)
		return IRQ_NONE;

	mss_spi_wr(s, MSS_QSPI_REG_STATUS, 0xff);
	mss_spi_wr(s, MSS_QSPI_REG_STATUS, 0);
	mss_spi_wr(s, MSS_QSPI_REG_STATUS, 0xff);

	/* clear the interrupt conditions */
	if (intfield & MSS_QSPI_IEN_TXDONE) {
		mss_spi_wr(s, MSS_QSPI_REG_STATUS,
			MSS_QSPI_IEN_TXDONE);

		/* check if we have more data to write */
		if (s->tx_len)
			mss_spi_wr_fifo(s);
	}

	if (intfield & MSS_QSPI_IEN_RXDONE) {
		mss_spi_wr(s, MSS_QSPI_REG_STATUS,
			MSS_QSPI_IEN_RXDONE);

		/* if rx_len is 0 and tx_len is 0, then wake up completion */
		if ((!s->rx_len) && (!s->tx_len))
			complete(&master->xfer_completion);

	}

	if (intfield & MSS_QSPI_IEN_RXAVAILABLE) {
		mss_spi_wr(s, MSS_QSPI_REG_STATUS,
			MSS_QSPI_IEN_RXAVAILABLE);
			mss_spi_rd_fifo(s);
	}

	if (intfield & MSS_QSPI_IEN_TXAVAILABLE) {
		mss_spi_wr(s, MSS_QSPI_REG_STATUS,
			MSS_QSPI_IEN_TXAVAILABLE);
	}

	if (intfield & MSS_QSPI_IEN_RXFIFOEMPTY) {
		mss_spi_wr(s, MSS_QSPI_REG_STATUS,
			MSS_QSPI_IEN_RXFIFOEMPTY);
	}

	if (intfield & MSS_QSPI_IEN_TXFIFOFULL) {
		mss_spi_wr(s, MSS_QSPI_REG_STATUS,
			MSS_QSPI_IEN_TXFIFOFULL);
	}

	return ret;
}


static int mss_spi_transfer_one_irq(struct spi_master *master,
				    struct spi_device *spi,
				    struct spi_transfer *tfr)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);

	/* fill in registers and fifos */
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
		dev_err(&spi->dev, "%s: %ld hz too slow for spi\n",
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

	if (spi_hz < DIV_ROUND_UP(clk_hz, 2 * MSS_QSPI_CLK_GEN_MAX)) {
		dev_err(&spi->dev,
			"%s: slave device %ld hz too slow for system clock %ld Hz, min speed = %ld)\n",
			__func__, spi_hz, clk_hz, clk_hz/30);
		return -EINVAL;
	}

	clk_gen = DIV_ROUND_UP(clk_hz, 2 * spi_hz);
	s->clk_gen = clk_gen;

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

	/* run in interrupt mode */
	return mss_spi_transfer_one_irq(master, spi, tfr);
}


static int mss_spi_prepare_message(struct spi_master *master,
				   struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct mss_spi_dsc *s = spi_master_get_devdata(master);
	int ret = 0;

	mss_spi_activate_cs(s);

	if (mss_spi_hw_mode_set(s, spi->mode)) {
		dev_err(&spi->dev, "unsupported mode: %x\n", spi->mode);
		ret = -EINVAL;
	}

	return ret;
}


static int mss_spi_unprepare_message(struct spi_master *master,
				     struct spi_message *msg)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);

	mss_spi_deactivate_cs(s);
	return 0;
}

static void mss_spi_handle_err(struct spi_master *master,
				     struct spi_message *msg)
{
	struct mss_spi_dsc *s = spi_master_get_devdata(master);

	mss_spi_deactivate_cs(s);
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
		master->num_chipselect = MSS_QSPI_MAX_CS;
	else
		master->num_chipselect = (u16)val;

	/* initialize the controller hardware */
	if (mss_spi_hw_init(s, pdev)) {
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
	dev_info(&pdev->dev, "Microsemi QSPI Controller %d up\n",
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

	/* release kernel resources */
	spi_unregister_master(master);
	free_irq(s->irq, s);
	iounmap(s->regs);
	spi_master_put(master);
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
	{ .compatible = "microsemi,ms-pf-mss-qspi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mss_spi_dt_ids);
#endif

static struct platform_driver mss_spi_driver = {
	.probe = mss_spi_probe,
	.driver = {
		.name = "microsemi-mss-qspi",
		.pm = MICROSEMI_SPI_PM_OPS,
		.of_match_table = of_match_ptr(mss_spi_dt_ids),
		.owner = THIS_MODULE,
	},
	.remove = mss_spi_remove,
};
module_platform_driver(mss_spi_driver);
MODULE_AUTHOR("Microsemi Corporation");
MODULE_DESCRIPTION("Microsemi MSS QSPI driver");
MODULE_LICENSE("GPL");
