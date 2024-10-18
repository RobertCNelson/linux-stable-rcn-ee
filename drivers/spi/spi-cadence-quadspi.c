// SPDX-License-Identifier: GPL-2.0-only
//
// Driver for Cadence QSPI Controller
//
// Copyright Altera Corporation (C) 2012-2014. All rights reserved.
// Copyright Intel Corporation (C) 2019-2020. All rights reserved.
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/sys_soc.h>
#include <linux/timer.h>

#define CQSPI_NAME			"cadence-qspi"
#define CQSPI_MAX_CHIPSELECT		16

/* Quirks */
#define CQSPI_NEEDS_WR_DELAY		BIT(0)
#define CQSPI_DISABLE_DAC_MODE		BIT(1)
#define CQSPI_SUPPORT_EXTERNAL_DMA	BIT(2)
#define CQSPI_NO_SUPPORT_WR_COMPLETION	BIT(3)
#define CQSPI_SLOW_SRAM		BIT(4)
#define CQSPI_NEEDS_APB_AHB_HAZARD_WAR	BIT(5)

/* Capabilities */
#define CQSPI_SUPPORTS_OCTAL		BIT(0)

#define CQSPI_OP_WIDTH(part) ((part).nbytes ? ilog2((part).buswidth) : 0)

enum {
	CLK_QSPI_APB = 0,
	CLK_QSPI_AHB,
	CLK_QSPI_NUM,
};

struct cqspi_st;

struct phy_setting {
	u8		rx;
	u8		tx;
	u8		read_delay;
};

struct cqspi_flash_pdata {
	struct cqspi_st		*cqspi;
	u32			clk_rate;
	u32			read_delay;
	u32			tshsl_ns;
	u32			tsd2d_ns;
	u32			tchsh_ns;
	u32			tslch_ns;
	u8			inst_width;
	u8			addr_width;
	u8			data_width;
	bool			dtr;
	u8			cs;
	bool			use_phy;
	bool			use_dqs;
	struct phy_setting	phy_setting;
	struct spi_mem_op	phy_read_op;
	u32			phy_tx_start;
	u32			phy_tx_end;
};

struct cqspi_st {
	struct platform_device	*pdev;
	struct spi_controller	*host;
	struct clk		*clk;
	struct clk		*clks[CLK_QSPI_NUM];
	unsigned int		sclk;

	void __iomem		*iobase;
	void __iomem		*ahb_base;
	resource_size_t		ahb_size;
	struct completion	transfer_complete;

	struct dma_chan		*rx_chan;
	struct completion	rx_dma_complete;
	dma_addr_t		mmap_phys_base;

	int			current_cs;
	unsigned long		master_ref_clk_hz;
	bool			is_decoded_cs;
	u32			fifo_depth;
	u32			fifo_width;
	u32			num_chipselect;
	bool			rclk_en;
	u32			trigger_address;
	u32			phase_detect_selector;
	u32			wr_delay;
	bool			use_direct_mode;
	bool			use_direct_mode_wr;
	struct cqspi_flash_pdata f_pdata[CQSPI_MAX_CHIPSELECT];
	bool			use_dma_read;
	u32			pd_dev_id;
	bool			wr_completion;
	bool			slow_sram;
	bool			apb_ahb_hazard;

	bool			is_jh7110; /* Flag for StarFive JH7110 SoC */
};

struct cqspi_driver_platdata {
	u32 hwcaps_mask;
	u8 quirks;
	int (*indirect_read_dma)(struct cqspi_flash_pdata *f_pdata,
				 u_char *rxbuf, loff_t from_addr, size_t n_rx);
	u32 (*get_dma_status)(struct cqspi_st *cqspi);
	int (*jh7110_clk_init)(struct platform_device *pdev,
			       struct cqspi_st *cqspi);
};

/* Operation timeout value */
#define CQSPI_TIMEOUT_MS			500
#define CQSPI_READ_TIMEOUT_MS			10
#define CQSPI_DLL_TIMEOUT_US			300

/* Runtime_pm autosuspend delay */
#define CQSPI_AUTOSUSPEND_TIMEOUT		2000

#define CQSPI_DUMMY_CLKS_PER_BYTE		8
#define CQSPI_DUMMY_BYTES_MAX			4
#define CQSPI_DUMMY_CLKS_MAX			31

#define CQSPI_STIG_DATA_LEN_MAX			8

/* Register map */
#define CQSPI_REG_CONFIG			0x00
#define CQSPI_REG_CONFIG_ENABLE_MASK		BIT(0)
#define CQSPI_REG_CONFIG_PHY_EN			BIT(3)
#define CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL	BIT(7)
#define CQSPI_REG_CONFIG_DECODE_MASK		BIT(9)
#define CQSPI_REG_CONFIG_CHIPSELECT_LSB		10
#define CQSPI_REG_CONFIG_DMA_MASK		BIT(15)
#define CQSPI_REG_CONFIG_BAUD_LSB		19
#define CQSPI_REG_CONFIG_DTR_PROTO		BIT(24)
#define CQSPI_REG_CONFIG_PHY_PIPELINE		BIT(25)
#define CQSPI_REG_CONFIG_DUAL_OPCODE		BIT(30)
#define CQSPI_REG_CONFIG_IDLE_LSB		31
#define CQSPI_REG_CONFIG_CHIPSELECT_MASK	0xF
#define CQSPI_REG_CONFIG_BAUD_MASK		0xF

#define CQSPI_REG_RD_INSTR			0x04
#define CQSPI_REG_RD_INSTR_OPCODE_LSB		0
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB	8
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_RD_INSTR_TYPE_DATA_LSB	16
#define CQSPI_REG_RD_INSTR_MODE_EN_LSB		20
#define CQSPI_REG_RD_INSTR_DUMMY_LSB		24
#define CQSPI_REG_RD_INSTR_TYPE_INSTR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_ADDR_MASK	0x3
#define CQSPI_REG_RD_INSTR_TYPE_DATA_MASK	0x3
#define CQSPI_REG_RD_INSTR_DUMMY_MASK		0x1F

#define CQSPI_REG_WR_INSTR			0x08
#define CQSPI_REG_WR_INSTR_OPCODE_LSB		0
#define CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB	12
#define CQSPI_REG_WR_INSTR_TYPE_DATA_LSB	16

#define CQSPI_REG_DELAY				0x0C
#define CQSPI_REG_DELAY_TSLCH_LSB		0
#define CQSPI_REG_DELAY_TCHSH_LSB		8
#define CQSPI_REG_DELAY_TSD2D_LSB		16
#define CQSPI_REG_DELAY_TSHSL_LSB		24
#define CQSPI_REG_DELAY_TSLCH_MASK		0xFF
#define CQSPI_REG_DELAY_TCHSH_MASK		0xFF
#define CQSPI_REG_DELAY_TSD2D_MASK		0xFF
#define CQSPI_REG_DELAY_TSHSL_MASK		0xFF

#define CQSPI_REG_READCAPTURE			0x10
#define CQSPI_REG_READCAPTURE_BYPASS_LSB	0
#define CQSPI_REG_READCAPTURE_DELAY_LSB		1
#define CQSPI_REG_READCAPTURE_DELAY_MASK	0xF
#define CQSPI_REG_READCAPTURE_EDGE_LSB		5
#define CQSPI_REG_READCAPTURE_DQS_LSB		8

#define CQSPI_REG_SIZE				0x14
#define CQSPI_REG_SIZE_ADDRESS_LSB		0
#define CQSPI_REG_SIZE_PAGE_LSB			4
#define CQSPI_REG_SIZE_BLOCK_LSB		16
#define CQSPI_REG_SIZE_ADDRESS_MASK		0xF
#define CQSPI_REG_SIZE_PAGE_MASK		0xFFF
#define CQSPI_REG_SIZE_BLOCK_MASK		0x3F

#define CQSPI_REG_SRAMPARTITION			0x18
#define CQSPI_REG_INDIRECTTRIGGER		0x1C

#define CQSPI_REG_DMA				0x20
#define CQSPI_REG_DMA_SINGLE_LSB		0
#define CQSPI_REG_DMA_BURST_LSB			8
#define CQSPI_REG_DMA_SINGLE_MASK		0xFF
#define CQSPI_REG_DMA_BURST_MASK		0xFF

#define CQSPI_REG_REMAP				0x24
#define CQSPI_REG_MODE_BIT			0x28

#define CQSPI_REG_SDRAMLEVEL			0x2C
#define CQSPI_REG_SDRAMLEVEL_RD_LSB		0
#define CQSPI_REG_SDRAMLEVEL_WR_LSB		16
#define CQSPI_REG_SDRAMLEVEL_RD_MASK		0xFFFF
#define CQSPI_REG_SDRAMLEVEL_WR_MASK		0xFFFF

#define CQSPI_REG_WR_COMPLETION_CTRL		0x38
#define CQSPI_REG_WR_DISABLE_AUTO_POLL		BIT(14)

#define CQSPI_REG_IRQSTATUS			0x40
#define CQSPI_REG_IRQMASK			0x44

#define CQSPI_REG_INDIRECTRD			0x60
#define CQSPI_REG_INDIRECTRD_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTRD_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTRD_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTRDWATERMARK		0x64
#define CQSPI_REG_INDIRECTRDSTARTADDR		0x68
#define CQSPI_REG_INDIRECTRDBYTES		0x6C

#define CQSPI_REG_CMDCTRL			0x90
#define CQSPI_REG_CMDCTRL_EXECUTE_MASK		BIT(0)
#define CQSPI_REG_CMDCTRL_INPROGRESS_MASK	BIT(1)
#define CQSPI_REG_CMDCTRL_DUMMY_LSB		7
#define CQSPI_REG_CMDCTRL_WR_BYTES_LSB		12
#define CQSPI_REG_CMDCTRL_WR_EN_LSB		15
#define CQSPI_REG_CMDCTRL_ADD_BYTES_LSB		16
#define CQSPI_REG_CMDCTRL_ADDR_EN_LSB		19
#define CQSPI_REG_CMDCTRL_RD_BYTES_LSB		20
#define CQSPI_REG_CMDCTRL_RD_EN_LSB		23
#define CQSPI_REG_CMDCTRL_OPCODE_LSB		24
#define CQSPI_REG_CMDCTRL_WR_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_ADD_BYTES_MASK	0x3
#define CQSPI_REG_CMDCTRL_RD_BYTES_MASK		0x7
#define CQSPI_REG_CMDCTRL_DUMMY_MASK		0x1F

#define CQSPI_REG_INDIRECTWR			0x70
#define CQSPI_REG_INDIRECTWR_START_MASK		BIT(0)
#define CQSPI_REG_INDIRECTWR_CANCEL_MASK	BIT(1)
#define CQSPI_REG_INDIRECTWR_DONE_MASK		BIT(5)

#define CQSPI_REG_INDIRECTWRWATERMARK		0x74
#define CQSPI_REG_INDIRECTWRSTARTADDR		0x78
#define CQSPI_REG_INDIRECTWRBYTES		0x7C

#define CQSPI_REG_INDTRIG_ADDRRANGE		0x80

#define CQSPI_REG_CMDADDRESS			0x94
#define CQSPI_REG_CMDREADDATALOWER		0xA0
#define CQSPI_REG_CMDREADDATAUPPER		0xA4
#define CQSPI_REG_CMDWRITEDATALOWER		0xA8
#define CQSPI_REG_CMDWRITEDATAUPPER		0xAC

#define CQSPI_REG_POLLING_STATUS		0xB0
#define CQSPI_REG_POLLING_STATUS_DUMMY_LSB	16

#define CQSPI_REG_PHY_CONFIG			0xB4
#define CQSPI_REG_PHY_CONFIG_RX_DEL_LSB		0
#define CQSPI_REG_PHY_CONFIG_RX_DEL_MASK	0x7F
#define CQSPI_REG_PHY_CONFIG_TX_DEL_LSB		16
#define CQSPI_REG_PHY_CONFIG_TX_DEL_MASK	0x7F
#define CQSPI_REG_PHY_CONFIG_DLL_RESET		BIT(30)
#define CQSPI_REG_PHY_CONFIG_RESYNC		BIT(31)

#define CQSPI_REG_PHY_DLL_MASTER		0xB8
#define CQSPI_REG_PHY_DLL_MASTER_INIT_DELAY_LSB	0
#define CQSPI_REG_PHY_DLL_MASTER_INIT_DELAY_VAL	16
#define CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LEN	0x7
#define CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LSB	20
#define CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_3	0x2
#define CQSPI_REG_PHY_DLL_MASTER_BYPASS		BIT(23)
#define CQSPI_REG_PHY_DLL_MASTER_CYCLE		BIT(24)

#define CQSPI_REG_DLL_OBS_LOW			0xBC
#define CQSPI_REG_DLL_OBS_LOW_DLL_LOCK_LSB	0
#define CQSPI_REG_DLL_OBS_LOW_LOOPBACK_LOCK_LSB	15

#define CQSPI_REG_OP_EXT_LOWER			0xE0
#define CQSPI_REG_OP_EXT_READ_LSB		24
#define CQSPI_REG_OP_EXT_WRITE_LSB		16
#define CQSPI_REG_OP_EXT_STIG_LSB		0

#define CQSPI_REG_VERSAL_DMA_SRC_ADDR		0x1000

#define CQSPI_REG_VERSAL_DMA_DST_ADDR		0x1800
#define CQSPI_REG_VERSAL_DMA_DST_SIZE		0x1804

#define CQSPI_REG_VERSAL_DMA_DST_CTRL		0x180C

#define CQSPI_REG_VERSAL_DMA_DST_I_STS		0x1814
#define CQSPI_REG_VERSAL_DMA_DST_I_EN		0x1818
#define CQSPI_REG_VERSAL_DMA_DST_I_DIS		0x181C
#define CQSPI_REG_VERSAL_DMA_DST_DONE_MASK	BIT(1)

#define CQSPI_REG_VERSAL_DMA_DST_ADDR_MSB	0x1828

#define CQSPI_REG_VERSAL_DMA_DST_CTRL_VAL	0xF43FFA00
#define CQSPI_REG_VERSAL_ADDRRANGE_WIDTH_VAL	0x6

/* Interrupt status bits */
#define CQSPI_REG_IRQ_MODE_ERR			BIT(0)
#define CQSPI_REG_IRQ_UNDERFLOW			BIT(1)
#define CQSPI_REG_IRQ_IND_COMP			BIT(2)
#define CQSPI_REG_IRQ_IND_RD_REJECT		BIT(3)
#define CQSPI_REG_IRQ_WR_PROTECTED_ERR		BIT(4)
#define CQSPI_REG_IRQ_ILLEGAL_AHB_ERR		BIT(5)
#define CQSPI_REG_IRQ_WATERMARK			BIT(6)
#define CQSPI_REG_IRQ_IND_SRAM_FULL		BIT(12)

#define CQSPI_IRQ_MASK_RD		(CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_IND_SRAM_FULL	| \
					 CQSPI_REG_IRQ_IND_COMP)

#define CQSPI_IRQ_MASK_WR		(CQSPI_REG_IRQ_IND_COMP		| \
					 CQSPI_REG_IRQ_WATERMARK	| \
					 CQSPI_REG_IRQ_UNDERFLOW)

#define CQSPI_IRQ_STATUS_MASK		0x1FFFF
#define CQSPI_DMA_UNALIGN		0x3

#define CQSPI_REG_VERSAL_DMA_VAL		0x602

#define CQSPI_PHY_INIT_RD			1
#define CQSPI_PHY_MAX_RD			4
#define CQSPI_PHY_MAX_DELAY			127
#define CQSPI_PHY_DDR_SEARCH_STEP		4
#define CQSPI_PHY_MAX_RX			63
#define CQSPI_PHY_MAX_TX			63
#define CQSPI_PHY_TX_LOOKUP_LOW_START		28
#define CQSPI_PHY_TX_LOOKUP_LOW_END		48
#define CQSPI_PHY_TX_LOOKUP_HIGH_START		60
#define CQSPI_PHY_TX_LOOKUP_HIGH_END		96
#define CQSPI_PHY_RX_LOW_SEARCH_START		0
#define CQSPI_PHY_RX_LOW_SEARCH_END		40
#define CQSPI_PHY_RX_HIGH_SEARCH_START		24
#define CQSPI_PHY_RX_HIGH_SEARCH_END		127
#define CQSPI_PHY_TX_LOW_SEARCH_START		0
#define CQSPI_PHY_TX_LOW_SEARCH_END		64
#define CQSPI_PHY_TX_HIGH_SEARCH_START		78
#define CQSPI_PHY_TX_HIGH_SEARCH_END		127
#define CQSPI_PHY_SEARCH_OFFSET		8

#define CQSPI_PHY_DEFAULT_TEMP		45
#define CQSPI_PHY_MIN_TEMP		-45
#define CQSPI_PHY_MAX_TEMP		130
#define CQSPI_PHY_MID_TEMP		(CQSPI_PHY_MIN_TEMP +	\
					 ((CQSPI_PHY_MAX_TEMP - CQSPI_PHY_MIN_TEMP) / 2))

static const u8 phy_tuning_pattern[] = {
0xFE, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xFE, 0xFE, 0x01, 0x01,
0x01, 0x01, 0x00, 0x00, 0xFE, 0xFE, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0xFE, 0xFE, 0xFF, 0x01,
0x01, 0x01, 0x01, 0x01, 0xFE, 0x00, 0xFE, 0xFE, 0x01, 0x01, 0x01, 0x01, 0xFE,
0x00, 0xFE, 0xFE, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0xFE, 0xFE,
0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0xFE, 0xFE, 0xFF, 0x01, 0x01, 0x01, 0x01,
0x01, 0x00, 0xFE, 0xFE, 0xFE, 0x01, 0x01, 0x01, 0x01, 0x00, 0xFE, 0xFE, 0xFE,
0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF,
0xFF, 0x00, 0xFE, 0xFE, 0xFE, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFE, 0xFE,
0xFE, 0xFE, 0x01, 0x01, 0x01, 0x01, 0xFE, 0xFE, 0xFE, 0xFE, 0x01,
};

static bool cqspi_is_idle(struct cqspi_st *cqspi)
{
	u32 reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);

	return reg & (1UL << CQSPI_REG_CONFIG_IDLE_LSB);
}

static int cqspi_wait_idle(struct cqspi_st *cqspi)
{
	const unsigned int poll_idle_retry = 3;
	unsigned int count = 0;
	unsigned long timeout;

	timeout = jiffies + msecs_to_jiffies(CQSPI_TIMEOUT_MS);
	while (1) {
		/*
		 * Read few times in succession to ensure the controller
		 * is indeed idle, that is, the bit does not transition
		 * low again.
		 */

		if (cqspi_is_idle(cqspi))
			count++;
		else
			count = 0;

		if (count >= poll_idle_retry)
			return 0;

		if (time_after(jiffies, timeout)) {
			/* Timeout, in busy mode. */

			dev_err(&cqspi->pdev->dev,
				"QSPI is still busy after %dms timeout.\n",
				CQSPI_TIMEOUT_MS);
			return -ETIMEDOUT;
		}
		cpu_relax();
	}
}

static void cqspi_set_tx_dll(void __iomem *reg_base, u8 dll)
{
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_PHY_CONFIG);
	reg &= ~(CQSPI_REG_PHY_CONFIG_TX_DEL_MASK <<
		CQSPI_REG_PHY_CONFIG_TX_DEL_LSB);
	reg |= (dll & CQSPI_REG_PHY_CONFIG_TX_DEL_MASK) <<
		CQSPI_REG_PHY_CONFIG_TX_DEL_LSB;
	reg |= CQSPI_REG_PHY_CONFIG_RESYNC;
	writel(reg, reg_base + CQSPI_REG_PHY_CONFIG);
}

static void cqspi_set_rx_dll(void __iomem *reg_base, u8 dll)
{
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_PHY_CONFIG);
	reg &= ~(CQSPI_REG_PHY_CONFIG_RX_DEL_MASK <<
		CQSPI_REG_PHY_CONFIG_RX_DEL_LSB);
	reg |= (dll & CQSPI_REG_PHY_CONFIG_RX_DEL_MASK) <<
		CQSPI_REG_PHY_CONFIG_RX_DEL_LSB;
	reg |= CQSPI_REG_PHY_CONFIG_RESYNC;
	writel(reg, reg_base + CQSPI_REG_PHY_CONFIG);
}

static int cqspi_resync_dll(struct cqspi_st *cqspi)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	int ret;

	ret = cqspi_wait_idle(cqspi);

	if (!ret) {
		reg = readl(reg_base + CQSPI_REG_CONFIG);
		reg &= ~(CQSPI_REG_CONFIG_ENABLE_MASK);
		writel(reg, reg_base + CQSPI_REG_CONFIG);

		reg = readl(reg_base + CQSPI_REG_PHY_CONFIG);
		reg &= ~(CQSPI_REG_PHY_CONFIG_DLL_RESET |
			 CQSPI_REG_PHY_CONFIG_RESYNC);
		writel(reg, reg_base + CQSPI_REG_PHY_CONFIG);

		reg = readl(reg_base + CQSPI_REG_PHY_DLL_MASTER);
		reg |= (CQSPI_REG_PHY_DLL_MASTER_INIT_DELAY_VAL
			<< CQSPI_REG_PHY_DLL_MASTER_INIT_DELAY_LSB);
		writel(reg, reg_base + CQSPI_REG_PHY_DLL_MASTER);

		reg = readl(reg_base + CQSPI_REG_PHY_CONFIG);
		reg |= CQSPI_REG_PHY_CONFIG_DLL_RESET;
		writel(reg, reg_base + CQSPI_REG_PHY_CONFIG);

		readl_poll_timeout(reg_base + CQSPI_REG_DLL_OBS_LOW, reg,
				   !(reg &= (1 << CQSPI_REG_DLL_OBS_LOW_DLL_LOCK_LSB)),
				   0, CQSPI_DLL_TIMEOUT_US);

		readl_poll_timeout(reg_base + CQSPI_REG_DLL_OBS_LOW, reg,
				   !(reg &= (1 << CQSPI_REG_DLL_OBS_LOW_LOOPBACK_LOCK_LSB)),
				   0, CQSPI_DLL_TIMEOUT_US);

		reg = readl(reg_base + CQSPI_REG_PHY_CONFIG);
		reg |= CQSPI_REG_PHY_CONFIG_RESYNC;
		writel(reg, reg_base + CQSPI_REG_PHY_CONFIG);

		reg = readl(reg_base + CQSPI_REG_CONFIG);
		reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
		writel(reg, reg_base + CQSPI_REG_CONFIG);
	}

	return ret;
}

/* TODO: Figure out how to get the temperature here. */
static int cqspi_get_temp(int *temp)
{
	return -EOPNOTSUPP;
}

static int cqspi_phy_apply_setting(struct cqspi_flash_pdata *f_pdata,
				   struct phy_setting *phy)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	unsigned int reg;

	reg = readl(cqspi->iobase + CQSPI_REG_READCAPTURE);
	reg |= (1 << CQSPI_REG_READCAPTURE_EDGE_LSB);
	writel(reg, cqspi->iobase + CQSPI_REG_READCAPTURE);

	cqspi_set_rx_dll(cqspi->iobase, phy->rx);
	cqspi_set_tx_dll(cqspi->iobase, phy->tx);
	f_pdata->phy_setting.read_delay = phy->read_delay;

	return cqspi_resync_dll(cqspi);
}

static int cqspi_phy_check_pattern(struct cqspi_flash_pdata *f_pdata,
				   struct spi_mem *mem)
{
	struct spi_mem_op op = f_pdata->phy_read_op;
	u8 *read_data;
	unsigned int size = sizeof(phy_tuning_pattern);
	int ret;

	read_data = kmalloc(size, GFP_KERNEL);
	if (!read_data)
		return -ENOMEM;

	op.data.buf.in = read_data;
	op.data.nbytes = size;

	ret = spi_mem_exec_op(mem, &op);
	if (ret)
		goto out;

	if (memcmp(read_data, phy_tuning_pattern,
		   ARRAY_SIZE(phy_tuning_pattern))) {
		ret = -EAGAIN;
		goto out;
	}

	ret = 0;

out:
	kfree(read_data);
	return ret;
}

static int cqspi_find_rx_low(struct cqspi_flash_pdata *f_pdata,
			     struct spi_mem *mem, struct phy_setting *phy)
{
	struct device *dev = &f_pdata->cqspi->pdev->dev;
	int ret;

	do {
		phy->rx = CQSPI_PHY_RX_LOW_SEARCH_START;
		do {
			ret = cqspi_phy_apply_setting(f_pdata, phy);
			if (!ret) {
				ret = cqspi_phy_check_pattern(f_pdata, mem);
				if (!ret)
					return 0;
			}

			phy->rx += CQSPI_PHY_DDR_SEARCH_STEP;
		} while (phy->rx <= CQSPI_PHY_RX_LOW_SEARCH_END);

		phy->read_delay++;
	} while (phy->read_delay <= CQSPI_PHY_MAX_RD);

	dev_dbg(dev, "Unable to find RX low\n");
	return -ENOENT;
}

static int cqspi_find_rx_low_sdr(struct cqspi_flash_pdata *f_pdata,
				 struct spi_mem *mem, struct phy_setting *phy)
{
	struct device *dev = &f_pdata->cqspi->pdev->dev;
	int ret;

	phy->rx = 0;
	do {
		ret = cqspi_phy_apply_setting(f_pdata, phy);
		if (!ret) {
			ret = cqspi_phy_check_pattern(f_pdata, mem);
			if (!ret)
				return 0;
		}
		phy->rx++;
	} while (phy->rx < CQSPI_PHY_MAX_DELAY - 1);

	dev_dbg(dev, "Unable to find RX low\n");
	return -ENOENT;
}

static int cqspi_find_rx_high(struct cqspi_flash_pdata *f_pdata,
			      struct spi_mem *mem, struct phy_setting *phy)
{
	struct device *dev = &f_pdata->cqspi->pdev->dev;
	int ret;

	do {
		phy->rx = CQSPI_PHY_RX_HIGH_SEARCH_END;
		do {
			ret = cqspi_phy_apply_setting(f_pdata, phy);
			if (!ret) {
				ret = cqspi_phy_check_pattern(f_pdata, mem);
				if (!ret)
					return 0;
			}

			phy->rx -= CQSPI_PHY_DDR_SEARCH_STEP;
		} while (phy->rx >= CQSPI_PHY_RX_HIGH_SEARCH_START);

		phy->read_delay--;
	} while (phy->read_delay >= CQSPI_PHY_INIT_RD);

	dev_dbg(dev, "Unable to find RX high\n");
	return -ENOENT;
}

static int cqspi_find_rx_high_sdr(struct cqspi_flash_pdata *f_pdata,
				  struct spi_mem *mem, struct phy_setting *phy,
				  u8 lowerbound)
{
	struct device *dev = &f_pdata->cqspi->pdev->dev;
	int ret;

	phy->rx = CQSPI_PHY_MAX_DELAY;
	do {
		ret = cqspi_phy_apply_setting(f_pdata, phy);
		if (!ret) {
			ret = cqspi_phy_check_pattern(f_pdata, mem);
			if (!ret)
				return 0;
		}
		phy->rx--;
	} while (phy->rx > lowerbound);

	dev_dbg(dev, "Unable to find RX high\n");
	return -ENOENT;
}

static int cqspi_find_tx_low(struct cqspi_flash_pdata *f_pdata,
			     struct spi_mem *mem, struct phy_setting *phy)
{
	struct device *dev = &f_pdata->cqspi->pdev->dev;
	int ret;

	do {
		phy->tx = CQSPI_PHY_TX_LOW_SEARCH_START;
		do {
			ret = cqspi_phy_apply_setting(f_pdata, phy);
			if (!ret) {
				ret = cqspi_phy_check_pattern(f_pdata, mem);
				if (!ret)
					return 0;
			}

			phy->tx += CQSPI_PHY_DDR_SEARCH_STEP;
		} while (phy->tx <= CQSPI_PHY_TX_LOW_SEARCH_END);

		phy->read_delay++;
	} while (phy->read_delay <= CQSPI_PHY_MAX_RD);

	dev_dbg(dev, "Unable to find TX low\n");
	return -ENOENT;
}

static int cqspi_find_tx_high(struct cqspi_flash_pdata *f_pdata,
			      struct spi_mem *mem, struct phy_setting *phy)
{
	struct device *dev = &f_pdata->cqspi->pdev->dev;
	int ret;

	do {
		phy->tx = CQSPI_PHY_TX_HIGH_SEARCH_END;
		do {
			ret = cqspi_phy_apply_setting(f_pdata, phy);
			if (!ret) {
				ret = cqspi_phy_check_pattern(f_pdata, mem);
				if (!ret)
					return 0;
			}

			phy->tx -= CQSPI_PHY_DDR_SEARCH_STEP;
		} while (phy->tx >= CQSPI_PHY_TX_HIGH_SEARCH_START);

		phy->read_delay--;
	} while (phy->read_delay >= CQSPI_PHY_INIT_RD);

	dev_dbg(dev, "Unable to find TX high\n");
	return -ENOENT;
}

static int cqspi_phy_find_gaplow(struct cqspi_flash_pdata *f_pdata,
				 struct spi_mem *mem,
				 struct phy_setting *bottomleft,
				 struct phy_setting *topright,
				 struct phy_setting *gaplow)
{
	struct phy_setting left, right, mid;
	int ret;

	left = *bottomleft;
	right = *topright;

	mid.tx = left.tx + ((right.tx - left.tx) / 2);
	mid.rx = left.rx + ((right.rx - left.rx) / 2);
	mid.read_delay = left.read_delay;

	do {
		ret = cqspi_phy_apply_setting(f_pdata, &mid);
		if (!ret)
			ret = cqspi_phy_check_pattern(f_pdata, mem);

		if (ret) {
			/* The pattern was not found. Go to the lower half. */
			right.tx = mid.tx;
			right.rx = mid.rx;

			mid.tx = left.tx + ((mid.tx - left.tx) / 2);
			mid.rx = left.rx + ((mid.rx - left.rx) / 2);
		} else {
			/* The pattern was found. Go to the upper half. */
			left.tx = mid.tx;
			left.rx = mid.rx;

			mid.tx = mid.tx + ((right.tx - mid.tx) / 2);
			mid.rx = mid.rx + ((right.rx - mid.rx) / 2);
		}

	/* Break the loop if the window has closed. */
	} while ((right.tx - left.tx >= 2) && (right.rx - left.rx >= 2));

	*gaplow = mid;
	return 0;
}

static int cqspi_phy_find_gaphigh(struct cqspi_flash_pdata *f_pdata,
				  struct spi_mem *mem,
				  struct phy_setting *bottomleft,
				  struct phy_setting *topright,
				  struct phy_setting *gaphigh)
{
	struct phy_setting left, right, mid;
	int ret;

	left = *bottomleft;
	right = *topright;

	mid.tx = left.tx + ((right.tx - left.tx) / 2);
	mid.rx = left.rx + ((right.rx - left.rx) / 2);
	mid.read_delay = right.read_delay;

	do {
		ret = cqspi_phy_apply_setting(f_pdata, &mid);
		if (!ret)
			ret = cqspi_phy_check_pattern(f_pdata, mem);

		if (ret) {
			/* The pattern was not found. Go to the upper half. */
			left.tx = mid.tx;
			left.rx = mid.rx;

			mid.tx = mid.tx + ((right.tx - mid.tx) / 2);
			mid.rx = mid.rx + ((right.rx - mid.rx) / 2);
		} else {
			/* The pattern was found. Go to the lower half. */
			right.tx = mid.tx;
			right.rx = mid.rx;

			mid.tx = left.tx + ((mid.tx - left.tx) / 2);
			mid.rx = left.rx + ((mid.rx - left.rx) / 2);
		}

	/* Break the loop if the window has closed. */
	} while ((right.tx - left.tx >= 2) && (right.rx - left.rx >= 2));

	*gaphigh = mid;
	return 0;
}

static int cqspi_phy_calibrate(struct cqspi_flash_pdata *f_pdata,
			       struct spi_mem *mem)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	struct phy_setting rxlow, rxhigh, txlow, txhigh;
	struct phy_setting srxlow, srxhigh;
	struct phy_setting bottomleft, topright, searchpoint;
	struct phy_setting gaplow, gaphigh;
	struct phy_setting backuppoint, backupcornerpoint;
	int ret, tmp;
	bool primary = 1, secondary = 1;

	f_pdata->use_phy = true;

	/*
	 * Finding rx fails at some of the tx values based on the H/W platform.
	 * A window of tx values is used to find the rx without errors. This
	 * can increase the number of CPU cycles taken for the PHY tuning in
	 * the cases where more tx values need to be parsed to find a stable
	 * rx.
	 */

	/* ***********************Golden rxlow search*********************** */

	/*
	 *
	 *		rx
	 *	    127	^
	 *		|
	 *		|	xxxxx     ++++++++++++++++++++
	 *		|	xxxxxx     +++++++++++++++++++
	 *		|	xxxxxxx     ++++++++++++++++++
	 *		|	xxxxxxxx     +++++++++++++++++
	 *		|	xxxxxxxxx     ++++++++++++++++
	 *		|	xxxxxxxxxx     +++++++++++++++
	 *		|	xxxxxxxxxxx     ++++++++++++++
	 *		|	|xxxxx|xxxxx     +++++++++++++
	 *		|	|xxxxx|xxxxxx     ++++++++++++
	 *	search	|	|xxxxx|xxxxxxx     +++++++++++
	 *	rxlow --------->|xxxxx|xxxxxxxx     ++++++++++
	 *		|	|xxxxx|xxxxxxxxx     +++++++++
	 *		|	|xxxxx|xxxxxxxxxx     ++++++++
	 *		|	|xxxxx|xxxxxxxxxxx     +++++++
	 *		|	|     |
	 *		--------|-----|----------------------------> tx
	 *		0	|     |				 127
	 *		    txlow     txlow
	 *		    start     end
	 *
	 */

	/*
	 *	|----------------------------------------------------------|
	 *	| Primary | Secondary | Final                              |
	 *	| Search  | Search    | Point                              |
	 *	|---------|-----------|------------------------------------|
	 *	| Fail    | Fail      | Return Fail                        |
	 *	|---------|-----------|------------------------------------|
	 *	| Fail    | Pass      | Return Fail                        |
	 *	|---------|-----------|------------------------------------|
	 *	| Pass    | Fail      | Return Fail                        |
	 *	|---------|-----------|------------------------------------|
	 *	| Pass    | Pass      | rx = min(primary.rx, secondary.rx) |
	 *	|         |           | tx = primary.tx                    |
	 *	|         |           | read_delay =                       |
	 *	|	  |	      |		min(primary.read_delay,    |
	 *	|	  |	      |		    secondary.read_delay)  |
	 *	|----------------------------------------------------------|
	 */

	/* *******************Golden Primary rxlow search******************* */
	/*
	 * To find the rx boundaries, we fix a valid tx and search through rx
	 * range, read_delay values. As we are not sure of a valid tx we use a
	 * window of tx values to find the rx boundaries.
	 */

	rxlow.tx = CQSPI_PHY_TX_LOOKUP_LOW_START;
	do {
		dev_dbg(dev, "Searching for Golden Primary rxlow on TX = %d\n",
			rxlow.tx);
		rxlow.read_delay = CQSPI_PHY_INIT_RD;
		ret = cqspi_find_rx_low(f_pdata, mem, &rxlow);
		rxlow.tx += CQSPI_PHY_DDR_SEARCH_STEP;
	} while (ret && rxlow.tx <= CQSPI_PHY_TX_LOOKUP_LOW_END);
	if (ret)
		goto out;
	dev_dbg(dev, "Golden Primary rxlow: RX: %d TX: %d RD: %d\n", rxlow.rx,
		rxlow.tx, rxlow.read_delay);

	/* ******************Golden Secondary rxlow search****************** */
	/* Search for one more rxlow at different tx */

	if (rxlow.tx <= (CQSPI_PHY_TX_LOOKUP_LOW_END -
			 CQSPI_PHY_SEARCH_OFFSET))
		srxlow.tx = rxlow.tx + CQSPI_PHY_SEARCH_OFFSET;
	else
		srxlow.tx = CQSPI_PHY_TX_LOOKUP_LOW_END;
	dev_dbg(dev, "Searching for Golden Secondary rxlow on TX = %d\n",
		srxlow.tx);
	srxlow.read_delay = CQSPI_PHY_INIT_RD;
	ret = cqspi_find_rx_low(f_pdata, mem, &srxlow);
	if (ret)
		goto out;
	dev_dbg(dev, "Golden Secondary rxlow: RX: %d TX: %d RD: %d\n",
		srxlow.rx, srxlow.tx, srxlow.read_delay);

	rxlow.rx = min(rxlow.rx, srxlow.rx);
	rxlow.read_delay = min(rxlow.read_delay, srxlow.read_delay);
	dev_dbg(dev, "Golden Final rxlow: RX: %d TX: %d RD: %d\n", rxlow.rx,
		rxlow.tx, rxlow.read_delay);

	/* **********************Golden rxhigh search********************** */

	/*
	 *
	 *		rx
	 *	    127	^
	 *		|
	 *		|	|xxxx     ++++++++++++++++++++
	 *		|	|xxxxx     +++++++++++++++++++
	 *    search	|	|xxxxxx     ++++++++++++++++++
	 *    rxhigh  --------->|xxxxxxx     +++++++++++++++++
	 *    on fixed  |	|xxxxxxxx     ++++++++++++++++
	 *    tx	|	|xxxxxxxxx     +++++++++++++++
	 *		|	|xxxxxxxxxx     ++++++++++++++
	 *		|	xxxxxxxxxxxx     +++++++++++++
	 *		|	xxxxxxxxxxxxx     ++++++++++++
	 *		|	xxxxxxxxxxxxxx     +++++++++++
	 *		|	xxxxxxxxxxxxxxx     ++++++++++
	 *		|	xxxxxxxxxxxxxxxx     +++++++++
	 *		|	xxxxxxxxxxxxxxxxx     ++++++++
	 *		|	xxxxxxxxxxxxxxxxxx     +++++++
	 *		|
	 *		-------------------------------------------> tx
	 *		0					 127
	 *
	 */

	/*
	 *	|----------------------------------------------------------|
	 *	| Primary | Secondary | Final                              |
	 *	| Search  | Search    | Point                              |
	 *	|---------|-----------|------------------------------------|
	 *	| Fail    | Fail      | Return Fail                        |
	 *	|---------|-----------|------------------------------------|
	 *	| Fail    | Pass      | Choose Secondary                   |
	 *	|---------|-----------|------------------------------------|
	 *	| Pass    | Fail      | Choose Primary                     |
	 *	|---------|-----------|------------------------------------|
	 *	| Pass    | Pass      | if (secondary.rx > primary.rx)     |
	 *	|         |           |		Choose Secondary           |
	 *	|         |           | else                               |
	 *	|	  |	      |		Choose Primary             |
	 *	|----------------------------------------------------------|
	 */

	/* ******************Golden Primary rxhigh search****************** */
	/*
	 * To find rxhigh we use the tx values of rxlow. Start the read_delay
	 * from maximum and decrement it. As these are valid values and rxhigh
	 * read_delay is always greater than or equal to rxlow read_delay.
	 */

	rxhigh.tx = rxlow.tx;
	dev_dbg(dev, "Searching for Golden Primary rxhigh on TX = %d\n",
		rxhigh.tx);
	rxhigh.read_delay = CQSPI_PHY_MAX_RD;
	ret = cqspi_find_rx_high(f_pdata, mem, &rxhigh);
	if (ret)
		primary = 0;
	dev_dbg(dev, "Golden Primary rxhigh: RX: %d TX: %d RD: %d\n",
		rxhigh.rx, rxhigh.tx, rxhigh.read_delay);

	/* *****************Golden Secondary rxhigh search***************** */
	/* Search for one more rxhigh at different tx */

	if (rxhigh.tx <=
	    (CQSPI_PHY_TX_LOOKUP_LOW_END - CQSPI_PHY_SEARCH_OFFSET))
		srxhigh.tx = rxhigh.tx + CQSPI_PHY_SEARCH_OFFSET;
	else
		srxhigh.tx = CQSPI_PHY_TX_LOOKUP_LOW_END;
	dev_dbg(dev, "Searching for Golden Secondary rxhigh on TX = %d\n",
		srxhigh.tx);
	srxhigh.read_delay = CQSPI_PHY_MAX_RD;
	ret = cqspi_find_rx_high(f_pdata, mem, &srxhigh);
	if (ret)
		secondary = 0;
	dev_dbg(dev, "Golden Secondary rxhigh: RX: %d TX: %d RD: %d\n",
		srxhigh.rx, srxhigh.tx, srxhigh.read_delay);

	if (primary || secondary) {
		if (srxhigh.rx > rxhigh.rx)
			rxhigh = srxhigh;
	} else {
		goto out;
	}
	dev_dbg(dev, "Golden Final rxhigh: RX: %d TX: %d RD: %d\n", rxhigh.rx,
		rxhigh.tx, rxhigh.read_delay);

	primary = 1;
	secondary = 1;

	/*
	 * Check a different point if rxlow and rxhigh are on the same read
	 * delay. This avoids mistaking the failing region for an RX boundary.
	 */

	if (rxlow.read_delay == rxhigh.read_delay) {
		dev_dbg(dev, "rxlow and rxhigh at the same read delay.\n");

		/* *******************Backup rxlow search******************* */

		/* Look for RX boundaries at upper TX range. */

		/*
		 *
		 *		rx
		 *	    127	^
		 *		|
		 *		|	xxxxx     ++++++++++++++++++++
		 *		|	xxxxxx     +++++++++++++++++++
		 *		|	xxxxxxx     ++++++++++++++++++
		 *		|	xxxxxxxx     +++++++++++++++++
		 *		|	xxxxxxxxx     ++++++++++++++++
		 *		|	xxxxxxxxxx     +++++++++++++++
		 *		|	xxxxxxxxxxx     ++++++++++++++
		 *		|	xxxxxxxxxxxx     +++++++|++++|
		 *		|	xxxxxxxxxxxxx     ++++++|++++|
		 *	search	|	xxxxxxxxxxxxxx     +++++|++++|
		 *	rxlow --------------------------------->|++++|
		 *		|	xxxxxxxxxxxxxxxx     +++|++++|
		 *		|	xxxxxxxxxxxxxxxxx     ++|++++|
		 *		|	xxxxxxxxxxxxxxxxxx     +|++++|
		 *		|				|    |
		 *		--------------------------------|----|-----> tx
		 *		0				|    |	 127
		 *`					   txhigh    txhigh
		 *					    start    end
		 *
		 */

		/*
		 *	|-----------------------------------------------------|
		 *	| Primary | Secondary | Final                         |
		 *	| Search  | Search    | Point                         |
		 *	|---------|-----------|-------------------------------|
		 *	| Fail    | Fail      | Return Fail                   |
		 *	|---------|-----------|-------------------------------|
		 *	| Fail    | Pass      | Return Fail                   |
		 *	|---------|-----------|-------------------------------|
		 *	| Pass    | Fail      | Return Fail                   |
		 *	|---------|-----------|-------------------------------|
		 *	| Pass    | Pass      | rx =			      |
		 *	|	  |	      |	 min(primary.rx, secondary.rx)|
		 *	|         |           | tx = primary.tx               |
		 *	|         |           | read_delay =                  |
		 *	|	  |	      |	 min(primary.read_delay,      |
		 *	|	  |	      |	     secondary.read_delay)    |
		 *	|-----------------------------------------------------|
		 */

		/* ***************Backup Primary rxlow search*************** */
		/*
		 * Find the rx boundaries using the tx window at the higher
		 * end. We start at the window end and decrement the tx value
		 * until we find the valid point.
		 */

		backuppoint.tx = CQSPI_PHY_TX_LOOKUP_HIGH_END;
		do {
			dev_dbg(dev, "Searching for Backup Primary rxlow on TX = %d\n",
				backuppoint.tx);
			backuppoint.read_delay = CQSPI_PHY_INIT_RD;
			ret = cqspi_find_rx_low(f_pdata, mem, &backuppoint);
			backuppoint.tx -= CQSPI_PHY_DDR_SEARCH_STEP;
		} while (ret &&
			 backuppoint.tx >= CQSPI_PHY_TX_LOOKUP_HIGH_START);
		if (ret)
			goto out;
		dev_dbg(dev, "Backup Primary rxlow: RX: %d TX: %d RD: %d\n",
			backuppoint.rx, backuppoint.tx,
			backuppoint.read_delay);

		/* **************Backup Secondary rxlow search************** */
		/* Search for one more rxlow at different tx */

		if (backuppoint.tx >=
		    (CQSPI_PHY_TX_LOOKUP_HIGH_START + CQSPI_PHY_SEARCH_OFFSET))
			srxlow.tx = backuppoint.tx - CQSPI_PHY_SEARCH_OFFSET;
		else
			srxlow.tx = CQSPI_PHY_TX_LOOKUP_HIGH_START;
		dev_dbg(dev,
			"Searching for Backup Secondary rxlow on TX = %d\n",
			srxlow.tx);
		srxlow.read_delay = CQSPI_PHY_INIT_RD;
		ret = cqspi_find_rx_low(f_pdata, mem, &srxlow);
		if (ret)
			goto out;
		dev_dbg(dev, "Backup Secondary rxlow: RX: %d TX: %d RD: %d\n",
			srxlow.rx, srxlow.tx, srxlow.read_delay);

		backuppoint.rx = min(backuppoint.rx, srxlow.rx);
		backuppoint.read_delay =
			min(backuppoint.read_delay, srxlow.read_delay);
		dev_dbg(dev, "Backup Final rxlow: RX: %d TX: %d RD: %d\n",
			backuppoint.rx, backuppoint.tx,
			backuppoint.read_delay);

		if (backuppoint.rx < rxlow.rx) {
			rxlow = backuppoint;
			dev_dbg(dev, "Updating rxlow to the one at TX = %d\n",
				backuppoint.tx);
		}
		dev_dbg(dev, "Final rxlow: RX: %d TX: %d RD: %d\n", rxlow.rx,
			rxlow.tx, rxlow.read_delay);

		/* ******************Backup rxhigh search****************** */

		/*
		 *
		 *		rx
		 *	    127	^
		 *		|
		 *		|	xxxxx     +++++++++++++++++++|
		 *		|	xxxxxx     ++++++++++++++++++|
		 *    search	|	xxxxxxx     +++++++++++++++++|
		 *    rxhigh  -------------------------------------->|
		 *    on fixed	|	xxxxxxxxx     +++++++++++++++|
		 *    tx	|	xxxxxxxxxx     ++++++++++++++|
		 *		|	xxxxxxxxxxx     +++++++++++++|
		 *		|	xxxxxxxxxxxx     +++++++++++++
		 *		|	xxxxxxxxxxxxx     ++++++++++++
		 *		|	xxxxxxxxxxxxxx     +++++++++++
		 *		|	xxxxxxxxxxxxxxx     ++++++++++
		 *		|	xxxxxxxxxxxxxxxx     +++++++++
		 *		|	xxxxxxxxxxxxxxxxx     ++++++++
		 *		|	xxxxxxxxxxxxxxxxxx     +++++++
		 *		|
		 *		-------------------------------------------> tx
		 *		0					 127
		 *
		 */

		/*
		 *	|-----------------------------------------------------|
		 *	| Primary | Secondary | Final                         |
		 *	| Search  | Search    | Point                         |
		 *	|---------|-----------|-------------------------------|
		 *	| Fail    | Fail      | Return Fail                   |
		 *	|---------|-----------|-------------------------------|
		 *	| Fail    | Pass      | Choose Secondary              |
		 *	|---------|-----------|-------------------------------|
		 *	| Pass    | Fail      | Choose Primary                |
		 *	|---------|-----------|-------------------------------|
		 *	| Pass    | Pass      | if (secondary.rx > primary.rx)|
		 *	|         |           |		Choose Secondary      |
		 *	|         |           | else                          |
		 *	|	  |	      |		Choose Primary        |
		 *	|-----------------------------------------------------|
		 */

		/* **************Backup Primary rxhigh search************** */
		/*
		 * To find rxhigh we use the tx values of backuppoint. Start
		 * the read_delay from maximum and decrement it. As these are
		 * valid values and rxhigh read_delay is always greater than or
		 * equal to rxlow read_delay.
		 */

		dev_dbg(dev, "Searching for Backup Primary rxhigh on TX = %d\n",
			backuppoint.tx);
		backuppoint.read_delay = CQSPI_PHY_MAX_RD;
		ret = cqspi_find_rx_high(f_pdata, mem, &backuppoint);
		if (ret)
			primary = 0;
		dev_dbg(dev, "Backup Primary rxhigh: RX: %d TX: %d RD: %d\n",
			backuppoint.rx, backuppoint.tx,
			backuppoint.read_delay);

		/* *************Backup Secondary rxhigh search************* */
		/* Search for one more rxhigh at different tx */

		if (backuppoint.tx >=
		    (CQSPI_PHY_TX_LOOKUP_HIGH_START + CQSPI_PHY_SEARCH_OFFSET))
			srxhigh.tx = backuppoint.tx - CQSPI_PHY_SEARCH_OFFSET;
		else
			srxhigh.tx = CQSPI_PHY_TX_LOOKUP_HIGH_START;
		dev_dbg(dev,
			"Searching for Backup Secondary rxhigh on TX = %d\n",
			srxhigh.tx);
		srxhigh.read_delay = CQSPI_PHY_MAX_RD;
		ret = cqspi_find_rx_high(f_pdata, mem, &srxhigh);
		if (ret)
			secondary = 0;
		dev_dbg(dev, "Backup Secondary rxhigh: RX: %d TX: %d RD: %d\n",
			srxhigh.rx, srxhigh.tx, srxhigh.read_delay);

		if (primary || secondary) {
			if (srxhigh.rx > backuppoint.rx)
				backuppoint = srxhigh;
		} else {
			goto out;
		}
		dev_dbg(dev, "Backup Final rxhigh: RX: %d TX: %d RD: %d\n",
			backuppoint.rx, backuppoint.tx,
			backuppoint.read_delay);

		if (backuppoint.rx > rxhigh.rx) {
			rxhigh = backuppoint;
			dev_dbg(dev, "Updating rxhigh to the one at TX = %d\n",
				backuppoint.tx);
		}
		dev_dbg(dev, "Final rxhigh: RX: %d TX: %d RD: %d\n", rxhigh.rx,
			rxhigh.tx, rxhigh.read_delay);
	}

	/* ***********************Golden txlow search*********************** */
	/* Look for TX boundaries at 1/4 of RX window. */

	/*
	 *
	 *		rx
	 *	    127	^
	 *		|
	 *     rxhigh --------->xxxxx     ++++++++++++++++++++
	 *		|	xxxxxx     +++++++++++++++++++
	 *		|	xxxxxxx     ++++++++++++++++++
	 *		|	xxxxxxxx     +++++++++++++++++
	 *		|	xxxxxxxxx     ++++++++++++++++
	 *		|	xxxxxxxxxx     +++++++++++++++
	 *		|	xxxxxxxxxxx     ++++++++++++++
	 *		|	xxxxxxxxxxxx     +++++++++++++
	 *    fix rx	|	xxxxxxxxxxxxx     ++++++++++++
	 *    1/4 b/w ---------><------->xxxxx     +++++++++++
	 *    rxlow and	|	xxxx|xxxxxxxxxx     ++++++++++
	 *    rxhigh	|	xxxx|xxxxxxxxxxx     +++++++++
	 *		|	xxxx|xxxxxxxxxxxx     ++++++++
	 *	rxlow --------->xxxx|xxxxxxxxxxxxx     +++++++
	 *		|	    |
	 *		------------|------------------------------> tx
	 *		0	    |				 127
	 *		       search
	 *			txlow
	 *
	 */

	txlow.rx = rxlow.rx + ((rxhigh.rx - rxlow.rx) / 4);
	dev_dbg(dev, "Searching for Golden txlow on RX = %d\n", txlow.rx);
	txlow.read_delay = CQSPI_PHY_INIT_RD;
	ret = cqspi_find_tx_low(f_pdata, mem, &txlow);
	if (ret)
		goto out;
	dev_dbg(dev, "Golden txlow: RX: %d TX: %d RD: %d\n", txlow.rx,
		txlow.tx, txlow.read_delay);

	/* **********************Golden txhigh search********************** */
	/* Start from maximum read_delay and decrememt it */

	/*
	 *
	 *		rx
	 *	    127	^
	 *		|
	 *     rxhigh --------->xxxxx     ++++++++++++++++++++
	 *		|	xxxxxx     +++++++++++++++++++
	 *		|	xxxxxxx     ++++++++++++++++++
	 *		|	xxxxxxxx     +++++++++++++++++
	 *		|	xxxxxxxxx     ++++++++++++++++
	 *		|	xxxxxxxxxx     +++++++++++++++
	 *		|	xxxxxxxxxxx     ++++++++++++++
	 *		|	xxxxxxxxxxxx     +++++++++++++
	 *    fix rx	|	xxxxxxxxxxxxx     ++++++++++++
	 *    1/4 b/w --------------------------------><----->
	 *    rxlow and	|	xxxxxxxxxxxxxxx     ++++++|+++
	 *    rxhigh	|	xxxxxxxxxxxxxxxx     +++++|+++
	 *		|	xxxxxxxxxxxxxxxxx     ++++|+++
	 *	rxlow --------->xxxxxxxxxxxxxxxxxx     +++|+++
	 *		|				  |
	 *		----------------------------------|--------> tx
	 *		0				  |	 127
	 *					     search
	 *					     txhigh
	 *
	 */

	txhigh.rx = txlow.rx;
	dev_dbg(dev, "Searching for Golden txhigh on RX = %d\n", txhigh.rx);
	txhigh.read_delay = CQSPI_PHY_MAX_RD;
	ret = cqspi_find_tx_high(f_pdata, mem, &txhigh);
	if (ret)
		goto out;
	dev_dbg(dev, "Golden txhigh: RX: %d TX: %d RD: %d\n", txhigh.rx,
		txhigh.tx, txhigh.read_delay);

	/*
	 * Check a different point if txlow and txhigh are on the same read
	 * delay. This avoids mistaking the failing region for an TX boundary.
	 */

	if (txlow.read_delay == txhigh.read_delay) {
		/* *******************Backup txlow search******************* */
		/* Look for TX boundaries at 3/4 of RX window. */

		/*
		 *
		 *		rx
		 *	    127	^
		 *		|
		 *     rxhigh --------->xxxxx     ++++++++++++++++++++
		 *		|	xxxxxx     +++++++++++++++++++
		 *    fix rx	|	xxxxxxx     ++++++++++++++++++
		 *    3/4 b/w ---------><----->x     +++++++++++++++++
		 *    rxlow and	|	xxxx|xxxx     ++++++++++++++++
		 *    rxhigh	|	xxxx|xxxxx     +++++++++++++++
		 *		|	xxxx|xxxxxx     ++++++++++++++
		 *		|	xxxx|xxxxxxx     +++++++++++++
		 *		|	xxxx|xxxxxxxx     ++++++++++++
		 *		|	xxxx|xxxxxxxxx     +++++++++++
		 *		|	xxxx|xxxxxxxxxx     ++++++++++
		 *		|	xxxx|xxxxxxxxxxx     +++++++++
		 *		|	xxxx|xxxxxxxxxxxx     ++++++++
		 *	rxlow --------->xxxx|xxxxxxxxxxxxx     +++++++
		 *		|	    |
		 *		------------|------------------------------> tx
		 *		0	    |				 127
		 *		       search
		 *			txlow
		 *
		 */

		dev_dbg(dev, "txlow and txhigh at the same read delay.\n");
		backuppoint.rx = rxlow.rx + (3 * (rxhigh.rx - rxlow.rx) / 4);
		dev_dbg(dev, "Searching for Backup txlow on RX = %d\n",
			backuppoint.rx);
		backuppoint.read_delay = CQSPI_PHY_INIT_RD;
		ret = cqspi_find_tx_low(f_pdata, mem, &backuppoint);
		if (ret)
			goto out;
		dev_dbg(dev, "Backup txlow: RX: %d TX: %d RD: %d\n",
			backuppoint.rx, backuppoint.tx,
			backuppoint.read_delay);

		if (backuppoint.tx < txlow.tx) {
			txlow = backuppoint;
			dev_dbg(dev, "Updating txlow with the one at RX = %d\n",
				backuppoint.rx);
		}
		dev_dbg(dev, "Final txlow: RX: %d TX: %d RD: %d\n", txlow.rx,
			txlow.tx, txlow.read_delay);

		/* ******************Backup txhigh search****************** */
		/* Start from maximum read_delay and decrememt it */

		/*
		 *
		 *		rx
		 *	    127	^
		 *		|
		 *     rxhigh --------->xxxxx     ++++++++++++++++++++
		 *		|	xxxxxx     +++++++++++++++++++
		 *    fix rx	|	xxxxxxx     ++++++++++++++++++
		 *    3/4 b/w ------------------------------><------->
		 *    rxlow and	|	xxxxxxxxx     +++++++++++|++++
		 *    rxhigh	|	xxxxxxxxxx     ++++++++++|++++
		 *		|	xxxxxxxxxxx     +++++++++|++++
		 *		|	xxxxxxxxxxxx     ++++++++|++++
		 *		|	xxxxxxxxxxxxx     +++++++|++++
		 *		|	xxxxxxxxxxxxxx     ++++++|++++
		 *		|	xxxxxxxxxxxxxxx     +++++|++++
		 *		|	xxxxxxxxxxxxxxxx     ++++|++++
		 *		|	xxxxxxxxxxxxxxxxx     +++|++++
		 *	rxlow --------->xxxxxxxxxxxxxxxxxx     ++|++++
		 *		|				 |
		 *		---------------------------------|---------> tx
		 *		0				 |	 127
		 *						 search
		 *						 txhigh
		 *
		 */

		dev_dbg(dev, "Searching for Backup txhigh on RX = %d\n",
			backuppoint.rx);
		backuppoint.read_delay = CQSPI_PHY_MAX_RD;
		ret = cqspi_find_tx_high(f_pdata, mem, &backuppoint);
		if (ret)
			goto out;
		dev_dbg(dev, "Backup txhigh: RX: %d TX: %d RD: %d\n",
			backuppoint.rx, backuppoint.tx,
			backuppoint.read_delay);

		if (backuppoint.tx > txhigh.tx) {
			txhigh = backuppoint;
			dev_dbg(dev,
				"Updating txhigh with the one at RX = %d\n",
				backuppoint.rx);
		}
		dev_dbg(dev, "Final txhigh: RX: %d TX: %d RD: %d\n", txhigh.rx,
			txhigh.tx, txhigh.read_delay);
	}

	/*
	 * Set bottom left and top right corners. These are theoretical
	 * corners. They may not actually be "good" points. But the longest
	 * diagonal will be between these corners.
	 */

	bottomleft.tx = txlow.tx;
	bottomleft.rx = rxlow.rx;
	if (txlow.read_delay <= rxlow.read_delay)
		bottomleft.read_delay = txlow.read_delay;
	else
		bottomleft.read_delay = rxlow.read_delay;

	backupcornerpoint = bottomleft;
	backupcornerpoint.tx += 4;
	backupcornerpoint.rx += 4;
	ret = cqspi_phy_apply_setting(f_pdata, &backupcornerpoint);
	if (!ret)
		ret = cqspi_phy_check_pattern(f_pdata, mem);

	if (ret) {
		backupcornerpoint.read_delay--;
		ret = cqspi_phy_apply_setting(f_pdata, &backupcornerpoint);
		if (!ret)
			ret = cqspi_phy_check_pattern(f_pdata, mem);
	}

	/* TODO: if (ret) */

	if (!ret)
		bottomleft.read_delay = backupcornerpoint.read_delay;

	topright.tx = txhigh.tx;
	topright.rx = rxhigh.rx;
	if (txhigh.read_delay >= rxhigh.read_delay)
		topright.read_delay = txhigh.read_delay;
	else
		topright.read_delay = rxhigh.read_delay;

	backupcornerpoint = topright;
	backupcornerpoint.tx -= 4;
	backupcornerpoint.rx -= 4;
	ret = cqspi_phy_apply_setting(f_pdata, &backupcornerpoint);
	if (!ret)
		ret = cqspi_phy_check_pattern(f_pdata, mem);

	if (ret) {
		backupcornerpoint.read_delay++;
		ret = cqspi_phy_apply_setting(f_pdata, &backupcornerpoint);
		if (!ret)
			ret = cqspi_phy_check_pattern(f_pdata, mem);
	}

	/* TODO: if (ret) */

	if (!ret)
		topright.read_delay = backupcornerpoint.read_delay;

	dev_dbg(dev, "topright: RX: %d TX: %d RD: %d\n", topright.rx,
		topright.tx, topright.read_delay);
	dev_dbg(dev, "bottomleft: RX: %d TX: %d RD: %d\n", bottomleft.rx,
		bottomleft.tx, bottomleft.read_delay);

	ret = cqspi_phy_find_gaplow(f_pdata, mem, &bottomleft, &topright,
				    &gaplow);
	if (ret)
		goto out;
	dev_dbg(dev, "gaplow: RX: %d TX: %d RD: %d\n", gaplow.rx, gaplow.tx,
		gaplow.read_delay);

	if (bottomleft.read_delay == topright.read_delay) {
		/*
		 * If there is only one passing region, it means that the
		 * "true" topright is too small to find, so the start of the
		 * failing region is a good approximation. Put the tuning point
		 * in the middle and adjust for temperature.
		 */

		dev_dbg(dev,
			"bottomleft and topright at the same read delay.\n");

		topright = gaplow;
		searchpoint.read_delay = bottomleft.read_delay;
		searchpoint.tx =
			bottomleft.tx + ((topright.tx - bottomleft.tx) / 2);
		searchpoint.rx =
			bottomleft.rx + ((topright.rx - bottomleft.rx) / 2);

		ret = cqspi_get_temp(&tmp);
		if (ret) {
			/*
			 * Assume room temperature if it couldn't be obtained
			 * from the thermal sensor.
			 *
			 * TODO: Change it to dev_warn once support for finding
			 * out the temperature is added.
			 */

			dev_dbg(dev, "Unable to get temperature. Assuming room temperature\n");
			tmp = CQSPI_PHY_DEFAULT_TEMP;
		}

		if (tmp < CQSPI_PHY_MIN_TEMP || tmp > CQSPI_PHY_MAX_TEMP) {
			dev_err(dev,
				"Temperature outside operating range: %dC\n",
				tmp);
			ret = -EINVAL;
			goto out;
		}

		/* Avoid a divide-by-zero. */

		if (tmp == CQSPI_PHY_MID_TEMP)
			tmp++;
		dev_dbg(dev, "Temperature: %dC\n", tmp);

		searchpoint.tx += (topright.tx - bottomleft.tx) /
				  (330 / (tmp - CQSPI_PHY_MID_TEMP));
		searchpoint.rx += (topright.rx - bottomleft.rx) /
				  (330 / (tmp - CQSPI_PHY_MID_TEMP));
	} else {
		/*
		 * If there are two passing regions, find the start and end of
		 * the second one.
		 */

		ret = cqspi_phy_find_gaphigh(f_pdata, mem, &bottomleft,
					     &topright, &gaphigh);
		if (ret)
			goto out;
		dev_dbg(dev, "gaphigh: RX: %d TX: %d RD: %d\n", gaphigh.rx,
			gaphigh.tx, gaphigh.read_delay);

		/*
		 * Place the final tuning point in the corner furthest from the
		 * failing region but leave some margin for temperature
		 * changes.
		 */

		if ((abs(gaplow.tx - bottomleft.tx) +
		     abs(gaplow.rx - bottomleft.rx)) <
		    (abs(gaphigh.tx - topright.tx) +
		     abs(gaphigh.rx - topright.rx))) {
			searchpoint = topright;
			searchpoint.tx -= 16;
			searchpoint.rx -= (16 * (topright.rx - bottomleft.rx))
					  / (topright.tx - bottomleft.tx);
		} else {
			searchpoint = bottomleft;
			searchpoint.tx += 16;
			searchpoint.rx += (16 * (topright.rx - bottomleft.rx))
					  / (topright.tx - bottomleft.tx);
		}
	}

	/* Set the final PHY settings and check if they are working. */

	dev_dbg(dev, "Final tuning point: RX: %d TX: %d RD: %d\n",
		searchpoint.rx, searchpoint.tx, searchpoint.read_delay);
	ret = cqspi_phy_apply_setting(f_pdata, &searchpoint);
	if (!ret)
		ret = cqspi_phy_check_pattern(f_pdata, mem);

	if (ret) {
		dev_err(dev,
			"Failed to find pattern at final calibration point\n");
		ret = -EINVAL;
		goto out;
	}

	ret = 0;
	f_pdata->phy_setting.read_delay = searchpoint.read_delay;
	f_pdata->phy_setting.rx = searchpoint.rx;
	f_pdata->phy_setting.tx = searchpoint.tx;
out:
	if (ret)
		f_pdata->use_phy = false;
	return ret;
}

static void cqspi_phy_reset_setting(struct phy_setting *phy)
{
	phy->rx = 0;
	phy->tx = 127;
	phy->read_delay = 0;
}

static int cqspi_phy_calibrate_sdr(struct cqspi_flash_pdata *f_pdata,
				   struct spi_mem *mem)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	struct phy_setting rxlow, rxhigh, first, second, final;
	char window1 = 0;
	char window2 = 0;
	int ret;

	f_pdata->use_phy = true;
	cqspi_phy_reset_setting(&rxlow);
	cqspi_phy_reset_setting(&rxhigh);
	cqspi_phy_reset_setting(&first);

	do {
		ret = cqspi_find_rx_low_sdr(f_pdata, mem, &rxlow);

		if (ret)
			rxlow.read_delay++;
	} while (ret && rxlow.read_delay <= CQSPI_PHY_MAX_RD);

	rxhigh.read_delay = rxlow.read_delay;
	ret = cqspi_find_rx_high_sdr(f_pdata, mem, &rxhigh, rxlow.rx);
	if (ret)
		goto out;

	first.read_delay = rxlow.read_delay;
	window1 = rxhigh.rx - rxlow.rx;
	first.rx = rxlow.rx + (window1 / 2);

	dev_dbg(dev, "First tuning point: RX: %d TX: %d RD: %d\n", first.rx,
		first.tx, first.read_delay);
	ret = cqspi_phy_apply_setting(f_pdata, &first);
	if (!ret)
		ret = cqspi_phy_check_pattern(f_pdata, mem);

	if (ret || first.read_delay > CQSPI_PHY_MAX_RD)
		goto out;

	cqspi_phy_reset_setting(&rxlow);
	cqspi_phy_reset_setting(&rxhigh);
	cqspi_phy_reset_setting(&second);

	rxlow.read_delay = first.read_delay + 1;
	if (rxlow.read_delay > CQSPI_PHY_MAX_RD)
		goto compare;

	ret = cqspi_find_rx_low_sdr(f_pdata, mem, &rxlow);
	if (ret)
		goto compare;

	rxhigh.read_delay = rxlow.read_delay;
	ret = cqspi_find_rx_high_sdr(f_pdata, mem, &rxhigh, rxlow.rx);
	if (ret)
		goto compare;

	window2 = rxhigh.rx - rxlow.rx;
	second.rx = rxlow.rx + (window2 / 2);
	second.read_delay = rxlow.read_delay;

	dev_dbg(dev, "Second tuning point: RX: %d TX: %d RD: %d\n", second.rx,
		second.tx, second.read_delay);
	ret = cqspi_phy_apply_setting(f_pdata, &second);
	if (!ret)
		ret = cqspi_phy_check_pattern(f_pdata, mem);

	if (ret || second.read_delay > CQSPI_PHY_MAX_RD)
		window2 = 0;

compare:
	cqspi_phy_reset_setting(&final);
	if (window2 > window1) {
		final.rx = second.rx;
		final.read_delay = second.read_delay;
	} else {
		final.rx = first.rx;
		final.read_delay = first.read_delay;
	}

	dev_dbg(dev, "Final tuning point: RX: %d TX: %d RD: %d\n", final.rx,
		final.tx, final.read_delay);
	ret = cqspi_phy_apply_setting(f_pdata, &final);
	if (!ret)
		ret = cqspi_phy_check_pattern(f_pdata, mem);

	if (ret) {
		ret = -EINVAL;
		goto out;
	}

	f_pdata->phy_setting.read_delay = final.read_delay;
	f_pdata->phy_setting.rx = final.rx;
	f_pdata->phy_setting.tx = final.tx;

out:
	if (ret)
		f_pdata->use_phy = false;

	return ret;
}

static int cqspi_wait_for_bit(void __iomem *reg, const u32 mask, bool clr)
{
	u32 val;

	return readl_relaxed_poll_timeout(reg, val,
					  (((clr ? ~val : val) & mask) == mask),
					  10, CQSPI_TIMEOUT_MS * 1000);
}

static u32 cqspi_get_rd_sram_level(struct cqspi_st *cqspi)
{
	u32 reg = readl(cqspi->iobase + CQSPI_REG_SDRAMLEVEL);

	reg >>= CQSPI_REG_SDRAMLEVEL_RD_LSB;
	return reg & CQSPI_REG_SDRAMLEVEL_RD_MASK;
}

static u32 cqspi_get_versal_dma_status(struct cqspi_st *cqspi)
{
	u32 dma_status;

	dma_status = readl(cqspi->iobase +
					   CQSPI_REG_VERSAL_DMA_DST_I_STS);
	writel(dma_status, cqspi->iobase +
		   CQSPI_REG_VERSAL_DMA_DST_I_STS);

	return dma_status & CQSPI_REG_VERSAL_DMA_DST_DONE_MASK;
}

static irqreturn_t cqspi_irq_handler(int this_irq, void *dev)
{
	struct cqspi_st *cqspi = dev;
	unsigned int irq_status;
	struct device *device = &cqspi->pdev->dev;
	const struct cqspi_driver_platdata *ddata;

	ddata = of_device_get_match_data(device);

	/* Read interrupt status */
	irq_status = readl(cqspi->iobase + CQSPI_REG_IRQSTATUS);

	/* Clear interrupt */
	writel(irq_status, cqspi->iobase + CQSPI_REG_IRQSTATUS);

	if (cqspi->use_dma_read && ddata && ddata->get_dma_status) {
		if (ddata->get_dma_status(cqspi)) {
			complete(&cqspi->transfer_complete);
			return IRQ_HANDLED;
		}
	}

	else if (!cqspi->slow_sram)
		irq_status &= CQSPI_IRQ_MASK_RD | CQSPI_IRQ_MASK_WR;
	else
		irq_status &= CQSPI_REG_IRQ_WATERMARK | CQSPI_IRQ_MASK_WR;

	if (irq_status)
		complete(&cqspi->transfer_complete);

	return IRQ_HANDLED;
}

static unsigned int cqspi_calc_rdreg(const struct spi_mem_op *op)
{
	u32 rdreg = 0;

	rdreg |= CQSPI_OP_WIDTH(op->cmd) << CQSPI_REG_RD_INSTR_TYPE_INSTR_LSB;
	rdreg |= CQSPI_OP_WIDTH(op->addr) << CQSPI_REG_RD_INSTR_TYPE_ADDR_LSB;
	rdreg |= CQSPI_OP_WIDTH(op->data) << CQSPI_REG_RD_INSTR_TYPE_DATA_LSB;

	return rdreg;
}

static unsigned int cqspi_calc_dummy(const struct spi_mem_op *op)
{
	unsigned int dummy_clk;

	if (!op->dummy.nbytes)
		return 0;

	dummy_clk = op->dummy.nbytes * (8 / op->dummy.buswidth);
	if (op->cmd.dtr)
		dummy_clk /= 2;

	return dummy_clk;
}

static void cqspi_readdata_capture(struct cqspi_st *cqspi, const bool bypass,
				   const bool dqs, const unsigned int delay)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_READCAPTURE);

	if (bypass)
		reg |= (1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);
	else
		reg &= ~(1 << CQSPI_REG_READCAPTURE_BYPASS_LSB);

	reg &= ~(CQSPI_REG_READCAPTURE_DELAY_MASK
		 << CQSPI_REG_READCAPTURE_DELAY_LSB);

	reg |= (delay & CQSPI_REG_READCAPTURE_DELAY_MASK)
	       << CQSPI_REG_READCAPTURE_DELAY_LSB;

	if (dqs)
		reg |= (1 << CQSPI_REG_READCAPTURE_DQS_LSB);
	else
		reg &= ~(1 << CQSPI_REG_READCAPTURE_DQS_LSB);

	writel(reg, reg_base + CQSPI_REG_READCAPTURE);
}

static void cqspi_phy_enable(struct cqspi_flash_pdata *f_pdata, bool enable)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u32 reg;
	u8 dummy;

	if (enable) {
		cqspi_readdata_capture(cqspi, 1, f_pdata->use_dqs,
				       f_pdata->phy_setting.read_delay);

		reg = readl(reg_base + CQSPI_REG_CONFIG);
		reg |= CQSPI_REG_CONFIG_PHY_EN | CQSPI_REG_CONFIG_PHY_PIPELINE;
		writel(reg, reg_base + CQSPI_REG_CONFIG);

		/*
		 * Reduce dummy cycle by 1. This is a requirement of PHY mode
		 * operation for correctly reading the data.
		 */
		reg = readl(reg_base + CQSPI_REG_RD_INSTR);
		dummy = (reg >> CQSPI_REG_RD_INSTR_DUMMY_LSB) &
			CQSPI_REG_RD_INSTR_DUMMY_MASK;
		dummy--;
		reg &= ~(CQSPI_REG_RD_INSTR_DUMMY_MASK
			 << CQSPI_REG_RD_INSTR_DUMMY_LSB);

		reg |= (dummy & CQSPI_REG_RD_INSTR_DUMMY_MASK)
		       << CQSPI_REG_RD_INSTR_DUMMY_LSB;
		writel(reg, reg_base + CQSPI_REG_RD_INSTR);
	} else {
		cqspi_readdata_capture(cqspi, !cqspi->rclk_en, false,
				       f_pdata->read_delay);

		reg = readl(reg_base + CQSPI_REG_CONFIG);
		reg &= ~(CQSPI_REG_CONFIG_PHY_EN |
			 CQSPI_REG_CONFIG_PHY_PIPELINE);
		writel(reg, reg_base + CQSPI_REG_CONFIG);

		/*
		 * Dummy cycles were decremented when enabling PHY. Increment
		 * dummy cycle by 1 to restore the original value.
		 */
		reg = readl(reg_base + CQSPI_REG_RD_INSTR);
		dummy = (reg >> CQSPI_REG_RD_INSTR_DUMMY_LSB) &
			CQSPI_REG_RD_INSTR_DUMMY_MASK;
		dummy++;
		reg &= ~(CQSPI_REG_RD_INSTR_DUMMY_MASK
			 << CQSPI_REG_RD_INSTR_DUMMY_LSB);

		reg |= (dummy & CQSPI_REG_RD_INSTR_DUMMY_MASK)
		       << CQSPI_REG_RD_INSTR_DUMMY_LSB;
		writel(reg, reg_base + CQSPI_REG_RD_INSTR);
	}

	cqspi_wait_idle(cqspi);
}

static int cqspi_exec_flash_cmd(struct cqspi_st *cqspi, unsigned int reg)
{
	void __iomem *reg_base = cqspi->iobase;
	int ret;

	/* Write the CMDCTRL without start execution. */
	writel(reg, reg_base + CQSPI_REG_CMDCTRL);
	/* Start execute */
	reg |= CQSPI_REG_CMDCTRL_EXECUTE_MASK;
	writel(reg, reg_base + CQSPI_REG_CMDCTRL);

	/* Polling for completion. */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_CMDCTRL,
				 CQSPI_REG_CMDCTRL_INPROGRESS_MASK, 1);
	if (ret) {
		dev_err(&cqspi->pdev->dev,
			"Flash command execution timed out.\n");
		return ret;
	}

	/* Polling QSPI idle status. */
	return cqspi_wait_idle(cqspi);
}

static int cqspi_setup_opcode_ext(struct cqspi_flash_pdata *f_pdata,
				  const struct spi_mem_op *op,
				  unsigned int shift)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	u8 ext;

	if (op->cmd.nbytes != 2)
		return -EINVAL;

	/* Opcode extension is the LSB. */
	ext = op->cmd.opcode & 0xff;

	reg = readl(reg_base + CQSPI_REG_OP_EXT_LOWER);
	reg &= ~(0xff << shift);
	reg |= ext << shift;
	writel(reg, reg_base + CQSPI_REG_OP_EXT_LOWER);

	return 0;
}

static int cqspi_enable_dtr(struct cqspi_flash_pdata *f_pdata,
			    const struct spi_mem_op *op, unsigned int shift)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	int ret;

	reg = readl(reg_base + CQSPI_REG_CONFIG);

	/*
	 * We enable dual byte opcode here. The callers have to set up the
	 * extension opcode based on which type of operation it is.
	 */
	if (op->cmd.dtr) {
		reg |= CQSPI_REG_CONFIG_DTR_PROTO;
		reg |= CQSPI_REG_CONFIG_DUAL_OPCODE;

		/* Set up command opcode extension. */
		ret = cqspi_setup_opcode_ext(f_pdata, op, shift);
		if (ret)
			return ret;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DTR_PROTO;
		reg &= ~CQSPI_REG_CONFIG_DUAL_OPCODE;
	}

	writel(reg, reg_base + CQSPI_REG_CONFIG);

	return cqspi_wait_idle(cqspi);
}

static int cqspi_command_read(struct cqspi_flash_pdata *f_pdata,
			      const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u8 *rxbuf = op->data.buf.in;
	u8 opcode;
	size_t n_rx = op->data.nbytes;
	unsigned int rdreg;
	unsigned int reg;
	unsigned int dummy_clk;
	size_t read_len;
	int status;

	status = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_STIG_LSB);
	if (status)
		return status;

	if (!n_rx || n_rx > CQSPI_STIG_DATA_LEN_MAX || !rxbuf) {
		dev_err(&cqspi->pdev->dev,
			"Invalid input argument, len %zu rxbuf 0x%p\n",
			n_rx, rxbuf);
		return -EINVAL;
	}

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	rdreg = cqspi_calc_rdreg(op);
	writel(rdreg, reg_base + CQSPI_REG_RD_INSTR);

	dummy_clk = cqspi_calc_dummy(op);
	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return -EOPNOTSUPP;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_CMDCTRL_DUMMY_MASK)
		     << CQSPI_REG_CMDCTRL_DUMMY_LSB;

	reg |= (0x1 << CQSPI_REG_CMDCTRL_RD_EN_LSB);

	/* 0 means 1 byte. */
	reg |= (((n_rx - 1) & CQSPI_REG_CMDCTRL_RD_BYTES_MASK)
		<< CQSPI_REG_CMDCTRL_RD_BYTES_LSB);

	/* setup ADDR BIT field */
	if (op->addr.nbytes) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		writel(op->addr.val, reg_base + CQSPI_REG_CMDADDRESS);
	}

	status = cqspi_exec_flash_cmd(cqspi, reg);
	if (status)
		return status;

	reg = readl(reg_base + CQSPI_REG_CMDREADDATALOWER);

	/* Put the read value into rx_buf */
	read_len = (n_rx > 4) ? 4 : n_rx;
	memcpy(rxbuf, &reg, read_len);
	rxbuf += read_len;

	if (n_rx > 4) {
		reg = readl(reg_base + CQSPI_REG_CMDREADDATAUPPER);

		read_len = n_rx - read_len;
		memcpy(rxbuf, &reg, read_len);
	}

	/* Reset CMD_CTRL Reg once command read completes */
	writel(0, reg_base + CQSPI_REG_CMDCTRL);

	return 0;
}

static int cqspi_command_write(struct cqspi_flash_pdata *f_pdata,
			       const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u8 opcode;
	const u8 *txbuf = op->data.buf.out;
	size_t n_tx = op->data.nbytes;
	unsigned int reg;
	unsigned int data;
	size_t write_len;
	int ret;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_STIG_LSB);
	if (ret)
		return ret;

	if (n_tx > CQSPI_STIG_DATA_LEN_MAX || (n_tx && !txbuf)) {
		dev_err(&cqspi->pdev->dev,
			"Invalid input argument, cmdlen %zu txbuf 0x%p\n",
			n_tx, txbuf);
		return -EINVAL;
	}

	reg = cqspi_calc_rdreg(op);
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_CMDCTRL_OPCODE_LSB;

	if (op->addr.nbytes) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_ADDR_EN_LSB);
		reg |= ((op->addr.nbytes - 1) &
			CQSPI_REG_CMDCTRL_ADD_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_ADD_BYTES_LSB;

		writel(op->addr.val, reg_base + CQSPI_REG_CMDADDRESS);
	}

	if (n_tx) {
		reg |= (0x1 << CQSPI_REG_CMDCTRL_WR_EN_LSB);
		reg |= ((n_tx - 1) & CQSPI_REG_CMDCTRL_WR_BYTES_MASK)
			<< CQSPI_REG_CMDCTRL_WR_BYTES_LSB;
		data = 0;
		write_len = (n_tx > 4) ? 4 : n_tx;
		memcpy(&data, txbuf, write_len);
		txbuf += write_len;
		writel(data, reg_base + CQSPI_REG_CMDWRITEDATALOWER);

		if (n_tx > 4) {
			data = 0;
			write_len = n_tx - 4;
			memcpy(&data, txbuf, write_len);
			writel(data, reg_base + CQSPI_REG_CMDWRITEDATAUPPER);
		}
	}

	ret = cqspi_exec_flash_cmd(cqspi, reg);

	/* Reset CMD_CTRL Reg once command write completes */
	writel(0, reg_base + CQSPI_REG_CMDCTRL);

	return ret;
}

static int cqspi_read_setup(struct cqspi_flash_pdata *f_pdata,
			    const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int dummy_clk = 0;
	unsigned int reg;
	int ret;
	u8 opcode;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_READ_LSB);
	if (ret)
		return ret;

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	reg = opcode << CQSPI_REG_RD_INSTR_OPCODE_LSB;
	reg |= cqspi_calc_rdreg(op);

	/* Setup dummy clock cycles */
	dummy_clk = cqspi_calc_dummy(op);

	if (dummy_clk > CQSPI_DUMMY_CLKS_MAX)
		return -EOPNOTSUPP;

	if (dummy_clk)
		reg |= (dummy_clk & CQSPI_REG_RD_INSTR_DUMMY_MASK)
		       << CQSPI_REG_RD_INSTR_DUMMY_LSB;

	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	/* Set address width */
	reg = readl(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (op->addr.nbytes - 1);
	writel(reg, reg_base + CQSPI_REG_SIZE);
	readl(reg_base + CQSPI_REG_SIZE); /* Flush posted write. */
	return 0;
}

static int cqspi_indirect_read_execute(struct cqspi_flash_pdata *f_pdata,
				       u8 *rxbuf, loff_t from_addr,
				       const size_t n_rx)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	void __iomem *reg_base = cqspi->iobase;
	void __iomem *ahb_base = cqspi->ahb_base;
	unsigned int remaining = n_rx;
	unsigned int mod_bytes = n_rx % 4;
	unsigned int bytes_to_read = 0;
	u8 *rxbuf_end = rxbuf + n_rx;
	int ret = 0;

	writel(from_addr, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);
	writel(remaining, reg_base + CQSPI_REG_INDIRECTRDBYTES);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	/*
	 * On SoCFPGA platform reading the SRAM is slow due to
	 * hardware limitation and causing read interrupt storm to CPU,
	 * so enabling only watermark interrupt to disable all read
	 * interrupts later as we want to run "bytes to read" loop with
	 * all the read interrupts disabled for max performance.
	 */

	if (!cqspi->slow_sram)
		writel(CQSPI_IRQ_MASK_RD, reg_base + CQSPI_REG_IRQMASK);
	else
		writel(CQSPI_REG_IRQ_WATERMARK, reg_base + CQSPI_REG_IRQMASK);

	reinit_completion(&cqspi->transfer_complete);
	writel(CQSPI_REG_INDIRECTRD_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);
	readl(reg_base + CQSPI_REG_INDIRECTRD); /* Flush posted write. */

	while (remaining > 0) {
		if (!wait_for_completion_timeout(&cqspi->transfer_complete,
						 msecs_to_jiffies(CQSPI_READ_TIMEOUT_MS)))
			ret = -ETIMEDOUT;

		/*
		 * Disable all read interrupts until
		 * we are out of "bytes to read"
		 */
		if (cqspi->slow_sram)
			writel(0x0, reg_base + CQSPI_REG_IRQMASK);

		bytes_to_read = cqspi_get_rd_sram_level(cqspi);

		if (ret && bytes_to_read == 0) {
			dev_err(dev, "Indirect read timeout, no bytes\n");
			goto failrd;
		}

		while (bytes_to_read != 0) {
			unsigned int word_remain = round_down(remaining, 4);

			bytes_to_read *= cqspi->fifo_width;
			bytes_to_read = bytes_to_read > remaining ?
					remaining : bytes_to_read;
			bytes_to_read = round_down(bytes_to_read, 4);
			/* Read 4 byte word chunks then single bytes */
			if (bytes_to_read) {
				ioread32_rep(ahb_base, rxbuf,
					     (bytes_to_read / 4));
			} else if (!word_remain && mod_bytes) {
				unsigned int temp = ioread32(ahb_base);

				bytes_to_read = mod_bytes;
				memcpy(rxbuf, &temp, min((unsigned int)
							 (rxbuf_end - rxbuf),
							 bytes_to_read));
			}
			rxbuf += bytes_to_read;
			remaining -= bytes_to_read;
			bytes_to_read = cqspi_get_rd_sram_level(cqspi);
		}

		if (remaining > 0) {
			reinit_completion(&cqspi->transfer_complete);
			if (cqspi->slow_sram)
				writel(CQSPI_REG_IRQ_WATERMARK, reg_base + CQSPI_REG_IRQMASK);
		}
	}

	/* Check indirect done status */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_INDIRECTRD,
				 CQSPI_REG_INDIRECTRD_DONE_MASK, 0);
	if (ret) {
		dev_err(dev, "Indirect read completion error (%i)\n", ret);
		goto failrd;
	}

	/* Disable interrupt */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTRD_DONE_MASK, reg_base + CQSPI_REG_INDIRECTRD);

	return 0;

failrd:
	/* Disable interrupt */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect read */
	writel(CQSPI_REG_INDIRECTRD_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);
	return ret;
}

static void cqspi_controller_enable(struct cqspi_st *cqspi, bool enable)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);

	if (enable)
		reg |= CQSPI_REG_CONFIG_ENABLE_MASK;
	else
		reg &= ~CQSPI_REG_CONFIG_ENABLE_MASK;

	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static int cqspi_versal_indirect_read_dma(struct cqspi_flash_pdata *f_pdata,
					  u_char *rxbuf, loff_t from_addr,
					  size_t n_rx)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	void __iomem *reg_base = cqspi->iobase;
	u32 reg, bytes_to_dma;
	loff_t addr = from_addr;
	void *buf = rxbuf;
	dma_addr_t dma_addr;
	u8 bytes_rem;
	int ret = 0;

	bytes_rem = n_rx % 4;
	bytes_to_dma = (n_rx - bytes_rem);

	if (!bytes_to_dma)
		goto nondmard;

	ret = zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id, PM_OSPI_MUX_SEL_DMA);
	if (ret)
		return ret;

	cqspi_controller_enable(cqspi, 0);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg |= CQSPI_REG_CONFIG_DMA_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	cqspi_controller_enable(cqspi, 1);

	dma_addr = dma_map_single(dev, rxbuf, bytes_to_dma, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		dev_err(dev, "dma mapping failed\n");
		return -ENOMEM;
	}

	writel(from_addr, reg_base + CQSPI_REG_INDIRECTRDSTARTADDR);
	writel(bytes_to_dma, reg_base + CQSPI_REG_INDIRECTRDBYTES);
	writel(CQSPI_REG_VERSAL_ADDRRANGE_WIDTH_VAL,
	       reg_base + CQSPI_REG_INDTRIG_ADDRRANGE);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	/* Enable DMA done interrupt */
	writel(CQSPI_REG_VERSAL_DMA_DST_DONE_MASK,
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_I_EN);

	/* Default DMA periph configuration */
	writel(CQSPI_REG_VERSAL_DMA_VAL, reg_base + CQSPI_REG_DMA);

	/* Configure DMA Dst address */
	writel(lower_32_bits(dma_addr),
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_ADDR);
	writel(upper_32_bits(dma_addr),
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_ADDR_MSB);

	/* Configure DMA Src address */
	writel(cqspi->trigger_address, reg_base +
	       CQSPI_REG_VERSAL_DMA_SRC_ADDR);

	/* Set DMA destination size */
	writel(bytes_to_dma, reg_base + CQSPI_REG_VERSAL_DMA_DST_SIZE);

	/* Set DMA destination control */
	writel(CQSPI_REG_VERSAL_DMA_DST_CTRL_VAL,
	       reg_base + CQSPI_REG_VERSAL_DMA_DST_CTRL);

	writel(CQSPI_REG_INDIRECTRD_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	reinit_completion(&cqspi->transfer_complete);

	if (!wait_for_completion_timeout(&cqspi->transfer_complete,
					 msecs_to_jiffies(max_t(size_t, bytes_to_dma, 500)))) {
		ret = -ETIMEDOUT;
		goto failrd;
	}

	/* Disable DMA interrupt */
	writel(0x0, cqspi->iobase + CQSPI_REG_VERSAL_DMA_DST_I_DIS);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTRD_DONE_MASK,
	       cqspi->iobase + CQSPI_REG_INDIRECTRD);
	dma_unmap_single(dev, dma_addr, bytes_to_dma, DMA_FROM_DEVICE);

	cqspi_controller_enable(cqspi, 0);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_DMA_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	cqspi_controller_enable(cqspi, 1);

	ret = zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id,
					PM_OSPI_MUX_SEL_LINEAR);
	if (ret)
		return ret;

nondmard:
	if (bytes_rem) {
		addr += bytes_to_dma;
		buf += bytes_to_dma;
		ret = cqspi_indirect_read_execute(f_pdata, buf, addr,
						  bytes_rem);
		if (ret)
			return ret;
	}

	return 0;

failrd:
	/* Disable DMA interrupt */
	writel(0x0, reg_base + CQSPI_REG_VERSAL_DMA_DST_I_DIS);

	/* Cancel the indirect read */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTRD);

	dma_unmap_single(dev, dma_addr, bytes_to_dma, DMA_FROM_DEVICE);

	reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
	reg &= ~CQSPI_REG_CONFIG_DMA_MASK;
	writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);

	zynqmp_pm_ospi_mux_select(cqspi->pd_dev_id, PM_OSPI_MUX_SEL_LINEAR);

	return ret;
}

static int cqspi_write_setup(struct cqspi_flash_pdata *f_pdata,
			     const struct spi_mem_op *op)
{
	unsigned int reg;
	int ret;
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	u8 opcode;

	ret = cqspi_enable_dtr(f_pdata, op, CQSPI_REG_OP_EXT_WRITE_LSB);
	if (ret)
		return ret;

	if (op->cmd.dtr)
		opcode = op->cmd.opcode >> 8;
	else
		opcode = op->cmd.opcode;

	/* Set opcode. */
	reg = opcode << CQSPI_REG_WR_INSTR_OPCODE_LSB;
	reg |= CQSPI_OP_WIDTH(op->data) << CQSPI_REG_WR_INSTR_TYPE_DATA_LSB;
	reg |= CQSPI_OP_WIDTH(op->addr) << CQSPI_REG_WR_INSTR_TYPE_ADDR_LSB;
	writel(reg, reg_base + CQSPI_REG_WR_INSTR);
	reg = cqspi_calc_rdreg(op);
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	/*
	 * SPI NAND flashes require the address of the status register to be
	 * passed in the Read SR command. Also, some SPI NOR flashes like the
	 * cypress Semper flash expect a 4-byte dummy address in the Read SR
	 * command in DTR mode.
	 *
	 * But this controller does not support address phase in the Read SR
	 * command when doing auto-HW polling. So, disable write completion
	 * polling on the controller's side. spinand and spi-nor will take
	 * care of polling the status register.
	 */
	if (cqspi->wr_completion) {
		reg = readl(reg_base + CQSPI_REG_WR_COMPLETION_CTRL);
		reg |= CQSPI_REG_WR_DISABLE_AUTO_POLL;
		writel(reg, reg_base + CQSPI_REG_WR_COMPLETION_CTRL);
		/*
		 * DAC mode require auto polling as flash needs to be polled
		 * for write completion in case of bubble in SPI transaction
		 * due to slow CPU/DMA master.
		 */
		cqspi->use_direct_mode_wr = false;
	}

	reg = readl(reg_base + CQSPI_REG_SIZE);
	reg &= ~CQSPI_REG_SIZE_ADDRESS_MASK;
	reg |= (op->addr.nbytes - 1);
	writel(reg, reg_base + CQSPI_REG_SIZE);
	readl(reg_base + CQSPI_REG_SIZE); /* Flush posted write. */
	return 0;
}

static int cqspi_indirect_write_execute(struct cqspi_flash_pdata *f_pdata,
					loff_t to_addr, const u8 *txbuf,
					const size_t n_tx)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int remaining = n_tx;
	unsigned int write_bytes;
	int ret;

	writel(to_addr, reg_base + CQSPI_REG_INDIRECTWRSTARTADDR);
	writel(remaining, reg_base + CQSPI_REG_INDIRECTWRBYTES);

	/* Clear all interrupts. */
	writel(CQSPI_IRQ_STATUS_MASK, reg_base + CQSPI_REG_IRQSTATUS);

	writel(CQSPI_IRQ_MASK_WR, reg_base + CQSPI_REG_IRQMASK);

	reinit_completion(&cqspi->transfer_complete);
	writel(CQSPI_REG_INDIRECTWR_START_MASK,
	       reg_base + CQSPI_REG_INDIRECTWR);
	readl(reg_base + CQSPI_REG_INDIRECTWR); /* Flush posted write. */

	/*
	 * As per 66AK2G02 TRM SPRUHY8F section 11.15.5.3 Indirect Access
	 * Controller programming sequence, couple of cycles of
	 * QSPI_REF_CLK delay is required for the above bit to
	 * be internally synchronized by the QSPI module. Provide 5
	 * cycles of delay.
	 */
	if (cqspi->wr_delay)
		ndelay(cqspi->wr_delay);

	/*
	 * If a hazard exists between the APB and AHB interfaces, perform a
	 * dummy readback from the controller to ensure synchronization.
	 */
	if (cqspi->apb_ahb_hazard)
		readl(reg_base + CQSPI_REG_INDIRECTWR);

	if (n_tx >= SZ_1K && f_pdata->use_phy)
		cqspi_phy_enable(f_pdata, true);

	while (remaining > 0) {
		size_t write_words, mod_bytes;

		write_bytes = remaining;
		write_words = write_bytes / 4;
		mod_bytes = write_bytes % 4;
		/* Write 4 bytes at a time then single bytes. */
		if (write_words) {
			iowrite32_rep(cqspi->ahb_base, txbuf, write_words);
			txbuf += (write_words * 4);
		}
		if (mod_bytes) {
			unsigned int temp = 0xFFFFFFFF;

			memcpy(&temp, txbuf, mod_bytes);
			iowrite32(temp, cqspi->ahb_base);
			txbuf += mod_bytes;
		}

		if (!wait_for_completion_timeout(&cqspi->transfer_complete,
						 msecs_to_jiffies(CQSPI_TIMEOUT_MS))) {
			dev_err(dev, "Indirect write timeout\n");
			ret = -ETIMEDOUT;
			goto failwr;
		}

		remaining -= write_bytes;

		if (remaining > 0)
			reinit_completion(&cqspi->transfer_complete);
	}

	/* Check indirect done status */
	ret = cqspi_wait_for_bit(reg_base + CQSPI_REG_INDIRECTWR,
				 CQSPI_REG_INDIRECTWR_DONE_MASK, 0);
	if (ret) {
		dev_err(dev, "Indirect write completion error (%i)\n", ret);
		goto failwr;
	}

	if (n_tx >= SZ_1K && f_pdata->use_phy)
		cqspi_phy_enable(f_pdata, false);

	/* Disable interrupt. */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Clear indirect completion status */
	writel(CQSPI_REG_INDIRECTWR_DONE_MASK, reg_base + CQSPI_REG_INDIRECTWR);

	cqspi_wait_idle(cqspi);

	return 0;

failwr:
	if (n_tx >= SZ_1K && f_pdata->use_phy)
		cqspi_phy_enable(f_pdata, false);

	/* Disable interrupt. */
	writel(0, reg_base + CQSPI_REG_IRQMASK);

	/* Cancel the indirect write */
	writel(CQSPI_REG_INDIRECTWR_CANCEL_MASK,
	       reg_base + CQSPI_REG_INDIRECTWR);
	return ret;
}

static void cqspi_chipselect(struct cqspi_flash_pdata *f_pdata)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *reg_base = cqspi->iobase;
	unsigned int chip_select = f_pdata->cs;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	if (cqspi->is_decoded_cs) {
		reg |= CQSPI_REG_CONFIG_DECODE_MASK;
	} else {
		reg &= ~CQSPI_REG_CONFIG_DECODE_MASK;

		/* Convert CS if without decoder.
		 * CS0 to 4b'1110
		 * CS1 to 4b'1101
		 * CS2 to 4b'1011
		 * CS3 to 4b'0111
		 */
		chip_select = 0xF & ~(1 << chip_select);
	}

	reg &= ~(CQSPI_REG_CONFIG_CHIPSELECT_MASK
		 << CQSPI_REG_CONFIG_CHIPSELECT_LSB);
	reg |= (chip_select & CQSPI_REG_CONFIG_CHIPSELECT_MASK)
	    << CQSPI_REG_CONFIG_CHIPSELECT_LSB;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static unsigned int calculate_ticks_for_ns(const unsigned int ref_clk_hz,
					   const unsigned int ns_val)
{
	unsigned int ticks;

	ticks = ref_clk_hz / 1000;	/* kHz */
	ticks = DIV_ROUND_UP(ticks * ns_val, 1000000);

	return ticks;
}

static void cqspi_delay(struct cqspi_flash_pdata *f_pdata)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	void __iomem *iobase = cqspi->iobase;
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	unsigned int tshsl, tchsh, tslch, tsd2d;
	unsigned int reg;
	unsigned int tsclk;

	/* calculate the number of ref ticks for one sclk tick */
	tsclk = DIV_ROUND_UP(ref_clk_hz, cqspi->sclk);

	tshsl = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tshsl_ns);
	/* this particular value must be at least one sclk */
	if (tshsl < tsclk)
		tshsl = tsclk;

	tchsh = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tchsh_ns);
	tslch = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tslch_ns);
	tsd2d = calculate_ticks_for_ns(ref_clk_hz, f_pdata->tsd2d_ns);

	reg = (tshsl & CQSPI_REG_DELAY_TSHSL_MASK)
	       << CQSPI_REG_DELAY_TSHSL_LSB;
	reg |= (tchsh & CQSPI_REG_DELAY_TCHSH_MASK)
		<< CQSPI_REG_DELAY_TCHSH_LSB;
	reg |= (tslch & CQSPI_REG_DELAY_TSLCH_MASK)
		<< CQSPI_REG_DELAY_TSLCH_LSB;
	reg |= (tsd2d & CQSPI_REG_DELAY_TSD2D_MASK)
		<< CQSPI_REG_DELAY_TSD2D_LSB;
	writel(reg, iobase + CQSPI_REG_DELAY);
}

static void cqspi_config_baudrate_div(struct cqspi_st *cqspi)
{
	const unsigned int ref_clk_hz = cqspi->master_ref_clk_hz;
	void __iomem *reg_base = cqspi->iobase;
	u32 reg, div;

	/* Recalculate the baudrate divisor based on QSPI specification. */
	div = DIV_ROUND_UP(ref_clk_hz, 2 * cqspi->sclk) - 1;

	/* Maximum baud divisor */
	if (div > CQSPI_REG_CONFIG_BAUD_MASK) {
		div = CQSPI_REG_CONFIG_BAUD_MASK;
		dev_warn(&cqspi->pdev->dev,
			"Unable to adjust clock <= %d hz. Reduced to %d hz\n",
			cqspi->sclk, ref_clk_hz/((div+1)*2));
	}

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg &= ~(CQSPI_REG_CONFIG_BAUD_MASK << CQSPI_REG_CONFIG_BAUD_LSB);
	reg |= (div & CQSPI_REG_CONFIG_BAUD_MASK) << CQSPI_REG_CONFIG_BAUD_LSB;
	writel(reg, reg_base + CQSPI_REG_CONFIG);
}

static void cqspi_phy_set_dll_master(struct cqspi_st *cqspi)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;

	reg = readl(reg_base + CQSPI_REG_PHY_DLL_MASTER);
	reg &= ~((CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LEN
		  << CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LSB) |
		 CQSPI_REG_PHY_DLL_MASTER_BYPASS |
		 CQSPI_REG_PHY_DLL_MASTER_CYCLE);
	reg |= ((cqspi->phase_detect_selector
		 << CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LSB) |
		CQSPI_REG_PHY_DLL_MASTER_CYCLE);

	writel(reg, reg_base + CQSPI_REG_PHY_DLL_MASTER);
}

static void cqspi_phy_pre_config(struct cqspi_st *cqspi, const bool bypass,
				 const bool dqs,
				 struct cqspi_flash_pdata *f_pdata)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	u8 dummy;

	cqspi_readdata_capture(cqspi, bypass, dqs,
			       f_pdata->phy_setting.read_delay);

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg &= ~(CQSPI_REG_CONFIG_PHY_EN | CQSPI_REG_CONFIG_PHY_PIPELINE);
	reg |= CQSPI_REG_CONFIG_PHY_EN;
	writel(reg, reg_base + CQSPI_REG_CONFIG);

	reg = readl(reg_base + CQSPI_REG_RD_INSTR);
	dummy = (reg >> CQSPI_REG_RD_INSTR_DUMMY_LSB) &
		CQSPI_REG_RD_INSTR_DUMMY_MASK;
	dummy--;
	reg &= ~(CQSPI_REG_RD_INSTR_DUMMY_MASK << CQSPI_REG_RD_INSTR_DUMMY_LSB);

	reg |= (dummy & CQSPI_REG_RD_INSTR_DUMMY_MASK)
	       << CQSPI_REG_RD_INSTR_DUMMY_LSB;
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);

	cqspi_phy_set_dll_master(cqspi);
}

static void cqspi_phy_post_config(struct cqspi_st *cqspi,
				  const unsigned int delay)
{
	void __iomem *reg_base = cqspi->iobase;
	unsigned int reg;
	u8 dummy;

	reg = readl(reg_base + CQSPI_REG_READCAPTURE);
	reg &= ~(CQSPI_REG_READCAPTURE_DELAY_MASK
		 << CQSPI_REG_READCAPTURE_DELAY_LSB);

	reg |= (delay & CQSPI_REG_READCAPTURE_DELAY_MASK)
	       << CQSPI_REG_READCAPTURE_DELAY_LSB;
	writel(reg, reg_base + CQSPI_REG_READCAPTURE);

	reg = readl(reg_base + CQSPI_REG_CONFIG);
	reg &= ~(CQSPI_REG_CONFIG_PHY_EN | CQSPI_REG_CONFIG_PHY_PIPELINE);
	reg &= ~(CQSPI_REG_CONFIG_PHY_EN);
	writel(reg, reg_base + CQSPI_REG_CONFIG);

	reg = readl(reg_base + CQSPI_REG_RD_INSTR);
	dummy = (reg >> CQSPI_REG_RD_INSTR_DUMMY_LSB) &
		CQSPI_REG_RD_INSTR_DUMMY_MASK;
	dummy++;
	reg &= ~(CQSPI_REG_RD_INSTR_DUMMY_MASK << CQSPI_REG_RD_INSTR_DUMMY_LSB);

	reg |= (dummy & CQSPI_REG_RD_INSTR_DUMMY_MASK)
	       << CQSPI_REG_RD_INSTR_DUMMY_LSB;
	writel(reg, reg_base + CQSPI_REG_RD_INSTR);
}

static void cqspi_configure(struct cqspi_flash_pdata *f_pdata,
			    unsigned long sclk)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	int switch_cs = (cqspi->current_cs != f_pdata->cs);
	int switch_ck = (cqspi->sclk != sclk);

	if (switch_cs || switch_ck)
		cqspi_controller_enable(cqspi, 0);

	/* Switch chip select. */
	if (switch_cs) {
		cqspi->current_cs = f_pdata->cs;
		cqspi_chipselect(f_pdata);
	}

	/* Setup baudrate divisor and delays */
	if (switch_ck) {
		cqspi->sclk = sclk;
		cqspi_config_baudrate_div(cqspi);
		cqspi_delay(f_pdata);
		cqspi_readdata_capture(cqspi, !cqspi->rclk_en, false,
				       f_pdata->read_delay);
	}

	if (switch_cs || switch_ck)
		cqspi_controller_enable(cqspi, 1);
}

static ssize_t cqspi_write(struct cqspi_flash_pdata *f_pdata,
			   const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	loff_t to = op->addr.val;
	size_t len = op->data.nbytes;
	const u_char *buf = op->data.buf.out;
	int ret;

	ret = cqspi_write_setup(f_pdata, op);
	if (ret)
		return ret;

	/*
	 * Some flashes like the Cypress Semper flash expect a dummy 4-byte
	 * address (all 0s) with the read status register command in DTR mode.
	 * But this controller does not support sending dummy address bytes to
	 * the flash when it is polling the write completion register in DTR
	 * mode. So, we can not use direct mode when in DTR mode for writing
	 * data.
	 */
	if (!op->cmd.dtr && cqspi->use_direct_mode &&
	    cqspi->use_direct_mode_wr && ((to + len) <= cqspi->ahb_size)) {
		memcpy_toio(cqspi->ahb_base + to, buf, len);
		return cqspi_wait_idle(cqspi);
	}

	return cqspi_indirect_write_execute(f_pdata, to, buf, len);
}

/*
 * Check if PHY mode can be used on the given op. This is assuming it will be a
 * DAC mode read, since PHY won't work on any other type of operation anyway.
 */
static bool cqspi_phy_op_eligible(const struct spi_mem_op *op)
{
	/* PHY is only tuned for 8D-8D-8D. */
	if (!(op->cmd.dtr && op->addr.dtr && op->dummy.dtr && op->data.dtr))
		return false;
	if (op->cmd.buswidth != 8)
		return false;
	if (!(op->addr.nbytes) || op->addr.buswidth != 8)
		return false;
	if (!(op->dummy.nbytes) || op->dummy.buswidth != 8)
		return false;
	if (!(op->data.nbytes) || op->data.buswidth != 8)
		return false;

	return true;
}

static bool cqspi_phy_op_eligible_sdr(const struct spi_mem_op *op)
{
	if (op->cmd.dtr || op->addr.dtr || op->dummy.dtr || op->data.dtr)
		return false;
	if (!(op->addr.nbytes) || op->addr.buswidth < 1)
		return false;
	if (!(op->dummy.nbytes) || op->dummy.buswidth < 1)
		return false;
	if (!(op->data.nbytes) || op->data.buswidth < 1)
		return false;

	return true;
}

static bool cqspi_use_phy(struct cqspi_flash_pdata *f_pdata,
			  const struct spi_mem_op *op)
{
	if (!f_pdata->use_phy)
		return false;

	if (op->data.nbytes < 16)
		return false;

	if (f_pdata->use_dqs)
		return cqspi_phy_op_eligible(op);
	else
		return cqspi_phy_op_eligible_sdr(op);
}

static void cqspi_rx_dma_callback(void *param)
{
	struct cqspi_st *cqspi = param;

	complete(&cqspi->rx_dma_complete);
}

static int cqspi_direct_read_dma(struct cqspi_flash_pdata *f_pdata,
				 u_char *buf, loff_t from, size_t len)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	dma_addr_t dma_src = (dma_addr_t)cqspi->mmap_phys_base + from;
	int ret = 0;
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie;
	dma_addr_t dma_dst;
	struct device *ddev;

	ddev = cqspi->rx_chan->device->dev;
	dma_dst = dma_map_single(ddev, buf, len, DMA_FROM_DEVICE);
	if (dma_mapping_error(ddev, dma_dst)) {
		dev_err(dev, "dma mapping failed\n");
		return -ENOMEM;
	}
	tx = dmaengine_prep_dma_memcpy(cqspi->rx_chan, dma_dst, dma_src,
				       len, flags);
	if (!tx) {
		dev_err(dev, "device_prep_dma_memcpy error\n");
		ret = -EIO;
		goto err_unmap;
	}

	tx->callback = cqspi_rx_dma_callback;
	tx->callback_param = cqspi;
	cookie = tx->tx_submit(tx);
	reinit_completion(&cqspi->rx_dma_complete);

	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "dma_submit_error %d\n", cookie);
		ret = -EIO;
		goto err_unmap;
	}

	dma_async_issue_pending(cqspi->rx_chan);
	if (!wait_for_completion_timeout(&cqspi->rx_dma_complete,
					 msecs_to_jiffies(max_t(size_t, len, 500)))) {
		dmaengine_terminate_sync(cqspi->rx_chan);
		dev_err(dev, "DMA wait_for_completion_timeout\n");
		ret = -ETIMEDOUT;
		goto err_unmap;
	}

err_unmap:
	dma_unmap_single(ddev, dma_dst, len, DMA_FROM_DEVICE);

	return ret;
}

static void cqspi_memcpy_fromio(const struct spi_mem_op *op, void *to,
				const void __iomem *from, size_t count)
{
	if (op->data.buswidth == 8 && op->data.dtr) {
		/*
		 * 8D-8D-8D ops with odd length should be rejected by
		 * supports_op() so no need to worry about that.
		 */
		while (count && !IS_ALIGNED((unsigned long)from, 4)) {
			*(u16 *)to = __raw_readw(from);
			from += 2;
			to += 2;
			count -= 2;
		}

		/*
		 * The controller can work with both 32-bit and 64-bit
		 * platforms. 32-bit platforms won't have a readq. So use a
		 * readl instead.
		 */
		while (count >= 4) {
			*(u32 *)to = __raw_readl(from);
			from += 4;
			to += 4;
			count -= 4;
		}

		while (count) {
			*(u16 *)to = __raw_readw(from);
			from += 2;
			to += 2;
			count -= 2;
		}

		return;
	}

	memcpy_fromio(to, from, count);
}

static int cqspi_direct_read_execute(struct cqspi_flash_pdata *f_pdata,
				     const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	loff_t from = op->addr.val;
	loff_t from_aligned, to_aligned;
	size_t len = op->data.nbytes;
	size_t len_aligned;
	u_char *buf = op->data.buf.in;
	int ret;

	if (!cqspi->rx_chan || !virt_addr_valid(buf) || len <= 16) {
		cqspi_memcpy_fromio(op, buf, cqspi->ahb_base + from, len);
		return 0;
	}

	if (!cqspi_use_phy(f_pdata, op))
		return cqspi_direct_read_dma(f_pdata, buf, from, len);

	/*
	 * PHY reads must be 16-byte aligned, and they must be a multiple of 16
	 * bytes.
	 */
	from_aligned = (from + 0xF) & ~0xF;
	to_aligned = (from + len) & ~0xF;
	len_aligned = to_aligned - from_aligned;

	/* Read the unaligned part at the start. */
	if (from != from_aligned) {
		ret = cqspi_direct_read_dma(f_pdata, buf, from,
					    from_aligned - from);
		if (ret)
			return ret;
		buf += from_aligned - from;
	}

	if (len_aligned) {
		cqspi_phy_enable(f_pdata, true);
		ret = cqspi_direct_read_dma(f_pdata, buf, from_aligned,
					    len_aligned);
		cqspi_phy_enable(f_pdata, false);
		if (ret)
			return ret;
		buf += len_aligned;
	}

	/* Now read the remaining part, if any. */
	if (to_aligned != (from + len)) {
		ret = cqspi_direct_read_dma(f_pdata, buf, to_aligned,
					    (from + len) - to_aligned);
		if (ret)
			return ret;
		buf += (from + len) - to_aligned;
	}

	return 0;
}

static ssize_t cqspi_read(struct cqspi_flash_pdata *f_pdata,
			  const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = f_pdata->cqspi;
	struct device *dev = &cqspi->pdev->dev;
	const struct cqspi_driver_platdata *ddata;
	loff_t from = op->addr.val;
	size_t len = op->data.nbytes;
	u_char *buf = op->data.buf.in;
	u64 dma_align = (u64)(uintptr_t)buf;
	int ret;

	ddata = of_device_get_match_data(dev);

	ret = cqspi_read_setup(f_pdata, op);
	if (ret)
		return ret;

	if (cqspi->use_direct_mode && ((from + len) <= cqspi->ahb_size))
		return cqspi_direct_read_execute(f_pdata, op);

	if (cqspi->use_dma_read && ddata && ddata->indirect_read_dma &&
	    virt_addr_valid(buf) && ((dma_align & CQSPI_DMA_UNALIGN) == 0))
		return ddata->indirect_read_dma(f_pdata, buf, from, len);

	return cqspi_indirect_read_execute(f_pdata, buf, from, len);
}

static int cqspi_mem_process(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = spi_controller_get_devdata(mem->spi->controller);
	struct cqspi_flash_pdata *f_pdata;

	f_pdata = &cqspi->f_pdata[spi_get_chipselect(mem->spi, 0)];
	cqspi_configure(f_pdata, mem->spi->max_speed_hz);

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.buf.in) {
	/*
	 * Performing reads in DAC mode forces to read minimum 4 bytes
	 * which is unsupported on some flash devices during register
	 * reads, prefer STIG mode for such small reads.
	 */
		if (!op->addr.nbytes ||
		    op->data.nbytes <= CQSPI_STIG_DATA_LEN_MAX)
			return cqspi_command_read(f_pdata, op);

		return cqspi_read(f_pdata, op);
	}

	if (!op->addr.nbytes || !op->data.buf.out)
		return cqspi_command_write(f_pdata, op);

	return cqspi_write(f_pdata, op);
}

static int cqspi_exec_mem_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	int ret;
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	struct device *dev = &cqspi->pdev->dev;

	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err(&mem->spi->dev, "resume failed with %d\n", ret);
		return ret;
	}

	ret = cqspi_mem_process(mem, op);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	if (ret)
		dev_err(&mem->spi->dev, "operation failed with %d\n", ret);

	return ret;
}

static bool cqspi_supports_mem_op(struct spi_mem *mem,
				  const struct spi_mem_op *op)
{
	bool all_true, all_false;

	/*
	 * op->dummy.dtr is required for converting nbytes into ncycles.
	 * Also, don't check the dtr field of the op phase having zero nbytes.
	 */
	all_true = op->cmd.dtr &&
		   (!op->addr.nbytes || op->addr.dtr) &&
		   (!op->dummy.nbytes || op->dummy.dtr) &&
		   (!op->data.nbytes || op->data.dtr);

	all_false = !op->cmd.dtr && !op->addr.dtr && !op->dummy.dtr &&
		    !op->data.dtr;

	if (all_true) {
		/* Right now we only support 8-8-8 DTR mode. */
		if (op->cmd.nbytes && op->cmd.buswidth != 8)
			return false;
		if (op->addr.nbytes && op->addr.buswidth != 8)
			return false;
		if (op->data.nbytes && op->data.buswidth != 8)
			return false;
	} else if (!all_false) {
		/* Mixed DTR modes are not supported. */
		return false;
	}

	return spi_mem_default_supports_op(mem, op);
}

static void cqspi_mem_do_calibration(struct spi_mem *mem,
				     const struct spi_mem_op *op)
{
	struct cqspi_st *cqspi = spi_master_get_devdata(mem->spi->master);
	struct cqspi_flash_pdata *f_pdata;
	struct device *dev = &cqspi->pdev->dev;
	int ret;

	f_pdata = &cqspi->f_pdata[mem->spi->chip_select];
	f_pdata->phy_read_op = *op;

	if (cqspi->phase_detect_selector >
	    CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LEN) {
		dev_warn(dev,
			 "Phase Detect Selector should be in between [0, 7]. Skipping Calibration\n");
		return;
	}

	ret = cqspi_phy_check_pattern(f_pdata, mem);
	if (ret) {
		dev_warn(dev, "Pattern not found. Skipping calibration.\n");
		return;
	}

	if (cqspi_phy_op_eligible(op)) {
		f_pdata->use_dqs = true;

		cqspi_phy_pre_config(cqspi, 0, 1, f_pdata);
		ret = cqspi_phy_calibrate(f_pdata, mem);

	} else if (cqspi_phy_op_eligible_sdr(op)) {
		f_pdata->use_dqs = false;

		cqspi_phy_pre_config(cqspi, 1, 0, f_pdata);
		ret = cqspi_phy_calibrate_sdr(f_pdata, mem);

	} else {
		dev_warn(dev,
			 "Given read_op not eligible. Skipping Calibration.\n");
		return;
	}

	if (ret)
		dev_info(&cqspi->pdev->dev, "PHY calibration failed: %d\n",
			 ret);

	cqspi_phy_post_config(cqspi, f_pdata->read_delay);
}

static int cqspi_of_get_flash_pdata(struct platform_device *pdev,
				    struct cqspi_flash_pdata *f_pdata,
				    struct device_node *np)
{
	if (of_property_read_u32(np, "cdns,read-delay", &f_pdata->read_delay)) {
		dev_err(&pdev->dev, "couldn't determine read-delay\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tshsl-ns", &f_pdata->tshsl_ns)) {
		dev_err(&pdev->dev, "couldn't determine tshsl-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tsd2d-ns", &f_pdata->tsd2d_ns)) {
		dev_err(&pdev->dev, "couldn't determine tsd2d-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tchsh-ns", &f_pdata->tchsh_ns)) {
		dev_err(&pdev->dev, "couldn't determine tchsh-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,tslch-ns", &f_pdata->tslch_ns)) {
		dev_err(&pdev->dev, "couldn't determine tslch-ns\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "spi-max-frequency", &f_pdata->clk_rate)) {
		dev_err(&pdev->dev, "couldn't determine spi-max-frequency\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,phy-tx-start", &f_pdata->phy_tx_start))
		f_pdata->phy_tx_start = 16;

	if (of_property_read_u32(np, "cdns,phy-tx-end", &f_pdata->phy_tx_end))
		f_pdata->phy_tx_end = 48;

	return 0;
}

static int cqspi_of_get_pdata(struct cqspi_st *cqspi)
{
	struct device *dev = &cqspi->pdev->dev;
	struct device_node *np = dev->of_node;
	u32 id[2];

	cqspi->is_decoded_cs = of_property_read_bool(np, "cdns,is-decoded-cs");

	if (of_property_read_u32(np, "cdns,fifo-depth", &cqspi->fifo_depth)) {
		dev_err(dev, "couldn't determine fifo-depth\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,fifo-width", &cqspi->fifo_width)) {
		dev_err(dev, "couldn't determine fifo-width\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,trigger-address",
				 &cqspi->trigger_address)) {
		dev_err(dev, "couldn't determine trigger-address\n");
		return -ENXIO;
	}

	if (of_property_read_u32(np, "cdns,phase-detect-selector",
				 &cqspi->phase_detect_selector)) {
		dev_warn(dev, "couldn't determine phase-detect-selector\n");
		cqspi->phase_detect_selector =
			CQSPI_REG_PHY_DLL_MASTER_DLY_ELMTS_LEN + 1;
	}

	if (of_property_read_u32(np, "num-cs", &cqspi->num_chipselect))
		cqspi->num_chipselect = CQSPI_MAX_CHIPSELECT;

	cqspi->rclk_en = of_property_read_bool(np, "cdns,rclk-en");

	if (!of_property_read_u32_array(np, "power-domains", id,
					ARRAY_SIZE(id)))
		cqspi->pd_dev_id = id[1];

	return 0;
}

static void cqspi_controller_init(struct cqspi_st *cqspi)
{
	u32 reg;

	cqspi_controller_enable(cqspi, 0);

	/* Configure the remap address register, no remap */
	writel(0, cqspi->iobase + CQSPI_REG_REMAP);

	/* Disable all interrupts. */
	writel(0, cqspi->iobase + CQSPI_REG_IRQMASK);

	/* Configure the SRAM split to 1:1 . */
	writel(cqspi->fifo_depth / 2, cqspi->iobase + CQSPI_REG_SRAMPARTITION);

	/* Load indirect trigger address. */
	writel(cqspi->trigger_address,
	       cqspi->iobase + CQSPI_REG_INDIRECTTRIGGER);

	/* Program read watermark -- 1/2 of the FIFO. */
	writel(cqspi->fifo_depth * cqspi->fifo_width / 2,
	       cqspi->iobase + CQSPI_REG_INDIRECTRDWATERMARK);
	/* Program write watermark -- 1/8 of the FIFO. */
	writel(cqspi->fifo_depth * cqspi->fifo_width / 8,
	       cqspi->iobase + CQSPI_REG_INDIRECTWRWATERMARK);

	/* Disable direct access controller */
	if (!cqspi->use_direct_mode) {
		reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
		reg &= ~CQSPI_REG_CONFIG_ENB_DIR_ACC_CTRL;
		writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);
	}

	/* Enable DMA interface */
	if (cqspi->use_dma_read) {
		reg = readl(cqspi->iobase + CQSPI_REG_CONFIG);
		reg |= CQSPI_REG_CONFIG_DMA_MASK;
		writel(reg, cqspi->iobase + CQSPI_REG_CONFIG);
	}

	cqspi_controller_enable(cqspi, 1);
}

static int cqspi_request_mmap_dma(struct cqspi_st *cqspi)
{
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	cqspi->rx_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(cqspi->rx_chan)) {
		int ret = (-EPROBE_DEFER);

		cqspi->rx_chan = NULL;
		return dev_err_probe(&cqspi->pdev->dev, ret, "No Rx DMA available\n");
	}
	init_completion(&cqspi->rx_dma_complete);

	return 0;
}

static const char *cqspi_get_name(struct spi_mem *mem)
{
	struct cqspi_st *cqspi = spi_controller_get_devdata(mem->spi->controller);
	struct device *dev = &cqspi->pdev->dev;

	return devm_kasprintf(dev, GFP_KERNEL, "%s.%d", dev_name(dev),
			      spi_get_chipselect(mem->spi, 0));
}

static const struct spi_controller_mem_ops cqspi_mem_ops = {
	.exec_op = cqspi_exec_mem_op,
	.get_name = cqspi_get_name,
	.supports_op = cqspi_supports_mem_op,
	.do_calibration = cqspi_mem_do_calibration,
};

static const struct spi_controller_mem_caps cqspi_mem_caps = {
	.dtr = true,
};

static int cqspi_setup_flash(struct cqspi_st *cqspi)
{
	struct platform_device *pdev = cqspi->pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cqspi_flash_pdata *f_pdata;
	unsigned int cs;
	int ret;

	/* Get flash device data */
	for_each_available_child_of_node(dev->of_node, np) {
		ret = of_property_read_u32(np, "reg", &cs);
		if (ret) {
			dev_err(dev, "Couldn't determine chip select.\n");
			of_node_put(np);
			return ret;
		}

		if (cs >= CQSPI_MAX_CHIPSELECT) {
			dev_err(dev, "Chip select %d out of range.\n", cs);
			of_node_put(np);
			return -EINVAL;
		}

		f_pdata = &cqspi->f_pdata[cs];
		f_pdata->cqspi = cqspi;
		f_pdata->cs = cs;

		ret = cqspi_of_get_flash_pdata(pdev, f_pdata, np);
		if (ret) {
			of_node_put(np);
			return ret;
		}
	}

	return 0;
}

static int cqspi_jh7110_clk_init(struct platform_device *pdev, struct cqspi_st *cqspi)
{
	static struct clk_bulk_data qspiclk[] = {
		{ .id = "apb" },
		{ .id = "ahb" },
	};

	int ret = 0;

	ret = devm_clk_bulk_get(&pdev->dev, ARRAY_SIZE(qspiclk), qspiclk);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to get qspi clocks\n", __func__);
		return ret;
	}

	cqspi->clks[CLK_QSPI_APB] = qspiclk[0].clk;
	cqspi->clks[CLK_QSPI_AHB] = qspiclk[1].clk;

	ret = clk_prepare_enable(cqspi->clks[CLK_QSPI_APB]);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to enable CLK_QSPI_APB\n", __func__);
		return ret;
	}

	ret = clk_prepare_enable(cqspi->clks[CLK_QSPI_AHB]);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to enable CLK_QSPI_AHB\n", __func__);
		goto disable_apb_clk;
	}

	cqspi->is_jh7110 = true;

	return 0;

disable_apb_clk:
	clk_disable_unprepare(cqspi->clks[CLK_QSPI_APB]);

	return ret;
}

static void cqspi_jh7110_disable_clk(struct platform_device *pdev, struct cqspi_st *cqspi)
{
	clk_disable_unprepare(cqspi->clks[CLK_QSPI_AHB]);
	clk_disable_unprepare(cqspi->clks[CLK_QSPI_APB]);
}
static const struct soc_device_attribute k3_soc_devices[] = {
	{ .family = "AM64X", .revision = "SR1.0" },
	{ /* sentinel */ }
};

static int cqspi_probe(struct platform_device *pdev)
{
	const struct cqspi_driver_platdata *ddata;
	struct reset_control *rstc, *rstc_ocp, *rstc_ref;
	struct device *dev = &pdev->dev;
	struct spi_controller *host;
	struct resource *res_ahb;
	struct cqspi_st *cqspi;
	int ret;
	int irq;

	host = devm_spi_alloc_host(&pdev->dev, sizeof(*cqspi));
	if (!host) {
		dev_err(&pdev->dev, "devm_spi_alloc_host failed\n");
		return -ENOMEM;
	}
	host->mode_bits = SPI_RX_QUAD | SPI_TX_QUAD | SPI_RX_DUAL;
	host->mem_ops = &cqspi_mem_ops;
	host->mem_caps = &cqspi_mem_caps;
	host->dev.of_node = pdev->dev.of_node;

	cqspi = spi_controller_get_devdata(host);

	cqspi->pdev = pdev;
	cqspi->host = host;
	cqspi->is_jh7110 = false;
	platform_set_drvdata(pdev, cqspi);

	/* Obtain configuration from OF. */
	ret = cqspi_of_get_pdata(cqspi);
	if (ret) {
		dev_err(dev, "Cannot get mandatory OF data.\n");
		return -ENODEV;
	}

	/* Obtain QSPI clock. */
	cqspi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(cqspi->clk)) {
		dev_err(dev, "Cannot claim QSPI clock.\n");
		ret = PTR_ERR(cqspi->clk);
		return ret;
	}

	/* Obtain and remap controller address. */
	cqspi->iobase = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cqspi->iobase)) {
		dev_err(dev, "Cannot remap controller address.\n");
		ret = PTR_ERR(cqspi->iobase);
		return ret;
	}

	/* Obtain and remap AHB address. */
	cqspi->ahb_base = devm_platform_get_and_ioremap_resource(pdev, 1, &res_ahb);
	if (IS_ERR(cqspi->ahb_base)) {
		dev_err(dev, "Cannot remap AHB address.\n");
		ret = PTR_ERR(cqspi->ahb_base);
		return ret;
	}
	cqspi->mmap_phys_base = (dma_addr_t)res_ahb->start;
	cqspi->ahb_size = resource_size(res_ahb);

	init_completion(&cqspi->transfer_complete);

	/* Obtain IRQ line. */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENXIO;

	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;


	ret = clk_prepare_enable(cqspi->clk);
	if (ret) {
		dev_err(dev, "Cannot enable QSPI clock.\n");
		goto probe_clk_failed;
	}

	/* Obtain QSPI reset control */
	rstc = devm_reset_control_get_optional_exclusive(dev, "qspi");
	if (IS_ERR(rstc)) {
		ret = PTR_ERR(rstc);
		dev_err(dev, "Cannot get QSPI reset.\n");
		goto probe_reset_failed;
	}

	rstc_ocp = devm_reset_control_get_optional_exclusive(dev, "qspi-ocp");
	if (IS_ERR(rstc_ocp)) {
		ret = PTR_ERR(rstc_ocp);
		dev_err(dev, "Cannot get QSPI OCP reset.\n");
		goto probe_reset_failed;
	}

	if (of_device_is_compatible(pdev->dev.of_node, "starfive,jh7110-qspi")) {
		rstc_ref = devm_reset_control_get_optional_exclusive(dev, "rstc_ref");
		if (IS_ERR(rstc_ref)) {
			ret = PTR_ERR(rstc_ref);
			dev_err(dev, "Cannot get QSPI REF reset.\n");
			goto probe_reset_failed;
		}
		reset_control_assert(rstc_ref);
		reset_control_deassert(rstc_ref);
	}

	reset_control_assert(rstc);
	reset_control_deassert(rstc);

	reset_control_assert(rstc_ocp);
	reset_control_deassert(rstc_ocp);

	cqspi->master_ref_clk_hz = clk_get_rate(cqspi->clk);
	host->max_speed_hz = cqspi->master_ref_clk_hz;

	/* write completion is supported by default */
	cqspi->wr_completion = true;

	ddata  = of_device_get_match_data(dev);
	if (ddata) {
		if (ddata->quirks & CQSPI_NEEDS_WR_DELAY)
			cqspi->wr_delay = 50 * DIV_ROUND_UP(NSEC_PER_SEC,
						cqspi->master_ref_clk_hz);
		if (ddata->hwcaps_mask & CQSPI_SUPPORTS_OCTAL)
			host->mode_bits |= SPI_RX_OCTAL | SPI_TX_OCTAL;
		if (!(ddata->quirks & CQSPI_DISABLE_DAC_MODE)) {
			cqspi->use_direct_mode = true;
			cqspi->use_direct_mode_wr = true;
		}
		if (ddata->quirks & CQSPI_SUPPORT_EXTERNAL_DMA)
			cqspi->use_dma_read = true;
		if (ddata->quirks & CQSPI_NO_SUPPORT_WR_COMPLETION)
			cqspi->wr_completion = false;
		if (ddata->quirks & CQSPI_SLOW_SRAM)
			cqspi->slow_sram = true;
		if (ddata->quirks & CQSPI_NEEDS_APB_AHB_HAZARD_WAR)
			cqspi->apb_ahb_hazard = true;

		if (ddata->jh7110_clk_init) {
			ret = cqspi_jh7110_clk_init(pdev, cqspi);
			if (ret)
				goto probe_reset_failed;
		}

		if (of_device_is_compatible(pdev->dev.of_node,
					    "xlnx,versal-ospi-1.0")) {
			ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
			if (ret)
				goto probe_reset_failed;
		}
	}

	ret = devm_request_irq(dev, irq, cqspi_irq_handler, 0,
			       pdev->name, cqspi);
	if (ret) {
		dev_err(dev, "Cannot request IRQ.\n");
		goto probe_reset_failed;
	}

	cqspi_wait_idle(cqspi);
	cqspi_controller_init(cqspi);
	cqspi->current_cs = -1;
	cqspi->sclk = 0;

	host->num_chipselect = cqspi->num_chipselect;

	ret = cqspi_setup_flash(cqspi);
	if (ret) {
		dev_err(dev, "failed to setup flash parameters %d\n", ret);
		goto probe_setup_failed;
	}

	if (cqspi->use_direct_mode && !soc_device_match(k3_soc_devices)) {
		ret = cqspi_request_mmap_dma(cqspi);
		if (ret == -EPROBE_DEFER)
			goto probe_setup_failed;
	}

	ret = devm_pm_runtime_enable(dev);
	if (ret) {
		if (cqspi->rx_chan)
			dma_release_channel(cqspi->rx_chan);
		goto probe_setup_failed;
	}

	pm_runtime_set_autosuspend_delay(dev, CQSPI_AUTOSUSPEND_TIMEOUT);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_get_noresume(dev);

	ret = spi_register_controller(host);
	if (ret) {
		dev_err(&pdev->dev, "failed to register SPI ctlr %d\n", ret);
		goto probe_setup_failed;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;
probe_setup_failed:
	cqspi_controller_enable(cqspi, 0);
probe_reset_failed:
	if (cqspi->is_jh7110)
		cqspi_jh7110_disable_clk(pdev, cqspi);
	clk_disable_unprepare(cqspi->clk);
probe_clk_failed:
	return ret;
}

static void cqspi_remove(struct platform_device *pdev)
{
	struct cqspi_st *cqspi = platform_get_drvdata(pdev);

	spi_unregister_controller(cqspi->host);
	cqspi_controller_enable(cqspi, 0);

	if (cqspi->rx_chan)
		dma_release_channel(cqspi->rx_chan);

	clk_disable_unprepare(cqspi->clk);

	if (cqspi->is_jh7110)
		cqspi_jh7110_disable_clk(pdev, cqspi);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static void __maybe_unused cqspi_restore_context(struct cqspi_st *cqspi)
{
	struct device *dev = &cqspi->pdev->dev;
	int ret;

	if (!(cqspi->f_pdata->use_dqs))
		cqspi_phy_set_dll_master(cqspi);

	ret = cqspi_phy_apply_setting(cqspi->f_pdata,
				      &cqspi->f_pdata->phy_setting);

	if (ret)
		dev_err(dev, "Applying phy_setting failed during restore\n");
}

static int cqspi_runtime_suspend(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);

	cqspi_controller_enable(cqspi, 0);
	clk_disable_unprepare(cqspi->clk);
	return 0;
}

static int cqspi_runtime_resume(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);

	clk_prepare_enable(cqspi->clk);
	cqspi_wait_idle(cqspi);
	cqspi_controller_init(cqspi);

	/*
	 * Only restore context if PHY is enabled, or else skip this step
	 */
	if ((cqspi->f_pdata->use_phy) == true)
		cqspi_restore_context(cqspi);

	cqspi->current_cs = -1;
	cqspi->sclk = 0;
	return 0;
}

static int cqspi_suspend(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);
	int ret;

	ret = spi_controller_suspend(cqspi->host);
	if (ret)
		return ret;

	return pm_runtime_force_suspend(dev);
}

static int cqspi_resume(struct device *dev)
{
	struct cqspi_st *cqspi = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret) {
		dev_err(dev, "pm_runtime_force_resume failed on resume\n");
		return ret;
	}

	return spi_controller_resume(cqspi->host);
}

static const struct dev_pm_ops cqspi_dev_pm_ops = {
	RUNTIME_PM_OPS(cqspi_runtime_suspend, cqspi_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(cqspi_suspend, cqspi_resume)
};

static const struct cqspi_driver_platdata cdns_qspi = {
	.quirks = CQSPI_DISABLE_DAC_MODE,
};

static const struct cqspi_driver_platdata k2g_qspi = {
	.quirks = CQSPI_NEEDS_WR_DELAY,
};

static const struct cqspi_driver_platdata am654_ospi = {
	.hwcaps_mask = CQSPI_SUPPORTS_OCTAL,
	.quirks = CQSPI_NEEDS_WR_DELAY,
};

static const struct cqspi_driver_platdata intel_lgm_qspi = {
	.quirks = CQSPI_DISABLE_DAC_MODE,
};

static const struct cqspi_driver_platdata socfpga_qspi = {
	.quirks = CQSPI_DISABLE_DAC_MODE
			| CQSPI_NO_SUPPORT_WR_COMPLETION
			| CQSPI_SLOW_SRAM,
};

static const struct cqspi_driver_platdata versal_ospi = {
	.hwcaps_mask = CQSPI_SUPPORTS_OCTAL,
	.quirks = CQSPI_DISABLE_DAC_MODE | CQSPI_SUPPORT_EXTERNAL_DMA,
	.indirect_read_dma = cqspi_versal_indirect_read_dma,
	.get_dma_status = cqspi_get_versal_dma_status,
};

static const struct cqspi_driver_platdata jh7110_qspi = {
	.quirks = CQSPI_DISABLE_DAC_MODE,
	.jh7110_clk_init = cqspi_jh7110_clk_init,
};

static const struct cqspi_driver_platdata pensando_cdns_qspi = {
	.quirks = CQSPI_NEEDS_APB_AHB_HAZARD_WAR | CQSPI_DISABLE_DAC_MODE,
};

static const struct of_device_id cqspi_dt_ids[] = {
	{
		.compatible = "cdns,qspi-nor",
		.data = &cdns_qspi,
	},
	{
		.compatible = "ti,k2g-qspi",
		.data = &k2g_qspi,
	},
	{
		.compatible = "ti,am654-ospi",
		.data = &am654_ospi,
	},
	{
		.compatible = "intel,lgm-qspi",
		.data = &intel_lgm_qspi,
	},
	{
		.compatible = "xlnx,versal-ospi-1.0",
		.data = &versal_ospi,
	},
	{
		.compatible = "intel,socfpga-qspi",
		.data = &socfpga_qspi,
	},
	{
		.compatible = "starfive,jh7110-qspi",
		.data = &jh7110_qspi,
	},
	{
		.compatible = "amd,pensando-elba-qspi",
		.data = &pensando_cdns_qspi,
	},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, cqspi_dt_ids);

static struct platform_driver cqspi_platform_driver = {
	.probe = cqspi_probe,
	.remove_new = cqspi_remove,
	.driver = {
		.name = CQSPI_NAME,
		.pm = pm_ptr(&cqspi_dev_pm_ops),
		.of_match_table = cqspi_dt_ids,
	},
};

module_platform_driver(cqspi_platform_driver);

MODULE_DESCRIPTION("Cadence QSPI Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" CQSPI_NAME);
MODULE_AUTHOR("Ley Foon Tan <lftan@altera.com>");
MODULE_AUTHOR("Graham Moore <grmoore@opensource.altera.com>");
MODULE_AUTHOR("Vadivel Murugan R <vadivel.muruganx.ramuthevar@intel.com>");
MODULE_AUTHOR("Vignesh Raghavendra <vigneshr@ti.com>");
MODULE_AUTHOR("Pratyush Yadav <p.yadav@ti.com>");
