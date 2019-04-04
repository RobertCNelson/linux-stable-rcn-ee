/*
 * sun8i-emac driver
 *
 * Copyright (C) 2015-2016 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * This is the driver for Allwinner Ethernet MAC found in H3/A83T/A64 SoC
 *
 * TODO:
 * - MAC filtering
 * - Jumbo frame
 * - features rx-all (NETIF_F_RXALL_BIT)
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/skbuff.h>

#define SUN8I_EMAC_BASIC_CTL0	0x00
#define SUN8I_EMAC_BASIC_CTL1	0x04
#define SUN8I_EMAC_INT_STA	0x08
#define SUN8I_EMAC_INT_EN	0x0C
#define SUN8I_EMAC_TX_CTL0	0x10
#define SUN8I_EMAC_TX_CTL1	0x14
#define SUN8I_EMAC_TX_FLOW_CTL	0x1C
#define SUN8I_EMAC_RX_CTL0	0x24
#define SUN8I_EMAC_RX_CTL1	0x28
#define SUN8I_EMAC_RX_FRM_FLT	0x38
#define SUN8I_EMAC_MDIO_CMD	0x48
#define SUN8I_EMAC_MDIO_DATA	0x4C
#define SUN8I_EMAC_TX_DMA_STA	0xB0
#define SUN8I_EMAC_TX_CUR_DESC	0xB4
#define SUN8I_EMAC_TX_CUR_BUF	0xB8
#define SUN8I_EMAC_RX_DMA_STA	0xC0

#define MDIO_CMD_MII_BUSY	BIT(0)
#define MDIO_CMD_MII_WRITE	BIT(1)
#define MDIO_CMD_MII_PHY_REG_ADDR_MASK	GENMASK(8, 4)
#define MDIO_CMD_MII_PHY_REG_ADDR_SHIFT	4
#define MDIO_CMD_MII_PHY_ADDR_MASK	GENMASK(16, 12)
#define MDIO_CMD_MII_PHY_ADDR_SHIFT	12

#define SUN8I_EMAC_MACADDR_HI	0x50
#define SUN8I_EMAC_MACADDR_LO	0x54

#define SUN8I_EMAC_RX_DESC_LIST 0x34
#define SUN8I_EMAC_TX_DESC_LIST 0x20

#define SUN8I_EMAC_RX_DO_CRC BIT(27)
#define SUN8I_EMAC_RX_STRIP_FCS BIT(28)

#define SUN8I_COULD_BE_USED_BY_DMA BIT(31)

/* Used in RX_CTL1*/
#define RX_DMA_EN	BIT(30)
#define RX_DMA_START	BIT(31)
/* Used in TX_CTL1*/
#define TX_DMA_EN	BIT(30)
#define TX_DMA_START	BIT(31)

/* Used in RX_CTL0 */
#define RX_RECEIVER_EN		BIT(31)
/* Used in TX_CTL0 */
#define TX_TRANSMITTER_EN	BIT(31)

/* Basic CTL0 */
#define BCTL0_FD BIT(0)
#define BCTL0_SPEED_10		2
#define BCTL0_SPEED_100		3
#define BCTL0_SPEED_MASK	GENMASK(3, 2)
#define BCTL0_SPEED_SHIFT	2

#define FLOW_RX 1
#define FLOW_TX 2

#define RX_INT                  BIT(8)
#define TX_INT                  BIT(0)

/* Bits used in frame RX status */
#define DSC_RX_FIRST		BIT(9)
#define DSC_RX_LAST		BIT(8)

/* Bits used in frame TX ctl */
#define SUN8I_EMAC_MAGIC_TX_BIT	BIT(24)
#define SUN8I_EMAC_TX_DO_CRC	(BIT(27) | BIT(28))
#define DSC_TX_FIRST		BIT(29)
#define DSC_TX_LAST		BIT(30)
#define SUN8I_EMAC_WANT_INT	BIT(31)

enum emac_variant {
	NONE_EMAC,/* for be sure that variant is non-0 if set */
	A83T_EMAC,
	H3_EMAC,
	A64_EMAC,
};

static const char const estats_str[][ETH_GSTRING_LEN] = {
	/* errors */
	"rx_payload_error",
	"rx_CRC_error",
	"rx_phy_error",
	"rx_length_error",
	"rx_col_error",
	"rx_header_error",
	"rx_overflow_error",
	"rx_saf_error",
	"rx_daf_error",
	"rx_buf_error",
	/* misc infos */
	"tx_stop_queue",
	"rx_dma_ua",
	"rx_dma_stop",
	"tx_dma_ua",
	"tx_dma_stop",
	"rx_hw_csum",
	"tx_hw_csum",
	/* interrupts */
	"rx_int",
	"tx_int",
	"rx_early_int",
	"tx_early_int",
	"tx_underflow_int",
	/* debug */
	"tx_used_desc",
	"napi_schedule",
	"napi_underflow",
};

struct sun8i_emac_stats {
	u64 rx_payload_error;
	u64 rx_crc_error;
	u64 rx_phy_error;
	u64 rx_length_error;
	u64 rx_col_error;
	u64 rx_header_error;
	u64 rx_overflow_error;
	u64 rx_saf_fail;
	u64 rx_daf_fail;
	u64 rx_buf_error;

	u64 tx_stop_queue;
	u64 rx_dma_ua;
	u64 rx_dma_stop;
	u64 tx_dma_ua;
	u64 tx_dma_stop;
	u64 rx_hw_csum;
	u64 tx_hw_csum;

	u64 rx_int;
	u64 tx_int;
	u64 rx_early_int;
	u64 tx_early_int;
	u64 tx_underflow_int;

	u64 tx_used_desc;
	u64 napi_schedule;
	u64 napi_underflow;
};

/* The datasheet said that each descriptor can transfers up to 4096bytes
 * But latter, a register documentation reduce that value to 2048
 * Anyway using 2048 cause strange behaviours and even BSP driver use 2047
 */
#define DESC_BUF_MAX 2044
#if (DESC_BUF_MAX < (ETH_FRAME_LEN + 4))
#error "DESC_BUF_MAX must be set at minimum to ETH_FRAME_LEN + 4"
#endif

/* MAGIC value for knowing if a descriptor is available or not */
#define DCLEAN (BIT(16) | BIT(14) | BIT(12) | BIT(10) | BIT(9))

/* struct dma_desc - Structure of DMA descriptor used by the hardware
 * @status: Status of the frame written by HW, so RO for the
 *	driver (except for BIT(31) which is R/W)
 * @ctl: Information on the frame written by the driver (INT, len,...)
 * @buf_addr: physical address of the frame data
 * @next: physical address of next dma_desc
 */
struct dma_desc {
	u32 status;
	u32 ctl;
	u32 buf_addr;
	u32 next;
};

/* Describe how data from skb are DMA mapped (used in txinfo map member) */
#define MAP_SINGLE 1
#define MAP_PAGE 2

/* Structure for storing information about data in TX ring buffer */
struct txinfo {
	struct sk_buff *skb;
	int map;
};

struct sun8i_emac_priv {
	void __iomem *base;
	void __iomem *syscon;
	int irq;
	struct device *dev;
	struct net_device *ndev;
	struct mii_bus *mdio;
	struct napi_struct napi;
	spinlock_t tx_lock;/* control the access of transmit descriptors */
	int duplex;
	int speed;
	int link;
	int phy_interface;
	enum emac_variant variant;
	struct device_node *phy_node;
	struct clk *ahb_clk;
	struct clk *ephy_clk;
	bool use_internal_phy;

	struct reset_control *rst;
	struct reset_control *rst_ephy;

	struct dma_desc *dd_rx;
	dma_addr_t dd_rx_phy;
	struct dma_desc *dd_tx;
	dma_addr_t dd_tx_phy;
	struct sk_buff **rx_skb;
	struct txinfo *txl;

	int nbdesc_tx;
	int nbdesc_rx;
	int tx_slot;
	int tx_dirty;
	int rx_dirty;
	struct sun8i_emac_stats estats;
	u32 msg_enable;
	int flow_ctrl;
	int pause;
};

static irqreturn_t sun8i_emac_dma_interrupt(int irq, void *dev_id);

static void rb_inc(int *p, const int max)
{
	(*p)++;
	(*p) %= max;
}

/* Return the number of contiguous free descriptors
 * starting from tx_slot
 */
static int rb_tx_numfreedesc(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	if (priv->tx_slot < priv->tx_dirty)
		return priv->tx_dirty - priv->tx_slot;

	return (priv->nbdesc_tx - priv->tx_slot) + priv->tx_dirty;
}

/* Allocate a skb in a DMA descriptor
 *
 * @i index of slot to fill
*/
static int sun8i_emac_rx_skb(struct net_device *ndev, int i)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct dma_desc *ddesc;
	struct sk_buff *skb;

	ddesc = priv->dd_rx + i;

	ddesc->ctl = 0;

	skb = netdev_alloc_skb_ip_align(ndev, DESC_BUF_MAX);
	if (!skb)
		return -ENOMEM;

	/* should not happen */
	if (unlikely(priv->rx_skb[i]))
		dev_warn(priv->dev, "BUG: Leaking a skbuff\n");

	priv->rx_skb[i] = skb;

	ddesc->buf_addr = dma_map_single(priv->dev, skb->data,
					 DESC_BUF_MAX, DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->dev, ddesc->buf_addr)) {
		dev_err(priv->dev, "ERROR: Cannot map RX buffer for DMA\n");
		dev_kfree_skb(skb);
		return -EFAULT;
	}
	ddesc->ctl |= DESC_BUF_MAX;
	wmb();/* SUN8I_COULD_BE_USED_BY_DMA must be the last value written */
	ddesc->status = SUN8I_COULD_BE_USED_BY_DMA;

	return 0;
}

static void sun8i_emac_stop_tx(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v;

	netif_stop_queue(ndev);

	v = readl(priv->base + SUN8I_EMAC_TX_CTL0);
	v &= ~TX_TRANSMITTER_EN;/*Disable transmitter after current reception*/
	writel(v, priv->base + SUN8I_EMAC_TX_CTL0);
	v = readl(priv->base + SUN8I_EMAC_TX_CTL1);
	v &= ~TX_DMA_EN; /* Stop TX DMA */
	writel(v, priv->base + SUN8I_EMAC_TX_CTL1);
}

static void sun8i_emac_stop_rx(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v;

	v = readl(priv->base + SUN8I_EMAC_RX_CTL0);
	v &= ~RX_RECEIVER_EN; /* Disable receiver after current reception */
	writel(v, priv->base + SUN8I_EMAC_RX_CTL0);
	v = readl(priv->base + SUN8I_EMAC_RX_CTL1);
	v &= ~RX_DMA_EN; /* Stop RX DMA */
	writel(v, priv->base + SUN8I_EMAC_RX_CTL1);
}

static void sun8i_emac_start_rx(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v;

	v = readl(priv->base + SUN8I_EMAC_RX_CTL0);
	v |= RX_RECEIVER_EN;/* Enable receiver */
	writel(v, priv->base + SUN8I_EMAC_RX_CTL0);

	v = readl(priv->base + SUN8I_EMAC_RX_CTL1);
	v |= RX_DMA_START;
	v |= RX_DMA_EN;
	writel(v, priv->base + SUN8I_EMAC_RX_CTL1);
}

static void sun8i_emac_start_tx(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v;

	v = readl(priv->base + SUN8I_EMAC_TX_CTL0);
	v |= TX_TRANSMITTER_EN;
	writel(v, priv->base + SUN8I_EMAC_TX_CTL0);

	v = readl(priv->base + SUN8I_EMAC_TX_CTL1);
	v |= TX_DMA_START;
	v |= TX_DMA_EN;
	writel(v, priv->base + SUN8I_EMAC_TX_CTL1);
}

/* Set MAC address for slot index
 * @addr: the MAC address to set
 * @index: The index of slot where to set address.
 * The slot 0 is the main MACaddr
 */
static void sun8i_emac_set_macaddr(struct sun8i_emac_priv *priv,
				   const u8 *addr, int index)
{
	u32 v;

	dev_info(priv->dev, "device MAC address slot %d %02x:%02x:%02x:%02x:%02x:%02x\n",
		 index, addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	v = (addr[5] << 8) | addr[4];
	writel(v, priv->base + SUN8I_EMAC_MACADDR_HI + index * 8);
	v = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	writel(v, priv->base + SUN8I_EMAC_MACADDR_LO + index * 8);
}

static void sun8i_emac_set_link_mode(struct sun8i_emac_priv *priv)
{
	u32 v;

	v = readl(priv->base + SUN8I_EMAC_BASIC_CTL0);

	if (priv->duplex)
		v |= BCTL0_FD;
	else
		v &= ~BCTL0_FD;

	v &= ~BCTL0_SPEED_MASK;
	switch (priv->speed) {
	case 1000:
		break;
	case 100:
		v |= BCTL0_SPEED_100 << BCTL0_SPEED_SHIFT;
		break;
	case 10:
		v |= BCTL0_SPEED_10 << BCTL0_SPEED_SHIFT;
		break;
	}

	writel(v, priv->base + SUN8I_EMAC_BASIC_CTL0);
}

static void sun8i_emac_flow_ctrl(struct sun8i_emac_priv *priv, int duplex,
				 int fc)
{
	u32 flow = 0;

	netif_dbg(priv, link, priv->ndev, "%s %d %d\n", __func__,
		  duplex, fc);

	flow = readl(priv->base + SUN8I_EMAC_RX_CTL0);
	if (fc & FLOW_RX)
		flow |= BIT(16);
	else
		flow &= ~BIT(16);
	writel(flow, priv->base + SUN8I_EMAC_RX_CTL0);

	flow = readl(priv->base + SUN8I_EMAC_TX_FLOW_CTL);
	if (fc & FLOW_TX)
		flow |= BIT(0);
	else
		flow &= ~BIT(0);
	writel(flow, priv->base + SUN8I_EMAC_TX_FLOW_CTL);
}

/* Grab a frame into a skb from descriptor number i */
static int sun8i_emac_rx_from_ddesc(struct net_device *ndev, int i)
{
	struct sk_buff *skb;
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct dma_desc *ddesc = priv->dd_rx + i;
	int frame_len;
	int rxcsum_done = 0;

	if (ndev->features & NETIF_F_RXCSUM)
		rxcsum_done = 1;

	/* bit0/bit7 work only on IPv4/IPv6 TCP traffic,
	 * (not on ARP for example) so we dont raise rx_errors/discard frame
	 */
	/* the checksum or length of received frame's payload is wrong*/
	if (ddesc->status & BIT(0)) {
		priv->estats.rx_payload_error++;
		rxcsum_done = 0;
	}
	/* RX_CRC_ERR */
	if (ddesc->status & BIT(1)) {
		priv->ndev->stats.rx_errors++;
		priv->ndev->stats.rx_crc_errors++;
		priv->estats.rx_crc_error++;
		goto discard_frame;
	}
	/* RX_PHY_ERR */
	if ((ddesc->status & BIT(3))) {
		priv->ndev->stats.rx_errors++;
		priv->estats.rx_phy_error++;
		goto discard_frame;
	}
	/* RX_LENGTH_ERR */
	if ((ddesc->status & BIT(4))) {
		priv->ndev->stats.rx_errors++;
		priv->ndev->stats.rx_length_errors++;
		priv->estats.rx_length_error++;
		goto discard_frame;
	}
	/* RX_COL_ERR */
	if ((ddesc->status & BIT(6))) {
		priv->ndev->stats.rx_errors++;
		priv->estats.rx_col_error++;
		goto discard_frame;
	}
	/* RX_HEADER_ERR */
	if ((ddesc->status & BIT(7))) {
		priv->estats.rx_header_error++;
		rxcsum_done = 0;
	}
	/* RX_OVERFLOW_ERR */
	if ((ddesc->status & BIT(11))) {
		priv->ndev->stats.rx_over_errors++;
		priv->estats.rx_overflow_error++;
		goto discard_frame;
	}
	/* RX_NO_ENOUGTH_BUF_ERR */
	if ((ddesc->status & BIT(14))) {
		priv->ndev->stats.rx_errors++;
		priv->estats.rx_buf_error++;
		goto discard_frame;
	}

	/* BIT(9) is for the first frame, not having it is bad since we do not
	 * handle Jumbo frame
	 */
	if ((ddesc->status & DSC_RX_FIRST) == 0) {
		dev_warn_ratelimited(priv->dev, "BUG: Non-first frame received. This should not happen\n");
		goto discard_frame;
	}
	frame_len = (ddesc->status >> 16) & 0x3FFF;
	if (!(ndev->features & NETIF_F_RXFCS))
		frame_len -= ETH_FCS_LEN;

	skb = priv->rx_skb[i];

	netif_dbg(priv, rx_status, priv->ndev,
		  "%s from %02d %pad len=%d status=%x st=%x\n",
		  __func__, i, &ddesc, frame_len, ddesc->status, ddesc->ctl);

	skb_put(skb, frame_len);

	dma_unmap_single(priv->dev, ddesc->buf_addr, DESC_BUF_MAX,
			 DMA_FROM_DEVICE);
	skb->protocol = eth_type_trans(skb, priv->ndev);
	if (rxcsum_done) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		priv->estats.rx_hw_csum++;
	} else {
		skb->ip_summed = CHECKSUM_PARTIAL;
	}

	priv->ndev->stats.rx_packets++;
	priv->ndev->stats.rx_bytes += frame_len;
	priv->rx_skb[i] = NULL;

	/* this frame is not the last */
	if ((ddesc->status & DSC_RX_LAST) == 0) {
		dev_warn(priv->dev, "Multi frame not implemented currlen=%d\n",
			 frame_len);
	}

	sun8i_emac_rx_skb(ndev, i);
	napi_gro_receive(&priv->napi, skb);

	return 0;
	/* If the frame need to be dropped, we simply reuse the buffer */
discard_frame:
	ddesc->ctl = DESC_BUF_MAX;
	wmb();/* SUN8I_COULD_BE_USED_BY_DMA must be the last value written */
	ddesc->status = SUN8I_COULD_BE_USED_BY_DMA;
	return 0;
}

/* iterate over dma_desc for finding completed xmit.
 * Called from interrupt context, so no need to spinlock tx
 *
 * The problem is: how to know that a descriptor is sent and not just in
 * preparation.
 * Need to have status=0 and st set but this is the state of first frame just
 * before setting the own-by-DMA bit.
 * The solution is to used the artificial value DCLEAN.
 */
static int sun8i_emac_complete_xmit(struct net_device *ndev, int budget)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct dma_desc *ddesc;
	int frame_len;
	int work = 0;

	spin_lock(&priv->tx_lock);
	do {
		ddesc = priv->dd_tx + priv->tx_dirty;

		if (ddesc->status & SUN8I_COULD_BE_USED_BY_DMA)
			goto xmit_end;

		if (ddesc->status == DCLEAN)
			goto xmit_end;

		if (ddesc->status == 0 && !ddesc->ctl) {
			dev_err(priv->dev, "BUG: reached the void %d %d\n",
				priv->tx_dirty, priv->tx_slot);
			goto xmit_end;
		}

		/* TX_UNDERFLOW_ERR */
		if (ddesc->status & BIT(1))
			priv->ndev->stats.tx_errors++;
		/* TX_DEFER_ERR */
		if (ddesc->status & BIT(2))
			priv->ndev->stats.tx_errors++;
		/* BIT 6:3 numbers of collisions */
		if (ddesc->status & 0x78)
			priv->ndev->stats.collisions +=
				(ddesc->status & 0x78) >> 3;
		/* TX_COL_ERR_1 */
		if (ddesc->status & BIT(8))
			priv->ndev->stats.tx_errors++;
		/* TX_COL_ERR_0 */
		if (ddesc->status & BIT(9))
			priv->ndev->stats.tx_errors++;
		/* TX_CRS_ERR */
		if (ddesc->status & BIT(10))
			priv->ndev->stats.tx_carrier_errors++;
		/* TX_PAYLOAD_ERR */
		if (ddesc->status & BIT(12))
			priv->ndev->stats.tx_errors++;
		/* TX_LENGTH_ERR */
		if (ddesc->status & BIT(14))
			priv->ndev->stats.tx_errors++;
		/* TX_HEADER_ERR */
		if (ddesc->status & BIT(16))
			priv->ndev->stats.tx_errors++;
		frame_len = ddesc->ctl & 0x3FFF;
		if (priv->txl[priv->tx_dirty].map == MAP_SINGLE)
			dma_unmap_single(priv->dev, ddesc->buf_addr,
					 frame_len, DMA_TO_DEVICE);
		else
			dma_unmap_page(priv->dev, ddesc->buf_addr,
				       frame_len, DMA_TO_DEVICE);
		/* we can free skb only on last frame */
		if (priv->txl[priv->tx_dirty].skb && (ddesc->ctl & DSC_TX_LAST))
			dev_kfree_skb_irq(priv->txl[priv->tx_dirty].skb);

		priv->txl[priv->tx_dirty].skb = NULL;
		priv->txl[priv->tx_dirty].map = 0;
		ddesc->ctl = 0;
		wmb(); /* setting to DCLEAN is the last value to be set */
		ddesc->status = DCLEAN;
		work++;

		rb_inc(&priv->tx_dirty, priv->nbdesc_tx);
		ddesc = priv->dd_tx + priv->tx_dirty;
	} while (ddesc->ctl && !(ddesc->status & SUN8I_COULD_BE_USED_BY_DMA));

	if (netif_queue_stopped(ndev) &&
	    rb_tx_numfreedesc(ndev) > MAX_SKB_FRAGS + 1)
		netif_wake_queue(ndev);
xmit_end:
	spin_unlock(&priv->tx_lock);
	return work;
}

static int sun8i_emac_poll(struct napi_struct *napi, int budget)
{
	struct sun8i_emac_priv *priv =
		container_of(napi, struct sun8i_emac_priv, napi);
	struct net_device *ndev = priv->ndev;
	int worked;
	struct dma_desc *ddesc;

	priv->estats.napi_schedule++;
	worked = sun8i_emac_complete_xmit(ndev, budget);

	ddesc = priv->dd_rx + priv->rx_dirty;
	while (!(ddesc->status & SUN8I_COULD_BE_USED_BY_DMA) &&
	       worked < budget) {
		sun8i_emac_rx_from_ddesc(ndev, priv->rx_dirty);
		worked++;
		rb_inc(&priv->rx_dirty, priv->nbdesc_rx);
		ddesc = priv->dd_rx + priv->rx_dirty;
	};
	if (worked < budget) {
		priv->estats.napi_underflow++;
		napi_complete(&priv->napi);
		writel(RX_INT | TX_INT, priv->base + SUN8I_EMAC_INT_EN);
	}
	return worked;
}

static int sun8i_mdio_read(struct mii_bus *bus, int phy_addr, int phy_reg)
{
	struct net_device *ndev = bus->priv;
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int err;
	u32 reg;

	err = readl_poll_timeout(priv->base + SUN8I_EMAC_MDIO_CMD, reg,
				 !(reg & MDIO_CMD_MII_BUSY), 100, 10000);
	if (err) {
		dev_err(priv->dev, "%s timeout %x\n", __func__, reg);
		return err;
	}

	reg &= ~MDIO_CMD_MII_WRITE;
	reg &= ~MDIO_CMD_MII_PHY_REG_ADDR_MASK;
	reg |= (phy_reg << MDIO_CMD_MII_PHY_REG_ADDR_SHIFT) &
		MDIO_CMD_MII_PHY_REG_ADDR_MASK;

	reg &= ~MDIO_CMD_MII_PHY_ADDR_MASK;

	reg |= (phy_addr << MDIO_CMD_MII_PHY_ADDR_SHIFT) &
		MDIO_CMD_MII_PHY_ADDR_MASK;

	reg |= MDIO_CMD_MII_BUSY;

	writel(reg, priv->base + SUN8I_EMAC_MDIO_CMD);

	err = readl_poll_timeout(priv->base + SUN8I_EMAC_MDIO_CMD, reg,
				 !(reg & MDIO_CMD_MII_BUSY), 100, 10000);

	if (err) {
		dev_err(priv->dev, "%s timeout %x\n", __func__, reg);
		return err;
	}

	return readl(priv->base + SUN8I_EMAC_MDIO_DATA);
}

static int sun8i_mdio_write(struct mii_bus *bus, int phy_addr, int phy_reg,
			    u16 data)
{
	struct net_device *ndev = bus->priv;
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 reg;
	int err;

	err = readl_poll_timeout(priv->base + SUN8I_EMAC_MDIO_CMD, reg,
				 !(reg & MDIO_CMD_MII_BUSY), 100, 10000);
	if (err) {
		dev_err(priv->dev, "%s timeout %x\n", __func__, reg);
		return err;
	}

	reg &= ~MDIO_CMD_MII_PHY_REG_ADDR_MASK;
	reg |= (phy_reg << MDIO_CMD_MII_PHY_REG_ADDR_SHIFT) &
		MDIO_CMD_MII_PHY_REG_ADDR_MASK;

	reg &= ~MDIO_CMD_MII_PHY_ADDR_MASK;
	reg |= (phy_addr << MDIO_CMD_MII_PHY_ADDR_SHIFT) &
		MDIO_CMD_MII_PHY_ADDR_MASK;

	reg |= MDIO_CMD_MII_WRITE;
	reg |= MDIO_CMD_MII_BUSY;

	writel(reg, priv->base + SUN8I_EMAC_MDIO_CMD);
	writel(data, priv->base + SUN8I_EMAC_MDIO_DATA);
	dev_dbg(priv->dev, "%s %d %d %x %x\n", __func__, phy_addr, phy_reg,
		reg, data);

	err = readl_poll_timeout(priv->base + SUN8I_EMAC_MDIO_CMD, reg,
				 !(reg & MDIO_CMD_MII_BUSY), 100, 10000);
	if (err) {
		dev_err(priv->dev, "%s timeout %x\n", __func__, reg);
		return err;
	}

	return 0;
}

static int sun8i_emac_mdio_register(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct mii_bus *bus;
	int ret;

	bus = mdiobus_alloc();
	if (!bus) {
		netdev_err(ndev, "Failed to allocate a new mdio bus\n");
		return -ENOMEM;
	}

	bus->name = dev_name(priv->dev);
	bus->read = &sun8i_mdio_read;
	bus->write = &sun8i_mdio_write;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%x", bus->name, priv->dev->id);

	bus->parent = priv->dev;
	bus->priv = ndev;

	ret = of_mdiobus_register(bus, priv->dev->of_node);
	if (ret) {
		netdev_err(ndev, "Could not register a MDIO bus: %d\n", ret);
		mdiobus_free(bus);
		return ret;
	}

	priv->mdio = bus;

	return 0;
}

static void sun8i_emac_mdio_unregister(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	mdiobus_unregister(priv->mdio);
	mdiobus_free(priv->mdio);
}

/* Run within phydev->lock */
static void sun8i_emac_adjust_link(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;
	int new_state = 0;

	netif_dbg(priv, link, priv->ndev,
		  "%s link=%x duplex=%x speed=%x\n", __func__,
		  phydev->link, phydev->duplex, phydev->speed);
	if (!phydev)
		return;

	if (phydev->link) {
		if (phydev->duplex != priv->duplex) {
			new_state = 1;
			priv->duplex = phydev->duplex;
		}
		if (phydev->pause)
			sun8i_emac_flow_ctrl(priv, phydev->duplex,
					     priv->flow_ctrl);

		if (phydev->speed != priv->speed) {
			new_state = 1;
			priv->speed = phydev->speed;
		}

		if (priv->link == 0) {
			new_state = 1;
			priv->link = phydev->link;
		}

		netif_dbg(priv, link, priv->ndev,
			  "%s new=%d link=%d pause=%d\n",
			  __func__, new_state, priv->link, phydev->pause);
		if (new_state)
			sun8i_emac_set_link_mode(priv);
	} else if (priv->link != phydev->link) {
		new_state = 1;
		priv->link = 0;
		priv->speed = 0;
		priv->duplex = -1;
	}

	if (new_state)
		phy_print_status(phydev);
}

/* H3 specific bits for EPHY */
#define H3_EPHY_ADDR_SHIFT	20
#define H3_EPHY_LED_POL		BIT(17) /* 1: active low, 0: active high */
#define H3_EPHY_SHUTDOWN	BIT(16) /* 1: shutdown, 0: power up */
#define H3_EPHY_SELECT		BIT(15) /* 1: internal PHY, 0: external PHY */
#define H3_EPHY_DEFAULT_VALUE	0x58000
#define H3_EPHY_DEFAULT_MASK	GENMASK(31, 15)

/* H3/A64 specific bits */
#define SC_RMII_EN		BIT(13) /* 1: enable RMII (overrides EPIT) */

/* Generic system control EMAC_CLK bits */
#define SC_ETXDC_MASK		GENMASK(2, 0)
#define SC_ETXDC_SHIFT		10
#define SC_ERXDC_MASK		GENMASK(4, 0)
#define SC_ERXDC_SHIFT		5
#define SC_EPIT			BIT(2) /* 1: RGMII, 0: MII */
#define SC_ETCS_MASK		GENMASK(1, 0)
#define SC_ETCS_MII		0x0
#define SC_ETCS_EXT_GMII	0x1
#define SC_ETCS_INT_GMII	0x2

static int sun8i_emac_set_syscon_ephy(struct net_device *ndev, u32 *reg)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct device_node *node = priv->dev->of_node;
	int ret;

	*reg &= ~H3_EPHY_DEFAULT_MASK;
	*reg |= H3_EPHY_DEFAULT_VALUE;

	if (!priv->use_internal_phy) {
		/* switch to external PHY interface */
		*reg &= ~H3_EPHY_SELECT;
		return 0;
	}

	if (priv->phy_interface != PHY_INTERFACE_MODE_MII) {
		netdev_warn(ndev,
			    "Internal PHY requested, forcing MII mode.\n");
		priv->phy_interface = PHY_INTERFACE_MODE_MII;
	}

	*reg |= H3_EPHY_SELECT;
	*reg &= ~H3_EPHY_SHUTDOWN;

	if (of_property_read_bool(node, "allwinner,leds-active-low"))
		*reg |= H3_EPHY_LED_POL;

	ret = of_mdio_parse_addr(priv->dev, priv->phy_node);
	if (ret < 0) {
		netdev_err(ndev, "Could not parse MDIO addr\n");
		return ret;
	}

	/* of_mdio_parse_addr returns a valid (0 ~ 31) PHY
	 * address. No need to mask it again.
	 */
	*reg |= ret << H3_EPHY_ADDR_SHIFT;

	return 0;
}

static int sun8i_emac_set_syscon(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct device_node *node = priv->dev->of_node;
	int ret;
	u32 reg, val;

	reg = readl(priv->syscon);

	if (priv->variant == H3_EMAC) {
		ret = sun8i_emac_set_syscon_ephy(ndev, &reg);
		if (ret)
			return ret;
	}

	if (!of_property_read_u32(node, "allwinner,tx-delay", &val)) {
		if (val <= SC_ETXDC_MASK) {
			reg &= ~(SC_ETXDC_MASK << SC_ETXDC_SHIFT);
			reg |= (val << SC_ETXDC_SHIFT);
		} else {
			netdev_warn(ndev, "Invalid TX clock delay: %d\n", val);
		}
	}

	if (!of_property_read_u32(node, "allwinner,rx-delay", &val)) {
		if (val <= SC_ERXDC_MASK) {
			reg &= ~(SC_ERXDC_MASK << SC_ERXDC_SHIFT);
			reg |= (val << SC_ERXDC_SHIFT);
		} else {
			netdev_warn(ndev, "Invalid RX clock delay: %d\n", val);
		}
	}

	/* Clear interface mode bits */
	reg &= ~(SC_ETCS_MASK | SC_EPIT);
	if (priv->variant == H3_EMAC || priv->variant == A64_EMAC)
		reg &= ~SC_RMII_EN;

	switch (priv->phy_interface) {
	case PHY_INTERFACE_MODE_MII:
		/* default */
		break;
	case PHY_INTERFACE_MODE_RGMII:
		reg |= SC_EPIT | SC_ETCS_INT_GMII;
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (priv->variant == H3_EMAC || priv->variant == A64_EMAC) {
			reg |= SC_RMII_EN | SC_ETCS_EXT_GMII;
			break;
		}
		/* RMII not supported on A83T */
	default:
		netdev_err(ndev, "Unsupported interface mode: %s",
			   phy_modes(priv->phy_interface));
		return -EINVAL;
	}

	writel(reg, priv->syscon);

	return 0;
}

static void sun8i_emac_unset_syscon(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 reg = 0;

	if (priv->variant == H3_EMAC)
		reg = H3_EPHY_DEFAULT_VALUE;

	writel(reg, priv->syscon);
}

/* Set Management Data Clock, must be call after device reset */
static void sun8i_emac_set_mdc(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	unsigned long rate;
	u32 reg;

	rate = clk_get_rate(priv->ahb_clk);
	if (rate > 160000000)
		reg = 0x3 << 20; /* AHB / 128 */
	else if (rate > 80000000)
		reg = 0x2 << 20; /* AHB / 64 */
	else if (rate > 40000000)
		reg = 0x1 << 20; /* AHB / 32 */
	else
		reg = 0x0 << 20; /* AHB / 16 */
	netif_dbg(priv, link, ndev, "MDC auto : %x\n", reg);
	writel(reg, priv->base + SUN8I_EMAC_MDIO_CMD);
}

/* "power" the device, by enabling clk/reset/regulators */
static int sun8i_emac_power(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int ret;

	ret = clk_prepare_enable(priv->ahb_clk);
	if (ret) {
		netdev_err(ndev, "Could not enable AHB clock\n");
		return ret;
	}

	if (priv->rst) {
		ret = reset_control_deassert(priv->rst);
		if (ret) {
			netdev_err(ndev, "Could not deassert reset\n");
			goto err_reset;
		}
	}

	if (priv->ephy_clk) {
		ret = clk_prepare_enable(priv->ephy_clk);
		if (ret) {
			netdev_err(ndev, "Could not enable EPHY clock\n");
			goto err_ephy_clk;
		}
	}

	if (priv->rst_ephy) {
		ret = reset_control_deassert(priv->rst_ephy);
		if (ret) {
			netdev_err(ndev, "Could not deassert EPHY reset\n");
			goto err_ephy_reset;
		}
	}

	return 0;

err_ephy_reset:
	if (priv->ephy_clk)
		clk_disable_unprepare(priv->ephy_clk);
err_ephy_clk:
	if (priv->rst)
		reset_control_assert(priv->rst);
err_reset:
	clk_disable_unprepare(priv->ahb_clk);
	return ret;
}

/* "Unpower" the device, disabling clocks and regulators, asserting reset */
static void sun8i_emac_unpower(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	if (priv->rst_ephy)
		reset_control_assert(priv->rst_ephy);

	if (priv->ephy_clk)
		clk_disable_unprepare(priv->ephy_clk);

	if (priv->rst)
		reset_control_assert(priv->rst);

	clk_disable_unprepare(priv->ahb_clk);
}

static int sun8i_emac_init(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct device_node *node = priv->dev->of_node;
	const u8 *addr;

	/* Try to get MAC address from DT, or assign a random one */
	addr = of_get_mac_address(node);
	if (addr)
		ether_addr_copy(ndev->dev_addr, addr);
	else
		eth_hw_addr_random(ndev);

	priv->phy_interface = of_get_phy_mode(node);
	if (priv->phy_interface < 0) {
		netdev_err(ndev, "PHY interface mode node unspecified\n");
		return priv->phy_interface;
	}

	return sun8i_emac_power(ndev);
}

static void sun8i_emac_uninit(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	mdiobus_unregister(priv->mdio);

	sun8i_emac_unpower(ndev);
}

static int sun8i_emac_mdio_probe(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct phy_device *phydev = NULL;

	phydev = of_phy_connect(ndev, priv->phy_node, &sun8i_emac_adjust_link,
				0, priv->phy_interface);

	if (!phydev) {
		netdev_err(ndev, "Could not attach to PHY\n");
		return -ENODEV;
	}

	phy_attached_info(phydev);

	/* mask with MAC supported features */
	phydev->supported &= PHY_GBIT_FEATURES;
	phydev->advertising = phydev->supported;

	priv->link = 0;
	priv->speed = 0;
	priv->duplex = -1;

	return 0;
}

/* Allocate both RX and TX ring buffer and init them
 * This function also write the startbase of thoses ring in the device.
 * All structures that help managing thoses rings are also handled
 * by this functions (rx_skb/txl)
 */
static int sun8i_emac_alloc_rings(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct dma_desc *ddesc;
	int err, i;

	priv->rx_skb = kcalloc(priv->nbdesc_rx, sizeof(struct sk_buff *),
			      GFP_KERNEL);
	if (!priv->rx_skb) {
		err = -ENOMEM;
		goto rx_skb_error;
	}
	priv->txl = kcalloc(priv->nbdesc_tx, sizeof(struct txinfo), GFP_KERNEL);
	if (!priv->txl) {
		err = -ENOMEM;
		goto tx_error;
	}

	/* allocate/init RX ring */
	priv->dd_rx = dma_zalloc_coherent(priv->dev,
			priv->nbdesc_rx * sizeof(struct dma_desc),
			&priv->dd_rx_phy, GFP_KERNEL);
	if (!priv->dd_rx) {
		dev_err(priv->dev, "ERROR: cannot allocate DMA RX buffer");
		err = -ENOMEM;
		goto dma_rx_error;
	}
	ddesc = priv->dd_rx;
	for (i = 0; i < priv->nbdesc_rx; i++) {
		sun8i_emac_rx_skb(ndev, i);
		ddesc->next = (u32)priv->dd_rx_phy + (i + 1)
			* sizeof(struct dma_desc);
		ddesc++;
	}
	/* last descriptor point back to first one */
	ddesc--;
	ddesc->next = (u32)priv->dd_rx_phy;

	/* allocate/init TX ring */
	priv->dd_tx = dma_zalloc_coherent(priv->dev,
			priv->nbdesc_tx * sizeof(struct dma_desc),
			&priv->dd_tx_phy, GFP_KERNEL);
	if (!priv->dd_tx) {
		dev_err(priv->dev, "ERROR: cannot allocate DMA TX buffer");
		err = -ENOMEM;
		goto dma_tx_error;
	}
	ddesc = priv->dd_tx;
	for (i = 0; i < priv->nbdesc_tx; i++) {
		ddesc->status = DCLEAN;
		ddesc->ctl = 0;
		ddesc->next = (u32)(priv->dd_tx_phy + (i + 1)
			* sizeof(struct dma_desc));
		ddesc++;
	}
	/* last descriptor point back to first one */
	ddesc--;
	ddesc->next = (u32)priv->dd_tx_phy;
	i--;

	priv->tx_slot = 0;
	priv->tx_dirty = 0;
	priv->rx_dirty = 0;

	/* write start of RX ring descriptor */
	writel(priv->dd_rx_phy, priv->base + SUN8I_EMAC_RX_DESC_LIST);
	/* write start of TX ring descriptor */
	writel(priv->dd_tx_phy, priv->base + SUN8I_EMAC_TX_DESC_LIST);

	return 0;
dma_tx_error:
	dma_free_coherent(priv->dev, priv->nbdesc_rx * sizeof(struct dma_desc),
			  priv->dd_rx, priv->dd_rx_phy);
dma_rx_error:
	kfree(priv->txl);
tx_error:
	kfree(priv->rx_skb);
rx_skb_error:
	return err;
}

static int sun8i_emac_open(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int err;
	u32 v;

	err = request_irq(priv->irq, sun8i_emac_dma_interrupt, 0,
			  dev_name(priv->dev), ndev);
	if (err) {
		dev_err(priv->dev, "Cannot request IRQ: %d\n", err);
		return err;
	}

	/* Set interface mode (and configure internal PHY on H3) */
	err = sun8i_emac_set_syscon(ndev);
	if (err)
		goto err_irq;

	/* Do SOFT RST */
	v = readl(priv->base + SUN8I_EMAC_BASIC_CTL1);
	writel(v | 0x01, priv->base + SUN8I_EMAC_BASIC_CTL1);

	err = readl_poll_timeout(priv->base + SUN8I_EMAC_BASIC_CTL1, v,
				 !(v & 0x01), 100, 10000);
	if (err) {
		dev_err(priv->dev, "EMAC reset timeout\n");
		err = -EFAULT;
		goto err_syscon;
	}

	sun8i_emac_set_mdc(ndev);

	err = sun8i_emac_mdio_register(ndev);
	if (err)
		goto err_syscon;

	err = sun8i_emac_mdio_probe(ndev);
	if (err)
		goto err_syscon;

	/* DMA */
	v = (8 << 24);/* burst len */
	writel(v, priv->base + SUN8I_EMAC_BASIC_CTL1);

	writel(RX_INT | TX_INT, priv->base + SUN8I_EMAC_INT_EN);

	v = readl(priv->base + SUN8I_EMAC_RX_CTL0);
	/* CHECK_CRC */
	if (ndev->features & NETIF_F_RXCSUM)
		v |= SUN8I_EMAC_RX_DO_CRC;
	else
		v &= ~SUN8I_EMAC_RX_DO_CRC;
	/* STRIP_FCS */
	if (ndev->features & NETIF_F_RXFCS)
		v &= ~SUN8I_EMAC_RX_STRIP_FCS;
	else
		v |= SUN8I_EMAC_RX_STRIP_FCS;
	writel(v, priv->base + SUN8I_EMAC_RX_CTL0);

	v = readl(priv->base + SUN8I_EMAC_TX_CTL1);
	/* TX_MD Transmission starts after a full frame located in TX DMA FIFO*/
	v |= BIT(1);
	/* Undocumented bit (called TX_NEXT_FRM in BSP), the original comment is
	 * "Operating on second frame increase the performance
	 * especially when transmit store-and-forward is used."
	 */
	v |= BIT(2);
	writel(v, priv->base + SUN8I_EMAC_TX_CTL1);

	v = readl(priv->base + SUN8I_EMAC_RX_CTL1);
	/* RX_MD RX DMA reads data from RX DMA FIFO to host memory after a
	 * complete frame has been written to RX DMA FIFO
	*/
	v |= BIT(1);
	writel(v, priv->base + SUN8I_EMAC_RX_CTL1);

	sun8i_emac_set_macaddr(priv, ndev->dev_addr, 0);

	err = sun8i_emac_alloc_rings(ndev);
	if (err) {
		netdev_err(ndev, "Fail to allocate rings\n");
		goto err_mdio;
	}

	phy_start(ndev->phydev);

	sun8i_emac_start_rx(ndev);
	sun8i_emac_start_tx(ndev);

	netif_napi_add(ndev, &priv->napi, sun8i_emac_poll, 64);
	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	return 0;
err_mdio:
	phy_disconnect(ndev->phydev);
err_syscon:
	sun8i_emac_unset_syscon(ndev);
err_irq:
	free_irq(priv->irq, ndev);
	return err;
}

/* Clean the TX ring of any accepted skb for xmit */
static void sun8i_emac_tx_clean(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int i;
	struct dma_desc *ddesc;
	int frame_len;

	spin_lock(&priv->tx_lock);

	for (i = 0; i < priv->nbdesc_tx; i++) {
		if (priv->txl[i].skb) {
			ddesc = priv->dd_tx + i;
			frame_len = ddesc->ctl & 0x3FFF;
			switch (priv->txl[i].map) {
			case MAP_SINGLE:
				dma_unmap_single(priv->dev, ddesc->buf_addr,
						 frame_len, DMA_TO_DEVICE);
				break;
			case MAP_PAGE:
				dma_unmap_page(priv->dev, ddesc->buf_addr,
					       frame_len, DMA_TO_DEVICE);
				break;
			default:
				dev_err(priv->dev, "Trying to free an empty slot\n");
				continue;
			}
			dev_kfree_skb_any(priv->txl[i].skb);
			priv->txl[i].skb = NULL;
			ddesc->ctl = 0;
			ddesc->status = DCLEAN;
		}
	}
	priv->tx_slot = 0;
	priv->tx_dirty = 0;

	spin_unlock(&priv->tx_lock);
}

/* Clean the RX ring */
static void sun8i_emac_rx_clean(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int i;
	struct dma_desc *ddesc;

	/* clean RX ring */
	for (i = 0; i < priv->nbdesc_rx; i++)
		if (priv->rx_skb[i]) {
			ddesc = priv->dd_rx + i;
			dma_unmap_single(priv->dev, ddesc->buf_addr,
					 DESC_BUF_MAX, DMA_FROM_DEVICE);
			dev_kfree_skb_any(priv->rx_skb[i]);
			priv->rx_skb[i] = NULL;
		}
}

static int sun8i_emac_stop(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	napi_disable(&priv->napi);

	sun8i_emac_stop_tx(ndev);
	sun8i_emac_stop_rx(ndev);

	phy_stop(ndev->phydev);
	phy_disconnect(ndev->phydev);

	sun8i_emac_mdio_unregister(ndev);

	sun8i_emac_unset_syscon(ndev);

	free_irq(priv->irq, ndev);

	sun8i_emac_rx_clean(ndev);
	sun8i_emac_tx_clean(ndev);

	kfree(priv->rx_skb);
	kfree(priv->txl);

	dma_free_coherent(priv->dev, priv->nbdesc_rx * sizeof(struct dma_desc),
			  priv->dd_rx, priv->dd_rx_phy);
	dma_free_coherent(priv->dev, priv->nbdesc_tx * sizeof(struct dma_desc),
			  priv->dd_tx, priv->dd_tx_phy);

	return 0;
}

static netdev_tx_t sun8i_emac_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct dma_desc *ddesc;
	struct dma_desc *first;
	int i = 0, rbd_first;
	unsigned int len, fraglen, tlen;
	u32 v;
	int n;
	int nf;
	const skb_frag_t *frag;
	int do_csum = 0;

	if (skb_put_padto(skb, ETH_ZLEN))
		return NETDEV_TX_OK;
	len = skb_headlen(skb);

	n = skb_shinfo(skb)->nr_frags;

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		do_csum = 1;
		priv->estats.tx_hw_csum++;
	}
	netif_dbg(priv, tx_queued, ndev, "%s len=%u skblen=%u %x\n", __func__,
		  len, skb->len,
		  (skb->ip_summed == CHECKSUM_PARTIAL));

	spin_lock(&priv->tx_lock);

	/* check for contigous space
	 * We need at least 1(skb->data) + n(numfrags) + 1(one clean slot)
	 */
	if (rb_tx_numfreedesc(ndev) < n + 2) {
		dev_err_ratelimited(priv->dev, "BUG!: TX is full %d %d\n",
				    priv->tx_dirty, priv->tx_slot);
		netif_stop_queue(ndev);
		spin_unlock(&priv->tx_lock);
		return NETDEV_TX_BUSY;
	}
	i = priv->tx_slot;

	ddesc = priv->dd_tx + i;
	first = priv->dd_tx + i;
	rbd_first = i;

	priv->tx_slot = (i + 1 + n) % priv->nbdesc_tx;

	ddesc->buf_addr = dma_map_single(priv->dev, skb->data, len,
					 DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, ddesc->buf_addr)) {
		dev_err(priv->dev, "ERROR: Cannot map buffer for DMA\n");
		goto xmit_error;
	}
	priv->txl[i].map = MAP_SINGLE;
	priv->txl[i].skb = skb;

	tlen = len;
	ddesc->ctl = len;
	/* Undocumented bit that make it works
	 * Without it, packets never be sent on H3 SoC
	 */
	ddesc->ctl |= SUN8I_EMAC_MAGIC_TX_BIT;
	if (do_csum)
		ddesc->ctl |= SUN8I_EMAC_TX_DO_CRC;

	/* handle fragmented skb, one descriptor per fragment  */
	for (nf = 0; nf < n; nf++) {
		frag = &skb_shinfo(skb)->frags[nf];
		rb_inc(&i, priv->nbdesc_tx);
		priv->txl[i].skb = skb;
		ddesc = priv->dd_tx + i;
		fraglen = skb_frag_size(frag);
		ddesc->ctl = fraglen;
		tlen += fraglen,
		ddesc->ctl |= SUN8I_EMAC_MAGIC_TX_BIT;
		if (do_csum)
			ddesc->ctl |= SUN8I_EMAC_TX_DO_CRC;

		ddesc->buf_addr = skb_frag_dma_map(priv->dev, frag, 0,
				fraglen, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->dev, ddesc->buf_addr)) {
			dev_err(priv->dev, "Cannot map buffer for DMA\n");
			goto xmit_error;
		}
		priv->txl[i].map = MAP_PAGE;
		ddesc->status = SUN8I_COULD_BE_USED_BY_DMA;
	}

	/* frame end */
	ddesc->ctl |= DSC_TX_LAST;
	/* We want an interrupt after transmission */
	ddesc->ctl |= SUN8I_EMAC_WANT_INT;

	rb_inc(&i, priv->nbdesc_tx);

	/* frame begin */
	first->ctl |= DSC_TX_FIRST;
	wmb();/* SUN8I_COULD_BE_USED_BY_DMA must be the last value written */
	first->status = SUN8I_COULD_BE_USED_BY_DMA;
	priv->tx_slot = i;

	/* Trying to optimize this (recording DMA start/stop) seems
	 * to lead to errors. So we always start DMA.
	 */
	v = readl(priv->base + SUN8I_EMAC_TX_CTL1);
	v |= TX_DMA_START;
	v |= TX_DMA_EN;
	writel(v, priv->base + SUN8I_EMAC_TX_CTL1);

	if (rb_tx_numfreedesc(ndev) < MAX_SKB_FRAGS + 1) {
		netif_stop_queue(ndev);
		priv->estats.tx_stop_queue++;
	}
	priv->estats.tx_used_desc = rb_tx_numfreedesc(ndev);
	priv->ndev->stats.tx_packets++;
	priv->ndev->stats.tx_bytes += tlen;

	spin_unlock(&priv->tx_lock);

	return NETDEV_TX_OK;

xmit_error:
	/* destroy skb and return TX OK Documentation/DMA-API-HOWTO.txt */
	/* clean descritors from rbd_first to i */
	ddesc->ctl = 0;
	wmb(); /* setting to DCLEAN is the last value to be set */
	ddesc->status = DCLEAN;
	do {
		ddesc = priv->dd_tx + rbd_first;
		ddesc->ctl = 0;
		wmb(); /* setting to DCLEAN is the last value to be set */
		ddesc->status = DCLEAN;
		rb_inc(&rbd_first, priv->nbdesc_tx);
	} while (rbd_first != i);
	spin_unlock(&priv->tx_lock);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static int sun8i_emac_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int max_mtu;

	dev_info(priv->dev, "%s set MTU to %d\n", __func__, new_mtu);

	if (netif_running(ndev)) {
		dev_err(priv->dev, "%s: must be stopped to change its MTU\n",
			ndev->name);
		return -EBUSY;
	}

	max_mtu = SKB_MAX_HEAD(NET_SKB_PAD + NET_IP_ALIGN);

	if ((new_mtu < 68) || (new_mtu > max_mtu)) {
		dev_err(priv->dev, "%s: invalid MTU, max MTU is: %d\n",
			ndev->name, max_mtu);
		return -EINVAL;
	}

	ndev->mtu = new_mtu;
	netdev_update_features(ndev);
	return 0;
}

static netdev_features_t sun8i_emac_fix_features(struct net_device *ndev,
						 netdev_features_t features)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	netif_dbg(priv, drv, ndev, "%s %llx\n", __func__, features);
	return features;
}

static int sun8i_emac_set_features(struct net_device *ndev,
				   netdev_features_t features)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v;

	v = readl(priv->base + SUN8I_EMAC_BASIC_CTL0);
	if (features & NETIF_F_LOOPBACK && netif_running(ndev)) {
		netif_info(priv, hw, ndev, "Set loopback features");
		v |= BIT(1);
	} else {
		netif_info(priv, hw, ndev, "Unset loopback features");
		v &= ~BIT(1);
	}
	writel(v, priv->base + SUN8I_EMAC_BASIC_CTL0);

	v = readl(priv->base + SUN8I_EMAC_RX_CTL0);
	if (features & NETIF_F_RXCSUM) {
		v |= SUN8I_EMAC_RX_DO_CRC;
		netif_info(priv, hw, ndev, "Doing RX CRC check by hardware");
	} else {
		v &= ~SUN8I_EMAC_RX_DO_CRC;
		netif_info(priv, hw, ndev, "No RX CRC check by hardware");
	}
	if (features & NETIF_F_RXFCS) {
		v &= ~SUN8I_EMAC_RX_STRIP_FCS;
		netif_info(priv, hw, ndev, "Keep FCS");
	} else {
		v |= SUN8I_EMAC_RX_STRIP_FCS;
		netif_info(priv, hw, ndev, "Strip FCS");
	}
	writel(v, priv->base + SUN8I_EMAC_RX_CTL0);

	netif_dbg(priv, drv, ndev, "%s %llx %x\n", __func__, features, v);

	return 0;
}

static void sun8i_emac_set_rx_mode(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v = 0;
	int i = 0;
	struct netdev_hw_addr *ha;

	/* Receive all multicast frames */
	v |= BIT(16);
	/* Receive all control frames */
	v |= BIT(13);
	if (ndev->flags & IFF_PROMISC)
		v |= BIT(1);
	if (netdev_uc_count(ndev) > 7) {
		v |= BIT(1);
	} else {
		netdev_for_each_uc_addr(ha, ndev) {
			i++;
			sun8i_emac_set_macaddr(priv, ha->addr, i);
		}
	}
	writel(v, priv->base + SUN8I_EMAC_RX_FRM_FLT);
}

static void sun8i_emac_tx_timeout(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	netdev_err(ndev, "%s\n", __func__);

	sun8i_emac_stop_tx(ndev);

	sun8i_emac_tx_clean(ndev);

	/* write start of tx ring descriptor */
	writel(priv->dd_tx_phy, priv->base + SUN8I_EMAC_TX_DESC_LIST);

	sun8i_emac_start_tx(ndev);

	netdev_reset_queue(ndev);

	ndev->stats.tx_errors++;
	netif_wake_queue(ndev);
}

static int sun8i_emac_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct phy_device *phydev = ndev->phydev;

	if (!netif_running(ndev))
		return -EINVAL;

	if (!phydev)
		return -ENODEV;

	return phy_mii_ioctl(phydev, rq, cmd);
}

static int sun8i_emac_check_if_running(struct net_device *ndev)
{
	if (!netif_running(ndev))
		return -EINVAL;
	return 0;
}

static int sun8i_emac_get_sset_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(estats_str);
	}
	return -EOPNOTSUPP;
}

static int sun8i_emac_ethtool_get_settings(struct net_device *ndev,
					   struct ethtool_cmd *cmd)
{
	struct phy_device *phy = ndev->phydev;
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	if (!phy) {
		netdev_err(ndev, "%s: %s: PHY is not registered\n",
			   __func__, ndev->name);
		return -ENODEV;
	}

	if (!netif_running(ndev)) {
		dev_err(priv->dev, "interface disabled: we cannot track link speed / duplex setting\n");
		return -EBUSY;
	}

	return phy_ethtool_gset(phy, cmd);
}

static int sun8i_emac_ethtool_set_settings(struct net_device *ndev,
					   struct ethtool_cmd *cmd)
{
	struct phy_device *phy = ndev->phydev;

	return phy_ethtool_sset(phy, cmd);
}

static void sun8i_emac_ethtool_getdrvinfo(struct net_device *ndev,
					  struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, "sun8i_emac", sizeof(info->driver));
	strcpy(info->version, "00");
	info->fw_version[0] = '\0';
}

static void sun8i_emac_ethtool_stats(struct net_device *ndev,
				     struct ethtool_stats *dummy, u64 *data)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	memcpy(data, &priv->estats,
	       sun8i_emac_get_sset_count(ndev, ETH_SS_STATS) * sizeof(u64));
}

static void sun8i_emac_ethtool_strings(struct net_device *dev, u32 stringset,
				       u8 *buffer)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buffer, &estats_str, sizeof(estats_str));
		break;
	}
}

static u32 sun8i_emac_ethtool_getmsglevel(struct net_device *ndev)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	return priv->msg_enable;
}

static void sun8i_emac_ethtool_setmsglevel(struct net_device *ndev, u32 level)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	priv->msg_enable = level;
}

static void sun8i_emac_get_pauseparam(struct net_device *ndev,
				      struct ethtool_pauseparam *pause)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	pause->rx_pause = 0;
	pause->tx_pause = 0;
	pause->autoneg = ndev->phydev->autoneg;

	if (priv->flow_ctrl & FLOW_RX)
		pause->rx_pause = 1;
	if (priv->flow_ctrl & FLOW_TX)
		pause->tx_pause = 1;
}

static int sun8i_emac_set_pauseparam(struct net_device *ndev,
				     struct ethtool_pauseparam *pause)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	struct phy_device *phy = ndev->phydev;
	int new_pause = 0;
	int ret = 0;

	if (pause->rx_pause)
		new_pause |= FLOW_RX;
	if (pause->tx_pause)
		new_pause |= FLOW_TX;

	priv->flow_ctrl = new_pause;
	phy->autoneg = pause->autoneg;

	if (phy->autoneg) {
		if (netif_running(ndev))
			ret = phy_start_aneg(phy);
	} else {
		sun8i_emac_flow_ctrl(priv, phy->duplex, priv->flow_ctrl);
	}
	return ret;
}

static void sun8i_emac_ethtool_get_ringparam(struct net_device *ndev,
					     struct ethtool_ringparam *ring)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);

	ring->rx_pending = priv->nbdesc_rx;
	ring->tx_pending = priv->nbdesc_tx;
}

static int sun8i_emac_ethtool_set_ringparam(struct net_device *ndev,
					    struct ethtool_ringparam *ring)
{
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	int err;

	if (ring->rx_max_pending || ring->rx_mini_max_pending ||
	    ring->rx_jumbo_max_pending || ring->rx_mini_pending ||
	    ring->rx_jumbo_pending || ring->tx_max_pending)
		return -EINVAL;

	if (ring->tx_pending < MAX_SKB_FRAGS + 1) {
		netdev_err(ndev, "The number of TX descriptors is too low");
		return -EINVAL;
	}

	sun8i_emac_stop_tx(ndev);
	sun8i_emac_stop_rx(ndev);

	sun8i_emac_rx_clean(ndev);
	sun8i_emac_tx_clean(ndev);

	kfree(priv->rx_skb);
	kfree(priv->txl);

	dma_free_coherent(priv->dev, priv->nbdesc_rx * sizeof(struct dma_desc),
			  priv->dd_rx, priv->dd_rx_phy);
	dma_free_coherent(priv->dev, priv->nbdesc_tx * sizeof(struct dma_desc),
			  priv->dd_tx, priv->dd_tx_phy);

	priv->nbdesc_rx = ring->rx_pending;
	priv->nbdesc_tx = ring->tx_pending;
	err = sun8i_emac_alloc_rings(ndev);
	if (err) {
		/* Fatal error, we cannot re start */
		netdev_err(ndev, "Fail to allocate rings\n");
		return -EFAULT;
	}

	sun8i_emac_start_rx(ndev);
	sun8i_emac_start_tx(ndev);

	netif_start_queue(ndev);

	netdev_info(ndev, "Ring Param settings: rx: %d, tx %d\n",
		    ring->rx_pending, ring->tx_pending);
	return 0;
}

static const struct ethtool_ops sun8i_emac_ethtool_ops = {
	.begin = sun8i_emac_check_if_running,
	.get_settings = sun8i_emac_ethtool_get_settings,
	.set_settings = sun8i_emac_ethtool_set_settings,
	.get_link = ethtool_op_get_link,
	.get_pauseparam = sun8i_emac_get_pauseparam,
	.set_pauseparam = sun8i_emac_set_pauseparam,
	.get_ethtool_stats = sun8i_emac_ethtool_stats,
	.get_strings = sun8i_emac_ethtool_strings,
	.get_sset_count = sun8i_emac_get_sset_count,
	.get_drvinfo = sun8i_emac_ethtool_getdrvinfo,
	.get_msglevel = sun8i_emac_ethtool_getmsglevel,
	.set_msglevel = sun8i_emac_ethtool_setmsglevel,
	.get_ringparam = sun8i_emac_ethtool_get_ringparam,
	.set_ringparam = sun8i_emac_ethtool_set_ringparam,
};

static const struct net_device_ops sun8i_emac_netdev_ops = {
	.ndo_init = sun8i_emac_init,
	.ndo_uninit = sun8i_emac_uninit,
	.ndo_open = sun8i_emac_open,
	.ndo_start_xmit = sun8i_emac_xmit,
	.ndo_stop = sun8i_emac_stop,
	.ndo_change_mtu = sun8i_emac_change_mtu,
	.ndo_fix_features = sun8i_emac_fix_features,
	.ndo_set_features = sun8i_emac_set_features,
	.ndo_set_rx_mode = sun8i_emac_set_rx_mode,
	.ndo_tx_timeout = sun8i_emac_tx_timeout,
	.ndo_do_ioctl = sun8i_emac_ioctl,
	.ndo_set_mac_address = eth_mac_addr,
};

static irqreturn_t sun8i_emac_dma_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct sun8i_emac_priv *priv = netdev_priv(ndev);
	u32 v, u;

	v = readl(priv->base + SUN8I_EMAC_INT_STA);

	/* When this bit is asserted, a frame transmission is completed. */
	if (v & BIT(0)) {
		priv->estats.tx_int++;
		writel(0, priv->base + SUN8I_EMAC_INT_EN);
		napi_schedule(&priv->napi);
	}

	/* When this bit is asserted, the TX DMA FSM is stopped. */
	if (v & BIT(1))
		priv->estats.tx_dma_stop++;

	/* When this asserted, the TX DMA can not acquire next TX descriptor
	 * and TX DMA FSM is suspended.
	*/
	if (v & BIT(2))
		priv->estats.tx_dma_ua++;

	if (v & BIT(3))
		netif_dbg(priv, intr, ndev, "Unhandled interrupt TX TIMEOUT\n");

	if (v & BIT(4)) {
		netif_dbg(priv, intr, ndev, "Unhandled interrupt TX underflow\n");
		priv->estats.tx_underflow_int++;
	}

	/* When this bit asserted , the frame is transmitted to FIFO totally. */
	if (v & BIT(5)) {
		netif_dbg(priv, intr, ndev, "Unhandled interrupt TX_EARLY_INT\n");
		priv->estats.tx_early_int++;
	}

	/* When this bit is asserted, a frame reception is completed  */
	if (v & BIT(8)) {
		priv->estats.rx_int++;
		writel(0, priv->base + SUN8I_EMAC_INT_EN);
		napi_schedule(&priv->napi);
	}

	/* When this asserted, the RX DMA can not acquire next TX descriptor
	 * and RX DMA FSM is suspended.
	*/
	if (v & BIT(9)) {
		u = readl(priv->base + SUN8I_EMAC_RX_CTL1);
		netif_info(priv, intr, ndev, "Re-run RX DMA %x\n", u);
		writel(u | RX_DMA_START, priv->base + SUN8I_EMAC_RX_CTL1);
		priv->estats.rx_dma_ua++;
	}

	if (v & BIT(10)) {
		netif_dbg(priv, intr, ndev, "Unhandled interrupt RX_DMA_STOPPED_INT\n");
		priv->estats.rx_dma_stop++;
	}
	if (v & BIT(11))
		netif_dbg(priv, intr, ndev, "Unhandled interrupt RX_TIMEOUT\n");
	if (v & BIT(12))
		netif_dbg(priv, intr, ndev, "Unhandled interrupt RX OVERFLOW\n");
	if (v & BIT(13)) {
		netif_dbg(priv, intr, ndev, "Unhandled interrupt RX EARLY\n");
		priv->estats.rx_early_int++;
	}
	if (v & BIT(16))
		netif_dbg(priv, intr, ndev, "Unhandled interrupt RGMII\n");

	/* the datasheet state those register as read-only
	 * but nothing work(freeze) without writing to it
	 */
	writel(v & 0x3FFF, priv->base + SUN8I_EMAC_INT_STA);

	return IRQ_HANDLED;
}

static int sun8i_emac_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct sun8i_emac_priv *priv;
	struct net_device *ndev;
	struct resource *res;
	int ret;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	ndev = alloc_etherdev(sizeof(*priv));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	priv = netdev_priv(ndev);
	platform_set_drvdata(pdev, ndev);

	priv->variant = (enum emac_variant)of_device_get_match_data(&pdev->dev);
	if (!priv->variant) {
		dev_err(&pdev->dev, "Missing sun8i-emac variant\n");
		return -EINVAL;
	}

	priv->phy_node = of_parse_phandle(node, "phy", 0);
	if (!priv->phy_node) {
		netdev_err(ndev, "No associated PHY\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		ret = PTR_ERR(priv->base);
		dev_err(&pdev->dev, "Cannot request MMIO: %d\n", ret);
		return ret;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "syscon");
	priv->syscon = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->syscon)) {
		ret = PTR_ERR(priv->syscon);
		dev_err(&pdev->dev,
			"Cannot map system control registers: %d\n", ret);
		return ret;
	}

	priv->ahb_clk = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(priv->ahb_clk)) {
		ret = PTR_ERR(priv->ahb_clk);
		dev_err(&pdev->dev, "Cannot get AHB clock err=%d\n", ret);
		goto probe_err;
	}

	priv->rst = devm_reset_control_get_optional(&pdev->dev, "ahb");
	if (IS_ERR(priv->rst)) {
		ret = PTR_ERR(priv->rst);
		if (ret == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_info(&pdev->dev, "No MAC reset control found %d\n", ret);
		priv->rst = NULL;
	}

	if (priv->variant == H3_EMAC)
		priv->use_internal_phy = of_property_read_bool(node,
				"allwinner,use-internal-phy");

	if (priv->use_internal_phy) {
		priv->ephy_clk = devm_clk_get(&pdev->dev, "ephy");
		if (IS_ERR(priv->ephy_clk)) {
			ret = PTR_ERR(priv->ephy_clk);
			dev_err(&pdev->dev, "Cannot get EPHY clock err=%d\n",
				ret);
			goto probe_err;
		}

		priv->rst_ephy = devm_reset_control_get_optional(&pdev->dev,
								 "ephy");
		if (IS_ERR(priv->rst_ephy)) {
			ret = PTR_ERR(priv->rst_ephy);
			if (ret == -EPROBE_DEFER)
				goto probe_err;
			dev_info(&pdev->dev,
				 "No EPHY reset control found %d\n", ret);
			priv->rst_ephy = NULL;
		}
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		ret = priv->irq;
		dev_err(&pdev->dev, "Cannot claim IRQ: %d\n", ret);
		goto probe_err;
	}

	spin_lock_init(&priv->tx_lock);

	ndev->netdev_ops = &sun8i_emac_netdev_ops;
	ndev->ethtool_ops = &sun8i_emac_ethtool_ops;

	priv->ndev = ndev;
	priv->dev = &pdev->dev;

	ndev->hw_features = NETIF_F_SG | NETIF_F_HIGHDMA;
	ndev->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
		NETIF_F_RXCSUM;
	ndev->features |= ndev->hw_features;
	ndev->hw_features |= NETIF_F_RXFCS;
	ndev->hw_features |= NETIF_F_RXALL;
	ndev->hw_features |= NETIF_F_LOOPBACK;
	ndev->priv_flags |= IFF_UNICAST_FLT;

	ndev->watchdog_timeo = msecs_to_jiffies(5000);
	netif_carrier_off(ndev);

	/* Benched on OPIPC with 100M, setting more than 256 does not give any
	 * perf boost
	 */
	priv->nbdesc_rx = 128;
	priv->nbdesc_tx = 256;

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "ERROR: Register %s failed\n", ndev->name);
		goto probe_err;
	}

	return 0;

probe_err:
	free_netdev(ndev);
	return ret;
}

static int sun8i_emac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);

	unregister_netdev(ndev);
	platform_set_drvdata(pdev, NULL);
	free_netdev(ndev);

	return 0;
}

static const struct of_device_id sun8i_emac_of_match_table[] = {
	{ .compatible = "allwinner,sun8i-a83t-emac",
	  .data = (void *)A83T_EMAC },
	{ .compatible = "allwinner,sun8i-h3-emac",
	  .data = (void *)H3_EMAC },
	{ .compatible = "allwinner,sun50i-a64-emac",
	  .data = (void *)A64_EMAC },
	{}
};
MODULE_DEVICE_TABLE(of, sun8i_emac_of_match_table);

static struct platform_driver sun8i_emac_driver = {
	.probe          = sun8i_emac_probe,
	.remove         = sun8i_emac_remove,
	.driver         = {
		.name           = "sun8i-emac",
		.of_match_table	= sun8i_emac_of_match_table,
	},
};

module_platform_driver(sun8i_emac_driver);

MODULE_DESCRIPTION("sun8i Ethernet driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("LABBE Corentin <clabbe.montjoie@gmail.com");
