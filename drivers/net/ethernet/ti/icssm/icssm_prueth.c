// SPDX-License-Identifier: GPL-2.0

/* Texas Instruments ICSSM Ethernet Driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/etherdevice.h>
#include <linux/genalloc.h>
#include <linux/if_bridge.h>
#include <linux/if_hsr.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/remoteproc/pruss.h>
#include <linux/ptp_classify.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <net/pkt_cls.h>

#include "icssm_prueth.h"
#include "icssm_prueth_switch.h"
#include "icssm_vlan_mcast_filter_mmap.h"
#include "icssm_prueth_ecap.h"
#include "../icssg/icssg_mii_rt.h"
#include "../icssg/icss_iep.h"

#define OCMC_RAM_SIZE		(SZ_64K)

#define TX_START_DELAY		0x40
#define TX_CLK_DELAY_100M	0x6
#define HR_TIMER_TX_DELAY_US	100

#define TIMESYNC_SECONDS_BIT_MASK   0x0000ffffffffffff

static struct prueth_fw_offsets fw_offsets_v2_1;

static void icssm_prueth_set_fw_offsets(struct prueth *prueth)
{
	/* Set VLAN/Multicast filter control and table offsets */
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		prueth->fw_offsets->vlan_ctrl_byte  =
			ICSS_EMAC_FW_VLAN_FILTER_CTRL_BITMAP_OFFSET;
		prueth->fw_offsets->vlan_filter_tbl =
			ICSS_EMAC_FW_VLAN_FLTR_TBL_BASE_ADDR;

		prueth->fw_offsets->mc_ctrl_byte  =
			ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_OFFSET;
		prueth->fw_offsets->mc_filter_mask =
			ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OFFSET;
		prueth->fw_offsets->mc_filter_tbl =
			ICSS_EMAC_FW_MULTICAST_FILTER_TABLE;
	}
}

static void icssm_prueth_write_reg(struct prueth *prueth,
				   enum prueth_mem region,
				   unsigned int reg, u32 val)
{
	writel_relaxed(val, prueth->mem[region].va + reg);
}

/* Below macro is for 1528 Byte Frame support, to Allow even with
 * Redundancy tag
 */
#define PRUSS_MII_RT_RX_FRMS_MAX_SUPPORT_EMAC  (VLAN_ETH_FRAME_LEN + \
						ETH_FCS_LEN + \
						ICSSM_LRE_TAG_SIZE)

static void icssm_prueth_ptp_ts_enable(struct prueth_emac *emac)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u8 val = 0;

	if (emac->ptp_tx_enable) {
		/* Disable fw background task */
		val &= ~TIMESYNC_CTRL_BG_ENABLE;
		/* Enable forced 2-step */
		val |= TIMESYNC_CTRL_FORCED_2STEP;
	}

	writeb(val, sram + TIMESYNC_CTRL_VAR_OFFSET);
	/* disable PTP forwarding for switch */
	if (PRUETH_IS_SWITCH(emac->prueth))
		writeb(1, sram + DISABLE_PTP_FRAME_FORWARDING_CTRL_OFFSET);
}

static void icssm_prueth_ptp_tx_ts_enable(struct prueth_emac *emac,
					  bool enable)
{
	emac->ptp_tx_enable = enable;
	icssm_prueth_ptp_ts_enable(emac);
}

static bool icssm_prueth_ptp_tx_ts_is_enabled(struct prueth_emac *emac)
{
	return !!emac->ptp_tx_enable;
}

static void icssm_prueth_ptp_rx_ts_enable(struct prueth_emac *emac,
					  bool enable)
{
	emac->ptp_rx_enable = enable;
	icssm_prueth_ptp_ts_enable(emac);
}

static bool icssm_prueth_ptp_rx_ts_is_enabled(struct prueth_emac *emac)
{
	return !!emac->ptp_rx_enable;
}

/* ensure that order of PRUSS mem regions is same as enum prueth_mem */
static enum pruss_mem pruss_mem_ids[] = { PRUSS_MEM_DRAM0, PRUSS_MEM_DRAM1,
					  PRUSS_MEM_SHRD_RAM2 };

static const struct prueth_queue_info queue_infos[][NUM_QUEUES] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		[PRUETH_QUEUE1] = {
			P0_Q1_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET,
			P0_Q1_BD_OFFSET,
			P0_Q1_BD_OFFSET + ((HOST_QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P0_Q2_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET + 8,
			P0_Q2_BD_OFFSET,
			P0_Q2_BD_OFFSET + ((HOST_QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P0_Q3_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET + 16,
			P0_Q3_BD_OFFSET,
			P0_Q3_BD_OFFSET + ((HOST_QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P0_Q4_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET + 24,
			P0_Q4_BD_OFFSET,
			P0_Q4_BD_OFFSET + ((HOST_QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		[PRUETH_QUEUE1] = {
			P1_Q1_BUFFER_OFFSET,
			P1_Q1_BUFFER_OFFSET + ((QUEUE_1_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P1_Q1_BD_OFFSET,
			P1_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P1_Q2_BUFFER_OFFSET,
			P1_Q2_BUFFER_OFFSET + ((QUEUE_2_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P1_Q2_BD_OFFSET,
			P1_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P1_Q3_BUFFER_OFFSET,
			P1_Q3_BUFFER_OFFSET + ((QUEUE_3_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P1_Q3_BD_OFFSET,
			P1_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P1_Q4_BUFFER_OFFSET,
			P1_Q4_BUFFER_OFFSET + ((QUEUE_4_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P1_Q4_BD_OFFSET,
			P1_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		[PRUETH_QUEUE1] = {
			P2_Q1_BUFFER_OFFSET,
			P2_Q1_BUFFER_OFFSET + ((QUEUE_1_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P2_Q1_BD_OFFSET,
			P2_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P2_Q2_BUFFER_OFFSET,
			P2_Q2_BUFFER_OFFSET + ((QUEUE_2_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P2_Q2_BD_OFFSET,
			P2_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P2_Q3_BUFFER_OFFSET,
			P2_Q3_BUFFER_OFFSET + ((QUEUE_3_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P2_Q3_BD_OFFSET,
			P2_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P2_Q4_BUFFER_OFFSET,
			P2_Q4_BUFFER_OFFSET + ((QUEUE_4_SIZE - 1) *
					ICSS_BLOCK_SIZE),
			P2_Q4_BD_OFFSET,
			P2_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
};

const struct prueth_queue_desc queue_descs[][NUM_QUEUES] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		{ .rd_ptr = P0_Q1_BD_OFFSET, .wr_ptr = P0_Q1_BD_OFFSET, },
		{ .rd_ptr = P0_Q2_BD_OFFSET, .wr_ptr = P0_Q2_BD_OFFSET, },
		{ .rd_ptr = P0_Q3_BD_OFFSET, .wr_ptr = P0_Q3_BD_OFFSET, },
		{ .rd_ptr = P0_Q4_BD_OFFSET, .wr_ptr = P0_Q4_BD_OFFSET, },
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		{ .rd_ptr = P1_Q1_BD_OFFSET, .wr_ptr = P1_Q1_BD_OFFSET, },
		{ .rd_ptr = P1_Q2_BD_OFFSET, .wr_ptr = P1_Q2_BD_OFFSET, },
		{ .rd_ptr = P1_Q3_BD_OFFSET, .wr_ptr = P1_Q3_BD_OFFSET, },
		{ .rd_ptr = P1_Q4_BD_OFFSET, .wr_ptr = P1_Q4_BD_OFFSET, },
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		{ .rd_ptr = P2_Q1_BD_OFFSET, .wr_ptr = P2_Q1_BD_OFFSET, },
		{ .rd_ptr = P2_Q2_BD_OFFSET, .wr_ptr = P2_Q2_BD_OFFSET, },
		{ .rd_ptr = P2_Q3_BD_OFFSET, .wr_ptr = P2_Q3_BD_OFFSET, },
		{ .rd_ptr = P2_Q4_BD_OFFSET, .wr_ptr = P2_Q4_BD_OFFSET, },
	}
};

static void icssm_prueth_hostconfig(struct prueth *prueth)
{
	void __iomem *sram_base = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	void __iomem *sram;

	/* queue size lookup table */
	sram = sram_base + HOST_QUEUE_SIZE_ADDR;
	writew(HOST_QUEUE_1_SIZE, sram);
	writew(HOST_QUEUE_2_SIZE, sram + 2);
	writew(HOST_QUEUE_3_SIZE, sram + 4);
	writew(HOST_QUEUE_4_SIZE, sram + 6);

	/* queue information table */
	sram = sram_base + HOST_Q1_RX_CONTEXT_OFFSET;
	memcpy_toio(sram, queue_infos[PRUETH_PORT_QUEUE_HOST],
		    sizeof(queue_infos[PRUETH_PORT_QUEUE_HOST]));

	/* buffer offset table */
	sram = sram_base + HOST_QUEUE_OFFSET_ADDR;
	writew(P0_Q1_BUFFER_OFFSET, sram);
	writew(P0_Q2_BUFFER_OFFSET, sram + 2);
	writew(P0_Q3_BUFFER_OFFSET, sram + 4);
	writew(P0_Q4_BUFFER_OFFSET, sram + 6);

	/* buffer descriptor offset table*/
	sram = sram_base + HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR;
	writew(P0_Q1_BD_OFFSET, sram);
	writew(P0_Q2_BD_OFFSET, sram + 2);
	writew(P0_Q3_BD_OFFSET, sram + 4);
	writew(P0_Q4_BD_OFFSET, sram + 6);

	/* queue table */
	sram = sram_base + HOST_QUEUE_DESC_OFFSET;
	memcpy_toio(sram, queue_descs[PRUETH_PORT_QUEUE_HOST],
		    sizeof(queue_descs[PRUETH_PORT_QUEUE_HOST]));
}

static void icssm_prueth_mii_init(struct prueth *prueth)
{
	u32 txcfg_reg, txcfg, txcfg2;
	struct regmap *mii_rt;
	u32 rxcfg_reg, rxcfg;

	mii_rt = prueth->mii_rt;

	rxcfg = PRUSS_MII_RT_RXCFG_RX_ENABLE |
		PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS |
		PRUSS_MII_RT_RXCFG_RX_L2_EN |
		PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE |
		PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS;

	/* Configuration of Port 0 Rx */
	rxcfg_reg = PRUSS_MII_RT_RXCFG0;

	regmap_write(mii_rt, rxcfg_reg, rxcfg);

	/* Configuration of Port 1 Rx */
	rxcfg_reg = PRUSS_MII_RT_RXCFG1;

	rxcfg |= PRUSS_MII_RT_RXCFG_RX_MUX_SEL;

	regmap_write(mii_rt, rxcfg_reg, rxcfg);

	txcfg = PRUSS_MII_RT_TXCFG_TX_ENABLE |
		PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE |
		PRUSS_MII_RT_TXCFG_TX_32_MODE_EN |
		(TX_START_DELAY << PRUSS_MII_RT_TXCFG_TX_START_DELAY_SHIFT) |
		(TX_CLK_DELAY_100M << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);

	txcfg2 = txcfg;
	if (!PRUETH_IS_EMAC(prueth))
		txcfg2 |= PRUSS_MII_RT_TXCFG_TX_MUX_SEL;

	/* Configuration of Port 0 Tx */
	txcfg_reg = PRUSS_MII_RT_TXCFG0;

	regmap_write(mii_rt, txcfg_reg, txcfg2);

	txcfg2 = txcfg;
	if (PRUETH_IS_EMAC(prueth))
		txcfg2 |= PRUSS_MII_RT_TXCFG_TX_MUX_SEL;

	/* Configuration of Port 1 Tx */
	txcfg_reg = PRUSS_MII_RT_TXCFG1;

	regmap_write(mii_rt, txcfg_reg, txcfg2);

	txcfg_reg = PRUSS_MII_RT_RX_FRMS0;

	/* Min frame length should be set to 64 to allow receive of standard
	 * Ethernet frames such as PTP, LLDP that will not have the tag/rct.
	 * Actual size written to register is size - 1 per TRM. This also
	 * includes CRC/FCS.
	 */
	txcfg = FIELD_PREP(PRUSS_MII_RT_RX_FRMS_MIN_FRM_MASK,
			   (PRUSS_MII_RT_RX_FRMS_MIN_FRM - 1));

	/* For EMAC, set Max frame size to 1528 i.e size with VLAN.
	 * Actual size written to register is size - 1 as per TRM.
	 * Since driver support run time change of protocol, driver
	 * must overwrite the values based on Ethernet type.
	 */
	txcfg |= FIELD_PREP(PRUSS_MII_RT_RX_FRMS_MAX_FRM_MASK,
			    (PRUSS_MII_RT_RX_FRMS_MAX_SUPPORT_EMAC - 1));

	regmap_write(mii_rt, txcfg_reg, txcfg);

	txcfg_reg = PRUSS_MII_RT_RX_FRMS1;

	regmap_write(mii_rt, txcfg_reg, txcfg);
}

static void icssm_prueth_clearmem(struct prueth *prueth, enum prueth_mem region)
{
	memset_io(prueth->mem[region].va, 0, prueth->mem[region].size);
}

static void icssm_prueth_hostinit(struct prueth *prueth)
{
	/* Clear shared RAM */
	icssm_prueth_clearmem(prueth, PRUETH_MEM_SHARED_RAM);

	/* Clear OCMC RAM */
	icssm_prueth_clearmem(prueth, PRUETH_MEM_OCMC);

	/* Clear data RAMs */
	if (prueth->eth_node[PRUETH_MAC0])
		icssm_prueth_clearmem(prueth, PRUETH_MEM_DRAM0);
	if (prueth->eth_node[PRUETH_MAC1])
		icssm_prueth_clearmem(prueth, PRUETH_MEM_DRAM1);

	/* Initialize host queues in shared RAM */
	if (!PRUETH_IS_EMAC(prueth))
		icssm_prueth_sw_hostconfig(prueth);
	else
		icssm_prueth_hostconfig(prueth);

	/* Configure MII_RT */
	icssm_prueth_mii_init(prueth);
}

/* This function initialize the driver in EMAC or HSR or PRP mode
 * based on eth_type
 */
static void icssm_prueth_init_ethernet_mode(struct prueth *prueth)
{
	icssm_prueth_set_fw_offsets(prueth);
	icssm_prueth_hostinit(prueth);
}

static void icssm_prueth_port_enable(struct prueth_emac *emac, bool enable)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *port_ctrl, *vlan_ctrl;
	u32 vlan_ctrl_offset;
	void __iomem *ram;

	vlan_ctrl_offset = prueth->fw_offsets->vlan_ctrl_byte;

	ram = prueth->mem[emac->dram].va;
	port_ctrl = ram + PORT_CONTROL_ADDR;
	writeb(!!enable, port_ctrl);

	vlan_ctrl = ram + vlan_ctrl_offset;
	writeb(!!enable, vlan_ctrl);
}

static int icssm_prueth_emac_config(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	u32 sharedramaddr, ocmcaddr;
	void __iomem *dram_base;
	void __iomem *mac_addr;
	void __iomem *dram;
	void __iomem *sram;

	/* PRU needs local shared RAM address for C28 */
	sharedramaddr = ICSS_LOCAL_SHARED_RAM;
	/* PRU needs real global OCMC address for C30*/
	ocmcaddr = (u32)prueth->mem[PRUETH_MEM_OCMC].pa;
	sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	/* PRU needs local shared RAM address for C28 */
	sharedramaddr = ICSS_LOCAL_SHARED_RAM;
	/* PRU needs real global OCMC address for C30*/
	ocmcaddr = (u32)prueth->mem[PRUETH_MEM_OCMC].pa;

	/* Clear data RAM */
	icssm_prueth_clearmem(prueth, emac->dram);

	dram_base = prueth->mem[emac->dram].va;

	/* setup mac address */
	mac_addr = dram_base + PORT_MAC_ADDR;
	memcpy_toio(mac_addr, emac->mac_addr, 6);

	/* queue information table */
	dram = dram_base + TX_CONTEXT_Q1_OFFSET_ADDR;
	memcpy_toio(dram, queue_infos[emac->port_id],
		    sizeof(queue_infos[emac->port_id]));

	/* queue table */
	dram = dram_base + PORT_QUEUE_DESC_OFFSET;
	memcpy_toio(dram, queue_descs[emac->port_id],
		    sizeof(queue_descs[emac->port_id]));

	emac->rx_queue_descs = sram + HOST_QUEUE_DESC_OFFSET;
	emac->tx_queue_descs = dram;

	/* Set in constant table C28 of PRU0 to ICSS Shared memory */
	pru_rproc_set_ctable(emac->pru, PRU_C28, sharedramaddr);

	/* Set in constant table C30 of PRU0 to OCMC memory */
	pru_rproc_set_ctable(emac->pru, PRU_C30, ocmcaddr);

	return 0;
}

/* called back by PHY layer if there is change in link state of hw port*/
static void icssm_emac_adjust_link(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct phy_device *phydev = emac->phydev;
	struct prueth *prueth = emac->prueth;
	bool new_state = false;
	enum prueth_mem region;
	unsigned long flags;
	u32 port_status = 0;
	u32 txcfg, mask;
	u32 delay;

	spin_lock_irqsave(&emac->lock, flags);

	if (phydev->link) {
		/* check the mode of operation */
		if (phydev->duplex != emac->duplex) {
			new_state = true;
			emac->duplex = phydev->duplex;
		}
		if (phydev->speed != emac->speed) {
			new_state = true;
			emac->speed = phydev->speed;
		}
		if (!emac->link) {
			new_state = true;
			emac->link = 1;
		}
	} else if (emac->link) {
		new_state = true;
		emac->link = 0;
	}

	if (new_state) {
		phy_print_status(phydev);
		region = emac->dram;

		/* update phy/port status information based on PHY values*/
		if (emac->link) {
			port_status |= PORT_LINK_MASK;

			icssm_prueth_write_reg(prueth, region, PHY_SPEED_OFFSET,
					       emac->speed);

			delay = TX_CLK_DELAY_100M;
			delay = delay << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT;
			mask = PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_MASK;

			if (emac->port_id)
				txcfg = PRUSS_MII_RT_TXCFG1;
			else
				txcfg = PRUSS_MII_RT_TXCFG0;

			regmap_update_bits(prueth->mii_rt, txcfg, mask, delay);
		}

		writeb(port_status, prueth->mem[region].va +
		       PORT_STATUS_OFFSET);
	}

	if (emac->link) {
	       /* reactivate the transmit queue if it is stopped */
		if (netif_running(ndev) && netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
	} else {
		if (!netif_queue_stopped(ndev))
			netif_stop_queue(ndev);
	}

	spin_unlock_irqrestore(&emac->lock, flags);
}

static unsigned int
icssm_get_buff_desc_count(const struct prueth_queue_info *queue)
{
	unsigned int buffer_desc_count;

	buffer_desc_count = queue->buffer_desc_end -
			    queue->buffer_desc_offset;
	buffer_desc_count /= BD_SIZE;
	buffer_desc_count++;

	return buffer_desc_count;
}

static void icssm_get_block(struct prueth_queue_desc __iomem *queue_desc,
			    const struct prueth_queue_info *queue,
			    int *write_block, int *read_block)
{
	*write_block = (readw(&queue_desc->wr_ptr) -
			queue->buffer_desc_offset) / BD_SIZE;
	*read_block = (readw(&queue_desc->rd_ptr) -
		       queue->buffer_desc_offset) / BD_SIZE;
}

static u8 icssm_prueth_ptp_ts_event_type(struct sk_buff *skb, u8 *ptp_msgtype)
{
	unsigned int ptp_class = ptp_classify_raw(skb);
	struct ptp_header *hdr;
	u8 msgtype, event_type;

	if (ptp_class == PTP_CLASS_NONE)
		return PRUETH_PTP_TS_EVENTS;

	hdr = ptp_parse_header(skb, ptp_class);
	if (!hdr)
		return PRUETH_PTP_TS_EVENTS;

	msgtype = ptp_get_msgtype(hdr, ptp_class);
	/* Treat E2E Delay Req/Resp messages in the same way as P2P peer delay
	 * req/resp in driver here since firmware stores timestamps in the same
	 * memory location for either (since they cannot operate simultaneously
	 * anyway)
	 */
	switch (msgtype) {
	case PTP_MSGTYPE_SYNC:
		event_type = PRUETH_PTP_SYNC;
		break;
	case PTP_MSGTYPE_DELAY_REQ:
	case PTP_MSGTYPE_PDELAY_REQ:
		event_type = PRUETH_PTP_DLY_REQ;
		break;
	/* TODO: Check why PTP_MSGTYPE_DELAY_RESP needs timestamp
	 * and need for it.
	 */
	case 0x9:
	case PTP_MSGTYPE_PDELAY_RESP:
		event_type = PRUETH_PTP_DLY_RESP;
		break;
	default:
		event_type = PRUETH_PTP_TS_EVENTS;
	}

	if (ptp_msgtype)
		*ptp_msgtype = msgtype;

	return event_type;
}

static void icssm_prueth_ptp_tx_ts_reset(struct prueth_emac *emac, u8 event)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u32 ts_notify_offs, ts_offs;

	ts_offs = icssm_prueth_tx_ts_offs_get(emac->port_id - 1, event);
	ts_notify_offs = icssm_prueth_tx_ts_notify_offs_get(emac->port_id - 1,
							    event);

	writeb(0, sram + ts_notify_offs);
	memset_io(sram + ts_offs, 0, sizeof(u64));
}

static int icssm_prueth_ptp_tx_ts_enqueue(struct prueth_emac *emac,
					  struct sk_buff *skb)
{
	u8 event, changed = 0;
	unsigned long flags;

	if (skb_vlan_tagged(skb)) {
		__skb_pull(skb, VLAN_HLEN);
		changed += VLAN_HLEN;
	}

	skb_reset_mac_header(skb);
	event = icssm_prueth_ptp_ts_event_type(skb, NULL);
	__skb_push(skb, changed);
	if (event == PRUETH_PTP_TS_EVENTS) {
		netdev_err(emac->ndev, "invalid PTP event\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&emac->ptp_skb_lock, flags);
	if (emac->ptp_skb[event]) {
		dev_consume_skb_any(emac->ptp_skb[event]);
		icssm_prueth_ptp_tx_ts_reset(emac, event);
		netdev_warn(emac->ndev, "Dropped event waiting for tx ts.\n");
	}

	skb_get(skb);
	emac->ptp_skb[event] = skb;
	spin_unlock_irqrestore(&emac->ptp_skb_lock, flags);

	return 0;
}

irqreturn_t icssm_prueth_ptp_tx_irq_handle(int irq, void *dev)
{
	struct net_device *ndev = (struct net_device *)dev;
	struct prueth_emac *emac = netdev_priv(ndev);

	if (unlikely(netif_queue_stopped(ndev)))
		netif_wake_queue(ndev);

	if (icssm_prueth_ptp_tx_ts_is_enabled(emac))
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

/**
 * icssm_iep_get_timestamp_cycles - IEP get timestamp
 * @iep: icss_iep structure
 * @mem: io memory address
 *
 * To convert the 10 byte timestamp from firmware
 * i.e., nanoseconds part from 32-bit IEP counter(4 bytes)
 * seconds part updated by firmware(rev FW_REV1_0) in SRAM
 * (6 bytes) into 64-bit timestamp in ns
 *
 * Return: 64-bit converted timestamp
 */
u64 icssm_iep_get_timestamp_cycles(struct icss_iep *iep,
				   void __iomem *mem)
{
	u64 cycles, cycles_sec = 0;
	u32 cycles_ns;

	/* Copying the timestamp which has been captured
	 * by firmware at a hardware level.
	 * Its a static timestamp value.
	 */
	memcpy_fromio(&cycles_ns, mem, sizeof(cycles_ns));
	memcpy_fromio(&cycles_sec, mem + 4, sizeof(cycles_sec));

	/*To get the 6 bytes seconds part*/
	cycles_sec = (cycles_sec & TIMESYNC_SECONDS_BIT_MASK);
	cycles = cycles_ns + (cycles_sec * NSEC_PER_SEC);
	cycles = timecounter_cyc2time(&iep->tc, cycles);

	return cycles;
}

static u64 icssm_prueth_ptp_ts_get(struct prueth_emac *emac, u32 ts_offs)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u64 cycles;

	if (emac->prueth->fw_data->fw_rev == FW_REV_V1_0) {
		cycles = icssm_iep_get_timestamp_cycles(emac->prueth->iep,
							sram + ts_offs);
		/* 4 bytes of timestamp + 6 bytes of seconds counter */
		memset_io(sram + ts_offs, 0, 10);
	} else {
		memcpy_fromio(&cycles, sram + ts_offs, sizeof(cycles));
		memset_io(sram + ts_offs, 0, sizeof(cycles));
	}

	return cycles;
}

static void icssm_prueth_ptp_tx_ts_get(struct prueth_emac *emac, u8 event)
{
	struct skb_shared_hwtstamps ssh;
	struct sk_buff *skb;
	unsigned long flags;
	u64 ns;

	/* get the msg from list */
	spin_lock_irqsave(&emac->ptp_skb_lock, flags);
	skb = emac->ptp_skb[event];
	emac->ptp_skb[event] = NULL;
	spin_unlock_irqrestore(&emac->ptp_skb_lock, flags);
	if (!skb) {
		netdev_err(emac->ndev, "no tx msg %u found waiting for ts\n",
			   event);
		return;
	}

	/* get timestamp */
	ns = icssm_prueth_ptp_ts_get(emac,
				     icssm_prueth_tx_ts_offs_get
				     (emac->port_id - 1, event));

	memset(&ssh, 0, sizeof(ssh));
	ssh.hwtstamp = ns_to_ktime(ns);
	skb_tstamp_tx(skb, &ssh);
	dev_consume_skb_any(skb);
}

irqreturn_t icssm_prueth_ptp_tx_irq_work(int irq, void *dev)
{
	struct prueth_emac *emac = netdev_priv(dev);
	u32 ts_notify_offs, ts_notify_mask, i;
	void __iomem *sram;

	/* get and reset the ts notifications */
	sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	for (i = 0; i < PRUETH_PTP_TS_EVENTS; i++) {
		ts_notify_offs =
			icssm_prueth_tx_ts_notify_offs_get(emac->port_id - 1,
							   i);
		memcpy_fromio(&ts_notify_mask, sram + ts_notify_offs,
			      PRUETH_PTP_TS_NOTIFY_SIZE);
		memset_io(sram + ts_notify_offs, 0, PRUETH_PTP_TS_NOTIFY_SIZE);

		if (ts_notify_mask & PRUETH_PTP_TS_NOTIFY_MASK)
			icssm_prueth_ptp_tx_ts_get(emac, i);
	}

	return IRQ_HANDLED;
}

/**
 * icssm_emac_rx_irq - EMAC Rx interrupt handler
 * @irq: interrupt number
 * @dev_id: pointer to net_device
 *
 * EMAC Interrupt handler - we only schedule NAPI and not process any packets
 * here.
 *
 * Return: IRQ_HANDLED if the interrupt handled
 */
static irqreturn_t icssm_emac_rx_irq(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct prueth_emac *emac = netdev_priv(ndev);

	if (likely(netif_running(ndev))) {
		/* disable Rx system event */
		disable_irq_nosync(emac->rx_irq);
		napi_schedule(&emac->napi);
	}

	return IRQ_HANDLED;
}

/**
 * icssm_prueth_tx_enqueue - queue a packet to firmware for transmission
 *
 * @emac: EMAC data structure
 * @skb: packet data buffer
 * @queue_id: priority queue id
 *
 * Return: 0 (Success)
 */
static int icssm_prueth_tx_enqueue(struct prueth_emac *emac,
				   struct sk_buff *skb,
				   enum prueth_queue_id queue_id)
{
	struct prueth_queue_desc __iomem *queue_desc;
	const struct prueth_queue_info *txqueue;
	struct net_device *ndev = emac->ndev;
	struct prueth *prueth = emac->prueth;
	unsigned int buffer_desc_count;
	int free_blocks, update_block;
	bool buffer_wrapped = false;
	int write_block, read_block;
	void *src_addr, *dst_addr;
	int pkt_block_size;
	void __iomem *sram;
	void __iomem *dram;
	int txport, pktlen;
	u16 update_wr_ptr;
	u32 wr_buf_desc;
	void *ocmc_ram;

	if (!PRUETH_IS_EMAC(prueth))
		dram = prueth->mem[PRUETH_MEM_DRAM1].va;
	else
		dram = emac->prueth->mem[emac->dram].va;
	if (eth_skb_pad(skb)) {
		if (netif_msg_tx_err(emac) && net_ratelimit())
			netdev_err(ndev, "packet pad failed\n");
		return -ENOMEM;
	}

	/* which port to tx: MII0 or MII1 */
	txport = emac->tx_port_queue;
	src_addr = skb->data;
	pktlen = skb->len;
	/* Get the tx queue */
	queue_desc = emac->tx_queue_descs + queue_id;
	if (!PRUETH_IS_EMAC(prueth))
		txqueue = &sw_queue_infos[txport][queue_id];
	else
		txqueue = &queue_infos[txport][queue_id];

	buffer_desc_count = icssm_get_buff_desc_count(txqueue);

	/* the PRU firmware deals mostly in pointers already
	 * offset into ram, we would like to deal in indexes
	 * within the queue we are working with for code
	 * simplicity, calculate this here
	 */
	icssm_get_block(queue_desc, txqueue, &write_block, &read_block);

	if (write_block > read_block) {
		free_blocks = buffer_desc_count - write_block;
		free_blocks += read_block;
	} else if (write_block < read_block) {
		free_blocks = read_block - write_block;
	} else { /* they are all free */
		free_blocks = buffer_desc_count;
	}

	pkt_block_size = DIV_ROUND_UP(pktlen, ICSS_BLOCK_SIZE);
	if (pkt_block_size > free_blocks) /* out of queue space */
		return -ENOBUFS;

	/* calculate end BD address post write */
	update_block = write_block + pkt_block_size;

	/* Check for wrap around */
	if (update_block >= buffer_desc_count) {
		update_block %= buffer_desc_count;
		buffer_wrapped = true;
	}

	/* OCMC RAM is not cached and write order is not important */
	ocmc_ram = (__force void *)emac->prueth->mem[PRUETH_MEM_OCMC].va;
	dst_addr = ocmc_ram + txqueue->buffer_offset +
		   (write_block * ICSS_BLOCK_SIZE);

	/* Copy the data from socket buffer(DRAM) to PRU buffers(OCMC) */
	if (buffer_wrapped) { /* wrapped around buffer */
		int bytes = (buffer_desc_count - write_block) * ICSS_BLOCK_SIZE;
		int remaining;

		/* bytes is integral multiple of ICSS_BLOCK_SIZE but
		 * entire packet may have fit within the last BD
		 * if pkt_info.length is not integral multiple of
		 * ICSS_BLOCK_SIZE
		 */
		if (pktlen < bytes)
			bytes = pktlen;

		/* copy non-wrapped part */
		memcpy(dst_addr, src_addr, bytes);

		/* copy wrapped part */
		src_addr += bytes;
		remaining = pktlen - bytes;
		dst_addr = ocmc_ram + txqueue->buffer_offset;
		memcpy(dst_addr, src_addr, remaining);
	} else {
		memcpy(dst_addr, src_addr, pktlen);
	}

	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
	    icssm_prueth_ptp_tx_ts_is_enabled(emac)) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		icssm_prueth_ptp_tx_ts_enqueue(emac, skb);
	}

       /* update first buffer descriptor */
	wr_buf_desc = (pktlen << PRUETH_BD_LENGTH_SHIFT) &
		       PRUETH_BD_LENGTH_MASK;
	sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	if (!PRUETH_IS_EMAC(prueth))
		writel(wr_buf_desc, sram + readw(&queue_desc->wr_ptr));
	else
		writel(wr_buf_desc, dram + readw(&queue_desc->wr_ptr));

	/* update the write pointer in this queue descriptor, the firmware
	 * polls for this change so this will signal the start of transmission
	 */
	update_wr_ptr = txqueue->buffer_desc_offset + (update_block * BD_SIZE);
	writew(update_wr_ptr, &queue_desc->wr_ptr);

	return 0;
}

void icssm_parse_packet_info(struct prueth *prueth, u32 buffer_descriptor,
			     struct prueth_packet_info *pkt_info)
{
	pkt_info->start_offset = false;

	pkt_info->port = (buffer_descriptor & PRUETH_BD_PORT_MASK) >>
			 PRUETH_BD_PORT_SHIFT;
	pkt_info->length = (buffer_descriptor & PRUETH_BD_LENGTH_MASK) >>
			   PRUETH_BD_LENGTH_SHIFT;
	pkt_info->broadcast = !!(buffer_descriptor & PRUETH_BD_BROADCAST_MASK);
	pkt_info->error = !!(buffer_descriptor & PRUETH_BD_ERROR_MASK);
	pkt_info->sv_frame = false;
	pkt_info->lookup_success = !!(buffer_descriptor &
				      PRUETH_BD_LOOKUP_SUCCESS_MASK);
	pkt_info->flood = !!(buffer_descriptor & PRUETH_BD_SW_FLOOD_MASK);
	pkt_info->timestamp = !!(buffer_descriptor & PRUETH_BD_TIMESTAMP_MASK);
}

/**
 * icssm_emac_rx_packet - EMAC Receive function
 *
 * @emac: EMAC data structure
 * @bd_rd_ptr: Buffer descriptor read pointer
 * @pkt_info: packet information structure
 * @rxqueue: Receive queue information structure
 *
 * Get a packet from receive queue
 *
 * Return: 0 (Success)
 */
int icssm_emac_rx_packet(struct prueth_emac *emac, u16 *bd_rd_ptr,
			 struct prueth_packet_info *pkt_info,
			 const struct prueth_queue_info *rxqueue)
{
	struct net_device *ndev = emac->ndev;
	struct skb_shared_hwtstamps *ssh;
	unsigned int buffer_desc_count;
	int read_block, update_block;
	unsigned int actual_pkt_len;
	bool buffer_wrapped = false;
	void *src_addr, *dst_addr;
	u16 start_offset = 0;
	struct sk_buff *skb;
	int pkt_block_size;
	void *ocmc_ram;
	u64 ts;

	/* the PRU firmware deals mostly in pointers already
	 * offset into ram, we would like to deal in indexes
	 * within the queue we are working with for code
	 * simplicity, calculate this here
	 */
	buffer_desc_count = icssm_get_buff_desc_count(rxqueue);
	read_block = (*bd_rd_ptr - rxqueue->buffer_desc_offset) / BD_SIZE;
	pkt_block_size = DIV_ROUND_UP(pkt_info->length, ICSS_BLOCK_SIZE);
	if (pkt_info->timestamp)
		pkt_block_size++;

	/* calculate end BD address post read */
	update_block = read_block + pkt_block_size;

	/* Check for wrap around */
	if (update_block >= buffer_desc_count) {
		update_block %= buffer_desc_count;
		if (update_block)
			buffer_wrapped = true;
	}

	/* calculate new pointer in ram */
	*bd_rd_ptr = rxqueue->buffer_desc_offset + (update_block * BD_SIZE);

	/* Pkt len w/ HSR tag removed, If applicable */
	actual_pkt_len = pkt_info->length - start_offset;

	/* Allocate a socket buffer for this packet */
	skb = netdev_alloc_skb_ip_align(ndev, actual_pkt_len);
	if (!skb) {
		if (netif_msg_rx_err(emac) && net_ratelimit())
			netdev_err(ndev, "failed rx buffer alloc\n");
		return -ENOMEM;
	}

	dst_addr = skb->data;

	/* OCMC RAM is not cached and read order is not important */
	ocmc_ram = (__force void *)emac->prueth->mem[PRUETH_MEM_OCMC].va;

	/* Get the start address of the first buffer from
	 * the read buffer description
	 */
	src_addr = ocmc_ram + rxqueue->buffer_offset +
		   (read_block * ICSS_BLOCK_SIZE);
	src_addr += start_offset;

	/* Copy the data from PRU buffers(OCMC) to socket buffer(DRAM) */
	if (buffer_wrapped) { /* wrapped around buffer */
		int bytes = (buffer_desc_count - read_block) * ICSS_BLOCK_SIZE;
		int remaining;
		/* bytes is integral multiple of ICSS_BLOCK_SIZE but
		 * entire packet may have fit within the last BD
		 * if pkt_info.length is not integral multiple of
		 * ICSS_BLOCK_SIZE
		 */
		if (pkt_info->length < bytes)
			bytes = pkt_info->length;

		/* If applicable, account for the HSR tag removed */
		bytes -= start_offset;

		/* copy non-wrapped part */
		memcpy(dst_addr, src_addr, bytes);

		/* copy wrapped part */
		dst_addr += bytes;
		remaining = actual_pkt_len - bytes;

		src_addr = ocmc_ram + rxqueue->buffer_offset;
		memcpy(dst_addr, src_addr, remaining);
		src_addr += remaining;
	} else {
		memcpy(dst_addr, src_addr, actual_pkt_len);
		src_addr += actual_pkt_len;
	}

	if (PRUETH_IS_SWITCH(emac->prueth)) {
		skb->offload_fwd_mark = emac->offload_fwd_mark;
		if (!pkt_info->lookup_success)
			icssm_prueth_sw_learn_fdb(emac, skb->data + ETH_ALEN);
	}

	if (!pkt_info->sv_frame) {
		skb_put(skb, actual_pkt_len);
		if (icssm_prueth_ptp_rx_ts_is_enabled(emac) &&
		    pkt_info->timestamp) {
			src_addr = (void *)PTR_ALIGN((uintptr_t)src_addr,
						     ICSS_BLOCK_SIZE);
			if (emac->prueth->fw_data->fw_rev == FW_REV_V1_0) {
				ts = icssm_iep_get_timestamp_cycles
					(emac->prueth->iep,
					 (void __iomem *)src_addr);
			} else {
				memcpy(&ts, src_addr, sizeof(ts));
			}
			ssh = skb_hwtstamps(skb);
			memset(ssh, 0, sizeof(*ssh));
			ssh->hwtstamp = ns_to_ktime(ts);
		}

		/* send packet up the stack */
		skb->protocol = eth_type_trans(skb, ndev);
		local_bh_disable();
		netif_receive_skb(skb);
		local_bh_enable();
	} else {
		dev_kfree_skb_any(skb);
	}

	/* update stats */
	ndev->stats.rx_bytes += actual_pkt_len;
	ndev->stats.rx_packets++;

	return 0;
}

static irqreturn_t icssm_emac_rx_packets(struct prueth_emac *emac, int quota)
{
	struct prueth_queue_desc __iomem *queue_desc;
	const struct prueth_queue_info *rxqueue;
	struct prueth *prueth = emac->prueth;
	struct net_device_stats *ndevstats;
	struct prueth_packet_info pkt_info;
	int start_queue, end_queue;
	void __iomem *shared_ram;
	u16 bd_rd_ptr, bd_wr_ptr;
	u16 update_rd_ptr;
	u8 overflow_cnt;
	u32 rd_buf_desc;
	int used = 0;
	int i, ret;

	ndevstats = &emac->ndev->stats;
	shared_ram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	/*
	 * Start and end queue is made common for EMAC, RSTP
	 */
	start_queue = emac->rx_queue_start;
	end_queue = emac->rx_queue_end;

	/* search host queues for packets */
	for (i = start_queue; i <= end_queue; i++) {
		queue_desc = emac->rx_queue_descs + i;
		if (PRUETH_IS_SWITCH(emac->prueth))
			rxqueue = &sw_queue_infos[PRUETH_PORT_HOST][i];
		else
			rxqueue = &queue_infos[PRUETH_PORT_HOST][i];
		overflow_cnt = readb(&queue_desc->overflow_cnt);
		if (overflow_cnt > 0) {
			emac->ndev->stats.rx_over_errors += overflow_cnt;
			/* reset to zero */
			writeb(0, &queue_desc->overflow_cnt);
		}

		bd_rd_ptr = readw(&queue_desc->rd_ptr);
		bd_wr_ptr = readw(&queue_desc->wr_ptr);

		/* while packets are available in this queue */
		while (bd_rd_ptr != bd_wr_ptr) {
			/* get packet info from the read buffer descriptor */
			rd_buf_desc = readl(shared_ram + bd_rd_ptr);
			icssm_parse_packet_info(prueth, rd_buf_desc, &pkt_info);

			if (pkt_info.length <= 0) {
				/* a packet length of zero will cause us to
				 * never move the read pointer ahead, locking
				 * the driver, so we manually have to move it
				 * to the write pointer, discarding all
				 * remaining packets in this queue. This should
				 * never happen.
				 */
				update_rd_ptr = bd_wr_ptr;
				ndevstats->rx_length_errors++;
			} else if (pkt_info.length > EMAC_MAX_FRM_SUPPORT) {
				/* if the packet is too large we skip it but we
				 * still need to move the read pointer ahead
				 * and assume something is wrong with the read
				 * pointer as the firmware should be filtering
				 * these packets
				 */
				update_rd_ptr = bd_wr_ptr;
				ndevstats->rx_length_errors++;
			} else {
				update_rd_ptr = bd_rd_ptr;
				ret = icssm_emac_rx_packet(emac, &update_rd_ptr,
							   &pkt_info, rxqueue);
				if (ret)
					return IRQ_HANDLED;
				used++;
			}

			/* after reading the buffer descriptor we clear it
			 * to prevent improperly moved read pointer errors
			 * from simply looking like old packets.
			 */
			writel(0, shared_ram + bd_rd_ptr);

			/* update read pointer in queue descriptor */
			writew(update_rd_ptr, &queue_desc->rd_ptr);
			bd_rd_ptr = update_rd_ptr;

			/* all we have room for? */
			if (used >= quota)
				return used;
		}
	}

	return used;
}

static int icssm_emac_napi_poll(struct napi_struct *napi, int budget)
{
	struct prueth_emac *emac = container_of(napi, struct prueth_emac, napi);
	int num_rx_packets;

	num_rx_packets = icssm_emac_rx_packets(emac, budget);
	if (num_rx_packets < budget) {
		napi_complete_done(napi, num_rx_packets);
		enable_irq(emac->rx_irq);
	}

	return num_rx_packets;
}

static int icssm_emac_set_boot_pru(struct prueth_emac *emac,
				   struct net_device *ndev)
{
	const struct prueth_firmware *pru_firmwares;
	struct prueth *prueth = emac->prueth;
	const char *fw_name;
	int ret;

	pru_firmwares = &prueth->fw_data->fw_pru[emac->port_id - 1];
	fw_name = pru_firmwares->fw_name[prueth->eth_type];
	if (!fw_name) {
		netdev_err(ndev, "eth_type %d not supported\n",
			   prueth->eth_type);
		return -ENODEV;
	}

	ret = rproc_set_firmware(emac->pru, fw_name);
	if (ret) {
		netdev_err(ndev, "failed to set %s firmware: %d\n",
			   fw_name, ret);
		return ret;
	}

	ret = rproc_boot(emac->pru);
	if (ret) {
		netdev_err(ndev, "failed to boot %s firmware: %d\n",
			   fw_name, ret);
		return ret;
	}
	return ret;
}

static int icssm_emac_request_irqs(struct prueth_emac *emac)
{
	struct net_device *ndev = emac->ndev;
	int ret;

	ret = request_irq(emac->rx_irq, icssm_emac_rx_irq,
			  IRQF_TRIGGER_HIGH,
			  ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "unable to request RX IRQ\n");
		return ret;
	}

	if (emac->emac_ptp_tx_irq) {
		ret = request_threaded_irq(emac->emac_ptp_tx_irq,
					   icssm_prueth_ptp_tx_irq_handle,
					   icssm_prueth_ptp_tx_irq_work,
					   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					   ndev->name, ndev);
		if (ret) {
			netdev_err(ndev, "unable to request PTP TX IRQ\n");
			goto free_irq;
		}
	}

	return 0;

free_irq:
	free_irq(emac->rx_irq, ndev);
	return ret;
}

static int icssm_emac_sanitize_feature_flags(struct prueth_emac *emac)
{
	if ((PRUETH_IS_EMAC(emac->prueth) || PRUETH_IS_SWITCH(emac->prueth)) &&
	    (emac->ndev->features & NETIF_F_HW_HSR_TAG_RM)) {
		netdev_err(emac->ndev, "Error: Turn OFF HSR offload\n");
		return -EINVAL;
	}

	return 0;
}

/* Function to free memory related to sw/lre */
static void icssm_prueth_free_memory(struct prueth *prueth)
{
	if (PRUETH_IS_SWITCH(prueth))
		icssm_prueth_sw_free_fdb_table(prueth);
}

static void icssm_ptp_dram_init(struct prueth_emac *emac)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u64 temp64;

	writew(0, sram + MII_RX_CORRECTION_OFFSET);
	writew(0, sram + MII_TX_CORRECTION_OFFSET);

	/* Initialize RCF to 1 (Linux N/A) */
	writel(1 * 1024, sram + TIMESYNC_TC_RCF_OFFSET);

	/* This flag will be set and cleared by firmware */
	/* Write Sync0 period for sync signal generation in PTP
	 * memory in shared RAM
	 */
	writel(200000000 / 50, sram + TIMESYNC_SYNC0_WIDTH_OFFSET);

	/* Write CMP1 period for sync signal generation in PTP
	 * memory in shared RAM
	 */
	temp64 = 1000000;
	memcpy_toio(sram + TIMESYNC_CMP1_CMP_OFFSET, &temp64, sizeof(temp64));

	/* Write Sync0 period for sync signal generation in PTP
	 * memory in shared RAM
	 */
	writel(1000000, sram + TIMESYNC_CMP1_PERIOD_OFFSET);

	/* Configures domainNumber list. Firmware supports 2 domains */
	writeb(0, sram + TIMESYNC_DOMAIN_NUMBER_LIST);
	writeb(0, sram + TIMESYNC_DOMAIN_NUMBER_LIST + 1);

	/* Configure 1-step/2-step */
	writeb(1, sram + DISABLE_SWITCH_SYNC_RELAY_OFFSET);

	/* Configures the setting to Link local frame without HSR tag */
	writeb(0, sram + LINK_LOCAL_FRAME_HAS_HSR_TAG);

	/* Enable E2E/UDP PTP message timestamping */
	writeb(1, sram + PTP_IPV4_UDP_E2E_ENABLE);
}

/**
 * icssm_emac_ndo_open - EMAC device open
 * @ndev: network adapter device
 *
 * Called when system wants to start the interface.
 *
 * Return: 0 for a successful open, or appropriate error code
 */
static int icssm_emac_ndo_open(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap;
	int ret;

	ecap = prueth->ecap;
	/* set h/w MAC as user might have re-configured */
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	if (!prueth->emac_configured)
		icssm_prueth_init_ethernet_mode(prueth);

	ret = icssm_emac_sanitize_feature_flags(emac);
	if (ret)
		return ret;

	/* reset and start PRU firmware */
	if (!PRUETH_IS_EMAC(prueth)) {
		ret = icssm_prueth_sw_emac_config(emac);
		if (ret)
			return ret;

		if (PRUETH_IS_SWITCH(prueth))
			ret = icssm_prueth_sw_init_fdb_table(prueth);
	} else {
		icssm_prueth_emac_config(emac);
	}

	if (ret)
		return ret;

	/* restore stats */
	icssm_emac_set_stats(emac, &emac->stats);
	/* initialize ecap for interrupt pacing */
	if (!IS_ERR(ecap))
		ecap->init(emac);

	if (!prueth->emac_configured) {
		icssm_ptp_dram_init(emac);
		ret = icss_iep_init(prueth->iep, NULL, NULL, 0);
		if (ret) {
			netdev_err(ndev, "Failed to initialize iep: %d\n", ret);
			goto free_mem;
		}
	}

	if (!PRUETH_IS_EMAC(prueth)) {
		ret = icssm_prueth_sw_boot_prus(prueth, ndev);
		if (ret)
			goto iep_exit;
	} else {
		/* boot the PRU */
		ret = icssm_emac_set_boot_pru(emac, ndev);
		if (ret)
			goto iep_exit;
	}

	/* RSTP: Packet re-order
	 * Registering irq's for switch, irq's which are used for
	 * lre are used for switch as well.
	 */
	if (PRUETH_IS_EMAC(prueth))
		ret = icssm_emac_request_irqs(emac);
	else
		ret = icssm_prueth_common_request_irqs(emac);

	if (ret)
		goto rproc_shutdown;

	if (PRUETH_IS_EMAC(prueth)) {
		napi_enable(&emac->napi);
	} else {
		if (PRUETH_IS_SWITCH(prueth) && !prueth->emac_configured) {
			napi_enable(&prueth->napi_hpq);
			napi_enable(&prueth->napi_lpq);
		}
	}

	/* start PHY */
	phy_start(emac->phydev);

	/* enable the port and vlan */
	icssm_prueth_port_enable(emac, true);

	prueth->emac_configured |= BIT(emac->port_id);
	if (PRUETH_IS_SWITCH(prueth))
		prueth_sw_port_set_stp_state(prueth, emac->port_id,
					     BR_STATE_LEARNING);
	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "started\n");

	return 0;

rproc_shutdown:
	if (!PRUETH_IS_EMAC(prueth))
		icssm_prueth_sw_shutdown_prus(emac, ndev);
	else
		rproc_shutdown(emac->pru);

iep_exit:
	if (!prueth->emac_configured)
		icss_iep_exit(prueth->iep);
free_mem:
	icssm_prueth_free_memory(emac->prueth);
	return ret;
}

/**
 * icssm_emac_ndo_stop - EMAC device stop
 * @ndev: network adapter device
 *
 * Called when system wants to stop or down the interface.
 *
 * Return: Always 0 (Success)
 */
static int icssm_emac_ndo_stop(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int i;

	prueth->emac_configured &= ~BIT(emac->port_id);

	/* disable the mac port */
	icssm_prueth_port_enable(emac, false);

	/* stop PHY */
	phy_stop(emac->phydev);

	if (PRUETH_IS_EMAC(prueth)) {
		napi_disable(&emac->napi);
	} else {
		if (PRUETH_IS_SWITCH(prueth) && !prueth->emac_configured) {
			napi_disable(&prueth->napi_lpq);
			napi_disable(&prueth->napi_hpq);
		}
	}
	/* inform the upper layers. */
	netif_stop_queue(ndev);

	hrtimer_cancel(&emac->tx_hrtimer);

	/* stop the PRU */
	if (!PRUETH_IS_EMAC(prueth))
		icssm_prueth_sw_shutdown_prus(emac, ndev);
	else
		rproc_shutdown(emac->pru);

	icssm_emac_get_stats(emac, &emac->stats);

	/* free table memory of the switch */
	if (PRUETH_IS_SWITCH(emac->prueth))
		icssm_prueth_sw_free_fdb_table(prueth);

	/* Cleanup ptp related stuff for all protocols */
	icssm_prueth_ptp_tx_ts_enable(emac, 0);
	icssm_prueth_ptp_rx_ts_enable(emac, 0);
	for (i = 0; i < PRUETH_PTP_TS_EVENTS; i++) {
		if (emac->ptp_skb[i]) {
			icssm_prueth_ptp_tx_ts_reset(emac, i);
			dev_consume_skb_any(emac->ptp_skb[i]);
			emac->ptp_skb[i] = NULL;
		}
	}

	/* For EMAC, interrupt is per port.
	 * So free interrupts same way
	 */
	if (PRUETH_IS_EMAC(emac->prueth) || PRUETH_IS_SWITCH(prueth)) {
		if (emac->emac_ptp_tx_irq)
			free_irq(emac->emac_ptp_tx_irq, ndev);
	}

	if (PRUETH_IS_EMAC(emac->prueth)) {
		free_irq(emac->rx_irq, ndev);
	} else {
		/* Free interrupts on last port */
		icssm_prueth_common_free_irqs(emac);
	}

	/* free memory related to sw/lre */
	icssm_prueth_free_memory(emac->prueth);
	if (!prueth->emac_configured)
		icss_iep_exit(prueth->iep);

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "stopped\n");

	return 0;
}

static void icssm_prueth_change_to_switch_mode(struct prueth *prueth)
{
	bool portstatus[PRUETH_NUM_MACS];
	struct prueth_emac *emac;
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		portstatus[i] = netif_running(ndev);
		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_stop(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to stop: %d", ret);
			return;
		}
	}

	prueth->eth_type = PRUSS_ETHTYPE_SWITCH;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_open(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to start: %d", ret);
			return;
		}
	}

	dev_info(prueth->dev, "TI PRU ethernet now in Switch mode\n");
}

static void icssm_prueth_change_to_emac_mode(struct prueth *prueth)
{
	bool portstatus[PRUETH_NUM_MACS];
	struct prueth_emac *emac;
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		portstatus[i] = netif_running(ndev);
		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_stop(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to stop: %d", ret);
			return;
		}
	}

	prueth->eth_type = PRUSS_ETHTYPE_EMAC;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_open(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to start: %d", ret);
			return;
		}
	}

	dev_info(prueth->dev, "TI PRU ethernet now in Dual EMAC mode\n");
}

/* VLAN-tag PCP to priority queue map for EMAC/Switch/HSR/PRP used by driver
 * Index is PCP val / 2.
 *   low  - pcp 0..3 maps to Q4 for Host
 *   high - pcp 4..7 maps to Q3 for Host
 *   low  - pcp 0..3 maps to Q2 (FWD Queue) for PRU-x
 *   where x = 1 for PRUETH_PORT_MII0
 *             0 for PRUETH_PORT_MII1
 *   high - pcp 4..7 maps to Q1 (FWD Queue) for PRU-x
 */
static const unsigned short emac_pcp_tx_priority_queue_map[] = {
	PRUETH_QUEUE4, PRUETH_QUEUE4,
	PRUETH_QUEUE3, PRUETH_QUEUE3,
	PRUETH_QUEUE2, PRUETH_QUEUE2,
	PRUETH_QUEUE1, PRUETH_QUEUE1,
};

static u16 icssm_prueth_get_tx_queue_id(struct prueth *prueth,
					struct sk_buff *skb)
{
	u16 vlan_tci, pcp;
	int err;

	err = vlan_get_tag(skb, &vlan_tci);
	if (likely(err))
		pcp = 0;
	else
		pcp = (vlan_tci & VLAN_PRIO_MASK) >> VLAN_PRIO_SHIFT;

	/* Below code (pcp >>= 1) is made common for all
	 * protocols (i.e., EMAC, RSTP, HSR and PRP)*
	 * pcp value 0,1 will be updated to 0 mapped to QUEUE4
	 * pcp value 2,3 will be updated to 1 mapped to QUEUE4
	 * pcp value 4,5 will be updated to 2 mapped to QUEUE3
	 * pcp value 6,7 will be updated to 3 mapped to QUEUE3
	 */
	pcp >>= 1;

	return emac_pcp_tx_priority_queue_map[pcp];
}

/**
 * icssm_emac_ndo_start_xmit - EMAC Transmit function
 * @skb: SKB pointer
 * @ndev: EMAC network adapter
 *
 * Called by the system to transmit a packet - we queue the packet in
 * EMAC hardware transmit queue
 *
 * Return: enum netdev_tx
 */
static enum netdev_tx icssm_emac_ndo_start_xmit(struct sk_buff *skb,
						struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret;
	u16 qid;

	qid = icssm_prueth_get_tx_queue_id(emac->prueth, skb);
	ret = icssm_prueth_tx_enqueue(emac, skb, qid);
	if (ret) {
		if (ret != -ENOBUFS && netif_msg_tx_err(emac) &&
		    net_ratelimit())
			netdev_err(ndev, "packet queue failed: %d\n", ret);
		goto fail_tx;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;

fail_tx:
	if (ret == -ENOBUFS) {
		netif_stop_queue(ndev);
		hrtimer_start(&emac->tx_hrtimer,
			      ktime_set(0, HR_TIMER_TX_DELAY_US * 1000),
			      HRTIMER_MODE_REL_PINNED);
		ret = NETDEV_TX_BUSY;
	} else {
		/* error */
		ndev->stats.tx_dropped++;
		ret = NET_XMIT_DROP;
	}

	return ret;
}

/**
 * icssm_emac_ndo_tx_timeout - EMAC Transmit timeout function
 * @ndev: The EMAC network adapter
 * @txqueue: TX queue being used
 *
 * Called when system detects that a skb timeout period has expired
 * potentially due to a fault in the adapter in not being able to send
 * it out on the wire.
 */
static void icssm_emac_ndo_tx_timeout(struct net_device *ndev,
				      unsigned int txqueue)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (netif_msg_tx_err(emac))
		netdev_err(ndev, "xmit timeout");

	ndev->stats.tx_errors++;

	/* TODO: can we recover or need to reboot firmware? */

	netif_wake_queue(ndev);
}

/**
 * icssm_emac_ndo_get_stats64 - EMAC get statistics function
 * @ndev: The EMAC network adapter
 * @stats: rtnl_link_stats structure
 *
 * Called when system wants to get statistics from the device.
 *
 */
static void icssm_emac_ndo_get_stats64(struct net_device *ndev,
				       struct rtnl_link_stats64 *stats)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;

	icssm_emac_get_stats(emac, &pstats);

	stats->rx_packets = ndev->stats.rx_packets;
	stats->rx_bytes = ndev->stats.rx_bytes;
	stats->tx_packets = ndev->stats.tx_packets;
	stats->tx_bytes = ndev->stats.tx_bytes;
	stats->tx_errors = ndev->stats.tx_errors;
	stats->tx_dropped = ndev->stats.tx_dropped;
	stats->multicast = pstats.rx_mcast;

	stats->rx_over_errors = ndev->stats.rx_over_errors;
	stats->rx_length_errors = ndev->stats.rx_length_errors;
}

/* enable/disable MC filter */
static void icssm_emac_mc_filter_ctrl(struct prueth_emac *emac, bool enable)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *mc_filter_ctrl;
	void __iomem *ram;
	u32 mc_ctrl_byte;
	u32 reg;

	ram = prueth->mem[emac->dram].va;
	mc_ctrl_byte = prueth->fw_offsets->mc_ctrl_byte;
	mc_filter_ctrl = ram + mc_ctrl_byte;

	if (enable)
		reg = ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_ENABLED;
	else
		reg = ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_DISABLED;

	writeb(reg, mc_filter_ctrl);
}

/* reset MC filter bins */
static void icssm_emac_mc_filter_reset(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *mc_filter_tbl;
	u32 mc_filter_tbl_base;
	void __iomem *ram;

	ram = prueth->mem[emac->dram].va;
	mc_filter_tbl_base = prueth->fw_offsets->mc_filter_tbl;

	mc_filter_tbl = ram + mc_filter_tbl_base;
	memset_io(mc_filter_tbl, 0, ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES);
}

/* set MC filter hashmask */
static void icssm_emac_mc_filter_hashmask
		(struct prueth_emac *emac,
		 u8 mask[ICSS_EMAC_FW_MULTICAST_FILTER_MASK_SIZE_BYTES])
{
	struct prueth *prueth = emac->prueth;
	void __iomem *mc_filter_mask;
	u32 mc_filter_mask_base;
	void __iomem *ram;

	ram = prueth->mem[emac->dram].va;
	mc_filter_mask_base = prueth->fw_offsets->mc_filter_mask;

	mc_filter_mask = ram + mc_filter_mask_base;
	memcpy_toio(mc_filter_mask, mask,
		    ICSS_EMAC_FW_MULTICAST_FILTER_MASK_SIZE_BYTES);
}

static void icssm_emac_mc_filter_bin_update(struct prueth_emac *emac, u8 hash,
					    u8 val)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *mc_filter_tbl;
	u32 mc_filter_tbl_base;
	void __iomem *ram;

	ram = prueth->mem[emac->dram].va;
	mc_filter_tbl_base = prueth->fw_offsets->mc_filter_tbl;

	mc_filter_tbl = ram + mc_filter_tbl_base;
	writeb(val, mc_filter_tbl + hash);
}

void icssm_emac_mc_filter_bin_allow(struct prueth_emac *emac, u8 hash)
{
	icssm_emac_mc_filter_bin_update
		(emac, hash,
		 ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_ALLOWED);
}

void icssm_emac_mc_filter_bin_disallow(struct prueth_emac *emac, u8 hash)
{
	icssm_emac_mc_filter_bin_update
		(emac, hash,
		 ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_NOT_ALLOWED);
}

u8 icssm_emac_get_mc_hash(u8 *mac, u8 *mask)
{
	u8 hash;
	int j;

	for (j = 0, hash = 0; j < ETH_ALEN; j++)
		hash ^= (mac[j] & mask[j]);

	return hash;
}

/**
 * icssm_emac_ndo_set_rx_mode - EMAC set receive mode function
 * @ndev: The EMAC network adapter
 *
 * Called when system wants to set the receive mode of the device.
 *
 */
static void icssm_emac_ndo_set_rx_mode(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	bool promisc = ndev->flags & IFF_PROMISC;
	struct netdev_hw_addr *ha;
	struct prueth *prueth;
	unsigned long flags;
	void __iomem *sram;
	u32 mask, reg;
	u8 hash;

	prueth = emac->prueth;
	sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	reg = readl(sram + EMAC_PROMISCUOUS_MODE_OFFSET);

	/* for LRE, it is a shared table. So lock the access */
	spin_lock_irqsave(&emac->addr_lock, flags);

	/* Disable and reset multicast filter, allows allmulti */
	icssm_emac_mc_filter_ctrl(emac, false);
	icssm_emac_mc_filter_reset(emac);
	icssm_emac_mc_filter_hashmask(emac, emac->mc_filter_mask);

	if (PRUETH_IS_EMAC(prueth)) {
		switch (emac->port_id) {
		case PRUETH_PORT_MII0:
			mask = EMAC_P1_PROMISCUOUS_BIT;
			break;
		case PRUETH_PORT_MII1:
			mask = EMAC_P2_PROMISCUOUS_BIT;
			break;
		default:
			netdev_err(ndev, "%s: invalid port\n", __func__);
			goto unlock;
		}

		if (promisc) {
			/* Enable promiscuous mode */
			reg |= mask;
		} else {
			/* Disable promiscuous mode */
			reg &= ~mask;
		}

		writel(reg, sram + EMAC_PROMISCUOUS_MODE_OFFSET);

		if (promisc)
			goto unlock;
	}

	if (ndev->flags & IFF_ALLMULTI && !PRUETH_IS_SWITCH(prueth))
		goto unlock;

	icssm_emac_mc_filter_ctrl(emac, true);	/* all multicast blocked */

	if (netdev_mc_empty(ndev))
		goto unlock;

	netdev_for_each_mc_addr(ha, ndev) {
		hash = icssm_emac_get_mc_hash(ha->addr, emac->mc_filter_mask);
		icssm_emac_mc_filter_bin_allow(emac, hash);
	}

	/* Add bridge device's MC addresses as well */
	if (prueth->hw_bridge_dev) {
		netdev_for_each_mc_addr(ha, prueth->hw_bridge_dev) {
			hash = icssm_emac_get_mc_hash(ha->addr,
						      emac->mc_filter_mask);
			icssm_emac_mc_filter_bin_allow(emac, hash);
		}
	}

unlock:
	spin_unlock_irqrestore(&emac->addr_lock, flags);
}

static int icssm_emac_hwtstamp_config_set(struct net_device *ndev,
					  struct kernel_hwtstamp_config *cfg,
					  struct netlink_ext_ack *extack)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	/* reserved for future extensions */
	if (cfg->flags)
		return -EINVAL;

	if (cfg->tx_type != HWTSTAMP_TX_OFF && cfg->tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;

	switch (cfg->rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		icssm_prueth_ptp_rx_ts_enable(emac, 0);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		icssm_prueth_ptp_rx_ts_enable(emac, 1);
		cfg->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	default:
		return -ERANGE;
	}

	icssm_prueth_ptp_tx_ts_enable(emac, cfg->tx_type == HWTSTAMP_TX_ON);

	return 0;
}

static int icssm_emac_hwtstamp_config_get(struct net_device *ndev,
					  struct kernel_hwtstamp_config *cfg)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	cfg->flags = 0;
	cfg->tx_type = icssm_prueth_ptp_tx_ts_is_enabled(emac) ?
			HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;
	cfg->rx_filter = icssm_prueth_ptp_rx_ts_is_enabled(emac) ?
			HWTSTAMP_FILTER_PTP_V2_EVENT : HWTSTAMP_FILTER_NONE;

	return 0;
}

int icssm_emac_add_del_vid(struct prueth_emac *emac,
			   bool add, __be16 proto, u16 vid)
{
	struct prueth *prueth = emac->prueth;
	u32 vlan_filter_tbl;
	unsigned long flags;
	void __iomem *ram;
	u8 bit_index, val;
	u16 byte_index;

	vlan_filter_tbl = prueth->fw_offsets->vlan_filter_tbl;
	ram = prueth->mem[emac->dram].va;

	if (proto != htons(ETH_P_8021Q))
		return -EINVAL;

	if (vid >= ICSS_EMAC_FW_VLAN_FILTER_VID_MAX)
		return -EINVAL;

	/* By default, VLAN ID 0 (priority tagged packets) is routed to
	 * host, so nothing to be done if vid = 0
	 */
	if (!vid)
		return 0;

	/* for LRE, it is a shared table. So lock the access */
	spin_lock_irqsave(&emac->addr_lock, flags);

	/* VLAN filter table is 512 bytes (4096 bit) bitmap.
	 * Each bit controls enabling or disabling corresponding
	 * VID. Therefore byte index that controls a given VID is
	 * can calculated as vid / 8 and the bit within that byte
	 * that controls VID is given by vid % 8. Allow untagged
	 * frames to host by default.
	 */
	byte_index = vid / BITS_PER_BYTE;
	bit_index = vid % BITS_PER_BYTE;
	val = readb(ram + vlan_filter_tbl + byte_index);
	if (add)
		val |= BIT(bit_index);
	else
		val &= ~BIT(bit_index);
	writeb(val, ram + vlan_filter_tbl + byte_index);

	spin_unlock_irqrestore(&emac->addr_lock, flags);

	netdev_dbg(emac->ndev, "%s VID bit at byte index %d and bit %d\n",
		   add ? "Setting" : "Clearing", byte_index, bit_index);

	return 0;
}

static int icssm_emac_ndo_vlan_rx_add_vid(struct net_device *dev,
					  __be16 proto, u16 vid)
{
	struct prueth_emac *emac = netdev_priv(dev);

	return icssm_emac_add_del_vid(emac, true, proto, vid);
}

static int icssm_emac_ndo_vlan_rx_kill_vid(struct net_device *dev,
					   __be16 proto, u16 vid)
{
	struct prueth_emac *emac = netdev_priv(dev);

	return icssm_emac_add_del_vid(emac, false, proto, vid);
}

static int icssm_emac_get_port_parent_id(struct net_device *dev,
					 struct netdev_phys_item_id *ppid)
{
	struct prueth_emac *emac = netdev_priv(dev);
	struct prueth *prueth = emac->prueth;

	ppid->id_len = sizeof(prueth->base_mac);
	memcpy(&ppid->id, &prueth->base_mac, ppid->id_len);

	return 0;
}

static int icssm_emac_ndo_get_phys_port_name(struct net_device *ndev,
					     char *name, size_t len)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int err;

	err = snprintf(name, len, "p%d", emac->port_id);

	if (err >= len)
		return -EINVAL;

	return 0;
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = icssm_emac_ndo_open,
	.ndo_stop = icssm_emac_ndo_stop,
	.ndo_start_xmit = icssm_emac_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_tx_timeout = icssm_emac_ndo_tx_timeout,
	.ndo_get_stats64 = icssm_emac_ndo_get_stats64,
	.ndo_set_rx_mode = icssm_emac_ndo_set_rx_mode,
	.ndo_hwtstamp_get = icssm_emac_hwtstamp_config_get,
	.ndo_hwtstamp_set = icssm_emac_hwtstamp_config_set,
	.ndo_vlan_rx_add_vid = icssm_emac_ndo_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = icssm_emac_ndo_vlan_rx_kill_vid,
	.ndo_setup_tc = icssm_emac_ndo_setup_tc,
	.ndo_get_port_parent_id = icssm_emac_get_port_parent_id,
	.ndo_get_phys_port_name = icssm_emac_ndo_get_phys_port_name,
};

/* get emac_port corresponding to eth_node name */
static int icssm_prueth_node_port(struct device_node *eth_node)
{
	u32 port_id;
	int ret;

	ret = of_property_read_u32(eth_node, "reg", &port_id);
	if (ret)
		return ret;

	if (port_id == 0)
		return PRUETH_PORT_MII0;
	else if (port_id == 1)
		return PRUETH_PORT_MII1;
	else
		return PRUETH_PORT_INVALID;
}

/* get MAC instance corresponding to eth_node name */
static int icssm_prueth_node_mac(struct device_node *eth_node)
{
	u32 port_id;
	int ret;

	ret = of_property_read_u32(eth_node, "reg", &port_id);
	if (ret)
		return ret;

	if (port_id == 0)
		return PRUETH_MAC0;
	else if (port_id == 1)
		return PRUETH_MAC1;
	else
		return PRUETH_MAC_INVALID;
}

static enum hrtimer_restart icssm_emac_tx_timer_callback(struct hrtimer *timer)
{
	struct prueth_emac *emac =
			container_of(timer, struct prueth_emac, tx_hrtimer);

	if (netif_queue_stopped(emac->ndev))
		netif_wake_queue(emac->ndev);

	return HRTIMER_NORESTART;
}

static int icssm_prueth_netdev_init(struct prueth *prueth,
				    struct device_node *eth_node)
{
	const struct prueth_private_data *fw_data = prueth->fw_data;
	struct prueth_emac *emac;
	struct net_device *ndev;
	enum prueth_port port;
	enum prueth_mac mac;
	int ret;

	port = icssm_prueth_node_port(eth_node);
	if (port == PRUETH_PORT_INVALID)
		return -EINVAL;

	mac = icssm_prueth_node_mac(eth_node);
	if (mac == PRUETH_MAC_INVALID)
		return -EINVAL;

	ndev = devm_alloc_etherdev(prueth->dev, sizeof(*emac));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, prueth->dev);
	emac = netdev_priv(ndev);
	prueth->emac[mac] = emac;
	emac->prueth = prueth;
	emac->ndev = ndev;
	emac->port_id = port;
	memset(&emac->mc_filter_mask[0], 0xff, ETH_ALEN); /* default mask */

	/* by default eth_type is EMAC */
	switch (port) {
	case PRUETH_PORT_MII0:
		emac->tx_port_queue = PRUETH_PORT_QUEUE_MII0;

		/* packets from MII0 are on queues 1 through 2 */
		emac->rx_queue_start = PRUETH_QUEUE1;
		emac->rx_queue_end = PRUETH_QUEUE2;

		emac->dram = PRUETH_MEM_DRAM0;
		emac->pru = prueth->pru0;
		break;
	case PRUETH_PORT_MII1:
		emac->tx_port_queue = PRUETH_PORT_QUEUE_MII1;

		/* packets from MII1 are on queues 3 through 4 */
		emac->rx_queue_start = PRUETH_QUEUE3;
		emac->rx_queue_end = PRUETH_QUEUE4;

		emac->dram = PRUETH_MEM_DRAM1;
		emac->pru = prueth->pru1;
		break;
	default:
		return -EINVAL;
	}

	emac->rx_irq = of_irq_get_byname(eth_node, "rx");
	if (emac->rx_irq < 0) {
		ret = emac->rx_irq;
		if (ret != -EPROBE_DEFER)
			dev_err(prueth->dev, "could not get rx irq\n");
		goto free;
	}

	emac->emac_ptp_tx_irq = of_irq_get_byname(eth_node, "emac_ptp_tx");
	if (emac->emac_ptp_tx_irq < 0) {
		emac->emac_ptp_tx_irq = 0;
		dev_err(prueth->dev, "could not get ptp tx irq. Skipping PTP support\n");
	}

	spin_lock_init(&emac->lock);
	spin_lock_init(&emac->ptp_skb_lock);
	spin_lock_init(&emac->addr_lock);

	/* get mac address from DT and set private and netdev addr */
	ret = of_get_ethdev_address(eth_node, ndev);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(prueth->dev, "port %d: using random MAC addr: %pM\n",
			 port, ndev->dev_addr);
	}
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	/* connect PHY */
	emac->phydev = of_phy_get_and_connect(ndev, eth_node,
					      icssm_emac_adjust_link);
	if (!emac->phydev) {
		dev_dbg(prueth->dev, "PHY connection failed\n");
		ret = -EPROBE_DEFER;
		goto free;
	}

	/* remove unsupported modes */
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_10baseT_Full_BIT);

	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_10baseT_Half_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_100baseT_Half_BIT);

	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_Pause_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_Asym_Pause_BIT);

	/* Protocol switching
	 * Enabling L2 Firmware offloading
	 */
	if (fw_data->support_switch)
		ndev->features |= NETIF_F_HW_L2FW_DOFFLOAD;

	/*  Protocol switching
	 *  Enabling L2 Firmware offloading
	 */
	if (fw_data->support_switch)
		ndev->hw_features |= NETIF_F_HW_L2FW_DOFFLOAD;

	ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER | NETIF_F_HW_TC;

	ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;

	ndev->netdev_ops = &emac_netdev_ops;
	ndev->ethtool_ops = &emac_ethtool_ops;

	if (PRUETH_IS_EMAC(prueth))
		netif_napi_add(ndev, &emac->napi, icssm_emac_napi_poll);

	if (fw_data->support_switch && emac->port_id == PRUETH_PORT_MII0) {
		netif_napi_add(ndev, &prueth->napi_hpq,
			       icssm_prueth_lre_napi_poll_hpq);
		netif_napi_add(ndev, &prueth->napi_lpq,
			       icssm_prueth_lre_napi_poll_lpq);
	}

	hrtimer_init(&emac->tx_hrtimer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL_PINNED);
	emac->tx_hrtimer.function = icssm_emac_tx_timer_callback;

	if (fw_data->support_switch && emac->port_id == PRUETH_PORT_MII0) {
		prueth->hp->ndev = ndev;
		prueth->hp->priority = 0;
		prueth->lp->ndev = ndev;
		prueth->lp->priority = 1;
	}

	return 0;
free:
	emac->ndev = NULL;
	prueth->emac[mac] = NULL;

	return ret;
}

static void icssm_prueth_netdev_exit(struct prueth *prueth,
				     struct device_node *eth_node)
{
	struct prueth_emac *emac;
	enum prueth_mac mac;

	mac = icssm_prueth_node_mac(eth_node);
	if (mac == PRUETH_MAC_INVALID)
		return;

	emac = prueth->emac[mac];
	if (!emac)
		return;

	phy_disconnect(emac->phydev);

	if (PRUETH_IS_EMAC(prueth)) {
		netif_napi_del(&emac->napi);
	} else {
		if (PRUETH_IS_SWITCH(prueth) &&
		    emac->port_id == PRUETH_PORT_MII0) {
			netif_napi_del(&prueth->napi_hpq);
			netif_napi_del(&prueth->napi_lpq);
		}
	}

	prueth->emac[mac] = NULL;
}

bool icssm_prueth_sw_port_dev_check(const struct net_device *ndev)
{
	if (ndev->netdev_ops != &emac_netdev_ops)
		return false;

	if (ndev->features & NETIF_F_HW_L2FW_DOFFLOAD)
		return true;

	if (ndev->features & NETIF_F_HW_HSR_TAG_RM)
		return true;

	return false;
}

static void icssm_prueth_port_offload_fwd_mark_update(struct prueth *prueth)
{
	u8 all_slaves = BIT(PRUETH_PORT_MII0) | BIT(PRUETH_PORT_MII1);
	int set_val = 0;
	int i;

	if (prueth->br_members == all_slaves)
		set_val = 1;

	dev_dbg(prueth->dev, "set offload_fwd_mark %d, mbrs=0x%x\n",
		set_val, prueth->br_members);

	for (i = 0; i < PRUETH_NUM_MACS; i++)
		prueth->emac[i]->offload_fwd_mark = set_val;

	/* Bridge is created, load switch firmware, if not already in
	 * that mode
	 */
	if (set_val && !PRUETH_IS_SWITCH(prueth))
		icssm_prueth_change_to_switch_mode(prueth);

	/* Bridge is deleted, switch to Dual EMAC mode */
	if (!prueth->br_members && !PRUETH_IS_EMAC(prueth))
		icssm_prueth_change_to_emac_mode(prueth);
}

static int icssm_prueth_ndev_port_link(struct net_device *ndev,
				       struct net_device *br_ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	dev_dbg(prueth->dev, "%s: br_mbrs=0x%x %s\n",
		__func__, prueth->br_members, ndev->name);

	if (!prueth->br_members) {
		prueth->hw_bridge_dev = br_ndev;
	} else {
		/* This is adding the port to a second bridge, this is
		 * unsupported
		 */
		if (prueth->hw_bridge_dev != br_ndev)
			return -EOPNOTSUPP;
	}

	prueth->br_members |= BIT(emac->port_id);

	icssm_prueth_port_offload_fwd_mark_update(prueth);

	return NOTIFY_DONE;
}

static void icssm_prueth_ndev_port_unlink(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	dev_dbg(prueth->dev, "emac_sw_ndev_port_unlink\n");

	prueth->br_members &= ~BIT(emac->port_id);

	icssm_prueth_port_offload_fwd_mark_update(prueth);

	if (!prueth->br_members)
		prueth->hw_bridge_dev = NULL;
}

static int icssm_prueth_ndev_event(struct notifier_block *unused,
				   unsigned long event, void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	struct netdev_notifier_changeupper_info *info;
	int ret = NOTIFY_DONE;

	if (!icssm_prueth_sw_port_dev_check(ndev))
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_CHANGEUPPER:
		info = ptr;
		if (netif_is_bridge_master(info->upper_dev)) {
			if (info->linking)
				ret = icssm_prueth_ndev_port_link
					(ndev, info->upper_dev);
			else
				icssm_prueth_ndev_port_unlink(ndev);
		}
		break;
	default:
		return NOTIFY_DONE;
	}

	return notifier_from_errno(ret);
}

static int icssm_prueth_register_notifiers(struct prueth *prueth)
{
	int ret = 0;

	prueth->prueth_netdevice_nb.notifier_call = icssm_prueth_ndev_event;
	ret = register_netdevice_notifier(&prueth->prueth_netdevice_nb);
	if (ret) {
		dev_err(prueth->dev,
			"register netdevice notifier failed ret: %d\n", ret);
		return ret;
	}

	ret = icssm_prueth_sw_register_notifiers(prueth);
	if (ret)
		unregister_netdevice_notifier(&prueth->prueth_netdevice_nb);

	return ret;
}

static int icssm_prueth_probe(struct platform_device *pdev)
{
	struct device_node *eth0_node = NULL, *eth1_node = NULL;
	struct device_node *eth_node, *eth_ports_node;
	enum pruss_pru_id pruss_id0, pruss_id1;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct prueth *prueth;
	struct pruss *pruss;
	int i, ret;

	np = dev->of_node;
	if (!np)
		return -ENODEV; /* we don't support non DT */

	prueth = devm_kzalloc(dev, sizeof(*prueth), GFP_KERNEL);
	if (!prueth)
		return -ENOMEM;

	platform_set_drvdata(pdev, prueth);
	prueth->dev = dev;
	prueth->fw_data = device_get_match_data(dev);
	prueth->fw_offsets = &fw_offsets_v2_1;

	eth_ports_node = of_get_child_by_name(np, "ethernet-ports");
	if (!eth_ports_node)
		return -ENOENT;

	for_each_child_of_node(eth_ports_node, eth_node) {
		u32 reg;

		if (strcmp(eth_node->name, "ethernet-port"))
			continue;
		ret = of_property_read_u32(eth_node, "reg", &reg);
		if (ret < 0) {
			dev_err(dev, "%pOF error reading port_id %d\n",
				eth_node, ret);
			of_node_put(eth_node);
			return ret;
		}

		of_node_get(eth_node);

		if (reg == 0 && !eth0_node) {
			eth0_node = eth_node;
			if (!of_device_is_available(eth0_node)) {
				of_node_put(eth0_node);
				eth0_node = NULL;
			}
		} else if (reg == 1 && !eth1_node) {
			eth1_node = eth_node;
			if (!of_device_is_available(eth1_node)) {
				of_node_put(eth1_node);
				eth1_node = NULL;
			}
		} else {
			if (reg == 0 || reg == 1)
				dev_err(dev, "duplicate port reg value: %d\n",
					reg);
			else
				dev_err(dev, "invalid port reg value: %d\n",
					reg);

			of_node_put(eth_node);
		}
	}

	of_node_put(eth_ports_node);

	/* At least one node must be present and available else we fail */
	if (!eth0_node && !eth1_node) {
		dev_err(dev, "neither port0 nor port1 node available\n");
		return -ENODEV;
	}

	prueth->eth_node[PRUETH_MAC0] = eth0_node;
	prueth->eth_node[PRUETH_MAC1] = eth1_node;

	prueth->mii_rt = syscon_regmap_lookup_by_phandle(np, "ti,mii-rt");
	if (IS_ERR(prueth->mii_rt)) {
		dev_err(dev, "couldn't get mii-rt syscon regmap\n");
		return -ENODEV;
	}

	if (eth0_node) {
		prueth->pru0 = pru_rproc_get(np, 0, &pruss_id0);
		if (IS_ERR(prueth->pru0)) {
			ret = PTR_ERR(prueth->pru0);
			dev_err_probe(dev, ret, "unable to get PRU0");
			goto put_pru;
		}
	}

	if (eth1_node) {
		prueth->pru1 = pru_rproc_get(np, 1, &pruss_id1);
		if (IS_ERR(prueth->pru1)) {
			ret = PTR_ERR(prueth->pru1);
			dev_err_probe(dev, ret, "unable to get PRU1");
			goto put_pru;
		}
	}

	pruss = pruss_get(prueth->pru0 ? prueth->pru0 : prueth->pru1);
	if (IS_ERR(pruss)) {
		ret = PTR_ERR(pruss);
		dev_err(dev, "unable to get pruss handle\n");
		goto put_pru;
	}
	prueth->pruss = pruss;

	/* Configure PRUSS */
	if (eth0_node)
		pruss_cfg_gpimode(pruss, pruss_id0, PRUSS_GPI_MODE_MII);
	if (eth1_node)
		pruss_cfg_gpimode(pruss, pruss_id1, PRUSS_GPI_MODE_MII);
	pruss_cfg_miirt_enable(pruss, true);
	pruss_cfg_xfr_enable(pruss, PRU_TYPE_PRU, true);

	/* Get PRUSS mem resources */
	/* OCMC is system resource which we get separately */
	for (i = 0; i < ARRAY_SIZE(pruss_mem_ids); i++) {
		/* skip appropriate DRAM if not required */
		if (!eth0_node && i == PRUETH_MEM_DRAM0)
			continue;

		if (!eth1_node && i == PRUETH_MEM_DRAM1)
			continue;

		ret = pruss_request_mem_region(pruss, pruss_mem_ids[i],
					       &prueth->mem[i]);
		if (ret) {
			dev_err(dev, "unable to get PRUSS resource %d: %d\n",
				i, ret);
			goto put_mem;
		}
	}

	prueth->sram_pool = of_gen_pool_get(np, "sram", 0);
	if (!prueth->sram_pool) {
		dev_err(dev, "unable to get SRAM pool\n");
		ret = -ENODEV;
		goto put_mem;
	}

	prueth->ocmc_ram_size = OCMC_RAM_SIZE;
	/* Decreased by 8KB to address the reserved region for AM33x */
	if (prueth->fw_data->driver_data == PRUSS_AM33XX)
		prueth->ocmc_ram_size = (SZ_64K - SZ_8K);

	prueth->mem[PRUETH_MEM_OCMC].va =
			(void __iomem *)gen_pool_alloc(prueth->sram_pool,
						       prueth->ocmc_ram_size);
	if (!prueth->mem[PRUETH_MEM_OCMC].va) {
		dev_err(dev, "unable to allocate OCMC resource\n");
		ret = -ENOMEM;
		goto put_mem;
	}
	prueth->mem[PRUETH_MEM_OCMC].pa = gen_pool_virt_to_phys
		(prueth->sram_pool, (unsigned long)
		 prueth->mem[PRUETH_MEM_OCMC].va);
	prueth->mem[PRUETH_MEM_OCMC].size = prueth->ocmc_ram_size;
	dev_dbg(dev, "ocmc: pa %pa va %p size %#zx\n",
		&prueth->mem[PRUETH_MEM_OCMC].pa,
		prueth->mem[PRUETH_MEM_OCMC].va,
		prueth->mem[PRUETH_MEM_OCMC].size);

	/* RSTP: Packet re-order
	 * configuring the interrupts for switch
	 */
	if (prueth->fw_data->support_switch) {
		/* need to configure interrupts per queue common for
		 * both ports
		 */
		prueth->hp = devm_kzalloc(dev,
					  sizeof(struct prueth_ndev_priority),
					  GFP_KERNEL);
		if (!prueth->hp) {
			ret = -ENOMEM;
			goto free_pool;
		}
		prueth->lp = devm_kzalloc(dev,
					  sizeof(struct prueth_ndev_priority),
					  GFP_KERNEL);
		/* cit Feature - RSTP: Packet re-order
		 * configuring the interrupts for switch
		 */
		if (!prueth->lp) {
			ret = -ENOMEM;
			goto free_pool;
		}

		prueth->rx_lpq_irq = of_irq_get_byname(np, "rx_lp");
		if (prueth->rx_lpq_irq < 0) {
			ret = prueth->rx_lpq_irq;
			if (ret != -EPROBE_DEFER)
				dev_err(prueth->dev, "could not get rx_lp irq\n");
			goto free_pool;
		}
		prueth->rx_hpq_irq = of_irq_get_byname(np, "rx_hp");
		if (prueth->rx_hpq_irq < 0) {
			ret = prueth->rx_hpq_irq;
			if (ret != -EPROBE_DEFER)
				dev_err(prueth->dev, "could not get rx_hp irq\n");
			goto free_pool;
		}
	}
	/* setup netdev interfaces */
	if (eth0_node) {
		ret = icssm_prueth_netdev_init(prueth, eth0_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth0_node->name, ret);
			}
			goto free_pool;
		}
	}

	if (eth1_node) {
		ret = icssm_prueth_netdev_init(prueth, eth1_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth1_node->name, ret);
			}
			goto netdev_exit;
		}
	}

	prueth->iep = icss_iep_get(np);
	if (IS_ERR(prueth->iep)) {
		ret = PTR_ERR(prueth->iep);
		dev_err(dev, "unable to get IEP\n");
		goto netdev_exit;
	}

	if (prueth->fw_data->fw_rev == FW_REV_V1_0)
		prueth->iep->sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	/* Make rx interrupt pacing optional so that users can use ECAP for
	 * other use cases if needed
	 */
	prueth->ecap = icssm_prueth_ecap_get(np);
	if (IS_ERR(prueth->ecap)) {
		ret = PTR_ERR(prueth->ecap);
		if (ret != -EPROBE_DEFER)
			dev_info(dev,
				 "No ECAP. Rx interrupt pacing disabled\n");
		else
			goto iep_put;
	}

	/* register the network devices */
	if (eth0_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC0]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII0");
			goto ecap_put;
		}

		prueth->registered_netdevs[PRUETH_MAC0] =
			prueth->emac[PRUETH_MAC0]->ndev;
	}

	if (eth1_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC1]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII1");
			goto netdev_unregister;
		}

		prueth->registered_netdevs[PRUETH_MAC1] =
			prueth->emac[PRUETH_MAC1]->ndev;
	}

	ret = icssm_prueth_register_notifiers(prueth);
	if (ret) {
		dev_err(dev, "can't register switchdev notifiers");
		goto netdev_unregister;
	}

	eth_random_addr(prueth->base_mac);

	dev_info(dev, "TI PRU ethernet driver initialized: %s EMAC mode\n",
		 (!eth0_node || !eth1_node) ? "single" : "dual");

	if (eth1_node)
		of_node_put(eth1_node);
	if (eth0_node)
		of_node_put(eth0_node);
	return 0;

netdev_unregister:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

ecap_put:
	if (!IS_ERR(prueth->ecap))
		icssm_prueth_ecap_put(prueth->ecap);

iep_put:
	icss_iep_put(prueth->iep);
	prueth->iep = NULL;

netdev_exit:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		icssm_prueth_netdev_exit(prueth, eth_node);
	}

free_pool:
	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va,
		      prueth->ocmc_ram_size);

put_mem:
	for (i = PRUETH_MEM_DRAM0; i < PRUETH_MEM_OCMC; i++) {
		if (prueth->mem[i].va)
			pruss_release_mem_region(pruss, &prueth->mem[i]);
	}
	pruss_put(prueth->pruss);

put_pru:
	if (eth1_node) {
		if (prueth->pru1)
			pru_rproc_put(prueth->pru1);
		of_node_put(eth1_node);
	}

	if (eth0_node) {
		if (prueth->pru0)
			pru_rproc_put(prueth->pru0);
		of_node_put(eth0_node);
	}

	return ret;
}

static void icssm_prueth_remove(struct platform_device *pdev)
{
	struct prueth *prueth = platform_get_drvdata(pdev);
	struct device_node *eth_node;
	int i;

	unregister_netdevice_notifier(&prueth->prueth_netdevice_nb);
	icssm_prueth_sw_unregister_notifiers(prueth);

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		icssm_prueth_netdev_exit(prueth, eth_node);
		of_node_put(eth_node);
	}

	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va,
		      prueth->ocmc_ram_size);

	for (i = PRUETH_MEM_DRAM0; i < PRUETH_MEM_OCMC; i++) {
		if (prueth->mem[i].va)
			pruss_release_mem_region(prueth->pruss,
						 &prueth->mem[i]);
	}

	icss_iep_put(prueth->iep);
	prueth->iep = NULL;

	pruss_put(prueth->pruss);

	if (prueth->eth_node[PRUETH_MAC0])
		pru_rproc_put(prueth->pru0);
	if (prueth->eth_node[PRUETH_MAC1])
		pru_rproc_put(prueth->pru1);
}

#ifdef CONFIG_PM_SLEEP
static int icssm_prueth_suspend(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			ret = icssm_emac_ndo_stop(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to stop: %d", ret);
				return ret;
			}
		}
	}

	return 0;
}

static int icssm_prueth_resume(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			ret = icssm_emac_ndo_open(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to start: %d", ret);
				return ret;
			}
			netif_device_attach(ndev);
		}
	}

	return 0;
}

#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops prueth_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(icssm_prueth_suspend, icssm_prueth_resume)
};

/* AM335x SoC-specific firmware data */
static struct prueth_private_data am335x_prueth_pdata = {
	.driver_data = PRUSS_AM33XX,
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am335x-pru0-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am335x-pru0-prusw-fw.elf",
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am335x-pru1-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am335x-pru1-prusw-fw.elf",
	},
	.fw_rev = FW_REV_V1_0,
	.support_switch = true,
};

/* AM437x SoC-specific firmware data */
static struct prueth_private_data am437x_prueth_pdata = {
	.driver_data = PRUSS_AM43XX,
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am437x-pru0-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am437x-pru0-prusw-fw.elf",
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am437x-pru1-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am437x-pru1-prusw-fw.elf",
	},
	.fw_rev = FW_REV_V1_0,
	.support_switch = true,
};

/* AM57xx SoC-specific firmware data */
static struct prueth_private_data am57xx_prueth_pdata = {
	.driver_data = PRUSS_AM57XX,
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am57xx-pru0-prueth-fw.elf",
	.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am57xx-pru0-prusw-fw.elf",
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am57xx-pru1-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am57xx-pru1-prusw-fw.elf",

	},
	.fw_rev = FW_REV_V2_1,
	.support_switch = true,
};

static const struct of_device_id prueth_dt_match[] = {
	{ .compatible = "ti,am57-prueth", .data = &am57xx_prueth_pdata, },
	{ .compatible = "ti,am4376-prueth", .data = &am437x_prueth_pdata, },
	{ .compatible = "ti,am3359-prueth", .data = &am335x_prueth_pdata, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, prueth_dt_match);

static struct platform_driver prueth_driver = {
	.probe = icssm_prueth_probe,
	.remove = icssm_prueth_remove,
	.driver = {
		.name = "prueth",
		.of_match_table = prueth_dt_match,
		.pm = &prueth_dev_pm_ops,
	},
};
module_platform_driver(prueth_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("PRUSS ICSSM Ethernet Driver");
MODULE_LICENSE("GPL");
