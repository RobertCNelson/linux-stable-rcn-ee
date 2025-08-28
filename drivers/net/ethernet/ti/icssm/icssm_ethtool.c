// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSM Ethernet Driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include "icssm_prueth.h"
#include "icssm_prueth_ecap.h"
#include "icssm_prueth_lre.h"
#include "icssm_vlan_mcast_filter_mmap.h"
#include "../icssg/icss_iep.h"

/* set PRU firmware statistics */
void icssm_emac_set_stats(struct prueth_emac *emac,
			  struct port_statistics *pstats)
{
	void __iomem *dram;

	dram = emac->prueth->mem[emac->dram].va;
	memcpy_toio(dram + STATISTICS_OFFSET, pstats, STAT_SIZE);

	writel(pstats->vlan_dropped, dram +
			ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_OFFSET);
	writel(pstats->multicast_dropped, dram +
			ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET);
}

/* get statistics maintained by the PRU firmware into @pstats */
void icssm_emac_get_stats(struct prueth_emac *emac,
			  struct port_statistics *pstats)
{
	void __iomem *dram;

	dram = emac->prueth->mem[emac->dram].va;
	memcpy_fromio(pstats, dram + STATISTICS_OFFSET, STAT_SIZE);

	pstats->vlan_dropped =
		readl(dram + ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_OFFSET);
	pstats->multicast_dropped =
		readl(dram + ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET);
}

/**
 * icssm_emac_get_drvinfo - Get EMAC driver information
 * @ndev: The network adapter
 * @info: ethtool info structure containing name and version
 *
 * Returns EMAC driver information (name and version)
 */
static void icssm_emac_get_drvinfo(struct net_device *ndev,
				   struct ethtool_drvinfo *info)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	strscpy(info->driver, dev_driver_string(prueth->dev),
		sizeof(info->driver));
}

#define PRUETH_STAT_OFFSET(m, stats_type)       \
{                                               \
	#m,                                     \
	offsetof(struct port_statistics, m),    \
	stats_type                              \
}

struct icssm_prueth_stats {
	char string[ETH_GSTRING_LEN];
	u32 offset;
	bool standard_stats;
};

static const struct icssm_prueth_stats prueth_ethtool_stats[] = {
	PRUETH_STAT_OFFSET(tx_bcast, false),
	PRUETH_STAT_OFFSET(tx_mcast, false),
	PRUETH_STAT_OFFSET(tx_ucast, false),
	PRUETH_STAT_OFFSET(tx_octets, true),
	PRUETH_STAT_OFFSET(rx_bcast, false),
	PRUETH_STAT_OFFSET(rx_mcast, false),
	PRUETH_STAT_OFFSET(rx_ucast, false),
	PRUETH_STAT_OFFSET(rx_octets, true),

	PRUETH_STAT_OFFSET(tx64byte, true),
	PRUETH_STAT_OFFSET(tx65_127byte, true),
	PRUETH_STAT_OFFSET(tx128_255byte, true),
	PRUETH_STAT_OFFSET(tx256_511byte, true),
	PRUETH_STAT_OFFSET(tx512_1023byte, true),
	PRUETH_STAT_OFFSET(tx1024byte, true),
	PRUETH_STAT_OFFSET(rx64byte, true),
	PRUETH_STAT_OFFSET(rx65_127byte, true),
	PRUETH_STAT_OFFSET(rx128_255byte, true),
	PRUETH_STAT_OFFSET(rx256_511byte, true),
	PRUETH_STAT_OFFSET(rx512_1023byte, true),
	PRUETH_STAT_OFFSET(rx1024byte, true),

	PRUETH_STAT_OFFSET(late_coll, true),
	PRUETH_STAT_OFFSET(single_coll, true),
	PRUETH_STAT_OFFSET(multi_coll, true),
	PRUETH_STAT_OFFSET(excess_coll, true),

	PRUETH_STAT_OFFSET(rx_misalignment_frames, false),
	PRUETH_STAT_OFFSET(stormprev_counter_bc, false),
	PRUETH_STAT_OFFSET(stormprev_counter_mc, false),
	PRUETH_STAT_OFFSET(stormprev_counter_uc, false),
	PRUETH_STAT_OFFSET(mac_rxerror, false),
	PRUETH_STAT_OFFSET(sfd_error, false),
	PRUETH_STAT_OFFSET(def_tx, true),
	PRUETH_STAT_OFFSET(mac_txerror, false),
	PRUETH_STAT_OFFSET(rx_oversized_frames, false),
	PRUETH_STAT_OFFSET(rx_undersized_frames, false),
	PRUETH_STAT_OFFSET(rx_crc_frames, true),
	PRUETH_STAT_OFFSET(dropped_packets, false),

	PRUETH_STAT_OFFSET(tx_hwq_overflow, false),
	PRUETH_STAT_OFFSET(tx_hwq_underflow, false),
	PRUETH_STAT_OFFSET(vlan_dropped, true),
	PRUETH_STAT_OFFSET(multicast_dropped, true),
};

void icssm_emac_update_hardware_stats(struct prueth_emac *emac)
{
	void __iomem *dram, *stat_base;
	int i;

	dram = emac->prueth->mem[emac->dram].va;
	stat_base = dram + STATISTICS_OFFSET;

	for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++)
		emac->emac_stats[i] = ioread32(stat_base + i * sizeof(u32));
}

static int icssm_emac_get_sset_count(struct net_device *ndev, int stringset)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	switch (stringset) {
	case ETH_SS_STATS:
		return ICSSM_NUM_STANDARD_STATS +
			icssm_prueth_lre_get_sset_count(emac->prueth);
	default:
		return -EOPNOTSUPP;
	}
}

static void icssm_emac_get_strings(struct net_device *ndev, u32 stringset,
				   u8 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++) {
			if (!prueth_ethtool_stats[i].standard_stats)
				ethtool_puts(&p,
					     prueth_ethtool_stats[i].string);
		}
		icssm_prueth_lre_get_strings(emac->prueth, p);
		break;
	default:
		break;
	}
}

static void icssm_emac_get_ethtool_stats(struct net_device *ndev,
					 struct ethtool_stats *stats, u64 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int i;

	icssm_emac_update_hardware_stats(emac);

	for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++) {
		if (!prueth_ethtool_stats[i].standard_stats)
			*(data++) = emac->emac_stats[i];
	}
	icssm_prueth_lre_update_stats(emac, data);
}

static int icssm_emac_get_regs_len(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	/* VLAN Table at the end of the memory map, after MultiCast
	 * filter region. So VLAN table base +
	 * size will give the entire size of reg dump in case of
	 * Dual-EMAC firmware.
	 */
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		return ICSS_EMAC_FW_VLAN_FLTR_TBL_BASE_ADDR +
		       ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES;
	}

	/* MultiCast table and VLAN filter table are in different
	 * memories in case of HSR/PRP firmware. Therefore add the sizes
	 * of individual region.
	 */
	if (PRUETH_IS_LRE(prueth)) {
		return  ICSS_LRE_FW_VLAN_FLTR_TBL_BASE_ADDR +
			ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES +
			ICSS_LRE_FW_MULTICAST_FILTER_TABLE +
			ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES;
	}

	return 0;
}

static void icssm_emac_get_regs(struct net_device *ndev,
				struct ethtool_regs *regs, void *p)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	void __iomem *ram;
	u8 *reg = p;

	regs->version = PRUETH_REG_DUMP_GET_VER(prueth);

	/* Dump firmware's VLAN and MC tables */
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		ram = prueth->mem[emac->dram].va;
		memcpy_fromio(reg, ram, icssm_emac_get_regs_len(ndev));
		return;
	}

	if (PRUETH_IS_LRE(prueth)) {
		size_t len = ICSS_LRE_FW_VLAN_FLTR_TBL_BASE_ADDR +
			ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES;

		ram =  prueth->mem[PRUETH_MEM_SHARED_RAM].va;
		memcpy_fromio(reg, ram, len);

		reg += len;

		ram = prueth->mem[PRUETH_MEM_DRAM1].va;
		len = ICSS_LRE_FW_MULTICAST_FILTER_TABLE +
			ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES;
		memcpy_fromio(reg, ram, len);
	}
}

static const struct ethtool_rmon_hist_range icssm_emac_rmon_ranges[] = {
	{    0,   64},
	{   65,  127},
	{  128,  255},
	{  256,  511},
	{  512,  1023},
	{ 1024,  EMAC_MAX_PKTLEN},
	{}
};

static void
icssm_emac_get_rmon_stats(struct net_device *ndev,
			  struct ethtool_rmon_stats *rmon_stats,
			  const struct ethtool_rmon_hist_range **ranges)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;

	*ranges = icssm_emac_rmon_ranges;
	icssm_emac_get_stats(emac, &pstats);

	rmon_stats->undersize_pkts = pstats.rx_undersized_frames;
	rmon_stats->oversize_pkts = pstats.rx_oversized_frames;

	rmon_stats->hist[0] = pstats.tx64byte;
	rmon_stats->hist[1] = pstats.tx65_127byte;
	rmon_stats->hist[2] = pstats.tx128_255byte;
	rmon_stats->hist[3] = pstats.tx256_511byte;
	rmon_stats->hist[4] = pstats.tx512_1023byte;

	rmon_stats->hist_tx[0] = pstats.rx64byte;
	rmon_stats->hist_tx[1] = pstats.rx65_127byte;
	rmon_stats->hist_tx[2] = pstats.rx128_255byte;
	rmon_stats->hist_tx[3] = pstats.rx256_511byte;
	rmon_stats->hist_tx[4] = pstats.rx1024byte;
}

static void
icssm_emac_get_eth_mac_stats(struct net_device *ndev,
			     struct ethtool_eth_mac_stats *mac_stats)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;

	icssm_emac_get_stats(emac, &pstats);

	mac_stats->LateCollisions = pstats.late_coll;
	mac_stats->SingleCollisionFrames = pstats.single_coll;
	mac_stats->MultipleCollisionFrames = pstats.multi_coll;
}

static int icssm_emac_get_ts_info(struct net_device *ndev,
				  struct kernel_ethtool_ts_info *info)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if ((PRUETH_IS_EMAC(emac->prueth) && !emac->emac_ptp_tx_irq) ||
	    (PRUETH_IS_LRE(emac->prueth) && !emac->hsr_ptp_tx_irq))
		return ethtool_op_get_ts_info(ndev, info);

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = icss_iep_get_ptp_clock_idx(emac->prueth->iep);
	info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);
	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) |
				BIT(HWTSTAMP_FILTER_PTP_V2_EVENT);

	return 0;
}

static int icssm_emac_get_coalesce(struct net_device *ndev,
				   struct ethtool_coalesce *coal,
				   struct kernel_ethtool_coalesce *kernel_coal,
				   struct netlink_ext_ack *extack)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap;

	ecap = prueth->ecap;
	if (IS_ERR(ecap))
		return -EOPNOTSUPP;

	return ecap->get_coalesce(emac, &coal->use_adaptive_rx_coalesce,
				  &coal->rx_coalesce_usecs);
}

static int icssm_emac_set_coalesce(struct net_device *ndev,
				   struct ethtool_coalesce *coal,
				   struct kernel_ethtool_coalesce *kernel_coal,
				   struct netlink_ext_ack *extack)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap;

	ecap = prueth->ecap;
	if (IS_ERR(ecap))
		return -EOPNOTSUPP;

	return ecap->set_coalesce(emac, coal->use_adaptive_rx_coalesce,
				  coal->rx_coalesce_usecs);
}

/* Ethtool support for EMAC adapter */
const struct ethtool_ops emac_ethtool_ops = {
	.get_drvinfo = icssm_emac_get_drvinfo,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_link = ethtool_op_get_link,
	.get_sset_count = icssm_emac_get_sset_count,
	.get_strings = icssm_emac_get_strings,
	.get_ethtool_stats = icssm_emac_get_ethtool_stats,
	.get_regs_len = icssm_emac_get_regs_len,
	.get_regs = icssm_emac_get_regs,
	.get_rmon_stats = icssm_emac_get_rmon_stats,
	.get_eth_mac_stats = icssm_emac_get_eth_mac_stats,
	.get_ts_info = icssm_emac_get_ts_info,
	.supported_coalesce_params = ETHTOOL_COALESCE_RX_USECS,
	.get_coalesce = icssm_emac_get_coalesce,
	.set_coalesce = icssm_emac_set_coalesce,
};
EXPORT_SYMBOL_GPL(emac_ethtool_ops);
