// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 AM65 Ethernet Switch SubSystem Driver
 *
 * Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include <net/xsk_buff_pool.h>
#include <net/xdp_sock_drv.h>
#include "am65-cpsw-nuss.h"

static int am65_cpsw_xsk_pool_enable(struct am65_cpsw_port *port,
				     struct xsk_buff_pool *pool, u16 qid)
{
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_rx_chn *rx_chn;
	bool need_update;
	u32 frame_size;
	int ret;

	/*
	 * As queues are shared between ports we can no longer
	 * support the case where zero copy (XSK Pool) is enabled
	 * for the queue on one port but not for other ports.
	 *
	 * Current solution is to drop the packet if Zero copy
	 * is not enabled for that port + queue but enabled for
	 * some other port + same queue.
	 */
	if (test_bit(qid, common->xdp_zc_queues))
		return -EINVAL;

	rx_chn = &common->rx_chns;
	if (qid >= common->rx_ch_num_flows || qid >= common->tx_ch_num)
		return -EINVAL;

	frame_size = xsk_pool_get_rx_frame_size(pool);
	if (frame_size < AM65_CPSW_MAX_PACKET_SIZE)
		return -EOPNOTSUPP;

	ret = xsk_pool_dma_map(pool, rx_chn->dma_dev, AM65_CPSW_RX_DMA_ATTR);
	if (ret) {
		netdev_err(port->ndev, "Failed to map xsk pool\n");
		return ret;
	}

	need_update = common->usage_count &&
		      am65_cpsw_xdp_is_enabled(port);
	if (need_update) {
		am65_cpsw_destroy_rxq(common, qid, true);
		am65_cpsw_destroy_txq(common, qid);
	}

	set_bit(qid, common->xdp_zc_queues);
	common->xsk_port_id[qid] = port->port_id;
	if (need_update) {
		am65_cpsw_create_rxq(common, qid);
		am65_cpsw_create_txq(common, qid);
	}

	return 0;
}

static int am65_cpsw_xsk_pool_disable(struct am65_cpsw_port *port,
				      struct xsk_buff_pool *pool, u16 qid)
{
	struct am65_cpsw_common *common = port->common;
	bool need_update;

	if (qid >= common->rx_ch_num_flows || qid >= common->tx_ch_num)
		return -EINVAL;

	if (!test_bit(qid, common->xdp_zc_queues))
		return -EINVAL;

	pool = xsk_get_pool_from_qid(port->ndev, qid);
	if (!pool)
		return -EINVAL;

	need_update = common->usage_count && am65_cpsw_xdp_is_enabled(port);
	if (need_update) {
		am65_cpsw_destroy_rxq(common, qid, true);
		am65_cpsw_destroy_txq(common, qid);
		synchronize_rcu();
	}

	xsk_pool_dma_unmap(pool, AM65_CPSW_RX_DMA_ATTR);
	clear_bit(qid, common->xdp_zc_queues);
	common->xsk_port_id[qid] = -EINVAL;
	if (need_update) {
		am65_cpsw_create_rxq(common, qid);
		am65_cpsw_create_txq(common, qid);
	}

	return 0;
}

int am65_cpsw_xsk_setup_pool(struct net_device *ndev,
			     struct xsk_buff_pool *pool, u16 qid)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	return pool ? am65_cpsw_xsk_pool_enable(port, pool, qid) :
		      am65_cpsw_xsk_pool_disable(port, pool, qid);
}

int am65_cpsw_xsk_wakeup(struct net_device *ndev, u32 qid, u32 flags)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (!netif_running(ndev) || !netif_carrier_ok(ndev))
		return -ENETDOWN;

	if (!am65_cpsw_xdp_is_enabled(port))
		return -EINVAL;

	if (qid >= common->rx_ch_num_flows || qid >= common->tx_ch_num)
		return -EINVAL;

	return 0;
}
