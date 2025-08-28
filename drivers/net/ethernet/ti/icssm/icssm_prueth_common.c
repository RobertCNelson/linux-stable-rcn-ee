// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSM Ethernet Driver
 *
 * Copyright (C) 2018-2022 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/if_vlan.h>
#include <linux/spinlock_types.h>

#include "icssm_prueth.h"
#include "icssm_prueth_switch.h"

static int prueth_common_icssm_emac_rx_packets(struct prueth_emac *emac,
					       int quota, u8 qid1, u8 qid2)
{
	u16 bd_rd_ptr, bd_wr_ptr, update_rd_ptr, bd_rd_ptr_o, bd_wr_ptr_o;
	struct prueth_queue_desc __iomem *queue_desc, *queue_desc_o;
	struct net_device_stats *ndevstats = &emac->ndev->stats;
	int ret, used = 0, port, port0_q_empty, port1_q_empty;
	unsigned int emac_max_pktlen = EMAC_MAX_FRM_SUPPORT;
	const struct prueth_queue_info *rxqueue, *rxqueue_o;
	struct prueth_packet_info pkt_info, pkt_info_o;
	const struct prueth_queue_info *rxqueue_p;
	struct prueth_packet_info *pkt_info_p;
	struct net_device_stats *ndevstats_o;
	struct net_device_stats *ndevstats_p;
	struct prueth *prueth = emac->prueth;
	u8 overflow_cnt, overflow_cnt_o;
	u32 rd_buf_desc, rd_buf_desc_o;
	struct prueth_emac *other_emac;
	u16 *bd_rd_ptr_p, *bd_wr_ptr_p;
	struct prueth_emac *emac_p;
	void __iomem *shared_ram;
	u32 pkt_ts, pkt_ts_o;
	void *ocmc_ram;
	u32 iep_wrap;

	ocmc_ram = (__force void *)prueth->mem[PRUETH_MEM_OCMC].va;
	shared_ram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	other_emac = prueth->emac[(emac->port_id ^ 0x3) - 1];
	ndevstats_o = &other_emac->ndev->stats;

	/* use the correct wrap value based on ICSSM version */
	iep_wrap = prueth->fw_offsets->iep_wrap;
	/* search host queues for packets */
	queue_desc = emac->rx_queue_descs + qid1;
	queue_desc_o = other_emac->rx_queue_descs + qid2;

	rxqueue = &sw_queue_infos[PRUETH_PORT_HOST][qid1];
	rxqueue_o = &sw_queue_infos[PRUETH_PORT_HOST][qid2];

	overflow_cnt = readb(&queue_desc->overflow_cnt);
	overflow_cnt_o = readb(&queue_desc_o->overflow_cnt);

	if (overflow_cnt > 0) {
		emac->ndev->stats.rx_over_errors += overflow_cnt;
		/* reset to zero */
		writeb(0, &queue_desc->overflow_cnt);
	}
	if (overflow_cnt_o > 0) {
		other_emac->ndev->stats.rx_over_errors += overflow_cnt_o;

		/* reset to zero */
		writeb(0, &queue_desc_o->overflow_cnt);
	}

	bd_rd_ptr = readw(&queue_desc->rd_ptr);
	bd_wr_ptr = readw(&queue_desc->wr_ptr);

	bd_rd_ptr_o = readw(&queue_desc_o->rd_ptr);
	bd_wr_ptr_o = readw(&queue_desc_o->wr_ptr);

	port0_q_empty = (bd_rd_ptr == bd_wr_ptr) ? 1 : 0;
	port1_q_empty = (bd_rd_ptr_o == bd_wr_ptr_o) ? 1 : 0;

	/* while packets are available in this queue */
	while (!port0_q_empty || !port1_q_empty) {
		/* get packet info from the read buffer descriptor */
		rd_buf_desc = readl(shared_ram + bd_rd_ptr);
		rd_buf_desc_o = readl(shared_ram + bd_rd_ptr_o);

		icssm_parse_packet_info(prueth, rd_buf_desc, &pkt_info);
		icssm_parse_packet_info(prueth, rd_buf_desc_o, &pkt_info_o);

		pkt_ts = readl((void __iomem *)(ocmc_ram +
			       ICSS_COMMON_TIMESTAMP_ARRAY_OFFSET +
			       bd_rd_ptr - SRAM_START_OFFSET));
		pkt_ts_o = readl((void __iomem *)(ocmc_ram +
				 ICSS_COMMON_TIMESTAMP_ARRAY_OFFSET +
				 bd_rd_ptr_o - SRAM_START_OFFSET));

		if (!port0_q_empty && !port1_q_empty) {
			/* Packets in both port queues */
			/* Calculate diff b/n timestamps and account for
			 * wraparound
			 */
			if (pkt_ts > pkt_ts_o)
				port = (pkt_ts - pkt_ts_o) > (iep_wrap / 2) ?
					0 : 1;
			else
				port = (pkt_ts_o - pkt_ts) > (iep_wrap / 2) ?
					1 : 0;

		} else if (!port0_q_empty) {
			/* Packet(s) in port0 queue only */
			port = 0;
		} else {
			/* Packet(s) in port1 queue only */
			port = 1;
		}

		/* Select correct data structures for queue/packet selected */
		if (port == 0) {
			pkt_info_p = &pkt_info;
			bd_wr_ptr_p = &bd_wr_ptr;
			bd_rd_ptr_p = &bd_rd_ptr;
			emac_p = emac;
			ndevstats_p = ndevstats;
			rxqueue_p = rxqueue;
		} else {
			pkt_info_p = &pkt_info_o;
			bd_wr_ptr_p = &bd_wr_ptr_o;
			bd_rd_ptr_p = &bd_rd_ptr_o;
			emac_p = other_emac;
			ndevstats_p = ndevstats_o;
			rxqueue_p = rxqueue_o;
		}

		if ((*pkt_info_p).length <= 0) {
			/* a packet length of zero will cause us to
			 * never move the read pointer ahead, locking
			 * the driver, so we manually have to move it
			 * to the write pointer, discarding all
			 * remaining packets in this queue. This should
			 * never happen.
			 */
			update_rd_ptr = *bd_wr_ptr_p;
			ndevstats_p->rx_length_errors++;
		} else if ((*pkt_info_p).length > emac_max_pktlen) {
			/* if the packet is too large we skip it but we
			 * still need to move the read pointer ahead
			 * and assume something is wrong with the read
			 * pointer as the firmware should be filtering
			 * these packets
			 */
			update_rd_ptr = *bd_wr_ptr_p;
			ndevstats_p->rx_length_errors++;
		} else {
			update_rd_ptr = *bd_rd_ptr_p;
			ret = icssm_emac_rx_packet(emac_p, &update_rd_ptr,
						   pkt_info_p, rxqueue_p);
			if (ret)
				return IRQ_HANDLED;

			used++;
		}

		/* after reading the buffer descriptor we clear it
		 * to prevent improperly moved read pointer errors
		 * from simply looking like old packets.
		 */

		/* update read pointer in queue descriptor */
		if (port == 0) {
			writel(0, shared_ram + bd_rd_ptr);
			writew(update_rd_ptr, &queue_desc->rd_ptr);
			bd_rd_ptr = update_rd_ptr;
		} else {
			writel(0, shared_ram + bd_rd_ptr_o);
			writew(update_rd_ptr, &queue_desc_o->rd_ptr);
			bd_rd_ptr_o = update_rd_ptr;
		}

		port0_q_empty = (bd_rd_ptr == bd_wr_ptr) ? 1 : 0;
		port1_q_empty = (bd_rd_ptr_o == bd_wr_ptr_o) ? 1 : 0;

		if (used >= quota)
			return used;
	}

	return used;
}

int icssm_prueth_lre_napi_poll_lpq(struct napi_struct *napi, int budget)
{
	struct prueth *prueth = container_of(napi, struct prueth, napi_lpq);
	struct net_device *ndev = prueth->lp->ndev;
	struct prueth_emac *emac = netdev_priv(ndev);
	u8 qid1 = PRUETH_QUEUE2, qid2 = PRUETH_QUEUE4;
	int num_rx_packets;

	num_rx_packets = prueth_common_icssm_emac_rx_packets(emac, budget,
							     qid1, qid2);
	if (num_rx_packets < budget) {
		napi_complete_done(napi, num_rx_packets);
		enable_irq(prueth->rx_lpq_irq);
	}

	return num_rx_packets;
}

int icssm_prueth_lre_napi_poll_hpq(struct napi_struct *napi, int budget)
{
	struct prueth *prueth = container_of(napi, struct prueth, napi_hpq);
	struct net_device *ndev = prueth->hp->ndev;
	struct prueth_emac *emac = netdev_priv(ndev);
	u8 qid1 = PRUETH_QUEUE1, qid2 = PRUETH_QUEUE3;
	int num_rx_packets;

	num_rx_packets = prueth_common_icssm_emac_rx_packets(emac, budget,
							     qid1, qid2);
	if (num_rx_packets < budget) {
		napi_complete_done(napi, num_rx_packets);
		enable_irq(prueth->rx_hpq_irq);
	}

	return num_rx_packets;
}

static irqreturn_t icssm_prueth_common_emac_rx_hardirq(int irq, void *dev_id)
{
	struct prueth_ndev_priority *ndev_prio =
		(struct prueth_ndev_priority *)dev_id;
	struct net_device *ndev = ndev_prio->ndev;
	struct prueth_emac *emac;
	struct prueth *prueth;

	emac = netdev_priv(ndev);
	prueth = emac->prueth;

	/* disable Rx system event */
	if (ndev_prio->priority == 1) {
		disable_irq_nosync(prueth->rx_lpq_irq);
		napi_schedule(&prueth->napi_lpq);
	} else {
		disable_irq_nosync(prueth->rx_hpq_irq);
		napi_schedule(&prueth->napi_hpq);
	}

	return IRQ_HANDLED;
}

int icssm_prueth_common_request_irqs(struct prueth_emac *emac)
{
	struct net_device *ndev = emac->ndev;
	struct prueth *prueth = emac->prueth;
	int ret;

	/* Feature - RSTP: Packet re-order
	 * Requesting the ptp tx irq's for switch
	 */
	if (PRUETH_IS_SWITCH(prueth)) {
		if (emac->emac_ptp_tx_irq) {
			ret = request_threaded_irq
				(emac->emac_ptp_tx_irq,
				 icssm_prueth_ptp_tx_irq_handle,
				 icssm_prueth_ptp_tx_irq_work,
				 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				 ndev->name, ndev);
			if (ret) {
				netdev_err(ndev, "unable to request PTP TX IRQ\n");
				return ret;
			}
		}
	}

	if (PRUETH_IS_LRE(prueth)) {
		if (emac->hsr_ptp_tx_irq) {
			ret = request_threaded_irq
				(emac->hsr_ptp_tx_irq,
				 icssm_prueth_ptp_tx_irq_handle,
				 icssm_prueth_ptp_tx_irq_work,
				 IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
				 ndev->name, ndev);
			if (ret) {
				netdev_err(ndev, "unable to request PTP TX IRQ\n");
				return ret;
			}
		}
	}

	/* HSR/PRP. Request irq when first port is initialized */
	if (prueth->emac_configured)
		return 0;

	ret = request_irq(prueth->rx_hpq_irq,
			  icssm_prueth_common_emac_rx_hardirq,
			  IRQF_TRIGGER_HIGH, "eth_hp_int", prueth->hp);
	if (ret) {
		netdev_err(emac->ndev, "unable to request RX HPQ IRQ\n");
		goto free_ptp_irq;
	}

	ret = request_irq(prueth->rx_lpq_irq,
			  icssm_prueth_common_emac_rx_hardirq,
			  IRQF_TRIGGER_HIGH, "eth_lp_int", prueth->lp);
	if (ret) {
		netdev_err(emac->ndev, "unable to request RX LPQ IRQ\n");
		goto free_rx_hpq_irq;
	}

	return 0;

free_rx_hpq_irq:
	free_irq(prueth->rx_hpq_irq, prueth->hp);

	/* RSTP: Packet re-order
	 * free the ptp tx irq's for switch
	 */
free_ptp_irq:
	if (PRUETH_IS_SWITCH(prueth)) {
		if (emac->emac_ptp_tx_irq)
			free_irq(emac->emac_ptp_tx_irq, emac->ndev);
	}

	if (PRUETH_IS_LRE(prueth)) {
		if (emac->hsr_ptp_tx_irq)
			free_irq(emac->hsr_ptp_tx_irq, emac->ndev);
	}
	return ret;
}

/**
 * icssm_prueth_common_free_irqs - free irq
 *
 * @emac: EMAC data structure
 *
 */
void icssm_prueth_common_free_irqs(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;

	if (PRUETH_IS_LRE(prueth)) {
		if (emac->hsr_ptp_tx_irq)
			free_irq(emac->hsr_ptp_tx_irq, emac->ndev);
	}

	/* HSR/PRP: free irqs when last port is down */
	if (prueth->emac_configured)
		return;

	free_irq(prueth->rx_lpq_irq, prueth->lp);
	free_irq(prueth->rx_hpq_irq, prueth->hp);
}
