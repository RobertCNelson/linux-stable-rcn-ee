// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 AM65 Ethernet QoS submodule
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * quality of service module includes:
 * Enhanced Scheduler Traffic (EST - P802.1Qbv/D2.2)
 * Interspersed Express Traffic (IET - P802.3br/D2.0)
 */

#include <linux/bitfield.h>
#include <linux/pm_runtime.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/time.h>
#include <linux/units.h>
#include <net/pkt_cls.h>

#include "am65-cpsw-nuss.h"
#include "am65-cpsw-qos.h"
#include "am65-cpts.h"
#include "cpsw_ale.h"

#define AM65_CPSW_REG_FREQ			0x05c

#define AM64_CPSW_PN_CUT_THRU			0x3C0
#define AM64_CPSW_PN_SPEED			0x3C4

/* AM65_CPSW_REG_CTL register fields */
#define AM64_CPSW_CTL_CUT_THRU_EN		BIT(19)

/* Cut-Thru AM64_CPSW_PN_CUT_THRU */
#define  AM64_PN_CUT_THRU_TX_PRI		GENMASK(7, 0)
#define  AM64_PN_CUT_THRU_RX_PRI		GENMASK(15, 8)

/* Cut-Thru AM64_CPSW_PN_SPEED */
#define  AM64_PN_SPEED_VAL			GENMASK(3, 0)
#define  AM64_PN_SPEED_AUTO_EN			BIT(8)
#define  AM64_PN_AUTO_SPEED			GENMASK(15, 12)

#define TO_MBPS(x)	DIV_ROUND_UP((x), BYTES_PER_MBIT)

enum timer_act {
	TACT_PROG,		/* need program timer */
	TACT_NEED_STOP,		/* need stop first */
	TACT_SKIP_PROG,		/* just buffer can be updated */
};

/* number of traffic classes (fifos) per port */
#define AM65_CPSW_PN_TC_NUM			8

static void am65_cpsw_iet_change_preemptible_tcs(struct am65_cpsw_port *port, u8 preemptible_tcs);

static u32
am65_cpsw_qos_tx_rate_calc(u32 rate_mbps, unsigned long bus_freq)
{
	u32 ir;

	bus_freq /= 1000000;
	ir = DIV_ROUND_UP(((u64)rate_mbps * 32768),  bus_freq);
	return ir;
}

static void am65_cpsw_tx_pn_shaper_reset(struct am65_cpsw_port *port)
{
	int prio;

	for (prio = 0; prio < AM65_CPSW_PN_FIFO_PRIO_NUM; prio++) {
		writel(0, port->port_base + AM65_CPSW_PN_REG_PRI_CIR(prio));
		writel(0, port->port_base + AM65_CPSW_PN_REG_PRI_EIR(prio));
	}
}

static void am65_cpsw_tx_pn_shaper_apply(struct am65_cpsw_port *port)
{
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	struct am65_cpsw_common *common = port->common;
	struct tc_mqprio_qopt_offload *mqprio;
	bool enable, shaper_susp = false;
	u32 rate_mbps;
	int tc, prio;

	mqprio = &p_mqprio->mqprio_hw;
	/* takes care of no link case as well */
	if (p_mqprio->max_rate_total > port->qos.link_speed)
		shaper_susp = true;

	am65_cpsw_tx_pn_shaper_reset(port);

	enable = p_mqprio->shaper_en && !shaper_susp;
	if (!enable)
		return;

	/* Rate limit is specified per Traffic Class but
	 * for CPSW, rate limit can be applied per priority
	 * at port FIFO.
	 *
	 * We have assigned the same priority (TCn) to all queues
	 * of a Traffic Class so they share the same shaper
	 * bandwidth.
	 */
	for (tc = 0; tc < mqprio->qopt.num_tc; tc++) {
		prio = tc;

		rate_mbps = TO_MBPS(mqprio->min_rate[tc]);
		rate_mbps = am65_cpsw_qos_tx_rate_calc(rate_mbps,
						       common->bus_freq);
		writel(rate_mbps,
		       port->port_base + AM65_CPSW_PN_REG_PRI_CIR(prio));

		rate_mbps = 0;

		if (mqprio->max_rate[tc]) {
			rate_mbps = mqprio->max_rate[tc] - mqprio->min_rate[tc];
			rate_mbps = TO_MBPS(rate_mbps);
			rate_mbps = am65_cpsw_qos_tx_rate_calc(rate_mbps,
							       common->bus_freq);
		}

		writel(rate_mbps,
		       port->port_base + AM65_CPSW_PN_REG_PRI_EIR(prio));
	}
}

static int am65_cpsw_mqprio_verify_shaper(struct am65_cpsw_port *port,
					  struct tc_mqprio_qopt_offload *mqprio)
{
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	struct netlink_ext_ack *extack = mqprio->extack;
	u64 min_rate_total = 0, max_rate_total = 0;
	u32 min_rate_msk = 0, max_rate_msk = 0;
	bool has_min_rate, has_max_rate;
	int num_tc, i;

	if (!(mqprio->flags & TC_MQPRIO_F_SHAPER))
		return 0;

	if (mqprio->shaper != TC_MQPRIO_SHAPER_BW_RATE)
		return 0;

	has_min_rate = !!(mqprio->flags & TC_MQPRIO_F_MIN_RATE);
	has_max_rate = !!(mqprio->flags & TC_MQPRIO_F_MAX_RATE);

	if (!has_min_rate && has_max_rate) {
		NL_SET_ERR_MSG_MOD(extack, "min_rate is required with max_rate");
		return -EOPNOTSUPP;
	}

	if (!has_min_rate)
		return 0;

	num_tc = mqprio->qopt.num_tc;

	for (i = num_tc - 1; i >= 0; i--) {
		u32 ch_msk;

		if (mqprio->min_rate[i])
			min_rate_msk |= BIT(i);
		min_rate_total +=  mqprio->min_rate[i];

		if (has_max_rate) {
			if (mqprio->max_rate[i])
				max_rate_msk |= BIT(i);
			max_rate_total +=  mqprio->max_rate[i];

			if (!mqprio->min_rate[i] && mqprio->max_rate[i]) {
				NL_SET_ERR_MSG_FMT_MOD(extack,
						       "TX tc%d rate max>0 but min=0",
						       i);
				return -EINVAL;
			}

			if (mqprio->max_rate[i] &&
			    mqprio->max_rate[i] < mqprio->min_rate[i]) {
				NL_SET_ERR_MSG_FMT_MOD(extack,
						       "TX tc%d rate min(%llu)>max(%llu)",
						       i, mqprio->min_rate[i],
						       mqprio->max_rate[i]);
				return -EINVAL;
			}
		}

		ch_msk = GENMASK(num_tc - 1, i);
		if ((min_rate_msk & BIT(i)) && (min_rate_msk ^ ch_msk)) {
			NL_SET_ERR_MSG_FMT_MOD(extack,
					       "Min rate must be set sequentially hi->lo tx_rate_msk%x",
					       min_rate_msk);
			return -EINVAL;
		}

		if ((max_rate_msk & BIT(i)) && (max_rate_msk ^ ch_msk)) {
			NL_SET_ERR_MSG_FMT_MOD(extack,
					       "Max rate must be set sequentially hi->lo tx_rate_msk%x",
					       max_rate_msk);
			return -EINVAL;
		}
	}

	min_rate_total = TO_MBPS(min_rate_total);
	max_rate_total = TO_MBPS(max_rate_total);

	p_mqprio->shaper_en = true;
	p_mqprio->max_rate_total = max_t(u64, min_rate_total, max_rate_total);

	return 0;
}

static void am65_cpsw_reset_tc_mqprio(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;

	p_mqprio->shaper_en = false;
	p_mqprio->max_rate_total = 0;

	am65_cpsw_tx_pn_shaper_reset(port);
	netdev_reset_tc(ndev);

	/* Reset all Queue priorities to 0 */
	writel(0, port->port_base + AM65_CPSW_PN_REG_TX_PRI_MAP);

	am65_cpsw_iet_change_preemptible_tcs(port, 0);
}

static int am65_cpsw_setup_mqprio(struct net_device *ndev, void *type_data)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_mqprio *p_mqprio = &port->qos.mqprio;
	struct tc_mqprio_qopt_offload *mqprio = type_data;
	struct am65_cpsw_common *common = port->common;
	struct tc_mqprio_qopt *qopt = &mqprio->qopt;
	int i, tc, offset, count, prio, ret;
	u8 num_tc = qopt->num_tc;
	u32 tx_prio_map = 0;

	memcpy(&p_mqprio->mqprio_hw, mqprio, sizeof(*mqprio));

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	if (!num_tc) {
		am65_cpsw_reset_tc_mqprio(ndev);
		ret = 0;
		goto exit_put;
	}

	ret = am65_cpsw_mqprio_verify_shaper(port, mqprio);
	if (ret)
		goto exit_put;

	netdev_set_num_tc(ndev, num_tc);

	/* Multiple Linux priorities can map to a Traffic Class
	 * A Traffic Class can have multiple contiguous Queues,
	 * Queues get mapped to Channels (thread_id),
	 *	if not VLAN tagged, thread_id is used as packet_priority
	 *	if VLAN tagged. VLAN priority is used as packet_priority
	 * packet_priority gets mapped to header_priority in p0_rx_pri_map,
	 * header_priority gets mapped to switch_priority in pn_tx_pri_map.
	 * As p0_rx_pri_map is left at defaults (0x76543210), we can
	 * assume that Queue_n gets mapped to header_priority_n. We can then
	 * set the switch priority in pn_tx_pri_map.
	 */

	for (tc = 0; tc < num_tc; tc++) {
		prio = tc;

		/* For simplicity we assign the same priority (TCn) to
		 * all queues of a Traffic Class.
		 */
		for (i = qopt->offset[tc]; i < qopt->offset[tc] + qopt->count[tc]; i++)
			tx_prio_map |= prio << (4 * i);

		count = qopt->count[tc];
		offset = qopt->offset[tc];
		netdev_set_tc_queue(ndev, tc, count, offset);
	}

	writel(tx_prio_map, port->port_base + AM65_CPSW_PN_REG_TX_PRI_MAP);

	am65_cpsw_tx_pn_shaper_apply(port);
	am65_cpsw_iet_change_preemptible_tcs(port, mqprio->preemptible_tcs);

exit_put:
	pm_runtime_put(common->dev);

	return ret;
}

static int am65_cpsw_iet_set_verify_timeout_count(struct am65_cpsw_port *port)
{
	int verify_time_ms = port->qos.iet.verify_time_ms;
	u32 val;

	/* The number of wireside clocks contained in the verify
	 * timeout counter. The default is 0x1312d0
	 * (10ms at 125Mhz in 1G mode).
	 */
	val = 125 * HZ_PER_MHZ;	/* assuming 125MHz wireside clock */

	val /= MILLIHZ_PER_HZ;		/* count per ms timeout */
	val *= verify_time_ms;		/* count for timeout ms */

	if (val > AM65_CPSW_PN_MAC_VERIFY_CNT_MASK)
		return -EINVAL;

	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_VERIFY);

	return 0;
}

static int am65_cpsw_iet_verify_wait(struct am65_cpsw_port *port)
{
	u32 ctrl, status;
	int try;

	try = 20;
	do {
		/* Reset the verify state machine by writing 1
		 * to LINKFAIL
		 */
		ctrl = readl(port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
		ctrl |= AM65_CPSW_PN_IET_MAC_LINKFAIL;
		writel(ctrl, port->port_base + AM65_CPSW_PN_REG_IET_CTRL);

		/* Clear MAC_LINKFAIL bit to start Verify. */
		ctrl = readl(port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
		ctrl &= ~AM65_CPSW_PN_IET_MAC_LINKFAIL;
		writel(ctrl, port->port_base + AM65_CPSW_PN_REG_IET_CTRL);

		msleep(port->qos.iet.verify_time_ms);

		status = readl(port->port_base + AM65_CPSW_PN_REG_IET_STATUS);
		if (status & AM65_CPSW_PN_MAC_VERIFIED)
			return 0;

		if (status & AM65_CPSW_PN_MAC_VERIFY_FAIL) {
			netdev_dbg(port->ndev,
				   "MAC Merge verify failed, trying again\n");
			continue;
		}

		if (status & AM65_CPSW_PN_MAC_RESPOND_ERR) {
			netdev_dbg(port->ndev, "MAC Merge respond error\n");
			return -ENODEV;
		}

		if (status & AM65_CPSW_PN_MAC_VERIFY_ERR) {
			netdev_dbg(port->ndev, "MAC Merge verify error\n");
			return -ENODEV;
		}
	} while (try-- > 0);

	netdev_dbg(port->ndev, "MAC Merge verify timeout\n");
	return -ETIMEDOUT;
}

static void am65_cpsw_iet_set_preempt_mask(struct am65_cpsw_port *port, u8 preemptible_tcs)
{
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
	val &= ~AM65_CPSW_PN_IET_MAC_PREMPT_MASK;
	val |= AM65_CPSW_PN_IET_MAC_SET_PREEMPT(preemptible_tcs);
	writel(val, port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
}

/* enable common IET_ENABLE only if at least 1 port has rx IET enabled.
 * UAPI doesn't allow tx enable without rx enable.
 */
void am65_cpsw_iet_common_enable(struct am65_cpsw_common *common)
{
	struct am65_cpsw_port *port;
	bool rx_enable = false;
	u32 val;
	int i;

	for (i = 0; i < common->port_num; i++) {
		port = &common->ports[i];
		val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
		rx_enable = !!(val & AM65_CPSW_PN_CTL_IET_PORT_EN);
		if (rx_enable)
			break;
	}

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);

	if (rx_enable)
		val |= AM65_CPSW_CTL_IET_EN;
	else
		val &= ~AM65_CPSW_CTL_IET_EN;

	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->iet_enabled = rx_enable;
}

/* CPSW does not have an IRQ to notify changes to the MAC Merge TX status
 * (active/inactive), but the preemptible traffic classes should only be
 * committed to hardware once TX is active. Resort to polling.
 */
void am65_cpsw_iet_commit_preemptible_tcs(struct am65_cpsw_port *port)
{
	u8 preemptible_tcs;
	int err;
	u32 val;

	if (port->qos.link_speed == SPEED_UNKNOWN)
		return;

	val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
	if (!(val & AM65_CPSW_PN_CTL_IET_PORT_EN))
		return;

	/* update common IET enable */
	am65_cpsw_iet_common_enable(port->common);

	/* update verify count */
	err = am65_cpsw_iet_set_verify_timeout_count(port);
	if (err) {
		netdev_err(port->ndev, "couldn't set verify count: %d\n", err);
		return;
	}

	val = readl(port->port_base + AM65_CPSW_PN_REG_IET_CTRL);
	if (!(val & AM65_CPSW_PN_IET_MAC_DISABLEVERIFY)) {
		err = am65_cpsw_iet_verify_wait(port);
		if (err)
			return;
	}

	preemptible_tcs = port->qos.iet.preemptible_tcs;
	am65_cpsw_iet_set_preempt_mask(port, preemptible_tcs);
}

static void am65_cpsw_iet_change_preemptible_tcs(struct am65_cpsw_port *port, u8 preemptible_tcs)
{
	struct am65_cpsw_ndev_priv *priv = am65_ndev_to_priv(port->ndev);

	port->qos.iet.preemptible_tcs = preemptible_tcs;
	mutex_lock(&priv->mm_lock);
	am65_cpsw_iet_commit_preemptible_tcs(port);
	mutex_unlock(&priv->mm_lock);
}

static void am65_cpsw_iet_link_state_update(struct net_device *ndev)
{
	struct am65_cpsw_ndev_priv *priv = am65_ndev_to_priv(ndev);
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	mutex_lock(&priv->mm_lock);
	am65_cpsw_iet_commit_preemptible_tcs(port);
	mutex_unlock(&priv->mm_lock);
}

static int am65_cpsw_port_est_enabled(struct am65_cpsw_port *port)
{
	return port->qos.est_oper || port->qos.est_admin;
}

static void am65_cpsw_est_enable(struct am65_cpsw_common *common, int enable)
{
	u32 val;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);

	if (enable)
		val |= AM65_CPSW_CTL_EST_EN;
	else
		val &= ~AM65_CPSW_CTL_EST_EN;

	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->est_enabled = enable;
}

static void am65_cpsw_port_est_enable(struct am65_cpsw_port *port, int enable)
{
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_CTL);
	if (enable)
		val |= AM65_CPSW_PN_CTL_EST_PORT_EN;
	else
		val &= ~AM65_CPSW_PN_CTL_EST_PORT_EN;

	writel(val, port->port_base + AM65_CPSW_PN_REG_CTL);
}

/* target new EST RAM buffer, actual toggle happens after cycle completion */
static void am65_cpsw_port_est_assign_buf_num(struct net_device *ndev,
					      int buf_num)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	if (buf_num)
		val |= AM65_CPSW_PN_EST_BUFSEL;
	else
		val &= ~AM65_CPSW_PN_EST_BUFSEL;

	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);
}

/* am65_cpsw_port_est_is_swapped() - Indicate if h/w is transitioned
 * admin -> oper or not
 *
 * Return true if already transitioned. i.e oper is equal to admin and buf
 * numbers match (est_oper->buf match with est_admin->buf).
 * false if before transition. i.e oper is not equal to admin, (i.e a
 * previous admin command is waiting to be transitioned to oper state
 * and est_oper->buf not match with est_oper->buf).
 */
static int am65_cpsw_port_est_is_swapped(struct net_device *ndev, int *oper,
					 int *admin)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_FIFO_STATUS);
	*oper = !!(val & AM65_CPSW_PN_FST_EST_BUFACT);

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	*admin = !!(val & AM65_CPSW_PN_EST_BUFSEL);

	return *admin == *oper;
}

/* am65_cpsw_port_est_get_free_buf_num() - Get free buffer number for
 * Admin to program the new schedule.
 *
 * Logic as follows:-
 * If oper is same as admin, return the other buffer (!oper) as the admin
 * buffer.  If oper is not the same, driver let the current oper to continue
 * as it is in the process of transitioning from admin -> oper. So keep the
 * oper by selecting the same oper buffer by writing to EST_BUFSEL bit in
 * EST CTL register. In the second iteration they will match and code returns.
 * The actual buffer to write command is selected later before it is ready
 * to update the schedule.
 */
static int am65_cpsw_port_est_get_free_buf_num(struct net_device *ndev)
{
	int oper, admin;
	int roll = 2;

	while (roll--) {
		if (am65_cpsw_port_est_is_swapped(ndev, &oper, &admin))
			return !oper;

		/* admin is not set, so hinder transition as it's not allowed
		 * to touch memory in-flight, by targeting same oper buf.
		 */
		am65_cpsw_port_est_assign_buf_num(ndev, oper);

		dev_info(&ndev->dev,
			 "Prev. EST admin cycle is in transit %d -> %d\n",
			 oper, admin);
	}

	return admin;
}

static void am65_cpsw_admin_to_oper(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	devm_kfree(&ndev->dev, port->qos.est_oper);

	port->qos.est_oper = port->qos.est_admin;
	port->qos.est_admin = NULL;
}

static void am65_cpsw_port_est_get_buf_num(struct net_device *ndev,
					   struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 val;

	val = readl(port->port_base + AM65_CPSW_PN_REG_EST_CTL);
	val &= ~AM65_CPSW_PN_EST_ONEBUF;
	writel(val, port->port_base + AM65_CPSW_PN_REG_EST_CTL);

	est_new->buf = am65_cpsw_port_est_get_free_buf_num(ndev);

	/* rolled buf num means changed buf while configuring */
	if (port->qos.est_oper && port->qos.est_admin &&
	    est_new->buf == port->qos.est_oper->buf)
		am65_cpsw_admin_to_oper(ndev);
}

static void am65_cpsw_est_set(struct net_device *ndev, int enable)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	int common_enable = 0;
	int i;

	am65_cpsw_port_est_enable(port, enable);

	for (i = 0; i < common->port_num; i++)
		common_enable |= am65_cpsw_port_est_enabled(&common->ports[i]);

	common_enable |= enable;
	am65_cpsw_est_enable(common, common_enable);
}

/* This update is supposed to be used in any routine before getting real state
 * of admin -> oper transition, particularly it's supposed to be used in some
 * generic routine for providing real state to Taprio Qdisc.
 */
static void am65_cpsw_est_update_state(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int oper, admin;

	if (!port->qos.est_admin)
		return;

	if (!am65_cpsw_port_est_is_swapped(ndev, &oper, &admin))
		return;

	am65_cpsw_admin_to_oper(ndev);
}

/* Fetch command count it's number of bytes in Gigabit mode or nibbles in
 * 10/100Mb mode. So, having speed and time in ns, recalculate ns to number of
 * bytes/nibbles that can be sent while transmission on given speed.
 */
static int am65_est_cmd_ns_to_cnt(u64 ns, int link_speed)
{
	u64 temp;

	temp = ns * link_speed;
	if (link_speed < SPEED_1000)
		temp <<= 1;

	return DIV_ROUND_UP(temp, 8 * 1000);
}

static void __iomem *am65_cpsw_est_set_sched_cmds(void __iomem *addr,
						  int fetch_cnt,
						  int fetch_allow)
{
	u32 prio_mask, cmd_fetch_cnt, cmd;

	do {
		if (fetch_cnt > AM65_CPSW_FETCH_CNT_MAX) {
			fetch_cnt -= AM65_CPSW_FETCH_CNT_MAX;
			cmd_fetch_cnt = AM65_CPSW_FETCH_CNT_MAX;
		} else {
			cmd_fetch_cnt = fetch_cnt;
			/* fetch count can't be less than 16? */
			if (cmd_fetch_cnt && cmd_fetch_cnt < 16)
				cmd_fetch_cnt = 16;

			fetch_cnt = 0;
		}

		prio_mask = fetch_allow & AM65_CPSW_FETCH_ALLOW_MSK;
		cmd = (cmd_fetch_cnt << AM65_CPSW_FETCH_CNT_OFFSET) | prio_mask;

		writel(cmd, addr);
		addr += 4;
	} while (fetch_cnt);

	return addr;
}

static int am65_cpsw_est_calc_cmd_num(struct net_device *ndev,
				      struct tc_taprio_qopt_offload *taprio,
				      int link_speed)
{
	int i, cmd_cnt, cmd_sum = 0;
	u32 fetch_cnt;

	for (i = 0; i < taprio->num_entries; i++) {
		if (taprio->entries[i].command != TC_TAPRIO_CMD_SET_GATES) {
			dev_err(&ndev->dev, "Only SET command is supported");
			return -EINVAL;
		}

		fetch_cnt = am65_est_cmd_ns_to_cnt(taprio->entries[i].interval,
						   link_speed);

		cmd_cnt = DIV_ROUND_UP(fetch_cnt, AM65_CPSW_FETCH_CNT_MAX);
		if (!cmd_cnt)
			cmd_cnt++;

		cmd_sum += cmd_cnt;

		if (!fetch_cnt)
			break;
	}

	return cmd_sum;
}

static int am65_cpsw_est_check_scheds(struct net_device *ndev,
				      struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	int cmd_num;

	cmd_num = am65_cpsw_est_calc_cmd_num(ndev, &est_new->taprio,
					     port->qos.link_speed);
	if (cmd_num < 0)
		return cmd_num;

	if (cmd_num > AM65_CPSW_FETCH_RAM_CMD_NUM / 2) {
		dev_err(&ndev->dev, "No fetch RAM");
		return -ENOMEM;
	}

	return 0;
}

static void am65_cpsw_est_set_sched_list(struct net_device *ndev,
					 struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	u32 fetch_cnt, fetch_allow, all_fetch_allow = 0;
	void __iomem *ram_addr, *max_ram_addr;
	struct tc_taprio_sched_entry *entry;
	int i, ram_size;

	ram_addr = port->fetch_ram_base;
	ram_size = AM65_CPSW_FETCH_RAM_CMD_NUM * 2;
	ram_addr += est_new->buf * ram_size;

	max_ram_addr = ram_size + ram_addr;
	for (i = 0; i < est_new->taprio.num_entries; i++) {
		entry = &est_new->taprio.entries[i];

		fetch_cnt = am65_est_cmd_ns_to_cnt(entry->interval,
						   port->qos.link_speed);
		fetch_allow = entry->gate_mask;
		if (fetch_allow > AM65_CPSW_FETCH_ALLOW_MAX)
			dev_dbg(&ndev->dev, "fetch_allow > 8 bits: %d\n",
				fetch_allow);

		ram_addr = am65_cpsw_est_set_sched_cmds(ram_addr, fetch_cnt,
							fetch_allow);

		if (!fetch_cnt && i < est_new->taprio.num_entries - 1) {
			dev_info(&ndev->dev,
				 "next scheds after %d have no impact", i + 1);
			break;
		}

		all_fetch_allow |= fetch_allow;
	}

	/* end cmd, enabling non-timed queues for potential over cycle time */
	if (ram_addr < max_ram_addr)
		writel(~all_fetch_allow & AM65_CPSW_FETCH_ALLOW_MSK, ram_addr);
}

/*
 * Enable ESTf periodic output, set cycle start time and interval.
 */
static int am65_cpsw_timer_set(struct net_device *ndev,
			       struct am65_cpsw_est *est_new)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpts *cpts = common->cpts;
	struct am65_cpts_estf_cfg cfg;

	cfg.ns_period = est_new->taprio.cycle_time;
	cfg.ns_start = est_new->taprio.base_time;

	return am65_cpts_estf_enable(cpts, port->port_id - 1, &cfg);
}

static void am65_cpsw_timer_stop(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = port->common->cpts;

	am65_cpts_estf_disable(cpts, port->port_id - 1);
}

static enum timer_act am65_cpsw_timer_act(struct net_device *ndev,
					  struct am65_cpsw_est *est_new)
{
	struct tc_taprio_qopt_offload *taprio_oper, *taprio_new;
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = port->common->cpts;
	u64 cur_time;
	s64 diff;

	if (!port->qos.est_oper)
		return TACT_PROG;

	taprio_new = &est_new->taprio;
	taprio_oper = &port->qos.est_oper->taprio;

	if (taprio_new->cycle_time != taprio_oper->cycle_time)
		return TACT_NEED_STOP;

	/* in order to avoid timer reset get base_time form oper taprio */
	if (!taprio_new->base_time && taprio_oper)
		taprio_new->base_time = taprio_oper->base_time;

	if (taprio_new->base_time == taprio_oper->base_time)
		return TACT_SKIP_PROG;

	/* base times are cycle synchronized */
	diff = taprio_new->base_time - taprio_oper->base_time;
	diff = diff < 0 ? -diff : diff;
	if (diff % taprio_new->cycle_time)
		return TACT_NEED_STOP;

	cur_time = am65_cpts_ns_gettime(cpts);
	if (taprio_new->base_time <= cur_time + taprio_new->cycle_time)
		return TACT_SKIP_PROG;

	/* TODO: Admin schedule at future time is not currently supported */
	return TACT_NEED_STOP;
}

static void am65_cpsw_stop_est(struct net_device *ndev)
{
	am65_cpsw_est_set(ndev, 0);
	am65_cpsw_timer_stop(ndev);
}

static void am65_cpsw_taprio_destroy(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	am65_cpsw_stop_est(ndev);

	devm_kfree(&ndev->dev, port->qos.est_admin);
	devm_kfree(&ndev->dev, port->qos.est_oper);

	port->qos.est_oper = NULL;
	port->qos.est_admin = NULL;

	am65_cpsw_reset_tc_mqprio(ndev);
}

static void am65_cpsw_cp_taprio(struct tc_taprio_qopt_offload *from,
				struct tc_taprio_qopt_offload *to)
{
	int i;

	*to = *from;
	for (i = 0; i < from->num_entries; i++)
		to->entries[i] = from->entries[i];
}

static int am65_cpsw_taprio_replace(struct net_device *ndev,
				    struct tc_taprio_qopt_offload *taprio)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct netlink_ext_ack *extack = taprio->mqprio.extack;
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpts *cpts = common->cpts;
	struct am65_cpsw_est *est_new;
	u64 cur_time, n;
	int ret, tact;

	if (!netif_running(ndev)) {
		NL_SET_ERR_MSG_MOD(extack, "interface is down, link speed unknown");
		return -ENETDOWN;
	}

	if (common->pf_p0_rx_ptype_rrobin) {
		NL_SET_ERR_MSG_MOD(extack,
				   "p0-rx-ptype-rrobin flag conflicts with taprio qdisc");
		return -EINVAL;
	}

	if (port->qos.link_speed == SPEED_UNKNOWN)
		return -ENOLINK;

	if (taprio->cycle_time_extension) {
		NL_SET_ERR_MSG_MOD(extack,
				   "cycle time extension not supported");
		return -EOPNOTSUPP;
	}

	est_new = devm_kzalloc(&ndev->dev,
			       struct_size(est_new, taprio.entries, taprio->num_entries),
			       GFP_KERNEL);
	if (!est_new)
		return -ENOMEM;

	ret = am65_cpsw_setup_mqprio(ndev, &taprio->mqprio);
	if (ret)
		return ret;

	am65_cpsw_cp_taprio(taprio, &est_new->taprio);

	am65_cpsw_est_update_state(ndev);

	ret = am65_cpsw_est_check_scheds(ndev, est_new);
	if (ret < 0)
		goto fail;

	tact = am65_cpsw_timer_act(ndev, est_new);
	if (tact == TACT_NEED_STOP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Can't toggle estf timer, stop taprio first");
		ret = -EINVAL;
		goto fail;
	}

	if (tact == TACT_PROG)
		am65_cpsw_timer_stop(ndev);

	am65_cpsw_port_est_get_buf_num(ndev, est_new);
	am65_cpsw_est_set_sched_list(ndev, est_new);
	am65_cpsw_port_est_assign_buf_num(ndev, est_new->buf);

	/* If the base-time is in the past, start schedule from the time:
	 * base_time + (N*cycle_time)
	 * where N is the smallest possible integer such that the above
	 * time is in the future.
	 */
	cur_time = am65_cpts_ns_gettime(cpts);
	if (est_new->taprio.base_time < cur_time) {
		n = div64_u64(cur_time - est_new->taprio.base_time, est_new->taprio.cycle_time);
		est_new->taprio.base_time += (n + 1) * est_new->taprio.cycle_time;
	}

	am65_cpsw_est_set(ndev, 1);

	if (tact == TACT_PROG) {
		ret = am65_cpsw_timer_set(ndev, est_new);
		if (ret) {
			NL_SET_ERR_MSG_MOD(extack,
					   "Failed to set cycle time");
			goto fail;
		}
	}

	devm_kfree(&ndev->dev, port->qos.est_admin);
	port->qos.est_admin = est_new;
	am65_cpsw_iet_change_preemptible_tcs(port, taprio->mqprio.preemptible_tcs);

	return 0;

fail:
	am65_cpsw_reset_tc_mqprio(ndev);
	devm_kfree(&ndev->dev, est_new);
	return ret;
}

static void am65_cpsw_est_link_up(struct net_device *ndev, int link_speed)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	ktime_t cur_time;
	s64 delta;

	if (!am65_cpsw_port_est_enabled(port))
		return;

	if (port->qos.link_down_time) {
		cur_time = ktime_get();
		delta = ktime_us_delta(cur_time, port->qos.link_down_time);
		if (delta > USEC_PER_SEC) {
			dev_err(&ndev->dev,
				"Link has been lost too long, stopping TAS");
			goto purge_est;
		}
	}

	return;

purge_est:
	am65_cpsw_taprio_destroy(ndev);
}

static int am65_cpsw_setup_taprio(struct net_device *ndev, void *type_data)
{
	struct tc_taprio_qopt_offload *taprio = type_data;
	int err = 0;

	switch (taprio->cmd) {
	case TAPRIO_CMD_REPLACE:
		err = am65_cpsw_taprio_replace(ndev, taprio);
		break;
	case TAPRIO_CMD_DESTROY:
		am65_cpsw_taprio_destroy(ndev);
		break;
	default:
		err = -EOPNOTSUPP;
	}

	return err;
}

static int am65_cpsw_tc_query_caps(struct net_device *ndev, void *type_data)
{
	struct tc_query_caps_base *base = type_data;

	switch (base->type) {
	case TC_SETUP_QDISC_MQPRIO: {
		struct tc_mqprio_caps *caps = base->caps;

		caps->validate_queue_counts = true;

		return 0;
	}

	case TC_SETUP_QDISC_TAPRIO: {
		struct tc_taprio_caps *caps = base->caps;

		caps->gate_mask_per_txq = true;

		return 0;
	}
	default:
		return -EOPNOTSUPP;
	}
}

static int am65_cpsw_qos_clsflower_add_policer(struct am65_cpsw_port *port,
					       struct netlink_ext_ack *extack,
					       struct flow_cls_offload *cls,
					       u64 rate_pkt_ps)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct flow_dissector *dissector = rule->match.dissector;
	static const u8 mc_mac[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
	struct am65_cpsw_qos *qos = &port->qos;
	struct flow_match_eth_addrs match;
	int ret;

	if (dissector->used_keys &
	    ~(BIT_ULL(FLOW_DISSECTOR_KEY_BASIC) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_CONTROL) |
	      BIT_ULL(FLOW_DISSECTOR_KEY_ETH_ADDRS))) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Unsupported keys used");
		return -EOPNOTSUPP;
	}

	if (flow_rule_match_has_control_flags(rule, extack))
		return -EOPNOTSUPP;

	if (!flow_rule_match_key(rule, FLOW_DISSECTOR_KEY_ETH_ADDRS)) {
		NL_SET_ERR_MSG_MOD(extack, "Not matching on eth address");
		return -EOPNOTSUPP;
	}

	flow_rule_match_eth_addrs(rule, &match);

	if (!is_zero_ether_addr(match.mask->src)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Matching on source MAC not supported");
		return -EOPNOTSUPP;
	}

	if (is_broadcast_ether_addr(match.key->dst) &&
	    is_broadcast_ether_addr(match.mask->dst)) {
		ret = cpsw_ale_rx_ratelimit_bc(port->common->ale, port->port_id, rate_pkt_ps);
		if (ret)
			return ret;

		qos->ale_bc_ratelimit.cookie = cls->cookie;
		qos->ale_bc_ratelimit.rate_packet_ps = rate_pkt_ps;
	} else if (ether_addr_equal_unaligned(match.key->dst, mc_mac) &&
		   ether_addr_equal_unaligned(match.mask->dst, mc_mac)) {
		ret = cpsw_ale_rx_ratelimit_mc(port->common->ale, port->port_id, rate_pkt_ps);
		if (ret)
			return ret;

		qos->ale_mc_ratelimit.cookie = cls->cookie;
		qos->ale_mc_ratelimit.rate_packet_ps = rate_pkt_ps;
	} else {
		NL_SET_ERR_MSG_MOD(extack, "Not supported matching key");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int am65_cpsw_qos_clsflower_policer_validate(const struct flow_action *action,
						    const struct flow_action_entry *act,
						    struct netlink_ext_ack *extack)
{
	if (act->police.exceed.act_id != FLOW_ACTION_DROP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when exceed action is not drop");
		return -EOPNOTSUPP;
	}

	if (act->police.notexceed.act_id != FLOW_ACTION_PIPE &&
	    act->police.notexceed.act_id != FLOW_ACTION_ACCEPT) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is not pipe or ok");
		return -EOPNOTSUPP;
	}

	if (act->police.notexceed.act_id == FLOW_ACTION_ACCEPT &&
	    !flow_action_is_last_entry(action, act)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when conform action is ok, but action is not last");
		return -EOPNOTSUPP;
	}

	if (act->police.rate_bytes_ps || act->police.peakrate_bytes_ps ||
	    act->police.avrate || act->police.overhead) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Offload not supported when bytes per second/peakrate/avrate/overhead is configured");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int am65_cpsw_qos_configure_clsflower(struct am65_cpsw_port *port,
					     struct flow_cls_offload *cls)
{
	struct flow_rule *rule = flow_cls_offload_flow_rule(cls);
	struct netlink_ext_ack *extack = cls->common.extack;
	const struct flow_action_entry *act;
	int i, ret;

	flow_action_for_each(i, act, &rule->action) {
		switch (act->id) {
		case FLOW_ACTION_POLICE:
			ret = am65_cpsw_qos_clsflower_policer_validate(&rule->action, act, extack);
			if (ret)
				return ret;

			return am65_cpsw_qos_clsflower_add_policer(port, extack, cls,
								   act->police.rate_pkt_ps);
		default:
			NL_SET_ERR_MSG_MOD(extack,
					   "Action not supported");
			return -EOPNOTSUPP;
		}
	}
	return -EOPNOTSUPP;
}

static int am65_cpsw_qos_delete_clsflower(struct am65_cpsw_port *port, struct flow_cls_offload *cls)
{
	struct am65_cpsw_qos *qos = &port->qos;

	if (cls->cookie == qos->ale_bc_ratelimit.cookie) {
		qos->ale_bc_ratelimit.cookie = 0;
		qos->ale_bc_ratelimit.rate_packet_ps = 0;
		cpsw_ale_rx_ratelimit_bc(port->common->ale, port->port_id, 0);
	}

	if (cls->cookie == qos->ale_mc_ratelimit.cookie) {
		qos->ale_mc_ratelimit.cookie = 0;
		qos->ale_mc_ratelimit.rate_packet_ps = 0;
		cpsw_ale_rx_ratelimit_mc(port->common->ale, port->port_id, 0);
	}

	return 0;
}

static int am65_cpsw_qos_setup_tc_clsflower(struct am65_cpsw_port *port,
					    struct flow_cls_offload *cls_flower)
{
	switch (cls_flower->command) {
	case FLOW_CLS_REPLACE:
		return am65_cpsw_qos_configure_clsflower(port, cls_flower);
	case FLOW_CLS_DESTROY:
		return am65_cpsw_qos_delete_clsflower(port, cls_flower);
	default:
		return -EOPNOTSUPP;
	}
}

static int am65_cpsw_qos_setup_tc_block_cb(enum tc_setup_type type, void *type_data, void *cb_priv)
{
	struct am65_cpsw_port *port = cb_priv;

	if (!tc_cls_can_offload_and_chain0(port->ndev, type_data))
		return -EOPNOTSUPP;

	switch (type) {
	case TC_SETUP_CLSFLOWER:
		return am65_cpsw_qos_setup_tc_clsflower(port, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static LIST_HEAD(am65_cpsw_qos_block_cb_list);

static int am65_cpsw_qos_setup_tc_block(struct net_device *ndev, struct flow_block_offload *f)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	return flow_block_cb_setup_simple(f, &am65_cpsw_qos_block_cb_list,
					  am65_cpsw_qos_setup_tc_block_cb,
					  port, port, true);
}

static void
am65_cpsw_qos_tx_p0_rate_apply(struct am65_cpsw_common *common,
			       int tx_ch, u32 rate_mbps)
{
	struct am65_cpsw_host *host = am65_common_get_host(common);
	u32 ch_cir;
	int i;

	ch_cir = am65_cpsw_qos_tx_rate_calc(rate_mbps, common->bus_freq);
	writel(ch_cir, host->port_base + AM65_CPSW_PN_REG_PRI_CIR(tx_ch));

	/* update rates for every port tx queues */
	for (i = 0; i < common->port_num; i++) {
		struct net_device *ndev = common->ports[i].ndev;

		if (!ndev)
			continue;
		netdev_get_tx_queue(ndev, tx_ch)->tx_maxrate = rate_mbps;
	}
}

int am65_cpsw_qos_ndo_tx_p0_set_maxrate(struct net_device *ndev,
					int queue, u32 rate_mbps)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = port->common;
	struct am65_cpsw_tx_chn *tx_chn;
	u32 ch_rate, tx_ch_rate_msk_new;
	u32 ch_msk = 0;
	int ret;

	dev_dbg(common->dev, "apply TX%d rate limiting %uMbps tx_rate_msk%x\n",
		queue, rate_mbps, common->tx_ch_rate_msk);

	if (common->pf_p0_rx_ptype_rrobin) {
		dev_err(common->dev, "TX Rate Limiting failed - rrobin mode\n");
		return -EINVAL;
	}

	ch_rate = netdev_get_tx_queue(ndev, queue)->tx_maxrate;
	if (ch_rate == rate_mbps)
		return 0;

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}
	ret = 0;

	tx_ch_rate_msk_new = common->tx_ch_rate_msk;
	if (rate_mbps && !(tx_ch_rate_msk_new & BIT(queue))) {
		tx_ch_rate_msk_new |= BIT(queue);
		ch_msk = GENMASK(common->tx_ch_num - 1, queue);
		ch_msk = tx_ch_rate_msk_new ^ ch_msk;
	} else if (!rate_mbps) {
		tx_ch_rate_msk_new &= ~BIT(queue);
		ch_msk = queue ? GENMASK(queue - 1, 0) : 0;
		ch_msk = tx_ch_rate_msk_new & ch_msk;
	}

	if (ch_msk) {
		dev_err(common->dev, "TX rate limiting has to be enabled sequentially hi->lo tx_rate_msk:%x tx_rate_msk_new:%x\n",
			common->tx_ch_rate_msk, tx_ch_rate_msk_new);
		ret = -EINVAL;
		goto exit_put;
	}

	tx_chn = &common->tx_chns[queue];
	tx_chn->rate_mbps = rate_mbps;
	common->tx_ch_rate_msk = tx_ch_rate_msk_new;

	if (!common->usage_count)
		/* will be applied on next netif up */
		goto exit_put;

	am65_cpsw_qos_tx_p0_rate_apply(common, queue, rate_mbps);

exit_put:
	pm_runtime_put(common->dev);
	return ret;
}

void am65_cpsw_qos_tx_p0_rate_init(struct am65_cpsw_common *common)
{
	struct am65_cpsw_host *host = am65_common_get_host(common);
	int tx_ch;

	for (tx_ch = 0; tx_ch < common->tx_ch_num; tx_ch++) {
		struct am65_cpsw_tx_chn *tx_chn = &common->tx_chns[tx_ch];
		u32 ch_cir;

		if (!tx_chn->rate_mbps)
			continue;

		ch_cir = am65_cpsw_qos_tx_rate_calc(tx_chn->rate_mbps,
						    common->bus_freq);
		writel(ch_cir,
		       host->port_base + AM65_CPSW_PN_REG_PRI_CIR(tx_ch));
	}
}

int am65_cpsw_qos_ndo_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			       void *type_data)
{
	switch (type) {
	case TC_QUERY_CAPS:
		return am65_cpsw_tc_query_caps(ndev, type_data);
	case TC_SETUP_QDISC_TAPRIO:
		return am65_cpsw_setup_taprio(ndev, type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return am65_cpsw_setup_mqprio(ndev, type_data);
	case TC_SETUP_BLOCK:
		return am65_cpsw_qos_setup_tc_block(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

static void am65_cpsw_cut_thru_dump(struct am65_cpsw_port *port)
{
	struct am65_cpsw_common *common = port->common;
	u32 contro, cut_thru, speed;

	contro = readl(common->cpsw_base + AM65_CPSW_REG_CTL);
	cut_thru = readl(port->port_base + AM64_CPSW_PN_CUT_THRU);
	speed = readl(port->port_base + AM64_CPSW_PN_SPEED);
	dev_dbg(common->dev,
		"Port%u: cut_thru dump control:%08x cut_thru:%08x hwspeed:%08x\n",
		port->port_id, contro, cut_thru, speed);
}

static u32 am65_cpsw_cut_thru_speed2hw(int link_speed)
{
	switch (link_speed) {
	case SPEED_10:
		return 1;
	case SPEED_100:
		return 2;
	case SPEED_1000:
		return 3;
	default:
		return 0;
	}
}

static void am65_cpsw_cut_thru_link_up(struct am65_cpsw_port *port)
{
	struct am65_cpsw_cut_thru *cut_thru = &port->qos.cut_thru;
	struct am65_cpsw_common *common = port->common;
	u32 val, speed;

	if (!cut_thru->enable)
		return;

	writel(AM64_PN_SPEED_AUTO_EN, port->port_base + AM64_CPSW_PN_SPEED);
	/* barrier */
	readl(port->port_base + AM64_CPSW_PN_SPEED);
	/* HW need 15us in 10/100 mode and 3us in 1G mode auto speed detection
	 * add delay with some margin
	 */
	usleep_range(40, 50);
	val = readl(port->port_base + AM64_CPSW_PN_SPEED);
	speed = FIELD_GET(AM64_PN_AUTO_SPEED, val);
	if (!speed) {
		dev_warn(common->dev,
			 "Port%u: cut_thru no speed auto detected switch to manual\n",
			 port->port_id);
		speed = am65_cpsw_cut_thru_speed2hw(port->qos.link_speed);
		if (!speed) {
			dev_err(common->dev,
				"Port%u: cut_thru speed configuration failed\n",
				port->port_id);
			return;
		}
		val = FIELD_PREP(AM64_PN_SPEED_VAL, speed);
		writel(val, port->port_base + AM64_CPSW_PN_SPEED);
	}

	val = FIELD_PREP(AM64_PN_CUT_THRU_TX_PRI, cut_thru->tx_pri_mask) |
	      FIELD_PREP(AM64_PN_CUT_THRU_RX_PRI, cut_thru->rx_pri_mask);

	if (port->qos.duplex) {
		writel(val, port->port_base + AM64_CPSW_PN_CUT_THRU);
		dev_info(common->dev, "Port%u: Enable cut_thru rx:%08x tx:%08x hwspeed:%u (%08x)\n",
			 port->port_id,
			 cut_thru->rx_pri_mask, cut_thru->tx_pri_mask,
			 speed, val);
	} else {
		writel(0, port->port_base + AM64_CPSW_PN_CUT_THRU);
		dev_info(common->dev, "Port%u: Disable cut_thru duplex=%d\n",
			 port->port_id, port->qos.duplex);
	}
	am65_cpsw_cut_thru_dump(port);
}

void am65_cpsw_qos_link_up(struct net_device *ndev, int link_speed, int duplex)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	port->qos.link_speed = link_speed;
	port->qos.duplex = duplex;
	am65_cpsw_tx_pn_shaper_apply(port);
	am65_cpsw_iet_link_state_update(ndev);
	am65_cpsw_cut_thru_link_up(port);

	am65_cpsw_est_link_up(ndev, link_speed);
	port->qos.link_down_time = 0;
}

void am65_cpsw_qos_link_down(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	port->qos.link_speed = SPEED_UNKNOWN;
	am65_cpsw_tx_pn_shaper_apply(port);
	am65_cpsw_iet_link_state_update(ndev);

	if (!port->qos.link_down_time)
		port->qos.link_down_time = ktime_get();
}

static void am65_cpsw_cut_thru_enable(struct am65_cpsw_common *common)
{
	u32 val;

	if (common->cut_thru_enabled) {
		common->cut_thru_enabled++;
		return;
	}

	/* Populate CPSW VBUS freq for auto speed detection */
	writel(common->bus_freq / 1000000,
	       common->cpsw_base + AM65_CPSW_REG_FREQ);

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);
	val |= AM64_CPSW_CTL_CUT_THRU_EN;
	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
	common->cut_thru_enabled++;
}

void am65_cpsw_qos_cut_thru_init(struct am65_cpsw_port *port)
{
	struct am65_cpsw_cut_thru *cut_thru = &port->qos.cut_thru;
	struct am65_cpsw_common *common = port->common;

	/* Enable cut_thr only if user has enabled priv flag */
	if (!cut_thru->enable)
		return;

	if (common->is_emac_mode) {
		cut_thru->enable = false;
		dev_info(common->dev, "Disable cut-thru, need Switch mode\n");
		return;
	}

	am65_cpsw_cut_thru_enable(common);

	/* en auto speed */
	writel(AM64_PN_SPEED_AUTO_EN, port->port_base + AM64_CPSW_PN_SPEED);
	dev_info(common->dev, "Init cut_thru\n");
	am65_cpsw_cut_thru_dump(port);
}

static void am65_cpsw_cut_thru_disable(struct am65_cpsw_common *common)
{
	u32 val;

	if (--common->cut_thru_enabled)
		return;

	val = readl(common->cpsw_base + AM65_CPSW_REG_CTL);
	val &= ~AM64_CPSW_CTL_CUT_THRU_EN;
	writel(val, common->cpsw_base + AM65_CPSW_REG_CTL);
}

void am65_cpsw_qos_cut_thru_cleanup(struct am65_cpsw_port *port)
{
	struct am65_cpsw_cut_thru *cut_thru = &port->qos.cut_thru;
	struct am65_cpsw_common *common = port->common;

	if (!cut_thru->enable)
		return;

	writel(0, port->port_base + AM64_CPSW_PN_CUT_THRU);
	writel(0, port->port_base + AM64_CPSW_PN_SPEED);

	am65_cpsw_cut_thru_disable(common);
	dev_info(common->dev, "Cleanup cut_thru\n");
	am65_cpsw_cut_thru_dump(port);
}
