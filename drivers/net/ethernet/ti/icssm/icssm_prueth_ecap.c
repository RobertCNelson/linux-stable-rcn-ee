// SPDX-License-Identifier: GPL-2.0

/* PRUETH Ecap driver for Interrupt pacing support. eCAP is used by
 * firmware to implement Rx Interrupt pacing for PRUETH driver using
 * ECAP1 and ECAP2.  Firmware uses ECAP as a timer to implement
 * interrupt pacing logic. For HSR/PRP, the interrupt pacing can
 * be enabled/disabled for both ports together as there is a common
 * control for both ports, where as for Dual EMAC, interrupt pacing
 * can be enabled or disabled independently for both Ethernet ports.
 * SRAM memory location stores the configuration for interrupt pacing
 * such as enable/disable flag and timeout values.
 *
 * TODO: This is marked as a HACK driver since the correct solution
 * is to move the initialization of the ECAP registers to firmware.
 * Driver has nothing to do ECAP as it is used by firmware and it
 * is expected that firmware does the initialization.
 *
 * Copyright (C) 2018-2020 Texas Instruments Incorporated - https://www.ti.com
 *	Murali Karicheri <m-karicheri2@ti.com>
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "icssm_switch.h"
#include "icssm_prueth_ecap.h"

/* ECAP registers */
#define ECAP_CAP1			8
#define ECAP_CAP2			0xC
#define ECAP_ECCTL2			0x2A

#define ECAP_ECCTL2_TSCTRSTOP_MASK	BIT(4)
#define ECAP_ECCTL2_CAP_APWM_MASK	BIT(9)

#define ECAP_ECCTL2_INIT_VAL		(ECAP_ECCTL2_TSCTRSTOP_MASK | \
					 ECAP_ECCTL2_CAP_APWM_MASK)
#define ECAP_CAP2_MAX_COUNT		0xFFFFFFFF

/* TODO: Driver assumes that ECAP runs at 200Mhz clock. But on some
 * platforms, PRU ICSS clock rate may be changed by user in which case
 * the pacing logic will not work as expected. Update the driver and
 * firmware if ECAP/PRUSS clock rate is ever changed. Based on this
 * assumption each tick is 5 nsec. i.e 1000/200
 */
#define ECAP_TICK_NSEC			5

/* in usec */
/* Duration of 3 frames of 1528 bytes each. If we go beyond this,
 * receive buffer overflow may happen assuming 4 MTU buffer. So
 * set this as the limit
 */
#define MAX_RX_TIMEOUT_USEC		(123 * 3)

/* Dual EMAC defaults */
static struct rx_int_pacing_offsets pacing_offsets_defaults[PRUETH_NUM_MACS] = {
	{ INTR_PAC_STATUS_OFFSET_PRU0, INTR_PAC_TMR_EXP_OFFSET_PRU0,
	  INTR_PAC_PREV_TS_OFFSET_PRU0 },
	{ INTR_PAC_STATUS_OFFSET_PRU1, INTR_PAC_TMR_EXP_OFFSET_PRU1,
	  INTR_PAC_PREV_TS_OFFSET_PRU1 },
};

static int icssm_prueth_ecap_config_pacing(struct prueth_emac *emac,
					   u32 use_adaptive,
					   u32 new_timeout_val)
{
	struct rx_int_pacing_offsets *offsets;
	struct prueth *prueth = emac->prueth;
	u8 val = INTR_PAC_DIS_ADP_LGC_DIS;
	struct prueth_ecap *ecap;
	void __iomem *sram;
	u32 pacing_ctrl;
	int port;

	ecap = prueth->ecap;
	sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	port = (emac->port_id == PRUETH_PORT_MII0) ?
				PRUETH_MAC0 : PRUETH_MAC1;
	offsets = &ecap->int_pacing_offsets[port];
	pacing_ctrl = offsets->rx_int_pacing_ctrl;

	if (!new_timeout_val) {
		/* disable pacing */
		writeb_relaxed(val, sram + pacing_ctrl);
		/* Timeout separate */
		if (PRUETH_IS_LRE(prueth) || PRUETH_IS_SWITCH(prueth)) {
			ecap->timeout[PRUETH_MAC0] = new_timeout_val;
			ecap->timeout[PRUETH_MAC1] = new_timeout_val;
		} else {
			ecap->timeout[port] = new_timeout_val;
		}
		return 0;
	}

	if (use_adaptive)
		val = INTR_PAC_ENA_ADP_LGC_ENA;
	else
		val = INTR_PAC_ENA_ADP_LGC_DIS;

	if (!ecap->timeout[port]) {
		/* disable to enable transition */
		writeb_relaxed(INTR_PAC_DIS_ADP_LGC_DIS, sram + pacing_ctrl);
		/* For EMAC set timeout for specific port and for
		 * LRE for both ports
		 */
		if (PRUETH_IS_LRE(prueth) || PRUETH_IS_SWITCH(prueth)) {
			offsets = &ecap->int_pacing_offsets[PRUETH_MAC0];
			writel_relaxed(new_timeout_val *
				       NSEC_PER_USEC / ECAP_TICK_NSEC,
				       sram + offsets->rx_int_pacing_exp);
			writel_relaxed(INTR_PAC_PREV_TS_RESET_VAL,
				       sram + offsets->rx_int_pacing_prev);
			ecap->timeout[PRUETH_MAC0] = new_timeout_val;

			offsets = &ecap->int_pacing_offsets[PRUETH_MAC1];
			writel_relaxed(new_timeout_val *
				       NSEC_PER_USEC / ECAP_TICK_NSEC,
				       sram + offsets->rx_int_pacing_exp);
			writel_relaxed(INTR_PAC_PREV_TS_RESET_VAL,
				       sram + offsets->rx_int_pacing_prev);
			ecap->timeout[PRUETH_MAC1] = new_timeout_val;

		} else if (PRUETH_IS_EMAC(prueth)) {
			if (!port) {
				offsets =
					&ecap->int_pacing_offsets[PRUETH_MAC0];
				writel_relaxed(new_timeout_val *
						NSEC_PER_USEC / ECAP_TICK_NSEC,
						sram +
						offsets->rx_int_pacing_exp);
				writel_relaxed(INTR_PAC_PREV_TS_RESET_VAL,
					       sram +
					       offsets->rx_int_pacing_prev);
				ecap->timeout[PRUETH_MAC0] = new_timeout_val;
			} else {
				offsets =
					&ecap->int_pacing_offsets[PRUETH_MAC1];
				writel_relaxed(new_timeout_val *
						NSEC_PER_USEC / ECAP_TICK_NSEC,
						sram +
						offsets->rx_int_pacing_exp);
				writel_relaxed(INTR_PAC_PREV_TS_RESET_VAL,
					       sram +
					       offsets->rx_int_pacing_prev);
				ecap->timeout[PRUETH_MAC1] = new_timeout_val;
			}
		}
	} else {
		/* update */
		if (PRUETH_IS_LRE(prueth) || PRUETH_IS_SWITCH(prueth)) {
			offsets = &ecap->int_pacing_offsets[PRUETH_MAC0];
			writel_relaxed(new_timeout_val *
				       NSEC_PER_USEC / ECAP_TICK_NSEC,
				       sram + offsets->rx_int_pacing_exp);
			ecap->timeout[PRUETH_MAC0] = new_timeout_val;

			offsets = &ecap->int_pacing_offsets[PRUETH_MAC1];
			writel_relaxed(new_timeout_val *
				       NSEC_PER_USEC / ECAP_TICK_NSEC,
				       sram + offsets->rx_int_pacing_exp);
			ecap->timeout[PRUETH_MAC1] = new_timeout_val;
		} else if (PRUETH_IS_EMAC(prueth)) {
			if (!port) {
				offsets =
					&ecap->int_pacing_offsets[PRUETH_MAC0];
				writel_relaxed(new_timeout_val *
						NSEC_PER_USEC / ECAP_TICK_NSEC,
						sram +
						offsets->rx_int_pacing_exp);
				ecap->timeout[PRUETH_MAC0] = new_timeout_val;
			} else {
				offsets =
					&ecap->int_pacing_offsets[PRUETH_MAC1];
				writel_relaxed(new_timeout_val *
						NSEC_PER_USEC / ECAP_TICK_NSEC,
						sram +
						offsets->rx_int_pacing_exp);
				ecap->timeout[PRUETH_MAC1] = new_timeout_val;
			}
		}
	}

	writeb_relaxed(val, sram + pacing_ctrl);

	return 0;
}

/**
 * icssm_prueth_ecap_init - ecap driver init
 *
 * @emac: EMAC data structure
 *
 */
static void icssm_prueth_ecap_init(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap;

	ecap = prueth->ecap;
	if (PRUETH_IS_LRE(prueth) || PRUETH_IS_SWITCH(prueth)) {
		/* Both ports shares the same location */
		ecap->int_pacing_offsets[PRUETH_MAC0].rx_int_pacing_ctrl =
			INTR_PAC_STATUS_OFFSET;
		ecap->int_pacing_offsets[PRUETH_MAC1].rx_int_pacing_ctrl =
			INTR_PAC_STATUS_OFFSET;
	}

	if (!prueth->emac_configured || PRUETH_IS_EMAC(prueth))
		icssm_prueth_ecap_config_pacing(emac, 0, 0);
}

static int icssm_prueth_ecap_get_coalesce(struct prueth_emac *emac,
					  u32 *use_adaptive_rx_coalesce,
					  u32 *rx_coalesce_usecs)
{
	struct rx_int_pacing_offsets *pacing_offsets;
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap;
	void __iomem *sram;
	int port;
	u32 val;

	ecap = prueth->ecap;
	sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	port = (emac->port_id == PRUETH_PORT_MII0) ?
			PRUETH_MAC0 : PRUETH_MAC1;
	pacing_offsets = &ecap->int_pacing_offsets[port];
	val = readb_relaxed(sram + pacing_offsets->rx_int_pacing_ctrl);
	*use_adaptive_rx_coalesce = (val == INTR_PAC_ENA_ADP_LGC_ENA);
	*rx_coalesce_usecs = ecap->timeout[port];

	return 0;
}

static int icssm_prueth_ecap_set_coalesce(struct prueth_emac *emac,
					  u32 use_adaptive_rx_coalesce,
					  u32 rx_coalesce_usecs)
{
	struct prueth *prueth = emac->prueth;
	int ret;

	if (rx_coalesce_usecs  > MAX_RX_TIMEOUT_USEC)
		return -EINVAL;

	mutex_lock(&prueth->mlock);
	/* Start or restart the pacing timer. */
	ret = icssm_prueth_ecap_config_pacing(emac, use_adaptive_rx_coalesce,
					      rx_coalesce_usecs);
	mutex_unlock(&prueth->mlock);

	return ret;
}

void icssm_prueth_ecap_put(struct prueth_ecap *ecap)
{
	device_lock(ecap->dev);
	ecap->client_np = NULL;
	device_unlock(ecap->dev);
	put_device(ecap->dev);
}
EXPORT_SYMBOL_GPL(icssm_prueth_ecap_put);

struct prueth_ecap *icssm_prueth_ecap_get(struct device_node *np)
{
	struct platform_device *pdev;
	struct device_node *ecap_np;
	struct prueth_ecap *ecap;

	ecap_np = of_parse_phandle(np, "ti,ecap", 0);
	if (!ecap_np || !of_device_is_available(ecap_np))
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(ecap_np);
	of_node_put(ecap_np);

	if (!pdev)
		/* probably IEP not yet probed */
		return ERR_PTR(-EPROBE_DEFER);

	ecap = platform_get_drvdata(pdev);
	if (!ecap)
		return ERR_PTR(-EPROBE_DEFER);

	device_lock(ecap->dev);
	if (ecap->client_np) {
		device_unlock(ecap->dev);
		dev_err(ecap->dev, "ECAP is already acquired by %s",
			ecap->client_np->name);
		return ERR_PTR(-EBUSY);
	}
	ecap->client_np = np;
	device_unlock(ecap->dev);
	get_device(ecap->dev);

	return ecap;
}
EXPORT_SYMBOL_GPL(icssm_prueth_ecap_get);

static int icssm_prueth_ecap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct prueth_ecap *ecap;
	struct resource *res;
	int i;

	ecap = devm_kzalloc(dev, sizeof(*ecap), GFP_KERNEL);
	if (!ecap)
		return -ENOMEM;

	ecap->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ecap->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ecap->base))
		return -ENODEV;

	/* Initialize the ECAP timer. It is a common timer used
	 * by firmware for rx interrupt pacing.
	 */
	writew_relaxed(ECAP_ECCTL2_INIT_VAL, ecap->base + ECAP_ECCTL2);
	writel_relaxed(ECAP_CAP2_MAX_COUNT, ecap->base + ECAP_CAP1);
	writel_relaxed(ECAP_CAP2_MAX_COUNT, ecap->base + ECAP_CAP2);

	/* initialize SRAM memory offsets for rx pace time control */
	for (i = 0; i < PRUETH_NUM_MACS; i++)
		ecap->int_pacing_offsets[i] = pacing_offsets_defaults[i];
	ecap->get_coalesce = icssm_prueth_ecap_get_coalesce;
	ecap->set_coalesce = icssm_prueth_ecap_set_coalesce;
	ecap->init = icssm_prueth_ecap_init;

	dev_set_drvdata(dev, ecap);

	return 0;
}

static const struct of_device_id prueth_ecap_of_match[] = {
	{ .compatible = "ti,pruss-ecap", },
	{ }
};
MODULE_DEVICE_TABLE(of, prueth_ecap_of_match);

static struct platform_driver prueth_ecap_driver = {
	.driver = {
		.name = "prueth-ecap",
		.of_match_table = prueth_ecap_of_match,
	},
	.probe = icssm_prueth_ecap_probe,
};
module_platform_driver(prueth_ecap_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TI PRUETH ECAP driver for Rx Interrupt pacing");
MODULE_AUTHOR("Murali Karicheri <m-karicheri2@ti.com>");
