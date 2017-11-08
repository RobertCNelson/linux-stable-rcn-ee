/* SPDX-License-Identifier: GPL-2.0 */
/**
 * irq-pruss-intc.h - PRU-ICSS INTC management
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __INCLUDE_LINUX_IRQCHIP_IRQ_PRUSS_INTC_H
#define __INCLUDE_LINUX_IRQCHIP_IRQ_PRUSS_INTC_H

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/**
 * struct pruss_intc_config - INTC configuration info
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss_intc_config {
	s8 sysev_to_ch[MAX_PRU_SYS_EVENTS];
	s8 ch_to_host[MAX_PRU_CHANNELS];
};

#if IS_ENABLED(CONFIG_TI_PRUSS)

/**
 * pruss_intc_configure() - configure the PRUSS INTC
 * @dev: device
 * @intc_config: PRU core-specific INTC configuration
 *
 * Configures the PRUSS INTC with the provided configuration from
 * a PRU core. Any existing event to channel mappings or channel to
 * host interrupt mappings are checked to make sure there are no
 * conflicting configuration between both the PRU cores. The function
 * is intended to be used only by the PRU remoteproc driver.
 *
 * Returns 0 on success, or a suitable error code otherwise
 */
int pruss_intc_configure(struct device *dev,
			 struct pruss_intc_config *intc_config);

/**
 * pruss_intc_unconfigure() - unconfigure the PRUSS INTC
 * @dev: device
 * @intc_config: PRU core specific INTC configuration
 *
 * Undo whatever was done in pruss_intc_configure() for a PRU core.
 * It should be sufficient to just mark the resources free in the
 * global map and disable the host interrupts and sysevents.
 */
int pruss_intc_unconfigure(struct device *dev,
			   struct pruss_intc_config *intc_config);
/**
 * pruss_intc_trigger() - trigger a PRU system event
 * @irq: linux IRQ number associated with a PRU system event
 *
 * Trigger an interrupt by signalling a specific PRU system event.
 * This can be used by PRUSS client users to raise/send an event to
 * a PRU or any other core that is listening on the host interrupt
 * mapped to that specific PRU system event. The @irq variable is the
 * Linux IRQ number associated with a specific PRU system event that
 * a client user/application uses. The interrupt mappings for this is
 * provided by the PRUSS INTC irqchip instance.
 *
 * Returns 0 on success, or an error value upon failure.
 */
int pruss_intc_trigger(unsigned int irq);

#else

static inline int pruss_intc_configure(struct device *dev,
				       struct pruss_intc_config *intc_config)
{
	return -ENOTSUPP;
}

static inline int pruss_intc_unconfigure(struct device *dev,
					 struct pruss_intc_config *intc_config)
{
	return -ENOTSUPP;
}

static inline int pruss_intc_trigger(unsigned int irq)
{
	return -ENOTSUPP;
}

#endif	/* CONFIG_TI_PRUSS */

#endif /* __INCLUDE_LINUX_IRQCHIP_IRQ_OMAP_INTC_H */
