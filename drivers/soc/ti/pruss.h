/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRUSS_H_
#define _PRUSS_H_

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT	2

/* maximum number of host interrupts */
#define MAX_PRU_HOST_INT	10

/**
 * struct pruss_intc_config - INTC configuration info
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss_intc_config {
	s8 sysev_to_ch[MAX_PRU_SYS_EVENTS];
	s8 ch_to_host[MAX_PRU_CHANNELS];
};

/**
 * struct pruss - PRUSS parent structure
 * @dev: pruss device pointer
 * @cfg: regmap for config region
 * @mem_regions: data for each of the PRUSS memory regions
 * @mem_in_use: to indicate if memory resource is in use
 * @host_mask: indicate which HOST IRQs are enabled
 * @lock: mutex to serialize access to resources
 */
struct pruss {
	struct device *dev;
	struct regmap *cfg;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	u32 host_mask;
	struct mutex lock; /* PRU resource lock */
};

int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config);
int pruss_intc_unconfigure(struct pruss *pruss,
			   struct pruss_intc_config *intc_config);

#endif	/* _PRUSS_H_ */
