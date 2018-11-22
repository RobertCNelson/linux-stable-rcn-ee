/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRUSS_H_
#define _PRUSS_H_

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT	2

/* maximum number of host interrupts */
#define MAX_PRU_HOST_INT	10

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

#endif	/* _PRUSS_H_ */
