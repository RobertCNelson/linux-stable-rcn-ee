/* SPDX-License-Identifier: GPL-2.0 */
/**
 * PRU-ICSS Remote Subsystem user interfaces
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __LINUX_REMOTEPROC_PRU_RPROC_H
#define __LINUX_REMOTEPROC_PRU_RPROC_H

/**
 * enum pru_ctable_idx - Configurable Constant table index identifiers
 */
enum pru_ctable_idx {
	PRU_C24 = 0,
	PRU_C25,
	PRU_C26,
	PRU_C27,
	PRU_C28,
	PRU_C29,
	PRU_C30,
	PRU_C31,
};

enum pruss_gpi_mode;

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

struct rproc *pru_rproc_get(struct device_node *node, int index);
void pru_rproc_put(struct rproc *rproc);
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr);
int pru_rproc_set_gpimode(struct rproc *rproc, enum pruss_gpi_mode mode);

#else

static inline struct rproc *pru_rproc_get(struct device_node *node, int index)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pru_rproc_put(struct rproc *rproc) { }

static inline int pru_rproc_set_ctable(struct rproc *rproc,
				       enum pru_ctable_idx c, u32 addr)
{
	return -ENOTSUPP;
}

static inline int pru_rproc_set_gpimode(struct rproc *rproc,
					enum pruss_gpi_mode mode)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_REMOTEPROC_PRU_RPROC_H */
