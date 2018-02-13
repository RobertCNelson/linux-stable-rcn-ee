/* SPDX-License-Identifier: GPL-2.0 */
/**
 * PRU-ICSS Remote Subsystem user interfaces
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __LINUX_REMOTEPROC_PRU_RPROC_H
#define __LINUX_REMOTEPROC_PRU_RPROC_H

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

struct rproc *pru_rproc_get(struct device_node *node, int index);
void pru_rproc_put(struct rproc *rproc);

#else

static inline struct rproc *pru_rproc_get(struct device_node *node, int index)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pru_rproc_put(struct rproc *rproc) { }

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_REMOTEPROC_PRU_RPROC_H */
