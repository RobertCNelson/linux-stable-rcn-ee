/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2022 Renesas Electronics Corp.
 */

#ifndef __ASM_DMA_NONCOHERENT_H
#define __ASM_DMA_NONCOHERENT_H

#include <linux/dma-direct.h>

enum dma_noncoherent_ops {
	NON_COHERENT_SYNC_DMA_FOR_DEVICE = 0,
	NON_COHERENT_SYNC_DMA_FOR_CPU,
	NON_COHERENT_DMA_PREP,
	NON_COHERENT_DMA_PMEM,
};

/*
 * struct riscv_cache_ops - Structure for CMO function pointers
 * @clean_range: Function pointer for clean cache
 * @inval_range: Function pointer for invalidate cache
 * @flush_range: Function pointer for flushing the cache
 */
struct riscv_cache_ops {
	void (*clean_range)(unsigned long addr, unsigned long size);
	void (*inval_range)(unsigned long addr, unsigned long size);
	void (*flush_range)(unsigned long addr, unsigned long size);
};

extern struct riscv_cache_ops zicbom_cmo_ops;

#ifdef CONFIG_RISCV_DMA_NONCOHERENT

extern struct riscv_cache_ops noncoherent_cache_ops;

void riscv_noncoherent_register_cache_ops(struct riscv_cache_ops *ops);

static inline void riscv_dma_noncoherent_clean(void *vaddr, size_t size)
{
	unsigned long addr = (unsigned long)vaddr;

	if (!noncoherent_cache_ops.clean_range)
		return;

	noncoherent_cache_ops.clean_range(addr, size);
}

static inline void riscv_dma_noncoherent_flush(void *vaddr, size_t size)
{
	unsigned long addr = (unsigned long)vaddr;

	if (!noncoherent_cache_ops.flush_range)
		return;

	noncoherent_cache_ops.flush_range(addr, size);
}

static inline void riscv_dma_noncoherent_inval(void *vaddr, size_t size)
{
	unsigned long addr = (unsigned long)vaddr;

	if (!noncoherent_cache_ops.inval_range)
		return;

	noncoherent_cache_ops.inval_range(addr, size);
}

#else

static void riscv_noncoherent_register_cache_ops(struct riscv_cache_ops *ops) {}

#endif

#endif	/* __ASM_DMA_NONCOHERENT_H */
