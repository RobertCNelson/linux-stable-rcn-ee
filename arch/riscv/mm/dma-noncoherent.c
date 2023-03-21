// SPDX-License-Identifier: GPL-2.0-only
/*
 * RISC-V specific functions to support DMA for non-coherent devices
 *
 * Copyright (c) 2021 Western Digital Corporation or its affiliates.
 */

#include <linux/dma-direct.h>
#include <linux/dma-map-ops.h>
#include <linux/mm.h>
#include <asm/cacheflush.h>
#include <asm/dma-noncoherent.h>

static bool noncoherent_supported;

struct riscv_cache_ops noncoherent_cache_ops = {
	.clean_range = NULL,
	.inval_range = NULL,
	.flush_range = NULL,
};
EXPORT_SYMBOL_GPL(noncoherent_cache_ops);

#ifdef CONFIG_RISCV_ISA_ZICBOM
#define ZICBOM_CMO_OP(_op, _start, _size, _cachesize)				\
	asm volatile("mv a0, %1\n\t"						\
		     "j 2f\n\t"							\
		     "3:\n\t"							\
		     "cbo." __stringify(_op) " (a0)\n\t"			\
		     "add a0, a0, %0\n\t"					\
		     "2:\n\t"							\
		     "bltu a0, %2, 3b\n\t"					\
		     : : "r"(_cachesize),					\
			 "r"((unsigned long)(_start) & ~((_cachesize) - 1UL)),	\
			 "r"((unsigned long)(_start) + (_size))			\
		     : "a0")

static void zicbom_cmo_clean_range(unsigned long addr, unsigned long size)
{
	ZICBOM_CMO_OP(clean, addr, size, riscv_cbom_block_size);
}

static void zicbom_cmo_flush_range(unsigned long addr, unsigned long size)
{
	ZICBOM_CMO_OP(flush, addr, size, riscv_cbom_block_size);
}

static void zicbom_cmo_inval_range(unsigned long addr, unsigned long size)
{
	ZICBOM_CMO_OP(inval, addr, size, riscv_cbom_block_size);
}

struct riscv_cache_ops zicbom_cmo_ops = {
	.clean_range = &zicbom_cmo_clean_range,
	.inval_range = &zicbom_cmo_inval_range,
	.flush_range = &zicbom_cmo_flush_range,
};
#else
struct riscv_cache_ops zicbom_cmo_ops = {
	.clean_range = NULL,
	.inval_range = NULL,
	.flush_range = NULL,
};
#endif
EXPORT_SYMBOL_GPL(zicbom_cmo_ops);

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
			      enum dma_data_direction dir)
{
	void *vaddr = phys_to_virt(paddr);

	switch (dir) {
	case DMA_TO_DEVICE:
		riscv_dma_noncoherent_clean(vaddr, size);
		break;
	case DMA_FROM_DEVICE:
		riscv_dma_noncoherent_clean(vaddr, size);
		break;
	case DMA_BIDIRECTIONAL:
		riscv_dma_noncoherent_flush(vaddr, size);
		break;
	default:
		break;
	}
}

void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
			   enum dma_data_direction dir)
{
	void *vaddr = phys_to_virt(paddr);

	switch (dir) {
	case DMA_TO_DEVICE:
		break;
	case DMA_FROM_DEVICE:
	case DMA_BIDIRECTIONAL:
		riscv_dma_noncoherent_flush(vaddr, size);
		break;
	default:
		break;
	}
}

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	void *flush_addr = page_address(page);

	riscv_dma_noncoherent_flush(flush_addr, size);
}

void arch_setup_dma_ops(struct device *dev, u64 dma_base, u64 size,
		const struct iommu_ops *iommu, bool coherent)
{
	WARN_TAINT(!coherent && riscv_cbom_block_size > ARCH_DMA_MINALIGN,
		   TAINT_CPU_OUT_OF_SPEC,
		   "%s %s: ARCH_DMA_MINALIGN smaller than riscv,cbom-block-size (%d < %d)",
		   dev_driver_string(dev), dev_name(dev),
		   ARCH_DMA_MINALIGN, riscv_cbom_block_size);

	WARN_TAINT(!coherent && !noncoherent_supported, TAINT_CPU_OUT_OF_SPEC,
		   "%s %s: device non-coherent but no non-coherent operations supported",
		   dev_driver_string(dev), dev_name(dev));

	dev->dma_coherent = coherent;
}

void riscv_noncoherent_supported(void)
{
	WARN(!riscv_cbom_block_size,
	     "Non-coherent DMA support enabled without a block size\n");
	noncoherent_supported = true;
}

void riscv_noncoherent_register_cache_ops(struct riscv_cache_ops *ops)
{
	if (!ops)
		return;

	noncoherent_cache_ops = *ops;
}
EXPORT_SYMBOL_GPL(riscv_noncoherent_register_cache_ops);
