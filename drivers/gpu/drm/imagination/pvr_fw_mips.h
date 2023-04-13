/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_FW_MIPS_H__
#define __PVR_FW_MIPS_H__

#include <linux/types.h>

/* Forward declaration from pvr_gem.h. */
struct pvr_gem_object;

/**
 * struct pvr_fw_mips_data - MIPS-specific data
 */
struct pvr_fw_mips_data {
	/** @pt_obj: Object representing MIPS pagetable. */
	struct pvr_gem_object *pt_obj;

	/** @pt: Pointer to CPU mapping of MIPS pagetable. */
	u32 *pt;

	/** @boot_code_dma_addr: DMA address of MIPS boot code. */
	dma_addr_t boot_code_dma_addr;

	/** @boot_data_dma_addr: DMA address of MIPS boot data. */
	dma_addr_t boot_data_dma_addr;

	/** @exception_code_dma_addr: DMA address of MIPS exception code. */
	dma_addr_t exception_code_dma_addr;

	/** @cache_policy: Cache policy for this processor. */
	u32 cache_policy;

	/** @pfn_mask: PFN mask for MIPS pagetable. */
	u32 pfn_mask;
};

#endif /* __PVR_FW_MIPS_H__ */
