// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 Heiko Stuebner <heiko@sntech.de>
 */

#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <asm/dma-noncoherent.h>
#include <asm/alternative.h>
#include <asm/cacheflush.h>
#include <asm/errata_list.h>
#include <asm/patch.h>
#include <asm/vendorid_list.h>

/*
 * dcache.ipa rs1 (invalidate, physical address)
 * | 31 - 25 | 24 - 20 | 19 - 15 | 14 - 12 | 11 - 7 | 6 - 0 |
 *   0000001    01010      rs1       000      00000  0001011
 * dache.iva rs1 (invalida, virtual address)
 *   0000001    00110      rs1       000      00000  0001011
 *
 * dcache.cpa rs1 (clean, physical address)
 * | 31 - 25 | 24 - 20 | 19 - 15 | 14 - 12 | 11 - 7 | 6 - 0 |
 *   0000001    01001      rs1       000      00000  0001011
 * dcache.cva rs1 (clean, virtual address)
 *   0000001    00100      rs1       000      00000  0001011
 *
 * dcache.cipa rs1 (clean then invalidate, physical address)
 * | 31 - 25 | 24 - 20 | 19 - 15 | 14 - 12 | 11 - 7 | 6 - 0 |
 *   0000001    01011      rs1       000      00000  0001011
 * dcache.civa rs1 (... virtual address)
 *   0000001    00111      rs1       000      00000  0001011
 *
 * sync.s (make sure all cache operations finished)
 * | 31 - 25 | 24 - 20 | 19 - 15 | 14 - 12 | 11 - 7 | 6 - 0 |
 *   0000000    11001     00000      000      00000  0001011
 */
#define THEAD_inval_A0	".long 0x0265000b"
#define THEAD_clean_A0	".long 0x0245000b"
#define THEAD_flush_A0	".long 0x0275000b"
#define THEAD_SYNC_S	".long 0x0190000b"

#define THEAD_CMO_OP(_op, _start, _size, _cachesize)				\
	asm volatile("mv a0, %1\n\t"						\
		     "j 2f\n\t"							\
		     "3:\n\t"							\
		     THEAD_##_op##_A0 "\n\t"					\
		     "add a0, a0, %0\n\t"					\
		     "2:\n\t"							\
		     "bltu a0, %2, 3b\n\t"					\
		     THEAD_SYNC_S						\
		     : : "r"(_cachesize),					\
			 "r"((unsigned long)(_start) & ~((_cachesize) - 1UL)),	\
			 "r"((unsigned long)(_start) + (_size))			\
		     : "a0")

static void thead_cmo_clean_range(unsigned long addr, unsigned long size)
{
	THEAD_CMO_OP(clean, addr, size, riscv_cbom_block_size);
}

static void thead_cmo_flush_range(unsigned long addr, unsigned long size)
{
	THEAD_CMO_OP(flush, addr, size, riscv_cbom_block_size);
}

static void thead_cmo_inval_range(unsigned long addr, unsigned long size)
{
	THEAD_CMO_OP(inval, addr, size, riscv_cbom_block_size);
}

static struct riscv_cache_ops thead_cmo_ops __maybe_unused = {
	.clean_range = thead_cmo_clean_range,
	.inval_range = thead_cmo_inval_range,
	.flush_range = thead_cmo_flush_range,
};

static bool errata_probe_pbmt(unsigned int stage,
			      unsigned long arch_id, unsigned long impid)
{
	if (!IS_ENABLED(CONFIG_ERRATA_THEAD_PBMT))
		return false;

	if (arch_id != 0 || impid != 0)
		return false;

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT ||
	    stage == RISCV_ALTERNATIVES_MODULE)
		return true;

	return false;
}

static bool errata_probe_cmo(unsigned int stage,
			     unsigned long arch_id, unsigned long impid)
{
	struct riscv_cache_ops thead_cmo_ops;

	if (!IS_ENABLED(CONFIG_ERRATA_THEAD_CMO))
		return false;

	if (arch_id != 0 || impid != 0)
		return false;

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT)
		return false;

	riscv_cbom_block_size = L1_CACHE_BYTES;
	riscv_noncoherent_supported();

	memset(&thead_cmo_ops, 0x0, sizeof(thead_cmo_ops));
	riscv_noncoherent_register_cache_ops(&thead_cmo_ops);

	return true;
}

static u32 thead_errata_probe(unsigned int stage,
			      unsigned long archid, unsigned long impid)
{
	u32 cpu_req_errata = 0;

	if (errata_probe_pbmt(stage, archid, impid))
		cpu_req_errata |= BIT(ERRATA_THEAD_PBMT);

	if (errata_probe_cmo(stage, archid, impid))
		cpu_req_errata |= BIT(ERRATA_THEAD_CMO);

	return cpu_req_errata;
}

void __init_or_module thead_errata_patch_func(struct alt_entry *begin, struct alt_entry *end,
					      unsigned long archid, unsigned long impid,
					      unsigned int stage)
{
	struct alt_entry *alt;
	u32 cpu_req_errata = thead_errata_probe(stage, archid, impid);
	u32 tmp;

	for (alt = begin; alt < end; alt++) {
		if (alt->vendor_id != THEAD_VENDOR_ID)
			continue;
		if (alt->errata_id >= ERRATA_THEAD_NUMBER)
			continue;

		tmp = (1U << alt->errata_id);
		if (cpu_req_errata & tmp) {
			/* On vm-alternatives, the mmu isn't running yet */
			if (stage == RISCV_ALTERNATIVES_EARLY_BOOT) {
				memcpy((void *)__pa_symbol(alt->old_ptr),
				       (void *)__pa_symbol(alt->alt_ptr), alt->alt_len);
			} else {
				mutex_lock(&text_mutex);
				patch_text_nosync(alt->old_ptr, alt->alt_ptr, alt->alt_len);
				mutex_unlock(&text_mutex);
			}
		}
	}

	if (stage == RISCV_ALTERNATIVES_EARLY_BOOT)
		local_flush_icache_all();
}
