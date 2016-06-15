/*
 *  linux/arch/arm/mm/pgd.c
 *
 *  Copyright (C) 1998-2005 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/slab.h>

#include <asm/cp15.h>
#include <asm/pgalloc.h>
#include <asm/page.h>
#include <asm/tlbflush.h>

#include "mm.h"

#ifdef CONFIG_ARM_LPAE
#define __pgd_alloc()	kmalloc(PTRS_PER_PGD * sizeof(pgd_t), GFP_KERNEL)
#define __pgd_free(pgd)	kfree(pgd)
#else
#define __pgd_alloc()	(pgd_t *)__get_free_pages(GFP_KERNEL, 2)
#define __pgd_free(pgd)	free_pages((unsigned long)pgd, 2)
#endif

#define FIRST_KERNEL_PGD_NR	(FIRST_USER_PGD_NR + USER_PTRS_PER_PGD)

#ifdef CONFIG_IPIPE
/* Copied from arch/i386/mm/pgdtable.c, maintains the list of pgds for the
   implementation of ipipe_pin_range_globally in arch/arm/mm/fault.c. */
DEFINE_SPINLOCK(pgd_lock);
struct page *pgd_list;

#define pgd_list_lock(flags) spin_lock_irqsave(&pgd_lock, flags)
#define pgd_list_unlock(flags) spin_unlock_irqrestore(&pgd_lock, flags)

static inline void pgd_list_add(pgd_t *pgd)
{
	struct page *page = virt_to_page(pgd);
	page->index = (unsigned long)pgd_list;
	if (pgd_list)
		set_page_private(pgd_list, (unsigned long)&page->index);
	pgd_list = page;
	set_page_private(page, (unsigned long)&pgd_list);
}

static inline void pgd_list_del(pgd_t *pgd)
{
	struct page *next, **pprev, *page = virt_to_page(pgd);
	next = (struct page *)page->index;
	pprev = (struct page **)page_private(page);
	*pprev = next;
	if (next)
		set_page_private(next, (unsigned long)pprev);
}
#else /* !CONFIG_IPIPE */
#define pgd_list_lock(flags) ((void) (flags))
#define pgd_list_unlock(flags) ((void) (flags))
#define pgd_list_add(pgd) do { } while (0)
#define pgd_list_del(pgd) do { } while (0)
#endif /* !CONFIG_IPIPE */

/*
 * need to get a 16k page for level 1
 */
pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *new_pgd, *init_pgd;
	pud_t *new_pud, *init_pud;
	pmd_t *new_pmd, *init_pmd;
	pte_t *new_pte, *init_pte;
	unsigned long flags;

	new_pgd = __pgd_alloc();
	if (!new_pgd)
		goto no_pgd;

	memset(new_pgd, 0, USER_PTRS_PER_PGD * sizeof(pgd_t));

	/*
	 * Copy over the kernel and IO PGD entries
	 */
	init_pgd = pgd_offset_k(0);
	pgd_list_lock(flags);
	memcpy(new_pgd + USER_PTRS_PER_PGD, init_pgd + USER_PTRS_PER_PGD,
		       (PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
	pgd_list_add(new_pgd);
	pgd_list_unlock(flags);

	clean_dcache_area(new_pgd, PTRS_PER_PGD * sizeof(pgd_t));

#ifdef CONFIG_ARM_LPAE
	/*
	 * Allocate PMD table for modules and pkmap mappings.
	 */
	new_pud = pud_alloc(mm, new_pgd + pgd_index(MODULES_VADDR),
			    MODULES_VADDR);
	if (!new_pud)
		goto no_pud;

	new_pmd = pmd_alloc(mm, new_pud, 0);
	if (!new_pmd)
		goto no_pmd;
#endif

	if (!vectors_high()) {
#ifdef CONFIG_ARM_FCSE
		/* FCSE does not work without high vectors. */
		BUG();
#endif /* CONFIG_ARM_FCSE */

		/*
		 * On ARM, first page must always be allocated since it
		 * contains the machine vectors. The vectors are always high
		 * with LPAE.
		 */
		new_pud = pud_alloc(mm, new_pgd, 0);
		if (!new_pud)
			goto no_pud;

		new_pmd = pmd_alloc(mm, new_pud, 0);
		if (!new_pmd)
			goto no_pmd;

		new_pte = pte_alloc_map(mm, NULL, new_pmd, 0);
		if (!new_pte)
			goto no_pte;

		init_pud = pud_offset(init_pgd, 0);
		init_pmd = pmd_offset(init_pud, 0);
		init_pte = pte_offset_map(init_pmd, 0);
		set_pte_ext(new_pte, *init_pte, 0);
		pte_unmap(init_pte);
		pte_unmap(new_pte);
	}

	return new_pgd;

no_pte:
	pmd_free(mm, new_pmd);
no_pmd:
	pud_free(mm, new_pud);
no_pud:
	__pgd_free(new_pgd);
no_pgd:
	return NULL;
}

void pgd_free(struct mm_struct *mm, pgd_t *pgd_base)
{
	unsigned long flags;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pgtable_t pte;

	if (!pgd_base)
		return;

	pgd = pgd_base + pgd_index(0);
	if (pgd_none_or_clear_bad(pgd))
		goto no_pgd;

	pud = pud_offset(pgd + pgd_index(fcse_va_to_mva(mm, 0)), 0);
	if (pud_none_or_clear_bad(pud))
		goto no_pud;

	pmd = pmd_offset(pud, 0);
	if (pmd_none_or_clear_bad(pmd))
		goto no_pmd;

	pte = pmd_pgtable(*pmd);
	pmd_clear(pmd);
	pte_free(mm, pte);
no_pmd:
	pud_clear(pud);
	pmd_free(mm, pmd);
no_pud:
	pgd_clear(pgd);
	pud_free(mm, pud);
no_pgd:
	pgd_list_lock(flags);
	pgd_list_del(pgd);
	pgd_list_unlock(flags);
#ifdef CONFIG_ARM_LPAE
	/*
	 * Free modules/pkmap or identity pmd tables.
	 */
	for (pgd = pgd_base; pgd < pgd_base + PTRS_PER_PGD; pgd++) {
		if (pgd_none_or_clear_bad(pgd))
			continue;
		if (pgd_val(*pgd) & L_PGD_SWAPPER)
			continue;
		pud = pud_offset(pgd, 0);
		if (pud_none_or_clear_bad(pud))
			continue;
		pmd = pmd_offset(pud, 0);
		pud_clear(pud);
		pmd_free(mm, pmd);
		pgd_clear(pgd);
		pud_free(mm, pud);

	}
#endif
	__pgd_free(pgd_base);
}
