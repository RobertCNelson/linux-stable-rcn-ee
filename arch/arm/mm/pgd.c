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
#define __pgd_alloc()	(pgd_t *)__get_free_pages(GFP_KERNEL | __GFP_REPEAT, 2)
#define __pgd_free(pgd)	free_pages((unsigned long)pgd, 2)
#endif

#ifdef CONFIG_IPIPE

/*
 * Provide support for pinning the PTEs referencing kernel mappings in
 * the current memory context, so that we don't get minor faults when
 * treading over kernel memory.  For this we need to maintain a map of
 * active PGDs.
 *
 * CAUTION: when LPAE is enabled, 1st level tables are small enough
 * (PTRS_PER_PGD for 3-level translation) that multiple PGDs can fit
 * into a single page, so we can't maintain our meta-data directly
 * into the struct page matching each PGD. Instead, we maintain this
 * data in a separate rbtree indexing the PGDs. To keep the
 * implementation straightforward, we also use this approach when LPAE
 * is disabled.
 */

#include <linux/rbtree.h>

static DEFINE_SPINLOCK(pgd_index_lock);
static struct rb_root pgd_table = RB_ROOT;

struct pgd_holder {
	pgd_t *pgd;
	struct rb_node rb;
};

#define pgd_table_lock(__flags)		\
	spin_lock_irqsave(&pgd_index_lock, __flags)

#define pgd_table_unlock(__flags)	\
	spin_unlock_irqrestore(&pgd_index_lock, __flags)

static inline struct pgd_holder *pgd_holder_alloc(pgd_t *pgd)
{
	struct pgd_holder *h;

	h = kmalloc(sizeof(*h), GFP_KERNEL);
	if (h == NULL)
		return NULL;

	h->pgd = pgd;

	return h;
}

static inline void pgd_holder_insert(struct pgd_holder *h)
{
	struct rb_node **rbp = &pgd_table.rb_node, *parent = NULL;
	struct pgd_holder *p;

	while (*rbp) {
		p = rb_entry(*rbp, struct pgd_holder, rb);
		parent = *rbp;
		if ((unsigned long)h->pgd < (unsigned long)p->pgd)
			rbp = &(*rbp)->rb_left;
		else {
			BUG_ON(h->pgd == p->pgd);
			rbp = &(*rbp)->rb_right;
		}
	}

  	rb_link_node(&h->rb, parent, rbp);
  	rb_insert_color(&h->rb, &pgd_table);
}

static inline void pgd_holder_free(pgd_t *pgd)
{
	struct rb_node *node = pgd_table.rb_node;
	struct pgd_holder *p;
	unsigned long flags;

	pgd_table_lock(flags);

	while (node) {
		p = rb_entry(node, struct pgd_holder, rb);
		if ((unsigned long)pgd < (unsigned long)p->pgd)
			node = node->rb_left;
		else if ((unsigned long)pgd > (unsigned long)p->pgd)
			node = node->rb_right;
		else {
			rb_erase(node, &pgd_table);
			pgd_table_unlock(flags);
			kfree(p);
			return;
		}
	}

	pgd_table_unlock(flags);

	BUG();
}

static void pin_k_mapping(pgd_t *pgd, unsigned long addr)
{
	unsigned int index = pgd_index(addr);
	pud_t *pud, *pud_k;
	pmd_t *pmd, *pmd_k;
	pgd_t *pgd_k;

	pgd += index;
	pgd_k = init_mm.pgd + index;

	if (!pgd_present(*pgd))
		set_pgd(pgd, *pgd_k);

	pud = pud_offset(pgd, addr);
	pud_k = pud_offset(pgd_k, addr);

	if (!pud_present(*pud))
		set_pud(pud, *pud_k);

	pmd   = pmd_offset(pud, addr);
	pmd_k = pmd_offset(pud_k, addr);

	copy_pmd(pmd, pmd_k);
}

void __ipipe_pin_range_globally(unsigned long start, unsigned long end)
{
	unsigned long next, addr = start;
	struct pgd_holder *h;
	unsigned long flags;
	struct rb_node *rb;

	do {
		next = pgd_addr_end(addr, end);
		pgd_table_lock(flags);
		for (rb = rb_first(&pgd_table); rb; rb = rb_next(rb)) {
			h = rb_entry(rb, struct pgd_holder, rb);
			pin_k_mapping(h->pgd, addr);
		}
		pgd_table_unlock(flags);

	} while (addr = next, addr != end);
}

#else  /* !CONFIG_IPIPE */

#define pgd_table_lock(__flags)		do { (void)(__flags); } while (0)
#define pgd_table_unlock(__flags)	do { (void)(__flags); } while (0)

static inline struct pgd_holder *pgd_holder_alloc(pgd_t *pgd)
{
	return NULL;
}

static inline void pgd_holder_insert(struct pgd_holder *h) { }

static inline void pgd_holder_free(pgd_t *pgd) { }

#endif  /* !CONFIG_IPIPE */

/*
 * need to get a 16k page for level 1
 */
pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *new_pgd, *init_pgd;
	pud_t *new_pud, *init_pud;
	pmd_t *new_pmd, *init_pmd;
	pte_t *new_pte, *init_pte;
	struct pgd_holder *h;
	unsigned long flags;

	new_pgd = __pgd_alloc();
	if (!new_pgd)
		goto no_pgd;

	memset(new_pgd, 0, USER_PTRS_PER_PGD * sizeof(pgd_t));

	/*
	 * Copy over the kernel and IO PGD entries
	 */
	init_pgd = pgd_offset_k(0);
	h = pgd_holder_alloc(new_pgd);
	pgd_table_lock(flags);
	memcpy(new_pgd + USER_PTRS_PER_PGD, init_pgd + USER_PTRS_PER_PGD,
		       (PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));

	pgd_holder_insert(h);
	pgd_table_unlock(flags);
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
		set_pte_ext(new_pte + 0, init_pte[0], 0);
		set_pte_ext(new_pte + 1, init_pte[1], 0);
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
	pgd_holder_free(pgd);

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
