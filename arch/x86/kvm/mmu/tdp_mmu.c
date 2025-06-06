// SPDX-License-Identifier: GPL-2.0
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "mmu.h"
#include "mmu_internal.h"
#include "mmutrace.h"
#include "tdp_iter.h"
#include "tdp_mmu.h"
#include "spte.h"

#include <asm/cmpxchg.h>
#include <trace/events/kvm.h>

/* Initializes the TDP MMU for the VM, if enabled. */
void kvm_mmu_init_tdp_mmu(struct kvm *kvm)
{
	INIT_LIST_HEAD(&kvm->arch.tdp_mmu_roots);
	spin_lock_init(&kvm->arch.tdp_mmu_pages_lock);
}

/* Arbitrarily returns true so that this may be used in if statements. */
static __always_inline bool kvm_lockdep_assert_mmu_lock_held(struct kvm *kvm,
							     bool shared)
{
	if (shared)
		lockdep_assert_held_read(&kvm->mmu_lock);
	else
		lockdep_assert_held_write(&kvm->mmu_lock);

	return true;
}

void kvm_mmu_uninit_tdp_mmu(struct kvm *kvm)
{
	/*
	 * Invalidate all roots, which besides the obvious, schedules all roots
	 * for zapping and thus puts the TDP MMU's reference to each root, i.e.
	 * ultimately frees all roots.
	 */
	kvm_tdp_mmu_invalidate_roots(kvm, KVM_VALID_ROOTS);
	kvm_tdp_mmu_zap_invalidated_roots(kvm, false);

#ifdef CONFIG_KVM_PROVE_MMU
	KVM_MMU_WARN_ON(atomic64_read(&kvm->arch.tdp_mmu_pages));
#endif
	WARN_ON(!list_empty(&kvm->arch.tdp_mmu_roots));

	/*
	 * Ensure that all the outstanding RCU callbacks to free shadow pages
	 * can run before the VM is torn down.  Putting the last reference to
	 * zapped roots will create new callbacks.
	 */
	rcu_barrier();
}

static void tdp_mmu_free_sp(struct kvm_mmu_page *sp)
{
	free_page((unsigned long)sp->external_spt);
	free_page((unsigned long)sp->spt);
	kmem_cache_free(mmu_page_header_cache, sp);
}

/*
 * This is called through call_rcu in order to free TDP page table memory
 * safely with respect to other kernel threads that may be operating on
 * the memory.
 * By only accessing TDP MMU page table memory in an RCU read critical
 * section, and freeing it after a grace period, lockless access to that
 * memory won't use it after it is freed.
 */
static void tdp_mmu_free_sp_rcu_callback(struct rcu_head *head)
{
	struct kvm_mmu_page *sp = container_of(head, struct kvm_mmu_page,
					       rcu_head);

	tdp_mmu_free_sp(sp);
}

void kvm_tdp_mmu_put_root(struct kvm *kvm, struct kvm_mmu_page *root)
{
	if (!refcount_dec_and_test(&root->tdp_mmu_root_count))
		return;

	/*
	 * The TDP MMU itself holds a reference to each root until the root is
	 * explicitly invalidated, i.e. the final reference should be never be
	 * put for a valid root.
	 */
	KVM_BUG_ON(!is_tdp_mmu_page(root) || !root->role.invalid, kvm);

	spin_lock(&kvm->arch.tdp_mmu_pages_lock);
	list_del_rcu(&root->link);
	spin_unlock(&kvm->arch.tdp_mmu_pages_lock);
	call_rcu(&root->rcu_head, tdp_mmu_free_sp_rcu_callback);
}

static bool tdp_mmu_root_match(struct kvm_mmu_page *root,
			       enum kvm_tdp_mmu_root_types types)
{
	if (WARN_ON_ONCE(!(types & KVM_VALID_ROOTS)))
		return false;

	if (root->role.invalid && !(types & KVM_INVALID_ROOTS))
		return false;

	if (likely(!is_mirror_sp(root)))
		return types & KVM_DIRECT_ROOTS;
	return types & KVM_MIRROR_ROOTS;
}

/*
 * Returns the next root after @prev_root (or the first root if @prev_root is
 * NULL) that matches with @types.  A reference to the returned root is
 * acquired, and the reference to @prev_root is released (the caller obviously
 * must hold a reference to @prev_root if it's non-NULL).
 *
 * Roots that doesn't match with @types are skipped.
 *
 * Returns NULL if the end of tdp_mmu_roots was reached.
 */
static struct kvm_mmu_page *tdp_mmu_next_root(struct kvm *kvm,
					      struct kvm_mmu_page *prev_root,
					      enum kvm_tdp_mmu_root_types types)
{
	struct kvm_mmu_page *next_root;

	/*
	 * While the roots themselves are RCU-protected, fields such as
	 * role.invalid are protected by mmu_lock.
	 */
	lockdep_assert_held(&kvm->mmu_lock);

	rcu_read_lock();

	if (prev_root)
		next_root = list_next_or_null_rcu(&kvm->arch.tdp_mmu_roots,
						  &prev_root->link,
						  typeof(*prev_root), link);
	else
		next_root = list_first_or_null_rcu(&kvm->arch.tdp_mmu_roots,
						   typeof(*next_root), link);

	while (next_root) {
		if (tdp_mmu_root_match(next_root, types) &&
		    kvm_tdp_mmu_get_root(next_root))
			break;

		next_root = list_next_or_null_rcu(&kvm->arch.tdp_mmu_roots,
				&next_root->link, typeof(*next_root), link);
	}

	rcu_read_unlock();

	if (prev_root)
		kvm_tdp_mmu_put_root(kvm, prev_root);

	return next_root;
}

/*
 * Note: this iterator gets and puts references to the roots it iterates over.
 * This makes it safe to release the MMU lock and yield within the loop, but
 * if exiting the loop early, the caller must drop the reference to the most
 * recent root. (Unless keeping a live reference is desirable.)
 *
 * If shared is set, this function is operating under the MMU lock in read
 * mode.
 */
#define __for_each_tdp_mmu_root_yield_safe(_kvm, _root, _as_id, _types)	\
	for (_root = tdp_mmu_next_root(_kvm, NULL, _types);		\
	     ({ lockdep_assert_held(&(_kvm)->mmu_lock); }), _root;		\
	     _root = tdp_mmu_next_root(_kvm, _root, _types))		\
		if (_as_id >= 0 && kvm_mmu_page_as_id(_root) != _as_id) {	\
		} else

#define for_each_valid_tdp_mmu_root_yield_safe(_kvm, _root, _as_id)	\
	__for_each_tdp_mmu_root_yield_safe(_kvm, _root, _as_id, KVM_VALID_ROOTS)

#define for_each_tdp_mmu_root_yield_safe(_kvm, _root)			\
	for (_root = tdp_mmu_next_root(_kvm, NULL, KVM_ALL_ROOTS);		\
	     ({ lockdep_assert_held(&(_kvm)->mmu_lock); }), _root;	\
	     _root = tdp_mmu_next_root(_kvm, _root, KVM_ALL_ROOTS))

/*
 * Iterate over all TDP MMU roots.  Requires that mmu_lock be held for write,
 * the implication being that any flow that holds mmu_lock for read is
 * inherently yield-friendly and should use the yield-safe variant above.
 * Holding mmu_lock for write obviates the need for RCU protection as the list
 * is guaranteed to be stable.
 */
#define __for_each_tdp_mmu_root(_kvm, _root, _as_id, _types)			\
	list_for_each_entry(_root, &_kvm->arch.tdp_mmu_roots, link)		\
		if (kvm_lockdep_assert_mmu_lock_held(_kvm, false) &&		\
		    ((_as_id >= 0 && kvm_mmu_page_as_id(_root) != _as_id) ||	\
		     !tdp_mmu_root_match((_root), (_types)))) {			\
		} else

/*
 * Iterate over all TDP MMU roots in an RCU read-side critical section.
 * It is safe to iterate over the SPTEs under the root, but their values will
 * be unstable, so all writes must be atomic. As this routine is meant to be
 * used without holding the mmu_lock at all, any bits that are flipped must
 * be reflected in kvm_tdp_mmu_spte_need_atomic_write().
 */
#define for_each_tdp_mmu_root_rcu(_kvm, _root, _as_id, _types)			\
	list_for_each_entry_rcu(_root, &_kvm->arch.tdp_mmu_roots, link)		\
		if ((_as_id >= 0 && kvm_mmu_page_as_id(_root) != _as_id) ||	\
		    !tdp_mmu_root_match((_root), (_types))) {			\
		} else

#define for_each_valid_tdp_mmu_root(_kvm, _root, _as_id)		\
	__for_each_tdp_mmu_root(_kvm, _root, _as_id, KVM_VALID_ROOTS)

static struct kvm_mmu_page *tdp_mmu_alloc_sp(struct kvm_vcpu *vcpu)
{
	struct kvm_mmu_page *sp;

	sp = kvm_mmu_memory_cache_alloc(&vcpu->arch.mmu_page_header_cache);
	sp->spt = kvm_mmu_memory_cache_alloc(&vcpu->arch.mmu_shadow_page_cache);

	return sp;
}

static void tdp_mmu_init_sp(struct kvm_mmu_page *sp, tdp_ptep_t sptep,
			    gfn_t gfn, union kvm_mmu_page_role role)
{
	INIT_LIST_HEAD(&sp->possible_nx_huge_page_link);

	set_page_private(virt_to_page(sp->spt), (unsigned long)sp);

	sp->role = role;
	sp->gfn = gfn;
	sp->ptep = sptep;
	sp->tdp_mmu_page = true;

	trace_kvm_mmu_get_page(sp, true);
}

static void tdp_mmu_init_child_sp(struct kvm_mmu_page *child_sp,
				  struct tdp_iter *iter)
{
	struct kvm_mmu_page *parent_sp;
	union kvm_mmu_page_role role;

	parent_sp = sptep_to_sp(rcu_dereference(iter->sptep));

	role = parent_sp->role;
	role.level--;

	tdp_mmu_init_sp(child_sp, iter->sptep, iter->gfn, role);
}

void kvm_tdp_mmu_alloc_root(struct kvm_vcpu *vcpu, bool mirror)
{
	struct kvm_mmu *mmu = vcpu->arch.mmu;
	union kvm_mmu_page_role role = mmu->root_role;
	int as_id = kvm_mmu_role_as_id(role);
	struct kvm *kvm = vcpu->kvm;
	struct kvm_mmu_page *root;

	if (mirror)
		role.is_mirror = true;

	/*
	 * Check for an existing root before acquiring the pages lock to avoid
	 * unnecessary serialization if multiple vCPUs are loading a new root.
	 * E.g. when bringing up secondary vCPUs, KVM will already have created
	 * a valid root on behalf of the primary vCPU.
	 */
	read_lock(&kvm->mmu_lock);

	for_each_valid_tdp_mmu_root_yield_safe(kvm, root, as_id) {
		if (root->role.word == role.word)
			goto out_read_unlock;
	}

	spin_lock(&kvm->arch.tdp_mmu_pages_lock);

	/*
	 * Recheck for an existing root after acquiring the pages lock, another
	 * vCPU may have raced ahead and created a new usable root.  Manually
	 * walk the list of roots as the standard macros assume that the pages
	 * lock is *not* held.  WARN if grabbing a reference to a usable root
	 * fails, as the last reference to a root can only be put *after* the
	 * root has been invalidated, which requires holding mmu_lock for write.
	 */
	list_for_each_entry(root, &kvm->arch.tdp_mmu_roots, link) {
		if (root->role.word == role.word &&
		    !WARN_ON_ONCE(!kvm_tdp_mmu_get_root(root)))
			goto out_spin_unlock;
	}

	root = tdp_mmu_alloc_sp(vcpu);
	tdp_mmu_init_sp(root, NULL, 0, role);

	/*
	 * TDP MMU roots are kept until they are explicitly invalidated, either
	 * by a memslot update or by the destruction of the VM.  Initialize the
	 * refcount to two; one reference for the vCPU, and one reference for
	 * the TDP MMU itself, which is held until the root is invalidated and
	 * is ultimately put by kvm_tdp_mmu_zap_invalidated_roots().
	 */
	refcount_set(&root->tdp_mmu_root_count, 2);
	list_add_rcu(&root->link, &kvm->arch.tdp_mmu_roots);

out_spin_unlock:
	spin_unlock(&kvm->arch.tdp_mmu_pages_lock);
out_read_unlock:
	read_unlock(&kvm->mmu_lock);
	/*
	 * Note, KVM_REQ_MMU_FREE_OBSOLETE_ROOTS will prevent entering the guest
	 * and actually consuming the root if it's invalidated after dropping
	 * mmu_lock, and the root can't be freed as this vCPU holds a reference.
	 */
	if (mirror) {
		mmu->mirror_root_hpa = __pa(root->spt);
	} else {
		mmu->root.hpa = __pa(root->spt);
		mmu->root.pgd = 0;
	}
}

static void handle_changed_spte(struct kvm *kvm, int as_id, gfn_t gfn,
				u64 old_spte, u64 new_spte, int level,
				bool shared);

static void tdp_account_mmu_page(struct kvm *kvm, struct kvm_mmu_page *sp)
{
	kvm_account_pgtable_pages((void *)sp->spt, +1);
#ifdef CONFIG_KVM_PROVE_MMU
	atomic64_inc(&kvm->arch.tdp_mmu_pages);
#endif
}

static void tdp_unaccount_mmu_page(struct kvm *kvm, struct kvm_mmu_page *sp)
{
	kvm_account_pgtable_pages((void *)sp->spt, -1);
#ifdef CONFIG_KVM_PROVE_MMU
	atomic64_dec(&kvm->arch.tdp_mmu_pages);
#endif
}

/**
 * tdp_mmu_unlink_sp() - Remove a shadow page from the list of used pages
 *
 * @kvm: kvm instance
 * @sp: the page to be removed
 */
static void tdp_mmu_unlink_sp(struct kvm *kvm, struct kvm_mmu_page *sp)
{
	tdp_unaccount_mmu_page(kvm, sp);

	if (!sp->nx_huge_page_disallowed)
		return;

	spin_lock(&kvm->arch.tdp_mmu_pages_lock);
	sp->nx_huge_page_disallowed = false;
	untrack_possible_nx_huge_page(kvm, sp);
	spin_unlock(&kvm->arch.tdp_mmu_pages_lock);
}

static void remove_external_spte(struct kvm *kvm, gfn_t gfn, u64 old_spte,
				 int level)
{
	kvm_pfn_t old_pfn = spte_to_pfn(old_spte);
	int ret;

	/*
	 * External (TDX) SPTEs are limited to PG_LEVEL_4K, and external
	 * PTs are removed in a special order, involving free_external_spt().
	 * But remove_external_spte() will be called on non-leaf PTEs via
	 * __tdp_mmu_zap_root(), so avoid the error the former would return
	 * in this case.
	 */
	if (!is_last_spte(old_spte, level))
		return;

	/* Zapping leaf spte is allowed only when write lock is held. */
	lockdep_assert_held_write(&kvm->mmu_lock);
	/* Because write lock is held, operation should success. */
	ret = kvm_x86_call(remove_external_spte)(kvm, gfn, level, old_pfn);
	KVM_BUG_ON(ret, kvm);
}

/**
 * handle_removed_pt() - handle a page table removed from the TDP structure
 *
 * @kvm: kvm instance
 * @pt: the page removed from the paging structure
 * @shared: This operation may not be running under the exclusive use
 *	    of the MMU lock and the operation must synchronize with other
 *	    threads that might be modifying SPTEs.
 *
 * Given a page table that has been removed from the TDP paging structure,
 * iterates through the page table to clear SPTEs and free child page tables.
 *
 * Note that pt is passed in as a tdp_ptep_t, but it does not need RCU
 * protection. Since this thread removed it from the paging structure,
 * this thread will be responsible for ensuring the page is freed. Hence the
 * early rcu_dereferences in the function.
 */
static void handle_removed_pt(struct kvm *kvm, tdp_ptep_t pt, bool shared)
{
	struct kvm_mmu_page *sp = sptep_to_sp(rcu_dereference(pt));
	int level = sp->role.level;
	gfn_t base_gfn = sp->gfn;
	int i;

	trace_kvm_mmu_prepare_zap_page(sp);

	tdp_mmu_unlink_sp(kvm, sp);

	for (i = 0; i < SPTE_ENT_PER_PAGE; i++) {
		tdp_ptep_t sptep = pt + i;
		gfn_t gfn = base_gfn + i * KVM_PAGES_PER_HPAGE(level);
		u64 old_spte;

		if (shared) {
			/*
			 * Set the SPTE to a nonpresent value that other
			 * threads will not overwrite. If the SPTE was
			 * already marked as frozen then another thread
			 * handling a page fault could overwrite it, so
			 * set the SPTE until it is set from some other
			 * value to the frozen SPTE value.
			 */
			for (;;) {
				old_spte = kvm_tdp_mmu_write_spte_atomic(sptep, FROZEN_SPTE);
				if (!is_frozen_spte(old_spte))
					break;
				cpu_relax();
			}
		} else {
			/*
			 * If the SPTE is not MMU-present, there is no backing
			 * page associated with the SPTE and so no side effects
			 * that need to be recorded, and exclusive ownership of
			 * mmu_lock ensures the SPTE can't be made present.
			 * Note, zapping MMIO SPTEs is also unnecessary as they
			 * are guarded by the memslots generation, not by being
			 * unreachable.
			 */
			old_spte = kvm_tdp_mmu_read_spte(sptep);
			if (!is_shadow_present_pte(old_spte))
				continue;

			/*
			 * Use the common helper instead of a raw WRITE_ONCE as
			 * the SPTE needs to be updated atomically if it can be
			 * modified by a different vCPU outside of mmu_lock.
			 * Even though the parent SPTE is !PRESENT, the TLB
			 * hasn't yet been flushed, and both Intel and AMD
			 * document that A/D assists can use upper-level PxE
			 * entries that are cached in the TLB, i.e. the CPU can
			 * still access the page and mark it dirty.
			 *
			 * No retry is needed in the atomic update path as the
			 * sole concern is dropping a Dirty bit, i.e. no other
			 * task can zap/remove the SPTE as mmu_lock is held for
			 * write.  Marking the SPTE as a frozen SPTE is not
			 * strictly necessary for the same reason, but using
			 * the frozen SPTE value keeps the shared/exclusive
			 * paths consistent and allows the handle_changed_spte()
			 * call below to hardcode the new value to FROZEN_SPTE.
			 *
			 * Note, even though dropping a Dirty bit is the only
			 * scenario where a non-atomic update could result in a
			 * functional bug, simply checking the Dirty bit isn't
			 * sufficient as a fast page fault could read the upper
			 * level SPTE before it is zapped, and then make this
			 * target SPTE writable, resume the guest, and set the
			 * Dirty bit between reading the SPTE above and writing
			 * it here.
			 */
			old_spte = kvm_tdp_mmu_write_spte(sptep, old_spte,
							  FROZEN_SPTE, level);
		}
		handle_changed_spte(kvm, kvm_mmu_page_as_id(sp), gfn,
				    old_spte, FROZEN_SPTE, level, shared);

		if (is_mirror_sp(sp)) {
			KVM_BUG_ON(shared, kvm);
			remove_external_spte(kvm, gfn, old_spte, level);
		}
	}

	if (is_mirror_sp(sp) &&
	    WARN_ON(kvm_x86_call(free_external_spt)(kvm, base_gfn, sp->role.level,
						    sp->external_spt))) {
		/*
		 * Failed to free page table page in mirror page table and
		 * there is nothing to do further.
		 * Intentionally leak the page to prevent the kernel from
		 * accessing the encrypted page.
		 */
		sp->external_spt = NULL;
	}

	call_rcu(&sp->rcu_head, tdp_mmu_free_sp_rcu_callback);
}

static void *get_external_spt(gfn_t gfn, u64 new_spte, int level)
{
	if (is_shadow_present_pte(new_spte) && !is_last_spte(new_spte, level)) {
		struct kvm_mmu_page *sp = spte_to_child_sp(new_spte);

		WARN_ON_ONCE(sp->role.level + 1 != level);
		WARN_ON_ONCE(sp->gfn != gfn);
		return sp->external_spt;
	}

	return NULL;
}

static int __must_check set_external_spte_present(struct kvm *kvm, tdp_ptep_t sptep,
						 gfn_t gfn, u64 old_spte,
						 u64 new_spte, int level)
{
	bool was_present = is_shadow_present_pte(old_spte);
	bool is_present = is_shadow_present_pte(new_spte);
	bool is_leaf = is_present && is_last_spte(new_spte, level);
	kvm_pfn_t new_pfn = spte_to_pfn(new_spte);
	int ret = 0;

	KVM_BUG_ON(was_present, kvm);

	lockdep_assert_held(&kvm->mmu_lock);
	/*
	 * We need to lock out other updates to the SPTE until the external
	 * page table has been modified. Use FROZEN_SPTE similar to
	 * the zapping case.
	 */
	if (!try_cmpxchg64(rcu_dereference(sptep), &old_spte, FROZEN_SPTE))
		return -EBUSY;

	/*
	 * Use different call to either set up middle level
	 * external page table, or leaf.
	 */
	if (is_leaf) {
		ret = kvm_x86_call(set_external_spte)(kvm, gfn, level, new_pfn);
	} else {
		void *external_spt = get_external_spt(gfn, new_spte, level);

		KVM_BUG_ON(!external_spt, kvm);
		ret = kvm_x86_call(link_external_spt)(kvm, gfn, level, external_spt);
	}
	if (ret)
		__kvm_tdp_mmu_write_spte(sptep, old_spte);
	else
		__kvm_tdp_mmu_write_spte(sptep, new_spte);
	return ret;
}

/**
 * handle_changed_spte - handle bookkeeping associated with an SPTE change
 * @kvm: kvm instance
 * @as_id: the address space of the paging structure the SPTE was a part of
 * @gfn: the base GFN that was mapped by the SPTE
 * @old_spte: The value of the SPTE before the change
 * @new_spte: The value of the SPTE after the change
 * @level: the level of the PT the SPTE is part of in the paging structure
 * @shared: This operation may not be running under the exclusive use of
 *	    the MMU lock and the operation must synchronize with other
 *	    threads that might be modifying SPTEs.
 *
 * Handle bookkeeping that might result from the modification of a SPTE.  Note,
 * dirty logging updates are handled in common code, not here (see make_spte()
 * and fast_pf_fix_direct_spte()).
 */
static void handle_changed_spte(struct kvm *kvm, int as_id, gfn_t gfn,
				u64 old_spte, u64 new_spte, int level,
				bool shared)
{
	bool was_present = is_shadow_present_pte(old_spte);
	bool is_present = is_shadow_present_pte(new_spte);
	bool was_leaf = was_present && is_last_spte(old_spte, level);
	bool is_leaf = is_present && is_last_spte(new_spte, level);
	bool pfn_changed = spte_to_pfn(old_spte) != spte_to_pfn(new_spte);

	WARN_ON_ONCE(level > PT64_ROOT_MAX_LEVEL);
	WARN_ON_ONCE(level < PG_LEVEL_4K);
	WARN_ON_ONCE(gfn & (KVM_PAGES_PER_HPAGE(level) - 1));

	/*
	 * If this warning were to trigger it would indicate that there was a
	 * missing MMU notifier or a race with some notifier handler.
	 * A present, leaf SPTE should never be directly replaced with another
	 * present leaf SPTE pointing to a different PFN. A notifier handler
	 * should be zapping the SPTE before the main MM's page table is
	 * changed, or the SPTE should be zeroed, and the TLBs flushed by the
	 * thread before replacement.
	 */
	if (was_leaf && is_leaf && pfn_changed) {
		pr_err("Invalid SPTE change: cannot replace a present leaf\n"
		       "SPTE with another present leaf SPTE mapping a\n"
		       "different PFN!\n"
		       "as_id: %d gfn: %llx old_spte: %llx new_spte: %llx level: %d",
		       as_id, gfn, old_spte, new_spte, level);

		/*
		 * Crash the host to prevent error propagation and guest data
		 * corruption.
		 */
		BUG();
	}

	if (old_spte == new_spte)
		return;

	trace_kvm_tdp_mmu_spte_changed(as_id, gfn, level, old_spte, new_spte);

	if (is_leaf)
		check_spte_writable_invariants(new_spte);

	/*
	 * The only times a SPTE should be changed from a non-present to
	 * non-present state is when an MMIO entry is installed/modified/
	 * removed. In that case, there is nothing to do here.
	 */
	if (!was_present && !is_present) {
		/*
		 * If this change does not involve a MMIO SPTE or frozen SPTE,
		 * it is unexpected. Log the change, though it should not
		 * impact the guest since both the former and current SPTEs
		 * are nonpresent.
		 */
		if (WARN_ON_ONCE(!is_mmio_spte(kvm, old_spte) &&
				 !is_mmio_spte(kvm, new_spte) &&
				 !is_frozen_spte(new_spte)))
			pr_err("Unexpected SPTE change! Nonpresent SPTEs\n"
			       "should not be replaced with another,\n"
			       "different nonpresent SPTE, unless one or both\n"
			       "are MMIO SPTEs, or the new SPTE is\n"
			       "a temporary frozen SPTE.\n"
			       "as_id: %d gfn: %llx old_spte: %llx new_spte: %llx level: %d",
			       as_id, gfn, old_spte, new_spte, level);
		return;
	}

	if (is_leaf != was_leaf)
		kvm_update_page_stats(kvm, level, is_leaf ? 1 : -1);

	/*
	 * Recursively handle child PTs if the change removed a subtree from
	 * the paging structure.  Note the WARN on the PFN changing without the
	 * SPTE being converted to a hugepage (leaf) or being zapped.  Shadow
	 * pages are kernel allocations and should never be migrated.
	 */
	if (was_present && !was_leaf &&
	    (is_leaf || !is_present || WARN_ON_ONCE(pfn_changed)))
		handle_removed_pt(kvm, spte_to_child_pt(old_spte, level), shared);
}

static inline int __must_check __tdp_mmu_set_spte_atomic(struct kvm *kvm,
							 struct tdp_iter *iter,
							 u64 new_spte)
{
	/*
	 * The caller is responsible for ensuring the old SPTE is not a FROZEN
	 * SPTE.  KVM should never attempt to zap or manipulate a FROZEN SPTE,
	 * and pre-checking before inserting a new SPTE is advantageous as it
	 * avoids unnecessary work.
	 */
	WARN_ON_ONCE(iter->yielded || is_frozen_spte(iter->old_spte));

	if (is_mirror_sptep(iter->sptep) && !is_frozen_spte(new_spte)) {
		int ret;

		/*
		 * Users of atomic zapping don't operate on mirror roots,
		 * so don't handle it and bug the VM if it's seen.
		 */
		if (KVM_BUG_ON(!is_shadow_present_pte(new_spte), kvm))
			return -EBUSY;

		ret = set_external_spte_present(kvm, iter->sptep, iter->gfn,
						iter->old_spte, new_spte, iter->level);
		if (ret)
			return ret;
	} else {
		u64 *sptep = rcu_dereference(iter->sptep);

		/*
		 * Note, fast_pf_fix_direct_spte() can also modify TDP MMU SPTEs
		 * and does not hold the mmu_lock.  On failure, i.e. if a
		 * different logical CPU modified the SPTE, try_cmpxchg64()
		 * updates iter->old_spte with the current value, so the caller
		 * operates on fresh data, e.g. if it retries
		 * tdp_mmu_set_spte_atomic()
		 */
		if (!try_cmpxchg64(sptep, &iter->old_spte, new_spte))
			return -EBUSY;
	}

	return 0;
}

/*
 * tdp_mmu_set_spte_atomic - Set a TDP MMU SPTE atomically
 * and handle the associated bookkeeping.  Do not mark the page dirty
 * in KVM's dirty bitmaps.
 *
 * If setting the SPTE fails because it has changed, iter->old_spte will be
 * refreshed to the current value of the spte.
 *
 * @kvm: kvm instance
 * @iter: a tdp_iter instance currently on the SPTE that should be set
 * @new_spte: The value the SPTE should be set to
 * Return:
 * * 0      - If the SPTE was set.
 * * -EBUSY - If the SPTE cannot be set. In this case this function will have
 *            no side-effects other than setting iter->old_spte to the last
 *            known value of the spte.
 */
static inline int __must_check tdp_mmu_set_spte_atomic(struct kvm *kvm,
						       struct tdp_iter *iter,
						       u64 new_spte)
{
	int ret;

	lockdep_assert_held_read(&kvm->mmu_lock);

	ret = __tdp_mmu_set_spte_atomic(kvm, iter, new_spte);
	if (ret)
		return ret;

	handle_changed_spte(kvm, iter->as_id, iter->gfn, iter->old_spte,
			    new_spte, iter->level, true);

	return 0;
}

/*
 * tdp_mmu_set_spte - Set a TDP MMU SPTE and handle the associated bookkeeping
 * @kvm:	      KVM instance
 * @as_id:	      Address space ID, i.e. regular vs. SMM
 * @sptep:	      Pointer to the SPTE
 * @old_spte:	      The current value of the SPTE
 * @new_spte:	      The new value that will be set for the SPTE
 * @gfn:	      The base GFN that was (or will be) mapped by the SPTE
 * @level:	      The level _containing_ the SPTE (its parent PT's level)
 *
 * Returns the old SPTE value, which _may_ be different than @old_spte if the
 * SPTE had voldatile bits.
 */
static u64 tdp_mmu_set_spte(struct kvm *kvm, int as_id, tdp_ptep_t sptep,
			    u64 old_spte, u64 new_spte, gfn_t gfn, int level)
{
	lockdep_assert_held_write(&kvm->mmu_lock);

	/*
	 * No thread should be using this function to set SPTEs to or from the
	 * temporary frozen SPTE value.
	 * If operating under the MMU lock in read mode, tdp_mmu_set_spte_atomic
	 * should be used. If operating under the MMU lock in write mode, the
	 * use of the frozen SPTE should not be necessary.
	 */
	WARN_ON_ONCE(is_frozen_spte(old_spte) || is_frozen_spte(new_spte));

	old_spte = kvm_tdp_mmu_write_spte(sptep, old_spte, new_spte, level);

	handle_changed_spte(kvm, as_id, gfn, old_spte, new_spte, level, false);

	/*
	 * Users that do non-atomic setting of PTEs don't operate on mirror
	 * roots, so don't handle it and bug the VM if it's seen.
	 */
	if (is_mirror_sptep(sptep)) {
		KVM_BUG_ON(is_shadow_present_pte(new_spte), kvm);
		remove_external_spte(kvm, gfn, old_spte, level);
	}

	return old_spte;
}

static inline void tdp_mmu_iter_set_spte(struct kvm *kvm, struct tdp_iter *iter,
					 u64 new_spte)
{
	WARN_ON_ONCE(iter->yielded);
	iter->old_spte = tdp_mmu_set_spte(kvm, iter->as_id, iter->sptep,
					  iter->old_spte, new_spte,
					  iter->gfn, iter->level);
}

#define tdp_root_for_each_pte(_iter, _kvm, _root, _start, _end)	\
	for_each_tdp_pte(_iter, _kvm, _root, _start, _end)

#define tdp_root_for_each_leaf_pte(_iter, _kvm, _root, _start, _end)	\
	tdp_root_for_each_pte(_iter, _kvm, _root, _start, _end)		\
		if (!is_shadow_present_pte(_iter.old_spte) ||		\
		    !is_last_spte(_iter.old_spte, _iter.level))		\
			continue;					\
		else

static inline bool __must_check tdp_mmu_iter_need_resched(struct kvm *kvm,
							  struct tdp_iter *iter)
{
	if (!need_resched() && !rwlock_needbreak(&kvm->mmu_lock))
		return false;

	/* Ensure forward progress has been made before yielding. */
	return iter->next_last_level_gfn != iter->yielded_gfn;
}

/*
 * Yield if the MMU lock is contended or this thread needs to return control
 * to the scheduler.
 *
 * If this function should yield and flush is set, it will perform a remote
 * TLB flush before yielding.
 *
 * If this function yields, iter->yielded is set and the caller must skip to
 * the next iteration, where tdp_iter_next() will reset the tdp_iter's walk
 * over the paging structures to allow the iterator to continue its traversal
 * from the paging structure root.
 *
 * Returns true if this function yielded.
 */
static inline bool __must_check tdp_mmu_iter_cond_resched(struct kvm *kvm,
							  struct tdp_iter *iter,
							  bool flush, bool shared)
{
	KVM_MMU_WARN_ON(iter->yielded);

	if (!tdp_mmu_iter_need_resched(kvm, iter))
		return false;

	if (flush)
		kvm_flush_remote_tlbs(kvm);

	rcu_read_unlock();

	if (shared)
		cond_resched_rwlock_read(&kvm->mmu_lock);
	else
		cond_resched_rwlock_write(&kvm->mmu_lock);

	rcu_read_lock();

	WARN_ON_ONCE(iter->gfn > iter->next_last_level_gfn);

	iter->yielded = true;
	return true;
}

static inline gfn_t tdp_mmu_max_gfn_exclusive(void)
{
	/*
	 * Bound TDP MMU walks at host.MAXPHYADDR.  KVM disallows memslots with
	 * a gpa range that would exceed the max gfn, and KVM does not create
	 * MMIO SPTEs for "impossible" gfns, instead sending such accesses down
	 * the slow emulation path every time.
	 */
	return kvm_mmu_max_gfn() + 1;
}

static void __tdp_mmu_zap_root(struct kvm *kvm, struct kvm_mmu_page *root,
			       bool shared, int zap_level)
{
	struct tdp_iter iter;

	for_each_tdp_pte_min_level_all(iter, root, zap_level) {
retry:
		if (tdp_mmu_iter_cond_resched(kvm, &iter, false, shared))
			continue;

		if (!is_shadow_present_pte(iter.old_spte))
			continue;

		if (iter.level > zap_level)
			continue;

		if (!shared)
			tdp_mmu_iter_set_spte(kvm, &iter, SHADOW_NONPRESENT_VALUE);
		else if (tdp_mmu_set_spte_atomic(kvm, &iter, SHADOW_NONPRESENT_VALUE))
			goto retry;
	}
}

static void tdp_mmu_zap_root(struct kvm *kvm, struct kvm_mmu_page *root,
			     bool shared)
{

	/*
	 * The root must have an elevated refcount so that it's reachable via
	 * mmu_notifier callbacks, which allows this path to yield and drop
	 * mmu_lock.  When handling an unmap/release mmu_notifier command, KVM
	 * must drop all references to relevant pages prior to completing the
	 * callback.  Dropping mmu_lock with an unreachable root would result
	 * in zapping SPTEs after a relevant mmu_notifier callback completes
	 * and lead to use-after-free as zapping a SPTE triggers "writeback" of
	 * dirty accessed bits to the SPTE's associated struct page.
	 */
	WARN_ON_ONCE(!refcount_read(&root->tdp_mmu_root_count));

	kvm_lockdep_assert_mmu_lock_held(kvm, shared);

	rcu_read_lock();

	/*
	 * Zap roots in multiple passes of decreasing granularity, i.e. zap at
	 * 4KiB=>2MiB=>1GiB=>root, in order to better honor need_resched() (all
	 * preempt models) or mmu_lock contention (full or real-time models).
	 * Zapping at finer granularity marginally increases the total time of
	 * the zap, but in most cases the zap itself isn't latency sensitive.
	 *
	 * If KVM is configured to prove the MMU, skip the 4KiB and 2MiB zaps
	 * in order to mimic the page fault path, which can replace a 1GiB page
	 * table with an equivalent 1GiB hugepage, i.e. can get saddled with
	 * zapping a 1GiB region that's fully populated with 4KiB SPTEs.  This
	 * allows verifying that KVM can safely zap 1GiB regions, e.g. without
	 * inducing RCU stalls, without relying on a relatively rare event
	 * (zapping roots is orders of magnitude more common).  Note, because
	 * zapping a SP recurses on its children, stepping down to PG_LEVEL_4K
	 * in the iterator itself is unnecessary.
	 */
	if (!IS_ENABLED(CONFIG_KVM_PROVE_MMU)) {
		__tdp_mmu_zap_root(kvm, root, shared, PG_LEVEL_4K);
		__tdp_mmu_zap_root(kvm, root, shared, PG_LEVEL_2M);
	}
	__tdp_mmu_zap_root(kvm, root, shared, PG_LEVEL_1G);
	__tdp_mmu_zap_root(kvm, root, shared, root->role.level);

	rcu_read_unlock();
}

bool kvm_tdp_mmu_zap_sp(struct kvm *kvm, struct kvm_mmu_page *sp)
{
	u64 old_spte;

	/*
	 * This helper intentionally doesn't allow zapping a root shadow page,
	 * which doesn't have a parent page table and thus no associated entry.
	 */
	if (WARN_ON_ONCE(!sp->ptep))
		return false;

	old_spte = kvm_tdp_mmu_read_spte(sp->ptep);
	if (WARN_ON_ONCE(!is_shadow_present_pte(old_spte)))
		return false;

	tdp_mmu_set_spte(kvm, kvm_mmu_page_as_id(sp), sp->ptep, old_spte,
			 SHADOW_NONPRESENT_VALUE, sp->gfn, sp->role.level + 1);

	return true;
}

/*
 * If can_yield is true, will release the MMU lock and reschedule if the
 * scheduler needs the CPU or there is contention on the MMU lock. If this
 * function cannot yield, it will not release the MMU lock or reschedule and
 * the caller must ensure it does not supply too large a GFN range, or the
 * operation can cause a soft lockup.
 */
static bool tdp_mmu_zap_leafs(struct kvm *kvm, struct kvm_mmu_page *root,
			      gfn_t start, gfn_t end, bool can_yield, bool flush)
{
	struct tdp_iter iter;

	end = min(end, tdp_mmu_max_gfn_exclusive());

	lockdep_assert_held_write(&kvm->mmu_lock);

	rcu_read_lock();

	for_each_tdp_pte_min_level(iter, kvm, root, PG_LEVEL_4K, start, end) {
		if (can_yield &&
		    tdp_mmu_iter_cond_resched(kvm, &iter, flush, false)) {
			flush = false;
			continue;
		}

		if (!is_shadow_present_pte(iter.old_spte) ||
		    !is_last_spte(iter.old_spte, iter.level))
			continue;

		tdp_mmu_iter_set_spte(kvm, &iter, SHADOW_NONPRESENT_VALUE);

		/*
		 * Zappings SPTEs in invalid roots doesn't require a TLB flush,
		 * see kvm_tdp_mmu_zap_invalidated_roots() for details.
		 */
		if (!root->role.invalid)
			flush = true;
	}

	rcu_read_unlock();

	/*
	 * Because this flow zaps _only_ leaf SPTEs, the caller doesn't need
	 * to provide RCU protection as no 'struct kvm_mmu_page' will be freed.
	 */
	return flush;
}

/*
 * Zap leaf SPTEs for the range of gfns, [start, end), for all *VALID** roots.
 * Returns true if a TLB flush is needed before releasing the MMU lock, i.e. if
 * one or more SPTEs were zapped since the MMU lock was last acquired.
 */
bool kvm_tdp_mmu_zap_leafs(struct kvm *kvm, gfn_t start, gfn_t end, bool flush)
{
	struct kvm_mmu_page *root;

	lockdep_assert_held_write(&kvm->mmu_lock);
	for_each_valid_tdp_mmu_root_yield_safe(kvm, root, -1)
		flush = tdp_mmu_zap_leafs(kvm, root, start, end, true, flush);

	return flush;
}

void kvm_tdp_mmu_zap_all(struct kvm *kvm)
{
	struct kvm_mmu_page *root;

	/*
	 * Zap all direct roots, including invalid direct roots, as all direct
	 * SPTEs must be dropped before returning to the caller. For TDX, mirror
	 * roots don't need handling in response to the mmu notifier (the caller).
	 *
	 * Zap directly even if the root is also being zapped by a concurrent
	 * "fast zap".  Walking zapped top-level SPTEs isn't all that expensive
	 * and mmu_lock is already held, which means the other thread has yielded.
	 *
	 * A TLB flush is unnecessary, KVM zaps everything if and only the VM
	 * is being destroyed or the userspace VMM has exited.  In both cases,
	 * KVM_RUN is unreachable, i.e. no vCPUs will ever service the request.
	 */
	lockdep_assert_held_write(&kvm->mmu_lock);
	__for_each_tdp_mmu_root_yield_safe(kvm, root, -1,
					   KVM_DIRECT_ROOTS | KVM_INVALID_ROOTS)
		tdp_mmu_zap_root(kvm, root, false);
}

/*
 * Zap all invalidated roots to ensure all SPTEs are dropped before the "fast
 * zap" completes.
 */
void kvm_tdp_mmu_zap_invalidated_roots(struct kvm *kvm, bool shared)
{
	struct kvm_mmu_page *root;

	if (shared)
		read_lock(&kvm->mmu_lock);
	else
		write_lock(&kvm->mmu_lock);

	for_each_tdp_mmu_root_yield_safe(kvm, root) {
		if (!root->tdp_mmu_scheduled_root_to_zap)
			continue;

		root->tdp_mmu_scheduled_root_to_zap = false;
		KVM_BUG_ON(!root->role.invalid, kvm);

		/*
		 * A TLB flush is not necessary as KVM performs a local TLB
		 * flush when allocating a new root (see kvm_mmu_load()), and
		 * when migrating a vCPU to a different pCPU.  Note, the local
		 * TLB flush on reuse also invalidates paging-structure-cache
		 * entries, i.e. TLB entries for intermediate paging structures,
		 * that may be zapped, as such entries are associated with the
		 * ASID on both VMX and SVM.
		 */
		tdp_mmu_zap_root(kvm, root, shared);

		/*
		 * The referenced needs to be put *after* zapping the root, as
		 * the root must be reachable by mmu_notifiers while it's being
		 * zapped
		 */
		kvm_tdp_mmu_put_root(kvm, root);
	}

	if (shared)
		read_unlock(&kvm->mmu_lock);
	else
		write_unlock(&kvm->mmu_lock);
}

/*
 * Mark each TDP MMU root as invalid to prevent vCPUs from reusing a root that
 * is about to be zapped, e.g. in response to a memslots update.  The actual
 * zapping is done separately so that it happens with mmu_lock with read,
 * whereas invalidating roots must be done with mmu_lock held for write (unless
 * the VM is being destroyed).
 *
 * Note, kvm_tdp_mmu_zap_invalidated_roots() is gifted the TDP MMU's reference.
 * See kvm_tdp_mmu_alloc_root().
 */
void kvm_tdp_mmu_invalidate_roots(struct kvm *kvm,
				  enum kvm_tdp_mmu_root_types root_types)
{
	struct kvm_mmu_page *root;

	/*
	 * Invalidating invalid roots doesn't make sense, prevent developers from
	 * having to think about it.
	 */
	if (WARN_ON_ONCE(root_types & KVM_INVALID_ROOTS))
		root_types &= ~KVM_INVALID_ROOTS;

	/*
	 * mmu_lock must be held for write to ensure that a root doesn't become
	 * invalid while there are active readers (invalidating a root while
	 * there are active readers may or may not be problematic in practice,
	 * but it's uncharted territory and not supported).
	 *
	 * Waive the assertion if there are no users of @kvm, i.e. the VM is
	 * being destroyed after all references have been put, or if no vCPUs
	 * have been created (which means there are no roots), i.e. the VM is
	 * being destroyed in an error path of KVM_CREATE_VM.
	 */
	if (IS_ENABLED(CONFIG_PROVE_LOCKING) &&
	    refcount_read(&kvm->users_count) && kvm->created_vcpus)
		lockdep_assert_held_write(&kvm->mmu_lock);

	/*
	 * As above, mmu_lock isn't held when destroying the VM!  There can't
	 * be other references to @kvm, i.e. nothing else can invalidate roots
	 * or get/put references to roots.
	 */
	list_for_each_entry(root, &kvm->arch.tdp_mmu_roots, link) {
		if (!tdp_mmu_root_match(root, root_types))
			continue;

		/*
		 * Note, invalid roots can outlive a memslot update!  Invalid
		 * roots must be *zapped* before the memslot update completes,
		 * but a different task can acquire a reference and keep the
		 * root alive after its been zapped.
		 */
		if (!root->role.invalid) {
			root->tdp_mmu_scheduled_root_to_zap = true;
			root->role.invalid = true;
		}
	}
}

/*
 * Installs a last-level SPTE to handle a TDP page fault.
 * (NPT/EPT violation/misconfiguration)
 */
static int tdp_mmu_map_handle_target_level(struct kvm_vcpu *vcpu,
					  struct kvm_page_fault *fault,
					  struct tdp_iter *iter)
{
	struct kvm_mmu_page *sp = sptep_to_sp(rcu_dereference(iter->sptep));
	u64 new_spte;
	int ret = RET_PF_FIXED;
	bool wrprot = false;

	if (WARN_ON_ONCE(sp->role.level != fault->goal_level))
		return RET_PF_RETRY;

	if (is_shadow_present_pte(iter->old_spte) &&
	    (fault->prefetch || is_access_allowed(fault, iter->old_spte)) &&
	    is_last_spte(iter->old_spte, iter->level)) {
		WARN_ON_ONCE(fault->pfn != spte_to_pfn(iter->old_spte));
		return RET_PF_SPURIOUS;
	}

	if (unlikely(!fault->slot))
		new_spte = make_mmio_spte(vcpu, iter->gfn, ACC_ALL);
	else
		wrprot = make_spte(vcpu, sp, fault->slot, ACC_ALL, iter->gfn,
				   fault->pfn, iter->old_spte, fault->prefetch,
				   false, fault->map_writable, &new_spte);

	if (new_spte == iter->old_spte)
		ret = RET_PF_SPURIOUS;
	else if (tdp_mmu_set_spte_atomic(vcpu->kvm, iter, new_spte))
		return RET_PF_RETRY;
	else if (is_shadow_present_pte(iter->old_spte) &&
		 (!is_last_spte(iter->old_spte, iter->level) ||
		  WARN_ON_ONCE(leaf_spte_change_needs_tlb_flush(iter->old_spte, new_spte))))
		kvm_flush_remote_tlbs_gfn(vcpu->kvm, iter->gfn, iter->level);

	/*
	 * If the page fault was caused by a write but the page is write
	 * protected, emulation is needed. If the emulation was skipped,
	 * the vCPU would have the same fault again.
	 */
	if (wrprot && fault->write)
		ret = RET_PF_WRITE_PROTECTED;

	/* If a MMIO SPTE is installed, the MMIO will need to be emulated. */
	if (unlikely(is_mmio_spte(vcpu->kvm, new_spte))) {
		vcpu->stat.pf_mmio_spte_created++;
		trace_mark_mmio_spte(rcu_dereference(iter->sptep), iter->gfn,
				     new_spte);
		ret = RET_PF_EMULATE;
	} else {
		trace_kvm_mmu_set_spte(iter->level, iter->gfn,
				       rcu_dereference(iter->sptep));
	}

	return ret;
}

/*
 * tdp_mmu_link_sp - Replace the given spte with an spte pointing to the
 * provided page table.
 *
 * @kvm: kvm instance
 * @iter: a tdp_iter instance currently on the SPTE that should be set
 * @sp: The new TDP page table to install.
 * @shared: This operation is running under the MMU lock in read mode.
 *
 * Returns: 0 if the new page table was installed. Non-0 if the page table
 *          could not be installed (e.g. the atomic compare-exchange failed).
 */
static int tdp_mmu_link_sp(struct kvm *kvm, struct tdp_iter *iter,
			   struct kvm_mmu_page *sp, bool shared)
{
	u64 spte = make_nonleaf_spte(sp->spt, !kvm_ad_enabled);
	int ret = 0;

	if (shared) {
		ret = tdp_mmu_set_spte_atomic(kvm, iter, spte);
		if (ret)
			return ret;
	} else {
		tdp_mmu_iter_set_spte(kvm, iter, spte);
	}

	tdp_account_mmu_page(kvm, sp);

	return 0;
}

static int tdp_mmu_split_huge_page(struct kvm *kvm, struct tdp_iter *iter,
				   struct kvm_mmu_page *sp, bool shared);

/*
 * Handle a TDP page fault (NPT/EPT violation/misconfiguration) by installing
 * page tables and SPTEs to translate the faulting guest physical address.
 */
int kvm_tdp_mmu_map(struct kvm_vcpu *vcpu, struct kvm_page_fault *fault)
{
	struct kvm_mmu_page *root = tdp_mmu_get_root_for_fault(vcpu, fault);
	struct kvm *kvm = vcpu->kvm;
	struct tdp_iter iter;
	struct kvm_mmu_page *sp;
	int ret = RET_PF_RETRY;

	kvm_mmu_hugepage_adjust(vcpu, fault);

	trace_kvm_mmu_spte_requested(fault);

	rcu_read_lock();

	for_each_tdp_pte(iter, kvm, root, fault->gfn, fault->gfn + 1) {
		int r;

		if (fault->nx_huge_page_workaround_enabled)
			disallowed_hugepage_adjust(fault, iter.old_spte, iter.level);

		/*
		 * If SPTE has been frozen by another thread, just give up and
		 * retry, avoiding unnecessary page table allocation and free.
		 */
		if (is_frozen_spte(iter.old_spte))
			goto retry;

		if (iter.level == fault->goal_level)
			goto map_target_level;

		/* Step down into the lower level page table if it exists. */
		if (is_shadow_present_pte(iter.old_spte) &&
		    !is_large_pte(iter.old_spte))
			continue;

		/*
		 * The SPTE is either non-present or points to a huge page that
		 * needs to be split.
		 */
		sp = tdp_mmu_alloc_sp(vcpu);
		tdp_mmu_init_child_sp(sp, &iter);
		if (is_mirror_sp(sp))
			kvm_mmu_alloc_external_spt(vcpu, sp);

		sp->nx_huge_page_disallowed = fault->huge_page_disallowed;

		if (is_shadow_present_pte(iter.old_spte)) {
			/* Don't support large page for mirrored roots (TDX) */
			KVM_BUG_ON(is_mirror_sptep(iter.sptep), vcpu->kvm);
			r = tdp_mmu_split_huge_page(kvm, &iter, sp, true);
		} else {
			r = tdp_mmu_link_sp(kvm, &iter, sp, true);
		}

		/*
		 * Force the guest to retry if installing an upper level SPTE
		 * failed, e.g. because a different task modified the SPTE.
		 */
		if (r) {
			tdp_mmu_free_sp(sp);
			goto retry;
		}

		if (fault->huge_page_disallowed &&
		    fault->req_level >= iter.level) {
			spin_lock(&kvm->arch.tdp_mmu_pages_lock);
			if (sp->nx_huge_page_disallowed)
				track_possible_nx_huge_page(kvm, sp);
			spin_unlock(&kvm->arch.tdp_mmu_pages_lock);
		}
	}

	/*
	 * The walk aborted before reaching the target level, e.g. because the
	 * iterator detected an upper level SPTE was frozen during traversal.
	 */
	WARN_ON_ONCE(iter.level == fault->goal_level);
	goto retry;

map_target_level:
	ret = tdp_mmu_map_handle_target_level(vcpu, fault, &iter);

retry:
	rcu_read_unlock();
	return ret;
}

/* Used by mmu notifier via kvm_unmap_gfn_range() */
bool kvm_tdp_mmu_unmap_gfn_range(struct kvm *kvm, struct kvm_gfn_range *range,
				 bool flush)
{
	enum kvm_tdp_mmu_root_types types;
	struct kvm_mmu_page *root;

	types = kvm_gfn_range_filter_to_root_types(kvm, range->attr_filter) | KVM_INVALID_ROOTS;

	__for_each_tdp_mmu_root_yield_safe(kvm, root, range->slot->as_id, types)
		flush = tdp_mmu_zap_leafs(kvm, root, range->start, range->end,
					  range->may_block, flush);

	return flush;
}

/*
 * Mark the SPTEs range of GFNs [start, end) unaccessed and return non-zero
 * if any of the GFNs in the range have been accessed.
 *
 * No need to mark the corresponding PFN as accessed as this call is coming
 * from the clear_young() or clear_flush_young() notifier, which uses the
 * return value to determine if the page has been accessed.
 */
static void kvm_tdp_mmu_age_spte(struct kvm *kvm, struct tdp_iter *iter)
{
	u64 new_spte;

	if (spte_ad_enabled(iter->old_spte)) {
		iter->old_spte = tdp_mmu_clear_spte_bits_atomic(iter->sptep,
								shadow_accessed_mask);
		new_spte = iter->old_spte & ~shadow_accessed_mask;
	} else {
		new_spte = mark_spte_for_access_track(iter->old_spte);
		/*
		 * It is safe for the following cmpxchg to fail. Leave the
		 * Accessed bit set, as the spte is most likely young anyway.
		 */
		if (__tdp_mmu_set_spte_atomic(kvm, iter, new_spte))
			return;
	}

	trace_kvm_tdp_mmu_spte_changed(iter->as_id, iter->gfn, iter->level,
				       iter->old_spte, new_spte);
}

static bool __kvm_tdp_mmu_age_gfn_range(struct kvm *kvm,
					struct kvm_gfn_range *range,
					bool test_only)
{
	enum kvm_tdp_mmu_root_types types;
	struct kvm_mmu_page *root;
	struct tdp_iter iter;
	bool ret = false;

	types = kvm_gfn_range_filter_to_root_types(kvm, range->attr_filter);

	/*
	 * Don't support rescheduling, none of the MMU notifiers that funnel
	 * into this helper allow blocking; it'd be dead, wasteful code.  Note,
	 * this helper must NOT be used to unmap GFNs, as it processes only
	 * valid roots!
	 */
	WARN_ON(types & ~KVM_VALID_ROOTS);

	guard(rcu)();
	for_each_tdp_mmu_root_rcu(kvm, root, range->slot->as_id, types) {
		tdp_root_for_each_leaf_pte(iter, kvm, root, range->start, range->end) {
			if (!is_accessed_spte(iter.old_spte))
				continue;

			if (test_only)
				return true;

			ret = true;
			kvm_tdp_mmu_age_spte(kvm, &iter);
		}
	}

	return ret;
}

bool kvm_tdp_mmu_age_gfn_range(struct kvm *kvm, struct kvm_gfn_range *range)
{
	return __kvm_tdp_mmu_age_gfn_range(kvm, range, false);
}

bool kvm_tdp_mmu_test_age_gfn(struct kvm *kvm, struct kvm_gfn_range *range)
{
	return __kvm_tdp_mmu_age_gfn_range(kvm, range, true);
}

/*
 * Remove write access from all SPTEs at or above min_level that map GFNs
 * [start, end). Returns true if an SPTE has been changed and the TLBs need to
 * be flushed.
 */
static bool wrprot_gfn_range(struct kvm *kvm, struct kvm_mmu_page *root,
			     gfn_t start, gfn_t end, int min_level)
{
	struct tdp_iter iter;
	u64 new_spte;
	bool spte_set = false;

	rcu_read_lock();

	BUG_ON(min_level > KVM_MAX_HUGEPAGE_LEVEL);

	for_each_tdp_pte_min_level(iter, kvm, root, min_level, start, end) {
retry:
		if (tdp_mmu_iter_cond_resched(kvm, &iter, false, true))
			continue;

		if (!is_shadow_present_pte(iter.old_spte) ||
		    !is_last_spte(iter.old_spte, iter.level) ||
		    !(iter.old_spte & PT_WRITABLE_MASK))
			continue;

		new_spte = iter.old_spte & ~PT_WRITABLE_MASK;

		if (tdp_mmu_set_spte_atomic(kvm, &iter, new_spte))
			goto retry;

		spte_set = true;
	}

	rcu_read_unlock();
	return spte_set;
}

/*
 * Remove write access from all the SPTEs mapping GFNs in the memslot. Will
 * only affect leaf SPTEs down to min_level.
 * Returns true if an SPTE has been changed and the TLBs need to be flushed.
 */
bool kvm_tdp_mmu_wrprot_slot(struct kvm *kvm,
			     const struct kvm_memory_slot *slot, int min_level)
{
	struct kvm_mmu_page *root;
	bool spte_set = false;

	lockdep_assert_held_read(&kvm->mmu_lock);

	for_each_valid_tdp_mmu_root_yield_safe(kvm, root, slot->as_id)
		spte_set |= wrprot_gfn_range(kvm, root, slot->base_gfn,
			     slot->base_gfn + slot->npages, min_level);

	return spte_set;
}

static struct kvm_mmu_page *tdp_mmu_alloc_sp_for_split(void)
{
	struct kvm_mmu_page *sp;

	sp = kmem_cache_zalloc(mmu_page_header_cache, GFP_KERNEL_ACCOUNT);
	if (!sp)
		return NULL;

	sp->spt = (void *)get_zeroed_page(GFP_KERNEL_ACCOUNT);
	if (!sp->spt) {
		kmem_cache_free(mmu_page_header_cache, sp);
		return NULL;
	}

	return sp;
}

/* Note, the caller is responsible for initializing @sp. */
static int tdp_mmu_split_huge_page(struct kvm *kvm, struct tdp_iter *iter,
				   struct kvm_mmu_page *sp, bool shared)
{
	const u64 huge_spte = iter->old_spte;
	const int level = iter->level;
	int ret, i;

	/*
	 * No need for atomics when writing to sp->spt since the page table has
	 * not been linked in yet and thus is not reachable from any other CPU.
	 */
	for (i = 0; i < SPTE_ENT_PER_PAGE; i++)
		sp->spt[i] = make_small_spte(kvm, huge_spte, sp->role, i);

	/*
	 * Replace the huge spte with a pointer to the populated lower level
	 * page table. Since we are making this change without a TLB flush vCPUs
	 * will see a mix of the split mappings and the original huge mapping,
	 * depending on what's currently in their TLB. This is fine from a
	 * correctness standpoint since the translation will be the same either
	 * way.
	 */
	ret = tdp_mmu_link_sp(kvm, iter, sp, shared);
	if (ret)
		goto out;

	/*
	 * tdp_mmu_link_sp_atomic() will handle subtracting the huge page we
	 * are overwriting from the page stats. But we have to manually update
	 * the page stats with the new present child pages.
	 */
	kvm_update_page_stats(kvm, level - 1, SPTE_ENT_PER_PAGE);

out:
	trace_kvm_mmu_split_huge_page(iter->gfn, huge_spte, level, ret);
	return ret;
}

static int tdp_mmu_split_huge_pages_root(struct kvm *kvm,
					 struct kvm_mmu_page *root,
					 gfn_t start, gfn_t end,
					 int target_level, bool shared)
{
	struct kvm_mmu_page *sp = NULL;
	struct tdp_iter iter;

	rcu_read_lock();

	/*
	 * Traverse the page table splitting all huge pages above the target
	 * level into one lower level. For example, if we encounter a 1GB page
	 * we split it into 512 2MB pages.
	 *
	 * Since the TDP iterator uses a pre-order traversal, we are guaranteed
	 * to visit an SPTE before ever visiting its children, which means we
	 * will correctly recursively split huge pages that are more than one
	 * level above the target level (e.g. splitting a 1GB to 512 2MB pages,
	 * and then splitting each of those to 512 4KB pages).
	 */
	for_each_tdp_pte_min_level(iter, kvm, root, target_level + 1, start, end) {
retry:
		if (tdp_mmu_iter_cond_resched(kvm, &iter, false, shared))
			continue;

		if (!is_shadow_present_pte(iter.old_spte) || !is_large_pte(iter.old_spte))
			continue;

		if (!sp) {
			rcu_read_unlock();

			if (shared)
				read_unlock(&kvm->mmu_lock);
			else
				write_unlock(&kvm->mmu_lock);

			sp = tdp_mmu_alloc_sp_for_split();

			if (shared)
				read_lock(&kvm->mmu_lock);
			else
				write_lock(&kvm->mmu_lock);

			if (!sp) {
				trace_kvm_mmu_split_huge_page(iter.gfn,
							      iter.old_spte,
							      iter.level, -ENOMEM);
				return -ENOMEM;
			}

			rcu_read_lock();

			iter.yielded = true;
			continue;
		}

		tdp_mmu_init_child_sp(sp, &iter);

		if (tdp_mmu_split_huge_page(kvm, &iter, sp, shared))
			goto retry;

		sp = NULL;
	}

	rcu_read_unlock();

	/*
	 * It's possible to exit the loop having never used the last sp if, for
	 * example, a vCPU doing HugePage NX splitting wins the race and
	 * installs its own sp in place of the last sp we tried to split.
	 */
	if (sp)
		tdp_mmu_free_sp(sp);

	return 0;
}


/*
 * Try to split all huge pages mapped by the TDP MMU down to the target level.
 */
void kvm_tdp_mmu_try_split_huge_pages(struct kvm *kvm,
				      const struct kvm_memory_slot *slot,
				      gfn_t start, gfn_t end,
				      int target_level, bool shared)
{
	struct kvm_mmu_page *root;
	int r = 0;

	kvm_lockdep_assert_mmu_lock_held(kvm, shared);
	for_each_valid_tdp_mmu_root_yield_safe(kvm, root, slot->as_id) {
		r = tdp_mmu_split_huge_pages_root(kvm, root, start, end, target_level, shared);
		if (r) {
			kvm_tdp_mmu_put_root(kvm, root);
			break;
		}
	}
}

static bool tdp_mmu_need_write_protect(struct kvm *kvm, struct kvm_mmu_page *sp)
{
	/*
	 * All TDP MMU shadow pages share the same role as their root, aside
	 * from level, so it is valid to key off any shadow page to determine if
	 * write protection is needed for an entire tree.
	 */
	return kvm_mmu_page_ad_need_write_protect(kvm, sp) || !kvm_ad_enabled;
}

static void clear_dirty_gfn_range(struct kvm *kvm, struct kvm_mmu_page *root,
				  gfn_t start, gfn_t end)
{
	const u64 dbit = tdp_mmu_need_write_protect(kvm, root) ?
			 PT_WRITABLE_MASK : shadow_dirty_mask;
	struct tdp_iter iter;

	rcu_read_lock();

	tdp_root_for_each_pte(iter, kvm, root, start, end) {
retry:
		if (!is_shadow_present_pte(iter.old_spte) ||
		    !is_last_spte(iter.old_spte, iter.level))
			continue;

		if (tdp_mmu_iter_cond_resched(kvm, &iter, false, true))
			continue;

		KVM_MMU_WARN_ON(dbit == shadow_dirty_mask &&
				spte_ad_need_write_protect(iter.old_spte));

		if (!(iter.old_spte & dbit))
			continue;

		if (tdp_mmu_set_spte_atomic(kvm, &iter, iter.old_spte & ~dbit))
			goto retry;
	}

	rcu_read_unlock();
}

/*
 * Clear the dirty status (D-bit or W-bit) of all the SPTEs mapping GFNs in the
 * memslot.
 */
void kvm_tdp_mmu_clear_dirty_slot(struct kvm *kvm,
				  const struct kvm_memory_slot *slot)
{
	struct kvm_mmu_page *root;

	lockdep_assert_held_read(&kvm->mmu_lock);
	for_each_valid_tdp_mmu_root_yield_safe(kvm, root, slot->as_id)
		clear_dirty_gfn_range(kvm, root, slot->base_gfn,
				      slot->base_gfn + slot->npages);
}

static void clear_dirty_pt_masked(struct kvm *kvm, struct kvm_mmu_page *root,
				  gfn_t gfn, unsigned long mask, bool wrprot)
{
	const u64 dbit = (wrprot || tdp_mmu_need_write_protect(kvm, root)) ?
			  PT_WRITABLE_MASK : shadow_dirty_mask;
	struct tdp_iter iter;

	lockdep_assert_held_write(&kvm->mmu_lock);

	rcu_read_lock();

	tdp_root_for_each_leaf_pte(iter, kvm, root, gfn + __ffs(mask),
				    gfn + BITS_PER_LONG) {
		if (!mask)
			break;

		KVM_MMU_WARN_ON(dbit == shadow_dirty_mask &&
				spte_ad_need_write_protect(iter.old_spte));

		if (iter.level > PG_LEVEL_4K ||
		    !(mask & (1UL << (iter.gfn - gfn))))
			continue;

		mask &= ~(1UL << (iter.gfn - gfn));

		if (!(iter.old_spte & dbit))
			continue;

		iter.old_spte = tdp_mmu_clear_spte_bits(iter.sptep,
							iter.old_spte, dbit,
							iter.level);

		trace_kvm_tdp_mmu_spte_changed(iter.as_id, iter.gfn, iter.level,
					       iter.old_spte,
					       iter.old_spte & ~dbit);
	}

	rcu_read_unlock();
}

/*
 * Clear the dirty status (D-bit or W-bit) of all the 4k SPTEs mapping GFNs for
 * which a bit is set in mask, starting at gfn. The given memslot is expected to
 * contain all the GFNs represented by set bits in the mask.
 */
void kvm_tdp_mmu_clear_dirty_pt_masked(struct kvm *kvm,
				       struct kvm_memory_slot *slot,
				       gfn_t gfn, unsigned long mask,
				       bool wrprot)
{
	struct kvm_mmu_page *root;

	for_each_valid_tdp_mmu_root(kvm, root, slot->as_id)
		clear_dirty_pt_masked(kvm, root, gfn, mask, wrprot);
}

static int tdp_mmu_make_huge_spte(struct kvm *kvm,
				  struct tdp_iter *parent,
				  u64 *huge_spte)
{
	struct kvm_mmu_page *root = spte_to_child_sp(parent->old_spte);
	gfn_t start = parent->gfn;
	gfn_t end = start + KVM_PAGES_PER_HPAGE(parent->level);
	struct tdp_iter iter;

	tdp_root_for_each_leaf_pte(iter, kvm, root, start, end) {
		/*
		 * Use the parent iterator when checking for forward progress so
		 * that KVM doesn't get stuck continuously trying to yield (i.e.
		 * returning -EAGAIN here and then failing the forward progress
		 * check in the caller ad nauseam).
		 */
		if (tdp_mmu_iter_need_resched(kvm, parent))
			return -EAGAIN;

		*huge_spte = make_huge_spte(kvm, iter.old_spte, parent->level);
		return 0;
	}

	return -ENOENT;
}

static void recover_huge_pages_range(struct kvm *kvm,
				     struct kvm_mmu_page *root,
				     const struct kvm_memory_slot *slot)
{
	gfn_t start = slot->base_gfn;
	gfn_t end = start + slot->npages;
	struct tdp_iter iter;
	int max_mapping_level;
	bool flush = false;
	u64 huge_spte;
	int r;

	if (WARN_ON_ONCE(kvm_slot_dirty_track_enabled(slot)))
		return;

	rcu_read_lock();

	for_each_tdp_pte_min_level(iter, kvm, root, PG_LEVEL_2M, start, end) {
retry:
		if (tdp_mmu_iter_cond_resched(kvm, &iter, flush, true)) {
			flush = false;
			continue;
		}

		if (iter.level > KVM_MAX_HUGEPAGE_LEVEL ||
		    !is_shadow_present_pte(iter.old_spte))
			continue;

		/*
		 * Don't zap leaf SPTEs, if a leaf SPTE could be replaced with
		 * a large page size, then its parent would have been zapped
		 * instead of stepping down.
		 */
		if (is_last_spte(iter.old_spte, iter.level))
			continue;

		/*
		 * If iter.gfn resides outside of the slot, i.e. the page for
		 * the current level overlaps but is not contained by the slot,
		 * then the SPTE can't be made huge.  More importantly, trying
		 * to query that info from slot->arch.lpage_info will cause an
		 * out-of-bounds access.
		 */
		if (iter.gfn < start || iter.gfn >= end)
			continue;

		max_mapping_level = kvm_mmu_max_mapping_level(kvm, slot, iter.gfn);
		if (max_mapping_level < iter.level)
			continue;

		r = tdp_mmu_make_huge_spte(kvm, &iter, &huge_spte);
		if (r == -EAGAIN)
			goto retry;
		else if (r)
			continue;

		if (tdp_mmu_set_spte_atomic(kvm, &iter, huge_spte))
			goto retry;

		flush = true;
	}

	if (flush)
		kvm_flush_remote_tlbs_memslot(kvm, slot);

	rcu_read_unlock();
}

/*
 * Recover huge page mappings within the slot by replacing non-leaf SPTEs with
 * huge SPTEs where possible.
 */
void kvm_tdp_mmu_recover_huge_pages(struct kvm *kvm,
				    const struct kvm_memory_slot *slot)
{
	struct kvm_mmu_page *root;

	lockdep_assert_held_read(&kvm->mmu_lock);
	for_each_valid_tdp_mmu_root_yield_safe(kvm, root, slot->as_id)
		recover_huge_pages_range(kvm, root, slot);
}

/*
 * Removes write access on the last level SPTE mapping this GFN and unsets the
 * MMU-writable bit to ensure future writes continue to be intercepted.
 * Returns true if an SPTE was set and a TLB flush is needed.
 */
static bool write_protect_gfn(struct kvm *kvm, struct kvm_mmu_page *root,
			      gfn_t gfn, int min_level)
{
	struct tdp_iter iter;
	u64 new_spte;
	bool spte_set = false;

	BUG_ON(min_level > KVM_MAX_HUGEPAGE_LEVEL);

	rcu_read_lock();

	for_each_tdp_pte_min_level(iter, kvm, root, min_level, gfn, gfn + 1) {
		if (!is_shadow_present_pte(iter.old_spte) ||
		    !is_last_spte(iter.old_spte, iter.level))
			continue;

		new_spte = iter.old_spte &
			~(PT_WRITABLE_MASK | shadow_mmu_writable_mask);

		if (new_spte == iter.old_spte)
			break;

		tdp_mmu_iter_set_spte(kvm, &iter, new_spte);
		spte_set = true;
	}

	rcu_read_unlock();

	return spte_set;
}

/*
 * Removes write access on the last level SPTE mapping this GFN and unsets the
 * MMU-writable bit to ensure future writes continue to be intercepted.
 * Returns true if an SPTE was set and a TLB flush is needed.
 */
bool kvm_tdp_mmu_write_protect_gfn(struct kvm *kvm,
				   struct kvm_memory_slot *slot, gfn_t gfn,
				   int min_level)
{
	struct kvm_mmu_page *root;
	bool spte_set = false;

	lockdep_assert_held_write(&kvm->mmu_lock);
	for_each_valid_tdp_mmu_root(kvm, root, slot->as_id)
		spte_set |= write_protect_gfn(kvm, root, gfn, min_level);

	return spte_set;
}

/*
 * Return the level of the lowest level SPTE added to sptes.
 * That SPTE may be non-present.
 *
 * Must be called between kvm_tdp_mmu_walk_lockless_{begin,end}.
 */
static int __kvm_tdp_mmu_get_walk(struct kvm_vcpu *vcpu, u64 addr, u64 *sptes,
				  struct kvm_mmu_page *root)
{
	struct tdp_iter iter;
	gfn_t gfn = addr >> PAGE_SHIFT;
	int leaf = -1;

	for_each_tdp_pte(iter, vcpu->kvm, root, gfn, gfn + 1) {
		leaf = iter.level;
		sptes[leaf] = iter.old_spte;
	}

	return leaf;
}

int kvm_tdp_mmu_get_walk(struct kvm_vcpu *vcpu, u64 addr, u64 *sptes,
			 int *root_level)
{
	struct kvm_mmu_page *root = root_to_sp(vcpu->arch.mmu->root.hpa);
	*root_level = vcpu->arch.mmu->root_role.level;

	return __kvm_tdp_mmu_get_walk(vcpu, addr, sptes, root);
}

bool kvm_tdp_mmu_gpa_is_mapped(struct kvm_vcpu *vcpu, u64 gpa)
{
	struct kvm *kvm = vcpu->kvm;
	bool is_direct = kvm_is_addr_direct(kvm, gpa);
	hpa_t root = is_direct ? vcpu->arch.mmu->root.hpa :
				 vcpu->arch.mmu->mirror_root_hpa;
	u64 sptes[PT64_ROOT_MAX_LEVEL + 1], spte;
	int leaf;

	lockdep_assert_held(&kvm->mmu_lock);
	rcu_read_lock();
	leaf = __kvm_tdp_mmu_get_walk(vcpu, gpa, sptes, root_to_sp(root));
	rcu_read_unlock();
	if (leaf < 0)
		return false;

	spte = sptes[leaf];
	return is_shadow_present_pte(spte) && is_last_spte(spte, leaf);
}
EXPORT_SYMBOL_GPL(kvm_tdp_mmu_gpa_is_mapped);

/*
 * Returns the last level spte pointer of the shadow page walk for the given
 * gpa, and sets *spte to the spte value. This spte may be non-preset. If no
 * walk could be performed, returns NULL and *spte does not contain valid data.
 *
 * Contract:
 *  - Must be called between kvm_tdp_mmu_walk_lockless_{begin,end}.
 *  - The returned sptep must not be used after kvm_tdp_mmu_walk_lockless_end.
 *
 * WARNING: This function is only intended to be called during fast_page_fault.
 */
u64 *kvm_tdp_mmu_fast_pf_get_last_sptep(struct kvm_vcpu *vcpu, gfn_t gfn,
					u64 *spte)
{
	/* Fast pf is not supported for mirrored roots  */
	struct kvm_mmu_page *root = tdp_mmu_get_root(vcpu, KVM_DIRECT_ROOTS);
	struct tdp_iter iter;
	tdp_ptep_t sptep = NULL;

	for_each_tdp_pte(iter, vcpu->kvm, root, gfn, gfn + 1) {
		*spte = iter.old_spte;
		sptep = iter.sptep;
	}

	/*
	 * Perform the rcu_dereference to get the raw spte pointer value since
	 * we are passing it up to fast_page_fault, which is shared with the
	 * legacy MMU and thus does not retain the TDP MMU-specific __rcu
	 * annotation.
	 *
	 * This is safe since fast_page_fault obeys the contracts of this
	 * function as well as all TDP MMU contracts around modifying SPTEs
	 * outside of mmu_lock.
	 */
	return rcu_dereference(sptep);
}
