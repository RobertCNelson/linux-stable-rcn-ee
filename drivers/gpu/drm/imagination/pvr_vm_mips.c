// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_fw_mips.h"
#include "pvr_gem.h"
#include "pvr_rogue_mips.h"
#include "pvr_vm_mips.h"

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/types.h>

/**
 * pvr_vm_mips_init() - Initialise MIPS FW pagetable
 * @pvr_dev: Target PowerVR device.
 *
 * Returns:
 *  * 0 on success,
 *  * -%EINVAL,
 *  * Any error returned by pvr_gem_object_create(), or
 *  * And error returned by pvr_gem_object_vmap().
 */
int
pvr_vm_mips_init(struct pvr_device *pvr_dev)
{
	u32 pt_size = 1 << ROGUE_MIPSFW_LOG2_PAGETABLE_SIZE_4K(pvr_dev);
	struct pvr_fw_mips_data *mips_data;
	u32 phys_bus_width;
	int err;

	/* Page table size must be at most ROGUE_MIPSFW_MAX_NUM_PAGETABLE_PAGES * 4k pages. */
	if (pt_size > ROGUE_MIPSFW_MAX_NUM_PAGETABLE_PAGES * SZ_4K) {
		err = -EINVAL;
		goto err_out;
	}

	if (PVR_FEATURE_VALUE(pvr_dev, phys_bus_width, &phys_bus_width)) {
		err = -EINVAL;
		goto err_out;
	}

	mips_data = kzalloc(sizeof(*mips_data), GFP_KERNEL);
	if (!mips_data) {
		err = -ENOMEM;
		goto err_out;
	}

	mips_data->pt_obj = pvr_gem_object_create(pvr_dev, pt_size,
						  DRM_PVR_BO_DEVICE_PM_FW_PROTECT |
						  DRM_PVR_BO_CREATE_ZEROED);
	if (IS_ERR(mips_data->pt_obj)) {
		err = PTR_ERR(mips_data->pt_obj);
		goto err_kfree;
	}

	mips_data->pt = pvr_gem_object_vmap(mips_data->pt_obj, false);
	if (IS_ERR(mips_data->pt)) {
		err = PTR_ERR(mips_data->pt);
		goto err_put_obj;
	}

	mips_data->pfn_mask = (phys_bus_width > 32) ? ROGUE_MIPSFW_ENTRYLO_PFN_MASK_ABOVE_32BIT :
						      ROGUE_MIPSFW_ENTRYLO_PFN_MASK;

	mips_data->cache_policy = (phys_bus_width > 32) ? ROGUE_MIPSFW_CACHED_POLICY_ABOVE_32BIT :
							  ROGUE_MIPSFW_CACHED_POLICY;

	pvr_dev->fw_dev.processor_data.mips_data = mips_data;

	return 0;

err_put_obj:
	pvr_gem_object_put(mips_data->pt_obj);

err_kfree:
	kfree(mips_data);

err_out:
	return err;
}

/**
 * pvr_vm_mips_fini() - Release MIPS FW pagetable
 * @pvr_dev: Target PowerVR device.
 */
void
pvr_vm_mips_fini(struct pvr_device *pvr_dev)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	struct pvr_fw_mips_data *mips_data = fw_dev->processor_data.mips_data;

	pvr_gem_object_vunmap(mips_data->pt_obj, false);
	pvr_gem_object_put(mips_data->pt_obj);
	kfree(mips_data);
	fw_dev->processor_data.mips_data = NULL;
}

static u32
get_mips_pte_flags(bool read, bool write, int cache_policy)
{
	u32 flags = 0;

	if (read && write) /* Read/write. */
		flags |= ROGUE_MIPSFW_ENTRYLO_DIRTY_EN;
	else if (write)    /* Write only. */
		flags |= ROGUE_MIPSFW_ENTRYLO_READ_INHIBIT_EN;
	else
		WARN_ON(!read);

	flags |= cache_policy << ROGUE_MIPSFW_ENTRYLO_CACHE_POLICY_SHIFT;

	flags |= ROGUE_MIPSFW_ENTRYLO_VALID_EN | ROGUE_MIPSFW_ENTRYLO_GLOBAL_EN;

	return flags;
}

/**
 * pvr_vm_mips_map() - Map a FW object into MIPS address space
 * @pvr_dev: Target PowerVR device.
 * @fw_obj: FW object to map.
 *
 * Returns:
 *  * 0 on success,
 *  * -%EINVAL if object does not reside within FW address space, or
 *  * Any error returned by pvr_fw_get_dma_addr().
 */
int
pvr_vm_mips_map(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	struct pvr_fw_mips_data *mips_data = fw_dev->processor_data.mips_data;
	struct pvr_gem_object *pvr_obj = &fw_obj->base;
	u64 start = fw_obj->fw_mm_node.start;
	u64 size = fw_obj->fw_mm_node.size;
	u64 end;
	u32 cache_policy;
	u32 pte_flags;
	u32 start_pfn;
	u32 end_pfn;
	u32 pfn;
	int err;

	if (check_add_overflow(start, size - 1, &end)) {
		err = -EINVAL;
		goto err_out;
	}

	if (start < ROGUE_FW_HEAP_BASE ||
	    start >= ROGUE_FW_HEAP_BASE + fw_dev->fw_heap_info.raw_size ||
	    end < ROGUE_FW_HEAP_BASE ||
	    end >= ROGUE_FW_HEAP_BASE + fw_dev->fw_heap_info.raw_size ||
	    (start & ROGUE_MIPSFW_PAGE_MASK_4K) ||
	    ((end + 1) & ROGUE_MIPSFW_PAGE_MASK_4K)) {
		err = -EINVAL;
		goto err_out;
	}

	start_pfn = (start & fw_dev->fw_heap_info.offset_mask) >> ROGUE_MIPSFW_LOG2_PAGE_SIZE_4K;
	end_pfn = (end & fw_dev->fw_heap_info.offset_mask) >> ROGUE_MIPSFW_LOG2_PAGE_SIZE_4K;

	if (pvr_obj->flags & PVR_BO_FW_FLAGS_DEVICE_UNCACHED)
		cache_policy = ROGUE_MIPSFW_UNCACHED_CACHE_POLICY;
	else
		cache_policy = mips_data->cache_policy;

	pte_flags = get_mips_pte_flags(true, true, cache_policy);

	for (pfn = start_pfn; pfn <= end_pfn; pfn++) {
		dma_addr_t dma_addr;
		u32 pte;

		err = pvr_fw_get_dma_addr(fw_obj,
					  (pfn - start_pfn) << ROGUE_MIPSFW_LOG2_PAGE_SIZE_4K,
					  &dma_addr);
		if (err)
			goto err_unmap_pages;

		pte = ((dma_addr >> ROGUE_MIPSFW_LOG2_PAGE_SIZE_4K)
		       << ROGUE_MIPSFW_ENTRYLO_PFN_SHIFT) & mips_data->pfn_mask;
		pte |= pte_flags;

		WRITE_ONCE(mips_data->pt[pfn], pte);
	}

	pvr_vm_mmu_flush(pvr_dev);

	return 0;

err_unmap_pages:
	for (; pfn >= start_pfn; pfn--)
		WRITE_ONCE(mips_data->pt[pfn], 0);

	pvr_vm_mmu_flush(pvr_dev);

err_out:
	return err;
}

/**
 * pvr_vm_mips_unmap() - Unmap a FW object into MIPS address space
 * @pvr_dev: Target PowerVR device.
 * @fw_obj: FW object to unmap.
 */
void
pvr_vm_mips_unmap(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	struct pvr_fw_mips_data *mips_data = fw_dev->processor_data.mips_data;
	u64 start = fw_obj->fw_mm_node.start;
	u64 size = fw_obj->fw_mm_node.size;
	u64 end = start + size;

	u32 start_pfn = (start & fw_dev->fw_heap_info.offset_mask) >>
			ROGUE_MIPSFW_LOG2_PAGE_SIZE_4K;
	u32 end_pfn = (end & fw_dev->fw_heap_info.offset_mask) >> ROGUE_MIPSFW_LOG2_PAGE_SIZE_4K;
	u32 pfn;

	for (pfn = start_pfn; pfn < end_pfn; pfn++)
		WRITE_ONCE(mips_data->pt[pfn], 0);

	pvr_vm_mmu_flush(pvr_dev);
}
