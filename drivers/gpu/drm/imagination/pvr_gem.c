// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#include "pvr_device.h"
#include "pvr_gem.h"
#include "pvr_rogue_meta.h"
#include "pvr_vm.h"
#include "pvr_vm_mips.h"

#include <drm/drm_gem.h>
#include <drm/drm_prime.h>

#include <linux/compiler.h>
#include <linux/compiler_attributes.h>
#include <linux/dma-buf.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/iosys-map.h>
#include <linux/log2.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>

static vm_fault_t pvr_gem_vm_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct drm_gem_object *gem_obj = vma->vm_private_data;
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);
	loff_t num_pages = gem_obj->size >> PAGE_SHIFT;
	pgoff_t page_offset;
	struct page *page;
	vm_fault_t ret;

	/* We don't use vmf->pgoff since that has the fake offset */
	page_offset = (vmf->address - vma->vm_start) >> PAGE_SHIFT;

	if (page_offset >= num_pages || WARN_ON_ONCE(!pvr_obj->pages)) {
		ret = VM_FAULT_SIGBUS;
	} else {
		page = pvr_obj->pages[page_offset];

		ret = vmf_insert_page(vma, vmf->address, page);
	}

	return ret;
}

static void pvr_gem_vm_open(struct vm_area_struct *vma)
{
	struct drm_gem_object *gem_obj = vma->vm_private_data;
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);
	int err;

	WARN_ON(gem_obj->import_attach);

	err = pvr_gem_object_get_pages(pvr_obj);
	WARN_ON(err);

	drm_gem_vm_open(vma);
}

static void pvr_gem_vm_close(struct vm_area_struct *vma)
{
	struct drm_gem_object *gem_obj = vma->vm_private_data;
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);

	pvr_gem_object_put_pages(pvr_obj);
	drm_gem_vm_close(vma);
}

static const struct vm_operations_struct pvr_gem_vm_ops = {
	.fault = pvr_gem_vm_fault,
	.open = pvr_gem_vm_open,
	.close = pvr_gem_vm_close,
};

static void pvr_gem_free_object(struct drm_gem_object *gem_obj)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);

	if (gem_obj->import_attach)
		drm_prime_gem_destroy(gem_obj, pvr_obj->sgt);
	drm_gem_object_release(gem_obj);
	kfree(pvr_obj);
}

static struct sg_table *pvr_gem_get_sg_table(struct drm_gem_object *gem_obj)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);

	if (gem_obj->import_attach)
		return ERR_PTR(-EINVAL);

	return drm_prime_pages_to_sg(gem_obj->dev, pvr_obj->pages, gem_obj->size >> PAGE_SHIFT);
}

static int pvr_gem_mmap(struct drm_gem_object *gem_obj, struct vm_area_struct *vma)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);
	int err;

	if (gem_obj->import_attach) {
		/* Drop the reference drm_gem_mmap_obj() acquired.*/
		drm_gem_object_put(gem_obj);
		vma->vm_private_data = NULL;

		return dma_buf_mmap(gem_obj->dma_buf, vma, 0);
	}

	if (!(pvr_obj->flags & DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS)) {
		err = -EINVAL;
		goto err_out;
	}

	err = pvr_gem_object_get_pages(pvr_obj);
	if (err)
		goto err_out;

	vma->vm_flags |= VM_MIXEDMAP | VM_DONTEXPAND;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	if (!(pvr_obj->flags & PVR_BO_CPU_CACHED))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return 0;

err_out:
	return err;
}

static int pvr_gem_pin(struct drm_gem_object *gem_obj)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);

	WARN_ON(gem_obj->import_attach);

	return pvr_gem_object_get_pages(pvr_obj);
}

static void pvr_gem_unpin(struct drm_gem_object *gem_obj)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);

	WARN_ON(gem_obj->import_attach);

	pvr_gem_object_put_pages(pvr_obj);
}

static int pvr_gem_vmap(struct drm_gem_object *gem_obj, struct iosys_map *map)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);
	void *cpu_ptr;

	cpu_ptr = pvr_gem_object_vmap(pvr_obj, true);
	if (IS_ERR(cpu_ptr))
		return PTR_ERR(cpu_ptr);

	iosys_map_set_vaddr(map, cpu_ptr);

	return 0;
}

static void pvr_gem_vunmap(struct drm_gem_object *gem_obj, struct iosys_map *map)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);

	pvr_gem_object_vunmap(pvr_obj, true);
}

static const struct drm_gem_object_funcs pvr_gem_object_funcs = {
	.free = pvr_gem_free_object,
	.get_sg_table = pvr_gem_get_sg_table,
	.mmap = pvr_gem_mmap,
	.pin = pvr_gem_pin,
	.unpin = pvr_gem_unpin,
	.vm_ops = &pvr_gem_vm_ops,
	.vmap = pvr_gem_vmap,
	.vunmap = pvr_gem_vunmap,
};

static void pvr_free_fw_object(struct drm_gem_object *gem_obj)
{
	struct pvr_gem_object *pvr_obj = to_pvr_gem_object(gem_obj);
	struct pvr_fw_object *fw_obj = to_pvr_fw_object(pvr_obj);

	WARN_ON(gem_obj->import_attach);
	drm_gem_object_release(gem_obj);
	kfree(fw_obj);
}

/* FW objects may not be mmap'ed or exported. */
static const struct drm_gem_object_funcs pvr_gem_fw_object_funcs = {
	.free = pvr_free_fw_object,
};

/**
 * pvr_gem_object_flags_validate() - Verify that a collection of PowerVR GEM
 * mapping and/or creation flags form a valid combination.
 * @flags: PowerVR GEM mapping/creation flags to validate.
 *
 * This function explicitly allows kernel-only flags. All ioctl entrypoints
 * should do their own validation as well as relying on this function.
 *
 * Return:
 *  * %true if @flags contains valid mapping and/or creation flags, or
 *  * %false otherwise.
 */
static bool
pvr_gem_object_flags_validate(u64 flags)
{
	static const u64 invalid_combinations[] = {
		/*
		 * Memory flagged as PM/FW-protected cannot be mapped to
		 * userspace. To make this explicit, we require that the two
		 * flags allowing each of these respective features are never
		 * specified together.
		 */
		(DRM_PVR_BO_DEVICE_PM_FW_PROTECT |
		 DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS),
	};

	int i;

	/*
	 * Check for bits set in undefined regions. Reserved regions refer to
	 * options that can only be set by the kernel. These are explicitly
	 * allowed in most cases, and must be checked specifically in IOCTL
	 * callback code.
	 */
	if ((flags & PVR_BO_UNDEFINED_MASK) != 0)
		return false;

	/*
	 * Check for all combinations of flags marked as invalid in the array
	 * above.
	 */
	for (i = 0; i < ARRAY_SIZE(invalid_combinations); ++i) {
		u64 combo = invalid_combinations[i];

		if ((flags & combo) == combo)
			return false;
	}

	return true;
}

/**
 * pvr_gem_object_into_handle() - Convert a reference to an object into a
 * userspace-accessible handle.
 * @pvr_obj: [IN] Target PowerVR-specific object.
 * @pvr_file: [IN] File to associate the handle with.
 * @handle: [OUT] Pointer to store the created handle in. Remains unmodified if
 * an error is encountered.
 *
 * If an error is encountered, ownership of @pvr_obj will not have been
 * transferred. If this function succeeds, however, further use of @pvr_obj is
 * considered undefined behaviour unless another reference to it is explicitly
 * held.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while attempting to allocate a handle on @pvr_file.
 */
int
pvr_gem_object_into_handle(struct pvr_gem_object *pvr_obj,
			   struct pvr_file *pvr_file, u32 *handle)
{
	struct drm_gem_object *gem_obj = from_pvr_gem_object(pvr_obj);
	struct drm_file *file = from_pvr_file(pvr_file);

	u32 new_handle;
	int err;

	err = drm_gem_handle_create(file, gem_obj, &new_handle);
	if (err)
		goto err_out;

	/*
	 * Release our reference to @pvr_obj, effectively transferring
	 * ownership to the handle.
	 */
	pvr_gem_object_put(pvr_obj);

	/*
	 * Do not store the new handle in @handle until no more errors can
	 * occur.
	 */
	*handle = new_handle;

	return 0;

err_out:
	return err;
}

/**
 * pvr_gem_object_from_handle() - Obtain a reference to an object from a
 * userspace handle.
 * @pvr_file: PowerVR-specific file to which @handle is associated.
 * @handle: Userspace handle referencing the target object.
 *
 * On return, @handle always maintains its reference to the requested object
 * (if it had one in the first place). If this function succeeds, the returned
 * object will hold an additional reference. When the caller is finished with
 * the returned object, they should call pvr_gem_object_put() on it to release
 * this reference.
 *
 * Return:
 *  * A pointer to the requested PowerVR-specific object on success, or
 *  * %NULL otherwise.
 */
struct pvr_gem_object *
pvr_gem_object_from_handle(struct pvr_file *pvr_file, u32 handle)
{
	struct drm_file *file = from_pvr_file(pvr_file);
	struct drm_gem_object *gem_obj;

	gem_obj = drm_gem_object_lookup(file, handle);
	if (!gem_obj)
		return NULL;

	return to_pvr_gem_object(gem_obj);
}

static int
pvr_gem_object_get_pages_locked(struct pvr_gem_object *pvr_obj)
{
	struct drm_gem_object *obj = from_pvr_gem_object(pvr_obj);
	struct page **pages;
	struct sg_table *sgt;
	int err;

	lockdep_assert_held(&pvr_obj->lock);

	if (obj->import_attach) {
		err = -EINVAL;
		goto err_out;
	}

	if ((++pvr_obj->pages_ref_count) == 1) {
		WARN_ON(pvr_obj->pages);

		pages = drm_gem_get_pages(obj);
		if (IS_ERR(pages)) {
			err = PTR_ERR(pages);
			goto err_dec_ref_count;
		}

		sgt = drm_prime_pages_to_sg(obj->dev, pages, obj->size >> PAGE_SHIFT);
		if (IS_ERR(sgt)) {
			err = PTR_ERR(sgt);
			goto err_put_pages;
		}

		err = dma_map_sgtable(obj->dev->dev, sgt, DMA_BIDIRECTIONAL, 0);
		if (err)
			goto err_free_sgt;

		pvr_obj->pages = pages;
		pvr_obj->sgt = sgt;
	} else {
		WARN_ON(!pvr_obj->pages);
	}

	return 0;

err_free_sgt:
	sg_free_table(sgt);
	kfree(sgt);

err_put_pages:
	drm_gem_put_pages(obj, pages, false, false);

err_dec_ref_count:
	pvr_obj->pages_ref_count--;

err_out:
	return err;
}

/**
 * pvr_gem_object_get_pages: Get pages associated with a &struct pvr_gem_object
 * @pvr_obj: Target object
 *
 * This will fill out the pages array of the object. This must be called before
 * the object is mapped to userspace.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error returned by drm_gem_get_pages(), or
 *  * Any error returned by drm_prime_pages_to_sg(), or
 *  * Any error returned by dma_map_sgtable().
 */
int
pvr_gem_object_get_pages(struct pvr_gem_object *pvr_obj)
{
	int err;

	mutex_lock(&pvr_obj->lock);
	err = pvr_gem_object_get_pages_locked(pvr_obj);
	mutex_unlock(&pvr_obj->lock);

	return err;
}

static void
pvr_gem_object_put_pages_locked(struct pvr_gem_object *pvr_obj)
{
	struct drm_gem_object *gem_obj = from_pvr_gem_object(pvr_obj);

	lockdep_assert_held(&pvr_obj->lock);

	if (gem_obj->import_attach)
		return;

	if (pvr_obj->pages && (--pvr_obj->pages_ref_count) == 0) {
		sg_free_table(pvr_obj->sgt);
		kfree(pvr_obj->sgt);
		drm_gem_put_pages(gem_obj, pvr_obj->pages, true, true);
		pvr_obj->sgt = NULL;
		pvr_obj->pages = NULL;
	}
}

/**
 * pvr_gem_object_put_pages: Release pages associated with a &struct
 *                           pvr_gem_object
 * @pvr_obj: Target object
 */
void
pvr_gem_object_put_pages(struct pvr_gem_object *pvr_obj)
{
	mutex_lock(&pvr_obj->lock);
	pvr_gem_object_put_pages_locked(pvr_obj);
	mutex_unlock(&pvr_obj->lock);
}

/**
 * pvr_gem_object_vmap_prot() - Map a PowerVR GEM object into CPU virtual
 * address space without using information from the object's flags.
 * @pvr_obj: Target PowerVR GEM object.
 * @sync_to_cpu: Specifies whether the buffer should be synced to the CPU
 * immediately after mapping.
 * @prot: Page protection options for the mapping.
 *
 * Once the caller is finished with the CPU mapping, they must call
 * pvr_gem_object_vunmap() on @pvr_obj.
 *
 * Unlike pvr_gem_object_vmap(), this function does NOT use information from
 * the flags on @pvr_obj to determine page protection options. You probably
 * want to use pvr_gem_object_vmap() instead. If you really need to use this
 * function, be absolutely sure that @prot is compatible with the flags on
 * @pvr_obj. There are no safeguards!
 *
 * Return:
 *  * A pointer to the CPU mapping on success,
 *  * -%ENOMEM if the mapping fails, or
 *  * Any error encountered while attempting to acquire a reference to the
 *    backing pages for @pvr_obj.
 */
static void *
pvr_gem_object_vmap_prot(struct pvr_gem_object *pvr_obj, bool sync_to_cpu,
			 pgprot_t prot)
{
	struct drm_gem_object *gem_obj = from_pvr_gem_object(pvr_obj);
	/* The size of @pvr_obj is always CPU page-aligned. */
	size_t nr_pages = pvr_gem_object_size(pvr_obj) >> PAGE_SHIFT;

	int err;

	mutex_lock(&pvr_obj->lock);

	if ((++pvr_obj->vmap_ref_count) == 1) {
		if (gem_obj->import_attach) {
			struct iosys_map map;

			err = dma_buf_vmap(gem_obj->import_attach->dmabuf, &map);
			if (err)
				goto err_unlock;

			pvr_obj->vmap_cpu_addr = map.vaddr;
		} else {
			err = pvr_gem_object_get_pages_locked(pvr_obj);
			if (err)
				goto err_unlock;

			pvr_obj->vmap_cpu_addr = vmap(pvr_obj->pages, nr_pages, VM_MAP, prot);
			if (!pvr_obj->vmap_cpu_addr) {
				err = -ENOMEM;
				goto err_put_pages;
			}
		}
	}

	/*
	 * There's no need for sync operations on the CPU cache if we're not
	 * using the CPU cache.
	 */
	if ((pvr_obj->flags & PVR_BO_CPU_CACHED) && sync_to_cpu) {
		struct device *dev = gem_obj->dev->dev;

		dma_sync_sgtable_for_cpu(dev, pvr_obj->sgt, DMA_BIDIRECTIONAL);
	}

	mutex_unlock(&pvr_obj->lock);

	return pvr_obj->vmap_cpu_addr;

err_put_pages:
	pvr_gem_object_put_pages(pvr_obj);

err_unlock:
	mutex_unlock(&pvr_obj->lock);

	return ERR_PTR(err);
}

/**
 * pvr_gem_object_vmap() - Map a PowerVR GEM object into CPU virtual address
 * space.
 * @pvr_obj: Target PowerVR GEM object.
 * @sync_to_cpu: Specifies whether the buffer should be synced to the CPU
 * immediately after mapping.
 *
 * Once the caller is finished with the CPU mapping, they must call
 * pvr_gem_object_vunmap() on @pvr_obj.
 *
 * If @pvr_obj is not using the CPU cache, @sync_to_cpu is ignored.
 *
 * Return:
 *  * A pointer to the CPU mapping on success,
 *  * -%ENOMEM if the mapping fails, or
 *  * Any error encountered while attempting to acquire a reference to the
 *    backing pages for @pvr_obj.
 */
void *
pvr_gem_object_vmap(struct pvr_gem_object *pvr_obj, bool sync_to_cpu)
{
	pgprot_t prot;

	/* Determine parameters from @pvr_obj CPU caching strategy. */
	if (pvr_obj->flags & PVR_BO_CPU_CACHED) {
		prot = PAGE_KERNEL;
	} else {
		/* The default caching strategy is write-combined. */
		prot = pgprot_writecombine(PAGE_KERNEL);
	}

	return pvr_gem_object_vmap_prot(pvr_obj, sync_to_cpu, prot);
}

/**
 * pvr_gem_object_vunmap() - Unmap a PowerVR memory object from CPU virtual
 * address space.
 * @pvr_obj: Target PowerVR GEM object.
 * @sync_to_device: Specifies whether the buffer should be synced to the device
 * immediately before unmapping from the CPU.
 *
 * If @pvr_obj is not using the CPU cache, @sync_to_device is ignored.
 */
void
pvr_gem_object_vunmap(struct pvr_gem_object *pvr_obj, bool sync_to_device)
{
	struct drm_gem_object *gem_obj = from_pvr_gem_object(pvr_obj);

	mutex_lock(&pvr_obj->lock);

	if (WARN_ON(!pvr_obj->vmap_ref_count || !pvr_obj->vmap_cpu_addr)) {
		mutex_unlock(&pvr_obj->lock);
		return;
	}

	/*
	 * There's no need for sync operations on the CPU cache if we're not
	 * using the CPU cache.
	 */
	if ((pvr_obj->flags & PVR_BO_CPU_CACHED) && sync_to_device) {
		struct device *dev = gem_obj->dev->dev;

		dma_sync_sgtable_for_device(dev, pvr_obj->sgt,
					    DMA_BIDIRECTIONAL);
	}

	if ((--pvr_obj->vmap_ref_count) == 0) {
		if (gem_obj->import_attach) {
			struct iosys_map map = IOSYS_MAP_INIT_VADDR(pvr_obj->vmap_cpu_addr);

			dma_buf_vunmap(gem_obj->import_attach->dmabuf, &map);
		} else {
			vunmap(pvr_obj->vmap_cpu_addr);

			pvr_gem_object_put_pages_locked(pvr_obj);
		}

		pvr_obj->vmap_cpu_addr = NULL;
	}

	mutex_unlock(&pvr_obj->lock);
}

/**
 * pvr_gem_object_zero() - Zeroes the physical memory behind an object.
 * @pvr_obj: Target PowerVR GEM object.
 *
 * Return:
 *  * 0 on success, or
 *  * Any error encountered while attempting to map @pvr_obj to the CPU (see
 *    pvr_gem_object_vmap_unchecked()).
 */
static int
pvr_gem_object_zero(struct pvr_gem_object *pvr_obj)
{
	void *cpu_ptr;
	int err;

	/*
	 * We always map writecombined here so there's no need to flush the
	 * CPU cache afterwards.
	 */
	cpu_ptr = pvr_gem_object_vmap_prot(pvr_obj, false,
					   pgprot_writecombine(PAGE_KERNEL));
	if (IS_ERR(cpu_ptr)) {
		err = PTR_ERR(cpu_ptr);
		goto err_out;
	}

	memset(cpu_ptr, 0, pvr_gem_object_size(pvr_obj));

	pvr_gem_object_vunmap(pvr_obj, false);

	return 0;

err_out:
	return err;
}

/**
 * pvr_gem_object_init() - Initialises a PowerVR-specific buffer object.
 * @pvr_dev: Target PowerVR device.
 * @pvr_obj: PowerVR buffer object to initialise
 * @size: Size of the object to allocate in bytes. Must be greater than zero.
 * Any value which is not an exact multiple of the system page size will be
 * rounded up to satisfy this condition.
 * @flags: Options which affect both this operation and future mapping
 * operations performed on the returned object. Must be a combination of
 * DRM_PVR_BO_* and/or PVR_BO_* flags.
 * @funcs: Pointer to &struct drm_gem_object_funcs to assign to this object.
 * @is_imported: True if buffer object represents an imported buffer.
 *
 * The created object may be larger than @size, but can never be smaller. To
 * get the exact size, call pvr_gem_object_size() on the returned pointer.
 *
 * Return:
 *  * 0 on success,
 *  * -%EINVAL if @size is zero or @flags is not valid,
 *  * -%ENOMEM if sufficient physical memory cannot be allocated,
 *  * Any other error returned by drm_gem_object_init(), or
 *  * Any other error returned by drm_gem_create_mmap_offset().
 */
static int
pvr_gem_object_init(struct pvr_device *pvr_dev, struct pvr_gem_object *pvr_obj, size_t size,
		    u64 flags, const struct drm_gem_object_funcs *funcs, bool is_imported)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	struct drm_gem_object *gem_obj;
	int err;

	/* Verify @size and @flags before continuing. */
	if (size == 0 || !pvr_gem_object_flags_validate(flags)) {
		err = -EINVAL;
		goto err_out;
	}

	size = PAGE_ALIGN(size);

	/* FIXME: Compute any kernel-only options and apply them to @flags. */

	gem_obj = from_pvr_gem_object(pvr_obj);

	if (is_imported) {
		drm_gem_private_object_init(drm_dev, gem_obj, size);
	} else {
		err = drm_gem_object_init(drm_dev, gem_obj, size);
		if (err)
			goto err_out;

		/*
		 * Our buffers are kept pinned, so allocating them from the MOVABLE zone is a
		 * really bad idea, and conflicts with CMA. See comments above new_inode() why this
		 * is required _and_ expected if you're going to pin these pages.
		 */
		mapping_set_gfp_mask(gem_obj->filp->f_mapping, GFP_HIGHUSER |
				     __GFP_RETRY_MAYFAIL | __GFP_NOWARN);
	}

	err = drm_gem_create_mmap_offset(gem_obj);
	if (err)
		goto err_release;

	pvr_obj->pvr_dev = pvr_dev;
	mutex_init(&pvr_obj->lock);

	/* Safe to cast away the const-qualifier during initialization. */
	*(u64 *)&pvr_obj->flags = flags;

	gem_obj->funcs = funcs;

	/*
	 * Do this last because pvr_gem_object_zero() requires a fully
	 * configured instance of struct pvr_gem_object.
	 */
	if (flags & DRM_PVR_BO_CREATE_ZEROED)
		pvr_gem_object_zero(pvr_obj);

	return 0;

err_release:
	drm_gem_object_release(gem_obj);

err_out:
	return err;
}

static struct pvr_gem_object *
pvr_gem_object_create_internal(struct pvr_device *pvr_dev, size_t size, u64 flags, bool is_imported)
{
	struct pvr_gem_object *pvr_obj;
	int err;

	/* Allocate a powervr-specific buffer object, which includes a &struct drm_gem_object. */
	pvr_obj = kzalloc(sizeof(*pvr_obj), GFP_KERNEL);
	if (!pvr_obj) {
		err = -ENOMEM;
		goto err_out;
	}

	err = pvr_gem_object_init(pvr_dev, pvr_obj, size, flags, &pvr_gem_object_funcs,
				  is_imported);
	if (err)
		goto err_kfree_pvr_obj;

	return pvr_obj;

err_kfree_pvr_obj:
	kfree(pvr_obj);

err_out:
	return ERR_PTR(err);
}

/**
 * pvr_gem_object_create() - Creates a PowerVR-specific buffer object.
 * @pvr_dev: Target PowerVR device.
 * @size: Size of the object to allocate in bytes. Must be greater than zero.
 * Any value which is not an exact multiple of the system page size will be
 * rounded up to satisfy this condition.
 * @flags: Options which affect both this operation and future mapping
 * operations performed on the returned object. Must be a combination of
 * DRM_PVR_BO_* and/or PVR_BO_* flags.
 *
 * The created object may be larger than @size, but can never be smaller. To
 * get the exact size, call pvr_gem_object_size() on the returned pointer.
 *
 * Return:
 *  * The newly-minted PowerVR-specific buffer object on success,
 *  * -%EINVAL if @size is zero or @flags is not valid,
 *  * -%ENOMEM if sufficient physical memory cannot be allocated, or
 *  * Any other error returned by drm_gem_create_mmap_offset().
 */
struct pvr_gem_object *
pvr_gem_object_create(struct pvr_device *pvr_dev, size_t size, u64 flags)
{
	return pvr_gem_object_create_internal(pvr_dev, size, flags, false);
}

/**
 * pvr_gem_fw_vmap() - Map a FW object in firmware address space
 * @pvr_dev: Device pointer.
 * @fw_obj: FW object to map.
 * @dev_addr: Desired address in device space, if a specific address is
 *            required. 0 otherwise.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL if @fw_obj is already mapped but has no references, or
 *  * Any error returned by DRM.
 */
static int
pvr_gem_fw_vmap(struct pvr_device *pvr_dev, struct pvr_fw_object *fw_obj,
		u64 dev_addr)
{
	struct pvr_gem_object *pvr_obj = from_pvr_fw_object(fw_obj);
	struct drm_gem_object *gem_obj = from_pvr_gem_object(pvr_obj);
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;

	int err;

	err = pvr_gem_object_get_pages(pvr_obj);
	if (err)
		goto err_out;

	spin_lock(&fw_dev->fw_mm_lock);

	if (drm_mm_node_allocated(&fw_obj->fw_mm_node)) {
		err = -EINVAL;
		goto err_unlock;
	}

	if (!dev_addr) {
		/*
		 * Allocate from the main heap only (firmware heap minus
		 * config space).
		 */
		err = drm_mm_insert_node_in_range(&fw_dev->fw_mm, &fw_obj->fw_mm_node,
						  gem_obj->size, 0, 0,
						  fw_dev->fw_heap_info.gpu_addr,
						  fw_dev->fw_heap_info.gpu_addr +
						  fw_dev->fw_heap_info.size, 0);
		if (err)
			goto err_unlock;
	} else {
		fw_obj->fw_mm_node.start = dev_addr;
		fw_obj->fw_mm_node.size = gem_obj->size;
		err = drm_mm_reserve_node(&fw_dev->fw_mm, &fw_obj->fw_mm_node);
		if (err)
			goto err_unlock;
	}

	spin_unlock(&fw_dev->fw_mm_lock);

	/* Map object on GPU. */
	err = fw_dev->funcs->vm_map(pvr_dev, fw_obj);
	if (err)
		goto err_remove_node;

	fw_obj->fw_addr_offset = (u32)(fw_obj->fw_mm_node.start - fw_dev->fw_mm_base);

	return 0;

err_remove_node:
	spin_lock(&fw_dev->fw_mm_lock);
	drm_mm_remove_node(&fw_obj->fw_mm_node);

err_unlock:
	spin_unlock(&fw_dev->fw_mm_lock);

	pvr_gem_object_put_pages(pvr_obj);

err_out:
	return err;
}

/**
 * pvr_gem_fw_vunmap() - Unmap a previously mapped FW object
 * @fw_obj: FW object to unmap.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL if object is not currently mapped.
 */
static int
pvr_gem_fw_vunmap(struct pvr_fw_object *fw_obj)
{
	struct pvr_gem_object *pvr_obj = from_pvr_fw_object(fw_obj);
	struct drm_gem_object *gem_obj = from_pvr_gem_object(pvr_obj);
	struct pvr_device *pvr_dev = to_pvr_device(gem_obj->dev);
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	int err;

	fw_dev->funcs->vm_unmap(pvr_dev, fw_obj);

	spin_lock(&fw_dev->fw_mm_lock);

	if (!drm_mm_node_allocated(&fw_obj->fw_mm_node)) {
		spin_unlock(&fw_dev->fw_mm_lock);
		err = -EINVAL;
		goto err_out;
	}

	drm_mm_remove_node(&fw_obj->fw_mm_node);

	spin_unlock(&fw_dev->fw_mm_lock);

	pvr_gem_object_put_pages(pvr_obj);

	return 0;

err_out:
	return err;
}

static int
pvr_gem_create_fw_object_common(struct pvr_device *pvr_dev, size_t size,
				u64 flags, u64 dev_addr,
				struct pvr_fw_object **fw_obj_out)
{
	struct pvr_fw_object *fw_obj;
	int err;

	/* %DRM_PVR_BO_DEVICE_PM_FW_PROTECT is implicit for FW objects. */
	flags |= DRM_PVR_BO_DEVICE_PM_FW_PROTECT;

	fw_obj = kzalloc(sizeof(*fw_obj), GFP_KERNEL);
	if (!fw_obj) {
		err = -ENOMEM;
		goto err_out;
	}

	/*
	 * All firmware objects use the same mapping flags. See
	 * %PVR_BO_FW_FLAGS_* for details.
	 */
	err = pvr_gem_object_init(pvr_dev, &fw_obj->base, size, flags, &pvr_gem_fw_object_funcs,
				  false);
	if (err)
		goto err_fw_obj_free;

	err = pvr_gem_fw_vmap(pvr_dev, fw_obj, dev_addr);
	if (err)
		goto err_release_object;

	*fw_obj_out = fw_obj;

	return 0;

err_release_object:
	pvr_gem_object_put(&fw_obj->base);

err_fw_obj_free:
	kfree(fw_obj);

err_out:
	return err;
}

/**
 * pvr_gem_create_fw_object() - Create a FW object and map to firmware
 * @pvr_dev: PowerVR device pointer.
 * @size: Size of object, in bytes.
 * @flags: Options which affect both this operation and future mapping
 * operations performed on the returned object. Must be a combination of
 * DRM_PVR_BO_* and/or PVR_BO_* flags.
 * @fw_obj_out: Pointer to location to store created object pointer.
 *
 * %DRM_PVR_BO_DEVICE_PM_FW_PROTECT is implied for all FW objects. Consequently,
 * this function will fail if @flags has %DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS
 * set.
 *
 * Returns:
 *  * 0 on success, or
 *  * Any error returned by pvr_gem_create_fw_object_common().
 */
int
pvr_gem_create_fw_object(struct pvr_device *pvr_dev, size_t size, u64 flags,
			 struct pvr_fw_object **fw_obj_out)
{
	return pvr_gem_create_fw_object_common(pvr_dev, size, flags, 0,
					       fw_obj_out);
}

static void *
pvr_gem_create_and_map_fw_common(struct pvr_device *pvr_dev, size_t size,
				 u64 flags, u64 dev_addr,
				 struct pvr_fw_object **fw_obj_out)
{
	struct pvr_fw_object *fw_obj;
	void *cpu_ptr;
	int err;

	err = pvr_gem_create_fw_object_common(pvr_dev, size, flags, dev_addr,
					      &fw_obj);
	if (err)
		goto err_out;

	cpu_ptr = pvr_fw_object_vmap(fw_obj, true);
	if (IS_ERR(cpu_ptr)) {
		err = PTR_ERR(cpu_ptr);
		goto err_put_object;
	}

	*fw_obj_out = fw_obj;

	return cpu_ptr;

err_put_object:
	pvr_fw_object_release(fw_obj);

err_out:
	return ERR_PTR(err);
}

/**
 * pvr_gem_create_and_map_fw_object() - Create a FW object and map to firmware
 *                                      and CPU
 * @pvr_dev: PowerVR device pointer.
 * @size: Size of object, in bytes.
 * @flags: Options which affect both this operation and future mapping
 * operations performed on the returned object. Must be a combination of
 * DRM_PVR_BO_* and/or PVR_BO_* flags.
 * @fw_obj_out: Pointer to location to store created object pointer.
 *
 * %DRM_PVR_BO_DEVICE_PM_FW_PROTECT is implied for all FW objects. Consequently,
 * this function will fail if @flags has %DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS
 * set.
 *
 * Caller is responsible for calling pvr_gem_object_vunmap() to release the CPU
 * mapping.
 *
 * Returns:
 *  * Pointer to CPU mapping of newly created object, or
 *  * Any error returned by pvr_gem_create_fw_object(), or
 *  * Any error returned by pvr_gem_object_vmap().
 */
void *
pvr_gem_create_and_map_fw_object(struct pvr_device *pvr_dev, size_t size,
				 u64 flags, struct pvr_fw_object **fw_obj_out)
{
	return pvr_gem_create_and_map_fw_common(pvr_dev, size, flags, 0,
						fw_obj_out);
}

/**
 * pvr_gem_create_and_map_fw_object_offset() - Create a FW object and map to
 *                                             firmware at the provided offset
 *                                             and to the CPU.
 * @pvr_dev: PowerVR device pointer.
 * @dev_offset: Base address of desired FW mapping, offset from start of FW heap.
 * @size: Size of object, in bytes.
 * @flags: Options which affect both this operation and future mapping
 * operations performed on the returned object. Must be a combination of
 * DRM_PVR_BO_* and/or PVR_BO_* flags.
 * @fw_obj_out: Pointer to location to store created object pointer.
 *
 * %DRM_PVR_BO_DEVICE_PM_FW_PROTECT is implied for all FW objects. Consequently,
 * this function will fail if @flags has %DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS
 * set.
 *
 * Caller is responsible for calling pvr_gem_object_vunmap() to release the CPU
 * mapping.
 *
 * Returns:
 *  * Pointer to CPU mapping of newly created object, or
 *  * Any error returned by pvr_gem_create_fw_object(), or
 *  * Any error returned by pvr_gem_object_vmap().
 */
void *
pvr_gem_create_and_map_fw_object_offset(struct pvr_device *pvr_dev,
					u32 dev_offset, size_t size, u64 flags,
					struct pvr_fw_object **fw_obj_out)
{
	u64 dev_addr = pvr_dev->fw_dev.fw_mm_base + dev_offset;

	return pvr_gem_create_and_map_fw_common(pvr_dev, size, flags, dev_addr,
						fw_obj_out);
}

/**
 * pvr_gem_get_dma_addr() - Get DMA address for given offset in object
 * @pvr_obj: Pointer to object to lookup address in.
 * @offset: Offset within object to lookup address at.
 * @dma_addr_out: Pointer to location to store DMA address.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL if object is not currently backed, or if @offset is out of valid
 *    range for this object.
 */
int
pvr_gem_get_dma_addr(struct pvr_gem_object *pvr_obj, u32 offset,
		     dma_addr_t *dma_addr_out)
{
	u32 accumulated_offset = 0;
	struct scatterlist *sgl;
	unsigned int sgt_idx;

	if (!pvr_obj->sgt)
		return -EINVAL;

	for_each_sgtable_dma_sg(pvr_obj->sgt, sgl, sgt_idx) {
		u32 new_offset = accumulated_offset + sg_dma_len(sgl);

		if (offset >= accumulated_offset && offset < new_offset) {
			*dma_addr_out = sg_dma_address(sgl) +
					(offset - accumulated_offset);
			return 0;
		}

		accumulated_offset = new_offset;
	}

	return -EINVAL;
}

/**
 * pvr_fw_object_release() - Release FW object and unmap from FW space
 * @fw_obj: Object to release.
 */
void pvr_fw_object_release(struct pvr_fw_object *fw_obj)
{
	WARN_ON(pvr_gem_fw_vunmap(fw_obj));
	pvr_fw_object_put(fw_obj);
}

struct drm_gem_object *
__pvr_gem_prime_import_sg_table(struct drm_device *drm_dev,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt)
{
	struct pvr_device *pvr_dev = to_pvr_device(drm_dev);
	size_t size = attach->dmabuf->size;
	struct pvr_gem_object *pvr_obj;

	pvr_obj = pvr_gem_object_create_internal(pvr_dev, size, 0, true);
	if (IS_ERR(pvr_obj))
		return ERR_CAST(pvr_obj);

	pvr_obj->sgt = sgt;

	return from_pvr_gem_object(pvr_obj);
}
