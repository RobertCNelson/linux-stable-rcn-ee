/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_GEM_H__
#define __PVR_GEM_H__

#include "pvr_fw.h"
#include "pvr_rogue_heap_config.h"
#include "pvr_rogue_meta.h"

#include <uapi/drm/pvr_drm.h>

#include <drm/drm_gem.h>
#include <drm/drm_mm.h>

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/const.h>
#include <linux/compiler_attributes.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/types.h>

/* Forward declaration from "pvr_device.h". */
struct pvr_device;
struct pvr_file;

/**
 * DOC: Flags for DRM_IOCTL_PVR_CREATE_BO (kernel-only)
 *
 * Kernel-only values allowed in &pvr_gem_object->flags. The majority of options
 * for this field are specified in the UAPI header "pvr_drm.h" with a
 * DRM_PVR_BO_ prefix. To distinguish these internal options (which must exist
 * in ranges marked as "reserved" in the UAPI header), we drop the DRM prefix.
 * The public options should be used directly, DRM prefix and all.
 *
 * To avoid potentially confusing gaps in the UAPI options, these kernel-only
 * options are specified "in reverse", starting at bit 63.
 *
 * We use "reserved" to refer to bits defined here and not exposed in the UAPI.
 * Bits not defined anywhere are "undefined".
 *
 * Creation options
 *    These use the prefix PVR_BO_CREATE_.
 *
 *    *There are currently no kernel-only flags in this group.*
 *
 * Device mapping options
 *    These use the prefix PVR_BO_DEVICE_.
 *
 *    *There are currently no kernel-only flags in this group.*
 *
 * CPU mapping options
 *    These use the prefix PVR_BO_CPU_.
 *
 *    :CACHED: By default, all GEM objects are mapped write-combined on the
 *       CPU. Set this flag to override this behaviour and map the object
 *       cached.
 */
#define PVR_BO_CPU_CACHED BIT_ULL(63)

/* Bits 62..4 are undefined. */
/* Bits 3..0 are defined in the UAPI. */

/* Other utilities. */
#define PVR_BO_UNDEFINED_MASK GENMASK_ULL(62, 4)
#define PVR_BO_RESERVED_MASK (PVR_BO_UNDEFINED_MASK | GENMASK_ULL(63, 63))

/*
 * All firmware-mapped memory uses (mostly) the same flags. Specifically,
 * firmware-mapped memory should be:
 *  * Read/write on the device,
 *  * Read/write on the CPU, and
 *  * Write-combined on the CPU.
 *
 * The only variation is in caching on the device.
 */
#define PVR_BO_FW_FLAGS_DEVICE_CACHED (ULL(0))
#define PVR_BO_FW_FLAGS_DEVICE_UNCACHED DRM_PVR_BO_DEVICE_BYPASS_CACHE

/**
 * struct pvr_gem_object - powervr-specific wrapper for &struct drm_gem_object
 */
struct pvr_gem_object {
	/**
	 * @base: The underlying &struct drm_gem_object.
	 *
	 * Do not access this member directly, instead call
	 * from_pvr_gem_object().
	 */
	struct drm_gem_object base;

	/** @pvr_dev: Owning PowerVR device. */
	struct pvr_device *pvr_dev;

	/**
	 * @lock: Mutex protecting @pages_ref_count, @fw_mm_ref_count,
	 *        @vmap_ref_count and @vmap_cpu_addr, and writes to @pages, @sgt
	 *        and @mm_node.
	 */
	struct mutex lock;

	/**
	 * @pages_ref_count: Reference count for @pages. @lock must be held when
	 *                   accessing.
	 */
	int pages_ref_count;

	/**
	 * @pages: Array of page structures representing the memory backing
	 *         this object. @lock must be held when writing.
	 *         pvr_gem_get_pages() must be called before reading.
	 */
	struct page **pages;

	/**
	 * @sgt: Scatter-gather table representing the memory backing this
	 *       object. @lock must be held when writing. pvr_gem_get_pages()
	 *       must be called before reading.
	 */
	struct sg_table *sgt;

	/**
	 * @flags: Options set at creation-time. Some of these options apply to
	 * the creation operation itself (which are stored here for reference)
	 * with the remainder used for mapping options to both the device and
	 * CPU. These are used every time this object is mapped, but may be
	 * changed after creation.
	 *
	 * Must be a combination of DRM_PVR_BO_* and/or PVR_BO_* flags.
	 *
	 * .. note::
	 *
	 *    This member is declared const to indicate that none of these
	 *    options may change or be changed throughout the object's
	 *    lifetime.
	 */
	const u64 flags;

	/**
	 * @vmap_ref_count: Reference count for @vmap_cpu_addr. @lock must be
	 *                  held when accessing.
	 */
	int vmap_ref_count;

	/**
	 * @vmap_cpu_addr: CPU address of vmap mapping. Will be %NULL if object
	 *                 is not mapped. @lock must be held when accessing.
	 */
	void *vmap_cpu_addr;
};

/**
 * struct pvr_fw_object - container for firmware memory allocations
 */
struct pvr_fw_object {
	/** @base: The underlying PVR GEM object backing this allocation. */
	struct pvr_gem_object base;

	/**
	 * @fw_mm_node: Node representing mapping in FW address space. @pvr_obj->lock must
	 *              be held when writing.
	 */
	struct drm_mm_node fw_mm_node;

	/**
	 * @fw_addr_offset: Virtual address offset of firmware mapping. Only
	 *                  valid if @flags has %PVR_GEM_OBJECT_FLAGS_FW_MAPPED
	 *                  set.
	 */
	u32 fw_addr_offset;
};

static __always_inline struct drm_gem_object *
from_pvr_gem_object(struct pvr_gem_object *pvr_obj)
{
	return &pvr_obj->base;
}

static __always_inline struct pvr_gem_object *
to_pvr_gem_object(struct drm_gem_object *gem_obj)
{
	return container_of(gem_obj, struct pvr_gem_object, base);
}

static __always_inline struct pvr_gem_object *
from_pvr_fw_object(struct pvr_fw_object *fw_obj)
{
	return &fw_obj->base;
}

static __always_inline struct pvr_fw_object *
to_pvr_fw_object(struct pvr_gem_object *pvr_obj)
{
	return container_of(pvr_obj, struct pvr_fw_object, base);
}

/* Functions defined in pvr_gem.c */

struct drm_gem_object *
__pvr_gem_prime_import_sg_table(struct drm_device *drm_dev,
				struct dma_buf_attachment *attach,
				struct sg_table *sgt);

struct pvr_gem_object *pvr_gem_object_create(struct pvr_device *pvr_dev,
					     size_t size, u64 flags);

int pvr_gem_object_into_handle(struct pvr_gem_object *pvr_obj,
			       struct pvr_file *pvr_file, u32 *handle);
struct pvr_gem_object *pvr_gem_object_from_handle(struct pvr_file *pvr_file,
						  u32 handle);

int pvr_gem_object_get_pages(struct pvr_gem_object *pvr_obj);
void pvr_gem_object_put_pages(struct pvr_gem_object *pvr_obj);

void *pvr_gem_object_vmap(struct pvr_gem_object *pvr_obj, bool sync_to_cpu);
void pvr_gem_object_vunmap(struct pvr_gem_object *pvr_obj, bool sync_to_device);

int pvr_gem_create_fw_object(struct pvr_device *pvr_dev, size_t size, u64 flags,
			     struct pvr_fw_object **pvr_obj_out);

void *pvr_gem_create_and_map_fw_object(struct pvr_device *pvr_dev, size_t size,
				       u64 flags,
				       struct pvr_fw_object **pvr_obj_out);

void *
pvr_gem_create_and_map_fw_object_offset(struct pvr_device *pvr_dev,
					u32 dev_offset, size_t size, u64 flags,
					struct pvr_fw_object **pvr_obj_out);

int pvr_gem_get_dma_addr(struct pvr_gem_object *pvr_obj, u32 offset,
			 dma_addr_t *dma_addr_out);

/**
 * pvr_gem_object_get() - Acquire reference on pvr_gem_object
 * @pvr_obj: Pointer to object to acquire reference on.
 */
static __always_inline void
pvr_gem_object_get(struct pvr_gem_object *pvr_obj)
{
	drm_gem_object_get(from_pvr_gem_object(pvr_obj));
}

/**
 * pvr_gem_object_put() - Release reference on pvr_gem_object
 * @pvr_obj: Pointer to object to release reference on.
 */
static __always_inline void
pvr_gem_object_put(struct pvr_gem_object *pvr_obj)
{
	drm_gem_object_put(from_pvr_gem_object(pvr_obj));
}

static __always_inline size_t
pvr_gem_object_size(struct pvr_gem_object *pvr_obj)
{
	return from_pvr_gem_object(pvr_obj)->size;
}

/**
 * pvr_gem_object_is_imported() - Return whether an object is imported
 * @pvr_obj: Pointer to object to test.
 *
 * Returns:
 *  * %true if object is imported, or
 *  * %false if object is not imported.
 */
static __always_inline bool
pvr_gem_object_is_imported(struct pvr_gem_object *pvr_obj)
{
	return pvr_obj->base.import_attach;
}

void pvr_fw_object_release(struct pvr_fw_object *fw_obj);

static __always_inline void *
pvr_fw_object_vmap(struct pvr_fw_object *fw_obj, bool sync_to_cpu)
{
	return pvr_gem_object_vmap(from_pvr_fw_object(fw_obj), sync_to_cpu);
}

static __always_inline void
pvr_fw_object_vunmap(struct pvr_fw_object *fw_obj, bool sync_to_device)
{
	pvr_gem_object_vunmap(from_pvr_fw_object(fw_obj), sync_to_device);
}

/**
 * pvr_fw_object_get() - Acquire reference on pvr_fw_object
 * @fw_obj: Pointer to object to acquire reference on.
 */
static __always_inline void
pvr_fw_object_get(struct pvr_fw_object *fw_obj)
{
	pvr_gem_object_get(from_pvr_fw_object(fw_obj));
}

/**
 * pvr_fw_object_put() - Release reference on pvr_fw_object
 * @fw_obj: Pointer to object to release reference on.
 *
 * Note: This function must _not_ be used to release the reference obtained at
 * creation time via pvr_gem_create_fw_object(). Use pvr_fw_object_release()
 * instead.
 */
static __always_inline void
pvr_fw_object_put(struct pvr_fw_object *fw_obj)
{
	pvr_gem_object_put(from_pvr_fw_object(fw_obj));
}

/**
 * pvr_fw_get_dma_addr() - Get DMA address for given offset in firmware object
 * @fw_obj: Pointer to object to lookup address in.
 * @offset: Offset within object to lookup address at.
 * @dma_addr_out: Pointer to location to store DMA address.
 *
 * Returns:
 *  * 0 on success, or
 *  * -%EINVAL if object is not currently backed, or if @offset is out of valid
 *    range for this object.
 */
static __always_inline int
pvr_fw_get_dma_addr(struct pvr_fw_object *fw_obj, u32 offset, dma_addr_t *dma_addr_out)
{
	return pvr_gem_get_dma_addr(from_pvr_fw_object(fw_obj), offset, dma_addr_out);
}

/**
 * pvr_gem_get_fw_addr_offset() - Return address of object in firmware address space, with given
 *                                offset.
 * @fw_obj: Pointer to object.
 * @offset: Desired offset from start of object.
 * @fw_addr_out: Location to store address to.
 */
static __always_inline void
pvr_gem_get_fw_addr_offset(struct pvr_fw_object *fw_obj, u32 offset, u32 *fw_addr_out)
{
	struct pvr_gem_object *pvr_obj = from_pvr_fw_object(fw_obj);
	struct pvr_device *pvr_dev = pvr_obj->pvr_dev;

	*fw_addr_out = pvr_dev->fw_dev.funcs->get_fw_addr_with_offset(fw_obj, offset);
}

/**
 * pvr_gem_get_fw_addr() - Return address of object in firmware address space
 * @fw_obj: Pointer to object.
 * @fw_addr_out: Location to store address to.
 */
static __always_inline void
pvr_gem_get_fw_addr(struct pvr_fw_object *fw_obj, u32 *fw_addr_out)
{
	pvr_gem_get_fw_addr_offset(fw_obj, 0, fw_addr_out);
}

/**
 * pvr_gem_get_fw_gpu_addr() - Return address of object in GPU address space
 * @fw_obj: Pointer to object.
 * @gpu_addr_out: Location to store address to.
 *
 * Note that this function is not valid if firmware processor is MIPS.
 *
 * Returns :
 *  * %true on success, or
 *  * %false if object is not mapped to firmware address space.
 */
static __always_inline bool
pvr_gem_get_fw_gpu_addr(struct pvr_fw_object *fw_obj, u64 *gpu_addr_out)
{
	/* FIXME: Move to META-specific file */
	struct pvr_gem_object *pvr_obj = from_pvr_fw_object(fw_obj);
	struct pvr_device *pvr_dev = pvr_obj->pvr_dev;
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;

	if (fw_dev->processor_type != PVR_FW_PROCESSOR_TYPE_MIPS) {
		*gpu_addr_out = fw_obj->fw_addr_offset + fw_dev->fw_heap_info.gpu_addr;
		return true;
	}

	return false;
}

#endif /* __PVR_GEM_H__ */
