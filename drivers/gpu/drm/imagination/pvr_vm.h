/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_VM_H__
#define __PVR_VM_H__

#include "pvr_rogue_mmu_defs.h"

#include <uapi/drm/pvr_drm.h>

#include <linux/bitops.h>
#include <linux/types.h>

/* Forward declaration from "pvr_device.h" */
struct pvr_device;
struct pvr_file;

/* Forward declaration from "pvr_gem.h" */
struct pvr_gem_object;

/* Forward declaration from "pvr_vm.c" */
struct pvr_vm_context;

/* Forward declaration from <uapi/drm/pvr_drm.h> */
struct drm_pvr_ioctl_get_heap_info_args;

/**
 * DOC: Public API (constants)
 *
 * .. c:macro:: PVR_DEVICE_PAGE_SIZE
 *
 *    Fixed page size referenced by leaf nodes in the page table tree
 *    structure. In the current implementation, this value is pegged to the
 *    CPU page size (%PAGE_SIZE). It is therefore an error to specify a CPU
 *    page size which is not also a supported device page size. The supported
 *    device page sizes are: 4KiB, 16KiB, 64KiB, 256KiB, 1MiB and 2MiB.
 *
 * .. c:macro:: PVR_DEVICE_PAGE_SHIFT
 *
 *    Shift value used to efficiently multiply or divide by
 *    %PVR_DEVICE_PAGE_SIZE.
 *
 *    This value is derived from %PVR_DEVICE_PAGE_SIZE.
 *
 * .. c:macro:: PVR_DEVICE_PAGE_MASK
 *
 *    Mask used to round a value down to the nearest multiple of
 *    %PVR_DEVICE_PAGE_SIZE. When bitwise negated, it will indicate whether a
 *    value is already a multiple of %PVR_DEVICE_PAGE_SIZE.
 *
 *    This value is derived from %PVR_DEVICE_PAGE_SIZE.
 */

#define PVR_SHIFT_FROM_SIZE(size_) (__builtin_ctzll(size_))
#define PVR_MASK_FROM_SIZE(size_) (~((size_) - U64_C(1)))

/* PVR_DEVICE_PAGE_SIZE determines the page size */
#define PVR_DEVICE_PAGE_SIZE (PAGE_SIZE)
#define PVR_DEVICE_PAGE_SHIFT (PAGE_SHIFT)
#define PVR_DEVICE_PAGE_MASK (PAGE_MASK)

/* Functions defined in pvr_vm.c */

bool pvr_device_addr_is_valid(u64 device_addr);
bool pvr_device_addr_and_size_are_valid(u64 device_addr, u64 size);

struct pvr_vm_context *pvr_vm_create_context(struct pvr_device *pvr_dev,
					     bool is_userspace_context);

int pvr_vm_map(struct pvr_vm_context *vm_ctx,
	       struct pvr_gem_object *pvr_obj, u64 pvr_obj_offset,
	       u64 device_addr, u64 size);
int pvr_vm_unmap(struct pvr_vm_context *vm_ctx, u64 device_addr);

dma_addr_t pvr_vm_get_page_table_root_addr(struct pvr_vm_context *vm_ctx);

int pvr_static_data_areas_get(const struct pvr_device *pvr_dev,
			      struct drm_pvr_ioctl_dev_query_args *args);
int pvr_heap_info_get(const struct pvr_device *pvr_dev,
		      struct drm_pvr_ioctl_dev_query_args *args);
const struct drm_pvr_heap *pvr_find_heap_containing(struct pvr_device *pvr_dev,
						    u64 addr, u64 size);

struct pvr_gem_object *pvr_vm_find_gem_object(struct pvr_vm_context *vm_ctx,
					      u64 device_addr,
					      u64 *mapped_offset_out,
					      u64 *mapped_size_out);

int
pvr_vm_mmu_flush(struct pvr_device *pvr_dev);

struct pvr_fw_object *
pvr_vm_get_fw_mem_context(struct pvr_vm_context *vm_ctx);

struct pvr_vm_context *pvr_vm_context_lookup(struct pvr_file *pvr_file, u32 handle);
bool pvr_vm_context_put(struct pvr_vm_context *vm_ctx);
void pvr_destroy_vm_contexts_for_file(struct pvr_file *pvr_file);

#endif /* __PVR_VM_H__ */
