/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note OR MIT */
/* Copyright (c) 2022 Imagination Technologies Ltd. */

#ifndef __PVR_DRM_H__
#define __PVR_DRM_H__

#include "drm.h"

#include <linux/const.h>
#include <linux/types.h>

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * DOC: IOCTLS
 *
 * The PowerVR IOCTL argument structs have a few limitations in place, in
 * addition to the standard kernel restrictions:
 *
 *  - All members must be type-aligned.
 *  - The overall struct must be padded to 64-bit alignment.
 *  - Explicit padding is almost always required. This takes the form of
 *    &_padding_x members of sufficient size to pad to the next power-of-two
 *    alignment, where x is the offset into the struct in hexadecimal. Arrays
 *    are never used for alignment. Padding fields must be zeroed; this is
 *    always checked.
 *  - Unions may only appear as the last member of a struct.
 *  - Individual union members may grow in the future. The space between the
 *    end of a union member and the end of its containing union is considered
 *    "implicit padding" and must be zeroed. This is always checked.
 */

/* clang-format off */
/**
 * PVR_IOCTL() - Build a PowerVR IOCTL number
 * @_ioctl: An incrementing id for this IOCTL. Added to %DRM_COMMAND_BASE.
 * @_mode: Must be one of DRM_IO{R,W,WR}.
 * @_data: The type of the args struct passed by this IOCTL.
 *
 * The struct referred to by @_data must have a &drm_pvr_ioctl_ prefix and an
 * &_args suffix. They are therefore omitted from @_data.
 *
 * This should only be used to build the constants described below; it should
 * never be used to call an IOCTL directly.
 *
 * Return:
 * An IOCTL number to be passed to ioctl() from userspace.
 */
#define PVR_IOCTL(_ioctl, _mode, _data) \
	_mode(DRM_COMMAND_BASE + (_ioctl), struct drm_pvr_ioctl_##_data##_args)

#define DRM_IOCTL_PVR_DEV_QUERY PVR_IOCTL(0x00, DRM_IOWR, dev_query)
#define DRM_IOCTL_PVR_CREATE_BO PVR_IOCTL(0x01, DRM_IOWR, create_bo)
#define DRM_IOCTL_PVR_GET_BO_MMAP_OFFSET PVR_IOCTL(0x02, DRM_IOWR, get_bo_mmap_offset)
#define DRM_IOCTL_PVR_CREATE_VM_CONTEXT PVR_IOCTL(0x03, DRM_IOWR, create_vm_context)
#define DRM_IOCTL_PVR_DESTROY_VM_CONTEXT PVR_IOCTL(0x04, DRM_IOW, destroy_vm_context)
#define DRM_IOCTL_PVR_VM_MAP PVR_IOCTL(0x05, DRM_IOW, vm_map)
#define DRM_IOCTL_PVR_VM_UNMAP PVR_IOCTL(0x06, DRM_IOW, vm_unmap)
#define DRM_IOCTL_PVR_CREATE_CONTEXT PVR_IOCTL(0x07, DRM_IOWR, create_context)
#define DRM_IOCTL_PVR_DESTROY_CONTEXT PVR_IOCTL(0x08, DRM_IOW, destroy_context)
#define DRM_IOCTL_PVR_CREATE_FREE_LIST PVR_IOCTL(0x09, DRM_IOWR, create_free_list)
#define DRM_IOCTL_PVR_DESTROY_FREE_LIST PVR_IOCTL(0x0a, DRM_IOW, destroy_free_list)
#define DRM_IOCTL_PVR_CREATE_HWRT_DATASET PVR_IOCTL(0x0b, DRM_IOWR, create_hwrt_dataset)
#define DRM_IOCTL_PVR_DESTROY_HWRT_DATASET PVR_IOCTL(0x0c, DRM_IOW, destroy_hwrt_dataset)
#define DRM_IOCTL_PVR_SUBMIT_JOBS PVR_IOCTL(0x0d, DRM_IOW, submit_jobs)

/**
 * struct drm_pvr_obj_array - Container used to pass arrays of objects
 *
 * It is not unusual to have to extend objects to pass new parameters, and the DRM
 * ioctl infrastructure is supporting that by padding ioctl arguments with zeros
 * when the data passed by userspace is smaller than the struct defined in the
 * drm_ioctl_desc, thus keeping things backward compatible. This drm_pvr_obj_array
 * is just applying the same concepts to indirect objects passed through arrays
 * referenced from the main ioctl arguments structure: the stride basically defines
 * the size of the object passed by userspace, which allows the kernel driver to
 * pad things with zeros when it's smaller than the size of the object it expects.
 *
 * Use DRM_PVR_OBJ_ARRAY() to fill object array fields, unless you have a very
 * good reason not to.
 */
struct drm_pvr_obj_array {
	/** @stride: Stride of object struct. Used for versioning. */
	__u32 stride;

	/** @count: Number of objects in the array. */
	__u32 count;

	/** @array: User pointer to an array of objects. */
	__u64 array;
};

#define DRM_PVR_OBJ_ARRAY(cnt, ptr) \
	{ .stride = sizeof((ptr)[0]), .count = (cnt), .array = (__u64)(uintptr_t)(ptr) }

/* clang-format on */

struct drm_pvr_dev_query_gpu_info {
	/**
	 * @gpu_id: GPU identifier.
	 *
	 * For all currently supported GPUs this is the BVNC encoded as a 64-bit
	 * value as follows:
	 *
	 *    +--------+--------+--------+-------+
	 *    | 63..48 | 47..32 | 31..16 | 15..0 |
	 *    +========+========+========+=======+
	 *    | B      | V      | N      | C     |
	 *    +--------+--------+--------+-------+
	 */
	__u64 gpu_id;

	/**
	 * @num_phantoms: Number of Phantoms present.
	 */
	__u32 num_phantoms;
};

struct drm_pvr_dev_query_runtime_info {
	/*
	 * @free_list_min_pages: Minimum allowed free list size,
	 * in PM physical pages.
	 */
	__u64 free_list_min_pages;

	/*
	 * @free_list_max_pages: Maximum allowed free list size,
	 * in PM physical pages.
	 */
	__u64 free_list_max_pages;

	/*
	 * @common_store_alloc_region_size: Size of the Allocation
	 * Region within the Common Store used for coefficient and shared
	 * registers, in dwords.
	 */
	__u32 common_store_alloc_region_size;

	/**
	 * @common_store_partition_space_size: Size of the
	 * Partition Space within the Common Store for output buffers, in
	 * dwords.
	 */
	__u32 common_store_partition_space_size;

	/**
	 * @max_coeffs: Maximum coefficients, in dwords.
	 */
	__u32 max_coeffs;

	/**
	 * @cdm_max_local_mem_size_regs: Maximum amount of local
	 * memory available to a compute kernel, in dwords.
	 */
	__u32 cdm_max_local_mem_size_regs;
};

struct drm_pvr_dev_query_hwrt_info {
	/**
	 * @num_geomdatas: Number of geom data arguments
	 * required when creating a HWRT dataset.
	 */
	__u8 num_geomdatas;

	/**
	 * @num_rtdatas: Number of RT data arguments
	 * required when creating a HWRT dataset.
	 */
	__u8 num_rtdatas;

	/**
	 * @num_freelists: Number of free list data
	 * arguments required when creating a HWRT dataset.
	 */
	__u8 num_freelists;

	/** @_padding_3: Reserved - will be zeroed */
	__u8 _padding_3;

	/** @_padding_4: Reserved - will be zeroed */
	__u32 _padding_4;
};

struct drm_pvr_dev_query_quirks {
	/**
	 * @quirks: A userspace address for the hardware quirks __u32 array.
	 *
	 * The first @musthave_count items in the list are quirks that the
	 * client must support for this device. If userspace does not support
	 * all these quirks then functionality is not guaranteed and client
	 * initialisation must fail.
	 * The remaining quirks in the list affect userspace and the kernel or
	 * firmware. They are disabled by default and require userspace to
	 * opt-in. The opt-in mechanism depends on the quirk.
	 */
	__u64 quirks;

	/** @count: Length of @quirks (number of __u32). */
	__u16 count;

	/**
	 * @musthave_count: The number of entries in @quirks that are
	 * mandatory, starting at index 0.
	 */
	__u16 musthave_count;

	/** @_padding_c: Reserved. This field must be zeroed. */
	__u32 _padding_c;
};

struct drm_pvr_dev_query_enhancements {
	/**
	 * @enhancements: A userspace address for the hardware enhancements
	 * __u32 array.
	 *
	 * These enhancements affect userspace and the kernel or firmware. They
	 * are disabled by default and require userspace to opt-in. The opt-in
	 * mechanism depends on the quirk.
	 */
	__u64 enhancements;

	/** @count: Length of @enhancements (number of __u32). */
	__u16 count;

	/** @_padding_a: Reserved. This field must be zeroed. */
	__u16 _padding_a;

	/** @_padding_c: Reserved. This field must be zeroed. */
	__u32 _padding_c;
};

/**
 * enum drm_pvr_heap_id - Array index for heap info data returned by
 * DRM_PVR_DEV_QUERY_HEAP_INFO_GET.
 */
enum drm_pvr_heap_id {
	/** @DRM_PVR_HEAP_GENERAL: General purpose heap. */
	DRM_PVR_HEAP_GENERAL = 0,
	/** @DRM_PVR_HEAP_PDS_CODE_DATA: PDS code & data heap. */
	DRM_PVR_HEAP_PDS_CODE_DATA,
	/** @DRM_PVR_HEAP_USC_CODE: USC code heap. */
	DRM_PVR_HEAP_USC_CODE,
	/** @DRM_PVR_HEAP_RGNHDR: Region header heap. Only used if GPU has BRN63142. */
	DRM_PVR_HEAP_RGNHDR,
	/** @DRM_PVR_HEAP_VIS_TEST: Visibility test heap. */
	DRM_PVR_HEAP_VIS_TEST,
	/** @DRM_PVR_HEAP_TRANSFER_FRAG: Transfer fragment heap. */
	DRM_PVR_HEAP_TRANSFER_FRAG,

	/**
	 * @DRM_PVR_HEAP_COUNT: The number of heaps returned by
	 * DRM_PVR_DEV_QUERY_HEAP_INFO_GET. More heaps may be added, so this
	 * also serves as the copy limit when sent by the caller.
	 */
	DRM_PVR_HEAP_COUNT
	/* Please only add additional heaps above DRM_PVR_HEAP_COUNT! */
};

/*
 * DOC: Flags for DRM_PVR_DEV_QUERY_HEAP_INFO_GET.
 *
 * .. c:macro:: DRM_PVR_HEAP_FLAG_STATIC_CARVEOUT_AT_END
 *
 *    The static data area is at the end of the heap memory area, rather than
 *    at the beginning.
 *    The base address will be:
 *        drm_pvr_heap::base +
 *            (drm_pvr_heap::size - drm_pvr_heap::static_data_carveout_size)
 */
#define DRM_PVR_HEAP_FLAG_STATIC_CARVEOUT_AT_END _BITUL(0)

struct drm_pvr_heap {
	/** @base: Base address of heap. */
	__u64 base;

	/**
	 * @size: Size of heap, in bytes. Will be 0 if the heap is not present.
	 */
	__u64 size;

	/** @flags: Flags for this heap. See &enum drm_pvr_heap_flags. */
	__u32 flags;

	/** @page_size_log2: Log2 of page size. */
	__u32 page_size_log2;
};

/**
 * struct drm_pvr_dev_query_heap_info_args - Arguments for
 * %DRM_PVR_DEV_QUERY_HEAP_INFO_GET
 */
struct drm_pvr_dev_query_heap_info {
	/**
	 * @heaps: Array of struct drm_pvr_heap. If pointer is NULL, the count
	 * and stride will be updated with those known to the driver version, to
	 * facilitate allocation by the caller.
	 */
	struct drm_pvr_obj_array heaps;
};

enum drm_pvr_static_data_area_usage {
	/**
	 * @DRM_PVR_STATIC_DATA_AREA_EOT: End of Tile USC program.
	 *
	 * The End of Tile task runs at completion of a tile, and is responsible for emitting the
	 * tile to the Pixel Back End.
	 */
	DRM_PVR_STATIC_DATA_AREA_EOT = 0,

	/**
	 * @DRM_PVR_STATIC_DATA_AREA_FENCE: MCU fence area, used during cache flush and
	 * invalidation.
	 *
	 * This must point to valid physical memory but the contents otherwise are not used.
	 */
	DRM_PVR_STATIC_DATA_AREA_FENCE,

	/** @DRM_PVR_STATIC_DATA_AREA_VDM_SYNC: VDM sync program.
	 *
	 * The VDM sync program is used to synchronise multiple areas of the GPU hardware.
	 */
	DRM_PVR_STATIC_DATA_AREA_VDM_SYNC,

	/**
	 * @DRM_PVR_STATIC_DATA_AREA_YUV_CSC: YUV coefficients.
	 *
	 * Area contains up to 16 slots with stride of 64 bytes. Each is a 3x4 matrix of u16 fixed
	 * point numbers, with 1 sign bit, 2 integer bits and 13 fractional bits.
	 *
	 * The slots are :
	 * 0 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_RGB_IDENTITY_KHR
	 * 1 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_IDENTITY_KHR (full range)
	 * 2 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_IDENTITY_KHR (conformant range)
	 * 3 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_709_KHR (full range)
	 * 4 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_709_KHR (conformant range)
	 * 5 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_601_KHR (full range)
	 * 6 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_601_KHR (conformant range)
	 * 7 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_2020_KHR (full range)
	 * 8 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_2020_KHR (conformant range)
	 * 9 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_601_KHR (conformant range, 10 bit)
	 * 10 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_709_KHR (conformant range, 10 bit)
	 * 11 = VK_SAMPLER_YCBCR_MODEL_CONVERSION_YCBCR_2020_KHR (conformant range, 10 bit)
	 * 14 = Identity (biased)
	 * 15 = Identity
	 */
	DRM_PVR_STATIC_DATA_AREA_YUV_CSC,
};

struct drm_pvr_static_data_area {
	/**
	 * @id: Usage of static data area.
	 * See &enum drm_pvr_static_data_area_usage.
	 */
	__u16 area_usage;

	/**
	 * @location_heap_id: Array index of heap where this of static data
	 * area is located. This array is fetched using
	 * %DRM_PVR_DEV_QUERY_HEAP_INFO_GET.
	 */
	__u16 location_heap_id;

	/** @size: Size of static data area. */
	__u32 size;

	/**
	 * @offset: Offset of static data area from start of static data
	 * carveout.
	 */
	__u64 offset;
};

/**
 * struct drm_pvr_dev_query_static_data_areas_args - Arguments for
 * %DRM_PVR_DEV_QUERY_STATIC_DATA_AREAS_GET
 */
struct drm_pvr_dev_query_static_data_areas {
	/**
	 * @static_data_areas: Array of struct drm_pvr_static_data_area. If
	 * pointer is NULL, the count and stride will be updated with those
	 * known to the driver version, to facilitate allocation by the caller.
	 */
	struct drm_pvr_obj_array static_data_areas;
};

/**
 * enum drm_pvr_dev_query - Arguments for &drm_pvr_ioctl_dev_query_args.type
 *
 * Append only. Do not reorder.
 */
enum drm_pvr_dev_query {
	/* struct drm_pvr_dev_query_gpu_info */
	DRM_PVR_DEV_QUERY_GPU_INFO_GET = 0,

	/* struct drm_pvr_dev_query_runtime_info */
	DRM_PVR_DEV_QUERY_RUNTIME_INFO_GET,

	/* struct drm_pvr_dev_query_hwrt_info */
	DRM_PVR_DEV_QUERY_HWRT_INFO_GET,

	/* struct drm_pvr_dev_query_quirks */
	DRM_PVR_DEV_QUERY_QUIRKS_GET,

	/* struct drm_pvr_dev_query_enhancements */
	DRM_PVR_DEV_QUERY_ENHANCEMENTS_GET,

	/* struct drm_pvr_dev_query_heap_info */
	DRM_PVR_DEV_QUERY_HEAP_INFO_GET,

	/* struct drm_pvr_dev_query_static_data_areas */
	DRM_PVR_DEV_QUERY_STATIC_DATA_AREAS_GET,
};

/**
 * struct drm_pvr_ioctl_dev_query_args - Arguments for %DRM_IOCTL_PVR_DEV_QUERY
 */
struct drm_pvr_ioctl_dev_query_args {
	/**
	 * @type: Type of query and output struct. See enum drm_pvr_dev_query.
	 */
	__u32 type;

	/**
	 * @size: Size of the receiving struct, see @type.
	 *
	 * After a successful call this will be updated to the written byte
	 * length.
	 * Can also be used to get the minimum byte length (see @pointer).
	 * This allows additional fields to be appended to the structs in
	 * future.
	 */
	__u32 size;

	/**
	 * @pointer: [OUT] Pointer to receiving struct @type.
	 *
	 * Must be large enough to contain @size bytes.
	 * If pointer is NULL, the expected size will be returned in the @size
	 * field, but no other data will be written.
	 */
	__u64 pointer;
};

/**
 * DOC: Flags for CREATE_BO
 *
 * The &drm_pvr_ioctl_create_bo_args.flags field is 64 bits wide and consists
 * of three groups of flags: creation, device mapping and CPU mapping.
 *
 * We use "device" to refer to the GPU here because of the ambiguity between
 * CPU and GPU in some fonts.
 *
 * Creation options
 *    These use the prefix ``DRM_PVR_BO_CREATE_``.
 *
 *    :ZEROED: Require the allocated buffer to be zeroed before returning. Note
 *      that this is an active operation, and is never zero cost. Unless it is
 *      explicitly required, this option should not be set.
 *
 * Device mapping options
 *    These use the prefix ``DRM_PVR_BO_DEVICE_``.
 *
 *    :BYPASS_CACHE: There are very few situations where this flag is useful.
 *       By default, the device flushes its memory caches after every job.
 *    :PM_FW_PROTECT: Specify that only the Parameter Manager (PM) and/or
 *       firmware processor should be allowed to access this memory when mapped
 *       to the device. It is not valid to specify this flag with
 *       CPU_ALLOW_USERSPACE_ACCESS.
 *
 * CPU mapping options
 *    These use the prefix ``DRM_PVR_BO_CPU_``.
 *
 *    :ALLOW_USERSPACE_ACCESS: Allow userspace to map and access the contents
 *       of this memory. It is not valid to specify this flag with
 *       DEVICE_PM_FW_PROTECT.
 */
#define DRM_PVR_BO_DEVICE_BYPASS_CACHE _BITULL(0)
#define DRM_PVR_BO_DEVICE_PM_FW_PROTECT _BITULL(1)
#define DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS _BITULL(2)
#define DRM_PVR_BO_CREATE_ZEROED _BITULL(3)
/* Bits 4..63 are reserved. */

/**
 * struct drm_pvr_ioctl_create_bo_args - Arguments for %DRM_IOCTL_PVR_CREATE_BO
 */
struct drm_pvr_ioctl_create_bo_args {
	/**
	 * @size: [IN/OUT] Unaligned size of buffer object to create. On
	 * return, this will be populated with the actual aligned size of the
	 * new buffer.
	 */
	__u64 size;

	/**
	 * @handle: [OUT] GEM handle of the new buffer object for use in
	 * userspace.
	 */
	__u32 handle;

	/** @_padding_c: Reserved. This field must be zeroed. */
	__u32 _padding_c;

	/**
	 * @flags: [IN] Options which will affect the behaviour of this
	 * creation operation and future mapping operations on the created
	 * object. This field must be a valid combination of DRM_PVR_BO_*
	 * values, with all bits marked as reserved set to zero.
	 */
	__u64 flags;
};

/**
 * struct drm_pvr_ioctl_get_bo_mmap_offset_args - Arguments for
 * %DRM_IOCTL_PVR_GET_BO_MMAP_OFFSET
 *
 * Like other DRM drivers, the "mmap" IOCTL doesn't actually map any memory.
 * Instead, it allocates a fake offset which refers to the specified buffer
 * object. This offset can be used with a real mmap call on the DRM device
 * itself.
 */
struct drm_pvr_ioctl_get_bo_mmap_offset_args {
	/** @handle: [IN] GEM handle of the buffer object to be mapped. */
	__u32 handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;

	/** @offset: [OUT] Fake offset to use in the real mmap call. */
	__u64 offset;
};

/**
 * struct drm_pvr_ioctl_create_vm_context_args - Arguments for
 * %DRM_IOCTL_PVR_CREATE_VM_CONTEXT
 */
struct drm_pvr_ioctl_create_vm_context_args {
	/** @handle: [OUT] Handle for new VM context. */
	__u32 handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;
};

/**
 * struct drm_pvr_ioctl_destroy_vm_context_args - Arguments for
 * %DRM_IOCTL_PVR_DESTROY_VM_CONTEXT
 */
struct drm_pvr_ioctl_destroy_vm_context_args {
	/**
	 * @handle: [IN] Handle for VM context to be destroyed.
	 */
	__u32 handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;
};

/**
 * DOC: VM UAPI
 *
 * The VM UAPI allows userspace to create buffer object mappings in GPU virtual address space.
 *
 * The client is responsible for managing GPU address space. It should allocate mappings within
 * the heaps returned by %DRM_PVR_DEV_QUERY_HEAP_INFO_GET.
 *
 * %DRM_IOCTL_PVR_VM_MAP creates a new mapping. The client provides the target virtual address for
 * the mapping. Size and offset within the mapped buffer object can be specified, so the client can
 * partially map a buffer.
 *
 * %DRM_IOCTL_PVR_VM_UNMAP removes a mapping. The entire mapping will be removed from GPU address
 * space. For this reason only the start address is provided by the client.
 */

/**
 * struct drm_pvr_ioctl_vm_map_args - Arguments for %DRM_IOCTL_PVR_VM_MAP.
 */
struct drm_pvr_ioctl_vm_map_args {
	/**
	 * @vm_context_handle: [IN] Handle for VM context for this mapping to
	 *                          exist in.
	 */
	__u32 vm_context_handle;

	/** @flags: [IN] Flags which affect this mapping. Currently always 0. */
	__u32 flags;

	/**
	 * @device_addr: [IN] Requested device-virtual address for the mapping.
	 * This must be non-zero and aligned to the device page size for the
	 * heap containing the requested address. It is an error to specify an
	 * address which is not contained within one of the heaps returned by
	 * %DRM_PVR_DEV_QUERY_HEAP_INFO_GET.
	 */
	__u64 device_addr;

	/**
	 * @handle: [IN] Handle of the target buffer object. This must be a
	 * valid handle returned by %DRM_IOCTL_PVR_CREATE_BO.
	 */
	__u32 handle;

	/** @_padding_14: Reserved. This field must be zeroed. */
	__u32 _padding_14;

	/**
	 * @offset: [IN] Offset into the target bo from which to begin the
	 * mapping.
	 */
	__u64 offset;

	/**
	 * @size: [IN] Size of the requested mapping. Must be aligned to
	 * the device page size for the heap containing the requested address,
	 * as well as the host page size. When added to @device_addr, the
	 * result must not overflow the heap which contains @device_addr (i.e.
	 * the range specified by @device_addr and @size must be completely
	 * contained within a single heap specified by
	 * %DRM_PVR_DEV_QUERY_HEAP_INFO_GET).
	 */
	__u64 size;
};

/**
 * struct drm_pvr_ioctl_vm_unmap_args - Arguments for %DRM_IOCTL_PVR_VM_UNMAP.
 */
struct drm_pvr_ioctl_vm_unmap_args {
	/**
	 * @vm_context_handle: [IN] Handle for VM context that this mapping
	 *                          exists in.
	 */
	__u32 vm_context_handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;

	/**
	 * @device_addr: [IN] Device-virtual address at the start of the target
	 * mapping. This must be non-zero.
	 */
	__u64 device_addr;
};

/**
 * enum drm_pvr_ctx_priority - Arguments for
 * &drm_pvr_ioctl_create_context_args.priority
 */
enum drm_pvr_ctx_priority {
	DRM_PVR_CTX_PRIORITY_LOW = -512,
	DRM_PVR_CTX_PRIORITY_NORMAL = 0,
	/* A priority above NORMAL requires CAP_SYS_NICE or DRM_MASTER. */
	DRM_PVR_CTX_PRIORITY_HIGH = 512,
};

/* clang-format off */

/**
 * enum drm_pvr_ctx_type - Arguments for
 * &drm_pvr_ioctl_create_context_args.type
 */
enum drm_pvr_ctx_type {
	/**
	 * @DRM_PVR_CTX_TYPE_RENDER: Render context. Use &struct
	 * drm_pvr_ioctl_create_render_context_args for context creation arguments.
	 */
	DRM_PVR_CTX_TYPE_RENDER = 0,

	/**
	 * @DRM_PVR_CTX_TYPE_COMPUTE: Compute context. Use &struct
	 * drm_pvr_ioctl_create_compute_context_args for context creation arguments.
	 */
	DRM_PVR_CTX_TYPE_COMPUTE,

	/**
	 * @DRM_PVR_CTX_TYPE_TRANSFER_FRAG: Transfer context for fragment data masters. Use
	 * &struct drm_pvr_ioctl_create_transfer_context_args for context creation arguments.
	 */
	DRM_PVR_CTX_TYPE_TRANSFER_FRAG,
};

/* clang-format on */

/**
 * struct drm_pvr_ioctl_create_context_args - Arguments for
 * %DRM_IOCTL_PVR_CREATE_CONTEXT
 */
struct drm_pvr_ioctl_create_context_args {
	/**
	 * @type: [IN] Type of context to create.
	 *
	 * This must be one of the values defined by &enum drm_pvr_ctx_type.
	 */
	__u32 type;

	/** @flags: [IN] Flags for context. */
	__u32 flags;

	/**
	 * @priority: [IN] Priority of new context.
	 *
	 * This must be one of the values defined by &enum drm_pvr_ctx_priority.
	 */
	__s32 priority;

	/** @handle: [OUT] Handle for new context. */
	__u32 handle;

	/**
	 * @static_context_state: [IN] Pointer to static context state to copy to
	 *                             new context.
	 *
	 * The state differs based on the value of @type:
	 * * For %DRM_PVR_CTX_TYPE_RENDER, state should be of type
	 *   &struct rogue_fwif_static_rendercontext_state.
	 * * For %DRM_PVR_CTX_TYPE_COMPUTE, state should be of type
	 *   &struct rogue_fwif_static_computecontext_state.
	 */
	__u64 static_context_state;

	/**
	 * @static_context_state_len: [IN] Length of static context state, in bytes.
	 */
	__u32 static_context_state_len;

	/**
	 * @vm_context_handle: [IN] Handle for VM context that this context is
	 *                          associated with.
	 */
	__u32 vm_context_handle;

	/**
	 * @callstack_addr: [IN] Address for initial call stack pointer. Only valid
	 *                       if @type is %DRM_PVR_CTX_TYPE_RENDER, otherwise
	 *                       must be 0.
	 */
	__u64 callstack_addr;
};

/**
 * struct drm_pvr_ioctl_destroy_context_args - Arguments for
 * %DRM_IOCTL_PVR_DESTROY_CONTEXT
 */
struct drm_pvr_ioctl_destroy_context_args {
	/**
	 * @handle: [IN] Handle for context to be destroyed.
	 */
	__u32 handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;
};

/**
 * struct drm_pvr_ioctl_create_free_list_args - Arguments for
 * %DRM_IOCTL_PVR_CREATE_FREE_LIST
 *
 * Free list arguments have the following constraints :
 *
 * - &max_num_pages must be greater than zero.
 * - &grow_threshold must be between 0 and 100.
 * - &grow_num_pages must be less than or equal to &max_num_pages.
 * - &initial_num_pages, &max_num_pages and &grow_num_pages must be multiples
 *   of 4.
 *
 * When &grow_num_pages is 0 :
 * - &initial_num_pages must be equal to &max_num_pages
 *
 * When &grow_num_pages is non-zero :
 * - &initial_num_pages must be less than &max_num_pages.
 */
struct drm_pvr_ioctl_create_free_list_args {
	/**
	 * @free_list_gpu_addr: [IN] Address of GPU mapping of buffer object
	 *                           containing memory to be used by free list.
	 *
	 * The mapped region of the buffer object must be at least
	 * @max_num_pages * sizeof(__u32).
	 *
	 * The buffer object must have been created with
	 * %DRM_PVR_BO_DEVICE_PM_FW_PROTECT set and
	 * %DRM_PVR_BO_CPU_ALLOW_USERSPACE_ACCESS not set.
	 */
	__u64 free_list_gpu_addr;

	/** @initial_num_pages: [IN] Pages initially allocated to free list. */
	__u32 initial_num_pages;

	/** @max_num_pages: [IN] Maximum number of pages in free list. */
	__u32 max_num_pages;

	/** @grow_num_pages: [IN] Pages to grow free list by per request. */
	__u32 grow_num_pages;

	/**
	 * @grow_threshold: [IN] Percentage of FL memory used that should
	 *                       trigger a new grow request.
	 */
	__u32 grow_threshold;

	/**
	 * @vm_context_handle: [IN] Handle for VM context that the free list buffer
	 *                          object is mapped in.
	 */
	__u32 vm_context_handle;

	/**
	 * @handle: [OUT] Handle for created free list.
	 */
	__u32 handle;
};

/**
 * struct drm_pvr_ioctl_destroy_free_list_args - Arguments for
 * %DRM_IOCTL_PVR_DESTROY_FREE_LIST
 */
struct drm_pvr_ioctl_destroy_free_list_args {
	/**
	 * @handle: [IN] Handle for free list to be destroyed.
	 */
	__u32 handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;
};

struct drm_pvr_create_hwrt_geom_data_args {
	/** @tpc_dev_addr: [IN] Tail pointer cache GPU virtual address. */
	__u64 tpc_dev_addr;

	/** @tpc_size: [IN] Size of TPC, in bytes. */
	__u32 tpc_size;

	/** @tpc_stride: [IN] Stride between layers in TPC, in pages */
	__u32 tpc_stride;

	/** @vheap_table_dev_addr: [IN] VHEAP table GPU virtual address. */
	__u64 vheap_table_dev_addr;

	/** @rtc_dev_addr: [IN] Render Target Cache virtual address. */
	__u64 rtc_dev_addr;
};

struct drm_pvr_create_hwrt_rt_data_args {
	/** @pm_mlist_dev_addr: [IN] PM MLIST GPU virtual address. */
	__u64 pm_mlist_dev_addr;

	/** @macrotile_array_dev_addr: [IN] Macrotile array GPU virtual address. */
	__u64 macrotile_array_dev_addr;

	/** @region_header_dev_addr: [IN] Region header array GPU virtual address. */
	__u64 region_header_dev_addr;
};

/**
 * struct drm_pvr_ioctl_create_hwrt_dataset_args - Arguments for
 * %DRM_IOCTL_PVR_CREATE_HWRT_DATASET
 */
struct drm_pvr_ioctl_create_hwrt_dataset_args {
	/** @geom_data_args: [IN] Geometry data arguments. */
	struct drm_pvr_create_hwrt_geom_data_args geom_data_args;

	/** @rt_data_args: [IN] Array of render target arguments. */
	struct drm_pvr_create_hwrt_rt_data_args rt_data_args[2];

	/**
	 * @free_list_args: [IN] Array of free list handles.
	 *
	 * free_list_handles[0] must have initial size of at least that reported
	 * by %DRM_PVR_DEV_QUERY_RUNTIME_INFO::free_list_min_pages.
	 */
	__u32 free_list_handles[2];

	/** @width: [IN] Width in pixels. */
	__u32 width;

	/** @height: [IN] Height in pixels. */
	__u32 height;

	/** @samples: [IN] Number of samples. */
	__u32 samples;

	/** @layers: [IN] Number of layers. */
	__u32 layers;

	/** @isp_merge_lower_x: [IN] Lower X coefficient for triangle merging. */
	__u32 isp_merge_lower_x;

	/** @isp_merge_lower_y: [IN] Lower Y coefficient for triangle merging. */
	__u32 isp_merge_lower_y;

	/** @isp_merge_scale_x: [IN] Scale X coefficient for triangle merging. */
	__u32 isp_merge_scale_x;

	/** @isp_merge_scale_y: [IN] Scale Y coefficient for triangle merging. */
	__u32 isp_merge_scale_y;

	/** @isp_merge_upper_x: [IN] Upper X coefficient for triangle merging. */
	__u32 isp_merge_upper_x;

	/** @isp_merge_upper_y: [IN] Upper Y coefficient for triangle merging. */
	__u32 isp_merge_upper_y;

	/**
	 * @region_header_size: [IN] Size of region header array. This common field is used by
	 *                           both render targets in this data set.
	 *
	 * The units for this field differ depending on what version of the simple internal
	 * parameter format the device uses. If format 2 is in use then this is interpreted as the
	 * number of region headers. For other formats it is interpreted as the size in dwords.
	 */
	__u32 region_header_size;

	/**
	 * @handle: [OUT] Handle for created HWRT dataset.
	 */
	__u32 handle;
};

/**
 * struct drm_pvr_ioctl_destroy_hwrt_dataset_args - Arguments for
 * %DRM_IOCTL_PVR_DESTROY_HWRT_DATASET
 */
struct drm_pvr_ioctl_destroy_hwrt_dataset_args {
	/**
	 * @handle: [IN] Handle for HWRT dataset to be destroyed.
	 */
	__u32 handle;

	/** @_padding_4: Reserved. This field must be zeroed. */
	__u32 _padding_4;
};

/**
 * DOC: Flags for the drm_pvr_sync_op object.
 *
 * Operations
 * ~~~~~~~~~~
 * .. c:macro:: DRM_PVR_SYNC_OP_HANDLE_TYPE_MASK
 *
 *    Handle type mask for the drm_pvr_sync_op::flags field.
 *
 * .. c:macro:: DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_SYNCOBJ
 *
 *    Indicates the handle passed in drm_pvr_sync_op::handle is a syncobj handle.
 *    This is the default type.
 *
 * .. c:macro:: DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_TIMELINE_SYNCOBJ
 *
 *    Indicates the handle passed in drm_pvr_sync_op::handle is a timeline syncobj handle.
 *
 * .. c:macro:: DRM_PVR_SYNC_OP_FLAG_SIGNAL
 *
 *    Signal operation requested. The out-fence bound to the job will be attached to
 *    the syncobj whose handle is passed in drm_pvr_sync_op::handle.
 *
 * .. c:macro:: DRM_PVR_SYNC_OP_FLAG_WAIT
 *
 *    Wait operation requested. The job will wait for this particular syncobj or syncobj
 *    point to be signaled before being started.
 *    This is the default operation.
 */
#define DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_MASK 0xf
#define DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_SYNCOBJ 0
#define DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_TIMELINE_SYNCOBJ 1
#define DRM_PVR_SYNC_OP_FLAG_SIGNAL _BITULL(31)
#define DRM_PVR_SYNC_OP_FLAG_WAIT 0

#define DRM_PVR_SYNC_OP_FLAGS_MASK (DRM_PVR_SYNC_OP_FLAG_HANDLE_TYPE_MASK | \
				    DRM_PVR_SYNC_OP_FLAG_SIGNAL)

/**
 * struct drm_pvr_sync_op - Object describing a sync operation
 */
struct drm_pvr_sync_op {
	/** @handle: Handle of sync object. */
	__u32 handle;

	/** @flags: Combination of DRM_PVR_SYNC_OP_FLAG_ flags. */
	__u32 flags;

	/** @value: Timeline value for this drm_syncobj. MBZ for a binary syncobj. */
	__u64 value;
};

/**
 * DOC: Flags for SUBMIT_JOB ioctl geometry command.
 *
 * Operations
 * ~~~~~~~~~~
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_GEOM_CMD_FIRST
 *
 *    Indicates if this the first command to be issued for a render.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_GEOM_CMD_LAST
 *
 *    Indicates if this the last command to be issued for a render.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_GEOM_CMD_SINGLE_CORE
 *
 *    Forces to use single core in a multi core device.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_GEOM_CMD_FLAGS_MASK
 *
 *    Logical OR of all the geometry cmd flags.
 */
#define DRM_PVR_SUBMIT_JOB_GEOM_CMD_FIRST _BITULL(0)
#define DRM_PVR_SUBMIT_JOB_GEOM_CMD_LAST _BITULL(1)
#define DRM_PVR_SUBMIT_JOB_GEOM_CMD_SINGLE_CORE _BITULL(2)
#define DRM_PVR_SUBMIT_JOB_GEOM_CMD_FLAGS_MASK                                 \
	(DRM_PVR_SUBMIT_JOB_GEOM_CMD_FIRST |                                   \
	 DRM_PVR_SUBMIT_JOB_GEOM_CMD_LAST |                                    \
	 DRM_PVR_SUBMIT_JOB_GEOM_CMD_SINGLE_CORE)

/**
 * DOC: Flags for SUBMIT_JOB ioctl fragment command.
 *
 * Operations
 * ~~~~~~~~~~
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_FRAG_CMD_SINGLE_CORE
 *
 *    Use single core in a multi core setup.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_FRAG_CMD_DEPTHBUFFER
 *
 *    Indicates whether a depth buffer is present.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_FRAG_CMD_STENCILBUFFER
 *
 *    Indicates whether a stencil buffer is present.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_FRAG_CMD_PREVENT_CDM_OVERLAP
 *
 *    Disallow compute overlapped with this render.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_FRAG_CMD_GET_VIS_RESULTS
 *
 *    Indicates whether this render produces visibility results.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_FRAG_CMD_FLAGS_MASK
 *
 *    Logical OR of all the fragment cmd flags.
 */
#define DRM_PVR_SUBMIT_JOB_FRAG_CMD_SINGLE_CORE _BITULL(0)
#define DRM_PVR_SUBMIT_JOB_FRAG_CMD_DEPTHBUFFER _BITULL(1)
#define DRM_PVR_SUBMIT_JOB_FRAG_CMD_STENCILBUFFER _BITULL(2)
#define DRM_PVR_SUBMIT_JOB_FRAG_CMD_PREVENT_CDM_OVERLAP _BITULL(3)
#define DRM_PVR_SUBMIT_JOB_FRAG_CMD_GET_VIS_RESULTS _BITULL(5)
#define DRM_PVR_SUBMIT_JOB_FRAG_CMD_FLAGS_MASK                                 \
	(DRM_PVR_SUBMIT_JOB_FRAG_CMD_SINGLE_CORE |                             \
	 DRM_PVR_SUBMIT_JOB_FRAG_CMD_DEPTHBUFFER |                             \
	 DRM_PVR_SUBMIT_JOB_FRAG_CMD_STENCILBUFFER |                           \
	 DRM_PVR_SUBMIT_JOB_FRAG_CMD_PREVENT_CDM_OVERLAP |                     \
	 DRM_PVR_SUBMIT_JOB_FRAG_CMD_GET_VIS_RESULTS)

/**
 * DOC: Flags for SUBMIT_JOB ioctl compute command.
 *
 * Operations
 * ~~~~~~~~~~
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_PREVENT_ALL_OVERLAP
 *
 *    Disallow other jobs overlapped with this compute.
 *
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_SINGLE_CORE
 *
 *    Forces to use single core in a multi core device.
 */
#define DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_PREVENT_ALL_OVERLAP _BITULL(0)
#define DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_SINGLE_CORE _BITULL(1)
#define DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_FLAGS_MASK         \
	(DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_PREVENT_ALL_OVERLAP | \
	 DRM_PVR_SUBMIT_JOB_COMPUTE_CMD_SINGLE_CORE)

/**
 * DOC: Flags for SUBMIT_JOB ioctl transfer command.
 *
 * Operations
 * ~~~~~~~~~~
 * .. c:macro:: DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_SINGLE_CORE
 *
 *    Forces job to use a single core in a multi core device.
 */
#define DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_SINGLE_CORE _BITULL(0)

#define DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_FLAGS_MASK \
	DRM_PVR_SUBMIT_JOB_TRANSFER_CMD_SINGLE_CORE

/**
 * enum drm_pvr_job_type - Arguments for &drm_pvr_job.job_type
 */
enum drm_pvr_job_type {
	DRM_PVR_JOB_TYPE_GEOMETRY = 0,
	DRM_PVR_JOB_TYPE_FRAGMENT,
	DRM_PVR_JOB_TYPE_COMPUTE,
	DRM_PVR_JOB_TYPE_TRANSFER_FRAG,
};

/**
 * struct drm_pvr_hwrt_data_ref - Reference HWRT data
 */
struct drm_pvr_hwrt_data_ref {
	/** @set_handle: HWRT data set handle. */
	__u32 set_handle;

	/** @data_index: Index of the HWRT data inside the data set. */
	__u32 data_index;
};

/**
 * struct drm_pvr_job - Job arguments passed to the %DRM_IOCTL_PVR_SUBMIT_JOBS ioctl
 */
struct drm_pvr_job {
	/**
	 * @type: [IN] Type of job being submitted
	 *
	 * This must be one of the values defined by &enum drm_pvr_job_type.
	 */
	__u32 type;

	/**
	 * @context: [IN] Context handle.
	 *
	 * When @job_type is %DRM_PVR_JOB_TYPE_RENDER, %DRM_PVR_JOB_TYPE_COMPUTE or
	 * %DRM_PVR_JOB_TYPE_TRANSFER_FRAG, this must be a valid handle returned by
	 * %DRM_IOCTL_PVR_CREATE_CONTEXT. The type of context must be compatible
	 * with the type of job being submitted.
	 *
	 * When @job_type is %DRM_PVR_JOB_TYPE_NULL, this must be zero.
	 */
	__u32 context_handle;

	/**
	 * @flags: [IN] Flags for command.
	 *
	 * Those are job-dependent. See DRM_PVR_SUBMIT_JOB_xxx_
	 */
	__u32 flags;

	/**
	 * @cmd_stream_len: [IN] Length of command stream, in bytes.
	 */
	__u32 cmd_stream_len;

	/**
	 * @cmd_stream: [IN] Pointer to command stream for command.
	 *
	 * The command stream must be u64-aligned.
	 */
	__u64 cmd_stream;

	/** @sync_ops: [IN] Fragment sync operations. */
	struct drm_pvr_obj_array sync_ops;

	/**
	 * @hwrt: [IN] HWRT data used by render jobs (geometry or fragment).
	 *
	 * Must be zero for non-render jobs.
	 */
	struct drm_pvr_hwrt_data_ref hwrt;
};

/**
 * struct drm_pvr_ioctl_submit_jobs_args - Arguments for %DRM_IOCTL_PVR_SUBMIT_JOB
 */
struct drm_pvr_ioctl_submit_jobs_args {
	/** @jobs: [IN] Array of jobs to submit. */
	struct drm_pvr_obj_array jobs;
};

/* Definitions for coredump decoding in userspace. */

#define PVR_COREDUMP_HEADER_MAGIC 0x21525650 /* PVR! */
#define PVR_COREDUMP_HEADER_VERSION_MAJ 1
#define PVR_COREDUMP_HEADER_VERSION_MIN 0

/**
 * struct pvr_coredump_header - Header of PowerVR coredump
 */
struct pvr_coredump_header {
	/** @magic: Will be %PVR_COREDUMP_HEADER_MAGIC. */
	__u32 magic;
	/** @major_version: Will be %PVR_COREDUMP_HEADER_VERSION_MAJ. */
	__u32 major_version;
	/** @minor_version: Will be %PVR_COREDUMP_HEADER_VERSION_MIN. */
	__u32 minor_version;
	/** @flags: Flags for this coredump. Currently no flags are defined, this should be zero. */
	__u32 flags;
	/** @size: Size of coredump (including this header) in bytes. */
	__u32 size;
	/** @padding: Reserved. This field must be zero. */
	__u32 padding;
};

/**
 * enum pvr_coredump_block_type - Valid coredump block types
 */
enum pvr_coredump_block_type {
	/**
	 * %PVR_COREDUMP_BLOCK_TYPE_DEVINFO: Device information block.
	 *
	 * Block data is &struct pvr_coredump_block_devinfo.
	 */
	PVR_COREDUMP_BLOCK_TYPE_DEVINFO = 0,

	/**
	 * %PVR_COREDUMP_BLOCK_TYPE_REGISTERS: Register block.
	 *
	 * Block data is an array of &struct pvr_coredump_block_register. Number of registers is
	 * determined by block size.
	 */
	PVR_COREDUMP_BLOCK_TYPE_REGISTERS,

	/**
	 * %PVR_COREDUMP_BLOCK_TYPE_CONTEXT_RESET_DATA: Context reset data block.
	 *
	 * Block data is &struct pvr_coredump_block_reset_data.
	 */
	PVR_COREDUMP_BLOCK_TYPE_CONTEXT_RESET_DATA,

	/**
	 * %PVR_COREDUMP_BLOCK_TYPE_HWRINFO: Hardware Reset information block.
	 *
	 * Block data is &struct pvr_coredump_block_hwrinfo.
	 */
	PVR_COREDUMP_BLOCK_TYPE_HWRINFO,
};

/**
 * struct pvr_coredump_block_header - Header of PowerVR coredump block
 *
 * Block data immediately follows this header. The format is determined by @type.
 */
struct pvr_coredump_block_header {
	/** @type: Block type. One of %PVR_COREDUMP_BLOCK_TYPE_*. */
	__u32 type;
	/** @size: Size of block data following this header, in bytes. */
	__u32 size;
	/** @flags: Type dependent flags. */
	__u32 flags;
	/** @padding: Reserved. This field must be zero. */
	__u32 padding;
};

#define PVR_COREDUMP_PROCESS_NAME_LEN 16
#define PVR_COREDUMP_VERSION_LEN      65
#define PVR_COREDUMP_DEVINFO_PADDING (8 - ((PVR_COREDUMP_PROCESS_NAME_LEN + \
					    PVR_COREDUMP_VERSION_LEN) & 7))

/**
 * struct pvr_coredump_block_devinfo - Device information block
 */
struct pvr_coredump_block_devinfo {
	/** @gpu_id: GPU ID. */
	__u64 gpu_id;
	/** @fw_version: Version of PowerVR firmware on system that created the coredump. */
	struct {
		/** @major: Major version number. */
		__u32 major;
		/** @minor: Minor version number. */
		__u32 minor;
	} fw_version;
	/** @process_name: Name of process that submitted the failed job. */
	char process_name[PVR_COREDUMP_PROCESS_NAME_LEN];
	/** @kernel_version: String of kernel version on system that created the coredump. */
	char kernel_version[PVR_COREDUMP_VERSION_LEN];
	/** @padding: Reserved. This field must be zero. */
	__u8 padding[PVR_COREDUMP_DEVINFO_PADDING];
};

/** %PVR_COREDUMP_REGISTER_FLAG_SIZE_MASK: Mask of register size field. */
#define PVR_COREDUMP_REGISTER_FLAG_SIZE_MASK 7
/** %PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT: Register is 32-bits wide. */
#define PVR_COREDUMP_REGISTER_FLAG_SIZE_32BIT 2
/** %PVR_COREDUMP_REGISTER_FLAG_SIZE_64BIT: Register is 64-bits wide. */
#define PVR_COREDUMP_REGISTER_FLAG_SIZE_64BIT 3

/**
 * struct pvr_coredump_block_register - PowerVR register dump
 */
struct pvr_coredump_block_register {
	/** @offset: Offset of register. */
	__u32 offset;
	/** @flags: Flags for this register. Combination of %PVR_COREDUMP_REGISTER_FLAG_*. */
	__u32 flags;
	/** @value: Value of register. */
	__u64 value;
};

/** %PVR_COREDUMP_RESET_DATA_FLAG_PF: Set if a page fault happened. */
#define PVR_COREDUMP_RESET_DATA_FLAG_PF _BITUL(0)
/** %PVR_COREDUMP_RESET_DATA_FLAG_ALL_CTXS: Set if reset applicable to all contexts. */
#define PVR_COREDUMP_RESET_DATA_FLAG_ALL_CTXS _BITUL(1)

/** %PVR_COREDUMP_RESET_REASON_NONE: No reset reason recorded. */
#define PVR_COREDUMP_RESET_REASON_NONE 0
/** %PVR_COREDUMP_RESET_REASON_GUILTY_LOCKUP: Caused a reset due to locking up. */
#define PVR_COREDUMP_RESET_REASON_GUILTY_LOCKUP 1
/** %PVR_COREDUMP_RESET_REASON_INNOCENT_LOCKUP: Affected by another context locking up. */
#define PVR_COREDUMP_RESET_REASON_INNOCENT_LOCKUP 2
/** %PVR_COREDUMP_RESET_REASON_GUILTY_OVERRUNING: Overran the global deadline. */
#define PVR_COREDUMP_RESET_REASON_GUILTY_OVERRUNING 3
/** %PVR_COREDUMP_RESET_REASON_INNOCENT_OVERRUNING: Affected by another context overrunning. */
#define PVR_COREDUMP_RESET_REASON_INNOCENT_OVERRUNING 4
/** %PVR_COREDUMP_RESET_REASON_HARD_CONTEXT_SWITCH: Forced reset to meet scheduling requirements. */
#define PVR_COREDUMP_RESET_REASON_HARD_CONTEXT_SWITCH 5
/** %PVR_COREDUMP_RESET_REASON_FW_WATCHDOG: FW Safety watchdog triggered. */
#define PVR_COREDUMP_RESET_REASON_FW_WATCHDOG 12
/** %PVR_COREDUMP_RESET_REASON_FW_PAGEFAULT: FW page fault (no HWR). */
#define PVR_COREDUMP_RESET_REASON_FW_PAGEFAULT 13
/** %PVR_COREDUMP_RESET_REASON_FW_EXEC_ERR: FW execution error (GPU reset requested). */
#define PVR_COREDUMP_RESET_REASON_FW_EXEC_ERR 14
/** %PVR_COREDUMP_RESET_REASON_HOST_WDG_FW_ERR: Host watchdog detected FW error. */
#define PVR_COREDUMP_RESET_REASON_HOST_WDG_FW_ERR 15
/** %PVR_COREDUMP_RESET_REASON_GEOM_OOM_DISABLED: Geometry DM OOM event is not allowed. */
#define PVR_COREDUMP_RESET_REASON_GEOM_OOM_DISABLED 16

/** %PVR_COREDUMP_DM_GP: General purpose Data Master. */
#define PVR_COREDUMP_DM_GP 0
/** %PVR_COREDUMP_DM_2D: 2D Data Master. */
#define PVR_COREDUMP_DM_2D 1
/** %PVR_COREDUMP_DM_GEOM: Geometry Data Master. */
#define PVR_COREDUMP_DM_GEOM 2
/** %PVR_COREDUMP_DM_FRAG: Fragment Data Master. */
#define PVR_COREDUMP_DM_FRAG 3
/** %PVR_COREDUMP_DM_CDM: Compute Data Master. */
#define PVR_COREDUMP_DM_CDM 4
/** %PVR_COREDUMP_DM_RAY: Ray tracing Data Master. */
#define PVR_COREDUMP_DM_RAY 5
/** %PVR_COREDUMP_DM_GEOM2: Geometry 2 Data Master. */
#define PVR_COREDUMP_DM_GEOM2 6
/** %PVR_COREDUMP_DM_GEOM3: Geometry 3 Data Master. */
#define PVR_COREDUMP_DM_GEOM3 7
/** %PVR_COREDUMP_DM_GEOM4: Geometry 4 Data Master. */
#define PVR_COREDUMP_DM_GEOM4 8

/**
 * struct pvr_coredump_block_reset_data - Firmware context reset data
 */
struct pvr_coredump_block_reset_data {
	/** @context_id: FW ID of context affected by the reset */
	__u32 context_id;
	/** @reset_reason: Reason for reset. One of %PVR_COREDUMP_RESET_REASON_*. */
	__u32 reset_reason;
	/** @dm: Data Master affected by the reset. One of %PVR_COREDUMP_DM_. */
	__u32 dm;
	/** @reset_job_ref: Internal job ref running at the time of reset. */
	__u32 reset_job_ref;
	/** @flags: Reset data flags. Combination of %PVR_COREDUMP_RESET_DATA_FLAG_*. */
	__u32 flags;
	/** @padding: Reserved. This field must be zero. */
	__u32 padding;
	/**
	 * @fault_address: Page fault address. Only valid when %PVR_COREDUMP_RESET_DATA_FLAG_PF is
	 *                 set in @flags.
	 */
	__u64 fault_address;
};

/** %PVR_COREDUMP_HWRTYPE_UNKNOWNFAILURE: HWR triggered by unknown failure. */
#define PVR_COREDUMP_HWRTYPE_UNKNOWNFAILURE 0
/** %PVR_COREDUMP_HWRTYPE_OVERRUN: HWR triggered by overrun. */
#define PVR_COREDUMP_HWRTYPE_OVERRUN 1
/** %PVR_COREDUMP_HWRTYPE_POLLFAILURE: HWR triggered by poll timeout. */
#define PVR_COREDUMP_HWRTYPE_POLLFAILURE 2
/** %PVR_COREDUMP_HWRTYPE_BIF0FAULT: HWR triggered by fault from Bus Interface 0. */
#define PVR_COREDUMP_HWRTYPE_BIF0FAULT 3
/** %PVR_COREDUMP_HWRTYPE_BIF1: HWR triggered by fault from Bus Interface 1. */
#define PVR_COREDUMP_HWRTYPE_BIF1FAULT 4
/** %PVR_COREDUMP_HWRTYPE_TEXASBIF0FAULT: HWR triggered by fault from Texas Bus Interface 0. */
#define PVR_COREDUMP_HWRTYPE_TEXASBIF0FAULT 5
/** %PVR_COREDUMP_HWRTYPE_MMUFAULT: HWR triggered by MMU fault. */
#define PVR_COREDUMP_HWRTYPE_MMUFAULT 6
/** %PVR_COREDUMP_HWRTYPE_MMUMETAFAULT: HWR triggered by MMU fault caused by META FW processor. */
#define PVR_COREDUMP_HWRTYPE_MMUMETAFAULT 7
/** %PVR_COREDUMP_HWRTYPE_MIPSTLBFAULT: HWR triggered by TLB fault from MIPS FW processor. */
#define PVR_COREDUMP_HWRTYPE_MIPSTLBFAULT 8
/** %PVR_COREDUMP_HWRTYPE_ECCFAULT: HWR triggered by ECC fault. */
#define PVR_COREDUMP_HWRTYPE_ECCFAULT 9
/** %PVR_COREDUMP_HWRTYPE_MMURISCVFAULT: HWR triggered by MMU fault from RISC-V FW processor. */
#define PVR_COREDUMP_HWRTYPE_MMURISCVFAULT 10

/* DM is working if all flags are cleared */
#define PVR_COREDUMP_HWRINFO_DM_STATE_WORKING 0
/* DM is idle and ready for HWR */
#define PVR_COREDUMP_HWRINFO_DM_STATE_READY_FOR_HWR _BITUL(0)
/* DM need to skip to next cmd before resuming processing */
#define PVR_COREDUMP_HWRINFO_DM_STATE_NEEDS_SKIP _BITUL(2)
/* DM need partial render cleanup before resuming processing */
#define PVR_COREDUMP_HWRINFO_DM_STATE_NEEDS_PR_CLEANUP _BITUL(3)
/* DM need to increment Recovery Count once fully recovered */
#define PVR_COREDUMP_HWRINFO_DM_STATE_NEEDS_TRACE_CLEAR _BITUL(4)
/* DM was identified as locking up and causing HWR */
#define PVR_COREDUMP_HWRINFO_DM_STATE_GUILTY_LOCKUP _BITUL(5)
/* DM was innocently affected by another lockup which caused HWR */
#define PVR_COREDUMP_HWRINFO_DM_STATE_INNOCENT_LOCKUP _BITUL(6)
/* DM was identified as over-running and causing HWR */
#define PVR_COREDUMP_HWRINFO_DM_STATE_GUILTY_OVERRUNING _BITUL(7)
/* DM was innocently affected by another DM over-running which caused HWR */
#define PVR_COREDUMP_HWRINFO_DM_STATE_INNOCENT_OVERRUNING _BITUL(8)
/* DM was forced into HWR as it delayed more important workloads */
#define PVR_COREDUMP_HWRINFO_DM_STATE_HARD_CONTEXT_SWITCH _BITUL(9)
/* DM was forced into HWR due to an uncorrected GPU ECC error */
#define PVR_COREDUMP_HWRINFO_DM_STATE_GPU_ECC_HWR _BITUL(10)

struct pvr_coredump_hwrinfo_bifinfo {
	/** @bif_req_status: Request status for affected BIF. */
	__u64 bif_req_status;
	/** @bif_mmu_status: MMU status for affected BIF. */
	__u64 bif_mmu_status;
};

struct pvr_coredump_hwrinfo_eccinfo {
	/** @fault_gpu: GPU fault information. */
	__u32 fault_gpu;
};

struct pvr_coredump_hwrinfo_mmuinfo {
	/** @mmu_status: MMU status. */
	__u64 mmu_status[2];
};

struct pvr_coredump_hwrinfo_pollinfo {
	/** @thread_num: Number of thread which timed out on a poll. */
	__u32 thread_num;
	/** @cr_poll_addr: Address of timed out poll. */
	__u32 cr_poll_addr;
	/** @cr_poll_mask: Mask of timed out poll. */
	__u32 cr_poll_mask;
	/** @cr_poll_last_value: Last value read from polled location. */
	__u32 cr_poll_last_value;
};

struct pvr_coredump_hwrinfo_tlbinfo {
	/** @bad_addr: Virtual address of failed access. */
	__u32 bad_addr;
	/** @entry_lo: MIPS TLB EntryLo for failed access. */
	__u32 entry_lo;
};

/**
 * struct pvr_coredump_block_hwrinfo - Firmware hardware reset information
 */
struct pvr_coredump_block_hwrinfo {
	/** @hwr_type: Type of HWR event. One of %PVR_COREDUMP_HWRTYPE_*. */
	__u32 hwr_type;
	/** @dm: Data Master affected by the HWR event. One of %PVR_COREDUMP_DM_. */
	__u32 dm;
	/** @core_id: ID of GPU core affected by the HWR event. */
	__u32 core_id;
	/** @event_status: Event status of Data Master. */
	__u32 event_status;
	/** @dm_state: Data Master state. Combination of %PVR_COREDUMP_HWRINFO_DM_STATE_. */
	__u32 dm_state;
	/** @active_hwrt_data: FW address of affected HWRT data. */
	__u32 active_hwrt_data;

	/** @hwr_data: HWR type specific data. Determined by @hwr_type. */
	union {
		/**
		 * @bif_info: Bus Interface specific information.
		 *
		 * Used for %PVR_COREDUMP_HWRTYPE_BIF0FAULT, %PVR_COREDUMP_HWRTYPE_BIF1FAULT,
		 * %PVR_COREDUMP_HWRTYPE_TEXASBIF0FAULT and %PVR_COREDUMP_HWRTYPE_MMURISCVFAULT.
		 */
		struct pvr_coredump_hwrinfo_bifinfo bif_info;

		/**
		 * @mmu_info: MMU specific information.
		 *
		 * Used for %PVR_COREDUMP_HWRTYPE_MMUFAULT and %PVR_COREDUMP_HWRTYPE_MMUMETAFAULT.
		 */
		struct pvr_coredump_hwrinfo_mmuinfo mmu_info;

		/**
		 * @poll_info: Poll timeout specific information.
		 *
		 * Used for %PVR_COREDUMP_HWRTYPE_POLLFAILURE.
		 */
		struct pvr_coredump_hwrinfo_pollinfo poll_info;

		/**
		 * @tlb_info: MIPS TLB specific information.
		 *
		 * Used for %PVR_COREDUMP_HWRTYPE_MIPSTLBFAULT.
		 */
		struct pvr_coredump_hwrinfo_tlbinfo tlb_info;

		/**
		 * @ecc_info: ECC specific information.
		 *
		 * Used for %PVR_COREDUMP_HWRTYPE_ECCFAULT.
		 */
		struct pvr_coredump_hwrinfo_eccinfo ecc_info;
	} hwr_data;
};

#if defined(__cplusplus)
}
#endif

#endif /* __PVR_DRM_H__ */
