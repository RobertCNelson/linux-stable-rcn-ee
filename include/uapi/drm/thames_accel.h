/* SPDX-License-Identifier: MIT */
/* Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/ */
#ifndef _THAMES_DRM_H_
#define _THAMES_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * DOC: IOCTL IDs
 *
 * enum drm_thames_ioctl_id - IOCTL IDs
 *
 * Place new ioctls at the end, don't re-order, don't replace or remove entries.
 *
 * These IDs are not meant to be used directly. Use the DRM_IOCTL_THAMES_xxx
 * definitions instead.
 */
enum drm_thames_ioctl_id {
	/** @DRM_THAMES_BO_CREATE: Create a buffer object. */
	DRM_THAMES_BO_CREATE,

	/**
	 * @DRM_THAMES_BO_MMAP_OFFSET: Get the file offset to pass to
	 * mmap to map a GEM object.
	 */
	DRM_THAMES_BO_MMAP_OFFSET,
};

/**
 * DOC: IOCTL arguments
 */

/**
 * struct drm_thames_bo_create - Arguments passed to DRM_IOCTL_THAMES_BO_CREATE.
 */
struct drm_thames_bo_create {
	/**
	 * @size: Requested size for the object
	 *
	 * The (page-aligned) allocated size for the object will be returned.
	 */
	__u64 size;

	/**
	 * @iova: Returned IOVA for the object, in the DSPs' address space.
	 */
	__u64 iova;

	/**
	 * @handle: Returned handle for the object.
	 *
	 * Object handles are nonzero.
	 */
	__u32 handle;

	/** @pad: MBZ. */
	__u32 pad;
};

/**
 * struct drm_thames_bo_mmap_offset - Arguments passed to DRM_IOCTL_THAMES_BO_MMAP_OFFSET.
 */
struct drm_thames_bo_mmap_offset {
	/** @handle: Handle of the object we want an mmap offset for. */
	__u32 handle;

	/** @pad: MBZ. */
	__u32 pad;

	/** @offset: The fake offset to use for subsequent mmap calls. */
	__u64 offset;
};

/**
 * DRM_IOCTL_THAMES() - Build a thames IOCTL number
 * @__access: Access type. Must be R, W or RW.
 * @__id: One of the DRM_THAMES_xxx id.
 * @__type: Suffix of the type being passed to the IOCTL.
 *
 * Don't use this macro directly, use the DRM_IOCTL_THAMES_xxx
 * values instead.
 *
 * Return: An IOCTL number to be passed to ioctl() from userspace.
 */
#define DRM_IOCTL_THAMES(__access, __id, __type) \
	DRM_IO ## __access(DRM_COMMAND_BASE + DRM_THAMES_ ## __id, \
			   struct drm_thames_ ## __type)

enum {
	DRM_IOCTL_THAMES_BO_CREATE =
		DRM_IOCTL_THAMES(WR, BO_CREATE, bo_create),
	DRM_IOCTL_THAMES_BO_MMAP_OFFSET =
		DRM_IOCTL_THAMES(WR, BO_MMAP_OFFSET, bo_mmap_offset),
};

#if defined(__cplusplus)
}
#endif

#endif /* _THAMES_DRM_H_ */
