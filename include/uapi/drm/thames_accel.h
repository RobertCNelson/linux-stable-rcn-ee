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

	/** @DRM_THAMES_SUBMIT: Submit a job and BOs to run. */
	DRM_THAMES_SUBMIT,

	/** @DRM_THAMES_BO_PREP: Prepare a BO for CPU access after DSP writes. */
	DRM_THAMES_BO_PREP,

	/** @DRM_THAMES_BO_FINI: Finish CPU access and prepare BO for DSP access. */
	DRM_THAMES_BO_FINI,
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
 * struct drm_thames_job - A job to be run on the NPU
 *
 * The kernel will schedule the execution of this job taking into account its
 * dependencies with other jobs. All tasks in the same job will be executed
 * sequentially on the same core, to benefit from memory residency in SRAM.
 */
struct drm_thames_job {
	/** Input: BO handle for kernel. */
	__u32 kernel;

	/** Input: Size in bytes of the compiled kernel. */
	__u32 kernel_size;

	/** Input: BO handle for params BO. */
	__u32 params;

	/** Input: Size in bytes of the params BO. */
	__u32 params_size;

	/** Input: Pointer to a u32 array of the BOs that are read by the job. */
	__u64 in_bo_handles;

	/** Input: Pointer to a u32 array of the BOs that are written to by the job. */
	__u64 out_bo_handles;

	/** Input: Number of input BO handles passed in (size is that times 4). */
	__u32 in_bo_handle_count;

	/** Input: Number of output BO handles passed in (size is that times 4). */
	__u32 out_bo_handle_count;
};

/**
 * struct drm_thames_submit - ioctl argument for submitting commands to the NPU.
 *
 * The kernel will schedule the execution of these jobs in dependency order.
 */
struct drm_thames_submit {
	/** Input: Pointer to an array of struct drm_thames_job. */
	__u64 jobs;

	/** Input: Number of jobs passed in. */
	__u32 job_count;

	/** Reserved, must be zero. */
	__u32 pad;
};

/**
 * struct drm_thames_bo_prep - ioctl argument for preparing a BO for CPU access.
 *
 * This invalidates CPU caches and waits for pending DSP operations to complete.
 */
struct drm_thames_bo_prep {
	__u32 handle;
	__u32 reserved;
	__s64 timeout_ns;	/* absolute */
};

/**
 * struct drm_thames_bo_fini - ioctl argument for finishing CPU access to a BO.
 *
 * This flushes CPU caches to make CPU writes visible to the DSP.
 */
struct drm_thames_bo_fini {
	__u32 handle;
	__u32 reserved;
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
	DRM_IOCTL_THAMES_SUBMIT =
		DRM_IOCTL_THAMES(WR, SUBMIT, submit),
	DRM_IOCTL_THAMES_BO_PREP =
		DRM_IOCTL_THAMES(WR, BO_PREP, bo_prep),
	DRM_IOCTL_THAMES_BO_FINI =
		DRM_IOCTL_THAMES(WR, BO_FINI, bo_fini),
};

#if defined(__cplusplus)
}
#endif

#endif /* _THAMES_DRM_H_ */
