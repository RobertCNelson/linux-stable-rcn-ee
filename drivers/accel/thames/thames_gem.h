/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#ifndef __THAMES_GEM_H__
#define __THAMES_GEM_H__

#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_mm.h>

struct thames_device;

struct thames_gem_object {
	struct drm_gem_shmem_object base;

	struct thames_file_priv *driver_priv;

	struct drm_mm_node mm;

	u32 id;
	u32 context_id;
	u64 iova;
	size_t size;
	size_t offset;
};

struct drm_gem_object *thames_gem_create_object(struct drm_device *dev, size_t size);

int thames_ioctl_bo_create(struct drm_device *ddev, void *data, struct drm_file *file);

int thames_ioctl_bo_mmap_offset(struct drm_device *ddev, void *data, struct drm_file *file);

int thames_context_create(struct thames_file_priv *priv);

void thames_context_destroy(struct thames_file_priv *priv);

static inline struct thames_gem_object *to_thames_bo(struct drm_gem_object *obj)
{
	return container_of(to_drm_gem_shmem_obj(obj), struct thames_gem_object, base);
}

#endif
