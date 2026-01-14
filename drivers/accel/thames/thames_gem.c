// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#include "drm/drm_gem_shmem_helper.h"
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_print.h>
#include <drm/drm_utils.h>
#include <drm/thames_accel.h>
#include <linux/dma-mapping.h>
#include <linux/idr.h>

#include "thames_gem.h"
#include "thames_device.h"
#include "thames_drv.h"
#include "thames_rpmsg.h"

/*
 * DSP MMU permission flags for buffer object mappings.
 * These control read/write/execute permissions in the DSP's address space.
 */
#define THAMES_BO_PERM_READ (1 << 0)
#define THAMES_BO_PERM_WRITE (1 << 1)
#define THAMES_BO_PERM_EXEC (1 << 2)
#define THAMES_BO_PERM_RWX (THAMES_BO_PERM_READ | THAMES_BO_PERM_WRITE | THAMES_BO_PERM_EXEC)

static u64 thames_alloc_vaddr(struct thames_device *tdev, struct thames_gem_object *bo, size_t size)
{
	int ret;

	size = ALIGN(size, SZ_1M);

	mutex_lock(&tdev->mm_lock);
	ret = drm_mm_insert_node(&tdev->mm, &bo->mm, size);
	mutex_unlock(&tdev->mm_lock);

	if (ret)
		return 0;

	return bo->mm.start;
}

static void thames_free_vaddr(struct thames_device *tdev, struct thames_gem_object *bo)
{
	if (!drm_mm_node_allocated(&bo->mm))
		return;

	mutex_lock(&tdev->mm_lock);
	drm_mm_remove_node(&bo->mm);
	mutex_unlock(&tdev->mm_lock);
}

static int thames_context_destroy_on_core(struct thames_file_priv *priv, struct thames_core *core)
{
	struct thames_device *tdev = priv->tdev;
	int ret;

	ret = thames_rpmsg_send_destroy_context(core, priv->context_id);
	if (ret)
		dev_warn(tdev->ddev.dev, "Failed to destroy context on core %d: %d", core->index,
			 ret);

	return ret;
}

static int thames_context_create_on_core(struct thames_file_priv *priv, struct thames_core *core)
{
	struct thames_device *tdev = priv->tdev;
	int ret;

	ret = thames_rpmsg_send_create_context(core, priv->context_id);
	if (ret)
		dev_warn(tdev->ddev.dev, "Failed to create context on core %d: %d", core->index,
			 ret);

	return ret;
}

int thames_context_create(struct thames_file_priv *priv)
{
	struct thames_device *tdev = priv->tdev;
	int i, ret;

	ret = ida_alloc_min(&tdev->ctx_ida, 1, GFP_KERNEL);
	if (ret < 0)
		return ret;

	priv->context_id = ret;
	priv->context_valid = false;

	if (!tdev->num_cores) {
		dev_err(tdev->ddev.dev, "No cores available\n");
		ret = -ENODEV;
		goto err_free_id;
	}

	for (i = 0; i < tdev->num_cores; i++) {
		ret = thames_context_create_on_core(priv, &tdev->cores[i]);
		if (ret) {
			dev_err(tdev->ddev.dev, "Failed to create context on core %d: %d\n", i,
				ret);
			goto err_destroy_contexts;
		}
	}

	priv->context_valid = true;
	return 0;

err_destroy_contexts:
	for (i = i - 1; i >= 0; i--)
		thames_context_destroy_on_core(priv, &tdev->cores[i]);
err_free_id:
	ida_free(&tdev->ctx_ida, priv->context_id);
	return ret;
}

void thames_context_destroy(struct thames_file_priv *priv)
{
	struct thames_device *tdev = priv->tdev;
	int i;

	if (!priv->context_valid)
		return;

	for (i = 0; i < tdev->num_cores; i++)
		thames_context_destroy_on_core(priv, &tdev->cores[i]);

	ida_free(&tdev->ctx_ida, priv->context_id);
	priv->context_valid = false;
}

static int thames_bo_map_to_core(struct thames_gem_object *bo, struct thames_file_priv *file_priv,
				 struct thames_core *core, u64 vaddr, u64 paddr, u64 size,
				 u32 flags)
{
	struct thames_device *tdev = file_priv->tdev;
	int ret;

	ret = thames_rpmsg_send_map_bo(core, file_priv->context_id, bo->id, vaddr, paddr, size);
	if (ret)
		dev_warn(tdev->ddev.dev, "Failed to map buffer on core %d: %d", core->index, ret);

	return ret;
}

static int thames_bo_map_to_device(struct thames_gem_object *bo, struct thames_file_priv *file_priv)
{
	struct thames_device *tdev = file_priv->tdev;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	int i, ret;

	if (bo->iova)
		return 0;

	if (!file_priv->context_valid)
		return -EINVAL;

	if (!tdev->num_cores)
		return -ENODEV;

	sgt = drm_gem_shmem_get_pages_sgt(&bo->base);
	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	dma_addr = sg_dma_address(sgt->sgl);
	if (!dma_addr) {
		ret = -EINVAL;
		goto err_put_pages;
	}

	bo->iova = thames_alloc_vaddr(tdev, bo, bo->base.base.size);
	if (!bo->iova) {
		ret = -ENOMEM;
		goto err_put_pages;
	}

	bo->context_id = file_priv->context_id;

	for (i = 0; i < tdev->num_cores; i++) {
		ret = thames_bo_map_to_core(bo, file_priv, &tdev->cores[i], bo->iova, dma_addr,
					    bo->base.base.size, THAMES_BO_PERM_RWX);
		if (ret) {
			while (--i >= 0)
				thames_rpmsg_send_unmap_bo(&tdev->cores[i], bo->context_id, bo->id);
			goto err_free_vaddr;
		}
	}

	return 0;

err_free_vaddr:
	thames_free_vaddr(tdev, bo);
	bo->iova = 0;
err_put_pages:
	dma_resv_lock(bo->base.base.resv, NULL);
	drm_gem_shmem_put_pages_locked(&bo->base);
	dma_resv_unlock(bo->base.base.resv);
	return ret;
}

static void thames_bo_unmap_from_device(struct thames_gem_object *bo, struct thames_device *tdev)
{
	int i, ret, failed_unmaps = 0;

	if (!bo->iova)
		return;

	for (i = 0; i < tdev->num_cores; i++) {
		ret = thames_rpmsg_send_unmap_bo(&tdev->cores[i], bo->context_id, bo->id);
		if (ret) {
			dev_err(tdev->ddev.dev, "Failed to unmap BO %u from core %d: %d\n", bo->id,
				i, ret);
			failed_unmaps++;
		}
	}

	if (failed_unmaps)
		drm_WARN(&tdev->ddev, failed_unmaps > 0,
			 "BO %u: %d core(s) failed unmap, potential DSP-side leak\n", bo->id,
			 failed_unmaps);

	thames_free_vaddr(tdev, bo);
	bo->iova = 0;

	dma_resv_lock(bo->base.base.resv, NULL);
	drm_gem_shmem_put_pages_locked(&bo->base);
	dma_resv_unlock(bo->base.base.resv);
}

static void thames_gem_bo_free(struct drm_gem_object *obj)
{
	struct thames_gem_object *bo = to_thames_bo(obj);
	struct thames_device *tdev = to_thames_device(obj->dev);

	drm_WARN_ON(obj->dev, refcount_read(&bo->base.pages_use_count) > 1);

	if (bo->iova)
		thames_bo_unmap_from_device(bo, tdev);

	ida_free(&tdev->bo_ida, bo->id);

	drm_gem_free_mmap_offset(&bo->base.base);
	drm_gem_shmem_free(&bo->base);
}

static const struct drm_gem_object_funcs thames_gem_funcs = {
	.free = thames_gem_bo_free,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

struct drm_gem_object *thames_gem_create_object(struct drm_device *dev, size_t size)
{
	struct thames_device *tdev = to_thames_device(dev);
	struct thames_gem_object *obj;
	int bo_id;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return ERR_PTR(-ENOMEM);

	obj->base.base.funcs = &thames_gem_funcs;

	bo_id = ida_alloc_min(&tdev->bo_ida, 1, GFP_KERNEL);
	if (bo_id < 0) {
		kfree(obj);
		return ERR_PTR(bo_id);
	}
	obj->id = bo_id;

	return &obj->base.base;
}

int thames_ioctl_bo_create(struct drm_device *ddev, void *data, struct drm_file *file)
{
	struct thames_file_priv *file_priv = file->driver_priv;
	struct drm_thames_bo_create *args = data;
	struct drm_gem_shmem_object *mem;
	struct thames_gem_object *bo;
	int cookie, ret;

	if (!drm_dev_enter(ddev, &cookie))
		return -ENODEV;

	if (args->handle || args->iova) {
		ret = -EINVAL;
		goto err_exit;
	}

	if (!args->size) {
		ret = -EINVAL;
		goto err_exit;
	}

	mem = drm_gem_shmem_create(ddev, args->size);
	if (IS_ERR(mem))
		return PTR_ERR(mem);

	bo = to_thames_bo(&mem->base);

	ret = drm_gem_handle_create(file, &mem->base, &args->handle);
	drm_gem_object_put(&mem->base);
	if (ret) {
		dev_err(ddev->dev, "Failed to create gem handle: %d", ret);
		goto err_free;
	}

	ret = thames_bo_map_to_device(bo, file_priv);
	if (ret) {
		dev_err(ddev->dev, "Failed to map buffer to DSP on creation: %d", ret);
		goto err_free;
	}

	args->size = bo->base.base.size;
	args->iova = bo->iova;

	drm_dev_exit(cookie);

	return 0;

err_free:
	drm_gem_shmem_object_free(&mem->base);

err_exit:
	drm_dev_exit(cookie);

	return ret;
}

int thames_ioctl_bo_mmap_offset(struct drm_device *ddev, void *data, struct drm_file *file)
{
	struct drm_thames_bo_mmap_offset *args = data;
	struct drm_gem_object *obj;

	if (args->pad)
		return -EINVAL;

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	args->offset = drm_vma_node_offset_addr(&obj->vma_node);
	drm_gem_object_put(obj);

	return 0;
}
