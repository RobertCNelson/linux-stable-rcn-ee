/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "etnaviv_drv.h"
#include "etnaviv_gem.h"

struct get_pages_work {
	struct work_struct work;
	struct mm_struct *mm;
	struct task_struct *task;
	struct etnaviv_gem_object *etnaviv_obj;
};

static struct page **etnaviv_gem_userptr_do_get_pages(
	struct etnaviv_gem_object *etnaviv_obj, struct mm_struct *mm, struct task_struct *task)
{
	int ret = 0, pinned, npages = etnaviv_obj->base.size >> PAGE_SHIFT;
	struct page **pvec;
	uintptr_t ptr;

	pvec = drm_malloc_ab(npages, sizeof(struct page *));
	if (!pvec)
		return ERR_PTR(-ENOMEM);

	pinned = 0;
	ptr = etnaviv_obj->userptr.ptr;

	down_read(&mm->mmap_sem);
	while (pinned < npages) {
		ret = get_user_pages(task, mm, ptr, npages - pinned,
				     !etnaviv_obj->userptr.ro, 0,
				     pvec + pinned, NULL);
		if (ret < 0)
			break;

		ptr += ret * PAGE_SIZE;
		pinned += ret;
	}
	up_read(&mm->mmap_sem);

	if (ret < 0) {
		release_pages(pvec, pinned, 0);
		drm_free_large(pvec);
		return ERR_PTR(ret);
	}

	return pvec;
}

static void __etnaviv_gem_userptr_get_pages(struct work_struct *_work)
{
	struct get_pages_work *work = container_of(_work, typeof(*work), work);
	struct etnaviv_gem_object *etnaviv_obj = work->etnaviv_obj;
	struct drm_device *dev = etnaviv_obj->base.dev;
	struct page **pvec;

	pvec = etnaviv_gem_userptr_do_get_pages(etnaviv_obj, work->mm, work->task);

	mutex_lock(&dev->struct_mutex);
	if (IS_ERR(pvec)) {
		etnaviv_obj->userptr.work = ERR_CAST(pvec);
	} else {
		etnaviv_obj->userptr.work = NULL;
		etnaviv_obj->pages = pvec;
	}

	drm_gem_object_unreference(&etnaviv_obj->base);
	mutex_unlock(&dev->struct_mutex);

	mmput(work->mm);
	put_task_struct(work->task);
	kfree(work);
}

static int etnaviv_gem_userptr_get_pages(struct etnaviv_gem_object *etnaviv_obj)
{
	struct etnaviv_drm_private *priv;
	struct page **pvec = NULL;
	struct get_pages_work *work;
	struct mm_struct *mm;
	int ret, pinned, npages = etnaviv_obj->base.size >> PAGE_SHIFT;

	if (etnaviv_obj->userptr.work) {
		if (IS_ERR(etnaviv_obj->userptr.work)) {
			ret = PTR_ERR(etnaviv_obj->userptr.work);
			etnaviv_obj->userptr.work = NULL;
		} else {
			ret = -EAGAIN;
		}
		return ret;
	}

	mm = get_task_mm(etnaviv_obj->userptr.task);
	pinned = 0;
	if (mm == current->mm) {
		pvec = drm_malloc_ab(npages, sizeof(struct page *));
		if (!pvec) {
			mmput(mm);
			return -ENOMEM;
		}

		pinned = __get_user_pages_fast(etnaviv_obj->userptr.ptr, npages,
					       !etnaviv_obj->userptr.ro, pvec);
		if (pinned < 0) {
			drm_free_large(pvec);
			mmput(mm);
			return pinned;
		}

		if (pinned == npages) {
			etnaviv_obj->pages = pvec;
			mmput(mm);
			return 0;
		}
	}

	release_pages(pvec, pinned, 0);
	drm_free_large(pvec);

	work = kmalloc(sizeof(*work), GFP_KERNEL);
	if (!work) {
		mmput(mm);
		return -ENOMEM;
	}

	get_task_struct(current);
	drm_gem_object_reference(&etnaviv_obj->base);

	work->mm = mm;
	work->task = current;
	work->etnaviv_obj = etnaviv_obj;

	etnaviv_obj->userptr.work = &work->work;
	INIT_WORK(&work->work, __etnaviv_gem_userptr_get_pages);

	priv = etnaviv_obj->base.dev->dev_private;
	queue_work(priv->wq, &work->work);

	return -EAGAIN;
}

static void etnaviv_gem_userptr_release(struct etnaviv_gem_object *etnaviv_obj)
{
	if (etnaviv_obj->sgt) {
		/*
		 * For non-cached buffers, ensure the new pages are clean
		 * because display controller, GPU, etc. are not coherent:
		 */
		etnaviv_gem_scatterlist_unmap(etnaviv_obj);
		sg_free_table(etnaviv_obj->sgt);
		kfree(etnaviv_obj->sgt);
	}
	if (etnaviv_obj->pages) {
		int npages = etnaviv_obj->base.size >> PAGE_SHIFT;

		release_pages(etnaviv_obj->pages, npages, 0);
		drm_free_large(etnaviv_obj->pages);
	}
	put_task_struct(etnaviv_obj->userptr.task);
}

static const struct etnaviv_gem_ops etnaviv_gem_userptr_ops = {
	.get_pages = etnaviv_gem_userptr_get_pages,
	.release = etnaviv_gem_userptr_release,
};

int etnaviv_gem_new_userptr(struct drm_device *dev, struct drm_file *file,
	uintptr_t ptr, uint32_t size, uint32_t flags, uint32_t *handle)
{
	struct etnaviv_gem_object *etnaviv_obj;
	int ret;

	ret = mutex_lock_interruptible(&dev->struct_mutex);
	if (ret)
		return ret;

	ret = etnaviv_gem_new_private(dev, size, ETNA_BO_CACHED, &etnaviv_obj);
	if (ret == 0) {
		etnaviv_obj->ops = &etnaviv_gem_userptr_ops;
		etnaviv_obj->userptr.ptr = ptr;
		etnaviv_obj->userptr.task = current;
		etnaviv_obj->userptr.ro = !(flags & ETNA_USERPTR_WRITE);
		get_task_struct(current);
	}
	mutex_unlock(&dev->struct_mutex);

	if (ret)
		return ret;

	ret = drm_gem_handle_create(file, &etnaviv_obj->base, handle);

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(&etnaviv_obj->base);

	return ret;
}
