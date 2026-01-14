// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2019 Collabora ltd. */
/* Copyright 2024-2025 Tomeu Vizoso <tomeu@tomeuvizoso.net> */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#include "linux/dev_printk.h"
#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_print.h>
#include <drm/thames_accel.h>
#include <linux/platform_device.h>

#include "thames_core.h"
#include "thames_device.h"
#include "thames_drv.h"
#include "thames_gem.h"
#include "thames_job.h"
#include "thames_rpmsg.h"

#define JOB_TIMEOUT_MS 500

static struct thames_job *to_thames_job(struct drm_sched_job *sched_job)
{
	return container_of(sched_job, struct thames_job, base);
}

static const char *thames_fence_get_driver_name(struct dma_fence *fence)
{
	return "thames";
}

static const char *thames_fence_get_timeline_name(struct dma_fence *fence)
{
	return "thames";
}

static const struct dma_fence_ops thames_fence_ops = {
	.get_driver_name = thames_fence_get_driver_name,
	.get_timeline_name = thames_fence_get_timeline_name,
};

static struct dma_fence *thames_fence_create(struct thames_core *core)
{
	struct dma_fence *fence;

	fence = kzalloc(sizeof(*fence), GFP_KERNEL);
	if (!fence)
		return ERR_PTR(-ENOMEM);

	dma_fence_init(fence, &thames_fence_ops, &core->fence_lock, core->fence_context,
		       ++core->emit_seqno);

	return fence;
}

static void thames_job_hw_submit(struct thames_core *core, struct thames_job *job)
{
	int ret;

	/* Don't queue the job if a reset is in progress */
	if (atomic_read(&core->reset.pending))
		return;

	ret = thames_rpmsg_send_submit_job(core, job->file_priv->context_id, job->job_id,
					   to_thames_bo(job->kernel)->iova, job->kernel_size,
					   to_thames_bo(job->params)->iova, job->params_size,
					   &job->ipc_sequence);

	if (ret) {
		dev_err(core->dev, "Failed to submit kernel to DSP core %d\n", core->index);
		return;
	}
}

static int thames_acquire_object_fences(struct drm_gem_object **bos, int bo_count,
					struct drm_sched_job *job, bool is_write)
{
	int i, ret;

	for (i = 0; i < bo_count; i++) {
		ret = dma_resv_reserve_fences(bos[i]->resv, 1);
		if (ret)
			return ret;

		ret = drm_sched_job_add_implicit_dependencies(job, bos[i], is_write);
		if (ret)
			return ret;
	}

	return 0;
}

static void thames_attach_object_fences(struct drm_gem_object **bos, int bo_count,
					struct dma_fence *fence)
{
	int i;

	for (i = 0; i < bo_count; i++)
		dma_resv_add_fence(bos[i]->resv, fence, DMA_RESV_USAGE_WRITE);
}

static int thames_job_push(struct thames_job *job)
{
	struct thames_device *tdev = job->tdev;
	struct drm_gem_object **bos;
	struct ww_acquire_ctx acquire_ctx;
	int ret = 0;

	dev_dbg(tdev->ddev.dev, "Pushing job with %u in BOs and %u out BOs\n", job->in_bo_count,
		job->out_bo_count);
	bos = kvmalloc_array(job->in_bo_count + job->out_bo_count, sizeof(void *), GFP_KERNEL);
	memcpy(bos, job->in_bos, job->in_bo_count * sizeof(void *));
	memcpy(&bos[job->in_bo_count], job->out_bos, job->out_bo_count * sizeof(void *));

	ret = drm_gem_lock_reservations(bos, job->in_bo_count + job->out_bo_count, &acquire_ctx);
	if (ret)
		goto err;

	scoped_guard(mutex, &tdev->sched_lock)
	{
		drm_sched_job_arm(&job->base);

		job->inference_done_fence = dma_fence_get(&job->base.s_fence->finished);

		ret = thames_acquire_object_fences(job->in_bos, job->in_bo_count, &job->base,
						   false);
		if (ret)
			goto err_unlock;

		ret = thames_acquire_object_fences(job->out_bos, job->out_bo_count, &job->base,
						   true);
		if (ret)
			goto err_unlock;

		kref_get(&job->refcount); /* put by scheduler job completion */

		drm_sched_entity_push_job(&job->base);
	}

	thames_attach_object_fences(job->out_bos, job->out_bo_count, job->inference_done_fence);

err_unlock:
	drm_gem_unlock_reservations(bos, job->in_bo_count + job->out_bo_count, &acquire_ctx);
err:
	kvfree(bos);

	return ret;
}

static void thames_job_cleanup(struct kref *ref)
{
	struct thames_job *job = container_of(ref, struct thames_job, refcount);
	struct thames_device *tdev = job->tdev;
	unsigned int i;

	dma_fence_put(job->done_fence);
	dma_fence_put(job->inference_done_fence);

	ida_free(&tdev->job_ida, job->job_id);

	if (job->kernel)
		drm_gem_object_put(job->kernel);

	if (job->params)
		drm_gem_object_put(job->params);

	if (job->in_bos) {
		for (i = 0; i < job->in_bo_count; i++)
			drm_gem_object_put(job->in_bos[i]);

		kvfree(job->in_bos);
	}

	if (job->out_bos) {
		for (i = 0; i < job->out_bo_count; i++)
			drm_gem_object_put(job->out_bos[i]);

		kvfree(job->out_bos);
	}

	kfree(job);
}

static void thames_job_put(struct thames_job *job)
{
	kref_put(&job->refcount, thames_job_cleanup);
}

static void thames_job_free(struct drm_sched_job *sched_job)
{
	struct thames_job *job = to_thames_job(sched_job);

	drm_sched_job_cleanup(sched_job);

	thames_job_put(job);
}

static struct thames_core *sched_to_core(struct thames_device *tdev,
					 struct drm_gpu_scheduler *sched)
{
	unsigned int core;

	for (core = 0; core < tdev->num_cores; core++) {
		if (&tdev->cores[core].sched == sched)
			return &tdev->cores[core];
	}

	return NULL;
}

static struct dma_fence *thames_job_run(struct drm_sched_job *sched_job)
{
	struct thames_job *job = to_thames_job(sched_job);
	struct thames_device *tdev = job->tdev;
	struct thames_core *core = sched_to_core(tdev, sched_job->sched);
	struct dma_fence *fence = NULL;

	if (unlikely(job->base.s_fence->finished.error))
		return NULL;

	fence = thames_fence_create(core);
	if (IS_ERR(fence))
		return fence;

	if (job->done_fence)
		dma_fence_put(job->done_fence);
	job->done_fence = dma_fence_get(fence);

	scoped_guard(mutex, &core->job_lock)
	{
		core->in_flight_job = job;
		thames_job_hw_submit(core, job);
	}

	return fence;
}

static void thames_reset(struct thames_core *core, struct drm_sched_job *bad)
{
	if (!atomic_read(&core->reset.pending))
		return;

	drm_sched_stop(&core->sched, bad);
	scoped_guard(mutex, &core->job_lock) core->in_flight_job = NULL;
	thames_core_reset(core);
	atomic_set(&core->reset.pending, 0);
	drm_sched_start(&core->sched, 0);
}

static enum drm_gpu_sched_stat thames_job_timedout(struct drm_sched_job *sched_job)
{
	struct thames_job *job = to_thames_job(sched_job);
	struct thames_device *tdev = job->tdev;
	struct thames_core *core = sched_to_core(tdev, sched_job->sched);

	if (!core) {
		dev_err(tdev->ddev.dev, "Failed to find core for timed out job\n");
		return DRM_GPU_SCHED_STAT_NONE;
	}

	dev_err(core->dev, "Job %u timed out on DSP core %d\n", job->job_id, core->index);

	atomic_set(&core->reset.pending, 1);
	thames_reset(core, sched_job);

	return DRM_GPU_SCHED_STAT_RESET;
}

static void thames_reset_work(struct work_struct *work)
{
	struct thames_core *core;

	core = container_of(work, struct thames_core, reset.work);
	thames_reset(core, NULL);
}

static const struct drm_sched_backend_ops thames_sched_ops = { .run_job = thames_job_run,
							       .timedout_job = thames_job_timedout,
							       .free_job = thames_job_free };

int thames_job_init(struct thames_core *core)
{
	struct drm_sched_init_args args = {
		.ops = &thames_sched_ops,
		.num_rqs = DRM_SCHED_PRIORITY_COUNT,
		.credit_limit = 1,
		.timeout = msecs_to_jiffies(JOB_TIMEOUT_MS),
		.name = dev_name(core->dev),
		.dev = core->dev,
	};
	int ret;

	INIT_WORK(&core->reset.work, thames_reset_work);
	spin_lock_init(&core->fence_lock);
	mutex_init(&core->job_lock);

	core->reset.wq = alloc_ordered_workqueue("thames-reset-%d", 0, core->index);
	if (!core->reset.wq)
		return -ENOMEM;

	core->fence_context = dma_fence_context_alloc(1);

	args.timeout_wq = core->reset.wq;
	ret = drm_sched_init(&core->sched, &args);
	if (ret) {
		dev_err(core->dev, "Failed to create scheduler: %d.", ret);
		destroy_workqueue(core->reset.wq);
		return ret;
	}

	return 0;
}

void thames_job_fini(struct thames_core *core)
{
	drm_sched_fini(&core->sched);

	cancel_work_sync(&core->reset.work);
	destroy_workqueue(core->reset.wq);
}

int thames_job_open(struct thames_file_priv *thames_priv)
{
	struct thames_device *tdev = thames_priv->tdev;
	struct drm_gpu_scheduler **scheds =
		kmalloc_array(tdev->num_cores, sizeof(*scheds), GFP_KERNEL);
	unsigned int core;
	int ret;

	for (core = 0; core < tdev->num_cores; core++)
		scheds[core] = &tdev->cores[core].sched;

	ret = drm_sched_entity_init(&thames_priv->sched_entity, DRM_SCHED_PRIORITY_NORMAL, scheds,
				    tdev->num_cores, NULL);
	if (WARN_ON(ret))
		return ret;

	return 0;
}

void thames_job_close(struct thames_file_priv *thames_priv)
{
	struct drm_sched_entity *entity = &thames_priv->sched_entity;

	kfree(entity->sched_list);
	drm_sched_entity_destroy(entity);
}

static int thames_ioctl_submit_job(struct drm_device *dev, struct drm_file *file,
				   struct drm_thames_job *job)
{
	struct thames_device *tdev = to_thames_device(dev);
	struct thames_file_priv *file_priv = file->driver_priv;
	struct thames_job *tjob = NULL;
	int ret = 0;

	tjob = kzalloc(sizeof(*tjob), GFP_KERNEL);
	if (!tjob)
		return -ENOMEM;

	kref_init(&tjob->refcount);

	tjob->tdev = tdev;
	tjob->file_priv = file_priv;

	tjob->job_id = ida_alloc_min(&tdev->job_ida, 1, GFP_KERNEL);
	if (tjob->job_id < 0)
		goto out_put_job;

	ret = drm_sched_job_init(&tjob->base, &file_priv->sched_entity, 1, NULL, file->client_id);
	if (ret)
		goto out_put_job;

	tjob->kernel = drm_gem_object_lookup(file, job->kernel);
	if (!tjob->kernel) {
		ret = -ENOENT;
		goto out_cleanup_job;
	}

	tjob->kernel_size = job->kernel_size;

	if (job->params) {
		tjob->params = drm_gem_object_lookup(file, job->params);
		if (!tjob->params) {
			ret = -ENOENT;
			goto out_cleanup_job;
		}
		tjob->params_size = job->params_size;
	}

	ret = drm_gem_objects_lookup(file, u64_to_user_ptr(job->in_bo_handles),
				     job->in_bo_handle_count, &tjob->in_bos);
	if (ret)
		goto out_cleanup_job;

	tjob->in_bo_count = job->in_bo_handle_count;

	ret = drm_gem_objects_lookup(file, u64_to_user_ptr(job->out_bo_handles),
				     job->out_bo_handle_count, &tjob->out_bos);
	if (ret)
		goto out_cleanup_job;

	tjob->out_bo_count = job->out_bo_handle_count;

	ret = thames_job_push(tjob);

out_cleanup_job:
	if (ret)
		drm_sched_job_cleanup(&tjob->base);
out_put_job:
	thames_job_put(tjob);

	return ret;
}

#define THAMES_MAX_JOBS_PER_SUBMIT 256

int thames_ioctl_submit(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_thames_submit *args = data;
	struct drm_thames_job *jobs;
	size_t jobs_size;
	int ret = 0;
	unsigned int i = 0;

	if (args->pad)
		return -EINVAL;

	if (args->job_count == 0)
		return -EINVAL;

	if (args->job_count > THAMES_MAX_JOBS_PER_SUBMIT) {
		dev_err(dev->dev, "Job count %u exceeds maximum %u\n", args->job_count,
			THAMES_MAX_JOBS_PER_SUBMIT);
		return -EINVAL;
	}

	jobs_size = array_size(args->job_count, sizeof(*jobs));
	if (jobs_size == SIZE_MAX)
		return -EINVAL;

	jobs = kvmalloc_array(args->job_count, sizeof(*jobs), GFP_KERNEL);
	if (!jobs)
		return -ENOMEM;

	if (copy_from_user(jobs, u64_to_user_ptr(args->jobs), jobs_size)) {
		ret = -EFAULT;
		drm_dbg(dev, "Failed to copy incoming job array\n");
		goto exit;
	}

	for (i = 0; i < args->job_count; i++) {
		ret = thames_ioctl_submit_job(dev, file, &jobs[i]);
		if (ret)
			break;
	}

exit:
	kvfree(jobs);

	return ret;
}
