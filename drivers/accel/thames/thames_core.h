/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#ifndef __THAMES_CORE_H__
#define __THAMES_CORE_H__

#include <linux/rpmsg.h>
#include <drm/gpu_scheduler.h>
#include <linux/mutex_types.h>
#include <linux/completion.h>

struct thames_msg_buffer_op;

struct thames_core {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct thames_device *tdev;
	unsigned int index;

	/* RPMSG communication context */
	struct {
		struct rpmsg_endpoint *endpoint;

		struct {
			u32 sequence;
			u32 expected_data;
			bool success;
			struct completion completion;
		} ping_test;
	} rpmsg_ctx;

	struct mutex job_lock;
	struct thames_job *in_flight_job;

	spinlock_t fence_lock;

	struct {
		struct workqueue_struct *wq;
		struct work_struct work;
		atomic_t pending;
	} reset;

	struct drm_gpu_scheduler sched;
	u64 fence_context;
	u64 emit_seqno;
};

int thames_core_init(struct thames_core *core);
void thames_core_fini(struct thames_core *core);
void thames_core_reset(struct thames_core *core);
int thames_core_get_iova_range(struct rpmsg_device *rpdev, u64 *iova_start, u64 *iova_size);

#endif
