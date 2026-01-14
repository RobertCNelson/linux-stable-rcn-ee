/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright 2026 Texas Instruments Incorporated - https://www.ti.com/ */

#ifndef __THAMES_RPMSG_H__
#define __THAMES_RPMSG_H__

#include <linux/completion.h>
#include <linux/rpmsg.h>

struct thames_core;

int thames_rpmsg_init(struct thames_core *core);
void thames_rpmsg_fini(struct thames_core *core);

int thames_rpmsg_send_ping(struct thames_core *core, u32 ping_data, u32 *sequence);
int thames_rpmsg_send_create_context(struct thames_core *core, u32 context_id);
int thames_rpmsg_send_destroy_context(struct thames_core *core, u32 context_id);
int thames_rpmsg_send_map_bo(struct thames_core *core, u32 context_id, u32 bo_id, u64 vaddr,
			     u64 paddr, u64 size);
int thames_rpmsg_send_unmap_bo(struct thames_core *core, u32 context_id, u32 bo_id);
int thames_rpmsg_send_submit_job(struct thames_core *core, u32 context_id, u32 job_id,
				 u64 kernel_iova, u64 kernel_size, u64 args_iova, u64 args_size,
				 u32 *sequence);

int thames_rpmsg_ping_test(struct thames_core *core);

#endif /* __THAMES_RPMSG_H__ */
