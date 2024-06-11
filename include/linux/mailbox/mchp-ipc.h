/* SPDX-License-Identifier: GPL-2.0 */
/*
 *Copyright (c) 2024 Microchip Technology Inc. All rights reserved.
 */

#ifndef _LINUX_MCHP_IPC_H_
#define _LINUX_MCHP_IPC_H_

#include <linux/mailbox_controller.h>
#include <linux/types.h>

struct mchp_ipc_msg {
	u32 *buf;
	u16 size;
};

#if IS_ENABLED(CONFIG_MCHP_SBI_IPC_MBOX)
u32 mchp_ipc_get_chan_id(struct mbox_chan *chan);
#else
u32 mchp_ipc_get_chan_id(struct mbox_chan *chan) { return 0; }
#endif

#endif /* _LINUX_MCHP_IPC_H_ */
