// SPDX-License-Identifier: GPL-2.0
/*
 * Mi-V Inter-hart communication (IHC) message
 *
 *Copyright (c) 2021 Microchip Technology Inc. All rights reserved.
 */

#ifndef _LINUX_MIV_IHC_H_
#define _LINUX_MIV_IHC_H_

#include <linux/types.h>

/* Data structure for data-transfer protocol */
#define IHC_MAX_MESSAGE_SIZE 4U

struct miv_ihc_msg {
	u32 msg[IHC_MAX_MESSAGE_SIZE];
};

#endif /* _LINUX_MIV_IHC_H_ */