/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ARM64_ASM_SIGNAL_H
#define __ARM64_ASM_SIGNAL_H

#include <uapi/asm/signal.h>
#include <uapi/asm/siginfo.h>

#if defined(CONFIG_PREEMPT_RT)
#define ARCH_RT_DELAYS_SIGNAL_SEND
#endif

#endif
