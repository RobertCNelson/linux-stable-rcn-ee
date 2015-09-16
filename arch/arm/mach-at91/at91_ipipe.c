/*
 * linux/arch/arm/mach-at91/at91_ipipe.c
 *
 * Copyright (C) 2007 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
 *
 * Adaptation to AT91SAM926x:
 * Copyright (C) 2007 Gregory CLEMENT, Adeneo
 *
 * Adaptation to AT91SAM9G45:
 * Copyright (C) 2011 Gregory CLEMENT, Free Electrons
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/stringify.h>
#include <linux/err.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/ipipe.h>
#include <linux/export.h>
#include <linux/ipipe_tickdev.h>

#include <asm/io.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include "at91_tc.h"
#include "at91_ipipe.h"
#include "clock.h"

#if defined(CONFIG_ARCH_AT91RM9200)
#define AT91_ID_TC0 AT91RM9200_ID_TC0
#define AT91_ID_TC1 AT91RM9200_ID_TC1
#define AT91_ID_TC2 AT91RM9200_ID_TC2
#elif defined(CONFIG_ARCH_AT91SAM9260) || defined(CONFIG_ARCH_AT91SAM9G20)
#define AT91_ID_TC0 AT91SAM9260_ID_TC0
#define AT91_ID_TC1 AT91SAM9260_ID_TC1
#define AT91_ID_TC2 AT91SAM9260_ID_TC2
#elif defined(CONFIG_ARCH_AT91SAM9261)
#define AT91_ID_TC0 AT91SAM9261_ID_TC0
#define AT91_ID_TC1 AT91SAM9261_ID_TC1
#define AT91_ID_TC2 AT91SAM9261_ID_TC2
#elif defined(CONFIG_ARCH_AT91SAM9263)
#define AT91_ID_TC0 AT91SAM9263_ID_TCB
#define AT91_ID_TC1 AT91SAM9263_ID_TCB
#define AT91_ID_TC2 AT91SAM9263_ID_TCB
#elif defined(CONFIG_ARCH_AT91SAM9RL)
#define AT91_ID_TC0 AT91SAM9RL_ID_TC0
#define AT91_ID_TC1 AT91SAM9RL_ID_TC1
#define AT91_ID_TC2 AT91SAM9RL_ID_TC2
#elif defined(CONFIG_ARCH_AT91X40)
#define AT91_ID_TC0 AT91X40_ID_TC0
#define AT91_ID_TC1 AT91X40_ID_TC1
#define AT91_ID_TC2 AT91X40_ID_TC2
#elif defined(CONFIG_ARCH_AT91SAM9G45)
#define AT91_ID_TC0 AT91SAM9G45_ID_TCB
#define AT91_ID_TC1 AT91SAM9G45_ID_TCB
#define AT91_ID_TC2 AT91SAM9G45_ID_TCB
#else
#error "AT91 processor unsupported by Adeos"
#endif

#if (CONFIG_IPIPE_AT91_TC==0)
#   define KERNEL_TIMER_IRQ_NUM AT91_ID_TC0
#elif (CONFIG_IPIPE_AT91_TC==1)
#   define KERNEL_TIMER_IRQ_NUM AT91_ID_TC1
#elif (CONFIG_IPIPE_AT91_TC==2)
#   define KERNEL_TIMER_IRQ_NUM AT91_ID_TC2
#else
#error IPIPE_AT91_TC must be 0, 1 or 2.
#endif

#define TCNXCNS(timer,v) ((v) << ((timer)<<1))
#define AT91_TC_REG_MASK (0xffff)
#define AT91_TC_BASE ((unsigned long)AT91_VA_BASE_TCB0)

static unsigned max_delta_ticks;

static inline unsigned int at91_tc_read(unsigned int reg_offset)
{
	unsigned long addr = (AT91_TC_BASE + 0x40 * CONFIG_IPIPE_AT91_TC);

	return __raw_readl((void __iomem *)(addr + reg_offset));
}

static inline void at91_tc_write(unsigned int reg_offset, unsigned long value)
{
	unsigned long addr = (AT91_TC_BASE + 0x40 * CONFIG_IPIPE_AT91_TC);

	__raw_writel(value, (void __iomem *)(addr + reg_offset));
}

#define read_CV() at91_tc_read(AT91_TC_CV)
#define read_RC() at91_tc_read(AT91_TC_RC)
#define write_RC(value) at91_tc_write(AT91_TC_RC, value)

/*
 * Reprogram the timer
 */
static int at91_tc_set(unsigned long evt, void *timer);

/*
 * IRQ handler for the timer.
 */
static void at91_tc_ack(void)
{
	at91_tc_read(AT91_TC_SR);
}

static void at91_tc_request(struct ipipe_timer *timer, int steal)
{
	/* Enable CPCS interrupt. */
	at91_tc_write(AT91_TC_IER, AT91_TC_CPCS);
}

static void at91_tc_release(struct ipipe_timer *timer)
{
	/* Disable all interrupts. */
	at91_tc_write(AT91_TC_IDR, ~0ul);
}

static struct ipipe_timer at91_itimer = {
	.irq            = NR_IRQS_LEGACY + KERNEL_TIMER_IRQ_NUM,
	.request        = at91_tc_request,
	.set            = at91_tc_set,
	.ack            = at91_tc_ack,
	.release        = at91_tc_release,

	.name		= "at91_tc" __stringify(CONFIG_IPIPE_AT91_TC),
	.rating		= 250,
};

static int at91_tc_set(unsigned long evt, void *timer)
{
	unsigned short next_tick;

	if (evt > max_delta_ticks)
		evt = max_delta_ticks;

	__ipipe_tsc_update();

	next_tick = read_CV() + evt;
	write_RC(next_tick);
	if (evt >= AT91_TC_REG_MASK / 2
	    || (short)(next_tick - read_CV()) > 0)
		return 0;

	at91_itimer.min_delay_ticks = evt;
	return  -ETIME;
}

static struct __ipipe_tscinfo tsc_info = {
	.type = IPIPE_TSC_TYPE_FREERUNNING,
	.counter_vaddr = (AT91_TC_BASE +
			  0x40 * CONFIG_IPIPE_AT91_TC + AT91_TC_CV),
	.u = {
		{
			.counter_paddr = (AT91_BASE_TCB0 +
					  0x40 * CONFIG_IPIPE_AT91_TC +
					  AT91_TC_CV),
			.mask = AT91_TC_REG_MASK,
		},
	},
};

void at91_ipipe_init(struct clock_event_device *host_timer)
{
	unsigned char tc_divisors[] = { 2, 8, 32, 128, 0, };
	unsigned master_freq, divisor = 0, divided_freq = 0;
	unsigned long long wrap_ns;
	int tc_timer_clock;
	unsigned short v;
	struct clk *clk;

#ifdef CONFIG_ARCH_AT91SAM9263
	clk = clk_get(NULL, "tcb_clk");
#elif defined(CONFIG_ARCH_AT91SAM9G45)
	clk = clk_get(NULL, "tcb0_clk");
#else /* not AT91SAM9263 or AT91SAM9G45*/
	clk = clk_get(NULL, "tc"__stringify(CONFIG_IPIPE_AT91_TC) "_clk");
#endif
	clk_enable(clk);

	/* Disable the channel */
	at91_tc_write(AT91_TC_CCR, AT91_TC_CLKDIS);

	/* Disable all interrupts. */
	at91_tc_write(AT91_TC_IDR, ~0ul);

	master_freq = clk_get_rate(clk_get(NULL, "mck"));
	/* Find the first frequency above 1 MHz */
	for (tc_timer_clock = ARRAY_SIZE(tc_divisors) - 1;
	     tc_timer_clock >= 0; tc_timer_clock--) {
		divisor = tc_divisors[tc_timer_clock];
		divided_freq = (divisor
				? master_freq / divisor : AT91_SLOW_CLOCK);
		if (divided_freq > 1000000)
			break;
	}

	wrap_ns = (unsigned long long) (AT91_TC_REG_MASK + 1) * NSEC_PER_SEC;
	do_div(wrap_ns, divided_freq);

	if (divided_freq < 1000000)
		printk(KERN_INFO "AT91 I-pipe warning: could not find a"
		       " frequency greater than 1MHz\n");

	printk(KERN_INFO "AT91 I-pipe timer: div: %u, freq: %u.%06u MHz, wrap: "
	       "%u.%06u ms\n", divisor,
	       divided_freq / 1000000, divided_freq % 1000000,
	       (unsigned) wrap_ns / 1000000, (unsigned) wrap_ns % 1000000);

	/* Add a 1ms margin. It means that when an interrupt occurs, update_tsc
	   must be called within 1ms. update_tsc is called by acktimer when no
	   higher domain handles the timer, and called through set_dec when a
	   higher domain handles the timer. */
	wrap_ns -= 1000000;
	/* Set up the interrupt. */

	if (host_timer && host_timer->features & CLOCK_EVT_FEAT_ONESHOT
	    && host_timer->max_delta_ns > wrap_ns)
		host_timer->max_delta_ns = wrap_ns;

	/* No Sync. */
	at91_tc_write(AT91_TC_BCR, 0);

	/* program NO signal on XCN */
	v = __raw_readl((void __iomem *) (AT91_VA_BASE_TCB0 + AT91_TC_BMR));
	v &= ~TCNXCNS(CONFIG_IPIPE_AT91_TC, 3);
	v |= TCNXCNS(CONFIG_IPIPE_AT91_TC, 1); /* AT91_TC_TCNXCNS_NONE */
	__raw_writel(v, (void __iomem *) (AT91_VA_BASE_TCB0 + AT91_TC_BMR));

	/* Use the clock selected as input clock. */
	at91_tc_write(AT91_TC_CMR, tc_timer_clock);

	/* Load the TC register C. */
	write_RC(0xffff);

	/* Enable the channel. */
	at91_tc_write(AT91_TC_CCR, AT91_TC_CLKEN | AT91_TC_SWTRG);

	at91_itimer.freq = divided_freq;
	at91_itimer.min_delay_ticks = ipipe_timer_ns2ticks(&at91_itimer, 2000);
	max_delta_ticks = ipipe_timer_ns2ticks(&at91_itimer, wrap_ns);
	ipipe_timer_register(&at91_itimer);

	tsc_info.freq = divided_freq;
	__ipipe_tsc_register(&tsc_info);

#if 1
	at91_pic_muter_register();
#endif
}
