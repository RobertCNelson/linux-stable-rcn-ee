/*
 * Idle daemon for PowerPC.  Idle daemon will handle any action
 * that needs to be taken when the system becomes idle.
 *
 * Originally written by Cort Dougan (cort@cs.nmt.edu).
 * Subsequent 32-bit hacking by Tom Rini, Armin Kuster,
 * Paul Mackerras and others.
 *
 * iSeries supported added by Mike Corrigan <mikejc@us.ibm.com>
 *
 * Additional shared processor, SMT, and firmware support
 *    Copyright (c) 2003 Dave Engebretsen <engebret@us.ibm.com>
 *
 * 32-bit and 64-bit versions merged by Paul Mackerras <paulus@samba.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/sched.h>
#include <linux/ipipe.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/sysctl.h>
#include <linux/tick.h>

#include <asm/processor.h>
#include <asm/cputable.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/runlatch.h>
#include <asm/smp.h>


unsigned long cpuidle_disable = IDLE_NO_OVERRIDE;
EXPORT_SYMBOL(cpuidle_disable);

static int __init powersave_off(char *arg)
{
	ppc_md.power_save = NULL;
	cpuidle_disable = IDLE_POWERSAVE_OFF;
	return 0;
}
__setup("powersave=off", powersave_off);

#ifdef CONFIG_HOTPLUG_CPU
void arch_cpu_idle_dead(void)
{
	sched_preempt_enable_no_resched();
	cpu_die();
}
#endif

#ifdef CONFIG_IPIPE
static void __ipipe_halt_root(void)
{
	struct ipipe_percpu_domain_data *p;

	/*
	 * Emulate idle entry sequence over the root domain, which is
	 * stalled on entry.
	 */
	hard_local_irq_disable();

	p = ipipe_this_cpu_root_context();
	__clear_bit(IPIPE_STALL_FLAG, &p->status);

	if (unlikely(__ipipe_ipending_p(p)))
		__ipipe_sync_stage();
	else
		ppc_md.power_save();

	hard_local_irq_enable();
}
#else
static void __ipipe_halt_root(void)
{
	ppc_md.power_save();
	/*
	 * Some power_save functions return with
	 * interrupts enabled, some don't.
	 */
	if (irqs_disabled())
		local_irq_enable();
}
#endif

void arch_cpu_idle(void)
{
	ppc64_runlatch_off();

	if (ppc_md.power_save)
		__ipipe_halt_root();
	else {
		local_irq_enable();
		/*
		 * Go into low thread priority and possibly
		 * low power mode.
		 */
		HMT_low();
		HMT_very_low();
	}

	HMT_medium();
	ppc64_runlatch_on();
}

int powersave_nap;

#ifdef CONFIG_SYSCTL
/*
 * Register the sysctl to set/clear powersave_nap.
 */
static struct ctl_table powersave_nap_ctl_table[] = {
	{
		.procname	= "powersave-nap",
		.data		= &powersave_nap,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_dointvec,
	},
	{}
};
static struct ctl_table powersave_nap_sysctl_root[] = {
	{
		.procname	= "kernel",
		.mode		= 0555,
		.child		= powersave_nap_ctl_table,
	},
	{}
};

static int __init
register_powersave_nap_sysctl(void)
{
	register_sysctl_table(powersave_nap_sysctl_root);

	return 0;
}
__initcall(register_powersave_nap_sysctl);
#endif
