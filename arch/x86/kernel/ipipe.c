/*   -*- linux-c -*-
 *   linux/arch/x86/kernel/ipipe.c
 *
 *   Copyright (C) 2002-2012 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Architecture-dependent I-PIPE support for x86.
 */

#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/kprobes.h>
#include <linux/mm.h>
#include <linux/kgdb.h>
#include <linux/ipipe_tickdev.h>
#include <asm/asm-offsets.h>
#include <asm/unistd.h>
#include <asm/processor.h>
#include <asm/atomic.h>
#include <asm/hw_irq.h>
#include <asm/irq.h>
#include <asm/desc.h>
#include <asm/io.h>
#ifdef CONFIG_X86_LOCAL_APIC
#include <asm/tlbflush.h>
#include <asm/fixmap.h>
#include <asm/bitops.h>
#include <asm/mpspec.h>
#ifdef CONFIG_X86_IO_APIC
#include <asm/io_apic.h>
#endif	/* CONFIG_X86_IO_APIC */
#include <asm/apic.h>
#endif	/* CONFIG_X86_LOCAL_APIC */
#include <asm/traps.h>
#include <asm/tsc.h>
#include <asm/i387.h>
#include <asm/fpu-internal.h>
#include <asm/mce.h>

DEFINE_PER_CPU(unsigned long, __ipipe_cr2);
EXPORT_PER_CPU_SYMBOL_GPL(__ipipe_cr2);

void ipipe_raise_irq(unsigned int irq)
{
	struct pt_regs regs;
	unsigned long flags;

	flags = hard_local_irq_save();
	regs.flags = flags;
	regs.orig_ax = irq;  /* >= 0, IRQ won't be acked */
	regs.cs = __KERNEL_CS;
	__ipipe_handle_irq(&regs);
	hard_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_raise_irq);

int ipipe_get_sysinfo(struct ipipe_sysinfo *info)
{
	info->sys_nr_cpus = num_online_cpus();
	info->sys_cpu_freq = __ipipe_cpu_freq;
	info->sys_hrtimer_irq = per_cpu(ipipe_percpu.hrtimer_irq, 0);
	info->sys_hrtimer_freq = __ipipe_hrtimer_freq;
	info->sys_hrclock_freq = __ipipe_hrclock_freq;

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_get_sysinfo);

#ifdef CONFIG_X86_UV
asmlinkage void uv_bau_message_interrupt(struct pt_regs *regs);
#endif
#ifdef CONFIG_X86_MCE_THRESHOLD
asmlinkage void smp_threshold_interrupt(void);
#endif

void __ipipe_do_IRQ(unsigned int irq, void *cookie)
{
	void (*handler)(struct pt_regs *regs);
	struct pt_regs *regs;

	regs = __this_cpu_ptr(&ipipe_percpu.tick_regs);
	regs->orig_ax = ~__ipipe_get_irq_vector(irq);
	handler = (typeof(handler))cookie;
	__root_irq_trampoline(handler, regs);
}

#ifdef CONFIG_X86_LOCAL_APIC

static void __ipipe_noack_apic(unsigned irq, struct irq_desc *desc)
{
}

static void __ipipe_ack_apic(unsigned irq, struct irq_desc *desc)
{
	__ack_APIC_irq();
}

#endif	/* CONFIG_X86_LOCAL_APIC */

/*
 * __ipipe_enable_pipeline() -- We are running on the boot CPU, hw
 * interrupts are off, and secondary CPUs are still lost in space.
 */
void __init __ipipe_enable_pipeline(void)
{
	unsigned int irq;

#ifdef CONFIG_X86_LOCAL_APIC

	/* Map the APIC system vectors. */

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(LOCAL_TIMER_VECTOR),
			  __ipipe_do_IRQ, smp_apic_timer_interrupt,
			  __ipipe_ack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(SPURIOUS_APIC_VECTOR),
			  __ipipe_do_IRQ, smp_spurious_interrupt,
			  __ipipe_noack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(ERROR_APIC_VECTOR),
			  __ipipe_do_IRQ, smp_error_interrupt,
			  __ipipe_ack_apic);

#ifdef CONFIG_X86_THERMAL_VECTOR
	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(THERMAL_APIC_VECTOR),
			  __ipipe_do_IRQ, smp_thermal_interrupt,
			  __ipipe_ack_apic);
#endif /* CONFIG_X86_THERMAL_VECTOR */

#ifdef CONFIG_X86_MCE_THRESHOLD
	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(THRESHOLD_APIC_VECTOR),
			  __ipipe_do_IRQ, smp_threshold_interrupt,
			  __ipipe_ack_apic);
#endif /* CONFIG_X86_MCE_THRESHOLD */

#ifdef CONFIG_X86_UV
	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(UV_BAU_MESSAGE),
			  __ipipe_do_IRQ, uv_bau_message_interrupt,
			  __ipipe_ack_apic);
#endif /* CONFIG_X86_UV */

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(X86_PLATFORM_IPI_VECTOR),
			  __ipipe_do_IRQ, smp_x86_platform_ipi,
			  __ipipe_ack_apic);

	/*
	 * We expose two high priority APIC vectors the head domain
	 * may use respectively for hires timing and SMP rescheduling.
	 * We should never receive them in the root domain.
	 */
	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(IPIPE_HRTIMER_VECTOR),
			  __ipipe_do_IRQ, smp_spurious_interrupt,
			  __ipipe_ack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(IPIPE_RESCHEDULE_VECTOR),
			  __ipipe_do_IRQ, smp_spurious_interrupt,
			  __ipipe_ack_apic);

#ifdef CONFIG_IRQ_WORK
	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(IRQ_WORK_VECTOR),
			  __ipipe_do_IRQ, smp_irq_work_interrupt,
			  __ipipe_ack_apic);
#endif /* CONFIG_IRQ_WORK */

#endif	/* CONFIG_X86_LOCAL_APIC */

#ifdef CONFIG_SMP
	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(RESCHEDULE_VECTOR),
			  __ipipe_do_IRQ, smp_reschedule_interrupt,
			  __ipipe_ack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(CALL_FUNCTION_VECTOR),
			  __ipipe_do_IRQ, smp_call_function_interrupt,
			  __ipipe_ack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(CALL_FUNCTION_SINGLE_VECTOR),
			  __ipipe_do_IRQ, smp_call_function_single_interrupt,
			  __ipipe_ack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  IRQ_MOVE_CLEANUP_VECTOR,
			  __ipipe_do_IRQ, smp_irq_move_cleanup_interrupt,
			  __ipipe_ack_apic);

	ipipe_request_irq(ipipe_root_domain,
			  ipipe_apic_vector_irq(REBOOT_VECTOR),
			  __ipipe_do_IRQ, smp_reboot_interrupt,
			  __ipipe_ack_apic);
#endif	/* CONFIG_SMP */

	/*
	 * Finally, request the remaining ISA and IO-APIC
	 * interrupts. Interrupts which have already been requested
	 * will just beget a silent -EBUSY error, that's ok.
	 */
	for (irq = 0; irq < NR_IRQS; irq++)
		ipipe_request_irq(ipipe_root_domain, irq,
				  __ipipe_do_IRQ, do_IRQ,
				  NULL);
}

#ifdef CONFIG_SMP

void ipipe_set_irq_affinity(unsigned int irq, cpumask_t cpumask)
{
	if (WARN_ON_ONCE(irq_get_chip(irq)->irq_set_affinity == NULL))
		return;

	cpus_and(cpumask, cpumask, *cpu_online_mask);
	if (WARN_ON_ONCE(cpus_empty(cpumask)))
		return;

	irq_get_chip(irq)->irq_set_affinity(irq_get_irq_data(irq), &cpumask, true);
}
EXPORT_SYMBOL_GPL(ipipe_set_irq_affinity);

void ipipe_send_ipi(unsigned int ipi, cpumask_t cpumask)
{
	unsigned long flags;

	flags = hard_local_irq_save();

	cpu_clear(ipipe_processor_id(), cpumask);
	if (likely(!cpus_empty(cpumask)))
		apic->send_IPI_mask(&cpumask, ipipe_apic_irq_vector(ipi));

	hard_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_send_ipi);

void __ipipe_hook_critical_ipi(struct ipipe_domain *ipd)
{
	unsigned int ipi = IPIPE_CRITICAL_IPI;

	ipd->irqs[ipi].ackfn = __ipipe_ack_apic;
	ipd->irqs[ipi].handler = __ipipe_do_critical_sync;
	ipd->irqs[ipi].cookie = NULL;
	ipd->irqs[ipi].control = IPIPE_HANDLE_MASK|IPIPE_STICKY_MASK;
}

#endif	/* CONFIG_SMP */

static inline void __fixup_if(int s, struct pt_regs *regs)
{
	/*
	 * Have the saved hw state look like the domain stall bit, so
	 * that __ipipe_unstall_iret_root() restores the proper
	 * pipeline state for the root stage upon exit.
	 */
	if (s)
		regs->flags &= ~X86_EFLAGS_IF;
	else
		regs->flags |= X86_EFLAGS_IF;
}

void __ipipe_halt_root(void)
{
	struct ipipe_percpu_domain_data *p;

	/* Emulate sti+hlt sequence over the root domain. */

	hard_local_irq_disable();

	p = ipipe_this_cpu_root_context();

	trace_hardirqs_on();
	__clear_bit(IPIPE_STALL_FLAG, &p->status);

	if (unlikely(__ipipe_ipending_p(p))) {
		__ipipe_sync_stage();
		hard_local_irq_enable();
	} else {
#ifdef CONFIG_IPIPE_TRACE_IRQSOFF
		ipipe_trace_end(0x8000000E);
#endif /* CONFIG_IPIPE_TRACE_IRQSOFF */
		asm volatile("sti; hlt": : :"memory");
	}
}
EXPORT_SYMBOL_GPL(__ipipe_halt_root);

#if defined(CONFIG_X86_MCE) && defined(CONFIG_X86_32)
static void do_machine_check_vector(struct pt_regs *regs, long error_code)
{
	machine_check_vector(regs, error_code);
}
#endif

typedef void (*__ex_handler)(struct pt_regs *, long);

static __ex_handler __ipipe_std_extable[] = {
	[ex_do_divide_error] = do_divide_error,
	[ex_do_overflow] = do_overflow,
	[ex_do_bounds] = do_bounds,
	[ex_do_invalid_op] = do_invalid_op,
	[ex_do_coprocessor_segment_overrun] = do_coprocessor_segment_overrun,
	[ex_do_invalid_TSS] = do_invalid_TSS,
	[ex_do_segment_not_present] = do_segment_not_present,
	[ex_do_stack_segment] = do_stack_segment,
	[ex_do_general_protection] = do_general_protection,
	[ex_do_page_fault] = (__ex_handler)do_page_fault,
	[ex_do_spurious_interrupt_bug] = do_spurious_interrupt_bug,
	[ex_do_coprocessor_error] = do_coprocessor_error,
	[ex_do_alignment_check] = do_alignment_check,
#ifdef CONFIG_X86_MCE
#ifdef CONFIG_X86_32
	[ex_machine_check_vector] = do_machine_check_vector,
#else
	[ex_machine_check_vector] = do_machine_check,
#endif
#endif	/* X86_MCE */
	[ex_do_simd_coprocessor_error] = do_simd_coprocessor_error,
	[ex_do_device_not_available] = do_device_not_available,
#ifdef CONFIG_X86_32
	[ex_do_iret_error] = do_iret_error,
#endif
};

int __ipipe_handle_exception(struct pt_regs *regs, long error_code, int vector)
{
	bool hard_irqs_off = hard_irqs_disabled();
	bool root_entry = false;
	unsigned long flags = 0;
	unsigned long cr2 = 0;

#ifdef CONFIG_KGDB
	/* Fixup kgdb-own faults immediately. */
	if (__ipipe_probe_access) {
		const struct exception_table_entry *fixup =
			search_exception_tables(regs->ip);

		BUG_ON(!fixup);
		regs->ip = (unsigned long)&fixup->fixup + fixup->fixup;
		return 1;
	}
#endif /* CONFIG_KGDB */

	if (ipipe_root_p) {
		root_entry = true;

		local_save_flags(flags);
		/*
		 * Replicate hw interrupt state into the virtual mask
		 * before calling the I-pipe event handler over the
		 * root domain.
		 */
		if (hard_irqs_off)
			local_irq_disable();
	}

	/*
	 * XXX: We don't trace page faults (yet?). */
	if (vector == ex_do_page_fault)
		cr2 = native_read_cr2();

	if (unlikely(__ipipe_notify_trap(vector, regs))) {
		if (root_entry)
			ipipe_restore_root_nosync(flags);
		return 1;
	}

	if (likely(ipipe_root_p)) {
		if (!root_entry) {
			local_save_flags(flags);

			/*
			 * Sync Linux interrupt state with hardware state on
			 * entry.
			 */
			if (hard_irqs_off)
				local_irq_disable();
		}
		/*
		 * If root is not the topmost domain or in case we faulted in
		 * the iret path of x86-32, regs.flags does not match the root
		 * domain state. The fault handler or the low-level return
		 * code may evaluate it. So fix this up, either by the root
		 * state sampled on entry or, if we migrated to root, with the
		 * current state.
		 */
		__fixup_if(raw_irqs_disabled_flags(flags), regs);
	} else {
		/* Detect unhandled faults over the head domain. */
		struct ipipe_domain *ipd = ipipe_current_domain;

		/* Switch to root so that Linux can handle the fault cleanly. */
		hard_local_irq_disable();
		__ipipe_set_current_domain(ipipe_root_domain);

		/* Sync Linux interrupt state with hardware state on entry. */
		if (hard_irqs_off)
			local_irq_disable();

		ipipe_trace_panic_freeze();

		/* Always warn about user land and unfixable faults. */
		if (user_mode_vm(regs) ||
		    !search_exception_tables(instruction_pointer(regs))) {
			printk(KERN_ERR "BUG: Unhandled exception over domain"
			       " %s at 0x%lx - switching to ROOT\n",
			       ipd->name, instruction_pointer(regs));
			dump_stack();
			ipipe_trace_panic_dump();
#ifdef CONFIG_IPIPE_DEBUG
		/* Also report fixable ones when debugging is enabled. */
		} else {
			printk(KERN_WARNING "WARNING: Fixable exception over "
			       "domain %s at 0x%lx - switching to ROOT\n",
			       ipd->name, instruction_pointer(regs));
			dump_stack();
			ipipe_trace_panic_dump();
#endif /* CONFIG_IPIPE_DEBUG */
		}
	}

	if (vector == ex_do_page_fault)
		write_cr2(cr2);

	__ipipe_std_extable[vector](regs, error_code);

	/*
	 * Relevant for 64-bit: Restore root domain state as the low-level
	 * return code will not align it to regs.flags.
	 */
	if (root_entry)
		ipipe_restore_root_nosync(flags);

	return 0;
}

int __ipipe_divert_exception(struct pt_regs *regs, int vector)
{
	bool hard_irqs_off = hard_irqs_disabled();
	bool root_entry = false;
	unsigned long flags = 0;

	if (ipipe_root_p) {
		root_entry = true;
		local_save_flags(flags);
		if (hard_irqs_off) {
			/*
			 * Same root state handling as in
			 * __ipipe_handle_exception.
			 */
			local_irq_disable();
		}
	}
#ifdef CONFIG_KGDB
	/*
	 * Catch int1 and int3 for kgdb here. They may trigger over
	 * inconsistent states even when the root domain is active.
	 */
	if (kgdb_io_module_registered && (vector == 1 || vector == 3)) {
		unsigned int condition = 0;

		if (vector == 1) {
			if (!atomic_read(&kgdb_cpu_doing_single_step) != -1 &&
			    test_thread_flag(TIF_SINGLESTEP))
				goto skip_kgdb;
			get_debugreg(condition, 6);
		}
		if (!user_mode(regs) &&
		    !kgdb_handle_exception(vector, SIGTRAP, condition, regs)) {
			if (root_entry) {
				ipipe_restore_root_nosync(flags);
				__fixup_if(root_entry ?
					   raw_irqs_disabled_flags(flags) :
					   raw_irqs_disabled(), regs);
			}
			return 1;
		}
	}
skip_kgdb:
#endif /* CONFIG_KGDB */

	if (unlikely(__ipipe_notify_trap(vector, regs))) {
		if (root_entry)
			ipipe_restore_root_nosync(flags);
		return 1;
	}

	/* see __ipipe_handle_exception */
	if (likely(ipipe_root_p)) {
		if (!root_entry) {
			local_save_flags(flags);
			if (hard_irqs_off)
				local_irq_disable();
		}
		__fixup_if(raw_irqs_disabled_flags(flags), regs);
	}
	/*
	 * No need to restore root state in the 64-bit case, the Linux
	 * handler and the return code will take care of it.
	 */

	return 0;
}

int __ipipe_handle_irq(struct pt_regs *regs)
{
	struct ipipe_percpu_data *p = __ipipe_this_cpu_ptr(&ipipe_percpu);
	int irq, vector = regs->orig_ax, flags = 0;
	struct pt_regs *tick_regs;

	if (likely(vector < 0)) {
		irq = __this_cpu_read(vector_irq[~vector]);
		BUG_ON(irq < 0);
	} else { /* Software-generated. */
		irq = vector;
		flags = IPIPE_IRQF_NOACK;
	}

	/*
	 * Given our deferred dispatching model for regular IRQs, we
	 * only record CPU regs for the last timer interrupt, so that
	 * the timer handler charges CPU times properly. It is assumed
	 * that no other interrupt handler cares for such information.
	 */
	if (irq == p->hrtimer_irq || p->hrtimer_irq == -1) {
		tick_regs = &p->tick_regs;
		tick_regs->flags = regs->flags;
		tick_regs->cs = regs->cs;
		tick_regs->ip = regs->ip;
		tick_regs->bp = regs->bp;
#ifdef CONFIG_X86_64
		tick_regs->ss = regs->ss;
		tick_regs->sp = regs->sp;
#endif
		if (!__ipipe_root_p)
			tick_regs->flags &= ~X86_EFLAGS_IF;
	}

	__ipipe_dispatch_irq(irq, flags);

	if (user_mode(regs) && ipipe_test_thread_flag(TIP_MAYDAY))
		__ipipe_call_mayday(regs);

	if (!__ipipe_root_p ||
	    test_bit(IPIPE_STALL_FLAG, &__ipipe_root_status))
		return 0;

	return 1;
}

void __ipipe_arch_share_current(int flags)
{
	struct task_struct *p = current;

	/*
	 * Setup a clean extended FPU state for kernel threads.
	 */
	if (p->mm == NULL && use_xsave())
		memcpy(p->thread.fpu.state, init_xstate_buf, xstate_size);
}

#ifdef CONFIG_X86_32
void update_vsyscall(struct timekeeper *tk)
{
	if (tk->clock == &clocksource_tsc)
		ipipe_update_hostrt(tk);
}

void update_vsyscall_tz(void)
{
}

#ifdef CONFIG_IPIPE_WANT_CLOCKSOURCE
u64 __ipipe_get_cs_tsc(void);
EXPORT_SYMBOL_GPL(__ipipe_get_cs_tsc);
#endif

#endif /* CONFIG_X86_32 */

struct task_struct *__switch_to(struct task_struct *prev_p,
				struct task_struct *next_p);
EXPORT_SYMBOL_GPL(do_munmap);
EXPORT_SYMBOL_GPL(__switch_to);
EXPORT_SYMBOL_GPL(show_stack);
EXPORT_PER_CPU_SYMBOL_GPL(fpu_owner_task);

EXPORT_PER_CPU_SYMBOL_GPL(init_tss);
#ifdef CONFIG_SMP
EXPORT_PER_CPU_SYMBOL_GPL(cpu_tlbstate);
#endif /* CONFIG_SMP */

#if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)
EXPORT_SYMBOL(tasklist_lock);
#endif /* CONFIG_SMP || CONFIG_DEBUG_SPINLOCK */

#if defined(CONFIG_CC_STACKPROTECTOR) && defined(CONFIG_X86_64)
EXPORT_PER_CPU_SYMBOL_GPL(irq_stack_union);
#endif
