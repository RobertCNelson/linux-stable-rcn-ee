/*   -*- linux-c -*-
 *   arch/x86/include/asm/ipipe_64.h
 *
 *   Copyright (C) 2007-2012 Philippe Gerum.
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
 */

#ifndef __X86_IPIPE_64_H
#define __X86_IPIPE_64_H

#define ipipe_read_tsc(t)  do {		\
	unsigned int __a,__d;			\
	asm volatile("rdtsc" : "=a" (__a), "=d" (__d)); \
	(t) = ((unsigned long)__a) | (((unsigned long)__d)<<32); \
} while(0)

extern unsigned int cpu_khz;
#define __ipipe_cpu_freq	({ unsigned long long __freq = (1000ULL * cpu_khz); __freq; })
#define __ipipe_hrclock_freq	__ipipe_cpu_freq

#define ipipe_tsc2ns(t)	(((t) * 1000UL) / (__ipipe_hrclock_freq / 1000000UL))
#define ipipe_tsc2us(t)	((t) / (__ipipe_hrclock_freq / 1000000UL))

static inline const char *ipipe_clock_name(void)
{
	return "tsc";
}

/* Private interface -- Internal use only */

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
      __asm__("bsrq %1, %0":"=r"(ul)
	      :	"rm"(ul));
      return ul;
}

struct irq_desc;

#define __root_irq_trampoline(__handler__, __regs__)			\
	do {								\
		__asm__ __volatile__("movq   %%rsp, %%rax\n\t"		\
				     "pushq  $0\n\t"			\
				     "pushq  %%rax\n\t"			\
				     "pushfq \n\t"			\
				     "orq    %[x86if],(%%rsp)\n\t"	\
				     "pushq  %[kernel_cs]\n\t"		\
				     "pushq  $1f\n\t"			\
				     "pushq  %[vector]\n\t"		\
				     "subq   $9*8,%%rsp\n\t"		\
				     "movq   %%rdi,8*8(%%rsp)\n\t"	\
				     "movq   %%rsi,7*8(%%rsp)\n\t"	\
				     "movq   %%rdx,6*8(%%rsp)\n\t"	\
				     "movq   %%rcx,5*8(%%rsp)\n\t"	\
				     "movq   %%rax,4*8(%%rsp)\n\t"	\
				     "movq   %%r8,3*8(%%rsp)\n\t"	\
				     "movq   %%r9,2*8(%%rsp)\n\t"	\
				     "movq   %%r10,1*8(%%rsp)\n\t"	\
				     "movq   %%r11,(%%rsp)\n\t"		\
				     "call   *%[handler]\n\t"		\
				     "cli\n\t"				\
				     "jmp    exit_intr\n\t"		\
				     "1:     cli\n"			\
				     : /* no output */			\
				     : [kernel_cs] "i" (__KERNEL_CS),	\
				     [vector] "rm" (__regs__->orig_ax),	\
				     [handler] "r" (__handler__),	\
				       "D" (__regs__),			\
				     [x86if] "i" (X86_EFLAGS_IF)	\
				     : "rax");				\
	} while (0)

#ifdef CONFIG_PREEMPT
#define __ipipe_check_root_resched()			\
	(preempt_count() == 0 && need_resched() &&	\
	 per_cpu(irq_count, ipipe_processor_id()) < 0)
#else
#define __ipipe_check_root_resched()	0
#endif

#endif	/* !__X86_IPIPE_64_H */
