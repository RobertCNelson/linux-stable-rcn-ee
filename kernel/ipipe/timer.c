/* -*- linux-c -*-
 * linux/kernel/ipipe/timer.c
 *
 * Copyright (C) 2012 Gilles Chanteperdrix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * I-pipe timer request interface.
 */
#include <linux/ipipe.h>
#include <linux/percpu.h>
#include <linux/irqdesc.h>
#include <linux/cpumask.h>
#include <linux/spinlock.h>
#include <linux/ipipe_tickdev.h>
#include <linux/interrupt.h>
#include <linux/export.h>

unsigned long __ipipe_hrtimer_freq;

static LIST_HEAD(timers);
static IPIPE_DEFINE_SPINLOCK(lock);

static DEFINE_PER_CPU(struct ipipe_timer *, percpu_timer);

/*
 * Default request method: switch to oneshot mode if supported.
 */
static void ipipe_timer_default_request(struct ipipe_timer *timer, int steal)
{
	struct clock_event_device *evtdev = timer->host_timer;

	if (!(evtdev->features & CLOCK_EVT_FEAT_ONESHOT))
		return;

	if (clockevent_state_oneshot(evtdev) ||
		clockevent_state_oneshot_stopped(evtdev))
		timer->orig_mode = CLOCK_EVT_MODE_ONESHOT;
	else {
		if (clockevent_state_periodic(evtdev))
			timer->orig_mode = CLOCK_EVT_MODE_PERIODIC;
		else if (clockevent_state_shutdown(evtdev))
			timer->orig_mode = CLOCK_EVT_MODE_SHUTDOWN;
		else
			timer->orig_mode = CLOCK_EVT_MODE_UNUSED;
		evtdev->set_state_oneshot(evtdev);
		evtdev->set_next_event(timer->freq / HZ, evtdev);
	}
}

/*
 * Default release method: return the timer to the mode it had when
 * starting.
 */
static void ipipe_timer_default_release(struct ipipe_timer *timer)
{
	struct clock_event_device *evtdev = timer->host_timer;

	switch (timer->orig_mode) {
	case CLOCK_EVT_MODE_SHUTDOWN:
		evtdev->set_state_shutdown(evtdev);
		break;
	case CLOCK_EVT_MODE_PERIODIC:
		evtdev->set_state_periodic(evtdev);
	case CLOCK_EVT_MODE_ONESHOT:
		evtdev->set_next_event(timer->freq / HZ, evtdev);
		break;
	}
}

static int get_dev_mode(struct clock_event_device *evtdev)
{
	if (clockevent_state_oneshot(evtdev) ||
		clockevent_state_oneshot_stopped(evtdev))
		return CLOCK_EVT_MODE_ONESHOT;

	if (clockevent_state_periodic(evtdev))
		return CLOCK_EVT_MODE_PERIODIC;

	if (clockevent_state_shutdown(evtdev))
		return CLOCK_EVT_MODE_SHUTDOWN;

	return CLOCK_EVT_MODE_UNUSED;
}

void ipipe_host_timer_register(struct clock_event_device *evtdev)
{
	struct ipipe_timer *timer = evtdev->ipipe_timer;

	if (timer == NULL)
		return;

	timer->orig_mode = CLOCK_EVT_MODE_UNUSED;

	if (timer->request == NULL)
		timer->request = ipipe_timer_default_request;

	/*
	 * By default, use the same method as linux timer, on ARM at
	 * least, most set_next_event methods are safe to be called
	 * from Xenomai domain anyway.
	 */
	if (timer->set == NULL) {
		timer->timer_set = evtdev;
		timer->set = (typeof(timer->set))evtdev->set_next_event;
	}

	if (timer->release == NULL)
		timer->release = ipipe_timer_default_release;

	if (timer->name == NULL)
		timer->name = evtdev->name;

	if (timer->rating == 0)
		timer->rating = evtdev->rating;

	timer->freq = (1000000000ULL * evtdev->mult) >> evtdev->shift;

	if (timer->min_delay_ticks == 0)
		timer->min_delay_ticks =
			(evtdev->min_delta_ns * evtdev->mult) >> evtdev->shift;

	if (timer->max_delay_ticks == 0)
		timer->max_delay_ticks =
			(evtdev->max_delta_ns * evtdev->mult) >> evtdev->shift;

	if (timer->cpumask == NULL)
		timer->cpumask = evtdev->cpumask;

	timer->host_timer = evtdev;

	ipipe_timer_register(timer);
}

#ifdef CONFIG_HOTPLUG_CPU
void ipipe_host_timer_cleanup(struct clock_event_device *evtdev)
{
	struct ipipe_timer *timer = evtdev->ipipe_timer;
	unsigned long flags;

	if (timer == NULL)
		return;

	raw_spin_lock_irqsave(&lock, flags);
	list_del(&timer->link);
	raw_spin_unlock_irqrestore(&lock, flags);
}
#endif /* CONFIG_HOTPLUG_CPU */

/*
 * register a timer: maintain them in a list sorted by rating
 */
void ipipe_timer_register(struct ipipe_timer *timer)
{
	struct ipipe_timer *t;
	unsigned long flags;

	if (timer->timer_set == NULL)
		timer->timer_set = timer;

	if (timer->cpumask == NULL)
		timer->cpumask = cpumask_of(smp_processor_id());

	raw_spin_lock_irqsave(&lock, flags);

	list_for_each_entry(t, &timers, link) {
		if (t->rating <= timer->rating) {
			__list_add(&timer->link, t->link.prev, &t->link);
			goto done;
		}
	}
	list_add_tail(&timer->link, &timers);
  done:
	raw_spin_unlock_irqrestore(&lock, flags);
}

static void ipipe_timer_request_sync(void)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);
	struct clock_event_device *evtdev;
	int steal;

	if (!timer)
		return;

	evtdev = timer->host_timer;
	steal = evtdev != NULL && !clockevent_state_detached(evtdev);
	timer->request(timer, steal);
}

static void config_pcpu_timer(struct ipipe_timer *t, unsigned hrclock_freq)
{
	unsigned long long tmp;
	unsigned hrtimer_freq;

	if (__ipipe_hrtimer_freq != t->freq)
		__ipipe_hrtimer_freq = t->freq;

	hrtimer_freq = t->freq;
	if (__ipipe_hrclock_freq > UINT_MAX)
		hrtimer_freq /= 1000;

	t->c2t_integ = hrtimer_freq / hrclock_freq;
	tmp = (((unsigned long long)
		(hrtimer_freq % hrclock_freq)) << 32)
		+ hrclock_freq - 1;
	do_div(tmp, hrclock_freq);
	t->c2t_frac = tmp;
}

/* Set up a timer as per-cpu timer for ipipe */
static void install_pcpu_timer(unsigned cpu, unsigned hrclock_freq,
			      struct ipipe_timer *t)
{
	per_cpu(ipipe_percpu.hrtimer_irq, cpu) = t->irq;
	per_cpu(percpu_timer, cpu) = t;
	config_pcpu_timer(t, hrclock_freq);
}

static void select_root_only_timer(unsigned cpu, unsigned hrclock_khz,
				   const struct cpumask *mask,
				   struct ipipe_timer *t) {
	unsigned icpu;
	struct clock_event_device *evtdev;

	/*
	 * If no ipipe-supported CPU shares an interrupt with the
	 * timer, we do not need to care about it.
	 */
	for_each_cpu(icpu, mask) {
		if (t->irq == per_cpu(ipipe_percpu.hrtimer_irq, icpu)) {
			evtdev = t->host_timer;
			if (evtdev && clockevent_state_shutdown(evtdev))
				continue;
			goto found;
		}
	}

	return;

found:
	install_pcpu_timer(cpu, hrclock_khz, t);
}

/*
 * Choose per-cpu timers with the highest rating by traversing the
 * rating-sorted list for each CPU.
 */
int ipipe_select_timers(const struct cpumask *mask)
{
	unsigned hrclock_freq;
	unsigned long long tmp;
	struct ipipe_timer *t;
	struct clock_event_device *evtdev;
	unsigned long flags;
	unsigned cpu;
	cpumask_var_t fixup;

	if (!__ipipe_hrclock_ok()) {
		printk("I-pipe: high-resolution clock not working\n");
		return -ENODEV;
	}

	if (__ipipe_hrclock_freq > UINT_MAX) {
		tmp = __ipipe_hrclock_freq;
		do_div(tmp, 1000);
		hrclock_freq = tmp;
	} else
		hrclock_freq = __ipipe_hrclock_freq;


	if (!zalloc_cpumask_var(&fixup, GFP_KERNEL)) {
		WARN_ON(1);
		return -ENODEV;
	}

	raw_spin_lock_irqsave(&lock, flags);

	/* First, choose timers for the CPUs handled by ipipe */
	for_each_cpu(cpu, mask) {
		list_for_each_entry(t, &timers, link) {
			if (!cpumask_test_cpu(cpu, t->cpumask))
				continue;

			evtdev = t->host_timer;
			if (evtdev && clockevent_state_shutdown(evtdev))
				continue;
			goto found;
		}

		printk("I-pipe: could not find timer for cpu #%d\n",
		       cpu);
		goto err_remove_all;
found:
		install_pcpu_timer(cpu, hrclock_freq, t);
	}

	/*
	 * Second, check if we need to fix up any CPUs not supported
	 * by ipipe (but by Linux) whose interrupt may need to be
	 * forwarded because they have the same IRQ as an ipipe-enabled
	 * timer.
	 */
	cpumask_andnot(fixup, cpu_online_mask, mask);

	for_each_cpu(cpu, fixup) {
		list_for_each_entry(t, &timers, link) {
			if (!cpumask_test_cpu(cpu, t->cpumask))
				continue;

			select_root_only_timer(cpu, hrclock_freq, mask, t);
		}
	}

	raw_spin_unlock_irqrestore(&lock, flags);

	free_cpumask_var(fixup);
	flags = ipipe_critical_enter(ipipe_timer_request_sync);
	ipipe_timer_request_sync();
	ipipe_critical_exit(flags);

	return 0;

err_remove_all:
	raw_spin_unlock_irqrestore(&lock, flags);
	free_cpumask_var(fixup);

	for_each_cpu(cpu, mask) {
		per_cpu(ipipe_percpu.hrtimer_irq, cpu) = -1;
		per_cpu(percpu_timer, cpu) = NULL;
	}
	__ipipe_hrtimer_freq = 0;

	return -ENODEV;
}

static void ipipe_timer_release_sync(void)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);

	if (timer)
		timer->release(timer);
}

void ipipe_timers_release(void)
{
	unsigned long flags;
	unsigned cpu;

	flags = ipipe_critical_enter(ipipe_timer_release_sync);
	ipipe_timer_release_sync();
	ipipe_critical_exit(flags);

	for_each_online_cpu(cpu) {
		per_cpu(ipipe_percpu.hrtimer_irq, cpu) = -1;
		per_cpu(percpu_timer, cpu) = NULL;
		__ipipe_hrtimer_freq = 0;
	}
}

static void __ipipe_ack_hrtimer_irq(struct irq_desc *desc)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);

	/*
	 * Pseudo-IRQs like pipelined IPIs have no descriptor, we have
	 * to check for this.
	 */
	if (desc)
		desc->ipipe_ack(desc);

	if (timer->ack)
		timer->ack();

	if (desc)
		desc->ipipe_end(desc);
}

static int do_set_oneshot(struct clock_event_device *cdev)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);

	timer->orig_set_state_oneshot(cdev);
	timer->mode_handler(CLOCK_EVT_MODE_ONESHOT, cdev);

	return 0;
}

static int do_set_oneshot_stopped(struct clock_event_device *cdev)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);

	timer->mode_handler(CLOCK_EVT_MODE_SHUTDOWN, cdev);

	return 0;
}

static int do_set_periodic(struct clock_event_device *cdev)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);

	timer->mode_handler(CLOCK_EVT_MODE_PERIODIC, cdev);

	return 0;
}

static int do_set_shutdown(struct clock_event_device *cdev)
{
	struct ipipe_timer *timer = __ipipe_raw_cpu_read(percpu_timer);

	timer->mode_handler(CLOCK_EVT_MODE_SHUTDOWN, cdev);

	return 0;
}

int clockevents_program_event(struct clock_event_device *dev,
			      ktime_t expires, bool force);

struct grab_timer_data {
	void (*tick_handler)(void);
	void (*emumode)(enum clock_event_mode mode,
			struct clock_event_device *cdev);
	int (*emutick)(unsigned long evt,
		       struct clock_event_device *cdev);
	int retval;
};

static void grab_timer(void *arg)
{
	struct grab_timer_data *data = arg;
	struct clock_event_device *evtdev;
	struct ipipe_timer *timer;
	struct irq_desc *desc;
	unsigned long flags;
	int steal, ret;

	flags = hard_local_irq_save();

	timer = this_cpu_read(percpu_timer);
	evtdev = timer->host_timer;
	ret = ipipe_request_irq(ipipe_head_domain, timer->irq,
				(ipipe_irq_handler_t)data->tick_handler,
				NULL, __ipipe_ack_hrtimer_irq);
	if (ret < 0 && ret != -EBUSY) {
		hard_local_irq_restore(flags);
		data->retval = ret;
		return;
	}

	steal = !clockevent_state_detached(evtdev);
	if (steal && evtdev->ipipe_stolen == 0) {
		timer->real_mult = evtdev->mult;
		timer->real_shift = evtdev->shift;
		timer->orig_set_state_periodic = evtdev->set_state_periodic;
		timer->orig_set_state_oneshot = evtdev->set_state_oneshot;
		timer->orig_set_state_oneshot_stopped = evtdev->set_state_oneshot_stopped;
		timer->orig_set_state_shutdown = evtdev->set_state_shutdown;
		timer->orig_set_next_event = evtdev->set_next_event;
		timer->mode_handler = data->emumode;
		evtdev->mult = 1;
		evtdev->shift = 0;
		evtdev->max_delta_ns = UINT_MAX;
		if (timer->orig_set_state_periodic)
			evtdev->set_state_periodic = do_set_periodic;
		if (timer->orig_set_state_oneshot)
			evtdev->set_state_oneshot = do_set_oneshot;
		if (timer->orig_set_state_oneshot_stopped)
			evtdev->set_state_oneshot_stopped = do_set_oneshot_stopped;
		if (timer->orig_set_state_shutdown)
			evtdev->set_state_shutdown = do_set_shutdown;
		evtdev->set_next_event = data->emutick;
		evtdev->ipipe_stolen = 1;
	}

	hard_local_irq_restore(flags);

	data->retval = get_dev_mode(evtdev);

	desc = irq_to_desc(timer->irq);
	if (desc && irqd_irq_disabled(&desc->irq_data))
		ipipe_enable_irq(timer->irq);

	if (evtdev->ipipe_stolen && clockevent_state_oneshot(evtdev)) {
		ret = clockevents_program_event(evtdev,
						evtdev->next_event, true);
		if (ret)
			data->retval = ret;
	}
}

int ipipe_timer_start(void (*tick_handler)(void),
		      void (*emumode)(enum clock_event_mode mode,
				      struct clock_event_device *cdev),
		      int (*emutick)(unsigned long evt,
				     struct clock_event_device *cdev),
		      unsigned int cpu)
{
	struct grab_timer_data data;
	int ret;

	data.tick_handler = tick_handler;
	data.emutick = emutick;
	data.emumode = emumode;
	data.retval = -EINVAL;
	ret = smp_call_function_single(cpu, grab_timer, &data, true);

	return ret ?: data.retval;
}

static void release_timer(void *arg)
{
	struct clock_event_device *evtdev;
	struct ipipe_timer *timer;
	struct irq_desc *desc;
	unsigned long flags;

	flags = hard_local_irq_save();

	timer = this_cpu_read(percpu_timer);

	desc = irq_to_desc(timer->irq);
	if (desc && irqd_irq_disabled(&desc->irq_data))
		ipipe_disable_irq(timer->irq);

	ipipe_free_irq(ipipe_head_domain, timer->irq);

	evtdev = timer->host_timer;
	if (evtdev && evtdev->ipipe_stolen) {
		evtdev->mult = timer->real_mult;
		evtdev->shift = timer->real_shift;
		evtdev->set_state_periodic = timer->orig_set_state_periodic;
		evtdev->set_state_oneshot = timer->orig_set_state_oneshot;
		evtdev->set_state_oneshot_stopped = timer->orig_set_state_oneshot_stopped;
		evtdev->set_state_shutdown = timer->orig_set_state_shutdown;
		evtdev->set_next_event = timer->orig_set_next_event;
		evtdev->ipipe_stolen = 0;
		hard_local_irq_restore(flags);
		if (clockevent_state_oneshot(evtdev))
			clockevents_program_event(evtdev,
						  evtdev->next_event, true);
	} else
		hard_local_irq_restore(flags);
}

void ipipe_timer_stop(unsigned int cpu)
{
	smp_call_function_single(cpu, release_timer, NULL, true);
}

void ipipe_timer_set(unsigned long cdelay)
{
	unsigned long tdelay;
	struct ipipe_timer *t;

	t = __ipipe_raw_cpu_read(percpu_timer);

	/*
	 * Even though some architectures may use a 64 bits delay
	 * here, we voluntarily limit to 32 bits, 4 billions ticks
	 * should be enough for now. Would a timer needs more, an
	 * extra call to the tick handler would simply occur after 4
	 * billions ticks.
	 */
	if (cdelay > UINT_MAX)
		cdelay = UINT_MAX;

	tdelay = cdelay;
	if (t->c2t_integ != 1)
		tdelay *= t->c2t_integ;
	if (t->c2t_frac)
		tdelay += ((unsigned long long)cdelay * t->c2t_frac) >> 32;
	if (tdelay < t->min_delay_ticks)
		tdelay = t->min_delay_ticks;
	if (tdelay > t->max_delay_ticks)
		tdelay = t->max_delay_ticks;

	if (t->set(tdelay, t->timer_set) < 0)
		ipipe_raise_irq(t->irq);
}
EXPORT_SYMBOL_GPL(ipipe_timer_set);

const char *ipipe_timer_name(void)
{
	return per_cpu(percpu_timer, 0)->name;
}
EXPORT_SYMBOL_GPL(ipipe_timer_name);

unsigned ipipe_timer_ns2ticks(struct ipipe_timer *timer, unsigned ns)
{
	unsigned long long tmp;
	BUG_ON(!timer->freq);
	tmp = (unsigned long long)ns * timer->freq;
	do_div(tmp, 1000000000);
	return tmp;
}

#ifdef CONFIG_IPIPE_HAVE_HOSTRT
/*
 * NOTE: The architecture specific code must only call this function
 * when a clocksource suitable for CLOCK_HOST_REALTIME is enabled.
 * The event receiver is responsible for providing proper locking.
 */
void ipipe_update_hostrt(struct timekeeper *tk)
{
	struct tk_read_base *tkr = &tk->tkr_mono;
	struct clocksource *clock = tkr->clock;
	struct ipipe_hostrt_data data;
	struct timespec xt;

	xt.tv_sec = tk->xtime_sec;
	xt.tv_nsec = (long)(tkr->xtime_nsec >> tkr->shift);
	ipipe_root_only();
	data.live = 1;
	data.cycle_last = tkr->cycle_last;
	data.mask = clock->mask;
	data.mult = tkr->mult;
	data.shift = tkr->shift;
	data.wall_time_sec = xt.tv_sec;
	data.wall_time_nsec = xt.tv_nsec;
	data.wall_to_monotonic.tv_sec = tk->wall_to_monotonic.tv_sec;
	data.wall_to_monotonic.tv_nsec = tk->wall_to_monotonic.tv_nsec;
	__ipipe_notify_kevent(IPIPE_KEVT_HOSTRT, &data);
}

#endif /* CONFIG_IPIPE_HAVE_HOSTRT */

int clockevents_program_event(struct clock_event_device *dev, ktime_t expires,
			      bool force);

void __ipipe_timer_refresh_freq(unsigned int hrclock_freq)
{
	struct ipipe_timer *t = __ipipe_raw_cpu_read(percpu_timer);
	unsigned long flags;

	if (t && t->refresh_freq) {
		t->freq = t->refresh_freq();
		flags = hard_local_irq_save();
		config_pcpu_timer(t, hrclock_freq);
		hard_local_irq_restore(flags);
		clockevents_program_event(t->host_timer,
					  t->host_timer->next_event, false);
	}
}
