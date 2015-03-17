void __sp804_clocksource_and_sched_clock_init(void __iomem *,
					      unsigned long,
					      const char *, int);

static inline void sp804_clocksource_init(void __iomem *base, 
					  unsigned long phys, const char *name)
{
	__sp804_clocksource_and_sched_clock_init(base, phys, name, 0);
}

static inline void sp804_clocksource_and_sched_clock_init(void __iomem *base,
							  unsigned long phys,
							  const char *name)
{
	__sp804_clocksource_and_sched_clock_init(base, phys, name, 1);
}

void sp804_clockevents_init(void __iomem *, unsigned int, const char *);
