#ifndef __LINUX_PWRSEQ_H
#define __LINUX_PWRSEQ_H

#include <linux/of.h>

#define PWRSEQ_MAX_CLKS		3

struct pwrseq {
	const struct of_device_id *pwrseq_of_match_table;
	struct list_head node;
	int (*get)(struct device_node *np, struct pwrseq *p);
	int (*on)(struct pwrseq *p);
	void (*off)(struct pwrseq *p);
	void (*put)(struct pwrseq *p);
	void (*free)(struct pwrseq *p);
	int (*suspend)(struct pwrseq *p);
	int (*resume)(struct pwrseq *p);
	bool used;
};

/* used for power sequence instance list in one driver */
struct pwrseq_list_per_dev {
	struct pwrseq *pwrseq;
	struct list_head list;
};

#if IS_ENABLED(CONFIG_POWER_SEQUENCE)
int pwrseq_get(struct device_node *np, struct pwrseq *p);
int pwrseq_on(struct pwrseq *p);
void pwrseq_off(struct pwrseq *p);
void pwrseq_put(struct pwrseq *p);
void pwrseq_free(struct pwrseq *p);
int pwrseq_suspend(struct pwrseq *p);
int pwrseq_resume(struct pwrseq *p);
void pwrseq_register(struct pwrseq *pwrseq);
struct pwrseq *of_pwrseq_on(struct device_node *np);
void of_pwrseq_off(struct pwrseq *pwrseq);
int of_pwrseq_on_list(struct device_node *np, struct list_head *head);
void of_pwrseq_off_list(struct list_head *head);
#else
static inline int pwrseq_get(struct device_node *np, struct pwrseq *p)
{
	return 0;
}
static inline int pwrseq_on(struct pwrseq *p)
{
	return 0;
}
static inline void pwrseq_off(struct pwrseq *p) {}
static inline void pwrseq_put(struct pwrseq *p) {}
static inline void pwrseq_free(struct pwrseq *p) {}
static inline int pwrseq_suspend(struct pwrseq *p)
{
	return 0;
}
static inline int pwrseq_resume(struct pwrseq *p)
{
	return 0;
}
static inline void pwrseq_register(struct pwrseq *pwrseq) {}
static inline struct pwrseq *of_pwrseq_on(struct device_node *np)
{
	return NULL;
}
void of_pwrseq_off(struct pwrseq *pwrseq) {}
int of_pwrseq_on_list(struct device_node *np, struct list_head *head)
{
	return 0;
}
void of_pwrseq_off_list(struct list_head *head) {}
#endif /* CONFIG_POWER_SEQUENCE */

#endif  /* __LINUX_PWRSEQ_H */
