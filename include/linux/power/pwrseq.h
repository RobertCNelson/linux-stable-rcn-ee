#ifndef __LINUX_PWRSEQ_H
#define __LINUX_PWRSEQ_H

#include <linux/of.h>

#define PWRSEQ_MAX_CLKS		3

/**
 * struct pwrseq - the power sequence structure
 * @pwrseq_of_match_table: the OF device id table this pwrseq library supports
 * @node: the list pointer to be added to pwrseq list
 * @get: the API is used to get pwrseq instance from the device node
 * @on: do power on for this pwrseq instance
 * @off: do power off for this pwrseq instance
 * @put: release the resources on this pwrseq instance
 * @suspend: do suspend operation on this pwrseq instance
 * @resume: do resume operation on this pwrseq instance
 * @used: this pwrseq instance is used by device
 */
struct pwrseq {
	const struct of_device_id *pwrseq_of_match_table;
	struct list_head node;
	int (*get)(struct device_node *np, struct pwrseq *p);
	int (*on)(struct pwrseq *p);
	void (*off)(struct pwrseq *p);
	void (*put)(struct pwrseq *p);
	int (*suspend)(struct pwrseq *p);
	int (*resume)(struct pwrseq *p);
	bool used;
	bool suspended;
};

/* used for power sequence instance list in one driver */
struct pwrseq_list_per_dev {
	struct pwrseq *pwrseq;
	struct list_head list;
};

#if IS_ENABLED(CONFIG_POWER_SEQUENCE)
void pwrseq_register(struct pwrseq *pwrseq);
void pwrseq_unregister(struct pwrseq *pwrseq);
struct pwrseq *of_pwrseq_on(struct device_node *np);
void of_pwrseq_off(struct pwrseq *pwrseq);
int of_pwrseq_on_list(struct device_node *np, struct list_head *head);
void of_pwrseq_off_list(struct list_head *head);
int pwrseq_suspend(struct pwrseq *p);
int pwrseq_resume(struct pwrseq *p);
int pwrseq_suspend_list(struct list_head *head);
int pwrseq_resume_list(struct list_head *head);
#else
static inline void pwrseq_register(struct pwrseq *pwrseq) {}
static inline void pwrseq_unregister(struct pwrseq *pwrseq) {}
static inline struct pwrseq *of_pwrseq_on(struct device_node *np)
{
	return NULL;
}
static void of_pwrseq_off(struct pwrseq *pwrseq) {}
static int of_pwrseq_on_list(struct device_node *np, struct list_head *head)
{
	return 0;
}
static void of_pwrseq_off_list(struct list_head *head) {}
static int pwrseq_suspend(struct pwrseq *p)
{
	return 0;
}
static int pwrseq_resume(struct pwrseq *p)
{
	return 0;
}
static int pwrseq_suspend_list(struct list_head *head)
{
	return 0;
}
static int pwrseq_resume_list(struct list_head *head)
{
	return 0;
}
#endif /* CONFIG_POWER_SEQUENCE */

#endif  /* __LINUX_PWRSEQ_H */
