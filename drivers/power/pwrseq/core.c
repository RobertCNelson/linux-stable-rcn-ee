/*
 * core.c	power sequence core file
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Author: Peter Chen <peter.chen@nxp.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/power/pwrseq.h>

static DEFINE_MUTEX(pwrseq_list_mutex);
static LIST_HEAD(pwrseq_list);

int pwrseq_get(struct device_node *np, struct pwrseq *p)
{
	if (p && p->get)
		return p->get(np, p);

	return -ENOTSUPP;
}
EXPORT_SYMBOL_GPL(pwrseq_get);

int pwrseq_on(struct pwrseq *p)
{
	if (p && p->on)
		return p->on(p);

	return -ENOTSUPP;
}
EXPORT_SYMBOL_GPL(pwrseq_on);

void pwrseq_off(struct pwrseq *p)
{
	if (p && p->off)
		p->off(p);
}
EXPORT_SYMBOL_GPL(pwrseq_off);

void pwrseq_put(struct pwrseq *p)
{
	if (p && p->put)
		p->put(p);
}
EXPORT_SYMBOL_GPL(pwrseq_put);

int pwrseq_suspend(struct pwrseq *p)
{
	if (p && p->suspend)
		return p->suspend(p);

	return 0;
}
EXPORT_SYMBOL_GPL(pwrseq_suspend);

int pwrseq_resume(struct pwrseq *p)
{
	if (p && p->resume)
		return p->resume(p);

	return 0;
}
EXPORT_SYMBOL_GPL(pwrseq_resume);

void pwrseq_register(struct pwrseq *pwrseq)
{
	mutex_lock(&pwrseq_list_mutex);
	list_add(&pwrseq->node, &pwrseq_list);
	mutex_unlock(&pwrseq_list_mutex);
}
EXPORT_SYMBOL_GPL(pwrseq_register);

void pwrseq_unregister(struct pwrseq *pwrseq)
{
	mutex_lock(&pwrseq_list_mutex);
	list_del(&pwrseq->node);
	mutex_unlock(&pwrseq_list_mutex);
}
EXPORT_SYMBOL_GPL(pwrseq_unregister);

static struct pwrseq *pwrseq_find_available_instance(struct device_node *np)
{
	struct pwrseq *pwrseq;

	list_for_each_entry(pwrseq, &pwrseq_list, node) {
		if (pwrseq->used)
			continue;

		/* compare compatible string for pwrseq node */
		if (of_match_node(pwrseq->pwrseq_of_match_table, np)) {
			pwrseq->used = true;
			return pwrseq;
		}

		/* return generic pwrseq instance */
		if (!strcmp(pwrseq->pwrseq_of_match_table->compatible,
				"generic")) {
			pr_debug("using generic pwrseq instance for %s\n",
				np->full_name);
			pwrseq->used = true;
			return pwrseq;
		}
	}
	pr_warn("Can't find any pwrseq instances for %s\n", np->full_name);

	return NULL;
}

struct pwrseq *of_pwrseq_on(struct device_node *np)
{
	struct pwrseq *pwrseq;
	int ret;

	pwrseq = pwrseq_find_available_instance(np);
	if (!pwrseq)
		return ERR_PTR(-ENONET);

	ret = pwrseq_get(np, pwrseq);
	if (ret) {
		/* Mark current pwrseq as unused */
		pwrseq->used = false;
		return ERR_PTR(ret);
	}

	ret = pwrseq_on(pwrseq);
	if (ret)
		goto pwr_put;

	return pwrseq;

pwr_put:
	pwrseq_put(pwrseq);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(of_pwrseq_on);

void of_pwrseq_off(struct pwrseq *pwrseq)
{
	pwrseq_off(pwrseq);
	pwrseq_put(pwrseq);
}
EXPORT_SYMBOL_GPL(of_pwrseq_off);

int of_pwrseq_on_list(struct device_node *np, struct list_head *head)
{
	struct pwrseq *pwrseq;
	struct pwrseq_list_per_dev *pwrseq_list_node;

	pwrseq = of_pwrseq_on(np);
	if (IS_ERR(pwrseq))
		return PTR_ERR(pwrseq);

	pwrseq_list_node = kzalloc(sizeof(*pwrseq_list_node), GFP_KERNEL);
	if (!pwrseq_list_node) {
		of_pwrseq_off(pwrseq);
		return -ENOMEM;
	}
	pwrseq_list_node->pwrseq = pwrseq;
	list_add(&pwrseq_list_node->list, head);

	return 0;
}
EXPORT_SYMBOL_GPL(of_pwrseq_on_list);

void of_pwrseq_off_list(struct list_head *head)
{
	struct pwrseq *pwrseq;
	struct pwrseq_list_per_dev *pwrseq_list_node, *tmp_node;

	list_for_each_entry_safe(pwrseq_list_node, tmp_node, head, list) {
		pwrseq = pwrseq_list_node->pwrseq;
		of_pwrseq_off(pwrseq);
		list_del(&pwrseq_list_node->list);
		kfree(pwrseq_list_node);
	}
}
EXPORT_SYMBOL_GPL(of_pwrseq_off_list);
