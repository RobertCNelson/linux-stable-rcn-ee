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
 * along with this program.
 */

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/power/pwrseq.h>

static DEFINE_MUTEX(pwrseq_list_mutex);
static LIST_HEAD(pwrseq_list);

static int pwrseq_get(struct device_node *np, struct pwrseq *p)
{
	if (p && p->get)
		return p->get(np, p);

	return -ENOTSUPP;
}

static int pwrseq_on(struct pwrseq *p)
{
	if (p && p->on)
		return p->on(p);

	return -ENOTSUPP;
}

static void pwrseq_off(struct pwrseq *p)
{
	if (p && p->off)
		p->off(p);
}

static void pwrseq_put(struct pwrseq *p)
{
	if (p && p->put)
		p->put(p);
}

/**
 * pwrseq_register - Add pwrseq instance to global pwrseq list
 *
 * @pwrseq: the pwrseq instance
 */
void pwrseq_register(struct pwrseq *pwrseq)
{
	mutex_lock(&pwrseq_list_mutex);
	list_add(&pwrseq->node, &pwrseq_list);
	mutex_unlock(&pwrseq_list_mutex);
}
EXPORT_SYMBOL_GPL(pwrseq_register);

/**
 * pwrseq_unregister - Remove pwrseq instance from global pwrseq list
 *
 * @pwrseq: the pwrseq instance
 */
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

	mutex_lock(&pwrseq_list_mutex);
	list_for_each_entry(pwrseq, &pwrseq_list, node) {
		if (pwrseq->used)
			continue;

		/* compare compatible string for pwrseq node */
		if (of_match_node(pwrseq->pwrseq_of_match_table, np)) {
			pwrseq->used = true;
			mutex_unlock(&pwrseq_list_mutex);
			return pwrseq;
		}

		/* return generic pwrseq instance */
		if (!strcmp(pwrseq->pwrseq_of_match_table->compatible,
				"generic")) {
			pr_debug("using generic pwrseq instance for %s\n",
				np->full_name);
			pwrseq->used = true;
			mutex_unlock(&pwrseq_list_mutex);
			return pwrseq;
		}
	}
	mutex_unlock(&pwrseq_list_mutex);
	pr_debug("Can't find any pwrseq instances for %s\n", np->full_name);

	return NULL;
}

/**
 * of_pwrseq_on - Carry out power sequence on for device node
 *
 * @np: the device node would like to power on
 *
 * Carry out a single device power on.  If multiple devices
 * need to be handled, use of_pwrseq_on_list() instead.
 *
 * Return a pointer to the power sequence instance on success,
 * or an error code otherwise.
 */
struct pwrseq *of_pwrseq_on(struct device_node *np)
{
	struct pwrseq *pwrseq;
	int ret;

	pwrseq = pwrseq_find_available_instance(np);
	if (!pwrseq)
		return ERR_PTR(-ENOENT);

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

/**
 * of_pwrseq_off - Carry out power sequence off for this pwrseq instance
 *
 * @pwrseq: the pwrseq instance which related device would like to be off
 *
 * This API is used to power off single device, it is the opposite
 * operation for of_pwrseq_on.
 */
void of_pwrseq_off(struct pwrseq *pwrseq)
{
	pwrseq_off(pwrseq);
	pwrseq_put(pwrseq);
}
EXPORT_SYMBOL_GPL(of_pwrseq_off);

/**
 * of_pwrseq_on_list - Carry out power sequence on for list
 *
 * @np: the device node would like to power on
 * @head: the list head for pwrseq list on this bus
 *
 * This API is used to power on multiple devices at single bus.
 * If there are several devices on bus (eg, USB bus), uses this
 * this API. Otherwise, use of_pwrseq_on instead. After the device
 * is powered on successfully, it will be added to pwrseq list for
 * this bus. The caller needs to use mutex_lock for concurrent.
 *
 * Return 0 on success, or an error value otherwise.
 */
int of_pwrseq_on_list(struct device_node *np, struct list_head *head)
{
	struct pwrseq *pwrseq;
	struct pwrseq_list_per_dev *pwrseq_list_node;

	pwrseq_list_node = kzalloc(sizeof(*pwrseq_list_node), GFP_KERNEL);
	if (!pwrseq_list_node)
		return -ENOMEM;

	pwrseq = of_pwrseq_on(np);
	if (IS_ERR(pwrseq)) {
		kfree(pwrseq_list_node);
		return PTR_ERR(pwrseq);
	}

	pwrseq_list_node->pwrseq = pwrseq;
	list_add(&pwrseq_list_node->list, head);

	return 0;
}
EXPORT_SYMBOL_GPL(of_pwrseq_on_list);

/**
 * of_pwrseq_off_list - Carry out power sequence off for the list
 *
 * @head: the list head for pwrseq instance list on this bus
 *
 * This API is used to power off all devices on this bus, it is
 * the opposite operation for of_pwrseq_on_list.
 * The caller needs to use mutex_lock for concurrent.
 */
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

/**
 * pwrseq_suspend - Carry out power sequence suspend for this pwrseq instance
 *
 * @pwrseq: the pwrseq instance
 *
 * This API is used to do suspend operation on pwrseq instance.
 *
 * Return 0 on success, or an error value otherwise.
 */
int pwrseq_suspend(struct pwrseq *p)
{
	int ret = 0;

	if (p && p->suspend)
		ret = p->suspend(p);
	else
		return ret;

	if (!ret)
		p->suspended = true;
	else
		pr_err("%s failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_suspend);

/**
 * pwrseq_resume - Carry out power sequence resume for this pwrseq instance
 *
 * @pwrseq: the pwrseq instance
 *
 * This API is used to do resume operation on pwrseq instance.
 *
 * Return 0 on success, or an error value otherwise.
 */
int pwrseq_resume(struct pwrseq *p)
{
	int ret = 0;

	if (p && p->resume)
		ret = p->resume(p);
	else
		return ret;

	if (!ret)
		p->suspended = false;
	else
		pr_err("%s failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_resume);

/**
 * pwrseq_suspend_list - Carry out power sequence suspend for list
 *
 * @head: the list head for pwrseq instance list on this bus
 *
 * This API is used to do suspend on all power sequence instances on this bus.
 * The caller needs to use mutex_lock for concurrent.
 */
int pwrseq_suspend_list(struct list_head *head)
{
	struct pwrseq *pwrseq;
	struct pwrseq_list_per_dev *pwrseq_list_node;
	int ret = 0;

	list_for_each_entry(pwrseq_list_node, head, list) {
		ret = pwrseq_suspend(pwrseq_list_node->pwrseq);
		if (ret)
			break;
	}

	if (ret) {
		list_for_each_entry(pwrseq_list_node, head, list) {
			pwrseq = pwrseq_list_node->pwrseq;
			if (pwrseq->suspended)
				pwrseq_resume(pwrseq);
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_suspend_list);

/**
 * pwrseq_resume_list - Carry out power sequence resume for the list
 *
 * @head: the list head for pwrseq instance list on this bus
 *
 * This API is used to do resume on all power sequence instances on this bus.
 * The caller needs to use mutex_lock for concurrent.
 */
int pwrseq_resume_list(struct list_head *head)
{
	struct pwrseq_list_per_dev *pwrseq_list_node;
	int ret = 0;

	list_for_each_entry(pwrseq_list_node, head, list) {
		ret = pwrseq_resume(pwrseq_list_node->pwrseq);
		if (ret)
			break;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(pwrseq_resume_list);
