/*
 * dt-con.h
 *
 * Copyright (c) 2016 Konsulko Group
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_EXTCON_DT_CON_H
#define __LINUX_EXTCON_DT_CON_H

#include <linux/device.h>
#include <linux/platform_device.h>

struct dtcon_proxy;

struct dtcon_pin {
	struct list_head node;
	struct device_node *np;
	char *regstr;	/* connector reg prop */

	/* owner of the pin */
	struct dtcon_proxy *proxy;
	struct list_head proxy_node;
	void *data;

	/* proxy device data */
	struct device_node *param;

	/* pinctrl config device node */
	bool proxy_gpio;
	struct device_node *pinctrl;
	phandle match_mux;
};

struct dtcon_data {
	struct platform_device *pdev;
	struct extcon_dev *edev;

	struct device_node *connector;
	unsigned int connector_address_cells;
	unsigned int connector_size_cells;

	struct device_node *functions;
	struct device_node *plugged;
	struct list_head pin_list;

	struct list_head function_list;
};

struct dtcon_function {
	struct dtcon_data *dtcd;
	struct list_head node;
	const char *kind;
	struct device_node *np;
	struct list_head proxy_list;
	void *data;
};

struct dtcon_proxy {
	struct platform_device *pdev;
	struct dtcon_function *dtcf;
	struct list_head node;

	struct list_head proxy_pin_list;

	void *data;
};

struct dtcon_data *dtcon_data_from_platform_device(
		struct platform_device *pdev);

struct dtcon_proxy *dtcon_proxy_create(struct platform_device *pdev,
		const char *kind,
		int (*func_init)(struct dtcon_function *dtcf));
void dtcon_proxy_destroy(struct dtcon_proxy *proxy,
		void (*func_fini)(struct dtcon_function *dtcf));

struct dtcon_pin *dtcon_pin_lookup(struct dtcon_data *dtcd,
		const void *regp, int regsz);
struct dtcon_pin *dtcon_pin_lookup_by_node(struct dtcon_data *dtcd,
		struct device_node *np);
struct dtcon_pin *dtcon_pin_lookup_by_phandle(struct dtcon_data *dtcd,
		phandle phandle);
struct dtcon_pin *dtcon_proxy_pin_request(struct dtcon_proxy *proxy,
		const void *regp, int regsz, unsigned int flags);
int dtcon_proxy_pin_release(struct dtcon_proxy *proxy, struct dtcon_pin *dtcp);


#endif /* __LINUX_EXTCON_DT_CON_H */
