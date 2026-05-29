/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2025 - Texas Instruments Incorporated
 *
 * Aradhya Bhatia <a-bhatia1@ti.com>
 */

#ifndef __TIDSS_OLDI_H__
#define __TIDSS_OLDI_H__

#include "tidss_drv.h"

struct tidss_oldi;

/* OLDI PORTS */
#define OLDI_INPUT_PORT		0
#define OLDI_OUTPUT_PORT	1

/* Control MMR Registers */

/* Register offsets */
#define OLDI_PD_CTRL            0x100

/* Power control bits */
#define OLDI_PWRDOWN_TX(n)	BIT(n)

/* LVDS Bandgap reference Enable/Disable */
#define OLDI_PWRDN_BG		BIT(8)

enum tidss_oldi_link_type {
	OLDI_MODE_UNSUPPORTED,
	OLDI_MODE_SINGLE_LINK,
	OLDI_MODE_CLONE_SINGLE_LINK,
	OLDI_MODE_SECONDARY_CLONE_SINGLE_LINK,
	OLDI_MODE_DUAL_LINK,
	OLDI_MODE_SECONDARY_DUAL_LINK,
};

int tidss_oldi_create_devices(struct tidss_device *tidss);
void tidss_oldi_destroy_devices(struct tidss_device *tidss);

int tidss_oldi_register_driver(void);
void tidss_oldi_unregister_driver(void);

#endif /* __TIDSS_OLDI_H__ */
