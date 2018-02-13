/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * PRUSS Remote Processor specific types
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 */

#ifndef _PRU_REMOTEPROC_H_
#define _PRU_REMOTEPROC_H_

/**
 * enum pruss_rsc_types - PRU specific resource types
 *
 * @PRUSS_RSC_INTRS: Resource holding information on PRU PINTC configuration
 * @PRUSS_RSC_MAX: Indicates end of known/defined PRU resource types.
 *		   This should be the last definition.
 *
 * Introduce new custom resource types before PRUSS_RSC_MAX.
 */
enum pruss_rsc_types {
	PRUSS_RSC_INTRS	= 1,
	PRUSS_RSC_MAX	= 2,
};

/**
 * struct fw_rsc_pruss_intrmap - vendor resource to define PRU interrupts
 * @type: should be PRUSS_RSC_INTRS
 * @version: should be 1 or greater. 0 was for prototyping and is not supported
 * @num_maps: number of interrupt mappings that follow
 * @data: Array of 'num_maps' mappings.
 *		Each mapping is a triplet {s, c, h}
 *		s - system event id
 *		c - channel id
 *		h - host interrupt id
 *
 * PRU system events are mapped to channels, and these channels are mapped
 * to host interrupts. Events can be mapped to channels in a one-to-one or
 * many-to-one ratio (multiple events per channel), and channels can be
 * mapped to host interrupts in a one-to-one or many-to-one ratio (multiple
 * channels per interrupt).
 *
 * This resource is variable length due to the nature of INTC map.
 * The below data structure is scalable so it can support sufficiently
 * large number of sysevents and hosts.
 */
struct fw_rsc_pruss_intrmap {
	u16 type;
	u16 version;
	u8 num_maps;
	u8 data[];
} __packed;

#endif	/* _PRU_REMOTEPROC_H_ */
