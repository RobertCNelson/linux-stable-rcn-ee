// SPDX-License-Identifier: GPL-2.0
/*
 * mikroBUS manifest definition
 * based on Greybus Manifest Definition
 *
 * Copyright 2014-2015 Google Inc.
 * Copyright 2014-2015 Linaro Ltd.
 *
 * Released under the GPLv2 and BSD licenses.
 */

#ifndef __MIKROBUS_MANIFEST_H
#define __MIKROBUS_MANIFEST_H

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/property.h>

#include "mikrobus_core.h"

#define MIKROBUS_VERSION_MAJOR 0
#define MIKROBUS_VERSION_MINOR 2

enum mikrobus_descriptor_type {
  MIKROBUS_TYPE_INVALID = 0x00,
  GREYBUS_TYPE_INTERFACE = 0x01,
  GREYBUS_TYPE_STRING = 0x02,
  GREYBUS_TYPE_BUNDLE = 0x03,
  GREYBUS_TYPE_CPORT = 0x04,
  MIKROBUS_TYPE_MIKROBUS = 0x05,
  MIKROBUS_TYPE_PROPERTY = 0x06,
  MIKROBUS_TYPE_DEVICE = 0x07,
};

struct mikrobus_descriptor_string {
  __u8 length;
  __u8 id;
  __u8 string[0];
} __packed;

struct mikrobus_descriptor_property {
  __u8 length;
  __u8 id;
  __u8 propname_stringid;
  __u8 type;
  __u8 value[0];
} __packed;

struct mikrobus_descriptor_device {
  __u8 id;
  __u8 driver_stringid;
  __u8 num_properties;
  __u8 protocol;
  __le32 max_speed_hz;
  __u8 reg;
  __u8 mode;
  __u8 num_gpio_resources;
  __u8 cs_gpio;
  __u8 irq;
  __u8 irq_type;
  __u8 prop_link;
  __u8 gpio_link;
} __packed;

struct mikrobus_descriptor_mikrobus {
  __u8 num_devices;
  __u8 rst_gpio_state;
  __u8 pwm_gpio_state;
  __u8 int_gpio_state;
} __packed;

struct greybus_descriptor_interface {
  __u8 vendor_stringid;
  __u8 product_stringid;
  __u8 features;
  __u8 pad;
} __packed;

struct greybus_descriptor_bundle {
  __u8 id;
  __u8 class;
  __u8 pad[2];
} __packed;

struct greybus_descriptor_cport {
  __le16 id;
  __u8 bundle;
  __u8 protocol_id;
} __packed;

struct mikrobus_descriptor_header {
  __le16 size;
  __u8 type;
  __u8 pad;
} __packed;

struct mikrobus_descriptor {
  struct mikrobus_descriptor_header header;
  union {
	struct mikrobus_descriptor_string string;
	struct mikrobus_descriptor_device device;
	struct mikrobus_descriptor_property property;
	struct mikrobus_descriptor_mikrobus mikrobus;
	struct greybus_descriptor_interface interface;
	struct greybus_descriptor_bundle bundle;
	struct greybus_descriptor_cport cport;
  };
} __packed;

struct mikrobus_manifest_header {
  __le16 size;
  __u8 version_major;
  __u8 version_minor;
} __packed;

struct mikrobus_manifest {
  struct mikrobus_manifest_header header;
  struct mikrobus_descriptor descriptors[0];
} __packed;

bool mikrobus_manifest_parse(struct click_board_info *info, void *data,
							 size_t size);
size_t mikrobus_manifest_header_validate(void *data, size_t size);

#endif /* __MIKROBUS_MANIFEST_H */
