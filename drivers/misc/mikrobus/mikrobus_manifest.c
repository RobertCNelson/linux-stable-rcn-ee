// SPDX-License-Identifier: GPL-2.0
/*
 * mikroBUS manifest parsing
 * based on Greybus Manifest Parsing
 *
 * Copyright 2014-2015 Google Inc.
 * Copyright 2014-2015 Linaro Ltd.
 */

#define pr_fmt(fmt) "mikrobus_manifest: " fmt

#include <linux/bits.h>
#include <linux/types.h>
#include <linux/property.h>

#include "mikrobus_manifest.h"

struct manifest_desc {
  struct list_head links;
  size_t size;
  void *data;
  enum mikrobus_descriptor_type type;
};

static void release_manifest_descriptor(struct manifest_desc *descriptor)
{
  list_del(&descriptor->links);
  kfree(descriptor);
}

static void release_manifest_descriptors(struct click_board_info *info)
{
  struct manifest_desc *descriptor;
  struct manifest_desc *next;

  list_for_each_entry_safe(descriptor, next, &info->manifest_descs, links)
	  release_manifest_descriptor(descriptor);
}

static int identify_descriptor(struct click_board_info *info,
							   struct mikrobus_descriptor *desc, size_t size)
{
  struct mikrobus_descriptor_header *desc_header = &desc->header;
  struct manifest_desc *descriptor;
  size_t desc_size;
  size_t expected_size;

  if (size < sizeof(*desc_header)) {
	return -EINVAL;
  }

  desc_size = le16_to_cpu(desc_header->size);
  if (desc_size > size) {
	return -EINVAL;
  }

  expected_size = sizeof(*desc_header);

  switch (desc_header->type) {

  case GREYBUS_TYPE_STRING:
	expected_size += sizeof(struct mikrobus_descriptor_string);
	expected_size += desc->string.length;
	expected_size = ALIGN(expected_size, 4);
	break;
  case MIKROBUS_TYPE_PROPERTY:
	expected_size += sizeof(struct mikrobus_descriptor_property);
	expected_size += desc->property.length;
	expected_size = ALIGN(expected_size, 4);
	break;
  case MIKROBUS_TYPE_DEVICE:
	expected_size += sizeof(struct mikrobus_descriptor_device);
	break;
  case MIKROBUS_TYPE_MIKROBUS:
	expected_size += sizeof(struct mikrobus_descriptor_mikrobus);
	break;
  case GREYBUS_TYPE_INTERFACE:
	expected_size += sizeof(struct greybus_descriptor_interface);
	break;
  case GREYBUS_TYPE_CPORT:
	expected_size += sizeof(struct greybus_descriptor_cport);
	break;
  case GREYBUS_TYPE_BUNDLE:
	expected_size += sizeof(struct greybus_descriptor_bundle);
	break;
  case MIKROBUS_TYPE_INVALID:
  default:
	return -EINVAL;
  }

  descriptor = kzalloc(sizeof(*descriptor), GFP_KERNEL);
  if (!descriptor)
	return -ENOMEM;

  descriptor->size = desc_size;
  descriptor->data = (char *)desc + sizeof(*desc_header);
  descriptor->type = desc_header->type;
  list_add_tail(&descriptor->links, &info->manifest_descs);

  return desc_size;
}

static char *mikrobus_string_get(struct click_board_info *info, u8 string_id)
{
  struct mikrobus_descriptor_string *desc_string;
  struct manifest_desc *descriptor;
  bool found = false;
  char *string;

  if (!string_id)
	return NULL;

  list_for_each_entry(descriptor, &info->manifest_descs, links) {
	if (descriptor->type != GREYBUS_TYPE_STRING)
	  continue;

	desc_string = descriptor->data;
	if (desc_string->id == string_id) {
	  found = true;
	  break;
	}
  }
  if (!found)
	return ERR_PTR(-ENOENT);

  string = kmemdup(&desc_string->string, desc_string->length + 1, GFP_KERNEL);
  if (!string)
	return ERR_PTR(-ENOMEM);
  string[desc_string->length] = '\0';

  return string;
}

static void mikrobus_state_get(struct click_board_info *info)
{
  struct mikrobus_descriptor_mikrobus *mikrobus;
  struct greybus_descriptor_interface *interface;
  struct manifest_desc *descriptor;
  bool found = false;

  list_for_each_entry(descriptor, &info->manifest_descs, links) {
	if (descriptor->type == MIKROBUS_TYPE_MIKROBUS) {
	  mikrobus = descriptor->data;
	  found = true;
	  break;
	}
  }

  if (found) {
	info->num_devices = mikrobus->num_devices;
	info->rst_gpio_state = mikrobus->rst_gpio_state;
	info->pwm_gpio_state = mikrobus->pwm_gpio_state;
	info->int_gpio_state = mikrobus->int_gpio_state;
  } else {
	info->num_devices = 1;
	info->rst_gpio_state = MIKROBUS_GPIO_UNUSED;
	info->pwm_gpio_state = MIKROBUS_GPIO_UNUSED;
	info->int_gpio_state = MIKROBUS_GPIO_UNUSED;
  }

  list_for_each_entry(descriptor, &info->manifest_descs, links) {
	if (descriptor->type == GREYBUS_TYPE_INTERFACE) {
	  interface = descriptor->data;
	  break;
	}
  }
  info->name = mikrobus_string_get(info, interface->product_stringid);
}

static struct property_entry *
mikrobus_property_entry_get(struct click_board_info *info, u8 *prop_link,
							int num_properties)
{
  struct mikrobus_descriptor_property *desc_property;
  struct manifest_desc *descriptor;
  struct property_entry *properties;
  int i;
  char *prop_name;
  bool found = false;
  u8 *val_u8;
  u16 *val_u16;
  u32 *val_u32;
  u64 *val_u64;

  properties = kzalloc(sizeof(*properties) * num_properties, GFP_KERNEL);
  if (!properties)
	return ERR_PTR(-ENOMEM);

  for (i = 0; i < num_properties; i++) {

	list_for_each_entry(descriptor, &info->manifest_descs, links) {
	  if (descriptor->type != MIKROBUS_TYPE_PROPERTY)
		continue;

	  desc_property = descriptor->data;
	  if (desc_property->id == prop_link[i]) {
		found = true;
		break;
	  }
	}
	if (!found)
	  return ERR_PTR(-ENOENT);

	prop_name = mikrobus_string_get(info, desc_property->propname_stringid);

	switch (desc_property->type) {

	case MIKROBUS_PROPERTY_TYPE_U8:
	  val_u8 = kmemdup(&desc_property->value,
					   (desc_property->length) * sizeof(u8), GFP_KERNEL);
	  if (desc_property->length == 1)
		properties[i] = PROPERTY_ENTRY_U8(prop_name, *val_u8);
	  else
		properties[i] = PROPERTY_ENTRY_U8_ARRAY_LEN(
			prop_name, (void *)desc_property->value, desc_property->length);
	  break;
	case MIKROBUS_PROPERTY_TYPE_U16:
	  val_u16 = kmemdup(&desc_property->value,
						(desc_property->length) * sizeof(u16), GFP_KERNEL);
	  if (desc_property->length == 1)
		properties[i] = PROPERTY_ENTRY_U16(prop_name, *val_u16);
	  else
		properties[i] = PROPERTY_ENTRY_U16_ARRAY_LEN(
			prop_name, (void *)desc_property->value, desc_property->length);
	  break;
	case MIKROBUS_PROPERTY_TYPE_U32:
	  val_u32 = kmemdup(&desc_property->value,
						(desc_property->length) * sizeof(u32), GFP_KERNEL);
	  if (desc_property->length == 1)
		properties[i] = PROPERTY_ENTRY_U32(prop_name, *val_u32);
	  else
		properties[i] = PROPERTY_ENTRY_U32_ARRAY_LEN(
			prop_name, (void *)desc_property->value, desc_property->length);
	  break;
	case MIKROBUS_PROPERTY_TYPE_U64:
	  val_u64 = kmemdup(&desc_property->value,
						(desc_property->length) * sizeof(u64), GFP_KERNEL);
	  if (desc_property->length == 1)
		properties[i] = PROPERTY_ENTRY_U64(prop_name, *val_u64);
	  else
		properties[i] = PROPERTY_ENTRY_U64_ARRAY_LEN(
			prop_name, (void *)desc_property->value, desc_property->length);
	  break;
	default:
	  return ERR_PTR(-EINVAL);
	}
  }

  return properties;
}

static u8 *mikrobus_property_link_get(struct click_board_info *info, u8 prop_id,
									  u8 prop_type)
{
  struct mikrobus_descriptor_property *desc_property;
  struct manifest_desc *descriptor;
  bool found = false;
  u8 *val_u8;

  if (!prop_id)
	return NULL;

  list_for_each_entry(descriptor, &info->manifest_descs, links) {
	if (descriptor->type != MIKROBUS_TYPE_PROPERTY)
	  continue;

	desc_property = descriptor->data;
	if (desc_property->id == prop_id && desc_property->type == prop_type) {
	  found = true;
	  break;
	}
  }
  if (!found)
	return ERR_PTR(-ENOENT);

  val_u8 = kmemdup(&desc_property->value, desc_property->length, GFP_KERNEL);

  return val_u8;
}

static int
mikrobus_manifest_attach_device(struct click_board_info *info,
								struct mikrobus_descriptor_device *dev_desc)
{
  struct click_device_info *dev;
  struct gpiod_lookup_table *lookup;
  struct mikrobus_descriptor_property *desc_property;
  struct manifest_desc *descriptor;
  int i;
  u8 *prop_link;
  u8 *gpio_desc_link;

  dev = kzalloc(sizeof(*dev), GFP_KERNEL);
  if (!dev) {
	return -ENOMEM;
  }

  dev->id = dev_desc->id;
  dev->drv_name = mikrobus_string_get(info, dev_desc->driver_stringid);
  dev->protocol = dev_desc->protocol;
  dev->reg = dev_desc->reg;
  dev->irq = dev_desc->irq;
  dev->irq_type = dev_desc->irq_type;
  dev->max_speed_hz = le32_to_cpu(dev_desc->max_speed_hz);
  dev->mode = dev_desc->mode;
  dev->cs_gpio = dev_desc->cs_gpio;
  dev->num_gpio_resources = dev_desc->num_gpio_resources;
  dev->num_properties = dev_desc->num_properties;

  pr_info(
	  "device %d , number of properties=%d , number of gpio resources=%d \n",
	  dev->id, dev->num_properties, dev->num_gpio_resources);

  if (dev->num_properties > 0) {
	prop_link = mikrobus_property_link_get(info, dev_desc->prop_link,
										   MIKROBUS_PROPERTY_TYPE_LINK);
	dev->properties =
		mikrobus_property_entry_get(info, prop_link, dev->num_properties);
  }

  if (dev->num_gpio_resources > 0) {

	lookup = kzalloc(struct_size(lookup, table, dev->num_gpio_resources),
					 GFP_KERNEL);
	if (!lookup)
	  return -ENOMEM;

	gpio_desc_link = mikrobus_property_link_get(info, dev_desc->gpio_link,
												MIKROBUS_PROPERTY_TYPE_GPIO);
	for (i = 0; i < dev->num_gpio_resources; i++) {
	  list_for_each_entry(descriptor, &info->manifest_descs, links) {
		if (descriptor->type != MIKROBUS_TYPE_PROPERTY)
		  continue;

		desc_property = descriptor->data;
		if (desc_property->id == gpio_desc_link[i]) {
		  lookup->table[i].chip_hwnum = *desc_property->value;
		  lookup->table[i].con_id =
			  mikrobus_string_get(info, desc_property->propname_stringid);
		  break;
		}
	  }
	}
	dev->gpio_lookup = lookup;
  }

  list_add_tail(&dev->links, &info->devices);

  return 0;
}

static int mikrobus_manifest_parse_devices(struct click_board_info *info)
{
  struct mikrobus_descriptor_device *desc_device;
  struct manifest_desc *desc, *next;
  int devcount = 0;

  if (WARN_ON(!list_empty(&info->devices)))
	return false;

  list_for_each_entry_safe(desc, next, &info->manifest_descs, links) {
	if (desc->type != MIKROBUS_TYPE_DEVICE)
	  continue;
	desc_device = desc->data;
	mikrobus_manifest_attach_device(info, desc_device);
	devcount++;
  }

  return devcount;
}

bool mikrobus_manifest_parse(struct click_board_info *info, void *data,
							 size_t size)
{
  struct mikrobus_manifest *manifest;
  struct mikrobus_manifest_header *header;
  struct mikrobus_descriptor *desc;
  u16 manifest_size;
  int dev_count;

  if (WARN_ON(!list_empty(&info->manifest_descs)))
	return false;

  if (size < sizeof(*header))
	return false;

  manifest = data;
  header = &manifest->header;
  manifest_size = le16_to_cpu(header->size);

  if (manifest_size != size)
	return false;

  if (header->version_major > MIKROBUS_VERSION_MAJOR) {
	return false;
  }

  desc = manifest->descriptors;
  size -= sizeof(*header);
  while (size) {
	int desc_size;

	desc_size = identify_descriptor(info, desc, size);
	if (desc_size < 0) {
	  pr_err("invalid Manifest Descriptor");
	  return -EINVAL;
	}
	desc = (struct mikrobus_descriptor *)((char *)desc + desc_size);
	size -= desc_size;
  }

  mikrobus_state_get(info);
  dev_count = mikrobus_manifest_parse_devices(info);

  pr_info(" %s manifest parsed with %d device(s) \n", info->name,
		  info->num_devices);

  release_manifest_descriptors(info);

  return true;
}
EXPORT_SYMBOL_GPL(mikrobus_manifest_parse);

size_t mikrobus_manifest_header_validate(void *data, size_t size)
{
  struct mikrobus_manifest_header *header;
  u16 manifest_size;

  if (size < sizeof(*header))
	return 0;

  header = data;
  manifest_size = le16_to_cpu(header->size);

  if (header->version_major > MIKROBUS_VERSION_MAJOR) {
	return 0;
  }

  return manifest_size;
}
EXPORT_SYMBOL_GPL(mikrobus_manifest_header_validate);
