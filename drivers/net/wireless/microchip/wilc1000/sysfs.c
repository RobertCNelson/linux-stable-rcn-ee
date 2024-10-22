// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/kobject.h>
#include "cfg80211.h"

static struct kobject *wilc_kobj;
static int device_created;
static struct wilc *wl;

static ssize_t wilc_sysfs_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	int attr_val = -1;

	if (strcmp(attr->attr.name, "p2p_mode") == 0)
		attr_val = wl->attr_sysfs.p2p_mode;
	if (strcmp(attr->attr.name, "ant_swtch_mode") == 0)
		attr_val = wl->attr_sysfs.ant_swtch_mode;
	else if (strcmp(attr->attr.name, "antenna1") == 0)
		attr_val = wl->attr_sysfs.antenna1;
	else if (strcmp(attr->attr.name, "antenna2") == 0)
		attr_val = wl->attr_sysfs.antenna2;
	else if (strcmp(attr->attr.name, "fw_dbg_level") == 0)
		attr_val = wl->attr_sysfs.fw_dbg_level;
	else if (strcmp(attr->attr.name, "fw_dbg_mod_filter") == 0)
		attr_val = wl->attr_sysfs.fw_dbg_mod_filter;
	else if (strcmp(attr->attr.name, "coex_enabled") == 0)
		attr_val = wl->attr_sysfs.coex_enabled;
	else if (strcmp(attr->attr.name, "coex_inteface_type") == 0)
		attr_val = wl->attr_sysfs.coex_inteface_type;
	else if (strcmp(attr->attr.name, "coex_wlan_bt_priority") == 0)
		attr_val = wl->attr_sysfs.coex_wlan_bt_priority;
	else if (strcmp(attr->attr.name, "coex_antenna_mode") == 0)
		attr_val = wl->attr_sysfs.coex_antenna_mode;

	return sprintf(buf, "%d\n", attr_val);
}

static ssize_t wilc_sysfs_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	int attr_val;

	if (kstrtoint(buf, 10, &attr_val)) {
		pr_err("Failed to convert attribute string");
		return count;
	}

	if (wl->chip != WILC_S02) {
		if (strcmp(attr->attr.name, "p2p_mode") == 0) {
			wl->attr_sysfs.p2p_mode = (attr_val ? 1 : 0);
		} else if (strcmp(attr->attr.name, "ant_swtch_mode") == 0) {
			if (attr_val > ANT_SWTCH_DUAL_GPIO_CTRL)
				pr_err("Valid antenna switch modes:\n1-Single Antenna, 2-Dual Antenna\n");
			else
				wl->attr_sysfs.ant_swtch_mode = attr_val;
		} else if (strcmp(attr->attr.name, "antenna1") == 0) {
			wl->attr_sysfs.antenna1 = attr_val;
		} else if (strcmp(attr->attr.name, "antenna2") == 0) {
			wl->attr_sysfs.antenna2 = attr_val;
		}
	} else {
		if (strcmp(attr->attr.name, "fw_dbg_level") == 0) {
			if (attr_val < WILC_FW_PRINT_LVL_ERROR || attr_val > WILC_FW_PRINT_LVL_MAX) {
				pr_err("valid fw debug levels:\n 1-WILC_FW_PRINT_LVL_ERROR,\n 2-WILC_FW_PRINT_LVL_DEBUG\n 3-WILC_FW_PRINT_LVL_INFO,"
					"\n 4-WILC_FW_PRINT_LVL_FUN_PT\n 5-WILC_FW_PRINT_LVL_MAX\n");
			} else {
				wl->attr_sysfs.fw_dbg_level = attr_val;
				if (wl->initialized)
					wilc_set_fw_debug_level(wl);
			}
		} else if (strcmp(attr->attr.name, "fw_dbg_mod_filter") == 0) {
			wl->attr_sysfs.fw_dbg_mod_filter = attr_val;
			if (wl->initialized)
				wilc_set_fw_debug_level(wl);
		} else if (strcmp(attr->attr.name, "coex_enabled") == 0) {
			if ((attr_val & ~COEX_ENABLED_BIT) != 0)
				pr_err("Valid coex enable values:\n"
						"0-Disabled\n"
						"1-Enabled\n");
			else if (wl->attr_sysfs.coex_enabled != attr_val) {
				wl->attr_sysfs.coex_enabled = attr_val;
				wilc_set_coex_param(wl, WID_COEX_ENABLE,
						    wl->attr_sysfs.coex_enabled);
			}
		} else if (strcmp(attr->attr.name, "coex_inteface_type") == 0) {
			if ((attr_val & ~COEX_INTERFACE_TYPE_BIT) != 0)
				pr_err("Valid coex interface types:\n"
						"0- 3-wire interface (BT_Act, BT_Prio, WLAN_Act)\n"
						"1- 2-wire interface (BT_Prio, WLAN_Act)\n");
			else if (wl->attr_sysfs.coex_inteface_type != attr_val) {
				wl->attr_sysfs.coex_inteface_type = attr_val;
				wilc_set_coex_param(wl,
						    WID_COEX_INTERFACE_TYPE, wl->attr_sysfs.coex_inteface_type);
			}
		} else if (strcmp(attr->attr.name, "coex_wlan_bt_priority") == 0) {
			if ((attr_val & ~COEX_WLAN_BT_PRIORITY_BIT) != 0)
				pr_err("Valid coex wlan bt priority bit positions:\n"
						"Bit0 - WLAN Tx priority higher(1)/lower(0) than BT Low Priority\n"
						"Bit1 - WLAN Rx priority higher(1)/lower(0) than BT Low Priority\n");
			else if (wl->attr_sysfs.coex_wlan_bt_priority != attr_val) {
				wl->attr_sysfs.coex_wlan_bt_priority = attr_val;
				wilc_set_coex_param(wl, WID_COEX_PRIORITY_FLAGS,
						    wl->attr_sysfs.coex_wlan_bt_priority);
				}
		} else if (strcmp(attr->attr.name, "coex_antenna_mode") == 0) {
			if ((attr_val & ~COEX_ANTENNA_MODE_BIT) != 0)
				pr_err("Valid coex antenna modes:\n"
						"0-Dedicated Antenna\n"
						"1-Shared Antenna\n");
			else if (wl->attr_sysfs.coex_antenna_mode != attr_val) {
				wl->attr_sysfs.coex_antenna_mode = attr_val;
				wilc_set_coex_param(wl, WID_COEX_ANTENNA_MODE,
						    wl->attr_sysfs.coex_antenna_mode);
			}
		}
	}

	return count;
}

static struct kobj_attribute p2p_mode_attr =
	__ATTR(p2p_mode, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute ant_swtch_mode_attr =
	__ATTR(ant_swtch_mode, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute ant_swtch_antenna1_attr =
	__ATTR(antenna1, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute ant_swtch_antenna2_attr =
	__ATTR(antenna2, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute fw_dbg_level_attr =
	__ATTR(fw_dbg_level, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute fw_dbg_mod_filter_attr =
	__ATTR(fw_dbg_mod_filter, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute coex_enabled_attr =
	__ATTR(coex_enabled, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute coex_inteface_type_attr =
	__ATTR(coex_inteface_type, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute coex_wlan_bt_priority_attr =
	__ATTR(coex_wlan_bt_priority, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct kobj_attribute coex_antenna_mode_attr =
	__ATTR(coex_antenna_mode, 0664, wilc_sysfs_show, wilc_sysfs_store);

static struct attribute *wilc_attrs[] = {
	&p2p_mode_attr.attr,
	&ant_swtch_mode_attr.attr,
	&ant_swtch_antenna1_attr.attr,
	&ant_swtch_antenna2_attr.attr,
	&fw_dbg_level_attr.attr,
	&fw_dbg_mod_filter_attr.attr,
	&coex_enabled_attr.attr,
	&coex_inteface_type_attr.attr,
	&coex_wlan_bt_priority_attr.attr,
	&coex_antenna_mode_attr.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = wilc_attrs,
};

void wilc_sysfs_init(struct wilc *wilc)
{
	int retval;

	if (device_created)
		return;

	wilc_kobj = kobject_create_and_add("wilc", NULL);
	if (!wilc_kobj) {
		retval = -ENOMEM;
		return;
	}

	retval = sysfs_create_group(wilc_kobj, &attr_group);
	device_created = 1;
	wl = wilc;
	/* By default p2p mode is Group Owner */
	wl->attr_sysfs.p2p_mode = WILC_P2P_ROLE_GO;
	wl->attr_sysfs.ant_swtch_mode = ANT_SWTCH_INVALID_GPIO_CTRL;
	wl->attr_sysfs.antenna1 = 0xFF;
	wl->attr_sysfs.antenna2 = 0xFF;
	wl->attr_sysfs.fw_dbg_level = WILC_FW_PRINT_LVL_ERROR;
	wl->attr_sysfs.fw_dbg_mod_filter = DEFAULT_FW_DBG_MOD_LEVEL;
	/* BT wifi coex params */
	wl->attr_sysfs.coex_enabled = WLAN_BT_COEX_DISABLED;
	wl->attr_sysfs.coex_inteface_type = WLAN_BT_COEX_3WIRE_IF;
	wl->attr_sysfs.coex_wlan_bt_priority = WLAN_BT_COEX_PRIO_WLAN_TX_GT_BTLP | WLAN_BT_COEX_PRIO_WLAN_RX_GT_BTLP;
	wl->attr_sysfs.coex_antenna_mode = WLAN_BT_COEX_ANT_MODE_DEDICATED;
}

void wilc_sysfs_exit(void)
{
	device_created = 0;
	sysfs_remove_group(wilc_kobj, &attr_group);
	kobject_put(wilc_kobj);
}
