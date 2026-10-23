// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2013 Texas Instruments Inc.
 */

#include "acx.h"
#include "sysfs.h"
#include <linux/kstrtox.h>
#include "rx.h"


static ssize_t cc33xx_sysfs_read_fwlog(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buffer, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cc33xx *wl = dev_get_drvdata(dev);
	ssize_t len;
	int ret;

	ret = mutex_lock_interruptible(&wl->mutex);
	if (ret < 0)
		return -ERESTARTSYS;

	/* Check if the fwlog is still valid */
	if (wl->fwlog_size < 0) {
		mutex_unlock(&wl->mutex);
		return 0;
	}

	/* Seeking is not supported - old logs are not kept. Disregard pos. */
	len = min_t(size_t, count, wl->fwlog_size);
	wl->fwlog_size -= len;
	memcpy(buffer, wl->fwlog, len);

	/* Make room for new messages */
	memmove(wl->fwlog, wl->fwlog + len, wl->fwlog_size);

	mutex_unlock(&wl->mutex);

	return len;
}

static const struct bin_attribute fwlog_attr = {
	.attr = { .name = "fwlog", .mode = 0400 },
	.read = cc33xx_sysfs_read_fwlog,
};


static ssize_t regdomain_txControl_param_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return 0;
}



static ssize_t regdomain_txControl_param_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	struct acx_phy_regdomain_tx_control_params params;
	int ret;
	char* buffer;
	char * pToken;
	int converted_token = 0;

	buffer = kzalloc(count, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		cc33xx_warning("error in regdomain and tx control params set (memory): %d", ret);
		return ret;
	}

	strncpy(buffer, buf, count);
	if(-EFAULT == ret){
		cc33xx_warning("error in regdomain and tx control params set: %d", ret);
		kfree(buffer);
		return ret;
	}

	pToken = strsep(&buffer, " ");

	ret = kstrtoint(pToken, 10, &converted_token);
	if (ret < 0)
	{
		ret = -EINVAL;
		kfree(buffer);
		cc33xx_warning("error in bitmask value parsing");
		return ret;
    }

	params.bitmask = converted_token;


	pToken = strsep(&buffer, " ");

	ret = kstrtoint(pToken, 10, &converted_token);
	if (ret < 0)
	{
		ret = -EINVAL;
		kfree(buffer);
		cc33xx_warning("error in reg_domain value parsing");
		return ret;
    }

	params.reg_domain = converted_token;

	pToken = strsep(&buffer, " ");

	for(int i = 0; i < BLE_LIM_CHANNELS_COUNT ; i++)
	{
		ret = kstrtoint(pToken, 10, &converted_token);
		if (ret < 0)
		{
			ret = -EINVAL;
			kfree(buffer);
			cc33xx_warning("error in ble_ch_lim_1M value parsing");
			return ret;
		}

		params.ble_ch_lim_1M[i] = converted_token;

		pToken = strsep(&buffer, " ");
	}

	for(int i = 0; i < BLE_LIM_CHANNELS_COUNT ;i++)
	{

		ret = kstrtoint(pToken, 10, &converted_token);

		if (ret < 0)
		{
			ret = -EINVAL;
			kfree(buffer);
			cc33xx_warning("error in ble_ch_lim_2M value parsing");
			return ret;
		}

		params.ble_ch_lim_2M[i] = converted_token;

		pToken = strsep(&buffer, " ");
	}

	ret = kstrtoint(pToken, 10, &converted_token);
	if (ret < 0)
	{
		ret = -EINVAL;
		kfree(buffer);
		cc33xx_warning("error in country_code value parsing");
		return ret;
	}

	params.country_code = converted_token;

	pToken = strsep(&buffer, " ");

	for(int i = 0; i < REG_RULES_COUNT ; i++)
	{
		ret = kstrtoint(pToken, 10, &converted_token);
		if (ret < 0)
		{
			ret = -EINVAL;
			kfree(buffer);
			cc33xx_warning("error in country_code value parsing");
			return ret;
		}

		params.per_channel_power_limit[i] = converted_token;

		pToken = strsep(&buffer, " ");
	}

	kfree(buffer);

	ret = cc33xx_acx_set_regdoamin_and_tx_control_params(cc,&params);


	mutex_lock(&cc->mutex);

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		goto out;
	}



out:
	mutex_unlock(&cc->mutex);
	return count;
}

static DEVICE_ATTR_RW(regdomain_txControl_param);

static ssize_t wowlan_pattern_clear_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct cc33xx *wl = dev_get_drvdata(dev);
	int ret;
	unsigned int clear_flag;

	ret = kstrtouint(buf, 10, &clear_flag);
	if (ret < 0)
		return ret;

	if (clear_flag != 1) {
		cc33xx_error("Invalid value %u: must be 1 to clear patterns", clear_flag);
		return -EINVAL;
	}


	ret = mutex_lock_interruptible(&wl->mutex);
	if (ret < 0)
	return -ERESTARTSYS;

	if (unlikely(wl->state != CC33XX_STATE_ON)) {
		cc33xx_error("Device not ready for pattern clear");
		ret = -EAGAIN;
		goto out_unlock;
	}

	ret = cc33xx_clear_wowlan_search_patterns(wl);
	if (ret < 0)
		goto out_unlock;

	mutex_unlock(&wl->mutex);
	return count;

out_unlock:
	mutex_unlock(&wl->mutex);
	return ret;
}

static ssize_t wowlan_pattern_clear_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "Write 1 to clear all WoWLAN patterns\n");
}

static DEVICE_ATTR_RW(wowlan_pattern_clear);

static ssize_t wowlan_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cc33xx *wl = dev_get_drvdata(dev);
	unsigned int enable;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret < 0)
		return ret;

	if (enable != 0 && enable != 1) {
		cc33xx_error("Invalid value %u: must be 0 (disable) or 1 (enable)", enable);
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&wl->mutex);
	if (ret < 0)
	return -ERESTARTSYS;

	if (unlikely(wl->state != CC33XX_STATE_ON)) {
		cc33xx_error("Device not ready (state: %d)", wl->state);
		ret = -EAGAIN;
		goto out;
	}

	ret = cc33xx_set_wowlan_search_mode(wl, enable != 0);
	if (ret < 0)
		goto out;

	mutex_unlock(&wl->mutex);
	return count;

out:
	mutex_unlock(&wl->mutex);
	return ret;
}

static ssize_t wowlan_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct cc33xx *wl = dev_get_drvdata(dev);
	ssize_t len;

	if (!wl)
		return sysfs_emit(buf, "Device not ready\n");

	len = sysfs_emit(buf, "Current mode: %s\n\n",
			 wl->wowlan_search.enabled ? "ENABLED" : "DISABLED");
	len += sysfs_emit_at(buf, len, "Usage:\n");
	len += sysfs_emit_at(buf, len, "  echo 1 > wowlan_mode    # Enable WoWLAN\n");
	len += sysfs_emit_at(buf, len, "  echo 0 > wowlan_mode    # Disable WoWLAN\n");

	return len;
}

static DEVICE_ATTR_RW(wowlan_mode);

static ssize_t wowlan_pattern_search_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct cc33xx *wl = dev_get_drvdata(dev);
	char *input, *pattern_str, *offset_str, *plus_sign;
	u8 pattern_data[CC33XX_RX_FILTER_MAX_PATTERN_SIZE];
	u8 mask_data[CC33XX_RX_FILTER_MAX_PATTERN_SIZE];
	int pattern_len;
	int ret;
	u16 starting_offset = 0;
	bool has_mask = false;

	if (unlikely(wl->state != CC33XX_STATE_ON)) {
		cc33xx_error("Device not ready for adding pattern");
		return -EAGAIN;
	}

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	memcpy(input, buf, count);
	input[count] = '\0';

	if (count > 0 && input[count - 1] == '\n')
		input[count - 1] = '\0';

	plus_sign = strchr(input, '+');
	if (plus_sign) {
		*plus_sign = '\0';
		offset_str = input;
		pattern_str = plus_sign + 1;

		ret = kstrtou16(offset_str, 10, &starting_offset);
		if (ret < 0) {
			cc33xx_error("Invalid offset '%s': must be 0-%d",
				     offset_str, CC33XX_RX_FILTER_MAX_PATTERN_SIZE - 1);
			ret = -EINVAL;
			goto out_free;
		}

		if (starting_offset >= CC33XX_RX_FILTER_MAX_PATTERN_SIZE) {
			cc33xx_error("Offset %d too large (max %d)",
				     starting_offset, CC33XX_RX_FILTER_MAX_PATTERN_SIZE - 1);
			ret = -EINVAL;
			goto out_free;
		}
	} else {
		pattern_str = input;
		starting_offset = 0;
	}

	if (strlen(pattern_str) == 0) {
		ret = -EINVAL;
		goto out_free;
	}

	pattern_len = cc33xx_parse_wowlan_search_pattern(pattern_str, pattern_data,
							  mask_data,
							  CC33XX_RX_FILTER_MAX_PATTERN_SIZE,
							  &has_mask);
	if (pattern_len < 0) {
		cc33xx_error("Invalid pattern format: %s", pattern_str);
		ret = pattern_len;
		goto out_free;
	}

	if (has_mask) {
		if (pattern_len * 2 > CC33XX_RX_FILTER_MAX_PATTERN_SIZE) {
			cc33xx_error("Pattern+mask too large (%d bytes)",
				     pattern_len * 2);
			ret = -EINVAL;
			goto out_free;
		}

		memcpy(pattern_data + pattern_len, mask_data, pattern_len);
	}

	ret = mutex_lock_interruptible(&wl->mutex);
	if (ret < 0) {
		ret = -ERESTARTSYS;
		goto out_free;
	}

	ret = cc33xx_add_wowlan_search_pattern(wl, starting_offset,
					       pattern_data, pattern_len,
					       has_mask);
	mutex_unlock(&wl->mutex);

	if (ret < 0)
		goto out_free;

	kfree(input);
	return count;

out_free:
	kfree(input);
	return ret;
}

static ssize_t wowlan_pattern_search_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len;
	struct cc33xx_rx_filter *filter;
	struct cc33xx_rx_filter_field *field;
	struct cc33xx *wl = dev_get_drvdata(dev);
	int i, j, k;
	u16 offset;
	bool has_mask;

	if (!wl)
		return sysfs_emit(buf, "Device not ready\n");

	if (mutex_lock_interruptible(&wl->mutex))
		return -ERESTARTSYS;

	len = sysfs_emit(buf, "Active filters: %d/%d\n",
			 wl->wowlan_search.filter_count, CC33XX_MAX_RX_FILTERS);

	if (wl->wowlan_search.filter_count > 0) {
		len += sysfs_emit_at(buf, len, "\n");
		for (i = 0; i < wl->wowlan_search.filter_count; i++) {
			filter = wl->wowlan_search.active_filters[i];
			if (!filter || filter->num_fields == 0)
				continue;

			for (j = 0; j < filter->num_fields; j++) {
				field = &filter->fields[j];
				offset = le16_to_cpu(field->offset);
				has_mask = field->flags & CC33XX_RX_FILTER_FLAG_MASKED;

				len += sysfs_emit_at(buf, len, "  [%d] offset=%d: ", i, offset);

				for (k = 0; k < field->len; k++) {
					len += sysfs_emit_at(buf, len, "%02x%s",
							     field->pattern[k],
							     k < field->len - 1 ? ":" : "");
				}

				if (has_mask)
					len += sysfs_emit_at(buf, len, " (masked)");

				len += sysfs_emit_at(buf, len, "\n");
			}
		}
	}

	mutex_unlock(&wl->mutex);

	len += sysfs_emit_at(buf, len, "\nUsage:\n");
	len += sysfs_emit_at(buf, len, "  echo \"<hex_pattern>\" > wowlan_pattern_search\n");
	len += sysfs_emit_at(buf, len, "  echo \"<payload_offset>+<hex_pattern>\" > wowlan_pattern_search\n");
	len += sysfs_emit_at(buf, len, "\nIMPORTANT: Searches PAYLOAD ONLY (header is skipped).\n");
	len += sysfs_emit_at(buf, len, "Offset 0 = first payload byte.\n");
	len += sysfs_emit_at(buf, len, "\nLimits:\n");
	len += sysfs_emit_at(buf, len, "  Max filters: %d\n", CC33XX_MAX_RX_FILTERS);
	len += sysfs_emit_at(buf, len, "  Max pattern size: %d bytes\n",
			     CC33XX_RX_FILTER_MAX_PATTERN_SIZE);
	len += sysfs_emit_at(buf, len, "\nTo clear: echo 1 > wowlan_pattern_clear\n");

	return len;
}

static DEVICE_ATTR_RW(wowlan_pattern_search);


int wlcore_sysfs_init(struct cc33xx *wl)
{
	int ret;


	ret = device_create_file(wl->dev, &dev_attr_regdomain_txControl_param);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file regdomain_txControl_param");

	ret = device_create_file(wl->dev, &dev_attr_wowlan_pattern_clear);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_pattern_clear");

	ret = device_create_file(wl->dev, &dev_attr_wowlan_pattern_search);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_pattern_search");

	ret = device_create_file(wl->dev, &dev_attr_wowlan_mode);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_mode");

	/* Create sysfs file for the FW log */
	ret = device_create_bin_file(wl->dev, &fwlog_attr);
	if (ret < 0) {
		cc33xx_error("failed to create sysfs file fwlog");
	}

	return ret;
}

void wlcore_sysfs_free(struct cc33xx *wl)
{
	device_remove_bin_file(wl->dev, &fwlog_attr);
	device_remove_file(wl->dev, &dev_attr_regdomain_txControl_param);
	device_remove_file(wl->dev, &dev_attr_wowlan_mode);
	device_remove_file(wl->dev, &dev_attr_wowlan_pattern_search);
	device_remove_file(wl->dev, &dev_attr_wowlan_pattern_clear);
}
