/*
 * ads1015.c - lm_sensors driver for ads1015 12-bit 4-input ADC
 * (C) Copyright 2010
 * Dirk Eibach, Guntermann & Drunck GmbH <eibach@gdsys.de>
 *
 * Based on the ads7828 driver by Steve Hardy.
 *
 * Datasheet available at: http://focus.ti.com/lit/ds/symlink/ads1015.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * History: 2013-07-16 by Bas Laarhoven
 *
 *          Added support for the ADS1115 (16-bit version) converter.
 *          The ADC type (ADS1015 or ADS1115) is detected automagically.
 *          Added DT 'hi_res' configuration setting to add 3 decimal
 *          digits to the returned value. Fixed scaling for ADS1015.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/i2c/ads1015.h>

/* ADS1015 registers */
enum {
	ADS1015_CONVERSION = 0,
	ADS1015_CONFIG = 1,
};

/* PGA fullscale voltages in mV */
static const unsigned int fullscale_table[8] = {
	6144, 4096, 2048, 1024, 512, 256, 256, 256 };

/* Data rates in samples per second */
static const unsigned int ads1015_data_rate_table[8] = {
	128, 250, 490, 920, 1600, 2400, 3300, 3300 };

/* Data rates in samples per second */
static const unsigned int ads1115_data_rate_table[8] = {
	8, 16, 32, 64, 128, 250, 475, 860 };

#define ADS1015_DEFAULT_CHANNELS 0xff
#define ADS1015_DEFAULT_PGA 2
#define ADS1015_DEFAULT_DATA_RATE 4
#define ADS1015_DEFAULT_RESOLUTION 0

struct ads1015_data {
	struct device *hwmon_dev;
	struct mutex update_lock; /* mutex protect updates */
	struct ads1015_channel_data channel_data[ADS1015_CHANNELS];
	enum { eADS1X15 = 0, eADS1015, eADS1115 } ads1x15_type;
	enum { eNORMAL = 0, eHI_RES } hi_res;
};

static int ads1015_read_adc(struct i2c_client *client, unsigned int channel)
{
	u16 config;
	struct ads1015_data *data = i2c_get_clientdata(client);
	unsigned int pga = data->channel_data[channel].pga;
	unsigned int data_rate = data->channel_data[channel].data_rate;
	unsigned int conversion_time_ms;
	int res;

	mutex_lock(&data->update_lock);

	/* get channel parameters */
	res = i2c_smbus_read_word_swapped(client, ADS1015_CONFIG);
	if (res < 0)
		goto err_unlock;
	config = res;
	if (data->ads1x15_type == eADS1115) {
		conversion_time_ms = DIV_ROUND_UP(1000, ads1115_data_rate_table[data_rate]);
	} else {
		conversion_time_ms = DIV_ROUND_UP(1000, ads1015_data_rate_table[data_rate]);
	}

	/* setup and start single conversion */
	config &= 0x001f;
	config |= (1 << 15) | (1 << 8);
	config |= (channel & 0x0007) << 12;
	config |= (pga & 0x0007) << 9;
	config |= (data_rate & 0x0007) << 5;

	res = i2c_smbus_write_word_swapped(client, ADS1015_CONFIG, config);
	if (res < 0)
		goto err_unlock;

	/* wait until conversion finished */
	msleep(conversion_time_ms);
	res = i2c_smbus_read_word_swapped(client, ADS1015_CONFIG);
	if (res < 0)
		goto err_unlock;
	config = res;
	if (data->ads1x15_type == eADS1X15) {
		if (config & (1 << 15)) {
			data->ads1x15_type = eADS1015;
		} else {
			/* this still could be a ADS1115, that one's slower than the 1015! */
			conversion_time_ms = DIV_ROUND_UP(1000,
			        ads1115_data_rate_table[data_rate]) - conversion_time_ms;
			msleep( conversion_time_ms);
			res = i2c_smbus_read_word_swapped(client, ADS1015_CONFIG);
			if (res < 0)
				goto err_unlock;
			data->ads1x15_type = eADS1115;
			config = res;
		}
	}
	if (!(config & (1 << 15))) {
		data->ads1x15_type = eADS1X15;
		/* conversion not finished in time */
		res = -EIO;
		goto err_unlock;
	}

	res = i2c_smbus_read_word_swapped(client, ADS1015_CONVERSION);

err_unlock:
	mutex_unlock(&data->update_lock);
	return res;
}

static int ads1015_reg_to_muv(struct i2c_client *client, unsigned int channel,
			s16 reg, u16* mv, u16* uv, bool* sign)
{
	struct ads1015_data *data = i2c_get_clientdata(client);
	unsigned int pga = data->channel_data[channel].pga;
	int fullscale = fullscale_table[pga];
	u32 alt_uv;

	/*
	 *  Scale to 1/1024 mV units, these units make it easy
	 *  to convert to mV and uV values on low performance systems.
	 *
	 *  formula: muv = 1024 * fullscale * reg / 0x8000
	 */
	if (reg & (0x8000)) {
		alt_uv = DIV_ROUND_CLOSEST( fullscale * -reg, 32);
		*sign = true;
	} else {
		alt_uv = DIV_ROUND_CLOSEST( fullscale * reg, 32);
		*sign = false;
	}
	if (data->hi_res) {
		/*  Use hi_res calculation with fraction */
		*uv = ((alt_uv % 1024) * 1000) / 1024;
		*mv = alt_uv / 1024;
		return 1;
	} else {
		/*  Use original calculation and output format */
		*mv = DIV_ROUND_CLOSEST( alt_uv, 1024);
		return 0;
	}
}

/* sysfs callback function */
static ssize_t show_in(struct device *dev, struct device_attribute *da,
	char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
	int res;
	int index = attr->index;
	u16 mv;
	u16 uv;
	bool sign;

	res = ads1015_read_adc(client, index);
	if (res < 0)
		return res;

	if (ads1015_reg_to_muv( client, index, res, &mv, &uv, &sign)) {
		/* Show milli-Volts with decimal point (sorry, no comma) */
		/* TODO: Decide whether to show micro-Volts ??? */
		return sprintf(buf, "%s%d.%03u\n", (sign) ? "-" : "", mv, uv);
	} else {
		return sprintf(buf, (sign) ? "-%u\n" : "%u\n", mv);
	}
}

static const struct sensor_device_attribute ads1015_in[] = {
	SENSOR_ATTR(in0_input, S_IRUGO, show_in, NULL, 0),
	SENSOR_ATTR(in1_input, S_IRUGO, show_in, NULL, 1),
	SENSOR_ATTR(in2_input, S_IRUGO, show_in, NULL, 2),
	SENSOR_ATTR(in3_input, S_IRUGO, show_in, NULL, 3),
	SENSOR_ATTR(in4_input, S_IRUGO, show_in, NULL, 4),
	SENSOR_ATTR(in5_input, S_IRUGO, show_in, NULL, 5),
	SENSOR_ATTR(in6_input, S_IRUGO, show_in, NULL, 6),
	SENSOR_ATTR(in7_input, S_IRUGO, show_in, NULL, 7),
};

/*
 * Driver interface
 */

static int ads1015_remove(struct i2c_client *client)
{
	struct ads1015_data *data = i2c_get_clientdata(client);
	int k;

	hwmon_device_unregister(data->hwmon_dev);
	for (k = 0; k < ADS1015_CHANNELS; ++k)
		device_remove_file(&client->dev, &ads1015_in[k].dev_attr);
	return 0;
}

#ifdef CONFIG_OF
static int ads1015_get_channels_config_of(struct i2c_client *client)
{
	struct ads1015_data *data = i2c_get_clientdata(client);
	struct device_node *node;
	unsigned int hi_res = ADS1015_DEFAULT_RESOLUTION;
	const __be32 *property;
	int len;

	if (!client->dev.of_node
	    || !of_get_next_child(client->dev.of_node, NULL))
		return -EINVAL;

	property = of_get_property(client->dev.of_node, "hi_res", &len);
	if (property && len == sizeof(int)) {
		hi_res = be32_to_cpup(property);
		if (hi_res > 1) {
			dev_err(&client->dev,
				"invalid hi_res setting on %s\n",
				client->dev.of_node->full_name);
			hi_res = 0;
		}
	}
	data->hi_res = (hi_res) ? eHI_RES : eNORMAL;

	for_each_child_of_node(client->dev.of_node, node) {
		unsigned int channel;
		unsigned int pga = ADS1015_DEFAULT_PGA;
		unsigned int data_rate = ADS1015_DEFAULT_DATA_RATE;

		property = of_get_property(node, "reg", &len);
		if (!property || len != sizeof(int)) {
			dev_err(&client->dev, "invalid reg on %s\n",
				node->full_name);
			continue;
		}

		channel = be32_to_cpup(property);
		if (channel > ADS1015_CHANNELS) {
			dev_err(&client->dev,
				"invalid channel index %d on %s\n",
				channel, node->full_name);
			continue;
		}

		property = of_get_property(node, "ti,gain", &len);
		if (property && len == sizeof(int)) {
			pga = be32_to_cpup(property);
			if (pga > 6) {
				dev_err(&client->dev,
					"invalid gain on %s\n",
					node->full_name);
			}
			pga = 0;
		}

		property = of_get_property(node, "ti,datarate", &len);
		if (property && len == sizeof(int)) {
			data_rate = be32_to_cpup(property);
			if (data_rate > 7) {
				dev_err(&client->dev,
					"invalid data_rate on %s\n",
					node->full_name);
			}
			data_rate = 0;
		}

		data->channel_data[channel].enabled = true;
		data->channel_data[channel].pga = pga;
		data->channel_data[channel].data_rate = data_rate;
	}

	return 0;
}
#endif

static void ads1015_get_channels_config(struct i2c_client *client)
{
	unsigned int k;
	struct ads1015_data *data = i2c_get_clientdata(client);
	struct ads1015_platform_data *pdata = dev_get_platdata(&client->dev);

	/* prefer platform data */
	if (pdata) {
		memcpy(data->channel_data, pdata->channel_data,
		       sizeof(data->channel_data));
		return;
	}

#ifdef CONFIG_OF
	if (!ads1015_get_channels_config_of(client))
		return;
#endif

	/* fallback on default configuration */
	for (k = 0; k < ADS1015_CHANNELS; ++k) {
		data->channel_data[k].enabled = true;
		data->channel_data[k].pga = ADS1015_DEFAULT_PGA;
		data->channel_data[k].data_rate = ADS1015_DEFAULT_DATA_RATE;
	}
}

static int ads1015_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ads1015_data *data;
	int err;
	unsigned int k;

	data = devm_kzalloc(&client->dev, sizeof(struct ads1015_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	/* build sysfs attribute group */
	data->ads1x15_type = eADS1X15;	// 12 or 16 bits ADC is unknown yet
	ads1015_get_channels_config(client);
	for (k = 0; k < ADS1015_CHANNELS; ++k) {
		if (!data->channel_data[k].enabled)
			continue;
		err = device_create_file(&client->dev, &ads1015_in[k].dev_attr);
		if (err)
			goto exit_remove;
	}

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	for (k = 0; k < ADS1015_CHANNELS; ++k)
		device_remove_file(&client->dev, &ads1015_in[k].dev_attr);
	return err;
}

static const struct i2c_device_id ads1015_id[] = {
	{ "ads1015", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1015_id);

static struct i2c_driver ads1015_driver = {
	.driver = {
		.name = "ads1015",
	},
	.probe = ads1015_probe,
	.remove = ads1015_remove,
	.id_table = ads1015_id,
};

module_i2c_driver(ads1015_driver);

MODULE_AUTHOR("Dirk Eibach <eibach@gdsys.de>");
MODULE_DESCRIPTION("ADS1015 driver");
MODULE_LICENSE("GPL");
