// SPDX-License-Identifier: GPL-2.0-only
/*
 * The industrial I/O core
 *
 * Copyright (c) 2008 Jonathan Cameron
 *
 * Based on elements of hwmon and input subsystems.
 */

#define pr_fmt(fmt) "iio-core: " fmt

#include <linux/anon_inodes.h>
#include <linux/cdev.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/property.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/wordpart.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/events.h>
#include <linux/iio/iio-opaque.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "iio_core.h"
#include "iio_core_trigger.h"

/* IDA to assign each registered device a unique id */
static DEFINE_IDA(iio_ida);

static dev_t iio_devt;

#define IIO_DEV_MAX 256
const struct bus_type iio_bus_type = {
	.name = "iio",
};
EXPORT_SYMBOL(iio_bus_type);

static struct dentry *iio_debugfs_dentry;

static const char * const iio_direction[] = {
	[0] = "in",
	[1] = "out",
};

static const char * const iio_chan_type_name_spec[] = {
	[IIO_VOLTAGE] = "voltage",
	[IIO_CURRENT] = "current",
	[IIO_POWER] = "power",
	[IIO_ACCEL] = "accel",
	[IIO_ANGL_VEL] = "anglvel",
	[IIO_MAGN] = "magn",
	[IIO_LIGHT] = "illuminance",
	[IIO_INTENSITY] = "intensity",
	[IIO_PROXIMITY] = "proximity",
	[IIO_TEMP] = "temp",
	[IIO_INCLI] = "incli",
	[IIO_ROT] = "rot",
	[IIO_ANGL] = "angl",
	[IIO_TIMESTAMP] = "timestamp",
	[IIO_CAPACITANCE] = "capacitance",
	[IIO_ALTVOLTAGE] = "altvoltage",
	[IIO_CCT] = "cct",
	[IIO_PRESSURE] = "pressure",
	[IIO_HUMIDITYRELATIVE] = "humidityrelative",
	[IIO_ACTIVITY] = "activity",
	[IIO_STEPS] = "steps",
	[IIO_ENERGY] = "energy",
	[IIO_DISTANCE] = "distance",
	[IIO_VELOCITY] = "velocity",
	[IIO_CONCENTRATION] = "concentration",
	[IIO_RESISTANCE] = "resistance",
	[IIO_PH] = "ph",
	[IIO_UVINDEX] = "uvindex",
	[IIO_ELECTRICALCONDUCTIVITY] = "electricalconductivity",
	[IIO_COUNT] = "count",
	[IIO_INDEX] = "index",
	[IIO_GRAVITY]  = "gravity",
	[IIO_POSITIONRELATIVE]  = "positionrelative",
	[IIO_PHASE] = "phase",
	[IIO_MASSCONCENTRATION] = "massconcentration",
	[IIO_DELTA_ANGL] = "deltaangl",
	[IIO_DELTA_VELOCITY] = "deltavelocity",
	[IIO_COLORTEMP] = "colortemp",
	[IIO_CHROMATICITY] = "chromaticity",
	[IIO_ATTENTION] = "attention",
};

static const char * const iio_modifier_names[] = {
	[IIO_MOD_X] = "x",
	[IIO_MOD_Y] = "y",
	[IIO_MOD_Z] = "z",
	[IIO_MOD_X_AND_Y] = "x&y",
	[IIO_MOD_X_AND_Z] = "x&z",
	[IIO_MOD_Y_AND_Z] = "y&z",
	[IIO_MOD_X_AND_Y_AND_Z] = "x&y&z",
	[IIO_MOD_X_OR_Y] = "x|y",
	[IIO_MOD_X_OR_Z] = "x|z",
	[IIO_MOD_Y_OR_Z] = "y|z",
	[IIO_MOD_X_OR_Y_OR_Z] = "x|y|z",
	[IIO_MOD_ROOT_SUM_SQUARED_X_Y] = "sqrt(x^2+y^2)",
	[IIO_MOD_SUM_SQUARED_X_Y_Z] = "x^2+y^2+z^2",
	[IIO_MOD_LIGHT_BOTH] = "both",
	[IIO_MOD_LIGHT_IR] = "ir",
	[IIO_MOD_LIGHT_CLEAR] = "clear",
	[IIO_MOD_LIGHT_RED] = "red",
	[IIO_MOD_LIGHT_GREEN] = "green",
	[IIO_MOD_LIGHT_BLUE] = "blue",
	[IIO_MOD_LIGHT_UV] = "uv",
	[IIO_MOD_LIGHT_UVA] = "uva",
	[IIO_MOD_LIGHT_UVB] = "uvb",
	[IIO_MOD_LIGHT_DUV] = "duv",
	[IIO_MOD_QUATERNION] = "quaternion",
	[IIO_MOD_TEMP_AMBIENT] = "ambient",
	[IIO_MOD_TEMP_OBJECT] = "object",
	[IIO_MOD_NORTH_MAGN] = "from_north_magnetic",
	[IIO_MOD_NORTH_TRUE] = "from_north_true",
	[IIO_MOD_NORTH_MAGN_TILT_COMP] = "from_north_magnetic_tilt_comp",
	[IIO_MOD_NORTH_TRUE_TILT_COMP] = "from_north_true_tilt_comp",
	[IIO_MOD_RUNNING] = "running",
	[IIO_MOD_JOGGING] = "jogging",
	[IIO_MOD_WALKING] = "walking",
	[IIO_MOD_STILL] = "still",
	[IIO_MOD_ROOT_SUM_SQUARED_X_Y_Z] = "sqrt(x^2+y^2+z^2)",
	[IIO_MOD_I] = "i",
	[IIO_MOD_Q] = "q",
	[IIO_MOD_CO2] = "co2",
	[IIO_MOD_VOC] = "voc",
	[IIO_MOD_PM1] = "pm1",
	[IIO_MOD_PM2P5] = "pm2p5",
	[IIO_MOD_PM4] = "pm4",
	[IIO_MOD_PM10] = "pm10",
	[IIO_MOD_ETHANOL] = "ethanol",
	[IIO_MOD_H2] = "h2",
	[IIO_MOD_O2] = "o2",
	[IIO_MOD_LINEAR_X] = "linear_x",
	[IIO_MOD_LINEAR_Y] = "linear_y",
	[IIO_MOD_LINEAR_Z] = "linear_z",
	[IIO_MOD_PITCH] = "pitch",
	[IIO_MOD_YAW] = "yaw",
	[IIO_MOD_ROLL] = "roll",
};

/* relies on pairs of these shared then separate */
static const char * const iio_chan_info_postfix[] = {
	[IIO_CHAN_INFO_RAW] = "raw",
	[IIO_CHAN_INFO_PROCESSED] = "input",
	[IIO_CHAN_INFO_SCALE] = "scale",
	[IIO_CHAN_INFO_OFFSET] = "offset",
	[IIO_CHAN_INFO_CALIBSCALE] = "calibscale",
	[IIO_CHAN_INFO_CALIBBIAS] = "calibbias",
	[IIO_CHAN_INFO_PEAK] = "peak_raw",
	[IIO_CHAN_INFO_PEAK_SCALE] = "peak_scale",
	[IIO_CHAN_INFO_QUADRATURE_CORRECTION_RAW] = "quadrature_correction_raw",
	[IIO_CHAN_INFO_AVERAGE_RAW] = "mean_raw",
	[IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY]
	= "filter_low_pass_3db_frequency",
	[IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY]
	= "filter_high_pass_3db_frequency",
	[IIO_CHAN_INFO_SAMP_FREQ] = "sampling_frequency",
	[IIO_CHAN_INFO_FREQUENCY] = "frequency",
	[IIO_CHAN_INFO_PHASE] = "phase",
	[IIO_CHAN_INFO_HARDWAREGAIN] = "hardwaregain",
	[IIO_CHAN_INFO_HYSTERESIS] = "hysteresis",
	[IIO_CHAN_INFO_HYSTERESIS_RELATIVE] = "hysteresis_relative",
	[IIO_CHAN_INFO_INT_TIME] = "integration_time",
	[IIO_CHAN_INFO_ENABLE] = "en",
	[IIO_CHAN_INFO_CALIBHEIGHT] = "calibheight",
	[IIO_CHAN_INFO_CALIBWEIGHT] = "calibweight",
	[IIO_CHAN_INFO_DEBOUNCE_COUNT] = "debounce_count",
	[IIO_CHAN_INFO_DEBOUNCE_TIME] = "debounce_time",
	[IIO_CHAN_INFO_CALIBEMISSIVITY] = "calibemissivity",
	[IIO_CHAN_INFO_OVERSAMPLING_RATIO] = "oversampling_ratio",
	[IIO_CHAN_INFO_THERMOCOUPLE_TYPE] = "thermocouple_type",
	[IIO_CHAN_INFO_CALIBAMBIENT] = "calibambient",
	[IIO_CHAN_INFO_ZEROPOINT] = "zeropoint",
	[IIO_CHAN_INFO_TROUGH] = "trough_raw",
	[IIO_CHAN_INFO_CONVDELAY] = "convdelay",
};
/**
 * iio_device_id() - query the unique ID for the device
 * @indio_dev:		Device structure whose ID is being queried
 *
 * The IIO device ID is a unique index used for example for the naming
 * of the character device /dev/iio\:device[ID].
 *
 * Returns: Unique ID for the device.
 */
int iio_device_id(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	return iio_dev_opaque->id;
}
EXPORT_SYMBOL_GPL(iio_device_id);

/**
 * iio_buffer_enabled() - helper function to test if the buffer is enabled
 * @indio_dev:		IIO device structure for device
 *
 * Returns: True, if the buffer is enabled.
 */
bool iio_buffer_enabled(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	return iio_dev_opaque->currentmode & INDIO_ALL_BUFFER_MODES;
}
EXPORT_SYMBOL_GPL(iio_buffer_enabled);

#if defined(CONFIG_DEBUG_FS)
/*
 * There's also a CONFIG_DEBUG_FS guard in include/linux/iio/iio.h for
 * iio_get_debugfs_dentry() to make it inline if CONFIG_DEBUG_FS is undefined
 */
struct dentry *iio_get_debugfs_dentry(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	return iio_dev_opaque->debugfs_dentry;
}
EXPORT_SYMBOL_GPL(iio_get_debugfs_dentry);
#endif

/**
 * iio_find_channel_from_si() - get channel from its scan index
 * @indio_dev:		device
 * @si:			scan index to match
 *
 * Returns:
 * Constant pointer to iio_chan_spec, if scan index matches, NULL on failure.
 */
const struct iio_chan_spec
*iio_find_channel_from_si(struct iio_dev *indio_dev, int si)
{
	int i;

	for (i = 0; i < indio_dev->num_channels; i++)
		if (indio_dev->channels[i].scan_index == si)
			return &indio_dev->channels[i];
	return NULL;
}

/* This turns up an awful lot */
ssize_t iio_read_const_attr(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	return sysfs_emit(buf, "%s\n", to_iio_const_attr(attr)->string);
}
EXPORT_SYMBOL(iio_read_const_attr);

/**
 * iio_device_set_clock() - Set current timestamping clock for the device
 * @indio_dev: IIO device structure containing the device
 * @clock_id: timestamping clock POSIX identifier to set.
 *
 * Returns: 0 on success, or a negative error code.
 */
int iio_device_set_clock(struct iio_dev *indio_dev, clockid_t clock_id)
{
	int ret;
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	const struct iio_event_interface *ev_int = iio_dev_opaque->event_interface;

	ret = mutex_lock_interruptible(&iio_dev_opaque->mlock);
	if (ret)
		return ret;
	if ((ev_int && iio_event_enabled(ev_int)) ||
	    iio_buffer_enabled(indio_dev)) {
		mutex_unlock(&iio_dev_opaque->mlock);
		return -EBUSY;
	}
	iio_dev_opaque->clock_id = clock_id;
	mutex_unlock(&iio_dev_opaque->mlock);

	return 0;
}
EXPORT_SYMBOL(iio_device_set_clock);

/**
 * iio_device_get_clock() - Retrieve current timestamping clock for the device
 * @indio_dev: IIO device structure containing the device
 *
 * Returns: Clock ID of the current timestamping clock for the device.
 */
clockid_t iio_device_get_clock(const struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	return iio_dev_opaque->clock_id;
}
EXPORT_SYMBOL(iio_device_get_clock);

/**
 * iio_get_time_ns() - utility function to get a time stamp for events etc
 * @indio_dev: device
 *
 * Returns: Timestamp of the event in nanoseconds.
 */
s64 iio_get_time_ns(const struct iio_dev *indio_dev)
{
	struct timespec64 tp;

	switch (iio_device_get_clock(indio_dev)) {
	case CLOCK_REALTIME:
		return ktime_get_real_ns();
	case CLOCK_MONOTONIC:
		return ktime_get_ns();
	case CLOCK_MONOTONIC_RAW:
		return ktime_get_raw_ns();
	case CLOCK_REALTIME_COARSE:
		return ktime_to_ns(ktime_get_coarse_real());
	case CLOCK_MONOTONIC_COARSE:
		ktime_get_coarse_ts64(&tp);
		return timespec64_to_ns(&tp);
	case CLOCK_BOOTTIME:
		return ktime_get_boottime_ns();
	case CLOCK_TAI:
		return ktime_get_clocktai_ns();
	default:
		BUG();
	}
}
EXPORT_SYMBOL(iio_get_time_ns);

static int __init iio_init(void)
{
	int ret;

	/* Register sysfs bus */
	ret  = bus_register(&iio_bus_type);
	if (ret < 0) {
		pr_err("could not register bus type\n");
		goto error_nothing;
	}

	ret = alloc_chrdev_region(&iio_devt, 0, IIO_DEV_MAX, "iio");
	if (ret < 0) {
		pr_err("failed to allocate char dev region\n");
		goto error_unregister_bus_type;
	}

	iio_debugfs_dentry = debugfs_create_dir("iio", NULL);

	return 0;

error_unregister_bus_type:
	bus_unregister(&iio_bus_type);
error_nothing:
	return ret;
}

static void __exit iio_exit(void)
{
	if (iio_devt)
		unregister_chrdev_region(iio_devt, IIO_DEV_MAX);
	bus_unregister(&iio_bus_type);
	debugfs_remove(iio_debugfs_dentry);
}

#if defined(CONFIG_DEBUG_FS)
static ssize_t iio_debugfs_read_reg(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	unsigned int val = 0;
	int ret;

	if (*ppos > 0)
		return simple_read_from_buffer(userbuf, count, ppos,
					       iio_dev_opaque->read_buf,
					       iio_dev_opaque->read_buf_len);

	ret = indio_dev->info->debugfs_reg_access(indio_dev,
						  iio_dev_opaque->cached_reg_addr,
						  0, &val);
	if (ret) {
		dev_err(indio_dev->dev.parent, "%s: read failed\n", __func__);
		return ret;
	}

	iio_dev_opaque->read_buf_len = snprintf(iio_dev_opaque->read_buf,
						sizeof(iio_dev_opaque->read_buf),
						"0x%X\n", val);

	return simple_read_from_buffer(userbuf, count, ppos,
				       iio_dev_opaque->read_buf,
				       iio_dev_opaque->read_buf_len);
}

static ssize_t iio_debugfs_write_reg(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	unsigned int reg, val;
	char buf[80];
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;

	ret = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, userbuf,
				     count);
	if (ret < 0)
		return ret;

	buf[ret] = '\0';

	ret = sscanf(buf, "%i %i", &reg, &val);

	switch (ret) {
	case 1:
		iio_dev_opaque->cached_reg_addr = reg;
		break;
	case 2:
		iio_dev_opaque->cached_reg_addr = reg;
		ret = indio_dev->info->debugfs_reg_access(indio_dev, reg,
							  val, NULL);
		if (ret) {
			dev_err(indio_dev->dev.parent, "%s: write failed\n",
				__func__);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	return count;
}

static const struct file_operations iio_debugfs_reg_fops = {
	.open = simple_open,
	.read = iio_debugfs_read_reg,
	.write = iio_debugfs_write_reg,
};

static void iio_device_unregister_debugfs(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	debugfs_remove_recursive(iio_dev_opaque->debugfs_dentry);
}

static void iio_device_register_debugfs(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque;

	if (indio_dev->info->debugfs_reg_access == NULL)
		return;

	if (!iio_debugfs_dentry)
		return;

	iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	iio_dev_opaque->debugfs_dentry =
		debugfs_create_dir(dev_name(&indio_dev->dev),
				   iio_debugfs_dentry);

	debugfs_create_file("direct_reg_access", 0644,
			    iio_dev_opaque->debugfs_dentry, indio_dev,
			    &iio_debugfs_reg_fops);
}
#else
static void iio_device_register_debugfs(struct iio_dev *indio_dev)
{
}

static void iio_device_unregister_debugfs(struct iio_dev *indio_dev)
{
}
#endif /* CONFIG_DEBUG_FS */

static ssize_t iio_read_channel_ext_info(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	const struct iio_chan_spec_ext_info *ext_info;

	ext_info = &this_attr->c->ext_info[this_attr->address];

	return ext_info->read(indio_dev, ext_info->private, this_attr->c, buf);
}

static ssize_t iio_write_channel_ext_info(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	const struct iio_chan_spec_ext_info *ext_info;

	ext_info = &this_attr->c->ext_info[this_attr->address];

	return ext_info->write(indio_dev, ext_info->private,
			       this_attr->c, buf, len);
}

ssize_t iio_enum_available_read(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	const struct iio_enum *e = (const struct iio_enum *)priv;
	unsigned int i;
	size_t len = 0;

	if (!e->num_items)
		return 0;

	for (i = 0; i < e->num_items; ++i) {
		if (!e->items[i])
			continue;
		len += sysfs_emit_at(buf, len, "%s ", e->items[i]);
	}

	/* replace last space with a newline */
	buf[len - 1] = '\n';

	return len;
}
EXPORT_SYMBOL_GPL(iio_enum_available_read);

ssize_t iio_enum_read(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, char *buf)
{
	const struct iio_enum *e = (const struct iio_enum *)priv;
	int i;

	if (!e->get)
		return -EINVAL;

	i = e->get(indio_dev, chan);
	if (i < 0)
		return i;
	if (i >= e->num_items || !e->items[i])
		return -EINVAL;

	return sysfs_emit(buf, "%s\n", e->items[i]);
}
EXPORT_SYMBOL_GPL(iio_enum_read);

ssize_t iio_enum_write(struct iio_dev *indio_dev,
	uintptr_t priv, const struct iio_chan_spec *chan, const char *buf,
	size_t len)
{
	const struct iio_enum *e = (const struct iio_enum *)priv;
	int ret;

	if (!e->set)
		return -EINVAL;

	ret = __sysfs_match_string(e->items, e->num_items, buf);
	if (ret < 0)
		return ret;

	ret = e->set(indio_dev, chan, ret);
	return ret ? ret : len;
}
EXPORT_SYMBOL_GPL(iio_enum_write);

static const struct iio_mount_matrix iio_mount_idmatrix = {
	.rotation = {
		"1", "0", "0",
		"0", "1", "0",
		"0", "0", "1"
	}
};

static int iio_setup_mount_idmatrix(const struct device *dev,
				    struct iio_mount_matrix *matrix)
{
	*matrix = iio_mount_idmatrix;
	dev_info(dev, "mounting matrix not found: using identity...\n");
	return 0;
}

ssize_t iio_show_mount_matrix(struct iio_dev *indio_dev, uintptr_t priv,
			      const struct iio_chan_spec *chan, char *buf)
{
	const struct iio_mount_matrix *mtx;

	mtx = ((iio_get_mount_matrix_t *)priv)(indio_dev, chan);
	if (IS_ERR(mtx))
		return PTR_ERR(mtx);

	if (!mtx)
		mtx = &iio_mount_idmatrix;

	return sysfs_emit(buf, "%s, %s, %s; %s, %s, %s; %s, %s, %s\n",
			  mtx->rotation[0], mtx->rotation[1], mtx->rotation[2],
			  mtx->rotation[3], mtx->rotation[4], mtx->rotation[5],
			  mtx->rotation[6], mtx->rotation[7], mtx->rotation[8]);
}
EXPORT_SYMBOL_GPL(iio_show_mount_matrix);

/**
 * iio_read_mount_matrix() - retrieve iio device mounting matrix from
 *                           device "mount-matrix" property
 * @dev:	device the mounting matrix property is assigned to
 * @matrix:	where to store retrieved matrix
 *
 * If device is assigned no mounting matrix property, a default 3x3 identity
 * matrix will be filled in.
 *
 * Returns: 0 if success, or a negative error code on failure.
 */
int iio_read_mount_matrix(struct device *dev, struct iio_mount_matrix *matrix)
{
	size_t len = ARRAY_SIZE(iio_mount_idmatrix.rotation);
	int err;

	err = device_property_read_string_array(dev, "mount-matrix", matrix->rotation, len);
	if (err == len)
		return 0;

	if (err >= 0)
		/* Invalid number of matrix entries. */
		return -EINVAL;

	if (err != -EINVAL)
		/* Invalid matrix declaration format. */
		return err;

	/* Matrix was not declared at all: fallback to identity. */
	return iio_setup_mount_idmatrix(dev, matrix);
}
EXPORT_SYMBOL(iio_read_mount_matrix);

static ssize_t __iio_format_value(char *buf, size_t offset, unsigned int type,
				  int size, const int *vals)
{
	int tmp0, tmp1;
	s64 tmp2;
	bool scale_db = false;

	switch (type) {
	case IIO_VAL_INT:
		return sysfs_emit_at(buf, offset, "%d", vals[0]);
	case IIO_VAL_INT_PLUS_MICRO_DB:
		scale_db = true;
		fallthrough;
	case IIO_VAL_INT_PLUS_MICRO:
		if (vals[1] < 0)
			return sysfs_emit_at(buf, offset, "-%d.%06u%s",
					     abs(vals[0]), -vals[1],
					     scale_db ? " dB" : "");
		else
			return sysfs_emit_at(buf, offset, "%d.%06u%s", vals[0],
					     vals[1], scale_db ? " dB" : "");
	case IIO_VAL_INT_PLUS_NANO:
		if (vals[1] < 0)
			return sysfs_emit_at(buf, offset, "-%d.%09u",
					     abs(vals[0]), -vals[1]);
		else
			return sysfs_emit_at(buf, offset, "%d.%09u", vals[0],
					     vals[1]);
	case IIO_VAL_FRACTIONAL:
		tmp2 = div_s64((s64)vals[0] * 1000000000LL, vals[1]);
		tmp0 = (int)div_s64_rem(tmp2, 1000000000, &tmp1);
		if ((tmp2 < 0) && (tmp0 == 0))
			return sysfs_emit_at(buf, offset, "-0.%09u", abs(tmp1));
		else
			return sysfs_emit_at(buf, offset, "%d.%09u", tmp0,
					     abs(tmp1));
	case IIO_VAL_FRACTIONAL_LOG2:
		tmp2 = shift_right((s64)vals[0] * 1000000000LL, vals[1]);
		tmp0 = (int)div_s64_rem(tmp2, 1000000000LL, &tmp1);
		if (tmp0 == 0 && tmp2 < 0)
			return sysfs_emit_at(buf, offset, "-0.%09u", abs(tmp1));
		else
			return sysfs_emit_at(buf, offset, "%d.%09u", tmp0,
					     abs(tmp1));
	case IIO_VAL_INT_MULTIPLE:
	{
		int i;
		int l = 0;

		for (i = 0; i < size; ++i)
			l += sysfs_emit_at(buf, offset + l, "%d ", vals[i]);
		return l;
	}
	case IIO_VAL_CHAR:
		return sysfs_emit_at(buf, offset, "%c", (char)vals[0]);
	case IIO_VAL_INT_64:
		tmp2 = (s64)((((u64)vals[1]) << 32) | (u32)vals[0]);
		return sysfs_emit_at(buf, offset, "%lld", tmp2);
	default:
		return 0;
	}
}

/**
 * iio_format_value() - Formats a IIO value into its string representation
 * @buf:	The buffer to which the formatted value gets written
 *		which is assumed to be big enough (i.e. PAGE_SIZE).
 * @type:	One of the IIO_VAL_* constants. This decides how the val
 *		and val2 parameters are formatted.
 * @size:	Number of IIO value entries contained in vals
 * @vals:	Pointer to the values, exact meaning depends on the
 *		type parameter.
 *
 * Returns:
 * 0 by default, a negative number on failure or the total number of characters
 * written for a type that belongs to the IIO_VAL_* constant.
 */
ssize_t iio_format_value(char *buf, unsigned int type, int size, int *vals)
{
	ssize_t len;

	len = __iio_format_value(buf, 0, type, size, vals);
	if (len >= PAGE_SIZE - 1)
		return -EFBIG;

	return len + sysfs_emit_at(buf, len, "\n");
}
EXPORT_SYMBOL_GPL(iio_format_value);

ssize_t do_iio_read_channel_label(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *c,
				  char *buf)
{
	if (indio_dev->info->read_label)
		return indio_dev->info->read_label(indio_dev, c, buf);

	if (c->extend_name)
		return sysfs_emit(buf, "%s\n", c->extend_name);

	return -EINVAL;
}

static ssize_t iio_read_channel_label(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	return do_iio_read_channel_label(dev_to_iio_dev(dev),
					 to_iio_dev_attr(attr)->c, buf);
}

static ssize_t iio_read_channel_info(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int vals[INDIO_MAX_RAW_ELEMENTS];
	int ret;
	int val_len = 2;

	if (indio_dev->info->read_raw_multi)
		ret = indio_dev->info->read_raw_multi(indio_dev, this_attr->c,
							INDIO_MAX_RAW_ELEMENTS,
							vals, &val_len,
							this_attr->address);
	else if (indio_dev->info->read_raw)
		ret = indio_dev->info->read_raw(indio_dev, this_attr->c,
				    &vals[0], &vals[1], this_attr->address);
	else
		return -EINVAL;

	if (ret < 0)
		return ret;

	return iio_format_value(buf, ret, val_len, vals);
}

static ssize_t iio_format_list(char *buf, const int *vals, int type, int length,
			       const char *prefix, const char *suffix)
{
	ssize_t len;
	int stride;
	int i;

	switch (type) {
	case IIO_VAL_INT:
		stride = 1;
		break;
	default:
		stride = 2;
		break;
	}

	len = sysfs_emit(buf, prefix);

	for (i = 0; i <= length - stride; i += stride) {
		if (i != 0) {
			len += sysfs_emit_at(buf, len, " ");
			if (len >= PAGE_SIZE)
				return -EFBIG;
		}

		len += __iio_format_value(buf, len, type, stride, &vals[i]);
		if (len >= PAGE_SIZE)
			return -EFBIG;
	}

	len += sysfs_emit_at(buf, len, "%s\n", suffix);

	return len;
}

static ssize_t iio_format_avail_list(char *buf, const int *vals,
				     int type, int length)
{

	return iio_format_list(buf, vals, type, length, "", "");
}

static ssize_t iio_format_avail_range(char *buf, const int *vals, int type)
{
	int length;

	/*
	 * length refers to the array size , not the number of elements.
	 * The purpose is to print the range [min , step ,max] so length should
	 * be 3 in case of int, and 6 for other types.
	 */
	switch (type) {
	case IIO_VAL_INT:
		length = 3;
		break;
	default:
		length = 6;
		break;
	}

	return iio_format_list(buf, vals, type, length, "[", "]");
}

static ssize_t iio_read_channel_info_avail(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	const int *vals;
	int ret;
	int length;
	int type;

	if (!indio_dev->info->read_avail)
		return -EINVAL;

	ret = indio_dev->info->read_avail(indio_dev, this_attr->c,
					  &vals, &type, &length,
					  this_attr->address);

	if (ret < 0)
		return ret;
	switch (ret) {
	case IIO_AVAIL_LIST:
		return iio_format_avail_list(buf, vals, type, length);
	case IIO_AVAIL_RANGE:
		return iio_format_avail_range(buf, vals, type);
	default:
		return -EINVAL;
	}
}

/**
 * __iio_str_to_fixpoint() - Parse a fixed-point number from a string
 * @str: The string to parse
 * @fract_mult: Multiplier for the first decimal place, should be a power of 10
 * @integer: The integer part of the number
 * @fract: The fractional part of the number
 * @scale_db: True if this should parse as dB
 *
 * Returns:
 * 0 on success, or a negative error code if the string could not be parsed.
 */
static int __iio_str_to_fixpoint(const char *str, int fract_mult,
				 int *integer, int *fract, bool scale_db)
{
	int i = 0, f = 0;
	bool integer_part = true, negative = false;

	if (fract_mult == 0) {
		*fract = 0;

		return kstrtoint(str, 0, integer);
	}

	if (str[0] == '-') {
		negative = true;
		str++;
	} else if (str[0] == '+') {
		str++;
	}

	while (*str) {
		if ('0' <= *str && *str <= '9') {
			if (integer_part) {
				i = i * 10 + *str - '0';
			} else {
				f += fract_mult * (*str - '0');
				fract_mult /= 10;
			}
		} else if (*str == '\n') {
			if (*(str + 1) == '\0')
				break;
			return -EINVAL;
		} else if (!strncmp(str, " dB", sizeof(" dB") - 1) && scale_db) {
			/* Ignore the dB suffix */
			str += sizeof(" dB") - 1;
			continue;
		} else if (!strncmp(str, "dB", sizeof("dB") - 1) && scale_db) {
			/* Ignore the dB suffix */
			str += sizeof("dB") - 1;
			continue;
		} else if (*str == '.' && integer_part) {
			integer_part = false;
		} else {
			return -EINVAL;
		}
		str++;
	}

	if (negative) {
		if (i)
			i = -i;
		else
			f = -f;
	}

	*integer = i;
	*fract = f;

	return 0;
}

/**
 * iio_str_to_fixpoint() - Parse a fixed-point number from a string
 * @str: The string to parse
 * @fract_mult: Multiplier for the first decimal place, should be a power of 10
 * @integer: The integer part of the number
 * @fract: The fractional part of the number
 *
 * Returns:
 * 0 on success, or a negative error code if the string could not be parsed.
 */
int iio_str_to_fixpoint(const char *str, int fract_mult,
			int *integer, int *fract)
{
	return __iio_str_to_fixpoint(str, fract_mult, integer, fract, false);
}
EXPORT_SYMBOL_GPL(iio_str_to_fixpoint);

static ssize_t iio_write_channel_info(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, fract_mult = 100000;
	int integer, fract = 0;
	long long integer64;
	bool is_char = false;
	bool scale_db = false;
	bool is_64bit = false;

	/* Assumes decimal - precision based on number of digits */
	if (!indio_dev->info->write_raw)
		return -EINVAL;

	if (indio_dev->info->write_raw_get_fmt)
		switch (indio_dev->info->write_raw_get_fmt(indio_dev,
			this_attr->c, this_attr->address)) {
		case IIO_VAL_INT:
			fract_mult = 0;
			break;
		case IIO_VAL_INT_PLUS_MICRO_DB:
			scale_db = true;
			fallthrough;
		case IIO_VAL_INT_PLUS_MICRO:
			fract_mult = 100000;
			break;
		case IIO_VAL_INT_PLUS_NANO:
			fract_mult = 100000000;
			break;
		case IIO_VAL_CHAR:
			is_char = true;
			break;
		case IIO_VAL_INT_64:
			is_64bit = true;
			break;
		default:
			return -EINVAL;
		}

	if (is_char) {
		char ch;

		if (sscanf(buf, "%c", &ch) != 1)
			return -EINVAL;
		integer = ch;
	} else if (is_64bit) {
		ret = kstrtoll(buf, 0, &integer64);
		if (ret)
			return ret;

		fract = upper_32_bits(integer64);
		integer = lower_32_bits(integer64);
	} else {
		ret = __iio_str_to_fixpoint(buf, fract_mult, &integer, &fract,
					    scale_db);
		if (ret)
			return ret;
	}

	ret = indio_dev->info->write_raw(indio_dev, this_attr->c,
					 integer, fract, this_attr->address);
	if (ret)
		return ret;

	return len;
}

static
int __iio_device_attr_init(struct device_attribute *dev_attr,
			   const char *postfix,
			   struct iio_chan_spec const *chan,
			   ssize_t (*readfunc)(struct device *dev,
					       struct device_attribute *attr,
					       char *buf),
			   ssize_t (*writefunc)(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t len),
			   enum iio_shared_by shared_by)
{
	int ret = 0;
	char *name = NULL;
	char *full_postfix;

	sysfs_attr_init(&dev_attr->attr);

	/* Build up postfix of <extend_name>_<modifier>_postfix */
	if (chan->modified && (shared_by == IIO_SEPARATE)) {
		if (chan->extend_name)
			full_postfix = kasprintf(GFP_KERNEL, "%s_%s_%s",
						 iio_modifier_names[chan->channel2],
						 chan->extend_name,
						 postfix);
		else
			full_postfix = kasprintf(GFP_KERNEL, "%s_%s",
						 iio_modifier_names[chan->channel2],
						 postfix);
	} else {
		if (chan->extend_name == NULL || shared_by != IIO_SEPARATE)
			full_postfix = kstrdup(postfix, GFP_KERNEL);
		else
			full_postfix = kasprintf(GFP_KERNEL,
						 "%s_%s",
						 chan->extend_name,
						 postfix);
	}
	if (full_postfix == NULL)
		return -ENOMEM;

	if (chan->differential) { /* Differential can not have modifier */
		switch (shared_by) {
		case IIO_SHARED_BY_ALL:
			name = kasprintf(GFP_KERNEL, "%s", full_postfix);
			break;
		case IIO_SHARED_BY_DIR:
			name = kasprintf(GFP_KERNEL, "%s_%s",
						iio_direction[chan->output],
						full_postfix);
			break;
		case IIO_SHARED_BY_TYPE:
			name = kasprintf(GFP_KERNEL, "%s_%s-%s_%s",
					    iio_direction[chan->output],
					    iio_chan_type_name_spec[chan->type],
					    iio_chan_type_name_spec[chan->type],
					    full_postfix);
			break;
		case IIO_SEPARATE:
			if (!chan->indexed) {
				WARN(1, "Differential channels must be indexed\n");
				ret = -EINVAL;
				goto error_free_full_postfix;
			}
			name = kasprintf(GFP_KERNEL,
					    "%s_%s%d-%s%d_%s",
					    iio_direction[chan->output],
					    iio_chan_type_name_spec[chan->type],
					    chan->channel,
					    iio_chan_type_name_spec[chan->type],
					    chan->channel2,
					    full_postfix);
			break;
		}
	} else { /* Single ended */
		switch (shared_by) {
		case IIO_SHARED_BY_ALL:
			name = kasprintf(GFP_KERNEL, "%s", full_postfix);
			break;
		case IIO_SHARED_BY_DIR:
			name = kasprintf(GFP_KERNEL, "%s_%s",
						iio_direction[chan->output],
						full_postfix);
			break;
		case IIO_SHARED_BY_TYPE:
			name = kasprintf(GFP_KERNEL, "%s_%s_%s",
					    iio_direction[chan->output],
					    iio_chan_type_name_spec[chan->type],
					    full_postfix);
			break;

		case IIO_SEPARATE:
			if (chan->indexed)
				name = kasprintf(GFP_KERNEL, "%s_%s%d_%s",
						    iio_direction[chan->output],
						    iio_chan_type_name_spec[chan->type],
						    chan->channel,
						    full_postfix);
			else
				name = kasprintf(GFP_KERNEL, "%s_%s_%s",
						    iio_direction[chan->output],
						    iio_chan_type_name_spec[chan->type],
						    full_postfix);
			break;
		}
	}
	if (name == NULL) {
		ret = -ENOMEM;
		goto error_free_full_postfix;
	}
	dev_attr->attr.name = name;

	if (readfunc) {
		dev_attr->attr.mode |= 0444;
		dev_attr->show = readfunc;
	}

	if (writefunc) {
		dev_attr->attr.mode |= 0200;
		dev_attr->store = writefunc;
	}

error_free_full_postfix:
	kfree(full_postfix);

	return ret;
}

static void __iio_device_attr_deinit(struct device_attribute *dev_attr)
{
	kfree(dev_attr->attr.name);
}

int __iio_add_chan_devattr(const char *postfix,
			   struct iio_chan_spec const *chan,
			   ssize_t (*readfunc)(struct device *dev,
					       struct device_attribute *attr,
					       char *buf),
			   ssize_t (*writefunc)(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t len),
			   u64 mask,
			   enum iio_shared_by shared_by,
			   struct device *dev,
			   struct iio_buffer *buffer,
			   struct list_head *attr_list)
{
	int ret;
	struct iio_dev_attr *iio_attr, *t;

	iio_attr = kzalloc(sizeof(*iio_attr), GFP_KERNEL);
	if (iio_attr == NULL)
		return -ENOMEM;
	ret = __iio_device_attr_init(&iio_attr->dev_attr,
				     postfix, chan,
				     readfunc, writefunc, shared_by);
	if (ret)
		goto error_iio_dev_attr_free;
	iio_attr->c = chan;
	iio_attr->address = mask;
	iio_attr->buffer = buffer;
	list_for_each_entry(t, attr_list, l)
		if (strcmp(t->dev_attr.attr.name,
			   iio_attr->dev_attr.attr.name) == 0) {
			if (shared_by == IIO_SEPARATE)
				dev_err(dev, "tried to double register : %s\n",
					t->dev_attr.attr.name);
			ret = -EBUSY;
			goto error_device_attr_deinit;
		}
	list_add(&iio_attr->l, attr_list);

	return 0;

error_device_attr_deinit:
	__iio_device_attr_deinit(&iio_attr->dev_attr);
error_iio_dev_attr_free:
	kfree(iio_attr);
	return ret;
}

static int iio_device_add_channel_label(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int ret;

	if (!indio_dev->info->read_label && !chan->extend_name)
		return 0;

	ret = __iio_add_chan_devattr("label",
				     chan,
				     &iio_read_channel_label,
				     NULL,
				     0,
				     IIO_SEPARATE,
				     &indio_dev->dev,
				     NULL,
				     &iio_dev_opaque->channel_attr_list);
	if (ret < 0)
		return ret;

	return 1;
}

static int iio_device_add_info_mask_type(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 enum iio_shared_by shared_by,
					 const long *infomask)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int i, ret, attrcount = 0;

	for_each_set_bit(i, infomask, sizeof(*infomask)*8) {
		if (i >= ARRAY_SIZE(iio_chan_info_postfix))
			return -EINVAL;
		ret = __iio_add_chan_devattr(iio_chan_info_postfix[i],
					     chan,
					     &iio_read_channel_info,
					     &iio_write_channel_info,
					     i,
					     shared_by,
					     &indio_dev->dev,
					     NULL,
					     &iio_dev_opaque->channel_attr_list);
		if ((ret == -EBUSY) && (shared_by != IIO_SEPARATE))
			continue;
		if (ret < 0)
			return ret;
		attrcount++;
	}

	return attrcount;
}

static int iio_device_add_info_mask_type_avail(struct iio_dev *indio_dev,
					       struct iio_chan_spec const *chan,
					       enum iio_shared_by shared_by,
					       const long *infomask)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int i, ret, attrcount = 0;
	char *avail_postfix;

	for_each_set_bit(i, infomask, sizeof(*infomask) * 8) {
		if (i >= ARRAY_SIZE(iio_chan_info_postfix))
			return -EINVAL;
		avail_postfix = kasprintf(GFP_KERNEL,
					  "%s_available",
					  iio_chan_info_postfix[i]);
		if (!avail_postfix)
			return -ENOMEM;

		ret = __iio_add_chan_devattr(avail_postfix,
					     chan,
					     &iio_read_channel_info_avail,
					     NULL,
					     i,
					     shared_by,
					     &indio_dev->dev,
					     NULL,
					     &iio_dev_opaque->channel_attr_list);
		kfree(avail_postfix);
		if ((ret == -EBUSY) && (shared_by != IIO_SEPARATE))
			continue;
		if (ret < 0)
			return ret;
		attrcount++;
	}

	return attrcount;
}

static int iio_device_add_channel_sysfs(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int ret, attrcount = 0;
	const struct iio_chan_spec_ext_info *ext_info;

	if (chan->channel < 0)
		return 0;
	ret = iio_device_add_info_mask_type(indio_dev, chan,
					    IIO_SEPARATE,
					    &chan->info_mask_separate);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
						  IIO_SEPARATE,
						  &chan->info_mask_separate_available);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type(indio_dev, chan,
					    IIO_SHARED_BY_TYPE,
					    &chan->info_mask_shared_by_type);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
						  IIO_SHARED_BY_TYPE,
						  &chan->info_mask_shared_by_type_available);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type(indio_dev, chan,
					    IIO_SHARED_BY_DIR,
					    &chan->info_mask_shared_by_dir);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
						  IIO_SHARED_BY_DIR,
						  &chan->info_mask_shared_by_dir_available);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type(indio_dev, chan,
					    IIO_SHARED_BY_ALL,
					    &chan->info_mask_shared_by_all);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_info_mask_type_avail(indio_dev, chan,
						  IIO_SHARED_BY_ALL,
						  &chan->info_mask_shared_by_all_available);
	if (ret < 0)
		return ret;
	attrcount += ret;

	ret = iio_device_add_channel_label(indio_dev, chan);
	if (ret < 0)
		return ret;
	attrcount += ret;

	if (chan->ext_info) {
		unsigned int i = 0;

		for (ext_info = chan->ext_info; ext_info->name; ext_info++) {
			ret = __iio_add_chan_devattr(ext_info->name,
					chan,
					ext_info->read ?
					    &iio_read_channel_ext_info : NULL,
					ext_info->write ?
					    &iio_write_channel_ext_info : NULL,
					i,
					ext_info->shared,
					&indio_dev->dev,
					NULL,
					&iio_dev_opaque->channel_attr_list);
			i++;
			if (ret == -EBUSY && ext_info->shared)
				continue;

			if (ret)
				return ret;

			attrcount++;
		}
	}

	return attrcount;
}

/**
 * iio_free_chan_devattr_list() - Free a list of IIO device attributes
 * @attr_list: List of IIO device attributes
 *
 * This function frees the memory allocated for each of the IIO device
 * attributes in the list.
 */
void iio_free_chan_devattr_list(struct list_head *attr_list)
{
	struct iio_dev_attr *p, *n;

	list_for_each_entry_safe(p, n, attr_list, l) {
		kfree_const(p->dev_attr.attr.name);
		list_del(&p->l);
		kfree(p);
	}
}

static ssize_t name_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	return sysfs_emit(buf, "%s\n", indio_dev->name);
}

static DEVICE_ATTR_RO(name);

static ssize_t label_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	return sysfs_emit(buf, "%s\n", indio_dev->label);
}

static DEVICE_ATTR_RO(label);

static const char * const clock_names[] = {
	[CLOCK_REALTIME]	 	= "realtime",
	[CLOCK_MONOTONIC]	 	= "monotonic",
	[CLOCK_PROCESS_CPUTIME_ID]	= "process_cputime_id",
	[CLOCK_THREAD_CPUTIME_ID]	= "thread_cputime_id",
	[CLOCK_MONOTONIC_RAW]	 	= "monotonic_raw",
	[CLOCK_REALTIME_COARSE]	 	= "realtime_coarse",
	[CLOCK_MONOTONIC_COARSE] 	= "monotonic_coarse",
	[CLOCK_BOOTTIME]	 	= "boottime",
	[CLOCK_REALTIME_ALARM]		= "realtime_alarm",
	[CLOCK_BOOTTIME_ALARM]		= "boottime_alarm",
	[CLOCK_SGI_CYCLE]		= "sgi_cycle",
	[CLOCK_TAI]		 	= "tai",
};

static ssize_t current_timestamp_clock_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	const struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	const clockid_t clk = iio_device_get_clock(indio_dev);

	switch (clk) {
	case CLOCK_REALTIME:
	case CLOCK_MONOTONIC:
	case CLOCK_MONOTONIC_RAW:
	case CLOCK_REALTIME_COARSE:
	case CLOCK_MONOTONIC_COARSE:
	case CLOCK_BOOTTIME:
	case CLOCK_TAI:
		break;
	default:
		BUG();
	}

	return sysfs_emit(buf, "%s\n", clock_names[clk]);
}

static ssize_t current_timestamp_clock_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	clockid_t clk;
	int ret;

	ret = sysfs_match_string(clock_names, buf);
	if (ret < 0)
		return ret;
	clk = ret;

	switch (clk) {
	case CLOCK_REALTIME:
	case CLOCK_MONOTONIC:
	case CLOCK_MONOTONIC_RAW:
	case CLOCK_REALTIME_COARSE:
	case CLOCK_MONOTONIC_COARSE:
	case CLOCK_BOOTTIME:
	case CLOCK_TAI:
		break;
	default:
		return -EINVAL;
	}

	ret = iio_device_set_clock(dev_to_iio_dev(dev), clk);
	if (ret)
		return ret;

	return len;
}

int iio_device_register_sysfs_group(struct iio_dev *indio_dev,
				    const struct attribute_group *group)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	const struct attribute_group **new, **old = iio_dev_opaque->groups;
	unsigned int cnt = iio_dev_opaque->groupcounter;

	new = krealloc_array(old, cnt + 2, sizeof(*new), GFP_KERNEL);
	if (!new)
		return -ENOMEM;

	new[iio_dev_opaque->groupcounter++] = group;
	new[iio_dev_opaque->groupcounter] = NULL;

	iio_dev_opaque->groups = new;

	return 0;
}

static DEVICE_ATTR_RW(current_timestamp_clock);

static int iio_device_register_sysfs(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	int i, ret = 0, attrcount, attrn, attrcount_orig = 0;
	struct iio_dev_attr *p;
	struct attribute **attr, *clk = NULL;

	/* First count elements in any existing group */
	if (indio_dev->info->attrs) {
		attr = indio_dev->info->attrs->attrs;
		while (*attr++ != NULL)
			attrcount_orig++;
	}
	attrcount = attrcount_orig;
	/*
	 * New channel registration method - relies on the fact a group does
	 * not need to be initialized if its name is NULL.
	 */
	if (indio_dev->channels)
		for (i = 0; i < indio_dev->num_channels; i++) {
			const struct iio_chan_spec *chan =
				&indio_dev->channels[i];

			if (chan->type == IIO_TIMESTAMP)
				clk = &dev_attr_current_timestamp_clock.attr;

			ret = iio_device_add_channel_sysfs(indio_dev, chan);
			if (ret < 0)
				goto error_clear_attrs;
			attrcount += ret;
		}

	if (iio_dev_opaque->event_interface)
		clk = &dev_attr_current_timestamp_clock.attr;

	if (indio_dev->name)
		attrcount++;
	if (indio_dev->label)
		attrcount++;
	if (clk)
		attrcount++;

	iio_dev_opaque->chan_attr_group.attrs =
		kcalloc(attrcount + 1,
			sizeof(iio_dev_opaque->chan_attr_group.attrs[0]),
			GFP_KERNEL);
	if (iio_dev_opaque->chan_attr_group.attrs == NULL) {
		ret = -ENOMEM;
		goto error_clear_attrs;
	}
	/* Copy across original attributes, and point to original binary attributes */
	if (indio_dev->info->attrs) {
		memcpy(iio_dev_opaque->chan_attr_group.attrs,
		       indio_dev->info->attrs->attrs,
		       sizeof(iio_dev_opaque->chan_attr_group.attrs[0])
		       *attrcount_orig);
		iio_dev_opaque->chan_attr_group.is_visible =
			indio_dev->info->attrs->is_visible;
		iio_dev_opaque->chan_attr_group.bin_attrs =
			indio_dev->info->attrs->bin_attrs;
	}
	attrn = attrcount_orig;
	/* Add all elements from the list. */
	list_for_each_entry(p, &iio_dev_opaque->channel_attr_list, l)
		iio_dev_opaque->chan_attr_group.attrs[attrn++] = &p->dev_attr.attr;
	if (indio_dev->name)
		iio_dev_opaque->chan_attr_group.attrs[attrn++] = &dev_attr_name.attr;
	if (indio_dev->label)
		iio_dev_opaque->chan_attr_group.attrs[attrn++] = &dev_attr_label.attr;
	if (clk)
		iio_dev_opaque->chan_attr_group.attrs[attrn++] = clk;

	ret = iio_device_register_sysfs_group(indio_dev,
					      &iio_dev_opaque->chan_attr_group);
	if (ret)
		goto error_free_chan_attrs;

	return 0;

error_free_chan_attrs:
	kfree(iio_dev_opaque->chan_attr_group.attrs);
	iio_dev_opaque->chan_attr_group.attrs = NULL;
error_clear_attrs:
	iio_free_chan_devattr_list(&iio_dev_opaque->channel_attr_list);

	return ret;
}

static void iio_device_unregister_sysfs(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	iio_free_chan_devattr_list(&iio_dev_opaque->channel_attr_list);
	kfree(iio_dev_opaque->chan_attr_group.attrs);
	iio_dev_opaque->chan_attr_group.attrs = NULL;
	kfree(iio_dev_opaque->groups);
	iio_dev_opaque->groups = NULL;
}

static void iio_dev_release(struct device *device)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(device);
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	if (indio_dev->modes & INDIO_ALL_TRIGGERED_MODES)
		iio_device_unregister_trigger_consumer(indio_dev);
	iio_device_unregister_eventset(indio_dev);
	iio_device_unregister_sysfs(indio_dev);

	iio_device_detach_buffers(indio_dev);

	lockdep_unregister_key(&iio_dev_opaque->mlock_key);

	ida_free(&iio_ida, iio_dev_opaque->id);
	kfree(iio_dev_opaque);
}

const struct device_type iio_device_type = {
	.name = "iio_device",
	.release = iio_dev_release,
};

/**
 * iio_device_alloc() - allocate an iio_dev from a driver
 * @parent:		Parent device.
 * @sizeof_priv:	Space to allocate for private structure.
 *
 * Returns:
 * Pointer to allocated iio_dev on success, NULL on failure.
 */
struct iio_dev *iio_device_alloc(struct device *parent, int sizeof_priv)
{
	struct iio_dev_opaque *iio_dev_opaque;
	struct iio_dev *indio_dev;
	size_t alloc_size;

	if (sizeof_priv)
		alloc_size = ALIGN(sizeof(*iio_dev_opaque), IIO_DMA_MINALIGN) + sizeof_priv;
	else
		alloc_size = sizeof(*iio_dev_opaque);

	iio_dev_opaque = kzalloc(alloc_size, GFP_KERNEL);
	if (!iio_dev_opaque)
		return NULL;

	indio_dev = &iio_dev_opaque->indio_dev;

	if (sizeof_priv)
		ACCESS_PRIVATE(indio_dev, priv) = (char *)iio_dev_opaque +
			ALIGN(sizeof(*iio_dev_opaque), IIO_DMA_MINALIGN);

	indio_dev->dev.parent = parent;
	indio_dev->dev.type = &iio_device_type;
	indio_dev->dev.bus = &iio_bus_type;
	device_initialize(&indio_dev->dev);
	mutex_init(&iio_dev_opaque->mlock);
	mutex_init(&iio_dev_opaque->info_exist_lock);
	INIT_LIST_HEAD(&iio_dev_opaque->channel_attr_list);

	iio_dev_opaque->id = ida_alloc(&iio_ida, GFP_KERNEL);
	if (iio_dev_opaque->id < 0) {
		/* cannot use a dev_err as the name isn't available */
		pr_err("failed to get device id\n");
		kfree(iio_dev_opaque);
		return NULL;
	}

	if (dev_set_name(&indio_dev->dev, "iio:device%d", iio_dev_opaque->id)) {
		ida_free(&iio_ida, iio_dev_opaque->id);
		kfree(iio_dev_opaque);
		return NULL;
	}

	INIT_LIST_HEAD(&iio_dev_opaque->buffer_list);
	INIT_LIST_HEAD(&iio_dev_opaque->ioctl_handlers);

	lockdep_register_key(&iio_dev_opaque->mlock_key);
	lockdep_set_class(&iio_dev_opaque->mlock, &iio_dev_opaque->mlock_key);

	return indio_dev;
}
EXPORT_SYMBOL(iio_device_alloc);

/**
 * iio_device_free() - free an iio_dev from a driver
 * @dev:		the iio_dev associated with the device
 */
void iio_device_free(struct iio_dev *dev)
{
	if (dev)
		put_device(&dev->dev);
}
EXPORT_SYMBOL(iio_device_free);

static void devm_iio_device_release(void *iio_dev)
{
	iio_device_free(iio_dev);
}

/**
 * devm_iio_device_alloc - Resource-managed iio_device_alloc()
 * @parent:		Device to allocate iio_dev for, and parent for this IIO device
 * @sizeof_priv:	Space to allocate for private structure.
 *
 * Managed iio_device_alloc. iio_dev allocated with this function is
 * automatically freed on driver detach.
 *
 * Returns:
 * Pointer to allocated iio_dev on success, NULL on failure.
 */
struct iio_dev *devm_iio_device_alloc(struct device *parent, int sizeof_priv)
{
	struct iio_dev *iio_dev;
	int ret;

	iio_dev = iio_device_alloc(parent, sizeof_priv);
	if (!iio_dev)
		return NULL;

	ret = devm_add_action_or_reset(parent, devm_iio_device_release,
				       iio_dev);
	if (ret)
		return NULL;

	return iio_dev;
}
EXPORT_SYMBOL_GPL(devm_iio_device_alloc);

/**
 * iio_chrdev_open() - chrdev file open for buffer access and ioctls
 * @inode:	Inode structure for identifying the device in the file system
 * @filp:	File structure for iio device used to keep and later access
 *		private data
 *
 * Returns: 0 on success or -EBUSY if the device is already opened
 */
static int iio_chrdev_open(struct inode *inode, struct file *filp)
{
	struct iio_dev_opaque *iio_dev_opaque =
		container_of(inode->i_cdev, struct iio_dev_opaque, chrdev);
	struct iio_dev *indio_dev = &iio_dev_opaque->indio_dev;
	struct iio_dev_buffer_pair *ib;

	if (test_and_set_bit(IIO_BUSY_BIT_POS, &iio_dev_opaque->flags))
		return -EBUSY;

	iio_device_get(indio_dev);

	ib = kmalloc(sizeof(*ib), GFP_KERNEL);
	if (!ib) {
		iio_device_put(indio_dev);
		clear_bit(IIO_BUSY_BIT_POS, &iio_dev_opaque->flags);
		return -ENOMEM;
	}

	ib->indio_dev = indio_dev;
	ib->buffer = indio_dev->buffer;

	filp->private_data = ib;

	return 0;
}

/**
 * iio_chrdev_release() - chrdev file close buffer access and ioctls
 * @inode:	Inode structure pointer for the char device
 * @filp:	File structure pointer for the char device
 *
 * Returns: 0 for successful release.
 */
static int iio_chrdev_release(struct inode *inode, struct file *filp)
{
	struct iio_dev_buffer_pair *ib = filp->private_data;
	struct iio_dev_opaque *iio_dev_opaque =
		container_of(inode->i_cdev, struct iio_dev_opaque, chrdev);
	struct iio_dev *indio_dev = &iio_dev_opaque->indio_dev;

	kfree(ib);
	clear_bit(IIO_BUSY_BIT_POS, &iio_dev_opaque->flags);
	iio_device_put(indio_dev);

	return 0;
}

void iio_device_ioctl_handler_register(struct iio_dev *indio_dev,
				       struct iio_ioctl_handler *h)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	list_add_tail(&h->entry, &iio_dev_opaque->ioctl_handlers);
}

void iio_device_ioctl_handler_unregister(struct iio_ioctl_handler *h)
{
	list_del(&h->entry);
}

static long iio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct iio_dev_buffer_pair *ib = filp->private_data;
	struct iio_dev *indio_dev = ib->indio_dev;
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	struct iio_ioctl_handler *h;
	int ret;

	guard(mutex)(&iio_dev_opaque->info_exist_lock);
	/*
	 * The NULL check here is required to prevent crashing when a device
	 * is being removed while userspace would still have open file handles
	 * to try to access this device.
	 */
	if (!indio_dev->info)
		return -ENODEV;

	list_for_each_entry(h, &iio_dev_opaque->ioctl_handlers, entry) {
		ret = h->ioctl(indio_dev, filp, cmd, arg);
		if (ret != IIO_IOCTL_UNHANDLED)
			return ret;
	}

	return -ENODEV;
}

static const struct file_operations iio_buffer_fileops = {
	.owner = THIS_MODULE,
	.llseek = noop_llseek,
	.read = iio_buffer_read_outer_addr,
	.write = iio_buffer_write_outer_addr,
	.poll = iio_buffer_poll_addr,
	.unlocked_ioctl = iio_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.open = iio_chrdev_open,
	.release = iio_chrdev_release,
};

static const struct file_operations iio_event_fileops = {
	.owner = THIS_MODULE,
	.llseek = noop_llseek,
	.unlocked_ioctl = iio_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.open = iio_chrdev_open,
	.release = iio_chrdev_release,
};

static int iio_check_unique_scan_index(struct iio_dev *indio_dev)
{
	int i, j;
	const struct iio_chan_spec *channels = indio_dev->channels;

	if (!(indio_dev->modes & INDIO_ALL_BUFFER_MODES))
		return 0;

	for (i = 0; i < indio_dev->num_channels - 1; i++) {
		if (channels[i].scan_index < 0)
			continue;
		for (j = i + 1; j < indio_dev->num_channels; j++)
			if (channels[i].scan_index == channels[j].scan_index) {
				dev_err(&indio_dev->dev,
					"Duplicate scan index %d\n",
					channels[i].scan_index);
				return -EINVAL;
			}
	}

	return 0;
}

static int iio_check_extended_name(const struct iio_dev *indio_dev)
{
	unsigned int i;

	if (!indio_dev->info->read_label)
		return 0;

	for (i = 0; i < indio_dev->num_channels; i++) {
		if (indio_dev->channels[i].extend_name) {
			dev_err(&indio_dev->dev,
				"Cannot use labels and extend_name at the same time\n");
			return -EINVAL;
		}
	}

	return 0;
}

static const struct iio_buffer_setup_ops noop_ring_setup_ops;

static void iio_sanity_check_avail_scan_masks(struct iio_dev *indio_dev)
{
	unsigned int num_masks, masklength, longs_per_mask;
	const unsigned long *av_masks;
	int i;

	av_masks = indio_dev->available_scan_masks;
	masklength = iio_get_masklength(indio_dev);
	longs_per_mask = BITS_TO_LONGS(masklength);

	/*
	 * The code determining how many available_scan_masks is in the array
	 * will be assuming the end of masks when first long with all bits
	 * zeroed is encountered. This is incorrect for masks where mask
	 * consists of more than one long, and where some of the available masks
	 * has long worth of bits zeroed (but has subsequent bit(s) set). This
	 * is a safety measure against bug where array of masks is terminated by
	 * a single zero while mask width is greater than width of a long.
	 */
	if (longs_per_mask > 1)
		dev_warn(indio_dev->dev.parent,
			 "multi long available scan masks not fully supported\n");

	if (bitmap_empty(av_masks, masklength))
		dev_warn(indio_dev->dev.parent, "empty scan mask\n");

	for (num_masks = 0; *av_masks; num_masks++)
		av_masks += longs_per_mask;

	if (num_masks < 2)
		return;

	av_masks = indio_dev->available_scan_masks;

	/*
	 * Go through all the masks from first to one before the last, and see
	 * that no mask found later from the available_scan_masks array is a
	 * subset of mask found earlier. If this happens, then the mask found
	 * later will never get used because scanning the array is stopped when
	 * the first suitable mask is found. Drivers should order the array of
	 * available masks in the order of preference (presumably the least
	 * costy to access masks first).
	 */
	for (i = 0; i < num_masks - 1; i++) {
		const unsigned long *mask1;
		int j;

		mask1 = av_masks + i * longs_per_mask;
		for (j = i + 1; j < num_masks; j++) {
			const unsigned long *mask2;

			mask2 = av_masks + j * longs_per_mask;
			if (bitmap_subset(mask2, mask1, masklength))
				dev_warn(indio_dev->dev.parent,
					 "available_scan_mask %d subset of %d. Never used\n",
					 j, i);
		}
	}
}

/**
 * iio_active_scan_mask_index - Get index of the active scan mask inside the
 * available scan masks array
 * @indio_dev: the IIO device containing the active and available scan masks
 *
 * Returns: the index or -EINVAL if  active_scan_mask is not set
 */
int iio_active_scan_mask_index(struct iio_dev *indio_dev)

{
	const unsigned long *av_masks;
	unsigned int masklength = iio_get_masklength(indio_dev);
	int i = 0;

	if (!indio_dev->active_scan_mask)
		return -EINVAL;

	/*
	 * As in iio_scan_mask_match and iio_sanity_check_avail_scan_masks,
	 * the condition here do not handle multi-long masks correctly.
	 * It only checks the first long to be zero, and will use such mask
	 * as a terminator even if there was bits set after the first long.
	 *
	 * This should be fine since the available_scan_mask has already been
	 * sanity tested using iio_sanity_check_avail_scan_masks.
	 *
	 * See iio_scan_mask_match and iio_sanity_check_avail_scan_masks for
	 * more details
	 */
	av_masks = indio_dev->available_scan_masks;
	while (*av_masks) {
		if (indio_dev->active_scan_mask == av_masks)
			return i;
		av_masks += BITS_TO_LONGS(masklength);
		i++;
	}

	dev_warn(indio_dev->dev.parent,
		 "active scan mask is not part of the available scan masks\n");
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(iio_active_scan_mask_index);

int __iio_device_register(struct iio_dev *indio_dev, struct module *this_mod)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);
	struct fwnode_handle *fwnode = NULL;
	int ret;

	if (!indio_dev->info)
		return -EINVAL;

	iio_dev_opaque->driver_module = this_mod;

	/* If the calling driver did not initialize firmware node, do it here */
	if (dev_fwnode(&indio_dev->dev))
		fwnode = dev_fwnode(&indio_dev->dev);
	/* The default dummy IIO device has no parent */
	else if (indio_dev->dev.parent)
		fwnode = dev_fwnode(indio_dev->dev.parent);
	device_set_node(&indio_dev->dev, fwnode);

	fwnode_property_read_string(fwnode, "label", &indio_dev->label);

	ret = iio_check_unique_scan_index(indio_dev);
	if (ret < 0)
		return ret;

	ret = iio_check_extended_name(indio_dev);
	if (ret < 0)
		return ret;

	iio_device_register_debugfs(indio_dev);

	ret = iio_buffers_alloc_sysfs_and_mask(indio_dev);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"Failed to create buffer sysfs interfaces\n");
		goto error_unreg_debugfs;
	}

	if (indio_dev->available_scan_masks)
		iio_sanity_check_avail_scan_masks(indio_dev);

	ret = iio_device_register_sysfs(indio_dev);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"Failed to register sysfs interfaces\n");
		goto error_buffer_free_sysfs;
	}
	ret = iio_device_register_eventset(indio_dev);
	if (ret) {
		dev_err(indio_dev->dev.parent,
			"Failed to register event set\n");
		goto error_free_sysfs;
	}
	if (indio_dev->modes & INDIO_ALL_TRIGGERED_MODES)
		iio_device_register_trigger_consumer(indio_dev);

	if ((indio_dev->modes & INDIO_ALL_BUFFER_MODES) &&
		indio_dev->setup_ops == NULL)
		indio_dev->setup_ops = &noop_ring_setup_ops;

	if (iio_dev_opaque->attached_buffers_cnt)
		cdev_init(&iio_dev_opaque->chrdev, &iio_buffer_fileops);
	else if (iio_dev_opaque->event_interface)
		cdev_init(&iio_dev_opaque->chrdev, &iio_event_fileops);

	if (iio_dev_opaque->attached_buffers_cnt || iio_dev_opaque->event_interface) {
		indio_dev->dev.devt = MKDEV(MAJOR(iio_devt), iio_dev_opaque->id);
		iio_dev_opaque->chrdev.owner = this_mod;
	}

	/* assign device groups now; they should be all registered now */
	indio_dev->dev.groups = iio_dev_opaque->groups;

	ret = cdev_device_add(&iio_dev_opaque->chrdev, &indio_dev->dev);
	if (ret < 0)
		goto error_unreg_eventset;

	return 0;

error_unreg_eventset:
	iio_device_unregister_eventset(indio_dev);
error_free_sysfs:
	iio_device_unregister_sysfs(indio_dev);
error_buffer_free_sysfs:
	iio_buffers_free_sysfs_and_mask(indio_dev);
error_unreg_debugfs:
	iio_device_unregister_debugfs(indio_dev);
	return ret;
}
EXPORT_SYMBOL(__iio_device_register);

/**
 * iio_device_unregister() - unregister a device from the IIO subsystem
 * @indio_dev:		Device structure representing the device.
 */
void iio_device_unregister(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	cdev_device_del(&iio_dev_opaque->chrdev, &indio_dev->dev);

	scoped_guard(mutex, &iio_dev_opaque->info_exist_lock) {
		iio_device_unregister_debugfs(indio_dev);

		iio_disable_all_buffers(indio_dev);

		indio_dev->info = NULL;

		iio_device_wakeup_eventset(indio_dev);
		iio_buffer_wakeup_poll(indio_dev);
	}

	iio_buffers_free_sysfs_and_mask(indio_dev);
}
EXPORT_SYMBOL(iio_device_unregister);

static void devm_iio_device_unreg(void *indio_dev)
{
	iio_device_unregister(indio_dev);
}

int __devm_iio_device_register(struct device *dev, struct iio_dev *indio_dev,
			       struct module *this_mod)
{
	int ret;

	ret = __iio_device_register(indio_dev, this_mod);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, devm_iio_device_unreg, indio_dev);
}
EXPORT_SYMBOL_GPL(__devm_iio_device_register);

/**
 * __iio_device_claim_direct - Keep device in direct mode
 * @indio_dev:	the iio_dev associated with the device
 *
 * If the device is in direct mode it is guaranteed to stay
 * that way until __iio_device_release_direct() is called.
 *
 * Use with __iio_device_release_direct().
 *
 * Drivers should only call iio_device_claim_direct().
 *
 * Returns: true on success, false on failure.
 */
bool __iio_device_claim_direct(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	mutex_lock(&iio_dev_opaque->mlock);

	if (iio_buffer_enabled(indio_dev)) {
		mutex_unlock(&iio_dev_opaque->mlock);
		return false;
	}
	return true;
}
EXPORT_SYMBOL_GPL(__iio_device_claim_direct);

/**
 * __iio_device_release_direct - releases claim on direct mode
 * @indio_dev:	the iio_dev associated with the device
 *
 * Release the claim. Device is no longer guaranteed to stay
 * in direct mode.
 *
 * Drivers should only call iio_device_release_direct().
 *
 * Use with __iio_device_claim_direct()
 */
void __iio_device_release_direct(struct iio_dev *indio_dev)
{
	mutex_unlock(&to_iio_dev_opaque(indio_dev)->mlock);
}
EXPORT_SYMBOL_GPL(__iio_device_release_direct);

/**
 * iio_device_claim_buffer_mode - Keep device in buffer mode
 * @indio_dev:	the iio_dev associated with the device
 *
 * If the device is in buffer mode it is guaranteed to stay
 * that way until iio_device_release_buffer_mode() is called.
 *
 * Use with iio_device_release_buffer_mode().
 *
 * Returns: 0 on success, -EBUSY on failure.
 */
int iio_device_claim_buffer_mode(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	mutex_lock(&iio_dev_opaque->mlock);

	if (iio_buffer_enabled(indio_dev))
		return 0;

	mutex_unlock(&iio_dev_opaque->mlock);
	return -EBUSY;
}
EXPORT_SYMBOL_GPL(iio_device_claim_buffer_mode);

/**
 * iio_device_release_buffer_mode - releases claim on buffer mode
 * @indio_dev:	the iio_dev associated with the device
 *
 * Release the claim. Device is no longer guaranteed to stay
 * in buffer mode.
 *
 * Use with iio_device_claim_buffer_mode().
 */
void iio_device_release_buffer_mode(struct iio_dev *indio_dev)
{
	mutex_unlock(&to_iio_dev_opaque(indio_dev)->mlock);
}
EXPORT_SYMBOL_GPL(iio_device_release_buffer_mode);

/**
 * iio_device_get_current_mode() - helper function providing read-only access to
 *				   the opaque @currentmode variable
 * @indio_dev:			   IIO device structure for device
 */
int iio_device_get_current_mode(struct iio_dev *indio_dev)
{
	struct iio_dev_opaque *iio_dev_opaque = to_iio_dev_opaque(indio_dev);

	return iio_dev_opaque->currentmode;
}
EXPORT_SYMBOL_GPL(iio_device_get_current_mode);

subsys_initcall(iio_init);
module_exit(iio_exit);

MODULE_AUTHOR("Jonathan Cameron <jic23@kernel.org>");
MODULE_DESCRIPTION("Industrial I/O core");
MODULE_LICENSE("GPL");
