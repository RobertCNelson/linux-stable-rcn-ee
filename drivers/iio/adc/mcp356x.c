// SPDX-License-Identifier: GPL-2.0+
/*
 * IIO driver for MCP356X/MCP356XR and MCP346X/MCP346XR series ADC chip family
 *
 * Copyright (C) 2022-2023 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Marius Cristea <marius.cristea@microchip.com>
 *
 * Datasheet for MCP3561, MCP3562, MCP3564 can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/MCP3561-2-4-Family-Data-Sheet-DS20006181C.pdf
 * Datasheet for MCP3561R, MCP3562R, MCP3564R can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP3561_2_4R-Data-Sheet-DS200006391C.pdf
 * Datasheet for MCP3461, MCP3462, MCP3464 can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP3461-2-4-Two-Four-Eight-Channel-153.6-ksps-Low-Noise-16-Bit-Delta-Sigma-ADC-Data-Sheet-20006180D.pdf
 * Datasheet for MCP3461R, MCP3462R, MCP3464R can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP3461-2-4R-Family-Data-Sheet-DS20006404C.pdf
 *
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define MCP356X_ADCDATA		0x00
#define MCP356X_CONFIG0		0x01
#define MCP356X_CONFIG1		0x02
#define MCP356X_CONFIG2		0x03
#define MCP356X_CONFIG3		0x04
#define MCP356X_IRQ		0x05
#define MCP356X_MUX		0x06
#define MCP356X_SCAN		0x07
#define MCP356X_TIMER		0x08
#define MCP356X_OFFSETCAL	0x09
#define MCP356X_GAINCAL		0x0A
#define MCP356X_RESERVED_B	0x0B
#define MCP356X_RESERVED_C	0x0C
#define MCP356X_LOCK		0x0D
#define MCP356X_RESERVED_E	0x0E
#define MCP356X_CRCCFG		0x0F

#define MCP356X_FAST_CMD_CTRL	0
#define MCP356X_RD_CTRL		BIT(0)
#define MCP356X_WRT_CTRL	BIT(1)

#define MCP356X_FULL_RESET_CMD	GENMASK(3, 1)

#define MCP3461_HW_ID		BIT(3)
#define MCP3462_HW_ID		0x0009
#define MCP3464_HW_ID		0x000B

#define MCP3561_HW_ID		GENMASK(3, 2)
#define MCP3562_HW_ID		0x000D
#define MCP3564_HW_ID		GENMASK(3, 0)
#define MCP356X_HW_ID_MASK	GENMASK(3, 0)

#define MCP356XR_INT_VREF_MV	2400

/* MUX_VIN Input Selection
 */
#define MCP356X_INTERNAL_VCM	GENMASK(3, 0)
#define MCP356X_TEMP_DIODE_M	GENMASK(3, 1)
#define MCP356X_TEMP_DIODE_P	0b1101
#define MCP356X_REFIN_NEG	GENMASK(3, 2)
#define MCP356X_REFIN_POZ	0b1011
#define MCP356X_RESERVED	0b1010 /* do not use */
#define MCP356X_AVDD		0b1001
#define MCP356X_AGND		BIT(3)
#define MCP356X_CH7		GENMASK(2, 0)
#define MCP356X_CH6		GENMASK(2, 1)
#define MCP356X_CH5		0b0101
#define MCP356X_CH4		BIT(2)
#define MCP356X_CH3		GENMASK(1, 0)
#define MCP356X_CH2		BIT(1)
#define MCP356X_CH1		BIT(0)
#define MCP356X_CH0		0

#define MCP356X_ADC_MODE_MASK			GENMASK(1, 0)

#define MCP356X_ADC_DEFAULT_MODE		0
#define MCP356X_ADC_SHUTDOWN_MODE		BIT(0)
#define MCP356X_ADC_STANDBY			BIT(1)
#define MCP356X_ADC_CONVERSION_MODE		GENMASK(1, 0)

#define MCP356X_DATA_READY_MASK			BIT(6)

#define MCP356X_OVERSAMPLING_RATIO_32		0
#define MCP356X_OVERSAMPLING_RATIO_64		BIT(0)
#define MCP356X_OVERSAMPLING_RATIO_128		BIT(1)
#define MCP356X_OVERSAMPLING_RATIO_256		GENMASK(1, 0)
#define MCP356X_OVERSAMPLING_RATIO_512		BIT(2)
#define MCP356X_OVERSAMPLING_RATIO_1024		0x05
#define MCP356X_OVERSAMPLING_RATIO_2048		GENMASK(2, 1)
#define MCP356X_OVERSAMPLING_RATIO_4096		GENMASK(2, 0)
#define MCP356X_OVERSAMPLING_RATIO_8192		BIT(3)
#define MCP356X_OVERSAMPLING_RATIO_16384	0x09
#define MCP356X_OVERSAMPLING_RATIO_20480	0x0A
#define MCP356X_OVERSAMPLING_RATIO_24576	0x0B
#define MCP356X_OVERSAMPLING_RATIO_40960	0x0C
#define MCP356X_OVERSAMPLING_RATIO_49152	0x0D
#define MCP356X_OVERSAMPLING_RATIO_81920	GENMASK(3, 1)
#define MCP356X_OVERSAMPLING_RATIO_98304	GENMASK(3, 0)

#define MCP356X_OVERSAMPLING_RATIO_MASK		GENMASK(5, 2)
#define MCP356X_OVERSAMPLING_RATIO_SHIFT	0x02

#define MCP356X_HARDWARE_GAIN_MASK		GENMASK(5, 3)
#define MCP356X_HARDWARE_GAIN_SHIFT		0x03
#define MCP356X_DEFAULT_HARDWARE_GAIN		BIT(1)

#define MCP356X_CS_SEL_0_0_uA			0x0
#define MCP356X_CS_SEL_0_9_uA			BIT(0)
#define MCP356X_CS_SEL_3_7_uA			BIT(1)
#define MCP356X_CS_SEL_15_uA			GENMASK(1, 0)

#define MCP356X_CS_SEL_MASK			GENMASK(3, 2)

#define MCP356X_BOOST_CURRENT_x0_50		0
#define MCP356X_BOOST_CURRENT_x0_66		BIT(0)
#define MCP356X_BOOST_CURRENT_x1_00		BIT(1)
#define MCP356X_BOOST_CURRENT_x2_00		GENMASK(1, 0)

#define MCP356X_BOOST_CURRENT_MASK		GENMASK(7, 6)

/* Auto-Zeroing MUX Setting */
#define MCP356X_AZ_MUX_MASK			BIT(2)
/* Auto-Zeroing REF Setting */
#define MCP356X_AZ_REF_MASK			BIT(1)

#define MCP356X_SHARED_DEVATTRS_COUNT		1
#define MCP356X_PARTICULAR_DEVATTRS_COUNT	1

#define MAX_HWGAIN				64000

#define MCP356X_DATA_READY_TIMEOUT_MS		2000

enum mcp356x_ids {
	mcp3461,
	mcp3462,
	mcp3464,
	mcp3461r,
	mcp3462r,
	mcp3464r,
	mcp3561,
	mcp3562,
	mcp3564,
	mcp3561r,
	mcp3562r,
	mcp3564r,
};

static const unsigned int mcp356x_oversampling_avail[16] = {
	[MCP356X_OVERSAMPLING_RATIO_32] = 32,
	[MCP356X_OVERSAMPLING_RATIO_64] = 64,
	[MCP356X_OVERSAMPLING_RATIO_128] = 128,
	[MCP356X_OVERSAMPLING_RATIO_256] = 256,
	[MCP356X_OVERSAMPLING_RATIO_512] = 512,
	[MCP356X_OVERSAMPLING_RATIO_1024] = 1024,
	[MCP356X_OVERSAMPLING_RATIO_2048] = 2048,
	[MCP356X_OVERSAMPLING_RATIO_4096] = 4096,
	[MCP356X_OVERSAMPLING_RATIO_8192] = 8192,
	[MCP356X_OVERSAMPLING_RATIO_16384] = 16384,
	[MCP356X_OVERSAMPLING_RATIO_20480] = 20480,
	[MCP356X_OVERSAMPLING_RATIO_24576] = 24576,
	[MCP356X_OVERSAMPLING_RATIO_40960] = 40960,
	[MCP356X_OVERSAMPLING_RATIO_49152] = 49152,
	[MCP356X_OVERSAMPLING_RATIO_81920] = 81920,
	[MCP356X_OVERSAMPLING_RATIO_98304] = 98304
};

/*
 * Current Source/Sink Selection Bits for Sensor Bias (source on VIN+/sink on VIN-)
 */
static const char * const mcp356x_current_bias_avail[] = {
	[MCP356X_CS_SEL_0_0_uA] = "no_current(default)",
	[MCP356X_CS_SEL_0_9_uA] = "0.9_uA",
	[MCP356X_CS_SEL_3_7_uA] = "3.7_uA",
	[MCP356X_CS_SEL_15_uA] = "15_uA",
};

/*
 * BOOST[1:0]: ADC Bias Current Selection
 */
static const char * const mcp356x_boost_current_avail[] = {
	[MCP356X_BOOST_CURRENT_x0_50] = "x0.5",
	[MCP356X_BOOST_CURRENT_x0_66] = "x0.66",
	[MCP356X_BOOST_CURRENT_x1_00] = "x1_(default)",
	[MCP356X_BOOST_CURRENT_x2_00] = "x2",
};

/*
 * Calibration bias values
 */
static const int mcp356x_calib_bias[] = {
	-8388608,	/* min: -2^23		*/
	 1,		/* step: 1		*/
	 8388607	/* max:  2^23 - 1	*/
};

/*
 * Calibration scale values
 * The Gain Error Calibration register (GAINCAL) is an
 * unsigned 24-bit register that holds the digital gain error
 * calibration value, GAINCAL which could be calculated by
 * GAINCAL (V/V) = (GAINCAL[23:0])/8388608
 * The gain error calibration value range in equivalent voltage is [0; 2-2^(-23)]
 */
static const unsigned int mcp356x_calib_scale[] = {
		0,	/* min:  0		*/
		1,	/* step: 1/8388608	*/
	 16777215	/* max:  2 - 2^(-23)	*/
};

/* Programmable hardware gain x1/3, x1, x2, x4, x8, x16, x32, x64 */
static const int mcp356x_hwgain_frac[] = {
	3,
	10,
	1,
	1,
	2,
	1,
	4,
	1,
	8,
	1,
	16,
	1,
	32,
	1,
	64,
	1
};

static const int mcp356x_hwgain[] = {
	300,
	1000,
	2000,
	4000,
	8000,
	16000,
	32000,
	64000
};

/**
 * struct mcp356x_chip_info - chip specific data
 * @channels:		struct iio_chan_spec matching the device's capabilities
 * @num_channels:	number of channels
 * @int_vref_uv:	internal voltage reference value in microVolts
 * @has_vref:		Does the ADC has an internal voltage reference?
 */
struct mcp356x_chip_info {
	const struct	iio_chan_spec *channels;
	unsigned int	num_channels;
	unsigned int	int_vref_uv;
	bool		has_vref;
};

/**
 * struct mcp356x_state - working data for a ADC device
 * @chip_info:		chip specific data
 * @mcp356x_info:	information about iio device
 * @spi:		SPI device structure
 * @vref:		The regulator device used as a voltage reference in case
 *			external voltage reference is used
 * @vref_mv:		voltage reference value in miliVolts
 * @lock:		mutex to prevent concurrent reads/writes
 * @dev_addr:		hardware device address
 * @oversampling:	the index inside oversampling list of the ADC
 * @hwgain:		the index inside hardware gain list of the ADC
 * @calib_bias:		calibration bias value
 * @calib_scale:	calibration scale value
 * @current_boost_mode:	the index inside current boost list of the ADC
 * @current_bias_mode:	the index inside current bias list of the ADC
 * @auto_zeroing_mux:	set if ADC auto-zeroing algorithm is enabled
 * @auto_zeroing_ref:	set if ADC auto-Zeroing Reference Buffer Setting is enabled
 */
struct mcp356x_state {
	const struct mcp356x_chip_info *chip_info;
	struct iio_info		mcp356x_info;
	struct spi_device	*spi;
	struct regulator	*vref;
	unsigned short		vref_mv;
	struct mutex		lock; /*lock to prevent concurrent reads/writes */
	u8			dev_addr;
	unsigned int		oversampling;
	unsigned int		hwgain;
	int			calib_bias;
	int			calib_scale;
	unsigned int		current_boost_mode;
	unsigned int		current_bias_mode;
	bool			auto_zeroing_mux;
	bool			auto_zeroing_ref;
};

static inline u8 mcp356x_reg_write(u8 chip_addr, u8 reg)
{
	return ((chip_addr << 6) | (reg << 2) | MCP356X_WRT_CTRL);
}

static inline u8 mcp356x_reg_read(u8 chip_addr, u8 reg)
{
	return ((chip_addr << 6) | (reg << 2) | MCP356X_RD_CTRL);
}

static inline u8 mcp356x_reg_fast_cmd(u8 chip_addr, u8 cmd)
{
	return ((chip_addr << 6) | (cmd << 2));
}

static int mcp356x_read(struct mcp356x_state *adc, u8 reg, u32 *val, u8 len)
{
	int ret;
	u8 tmp_reg;

	tmp_reg = mcp356x_reg_read(adc->dev_addr, reg);

	ret = spi_write_then_read(adc->spi, &tmp_reg, 1, val, len);

	be32_to_cpus(val);
	*val >>= ((4 - len) * 8);

	return ret;
}

static int mcp356x_write(struct mcp356x_state *adc, u8 reg, u32 val, u8 len)
{
	val |= (mcp356x_reg_write(adc->dev_addr, reg) << (len * 8));
	val <<= (3 - len) * 8;
	cpu_to_be32s(&val);

	return spi_write(adc->spi, &val, len + 1);
}

static int mcp356x_fast_cmd(struct mcp356x_state *adc, u8 fast_cmd)
{
	u8 val;

	val = mcp356x_reg_fast_cmd(adc->dev_addr, fast_cmd);

	return spi_write(adc->spi, &val, 1);
}

static int mcp356x_update(struct mcp356x_state *adc, u8 reg, u32 mask, u32 val,
			  u8 len)
{
	u32 tmp;
	int ret;

	ret = mcp356x_read(adc, reg, &tmp, len);

	if (ret == 0) {
		val &= mask;
		val |= tmp & ~mask;
		ret = mcp356x_write(adc, reg, val, len);
	}

	return ret;
}

/* Custom IIO Device Attributes */
static int mcp356x_set_current_boost_mode(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  unsigned int mode)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);
	int ret;

	dev_dbg(&indio_dev->dev, "%s: %d\n", __func__, mode);

	mutex_lock(&adc->lock);
	ret = mcp356x_update(adc, MCP356X_CONFIG2, MCP356X_BOOST_CURRENT_MASK,
			     mode, 1);

	if (ret)
		dev_err(&indio_dev->dev, "Failed to configure CONFIG2 register\n");
	else
		adc->current_boost_mode = mode;

	mutex_unlock(&adc->lock);

	return ret;
}

static int mcp356x_get_current_boost_mode(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);

	return adc->current_boost_mode;
}

static int mcp356x_set_current_bias_mode(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 unsigned int mode)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);
	int ret;

	dev_dbg(&indio_dev->dev, "%s: %d\n", __func__, mode);

	mutex_lock(&adc->lock);
	ret = mcp356x_update(adc, MCP356X_CONFIG0, MCP356X_CS_SEL_MASK, mode, 1);

	if (ret)
		dev_err(&indio_dev->dev, "Failed to configure CONFIG0 register\n");
	else
		adc->current_bias_mode = mode;

	mutex_unlock(&adc->lock);

	return ret;
}

static int mcp356x_get_current_bias_mode(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);

	return adc->current_bias_mode;
}

static const struct iio_enum mcp356x_current_boost_mode_enum = {
	.items = mcp356x_boost_current_avail,
	.num_items = ARRAY_SIZE(mcp356x_boost_current_avail),
	.set = mcp356x_set_current_boost_mode,
	.get = mcp356x_get_current_boost_mode,
};

static const struct iio_enum mcp356x_current_bias_mode_enum = {
	.items = mcp356x_current_bias_avail,
	.num_items = ARRAY_SIZE(mcp356x_current_bias_avail),
	.set = mcp356x_set_current_bias_mode,
	.get = mcp356x_get_current_bias_mode,
};

static const struct iio_chan_spec_ext_info mcp356x_ext_info[] = {
	IIO_ENUM("boost_current", IIO_SHARED_BY_ALL, &mcp356x_current_boost_mode_enum),
	{
		.name = "boost_current_available",
		.shared = IIO_SHARED_BY_ALL,
		.read = iio_enum_available_read,
		.private = (uintptr_t)&mcp356x_current_boost_mode_enum,
	},
	IIO_ENUM("current_bias", IIO_SHARED_BY_ALL, &mcp356x_current_bias_mode_enum),
	{
		.name = "current_bias_available",
		.shared = IIO_SHARED_BY_ALL,
		.read = iio_enum_available_read,
		.private = (uintptr_t)&mcp356x_current_bias_mode_enum,
	},
	{}
};

static ssize_t mcp356x_auto_zeroing_mux_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp356x_state *adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", adc->auto_zeroing_mux);
}

static ssize_t mcp356x_auto_zeroing_mux_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp356x_state *adc = iio_priv(indio_dev);
	bool auto_zero;
	int ret;

	ret = kstrtobool(buf, &auto_zero);
	if (ret)
		return ret;

	mutex_lock(&adc->lock);
	ret = mcp356x_update(adc, MCP356X_CONFIG2, MCP356X_AZ_MUX_MASK,
			     (u32)auto_zero, 1);

	if (ret)
		dev_err(&indio_dev->dev, "Failed to update CONFIG2 register\n");
	else
		adc->auto_zeroing_mux = auto_zero;

	mutex_unlock(&adc->lock);

	return ret ? ret : len;
}

static ssize_t mcp356x_auto_zeroing_ref_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp356x_state *adc = iio_priv(indio_dev);

	return sysfs_emit(buf, "%d\n", adc->auto_zeroing_ref);
}

static ssize_t mcp356x_auto_zeroing_ref_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct mcp356x_state *adc = iio_priv(indio_dev);
	bool auto_zero;
	int ret;

	ret = kstrtobool(buf, &auto_zero);
	if (ret)
		return ret;

	mutex_lock(&adc->lock);
	ret = mcp356x_update(adc, MCP356X_CONFIG2, MCP356X_AZ_REF_MASK,
			     (u32)auto_zero, 1);

	if (ret)
		dev_err(&indio_dev->dev, "Failed to update CONFIG2 register\n");
	else
		adc->auto_zeroing_ref = auto_zero;

	mutex_unlock(&adc->lock);

	return ret ? ret : len;
}

#define MCP356X_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)

static IIO_DEVICE_ATTR(enable_auto_zeroing_ref, 0644,
		       mcp356x_auto_zeroing_ref_show,
		       mcp356x_auto_zeroing_ref_store, 0);

static struct attribute *mcp356x_particular_attributes[] = {
	MCP356X_DEV_ATTR(enable_auto_zeroing_ref),
	NULL
};

static IIO_DEVICE_ATTR(enable_auto_zeroing_mux, 0644,
		       mcp356x_auto_zeroing_mux_show,
		       mcp356x_auto_zeroing_mux_store, 0);

static struct attribute *mcp356x_shared_attributes[] = {
	MCP356X_DEV_ATTR(enable_auto_zeroing_mux),
	NULL,
};

static int mcp356x_prep_custom_attributes(struct mcp356x_state *adc,
					  struct iio_dev *indio_dev)
{
	int i;
	struct attribute **mcp356x_custom_attr;
	struct attribute_group *mcp356x_group;

	mcp356x_group = devm_kzalloc(&adc->spi->dev, sizeof(*mcp356x_group), GFP_KERNEL);

	if (!mcp356x_group)
		return (-ENOMEM);

	mcp356x_custom_attr = devm_kzalloc(&adc->spi->dev, (MCP356X_SHARED_DEVATTRS_COUNT +
		MCP356X_PARTICULAR_DEVATTRS_COUNT + 1) * sizeof(struct attribute *),
		GFP_KERNEL);

	if (!mcp356x_custom_attr)
		return (-ENOMEM);

	for (i = 0; i < MCP356X_SHARED_DEVATTRS_COUNT; i++)
		mcp356x_custom_attr[i] = mcp356x_shared_attributes[i];

	if (adc->chip_info->has_vref) {
		dev_dbg(&indio_dev->dev, "Setup custom attr for R variant\n");
		for (i = 0; i < MCP356X_PARTICULAR_DEVATTRS_COUNT; i++)
			mcp356x_custom_attr[MCP356X_SHARED_DEVATTRS_COUNT + i] =
				mcp356x_particular_attributes[i];
	}

	mcp356x_group->attrs = mcp356x_custom_attr;
	adc->mcp356x_info.attrs = mcp356x_group;

	return 0;
}

#define MCP356X_V_CHANNEL(index, addr, depth) {					\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (index),							\
	.address = (((addr) << 4) | MCP356X_AGND),				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all  = BIT(IIO_CHAN_INFO_HARDWAREGAIN)	|	\
				BIT(IIO_CHAN_INFO_CALIBBIAS)		|	\
				BIT(IIO_CHAN_INFO_CALIBSCALE)		|	\
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_CALIBSCALE) |	\
				BIT(IIO_CHAN_INFO_HARDWAREGAIN)		|	\
				BIT(IIO_CHAN_INFO_CALIBBIAS)		|	\
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.ext_info = mcp356x_ext_info,						\
	.scan_index = 0,							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = depth,						\
		.storagebits = 32,						\
		.endianness = IIO_BE,						\
	},									\
}

#define MCP356X_T_CHAN(depth) {							\
	.type = IIO_TEMP,							\
	.channel = 0,								\
	.address = ((MCP356X_TEMP_DIODE_P << 4) | MCP356X_TEMP_DIODE_M),	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all  = BIT(IIO_CHAN_INFO_HARDWAREGAIN)	|	\
			BIT(IIO_CHAN_INFO_CALIBBIAS)			|	\
			BIT(IIO_CHAN_INFO_CALIBSCALE)			|	\
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),			\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_CALIBSCALE) |	\
			BIT(IIO_CHAN_INFO_CALIBBIAS)			|	\
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),			\
	.ext_info = mcp356x_ext_info,						\
	.scan_index = 0,							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = depth,						\
		.storagebits = 32,						\
		.endianness = IIO_BE,						\
	},									\
}

#define MCP356X_V_CHANNEL_DIFF(chan1, chan2, addr, depth) {			\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (chan1),							\
	.channel2 = (chan2),							\
	.address = (addr),							\
	.differential = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all  = BIT(IIO_CHAN_INFO_HARDWAREGAIN)	|	\
			BIT(IIO_CHAN_INFO_CALIBBIAS)			|	\
			BIT(IIO_CHAN_INFO_CALIBSCALE)			|	\
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),			\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_CALIBSCALE) |	\
			BIT(IIO_CHAN_INFO_CALIBBIAS)			|	\
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),			\
	.ext_info = mcp356x_ext_info,						\
	.scan_index = 0,							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = depth,						 \
		.storagebits = 32,						\
		.endianness = IIO_BE,						\
	},									\
}

#define MCP3561_CHANNELS(depth) {			\
	MCP356X_V_CHANNEL(0, 0, depth),			\
	MCP356X_V_CHANNEL(1, 1, depth),			\
	MCP356X_V_CHANNEL_DIFF(0, 1, 0x01, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 0, 0x10, depth),	\
	MCP356X_T_CHAN(depth)				\
}

#define MCP3562_CHANNELS(depth) {			\
	MCP356X_V_CHANNEL(0, 0, depth),			\
	MCP356X_V_CHANNEL(1, 1, depth),			\
	MCP356X_V_CHANNEL(2, 2, depth),			\
	MCP356X_V_CHANNEL(3, 3, depth),			\
	MCP356X_T_CHAN(depth),				\
	MCP356X_V_CHANNEL_DIFF(0, 1, 0x01, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 0, 0x10, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 2, 0x02, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 3, 0x03, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 2, 0x12, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 3, 0x13, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 3, 0x23, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 0, 0x20, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 0, 0x30, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 1, 0x21, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 1, 0x31, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 2, 0x32, depth),	\
}

#define MCP3564_CHANNELS(depth) {			\
	MCP356X_V_CHANNEL(0, 0, depth),			\
	MCP356X_V_CHANNEL(1, 1, depth),			\
	MCP356X_V_CHANNEL(2, 2, depth),			\
	MCP356X_V_CHANNEL(3, 3, depth),			\
	MCP356X_V_CHANNEL(4, 4, depth),			\
	MCP356X_V_CHANNEL(5, 5, depth),			\
	MCP356X_V_CHANNEL(6, 6, depth),			\
	MCP356X_V_CHANNEL(7, 7, depth),			\
	MCP356X_T_CHAN(depth),				\
	MCP356X_V_CHANNEL_DIFF(0, 1, 0x01, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 0, 0x10, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 2, 0x02, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 3, 0x03, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 2, 0x12, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 3, 0x13, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 3, 0x23, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 0, 0x20, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 0, 0x30, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 1, 0x21, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 1, 0x31, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 2, 0x32, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 4, 0x04, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 5, 0x05, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 6, 0x06, depth),	\
	MCP356X_V_CHANNEL_DIFF(0, 7, 0x07, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 4, 0x14, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 5, 0x15, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 6, 0x16, depth),	\
	MCP356X_V_CHANNEL_DIFF(1, 7, 0x17, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 4, 0x24, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 5, 0x25, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 6, 0x26, depth),	\
	MCP356X_V_CHANNEL_DIFF(2, 7, 0x27, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 4, 0x34, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 5, 0x35, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 6, 0x36, depth),	\
	MCP356X_V_CHANNEL_DIFF(3, 7, 0x37, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 5, 0x45, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 6, 0x46, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 7, 0x47, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 6, 0x56, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 7, 0x57, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 7, 0x67, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 0, 0x40, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 0, 0x50, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 0, 0x60, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 0, 0x70, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 1, 0x41, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 1, 0x51, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 1, 0x61, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 1, 0x71, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 2, 0x42, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 2, 0x52, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 2, 0x62, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 2, 0x72, depth),	\
	MCP356X_V_CHANNEL_DIFF(4, 3, 0x43, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 3, 0x53, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 3, 0x63, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 3, 0x73, depth),	\
	MCP356X_V_CHANNEL_DIFF(5, 4, 0x54, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 4, 0x64, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 4, 0x74, depth),	\
	MCP356X_V_CHANNEL_DIFF(6, 5, 0x65, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 5, 0x75, depth),	\
	MCP356X_V_CHANNEL_DIFF(7, 6, 0x76, depth)	\
}

static const struct iio_chan_spec mcp3461_channels[] = MCP3561_CHANNELS(16);
static const struct iio_chan_spec mcp3462_channels[] = MCP3562_CHANNELS(16);
static const struct iio_chan_spec mcp3464_channels[] = MCP3564_CHANNELS(16);
static const struct iio_chan_spec mcp3561_channels[] = MCP3561_CHANNELS(24);
static const struct iio_chan_spec mcp3562_channels[] = MCP3562_CHANNELS(24);
static const struct iio_chan_spec mcp3564_channels[] = MCP3564_CHANNELS(24);

static const struct mcp356x_chip_info mcp356x_chip_infos_tbl[] = {
	[mcp3461] = {
		.channels = mcp3461_channels,
		.num_channels = ARRAY_SIZE(mcp3461_channels),
		.int_vref_uv = 0,
		.has_vref = false
	},
	[mcp3462] = {
		.channels = mcp3462_channels,
		.num_channels = ARRAY_SIZE(mcp3462_channels),
		.int_vref_uv = 0,
		.has_vref = false
	},
	[mcp3464] = {
		.channels = mcp3464_channels,
		.num_channels = ARRAY_SIZE(mcp3464_channels),
		.int_vref_uv = 0,
		.has_vref = false
	},
	[mcp3461r] = {
		.channels = mcp3461_channels,
		.num_channels = ARRAY_SIZE(mcp3461_channels),
		.int_vref_uv = MCP356XR_INT_VREF_MV,
		.has_vref = true
	},
	[mcp3462r] = {
		.channels = mcp3462_channels,
		.num_channels = ARRAY_SIZE(mcp3462_channels),
		.int_vref_uv = MCP356XR_INT_VREF_MV,
		.has_vref = true
	},
	[mcp3464r] = {
		.channels = mcp3464_channels,
		.num_channels = ARRAY_SIZE(mcp3464_channels),
		.int_vref_uv = MCP356XR_INT_VREF_MV,
		.has_vref = true
	},
	[mcp3561] = {
		.channels = mcp3561_channels,
		.num_channels = ARRAY_SIZE(mcp3561_channels),
		.int_vref_uv = 0,
		.has_vref = false
	},
	[mcp3562] = {
		.channels = mcp3562_channels,
		.num_channels = ARRAY_SIZE(mcp3562_channels),
		.int_vref_uv = 0,
		.has_vref = false
	},
	[mcp3564] = {
		.channels = mcp3564_channels,
		.num_channels = ARRAY_SIZE(mcp3564_channels),
		.int_vref_uv = 0,
		.has_vref = false
	},
	[mcp3561r] = {
		.channels = mcp3561_channels,
		.num_channels = ARRAY_SIZE(mcp3561_channels),
		.int_vref_uv = MCP356XR_INT_VREF_MV,
		.has_vref = true
	},
	[mcp3562r] = {
		.channels = mcp3562_channels,
		.num_channels = ARRAY_SIZE(mcp3562_channels),
		.int_vref_uv = MCP356XR_INT_VREF_MV,
		.has_vref = true
	},
	[mcp3564r] = {
		.channels = mcp3564_channels,
		.num_channels = ARRAY_SIZE(mcp3564_channels),
		.int_vref_uv = MCP356XR_INT_VREF_MV,
		.has_vref = true
	},
};

static int mcp356x_read_single_value(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *channel,
				     int *val)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);
	int ret, tmp, ret_read = 0;

	/* Configure MUX register with the requested channel */
	ret = mcp356x_write(adc, MCP356X_MUX, channel->address, 1);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to configure MUX register\n");
		return ret;
	}

	/* ADC Conversion starts by writing ADC_MODE[1:0] = 11 to CONFIG0[1:0] =  */
	ret = mcp356x_update(adc, MCP356X_CONFIG0, MCP356X_ADC_MODE_MASK,
			     MCP356X_ADC_CONVERSION_MODE, 1);
	if (ret) {
		dev_err(&indio_dev->dev,
			"Failed to configure CONFIG0 register\n");
		return ret;
	}

	/*
	 * Check if the conversion is ready. If not, wait a little bit, and
	 * in case of timeout exit with an error.
	 */

	ret = read_poll_timeout(mcp356x_read, ret_read,
				ret_read || !(tmp & MCP356X_DATA_READY_MASK),
				1000, MCP356X_DATA_READY_TIMEOUT_MS * 1000, true,
				adc, MCP356X_IRQ, &tmp, 1);

	/* failed to read status register */
	if (ret_read)
		return ret;

	if (ret)
		return -ETIMEDOUT;

	if (tmp & MCP356X_DATA_READY_MASK)
		/* failing to finish conversion */
		return -EBUSY;

	ret = mcp356x_read(adc, MCP356X_ADCDATA, &tmp, 4);
	if (ret)
		return ret;

	*val = tmp;

	return ret;
}

static int mcp356x_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel,
			      const int **vals, int *type,
			      int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*type = IIO_VAL_INT;
		*vals = mcp356x_oversampling_avail;
		*length = ARRAY_SIZE(mcp356x_oversampling_avail);
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*type = IIO_VAL_FRACTIONAL;
		*length = ARRAY_SIZE(mcp356x_hwgain_frac);
		*vals = mcp356x_hwgain_frac;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_CALIBBIAS:
		*vals = mcp356x_calib_bias;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_CALIBSCALE:
		*vals = mcp356x_calib_scale;
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int mcp356x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *channel,
			    int *val, int *val2, long mask)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);
	int ret;

	mutex_lock(&adc->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = mcp356x_read_single_value(indio_dev, channel, val);
		if (ret)
			ret = -EINVAL;
		else
			ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = adc->vref_mv;
		*val2 = channel->scan_type.realbits - 1;
		ret = IIO_VAL_FRACTIONAL_LOG2;
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = mcp356x_oversampling_avail[adc->oversampling];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = mcp356x_hwgain_frac[2 * adc->hwgain];
		*val2 = mcp356x_hwgain_frac[(2 * adc->hwgain) + 1];
		ret = IIO_VAL_FRACTIONAL;
		break;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = adc->calib_bias;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		*val = adc->calib_scale;
		ret = IIO_VAL_INT;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&adc->lock);

	return ret;
}

static int mcp356x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *channel, int val,
			     int val2, long mask)
{
	struct mcp356x_state *adc = iio_priv(indio_dev);
	int tmp;
	int ret = -EINVAL;

	mutex_lock(&adc->lock);
	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		if (val < mcp356x_calib_bias[0] && val > mcp356x_calib_bias[2])
			goto out;

		adc->calib_bias = val;
		ret = mcp356x_write(adc, MCP356X_OFFSETCAL, val, 3);
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val < mcp356x_calib_bias[0] && val > mcp356x_calib_bias[2])
			goto out;

		adc->calib_scale = val;
		ret = mcp356x_write(adc, MCP356X_GAINCAL, val, 3);
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (val < 0)
			goto out;

		adc->oversampling = find_closest(val, mcp356x_oversampling_avail,
						 ARRAY_SIZE(mcp356x_oversampling_avail));

		dev_dbg(&adc->spi->dev,
			"IIO_CHAN_INFO_OVERSAMPLING_RATIO index %d\n",
			adc->oversampling);

		ret = mcp356x_update(adc, MCP356X_CONFIG1, MCP356X_OVERSAMPLING_RATIO_MASK,
				     (adc->oversampling << MCP356X_OVERSAMPLING_RATIO_SHIFT),
				     1);
		if (ret)
			dev_err(&indio_dev->dev,
				"Failed to configure CONFIG1 register\n");

		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		/*
		 * calculate gain from read values.
		 * avoid using fractional numbers so
		 * multiply the value with 1000. In case of x1/3 gain
		 * the tmp will be 300
		 */
		tmp = ((val * 1000000) + val2) / 1000;
		if (tmp < 1 || tmp > MAX_HWGAIN)
			goto out;

		adc->hwgain = find_closest(tmp, mcp356x_hwgain,
					   ARRAY_SIZE(mcp356x_hwgain));

		dev_dbg(&adc->spi->dev,
			"IIO_CHAN_INFO_HARDWAREGAIN Gain:%d; index %d\n",
			tmp, adc->hwgain);

		/* Update GAIN in CONFIG2[5:3] -> GAIN[2:0]*/
		ret = mcp356x_update(adc, MCP356X_CONFIG2, MCP356X_HARDWARE_GAIN_MASK,
				     (adc->hwgain << MCP356X_HARDWARE_GAIN_SHIFT), 1);
		if (ret)
			dev_err(&indio_dev->dev,
				"Failed to configure CONFIG0 register\n");
		break;
	}

out:
	mutex_unlock(&adc->lock);

	return ret;
}

static int mcp356x_config(struct mcp356x_state *adc)
{
	int ret = 0;
	unsigned int tmp;

	dev_dbg(&adc->spi->dev, "%s: Start config...\n", __func__);

	/*
	 * The address is set on a per-device basis by fuses in the factory,
	 * configured on request. If not requested, the fuses are set for 0x1.
	 * The device address is part of the device markings to avoid
	 * potential confusion. This address is coded on two bits, so four possible
	 * addresses are available when multiple devices are present on the same
	 * SPI bus with only one Chip Select line for all devices.
	 */
	device_property_read_u32(&adc->spi->dev, "microchip,hw-device-address", &tmp);

	if (tmp > 3) {
		dev_err_probe(&adc->spi->dev, tmp,
			      "invalid device address. Must be in range 0-3.\n");
		return -EINVAL;
	}

	adc->dev_addr = 0xff & tmp;

	dev_dbg(&adc->spi->dev, "use device address %i\n", adc->dev_addr);

	ret = mcp356x_read(adc, MCP356X_RESERVED_E, &tmp, 2);

	if (ret == 0) {
		switch (tmp & MCP356X_HW_ID_MASK) {
		case MCP3461_HW_ID:
			dev_dbg(&adc->spi->dev, "Found MCP3461 chip\n");
			break;
		case MCP3462_HW_ID:
			dev_dbg(&adc->spi->dev, "Found MCP3462 chip\n");
			break;
		case MCP3464_HW_ID:
			dev_dbg(&adc->spi->dev, "Found MCP3464 chip\n");
			break;
		case MCP3561_HW_ID:
			dev_dbg(&adc->spi->dev, "Found MCP3561 chip\n");
			break;
		case MCP3562_HW_ID:
			dev_dbg(&adc->spi->dev, "Found MCP3562 chip\n");
			break;
		case MCP3564_HW_ID:
			dev_dbg(&adc->spi->dev, "Found MCP3564 chip\n");
			break;
		default:
			dev_err_probe(&adc->spi->dev, tmp,
				      "Unknown chip found\n");
			return -EINVAL;
		}
	} else {
		return ret;
	}

	/* Command sequence that ensures a recovery with
	 * the desired settings in any cases of loss-of-power scenario.
	 */

	/* Write LOCK register to 0xA5 (Write Access Password)
	 * Write access is allowed on the full register map.
	 */
	ret = mcp356x_write(adc, MCP356X_LOCK, 0x000000A5, 1);
	if (ret)
		return ret;

	/* Write IRQ register to 0x03 */
	/* IRQ --> IRQ Mode = Hi-Z IRQ Output  --> (0b00000011).
	 * IRQ = 0x00000003
	 */
	ret = mcp356x_write(adc, MCP356X_IRQ, 0x00000003, 1);
	if (ret)
		return ret;

	/* Device Full Reset Fast Command */
	ret = mcp356x_fast_cmd(adc, MCP356X_FULL_RESET_CMD);

	/* wait 1ms for the chip to restart after a full reset */
	mdelay(1);

	/* Reconfigure the ADC chip  */

	/* GAINCAL --> Disabled.
	 * Default value is GAINCAL = 0x00800000; which provides a gain of 1x
	 */
	ret = mcp356x_write(adc, MCP356X_GAINCAL, 0x00800000, 3);
	if (ret)
		return ret;

	adc->calib_scale = 0x00800000;

	/* OFFSETCAL --> 0 Counts of Offset Cancellation
	 * (Measured offset is negative).
	 * OFFSETCAL = 0x0
	 */
	ret = mcp356x_write(adc, MCP356X_OFFSETCAL, 0x00000000, 3);
	if (ret)
		return ret;

	/* TIMER --> Disabled.
	 * TIMER = 0x00000000
	 */
	ret = mcp356x_write(adc, MCP356X_TIMER, 0x00000000, 3);
	if (ret)
		return ret;

	/* SCAN --> Disabled.
	 * SCAN = 0x00000000
	 */
	ret = mcp356x_write(adc, MCP356X_SCAN, 0x00000000, 3);
	if (ret)
		return ret;

	/* MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
	 * MUX = 0x00000001
	 */
	ret = mcp356x_write(adc, MCP356X_MUX, 0x00000001, 1);
	if (ret)
		return ret;

	/* IRQ --> IRQ Mode = Hi-Z IRQ Output  --> (0b00000011).
	 * IRQ = 0x00000003
	 */
	ret = mcp356x_write(adc, MCP356X_IRQ, 0x00000003, 1);
	if (ret)
		return ret;

	/* CONFIG3
	 * Conv. Mod = One-Shot/Standby,
	 * FORMAT = 32-bit (right justified data): SGN extension + ADC data,
	 * CRC_FORMAT = 16b, CRC-COM = Disabled,
	 * OFFSETCAL = Enabled, GAINCAL = Enabled --> (10100011).
	 * CONFIG3 = 0x000000A3
	 *
	 */
	ret = mcp356x_write(adc, MCP356X_CONFIG3, 0x000000A3, 1);
	if (ret)
		return ret;

	/* CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 1 --> (0b10001101).
	 * CONFIG2 = 0x0000008D
	 */
	ret = mcp356x_write(adc, MCP356X_CONFIG2, 0x0000008D, 1);
	if (ret)
		return ret;

	adc->hwgain = 0x01;
	adc->auto_zeroing_mux = true;
	adc->auto_zeroing_ref = false;
	adc->current_boost_mode = MCP356X_BOOST_CURRENT_x1_00;

	/* CONFIG1 --> AMCLK = MCLK, OSR = 98304 --> (0b00111100).
	 * CONFIG1 = 0x0000003C
	 */
	ret = mcp356x_write(adc, MCP356X_CONFIG1, 0x0000003C, 1);
	if (ret)
		return ret;

	adc->oversampling = 0x0F;

	if (!adc->vref) {
		/* CONFIG0 --> VREF_SEL = Internal Voltage Reference 2.4v
		 * CLK_SEL = INTOSC w/o CLKOUT, CS_SEL = No Bias,
		 * ADC_MODE = Standby Mode --> (0b11100010).
		 * CONFIG0 = 0x000000E2
		 */
		ret = mcp356x_write(adc, MCP356X_CONFIG0, 0x000000E2, 1);

		dev_dbg(&adc->spi->dev, "%s: Using internal Vref\n",
			__func__);
		adc->vref_mv = MCP356XR_INT_VREF_MV;

	} else {
		/* CONFIG0 --> CLK_SEL = INTOSC w/o CLKOUT, CS_SEL = No Bias,
		 * ADC_MODE = Standby Mode --> (0b01100010).
		 * CONFIG0 = 0x000000E2
		 */
		ret = mcp356x_write(adc, MCP356X_CONFIG0, 0x00000062, 1);
	}
	adc->current_bias_mode = MCP356X_CS_SEL_0_0_uA;

	return ret;
}

static int mcp356x_probe(struct spi_device *spi)
{
	int ret, device_index;
	struct iio_dev *indio_dev;
	struct mcp356x_state *adc;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev) {
		dev_err_probe(&indio_dev->dev, PTR_ERR(indio_dev),
			      "Can't allocate iio device\n");
		return -ENOMEM;
	}

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	dev_dbg(&adc->spi->dev, "%s: probe(spi = 0x%p)\n", __func__, spi);

	adc->vref = devm_regulator_get_optional(&adc->spi->dev, "vref");
	if (IS_ERR(adc->vref)) {
		if (PTR_ERR(adc->vref) == -ENODEV) {
			adc->vref = NULL;
			dev_dbg(&adc->spi->dev, "%s: Using internal Vref\n",
				__func__);
		} else {
			dev_err_probe(&adc->spi->dev, PTR_ERR(adc->vref),
				      "failed to get regulator\n");
			return PTR_ERR(adc->vref);
		}
	} else {
		ret = regulator_enable(adc->vref);
		if (ret)
			return ret;

		dev_dbg(&adc->spi->dev, "%s: Using External Vref\n",
			__func__);

		ret = regulator_get_voltage(adc->vref);
		if (ret < 0) {
			dev_err_probe(&adc->spi->dev, ret,
				      "Failed to read vref regulator\n");
			goto error_disable_reg;
		}

		adc->vref_mv = ret / 1000;
	}

	spi_set_drvdata(spi, indio_dev);
	device_index = spi_get_device_id(spi)->driver_data;
	adc->chip_info = &mcp356x_chip_infos_tbl[device_index];

	adc->mcp356x_info.read_raw = mcp356x_read_raw;
	adc->mcp356x_info.write_raw = mcp356x_write_raw;
	adc->mcp356x_info.read_avail = mcp356x_read_avail;

	ret = mcp356x_prep_custom_attributes(adc, indio_dev);
	if (ret) {
		dev_err_probe(&adc->spi->dev, ret,
			      "Can't configure custom attributes for MCP356X device\n");
		goto error_disable_reg;
	}

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adc->mcp356x_info;

	indio_dev->channels = adc->chip_info->channels;
	indio_dev->num_channels = adc->chip_info->num_channels;
	indio_dev->masklength = adc->chip_info->num_channels - 1;

	/* initialize the chip access mutex */
	mutex_init(&adc->lock);

	/* Do any chip specific initialization, e.g:
	 * read/write some registers
	 * enable/disable certain channels
	 * change the sampling rate to the requested value
	 */
	ret = mcp356x_config(adc);
	if (ret) {
		dev_err_probe(&adc->spi->dev, ret,
			      "Can't configure MCP356X device\n");
		goto error_disable_reg;
	}

	dev_dbg(&adc->spi->dev, "%s: Vref (mV): %d\n", __func__, adc->vref_mv);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret) {
		dev_err_probe(&adc->spi->dev, ret,
			      "Can't register IIO device\n");
		goto error_disable_reg;
	}

	return 0;

error_disable_reg:
	if (adc->vref)
		regulator_disable(adc->vref);

	return ret;
}

static void mcp356x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct mcp356x_state *adc = iio_priv(indio_dev);

	if (adc->vref)
		regulator_disable(adc->vref);
}

static const struct of_device_id mcp356x_dt_ids[] = {
	{ .compatible = "microchip,mcp3461" },
	{ .compatible = "microchip,mcp3462" },
	{ .compatible = "microchip,mcp3464" },
	{ .compatible = "microchip,mcp3461r" },
	{ .compatible = "microchip,mcp3462r" },
	{ .compatible = "microchip,mcp3464r" },
	{ .compatible = "microchip,mcp3561" },
	{ .compatible = "microchip,mcp3562" },
	{ .compatible = "microchip,mcp3564" },
	{ .compatible = "microchip,mcp3561r" },
	{ .compatible = "microchip,mcp3562r" },
	{ .compatible = "microchip,mcp3564r" },
	{ }
};
MODULE_DEVICE_TABLE(of, mcp356x_dt_ids);

static const struct spi_device_id mcp356x_id[] = {
	{ "mcp3461",  mcp3461 },
	{ "mcp3462",  mcp3462 },
	{ "mcp3464",  mcp3464 },
	{ "mcp3461r", mcp3461r },
	{ "mcp3462r", mcp3462r },
	{ "mcp3464r", mcp3464r },
	{ "mcp3561",  mcp3561 },
	{ "mcp3562",  mcp3562 },
	{ "mcp3564",  mcp3564 },
	{ "mcp3561r", mcp3561r },
	{ "mcp3562r", mcp3562r },
	{ "mcp3564r", mcp3564r },
	{ }
};
MODULE_DEVICE_TABLE(spi, mcp356x_id);

static struct spi_driver mcp356x_driver = {
	.driver = {
		.name = "mcp3564",
		.of_match_table = mcp356x_dt_ids,
	},
	.probe = mcp356x_probe,
	.remove = mcp356x_remove,
	.id_table = mcp356x_id,
};

module_spi_driver(mcp356x_driver);

MODULE_AUTHOR("Marius Cristea <marius.cristea@microchip.com>");
MODULE_DESCRIPTION("Microchip MCP346x/MCP346xR and MCP356x/MCP346xR ADCs");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1.2");
