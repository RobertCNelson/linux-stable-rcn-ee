// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2014 Angelo Compagnucci <angelo.compagnucci@gmail.com>
 *
 * Driver for Texas Instruments' ADC128S052, ADC122S021 and ADC124S021 ADC chip.
 * Datasheets can be found here:
 * https://www.ti.com/lit/ds/symlink/adc128s052.pdf
 * https://www.ti.com/lit/ds/symlink/adc122s021.pdf
 * https://www.ti.com/lit/ds/symlink/adc124s021.pdf
 *
 * The adcxx4s communicates with a host processor via an SPI/Microwire Bus
 * interface. This driver supports the whole family of devices with a name
 * ADC<bb><c>S<sss>, where
 * bb is the resolution in number of bits (8, 10, 12)
 * c is the number of channels (1, 2, 4, 8)
 * sss is the maximum conversion speed (021 for 200 kSPS, 051 for 500 kSPS
 * and 101 for 1 MSPS)
 *
 * Complete datasheets are available at TI's website here:
 *   https://www.ti.com/lit/gpn/adc<bb><c>s<sss>.pdf
 *
 * 8, 10, and 12 bits converters send 12-bit data with
 * unavailable bits set to 0 in LSB.
 * Shift is calculated per resolution and used in scaling and
 * raw data read.
 */

#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

struct adc128_configuration {
	const struct iio_chan_spec	*channels;
	u8				num_channels;
};

struct adc128 {
	struct spi_device *spi;

	struct regulator *reg;
	struct mutex lock;

	u8 buffer[2] __aligned(IIO_DMA_MINALIGN);
};

static int adc128_adc_conversion(struct adc128 *adc, u8 channel)
{
	int ret;

	mutex_lock(&adc->lock);

	adc->buffer[0] = channel << 3;
	adc->buffer[1] = 0;

	ret = spi_write(adc->spi, &adc->buffer, 2);
	if (ret < 0) {
		mutex_unlock(&adc->lock);
		return ret;
	}

	ret = spi_read(adc->spi, &adc->buffer, 2);

	mutex_unlock(&adc->lock);

	if (ret < 0)
		return ret;

	return (adc->buffer[0] << 8 | adc->buffer[1]);
}

static int adc128_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *channel, int *val,
			   int *val2, long mask)
{
	struct adc128 *adc = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:

		ret = adc128_adc_conversion(adc, channel->channel);
		if (ret < 0)
			return ret;

		*val = (ret >> channel->scan_type.shift) &
			GENMASK(channel->scan_type.realbits - 1, 0);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:

		ret = regulator_get_voltage(adc->reg);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = channel->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}

}

#define _ADC128_VOLTAGE_CHANNEL(num, real_bits, store_bits)		\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = (num),					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.scan_index = (num),					\
		.scan_type = {						\
			.sign = 'u',					\
			.realbits = (real_bits),			\
			.storagebits = (store_bits),			\
			.shift = (12 - real_bits),			\
		},							\
	}

#define ADC082_VOLTAGE_CHANNEL(num) _ADC128_VOLTAGE_CHANNEL(num, 8, 16)
#define ADC102_VOLTAGE_CHANNEL(num) _ADC128_VOLTAGE_CHANNEL(num, 10, 16)
#define ADC128_VOLTAGE_CHANNEL(num) _ADC128_VOLTAGE_CHANNEL(num, 12, 16)

static const struct iio_chan_spec adc082s021_channels[] = {
	ADC082_VOLTAGE_CHANNEL(0),
	ADC082_VOLTAGE_CHANNEL(1),
};

static const struct iio_chan_spec adc102s021_channels[] = {
	ADC102_VOLTAGE_CHANNEL(0),
	ADC102_VOLTAGE_CHANNEL(1),
};

static const struct iio_chan_spec adc122s021_channels[] = {
	ADC128_VOLTAGE_CHANNEL(0),
	ADC128_VOLTAGE_CHANNEL(1),
};

static const struct iio_chan_spec adc124s021_channels[] = {
	ADC128_VOLTAGE_CHANNEL(0),
	ADC128_VOLTAGE_CHANNEL(1),
	ADC128_VOLTAGE_CHANNEL(2),
	ADC128_VOLTAGE_CHANNEL(3),
};

static const struct iio_chan_spec adc128s052_channels[] = {
	ADC128_VOLTAGE_CHANNEL(0),
	ADC128_VOLTAGE_CHANNEL(1),
	ADC128_VOLTAGE_CHANNEL(2),
	ADC128_VOLTAGE_CHANNEL(3),
	ADC128_VOLTAGE_CHANNEL(4),
	ADC128_VOLTAGE_CHANNEL(5),
	ADC128_VOLTAGE_CHANNEL(6),
	ADC128_VOLTAGE_CHANNEL(7),
};

static const struct adc128_configuration adc128_config[] = {
	{ adc082s021_channels, ARRAY_SIZE(adc082s021_channels) },
	{ adc102s021_channels, ARRAY_SIZE(adc102s021_channels) },
	{ adc122s021_channels, ARRAY_SIZE(adc122s021_channels) },
	{ adc124s021_channels, ARRAY_SIZE(adc124s021_channels) },
	{ adc128s052_channels, ARRAY_SIZE(adc128s052_channels) },
};

/* Ensure match with adc128_config indices */
enum adc128_configuration_index {
	ADC128_CONFIG_INDEX_082S,
	ADC128_CONFIG_INDEX_102S,
	ADC128_CONFIG_INDEX_122S,
	ADC128_CONFIG_INDEX_124S,
	ADC128_CONFIG_INDEX_128S,
};

static const struct iio_info adc128_info = {
	.read_raw = adc128_read_raw,
};

static void adc128_disable_regulator(void *reg)
{
	regulator_disable(reg);
}

static int adc128_probe(struct spi_device *spi)
{
	const struct adc128_configuration *config;
	struct iio_dev *indio_dev;
	struct adc128 *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adc128_info;

	config = spi_get_device_match_data(spi);

	indio_dev->channels = config->channels;
	indio_dev->num_channels = config->num_channels;

	adc->reg = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->reg))
		return PTR_ERR(adc->reg);

	ret = regulator_enable(adc->reg);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, adc128_disable_regulator,
				       adc->reg);
	if (ret)
		return ret;

	mutex_init(&adc->lock);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id adc128_of_match[] = {
	{ .compatible = "ti,adc082s021", .data = &adc128_config[ADC128_CONFIG_INDEX_082S] },
	{ .compatible = "ti,adc082s051", .data = &adc128_config[ADC128_CONFIG_INDEX_082S] },
	{ .compatible = "ti,adc082s101", .data = &adc128_config[ADC128_CONFIG_INDEX_082S] },
	{ .compatible = "ti,adc102s021", .data = &adc128_config[ADC128_CONFIG_INDEX_102S] },
	{ .compatible = "ti,adc102s051", .data = &adc128_config[ADC128_CONFIG_INDEX_102S] },
	{ .compatible = "ti,adc102s101", .data = &adc128_config[ADC128_CONFIG_INDEX_102S] },
	{ .compatible = "ti,adc122s021", .data = &adc128_config[ADC128_CONFIG_INDEX_122S] },
	{ .compatible = "ti,adc122s051", .data = &adc128_config[ADC128_CONFIG_INDEX_122S] },
	{ .compatible = "ti,adc122s101", .data = &adc128_config[ADC128_CONFIG_INDEX_122S] },
	{ .compatible = "ti,adc124s021", .data = &adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ .compatible = "ti,adc124s051", .data = &adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ .compatible = "ti,adc124s101", .data = &adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ .compatible = "ti,adc128s052", .data = &adc128_config[ADC128_CONFIG_INDEX_128S] },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, adc128_of_match);

static const struct spi_device_id adc128_id[] = {
	{ "adc082s021", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_082S] },
	{ "adc082s051", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_082S] },
	{ "adc082s101", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_082S] },
	{ "adc102s021", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_102S] },
	{ "adc102s051", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_102S] },
	{ "adc102s101", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_102S] },
	{ "adc122s021",	(kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_122S] },
	{ "adc122s051",	(kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_122S] },
	{ "adc122s101",	(kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_122S] },
	{ "adc124s021", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ "adc124s051", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ "adc124s101", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ "adc128s052", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_128S] },
	{ }
};
MODULE_DEVICE_TABLE(spi, adc128_id);

static const struct acpi_device_id adc128_acpi_match[] = {
	{ "AANT1280", (kernel_ulong_t)&adc128_config[ADC128_CONFIG_INDEX_124S] },
	{ }
};
MODULE_DEVICE_TABLE(acpi, adc128_acpi_match);

static struct spi_driver adc128_driver = {
	.driver = {
		.name = "adc128s052",
		.of_match_table = adc128_of_match,
		.acpi_match_table = adc128_acpi_match,
	},
	.probe = adc128_probe,
	.id_table = adc128_id,
};
module_spi_driver(adc128_driver);

MODULE_AUTHOR("Angelo Compagnucci <angelo.compagnucci@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments ADC128S052");
MODULE_LICENSE("GPL v2");
