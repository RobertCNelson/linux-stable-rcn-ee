// SPDX-License-Identifier: GPL-2.0+
/*
 * IIO driver for PAC1934 Multi-Channel DC Power/Energy Monitor
 *
 * Copyright (C) 2017-2023 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Bogdan Bolocan <bogdan.bolocan@microchip.com>
 * Author: Victor Tudose
 * Author: Marius Cristea <marius.cristea@microchip.com>
 *
 * Datasheet for PAC1931, PAC1932, PAC1933 and PAC1934 can be found here:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/PAC1931-Family-Data-Sheet-DS20005850E.pdf
 *
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <asm/unaligned.h>

/*
 * maximum acummulation time should be (17 * 60 * 1000) around 17 minutes@1024 sps
 * till PAC193X accumulation registers starts to saturate
 */
#define PAC193X_MAX_RFSH_LIMIT_MS		60000
/* 50msec is the timeout for validity of the cached registers */
#define PAC193X_MIN_POLLING_TIME_MS		50
/*
 * 1000usec is the minimum wait time for normal conversions when sample
 * rate doesn't change
 */
#define PAC193X_MIN_UPDATE_WAIT_TIME_US		1000

#define SHUNT_UOHMS_DEFAULT			100000
/* 32000mV */
#define PAC193X_VOLTAGE_MILLIVOLTS_MAX		32000
/* voltage bits resolution when set for unsigned values */
#define PAC193X_VOLTAGE_U_RES			16
/* voltage bits resolution when set for signed values */
#define PAC193X_VOLTAGE_S_RES			15
/* 100mV maximum for current shunts */
#define PAC193X_VSENSE_MILLIVOLTS_MAX		100
/* voltage bits resolution when set for unsigned values */
#define PAC193X_CURRENT_U_RES			16
/* voltage bits resolution when set for signed values */
#define PAC193X_CURRENT_S_RES			15

/* power resolution is 28 bits when unsigned */
#define PAC193X_POWER_U_RES			28
/* power resolution is 27 bits when signed */
#define PAC193X_POWER_S_RES			27

/* energy accumulation is 48 bits long */
#define PAC193X_ENERGY_U_RES			48

#define PAC193X_ENERGY_S_RES			47

#define BIT_INDEX_31				31

/*
 * max signed value that can be stored on 32 bits and 8 digits fractional value
 * (2^31 - 1) * 10^8 + 99999999
 */
#define PAC_193X_MAX_POWER_ACC			214748364799999999LL
/*
 * min signed value that can be stored on 32 bits and 8 digits fractional value
 * -(2^31) * 10^8 - 99999999
 */
#define PAC_193X_MIN_POWER_ACC			-214748364899999999LL

/* maximum value that device can measure - 32 V * 0.1 V */
#define PAC193X_PRODUCT_VOLTAGE_PV_FSR		3200000000000UL

#define PAC193X_MAX_NUM_CHANNELS		4
#define PAC1931_NUM_CHANNELS			1
#define PAC1932_NUM_CHANNELS			2
#define PAC1933_NUM_CHANNELS			3
#define PAC1934_NUM_CHANNELS			4
#define PAC193X_CH_1				0
#define PAC193X_CH_2				1
#define PAC193X_CH_3				2
#define PAC193X_CH_4				3
#define PAC193X_MEAS_REG_LEN			76
#define PAC193X_CTRL_REG_LEN			12

#define PAC193X_DEFAULT_CHIP_SAMP_SPEED		1024

/* I2C address map */
#define PAC193X_REFRESH_REG_ADDR		0x00
#define PAC193X_CTRL_REG_ADDR			0x01
#define PAC193X_ACC_COUNT_REG_ADDR		0x02
#define PAC193X_VPOWER_ACC_1_ADDR		0x03
#define PAC193X_VPOWER_ACC_2_ADDR		0x04
#define PAC193X_VPOWER_ACC_3_ADDR		0x05
#define PAC193X_VPOWER_ACC_4_ADDR		0x06
#define PAC193X_VBUS_1_ADDR			0x07
#define PAC193X_VBUS_2_ADDR			0x08
#define PAC193X_VBUS_3_ADDR			0x09
#define PAC193X_VBUS_4_ADDR			0x0A
#define PAC193X_VSENSE_1_ADDR			0x0B
#define PAC193X_VSENSE_2_ADDR			0x0C
#define PAC193X_VSENSE_3_ADDR			0x0D
#define PAC193X_VSENSE_4_ADDR			0x0E
#define PAC193X_VBUS_AVG_1_ADDR			0x0F
#define PAC193X_VBUS_AVG_2_ADDR			0x10
#define PAC193X_VBUS_AVG_3_ADDR			0x11
#define PAC193X_VBUS_AVG_4_ADDR			0x12
#define PAC193X_VSENSE_AVG_1_ADDR		0x13
#define PAC193X_VSENSE_AVG_2_ADDR		0x14
#define PAC193X_VSENSE_AVG_3_ADDR		0x15
#define PAC193X_VSENSE_AVG_4_ADDR		0x16
#define PAC193X_VPOWER_1_ADDR			0x17
#define PAC193X_VPOWER_2_ADDR			0x18
#define PAC193X_VPOWER_3_ADDR			0x19
#define PAC193X_VPOWER_4_ADDR			0x1A
#define PAC193X_REFRESH_V_REG_ADDR		0x1F
#define PAC193X_CTRL_STAT_REGS_ADDR		0x1C
#define PAC193X_PID_REG_ADDR			0xFD
#define PAC193X_MID_REG_ADDR			0xFE
#define PAC193X_RID_REG_ADDR			0xFF

/* PRODUCT ID REGISTER + MANUFACTURER ID REGISTER + REVISION ID REGISTER */
#define PAC193X_ID_REG_LEN			0x03
#define PAC193X_PID_IDX				0
#define PAC193X_MID_IDX				1
#define PAC193X_RID_IDX				2

#define PAC193X_ACPI_ARG_COUNT			4
#define PAC193X_ACPI_GET_NAMES_AND_MOHMS_VALS	1
#define PAC193X_ACPI_GET_UOHMS_VALS		2
#define PAC193X_ACPI_GET_BIPOLAR_SETTINGS	4
#define PAC193X_ACPI_GET_SAMP			5

#define PAC193X_SAMPLE_RATE_SHIFT		6

/*
 * these indexes are exactly describing the element order within a single
 * PAC193X phys channel IIO channel descriptor; see the static const struct
 * iio_chan_spec pac193x_single_channel[] declaration
 */
#define IIO_EN					0
#define IIO_POW					1
#define IIO_VOLT				2
#define IIO_CRT					3
#define IIO_VOLTAVG				4
#define IIO_CRTAVG				5

#define PAC193X_VBUS_SENSE_REG_LEN		2
#define PAC193X_ACC_REG_LEN			3
#define PAC193X_VPOWER_REG_LEN			4
#define PAC193X_VPOWER_ACC_REG_LEN		6
#define PAC193X_MAX_REGISTER_LENGTH		6

#define PAC193X_CUSTOM_ATTR_FOR_CHANNEL		2
#define PAC193X_SHARED_DEVATTRS_COUNT		1

/*
 * relative offsets when using multi-byte reads/writes even though these
 * bytes are read one after the other, they are not at adjacent memory
 * locations within the I2C memory map. The chip can skip some addresses
 */
#define PAC193X_CHANNEL_DIS_REG_OFF		0
#define PAC193X_NEG_PWR_REG_OFF			1

/*
 * when reading/writing multiple bytes from offset PAC193X_CHANNEL_DIS_REG_OFF,
 * the chip jumps over the 0x1E (REFRESH_G) and 0x1F (REFRESH_V) offsets
 */
#define PAC193X_SLOW_REG_OFF			2
#define PAC193X_CTRL_ACT_REG_OFF		3
#define PAC193X_CHANNEL_DIS_ACT_REG_OFF		4
#define PAC193X_NEG_PWR_ACT_REG_OFF		5
#define PAC193X_CTRL_LAT_REG_OFF		6
#define PAC193X_CHANNEL_DIS_LAT_REG_OFF		7
#define PAC193X_NEG_PWR_LAT_REG_OFF		8
#define PAC193X_PID_REG_OFF			9
#define PAC193X_MID_REG_OFF			10
#define PAC193X_REV_REG_OFF			11
#define PAC193X_CTRL_STATUS_INFO_LEN		12

#define PAC193X_MID				0x5D
#define PAC1934_PID				0x5B
#define PAC1933_PID				0x5A
#define PAC1932_PID				0x59
#define PAC1931_PID				0x58

/* Scale constant = (10^3 * 3.2 * 10^9 / 2^28) for mili Watt-second */
#define PAC193X_SCALE_CONSTANT			11921

#define PAC193X_MAX_VPOWER_RSHIFTED_BY_28B	11921
#define PAC193X_MAX_VSENSE_RSHIFTED_BY_16B	1525

#define PAC193X_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)

#define PAC193X_CRTL_SAMPLE_RATE_MASK	GENMASK(7, 6)
#define PAC193X_CRTL_SAMPLE_RATE_SET(x)	((u8)FIELD_PREP(PAC193X_CRTL_SAMPLE_RATE_MASK, (x)))
#define PAC193X_CHAN_SLEEP_MASK		BIT(5)
#define PAC193X_CHAN_SLEEP_SET		BIT(5)
#define PAC193X_CHAN_SINLE_MASK		BIT(4)
#define PAC193X_CHAN_SINLE_SHOT_SET	BIT(4)
#define PAC193X_CHAN_ALERT_MASK		BIT(3)
#define PAC193X_CHAN_ALERT_EN		BIT(3)
#define PAC193X_CHAN_ALERT_CC_MASK	BIT(2)
#define PAC193X_CHAN_ALERT_CC_EN	BIT(2)
#define PAC193X_CHAN_OVF_ALERT_MASK	BIT(1)
#define PAC193X_CHAN_OVF_ALERT_EN	BIT(1)
#define PAC193X_CHAN_OVF_MASK		BIT(0)

#define PAC193X_CHAN_DIS_CH1_OFF_MASK	BIT(7)
#define PAC193X_CHAN_DIS_CH1_OFF(x)	((u8)FIELD_PREP(PAC193X_CHAN_DIS_CH1_OFF_MASK, !(x)))
#define PAC193X_CHAN_DIS_CH2_OFF_MASK	BIT(6)
#define PAC193X_CHAN_DIS_CH2_OFF(x)	((u8)FIELD_PREP(PAC193X_CHAN_DIS_CH2_OFF_MASK, !(x)))
#define PAC193X_CHAN_DIS_CH3_OFF_MASK	BIT(5)
#define PAC193X_CHAN_DIS_CH3_OFF(x)	((u8)FIELD_PREP(PAC193X_CHAN_DIS_CH3_OFF_MASK, !(x)))
#define PAC193X_CHAN_DIS_CH4_OFF_MASK	BIT(4)
#define PAC193X_CHAN_DIS_CH4_OFF(x)	((u8)FIELD_PREP(PAC193X_CHAN_DIS_CH4_OFF_MASK, !(x)))
#define PAC193X_SMBUS_TIMEOUT_MASK	BIT(3)
#define PAC193X_SMBUS_TIMEOUT_EN(x)	FIELD_PREP(PAC193X_SMBUS_TIMEOUT_MASK, x)
#define PAC193X_SMBUS_BYTECOUNT_MASK	BIT(2)
#define PAC193X_SMBUS_BYTECOUNT_EN(x)	FIELD_PREP(PAC193X_SMBUS_BYTECOUNT_MASK, x)
#define PAC193X_SMBUS_NO_SKIP_MASK	BIT(1)
#define PAC193X_SMBUS_NO_SKIP_EN(x)	FIELD_PREP(PAC193X_SMBUS_NO_SKIP_MASK, x)

#define PAC193X_NEG_PWR_CH1_BIDI_MASK	BIT(7)
#define PAC193X_NEG_PWR_CH1_BIDI(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH1_BIDI_MASK, (x)))
#define PAC193X_NEG_PWR_CH2_BIDI_MASK	BIT(6)
#define PAC193X_NEG_PWR_CH2_BIDI(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH2_BIDI_MASK, (x)))
#define PAC193X_NEG_PWR_CH3_BIDI_MASK	BIT(5)
#define PAC193X_NEG_PWR_CH3_BIDI(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH3_BIDI_MASK, (x)))
#define PAC193X_NEG_PWR_CH4_BIDI_MASK	BIT(4)
#define PAC193X_NEG_PWR_CH4_BIDI(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH4_BIDI_MASK, (x)))
#define PAC193X_NEG_PWR_CH1_BIDV_MASK	BIT(3)
#define PAC193X_NEG_PWR_CH1_BIDV(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH1_BIDV_MASK, (x)))
#define PAC193X_NEG_PWR_CH2_BIDV_MASK	BIT(2)
#define PAC193X_NEG_PWR_CH2_BIDV(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH2_BIDV_MASK, (x)))
#define PAC193X_NEG_PWR_CH3_BIDV_MASK	BIT(1)
#define PAC193X_NEG_PWR_CH3_BIDV(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH3_BIDV_MASK, (x)))
#define PAC193X_NEG_PWR_CH4_BIDV_MASK	BIT(0)
#define PAC193X_NEG_PWR_CH4_BIDV(x)	((u8)FIELD_PREP(PAC193X_NEG_PWR_CH4_BIDV_MASK, (x)))

/*
 * Universal Unique Identifier (UUID),
 * 033771E0-1705-47B4-9535-D1BBE14D9A09, is
 * reserved to Microchip for the PAC193X and must not be changed
 */
#define PAC193X_DSM_UUID		"033771E0-1705-47B4-9535-D1BBE14D9A09"

enum pac193x_ids {
	PAC1931,
	PAC1932,
	PAC1933,
	PAC1934
};

enum pac193x_samps {
	PAC193X_SAMP_1024SPS,
	PAC193X_SAMP_256SPS,
	PAC193X_SAMP_64SPS,
	PAC193X_SAMP_8SPS
};

/**
 * struct pac193x_features - features of a pac193x instance
 * @phys_channels: number of physical channels supported by the chip
 * @prod_id: product ID
 */
struct pac193x_features {
	u8 phys_channels;
	u8 prod_id;
};

struct samp_rate_mapping {
	u16 samp_rate;
	u8 shift2value;
};

static const unsigned int samp_rate_map_tbl[] = {
	[PAC193X_SAMP_1024SPS] = 1024,
	[PAC193X_SAMP_256SPS] = 256,
	[PAC193X_SAMP_64SPS] = 64,
	[PAC193X_SAMP_8SPS] = 8,
};

static const struct pac193x_features pac193x_chip_config[] = {
	[PAC1931] = {
	    .phys_channels = 1,
	    .prod_id = PAC1931_PID,
	},
	[PAC1932] = {
	    .phys_channels = 2,
	    .prod_id = PAC1932_PID,
	},
	[PAC1933] = {
	    .phys_channels = 3,
	    .prod_id = PAC1933_PID,
	},
	[PAC1934] = {
	    .phys_channels = 4,
	    .prod_id = PAC1934_PID,
	},
};

/**
 * struct reg_data - data from the registers
 * @meas_regs: snapshot of raw measurements registers
 * @ctrl_regs: snapshot of control registers
 * @energy_sec_acc: snapshot of energy values
 * @vpower_acc: accumulated vpower values
 * @vpower: snapshot of vpower registers
 * @vbus: snapshot of vbus registers
 * @vbus_avg: averages of vbus registers
 * @vsense: snapshot of vsense registers
 * @vsense_avg: averages of vsense registers
 * @num_enabled_channels: count of how many chip channels are currently enabled
 */
struct reg_data {
	u8	meas_regs[PAC193X_MEAS_REG_LEN];
	u8	ctrl_regs[PAC193X_CTRL_REG_LEN];
	s64	energy_sec_acc[PAC193X_MAX_NUM_CHANNELS];
	s64	vpower_acc[PAC193X_MAX_NUM_CHANNELS];
	s32	vpower[PAC193X_MAX_NUM_CHANNELS];
	s32	vbus[PAC193X_MAX_NUM_CHANNELS];
	s32	vbus_avg[PAC193X_MAX_NUM_CHANNELS];
	s32	vsense[PAC193X_MAX_NUM_CHANNELS];
	s32	vsense_avg[PAC193X_MAX_NUM_CHANNELS];
	u8	num_enabled_channels;
};

/**
 * struct pac193x_chip_info - information about the chip
 * @client: the i2c-client attached to the device
 * @lock: lock used by the chip's mutex
 * @work_chip_rfsh: work queue used for refresh commands
 * @phys_channels: phys channels count
 * @active_channels: array of values, true means that channel is active
 * @bi_dir: array of bools, true means that channel is bidirectional
 * @chip_variant: chip variant
 * @chip_revision: chip revision
 * @shunts: shunts
 * @chip_reg_data: chip reg data
 * @sample_rate_value: sampling frequency
 * @channel_names: channel names
 * @pac193x_info: pac193x_info
 * @crt_samp_spd_bitfield:the current sampling speed
 * @jiffies_tstamp: chip's uptime
 */
struct pac193x_chip_info {
	struct i2c_client	*client;
	struct mutex		lock; /* lock to prevent concurent reads/writes */
	struct delayed_work	work_chip_rfsh;
	u8			phys_channels;
	bool			active_channels[PAC193X_MAX_NUM_CHANNELS];
	bool			bi_dir[PAC193X_MAX_NUM_CHANNELS];
	u8			chip_variant;
	u8			chip_revision;
	u32			shunts[PAC193X_MAX_NUM_CHANNELS];
	struct reg_data		chip_reg_data;
	s32			sample_rate_value;
	char			*channel_names[PAC193X_MAX_NUM_CHANNELS];
	struct iio_info		pac193x_info;
	u8			crt_samp_spd_bitfield;
	unsigned long		jiffies_tstamp;
};

#define TO_PAC193X_CHIP_INFO(d) container_of(d, struct pac193x_chip_info, work_chip_rfsh)

/* macros to extract the parameters */
#define PAC193X_MVBUS_SENSES(x)		sign_extend32((u32)(x), 15)

#define PAC193X_VPOWER_ACC_CHANNEL(_index, _si, _address) {			\
	.type = IIO_ENERGY,							\
	.address = (_address),							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)	|			\
				BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_si),							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = PAC193X_ENERGY_U_RES,				\
		.storagebits = PAC193X_ENERGY_U_RES,				\
		.endianness = IIO_CPU,						\
	}									\
}

#define PAC193X_VBUS_CHANNEL(_index, _si, _address) {				\
	.type = IIO_VOLTAGE,							\
	.address = (_address),							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_si),							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = PAC193X_VOLTAGE_U_RES,				\
		.storagebits = PAC193X_VOLTAGE_U_RES,				\
		.endianness = IIO_CPU,						\
	}									\
}

#define PAC193X_VBUS_AVG_CHANNEL(_index, _si, _address) {			\
	.type = IIO_VOLTAGE,							\
	.address = (_address),							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW)			\
				| BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_si),							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = PAC193X_VOLTAGE_U_RES,				\
		.storagebits = PAC193X_VOLTAGE_U_RES,				\
		.endianness = IIO_CPU,						\
	}									\
}

#define PAC193X_VSENSE_CHANNEL(_index, _si, _address) {				\
	.type = IIO_CURRENT,							\
	.address = (_address),							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_si),							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = PAC193X_CURRENT_U_RES,				\
		.storagebits = PAC193X_CURRENT_U_RES,				\
		.endianness = IIO_CPU,						\
	}									\
}

#define PAC193X_VSENSE_AVG_CHANNEL(_index, _si, _address) {			\
	.type = IIO_CURRENT,							\
	.address = (_address),							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW) |			\
			BIT(IIO_CHAN_INFO_SCALE),				\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_si),							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = PAC193X_CURRENT_U_RES,				\
		.storagebits = PAC193X_CURRENT_U_RES,				\
		.endianness = IIO_CPU,						\
	}									\
}

#define PAC193X_VPOWER_CHANNEL(_index, _si, _address) {				\
	.type = IIO_POWER,							\
	.address = (_address),							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |				\
			BIT(IIO_CHAN_INFO_SCALE),				\
	.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_si),							\
	.scan_type = {								\
		.sign = 'u',							\
		.realbits = PAC193X_POWER_U_RES,				\
		.storagebits = 32,						\
		.shift = 4,							\
		.endianness = IIO_CPU,						\
	}									\
}

static const struct iio_chan_spec pac193x_single_channel[] = {
	PAC193X_VPOWER_ACC_CHANNEL(0, 0, PAC193X_VPOWER_ACC_1_ADDR),
	PAC193X_VPOWER_CHANNEL(0, 0, PAC193X_VPOWER_1_ADDR),
	PAC193X_VBUS_CHANNEL(0, 0, PAC193X_VBUS_1_ADDR),
	PAC193X_VSENSE_CHANNEL(0, 0, PAC193X_VSENSE_1_ADDR),
	PAC193X_VBUS_AVG_CHANNEL(0, 0, PAC193X_VBUS_AVG_1_ADDR),
	PAC193X_VSENSE_AVG_CHANNEL(0, 0, PAC193X_VSENSE_AVG_1_ADDR),
};

/* Low-level I2c functions */
static int pac193x_i2c_read(struct i2c_client *client, u8 reg_addr, void *databuf, u8 len)
{
	int ret;
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr,
		 .len = 1,
		 .buf = (u8 *)&reg_addr,
		 .flags = 0
		},
		{ .addr = client->addr,
		 .len = len,
		 .buf = databuf,
		 .flags = I2C_M_RD
		}
	};

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return 0;
}

static int pac193x_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data, int len)
{
	int ret;
	u8 send[PAC193X_MAX_REGISTER_LENGTH + 1];

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));

	ret = i2c_master_send(client, send, len + 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed writing data from register 0x%02X\n", reg_addr);
		return ret;
	}

	return 0;
}

static int pac193x_match_samp_rate(struct pac193x_chip_info *chip_info, u32 new_samp_rate)
{
	int cnt;

	for (cnt = 0; cnt < ARRAY_SIZE(samp_rate_map_tbl); cnt++) {
		if (new_samp_rate == samp_rate_map_tbl[cnt]) {
			chip_info->crt_samp_spd_bitfield = cnt;
			return 0;
		}
	}

	/* not a valid sample rate value */
	return -EINVAL;
}

static ssize_t shunt_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int len = 0;
	int target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0') - 1;

	len += sprintf(buf, "%u\n", chip_info->shunts[target]);

	return len;
}

static ssize_t channel_name_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int len = 0;
	int target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0') - 1;

	len += sprintf(buf, "%s\n", chip_info->channel_names[target]);

	return len;
}

static ssize_t shunt_value_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int sh_val, target;

	target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0') - 1;
	if (kstrtouint(buf, 10, &sh_val)) {
		dev_err(dev, "Shunt value is not valid\n");
		return -EINVAL;
	}

	mutex_lock(&chip_info->lock);
	chip_info->shunts[target] = sh_val;
	mutex_unlock(&chip_info->lock);

	return count;
}

static int pac193x_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel,
			      const int **vals, int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT;
		*vals = samp_rate_map_tbl;
		*length = ARRAY_SIZE(samp_rate_map_tbl);
		return IIO_AVAIL_LIST;
	}

	return -EINVAL;
}

static int pac193x_send_refresh(struct pac193x_chip_info *chip_info,
				u8 refresh_cmd, u32 wait_time)
{
	/* this function only sends REFRESH or REFRESH_V */
	struct i2c_client *client = chip_info->client;
	int ret;
	u8 bidir_reg;
	bool revision_bug = false;

	if (chip_info->chip_revision == 2 || chip_info->chip_revision == 3) {
		/*
		 * chip rev 2 and 3 bug workaround
		 * see: PAC193X Family Data Sheet Errata DS80000836A.pdf
		 */
		revision_bug = true;

		bidir_reg = PAC193X_NEG_PWR_CH1_BIDI(chip_info->bi_dir[PAC193X_CH_1]) |
			PAC193X_NEG_PWR_CH2_BIDI(chip_info->bi_dir[PAC193X_CH_2]) |
			PAC193X_NEG_PWR_CH3_BIDI(chip_info->bi_dir[PAC193X_CH_3]) |
			PAC193X_NEG_PWR_CH4_BIDI(chip_info->bi_dir[PAC193X_CH_4]) |
			PAC193X_NEG_PWR_CH1_BIDV(chip_info->bi_dir[PAC193X_CH_1]) |
			PAC193X_NEG_PWR_CH2_BIDV(chip_info->bi_dir[PAC193X_CH_2]) |
			PAC193X_NEG_PWR_CH3_BIDV(chip_info->bi_dir[PAC193X_CH_3]) |
			PAC193X_NEG_PWR_CH4_BIDV(chip_info->bi_dir[PAC193X_CH_4]);

		ret = i2c_smbus_write_byte_data(client,
						PAC193X_CTRL_STAT_REGS_ADDR +
						PAC193X_NEG_PWR_REG_OFF,
						bidir_reg);
		if (ret)
			return ret;
	}

	ret = i2c_smbus_write_byte(client, refresh_cmd);
	if (ret) {
		dev_err(&client->dev, "%s - cannot send 0x%02X\n",
			__func__, refresh_cmd);
		return ret;
	}

	if (revision_bug) {
		/*
		 * chip rev 2 and 3 bug workaround - write again the same register
		 * write the updated registers back
		 */
		ret = i2c_smbus_write_byte_data(client,
						PAC193X_CTRL_STAT_REGS_ADDR +
						PAC193X_NEG_PWR_REG_OFF, bidir_reg);
		if (ret)
			return ret;
	}

	/* register data retrieval timestamp */
	chip_info->jiffies_tstamp = jiffies;

	/* wait till the data is available */
	usleep_range(wait_time, wait_time + 100);

	return ret;
}

static int pac193x_reg_snapshot(struct pac193x_chip_info *chip_info,
				bool do_refresh, u8 refresh_cmd, u32 wait_time)
{
	int ret;
	struct i2c_client *client = chip_info->client;
	u8 samp_shift, ctrl_regs_tmp;
	u8 *offset_reg_data_p;
	u16 tmp_value;
	u32 samp_rate, cnt, tmp;
	s64 curr_energy, inc;
	u64 tmp_energy;
	struct reg_data *reg_data;

	mutex_lock(&chip_info->lock);

	if (do_refresh) {
		ret = pac193x_send_refresh(chip_info, refresh_cmd, wait_time);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s - cannot send refresh\n",
				__func__);
			goto reg_snapshot_err;
		}
	}

	/* read the ctrl/status registers for this snapshot */
	ret = pac193x_i2c_read(client, PAC193X_CTRL_STAT_REGS_ADDR,
			       (u8 *)chip_info->chip_reg_data.ctrl_regs,
			       PAC193X_CTRL_REG_LEN);
	if (ret) {
		dev_err(&client->dev,
			"%s - cannot read ctrl/status registers\n",
			__func__);
		goto reg_snapshot_err;
	}

	reg_data = &chip_info->chip_reg_data;

	/* read the data registers */
	ret = pac193x_i2c_read(client, PAC193X_ACC_COUNT_REG_ADDR,
			       (u8 *)reg_data->meas_regs, PAC193X_MEAS_REG_LEN);
	if (ret) {
		dev_err(&client->dev,
			"%s - cannot read ACC_COUNT register: %d:%d\n",
			__func__, ret, PAC193X_MEAS_REG_LEN);
		goto reg_snapshot_err;
	}

	/* see how much shift is required by the sample rate */
	samp_rate = samp_rate_map_tbl[((reg_data->ctrl_regs[PAC193X_CTRL_LAT_REG_OFF]) >> 6)];
	samp_shift = get_count_order(samp_rate);

	ctrl_regs_tmp = reg_data->ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF];
	offset_reg_data_p = &reg_data->meas_regs[PAC193X_ACC_REG_LEN];

	/* start with VPOWER_ACC */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active, skip all fields if disabled */
		if (((ctrl_regs_tmp << cnt) & 0x80) == 0) {
			curr_energy = chip_info->chip_reg_data.energy_sec_acc[cnt];

			tmp_energy = get_unaligned_be48(offset_reg_data_p);

			if (chip_info->bi_dir[cnt])
				reg_data->vpower_acc[cnt] = sign_extend64(tmp_energy, 47);
			else
				reg_data->vpower_acc[cnt] = tmp_energy;

			/*
			 * compute the scaled to 1 second accumulated energy value;
			 * energy accumulator scaled to 1sec = VPOWER_ACC/2^samp_shift
			 * the chip's sampling rate is 2^samp_shift samples/sec
			 */
			inc = (reg_data->vpower_acc[cnt] >> samp_shift);

			/* add the power_acc field */
			curr_energy += inc;

			/* check if we have reached the upper/lower limit */
			if (curr_energy > (s64)PAC_193X_MAX_POWER_ACC)
				curr_energy = PAC_193X_MAX_POWER_ACC;
			else if (curr_energy < ((s64)PAC_193X_MIN_POWER_ACC))
				curr_energy = PAC_193X_MIN_POWER_ACC;

			reg_data->energy_sec_acc[cnt] = curr_energy;

			offset_reg_data_p += PAC193X_VPOWER_ACC_REG_LEN;
		}
	}

	/* continue with VBUS */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((ctrl_regs_tmp << cnt) & 0x80) == 0) {
			tmp_value = get_unaligned_be16(offset_reg_data_p);

			if (chip_info->bi_dir[cnt])
				reg_data->vbus[cnt] = PAC193X_MVBUS_SENSES(tmp_value);
			else
				reg_data->vbus[cnt] = tmp_value;

			offset_reg_data_p += PAC193X_VBUS_SENSE_REG_LEN;
		}
	}

	/* VSENSE */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((ctrl_regs_tmp << cnt) & 0x80) == 0) {
			tmp_value = get_unaligned_be16(offset_reg_data_p);

			if (chip_info->bi_dir[cnt])
				reg_data->vsense[cnt] = PAC193X_MVBUS_SENSES(tmp_value);
			else
				reg_data->vsense[cnt] = tmp_value;

			offset_reg_data_p += PAC193X_VBUS_SENSE_REG_LEN;
		}
	}

	/* VBUS_AVG */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((ctrl_regs_tmp << cnt) & 0x80) == 0) {
			tmp_value = get_unaligned_be16(offset_reg_data_p);

			if (chip_info->bi_dir[cnt])
				reg_data->vbus_avg[cnt] = PAC193X_MVBUS_SENSES(tmp_value);
			else
				reg_data->vbus_avg[cnt] = tmp_value;

			offset_reg_data_p += PAC193X_VBUS_SENSE_REG_LEN;
		}
	}

	/* VSENSE_AVG */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((ctrl_regs_tmp << cnt) & 0x80) == 0) {
			tmp_value = get_unaligned_be16(offset_reg_data_p);

			if (chip_info->bi_dir[cnt])
				reg_data->vsense_avg[cnt] = PAC193X_MVBUS_SENSES(tmp_value);
			else
				reg_data->vsense_avg[cnt] = tmp_value;

			offset_reg_data_p += PAC193X_VBUS_SENSE_REG_LEN;
		}
	}

	/* VPOWER */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (((ctrl_regs_tmp << cnt) & 0x80) == 0) {
			tmp = get_unaligned_be32(offset_reg_data_p) >> 4;

			if (chip_info->bi_dir[cnt])
				reg_data->vpower[cnt] = sign_extend32(tmp, 27);
			else
				reg_data->vpower[cnt] = tmp;

			offset_reg_data_p += PAC193X_VPOWER_REG_LEN;
		}
	}

reg_snapshot_err:
	mutex_unlock(&chip_info->lock);

	return ret;
}

static int pac193x_retrieve_data(struct pac193x_chip_info *chip_info,
				 u32 wait_time)
{
	int ret = 0;

	/*
	 * check if the minimal elapsed time has passed and if so,
	 * re-read the chip, otherwise the cached info is just fine
	 */
	if (time_after(jiffies, chip_info->jiffies_tstamp +
					 msecs_to_jiffies(PAC193X_MIN_POLLING_TIME_MS))) {
		ret = pac193x_reg_snapshot(chip_info, true, PAC193X_REFRESH_REG_ADDR,
					   wait_time);

		/*
		 * Re-schedule the work for the read registers on timeout
		 * (to prevent chip regs saturation)
		 */
		cancel_delayed_work_sync(&chip_info->work_chip_rfsh);
		schedule_delayed_work(&chip_info->work_chip_rfsh,
				      msecs_to_jiffies(PAC193X_MAX_RFSH_LIMIT_MS));
	}

	return ret;
}

static int pac193x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	s64 curr_energy;
	int ret, channel = chan->channel - 1;

	ret = pac193x_retrieve_data(chip_info, PAC193X_MIN_UPDATE_WAIT_TIME_US);
	if (ret < 0)
		return ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			switch (chan->address) {
			case PAC193X_VBUS_1_ADDR:
			case PAC193X_VBUS_2_ADDR:
			case PAC193X_VBUS_3_ADDR:
			case PAC193X_VBUS_4_ADDR:
				*val = chip_info->chip_reg_data.vbus[channel];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_CURRENT:
			switch (chan->address) {
			case PAC193X_VSENSE_1_ADDR:
			case PAC193X_VSENSE_2_ADDR:
			case PAC193X_VSENSE_3_ADDR:
			case PAC193X_VSENSE_4_ADDR:
				*val = chip_info->chip_reg_data.vsense[channel];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_POWER:
			switch (chan->address) {
			case PAC193X_VPOWER_1_ADDR:
			case PAC193X_VPOWER_2_ADDR:
			case PAC193X_VPOWER_3_ADDR:
			case PAC193X_VPOWER_4_ADDR:
				*val = chip_info->chip_reg_data.vpower[channel];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_ENERGY:
			switch (chan->address) {
			case PAC193X_VPOWER_ACC_1_ADDR:
			case PAC193X_VPOWER_ACC_2_ADDR:
			case PAC193X_VPOWER_ACC_3_ADDR:
			case PAC193X_VPOWER_ACC_4_ADDR:

				curr_energy = chip_info->chip_reg_data.energy_sec_acc[channel];
				*val = (u32)curr_energy;
				*val2 = (u32)(curr_energy >> 32);
				return IIO_VAL_INT_64;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			switch (chan->address) {
			case PAC193X_VBUS_AVG_1_ADDR:
			case PAC193X_VBUS_AVG_2_ADDR:
			case PAC193X_VBUS_AVG_3_ADDR:
			case PAC193X_VBUS_AVG_4_ADDR:
				*val = chip_info->chip_reg_data.vbus_avg[channel];
				return IIO_VAL_INT;

			default:
				return -EINVAL;
			}
			break;
		case IIO_CURRENT:
			switch (chan->address) {
			case PAC193X_VSENSE_AVG_1_ADDR:
			case PAC193X_VSENSE_AVG_2_ADDR:
			case PAC193X_VSENSE_AVG_3_ADDR:
			case PAC193X_VSENSE_AVG_4_ADDR:
				*val = chip_info->chip_reg_data.vsense_avg[channel];
				return IIO_VAL_INT;

			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->address) {
		/* Voltages - scale for millivolts */
		case PAC193X_VBUS_1_ADDR:
		case PAC193X_VBUS_2_ADDR:
		case PAC193X_VBUS_3_ADDR:
		case PAC193X_VBUS_4_ADDR:
		case PAC193X_VBUS_AVG_1_ADDR:
		case PAC193X_VBUS_AVG_2_ADDR:
		case PAC193X_VBUS_AVG_3_ADDR:
		case PAC193X_VBUS_AVG_4_ADDR:
			*val = PAC193X_VOLTAGE_MILLIVOLTS_MAX;
			if (chan->scan_type.sign == 'u')
				*val2 = PAC193X_VOLTAGE_U_RES;
			else
				*val2 = PAC193X_VOLTAGE_S_RES;
			return IIO_VAL_FRACTIONAL_LOG2;

		/*
		 * Currents - scale for mA - depends on the
		 * channel's shunt value
		 * (100mV * 1000000) / (2^16 * shunt(uohm))
		 */
		case PAC193X_VSENSE_1_ADDR:
		case PAC193X_VSENSE_2_ADDR:
		case PAC193X_VSENSE_3_ADDR:
		case PAC193X_VSENSE_4_ADDR:
		case PAC193X_VSENSE_AVG_1_ADDR:
		case PAC193X_VSENSE_AVG_2_ADDR:
		case PAC193X_VSENSE_AVG_3_ADDR:
		case PAC193X_VSENSE_AVG_4_ADDR:
			*val = PAC193X_MAX_VSENSE_RSHIFTED_BY_16B;
			if (chan->scan_type.sign == 'u')
				*val2 = chip_info->shunts[channel];
			else
				*val2 = chip_info->shunts[channel] >> 1;
			return IIO_VAL_FRACTIONAL;

		/*
		 * Power - uW - it will use the combined scale
		 * for current and voltage
		 * current(mA) * voltage(mV) = power (uW)
		 */
		case PAC193X_VPOWER_1_ADDR:
		case PAC193X_VPOWER_2_ADDR:
		case PAC193X_VPOWER_3_ADDR:
		case PAC193X_VPOWER_4_ADDR:
			*val = PAC193X_MAX_VPOWER_RSHIFTED_BY_28B;
			if (chan->scan_type.sign == 'u')
				*val2 = chip_info->shunts[channel];
			else
				*val2 = chip_info->shunts[channel] >> 1;
			return IIO_VAL_FRACTIONAL;
		case PAC193X_VPOWER_ACC_1_ADDR:
		case PAC193X_VPOWER_ACC_2_ADDR:
		case PAC193X_VPOWER_ACC_3_ADDR:
		case PAC193X_VPOWER_ACC_4_ADDR:

			/*
			 * expresses the 32 bit scale value
			 * here compute the scale for energy (mili Watt-second or miliJoule)
			 */
			*val = PAC193X_SCALE_CONSTANT;

			if (chan->scan_type.sign == 'u')
				*val2 = chip_info->shunts[channel];
			else
				*val2 = chip_info->shunts[channel] >> 1;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = chip_info->sample_rate_value;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int pac193x_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	struct i2c_client *client = chip_info->client;
	int ret = -EINVAL;
	s32 old_samp_rate;
	u8 ctrl_reg;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = pac193x_match_samp_rate(chip_info, (u16)val);
		if (ret)
			return ret;

		old_samp_rate = chip_info->sample_rate_value;
		chip_info->sample_rate_value = val;

		/*
		 * now lock the access to the chip, write the new
		 * sampling value and trigger a snapshot(incl refresh)
		 */
		mutex_lock(&chip_info->lock);

		ctrl_reg = PAC193X_CRTL_SAMPLE_RATE_SET(chip_info->crt_samp_spd_bitfield);
		ret = i2c_smbus_write_byte_data(client, PAC193X_CTRL_REG_ADDR, ctrl_reg);
		if (ret) {
			dev_err(&client->dev, "%s - can't update sample rate\n", __func__);
			chip_info->sample_rate_value = old_samp_rate;
			mutex_unlock(&chip_info->lock);
			return ret;
		}

		/*
		 * unlock the access towards the chip - register
		 * snapshot includes its own access lock
		 */
		mutex_unlock(&chip_info->lock);

		/*
		 * now, force a snapshot with refresh - call retrieve
		 * data in order to update the refresh timer
		 * alter the timestamp in order to force trigger a
		 * register snapshot and a timestamp update
		 */
		chip_info->jiffies_tstamp -=
			msecs_to_jiffies(PAC193X_MIN_POLLING_TIME_MS);
		ret = pac193x_retrieve_data(chip_info, (1024 / old_samp_rate) * 1000);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s - cannot snapshot ctrl and measurement regs\n",
				__func__);
			return ret;
		}

		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static void pac193x_work_periodic_rfsh(struct work_struct *work)
{
	struct pac193x_chip_info *chip_info = TO_PAC193X_CHIP_INFO((struct delayed_work *)work);
	struct i2c_client *client = chip_info->client;

	dev_dbg(&client->dev, "%s - Periodic refresh\n", __func__);

	/* do a REFRESH, then read */
	pac193x_reg_snapshot(chip_info, true, PAC193X_REFRESH_REG_ADDR,
			     PAC193X_MIN_UPDATE_WAIT_TIME_US);

	schedule_delayed_work(&chip_info->work_chip_rfsh,
			      msecs_to_jiffies(PAC193X_MAX_RFSH_LIMIT_MS));
}

static int pac193x_read_revision(struct pac193x_chip_info *chip_info, u8 *buf)
{
	int ret;
	struct i2c_client *client = chip_info->client;

	ret = pac193x_i2c_read(client, PAC193X_PID_REG_ADDR, buf, PAC193X_ID_REG_LEN);
	if (ret) {
		dev_err(&client->dev, "cannot read revision\n");
		return ret;
	}

	return 0;
}

static int pac193x_chip_identify(struct pac193x_chip_info *chip_info)
{
	int ret;
	struct i2c_client *client = chip_info->client;
	u8 rev_info[PAC193X_ID_REG_LEN];

	ret = pac193x_read_revision(chip_info, (u8 *)rev_info);
	if (ret) {
		dev_err_probe(&client->dev, ret, "cannot read revision\n");
		return ret;
	}

	if (rev_info[PAC193X_PID_IDX] != pac193x_chip_config[chip_info->chip_variant].prod_id) {
		dev_err_probe(&client->dev, ret,
			      "chip's product ID doesn't match the exact one for this part %d:%d\n",
			      rev_info[PAC193X_PID_IDX],
			      pac193x_chip_config[chip_info->chip_variant].prod_id);
		return -EINVAL;
	}

	dev_dbg(&client->dev, "Chip revision: 0x%02X\n", rev_info[PAC193X_RID_IDX]);
	chip_info->chip_revision = rev_info[PAC193X_RID_IDX];

	return 0;
}

static int pac193x_get_variant(struct pac193x_chip_info *chip_info)
{
	u8 rev_info[PAC193X_ID_REG_LEN];
	int ret;

	ret = pac193x_read_revision(chip_info, (u8 *)rev_info);
	if (!ret) {
		chip_info->chip_variant = rev_info[PAC193X_PID_IDX];
		chip_info->chip_revision = rev_info[PAC193X_RID_IDX];
		switch (chip_info->chip_variant) {
		case PAC1934_PID:
			chip_info->phys_channels = PAC1934_NUM_CHANNELS;
			break;
		case PAC1933_PID:
			chip_info->phys_channels = PAC1933_NUM_CHANNELS;
			break;
		case PAC1932_PID:
			chip_info->phys_channels = PAC1932_NUM_CHANNELS;
			break;
		case PAC1931_PID:
			chip_info->phys_channels = PAC1931_NUM_CHANNELS;
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}

/*
 * documentation related to the ACPI device definition
 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/PAC193X-Integration-Notes-for-Microsoft-Windows-10-and-Windows-11-Driver-Support-DS00002534.pdf
 */
static union acpi_object *pac193x_acpi_eval_function(acpi_handle handle,
						     int revision,
						     int function)
{
	acpi_status status;
	struct acpi_buffer buffer = {ACPI_ALLOCATE_BUFFER, NULL};
	union acpi_object args[PAC193X_ACPI_ARG_COUNT];
	struct acpi_object_list args_list;
	uuid_t uuid;

	uuid_parse(PAC193X_DSM_UUID, &uuid);

	args[0].type = ACPI_TYPE_BUFFER;
	args[0].buffer.length = sizeof(uuid_t);
	args[0].buffer.pointer = (u8 *)&uuid;

	args[1].type = ACPI_TYPE_INTEGER;
	args[1].integer.value = revision;

	args[2].type = ACPI_TYPE_INTEGER;
	args[2].integer.value = function;

	args[3].type = ACPI_TYPE_PACKAGE;
	args[3].package.count = 0;

	args_list.count = PAC193X_ACPI_ARG_COUNT;
	args_list.pointer = &args[0];

	status = acpi_evaluate_object(handle, "_DSM", &args_list, &buffer);

	if (ACPI_FAILURE(status)) {
		kfree(buffer.pointer);
		return NULL;
	}

	return buffer.pointer;
}

static char *pac193x_acpi_get_acpi_match_entry(acpi_handle handle)
{
	acpi_status status;
	union acpi_object *name_object;
	struct acpi_buffer buffer = {ACPI_ALLOCATE_BUFFER, NULL};

	status = acpi_evaluate_object(handle, "_HID", NULL, &buffer);
	name_object = buffer.pointer;

	return name_object->string.pointer;
}

static const char *pac193x_match_acpi_device(struct i2c_client *client,
					     struct pac193x_chip_info *chip_info)
{
	char *name;
	acpi_handle handle;
	union acpi_object *rez;
	unsigned short bi_dir_mask;
	int idx, i;

	handle = ACPI_HANDLE(&client->dev);
	name = pac193x_acpi_get_acpi_match_entry(handle);
	if (!name)
		return NULL;

	rez = pac193x_acpi_eval_function(handle, 0, PAC193X_ACPI_GET_NAMES_AND_MOHMS_VALS);

	if (!rez)
		return NULL;

	for (i = 0; i < rez->package.count; i += 2) {
		idx = i / 2;
		chip_info->channel_names[idx] =
			devm_kmemdup(&client->dev, rez->package.elements[i].string.pointer,
				     (size_t)rez->package.elements[i].string.length + 1,
				     GFP_KERNEL);
		chip_info->channel_names[idx][rez->package.elements[i].string.length] = '\0';
		chip_info->shunts[idx] =
			rez->package.elements[i + 1].integer.value * 1000;
		chip_info->active_channels[idx] = (chip_info->shunts[idx] != 0);
	}

	kfree(rez);

	rez = pac193x_acpi_eval_function(handle, 1, PAC193X_ACPI_GET_UOHMS_VALS);
	if (!rez) {
		/*
		 * initialising with default values
		 * we assume all channels are unidectional(the mask is zero)
		 * and assign the default sampling rate
		 */
		chip_info->sample_rate_value = PAC193X_DEFAULT_CHIP_SAMP_SPEED;
		return name;
	}

	for (i = 0; i < rez->package.count; i++) {
		idx = i;
		chip_info->shunts[idx] = rez->package.elements[i].integer.value;
		chip_info->active_channels[idx] = (chip_info->shunts[idx] != 0);
	}

	kfree(rez);

	rez = pac193x_acpi_eval_function(handle, 1, PAC193X_ACPI_GET_BIPOLAR_SETTINGS);
	if (!rez)
		return NULL;

	bi_dir_mask = rez->package.elements[0].integer.value;
	chip_info->bi_dir[0] = ((bi_dir_mask & (1 << 3)) | (bi_dir_mask & (1 << 7))) != 0;
	chip_info->bi_dir[1] = ((bi_dir_mask & (1 << 2)) | (bi_dir_mask & (1 << 6))) != 0;
	chip_info->bi_dir[2] = ((bi_dir_mask & (1 << 1)) | (bi_dir_mask & (1 << 5))) != 0;
	chip_info->bi_dir[3] = ((bi_dir_mask & (1 << 0)) | (bi_dir_mask & (1 << 4))) != 0;

	kfree(rez);

	rez = pac193x_acpi_eval_function(handle, 1, PAC193X_ACPI_GET_SAMP);
	if (!rez)
		return NULL;

	chip_info->sample_rate_value = rez->package.elements[0].integer.value;
	kfree(rez);

	return name;
}

static const struct of_device_id pac193x_of_match[] = {
	{
		.compatible = "microchip,pac1931",
		.data = &pac193x_chip_config[PAC1931]
	},
	{
		.compatible = "microchip,pac1932",
		.data = &pac193x_chip_config[PAC1932]
	},
	{
		.compatible = "microchip,pac1933",
		.data = &pac193x_chip_config[PAC1933]
	},
	{
		.compatible = "microchip,pac1934",
		.data = &pac193x_chip_config[PAC1934]
	},
	{}
};

static int pac193x_match_of_device(struct i2c_client *client,
				   struct pac193x_chip_info *chip_info)
{
	struct fwnode_handle *node, *fwnode;
	unsigned int current_channel;
	int idx, ret;

	chip_info->sample_rate_value = 1024;
	current_channel = 1;

	fwnode = dev_fwnode(&client->dev);
	fwnode_for_each_available_child_node(fwnode, node) {
		ret = fwnode_property_read_u32(node, "reg", &idx);
		if (ret) {
			dev_err_probe(&client->dev, ret,
				      "reading invalid channel index\n");
			fwnode_handle_put(node);
			return ret;
		}
		/* adjust idx to match channel index (1 to 4) from the datasheet */
		idx--;
		if (current_channel >= (chip_info->phys_channels + 1) ||
		    idx >= chip_info->phys_channels || idx < 0) {
			dev_err_probe(&client->dev, -EINVAL,
				      "%s: invalid channel_index %d value\n",
				      fwnode_get_name(node), idx);
			fwnode_handle_put(node);
			return -EINVAL;
		}

		/* enable channel */
		chip_info->active_channels[idx] = true;

		ret = fwnode_property_read_u32(node, "shunt-resistor-micro-ohms",
					       &chip_info->shunts[idx]);
		if (ret) {
			dev_err_probe(&client->dev, ret,
				      "%s: invalid shunt-resistor value: %d\n",
				      fwnode_get_name(node), chip_info->shunts[idx]);
			fwnode_handle_put(node);
			return ret;
		}

		ret = fwnode_property_read_string(node, "label",
						  (const char **)&chip_info->channel_names[idx]);
		if (ret) {
			dev_err_probe(&client->dev, ret,
				      "%s: invalid rail-name value\n",
				      fwnode_get_name(node));
			fwnode_handle_put(node);
			return ret;
		}

		chip_info->bi_dir[idx] =
			fwnode_property_read_bool(node, "bipolar");

		current_channel++;
	}

	return 0;
}

static int pac193x_chip_configure(struct pac193x_chip_info *chip_info)
{
	int cnt, ret;
	struct i2c_client *client = chip_info->client;
	u8 regs[PAC193X_CTRL_STATUS_INFO_LEN], idx, ctrl_reg;
	u32 wait_time;

	cnt = 0;
	chip_info->chip_reg_data.num_enabled_channels = 0;
	while (cnt < chip_info->phys_channels) {
		if (chip_info->active_channels[cnt])
			chip_info->chip_reg_data.num_enabled_channels++;
		cnt++;
	}

	/*
	 * read whatever information was gathered before the driver was loaded
	 * establish which channels are enabled/disabled and then establish the
	 * information retrieval mode (using SKIP or no).
	 * Read the chip ID values
	 */
	ret = pac193x_i2c_read(client, PAC193X_CTRL_STAT_REGS_ADDR,
			       (u8 *)regs, sizeof(regs));
	if (ret) {
		dev_err_probe(&client->dev, ret,
			      "%s - cannot read regs from 0x%02X\n",
			      __func__, PAC193X_CTRL_STAT_REGS_ADDR);
		return ret;
	}

	/* write the CHANNEL_DIS and the NEG_PWR registers */
	regs[PAC193X_CHANNEL_DIS_REG_OFF] =
		PAC193X_CHAN_DIS_CH1_OFF(chip_info->active_channels[PAC193X_CH_1]) |
		PAC193X_CHAN_DIS_CH2_OFF(chip_info->active_channels[PAC193X_CH_2]) |
		PAC193X_CHAN_DIS_CH3_OFF(chip_info->active_channels[PAC193X_CH_3]) |
		PAC193X_CHAN_DIS_CH4_OFF(chip_info->active_channels[PAC193X_CH_4]) |
		PAC193X_SMBUS_TIMEOUT_EN(0) |
		PAC193X_SMBUS_BYTECOUNT_EN(0) |
		PAC193X_SMBUS_NO_SKIP_EN(0);

	regs[PAC193X_NEG_PWR_REG_OFF] =
		PAC193X_NEG_PWR_CH1_BIDI(chip_info->bi_dir[PAC193X_CH_1]) |
		PAC193X_NEG_PWR_CH2_BIDI(chip_info->bi_dir[PAC193X_CH_2]) |
		PAC193X_NEG_PWR_CH3_BIDI(chip_info->bi_dir[PAC193X_CH_3]) |
		PAC193X_NEG_PWR_CH4_BIDI(chip_info->bi_dir[PAC193X_CH_4]) |
		PAC193X_NEG_PWR_CH1_BIDV(chip_info->bi_dir[PAC193X_CH_1]) |
		PAC193X_NEG_PWR_CH2_BIDV(chip_info->bi_dir[PAC193X_CH_2]) |
		PAC193X_NEG_PWR_CH3_BIDV(chip_info->bi_dir[PAC193X_CH_3]) |
		PAC193X_NEG_PWR_CH4_BIDV(chip_info->bi_dir[PAC193X_CH_4]);

	/* no SLOW triggered REFRESH, clear POR */
	regs[PAC193X_SLOW_REG_OFF] = 0;

	ret = pac193x_i2c_write(client, PAC193X_CTRL_STAT_REGS_ADDR, (u8 *)regs, 3);
	if (ret)
		return ret;

	ctrl_reg = PAC193X_CRTL_SAMPLE_RATE_SET(chip_info->crt_samp_spd_bitfield);
	ret = i2c_smbus_write_byte_data(client, PAC193X_CTRL_REG_ADDR, ctrl_reg);
	if (ret)
		return ret;

	/*
	 * send a REFRESH to the chip, so the new settings take place
	 * as well as resetting the accumulators
	 */
	ret = i2c_smbus_write_byte(client, PAC193X_REFRESH_REG_ADDR);
	if (ret) {
		dev_err(&client->dev,
			"%s - cannot send 0x%02X\n",
			__func__, PAC193X_REFRESH_REG_ADDR);
		return ret;
	}

	/*
	 * get the current(in the chip) sampling speed and compute the
	 * required timeout based on its value
	 * the timeout is 1/sampling_speed
	 */
	idx = regs[PAC193X_CTRL_ACT_REG_OFF] >> PAC193X_SAMPLE_RATE_SHIFT;
	wait_time = (1024 / samp_rate_map_tbl[idx]) * 1000;

	/*
	 * wait the maximum amount of time to be on the safe side
	 * the maximum wait time is for 8sps
	 */
	usleep_range(wait_time, wait_time + 100);

	INIT_DELAYED_WORK(&chip_info->work_chip_rfsh, pac193x_work_periodic_rfsh);
	/* Setup the latest moment for reading the regs before saturation */
	schedule_delayed_work(&chip_info->work_chip_rfsh,
			      msecs_to_jiffies(PAC193X_MAX_RFSH_LIMIT_MS));

	return 0;
}

static int pac193x_prep_iio_channels(struct pac193x_chip_info *chip_info, struct iio_dev *indio_dev)
{
	struct i2c_client *client;
	struct iio_chan_spec *ch_sp;
	int channel_size, attribute_count, cnt;
	void *dyn_ch_struct, *tmp_data;

	client = chip_info->client;

	/* find out dynamically how many IIO channels we need */
	attribute_count = 0;
	channel_size = 0;
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (chip_info->active_channels[cnt]) {
			/* add the size of the properties of one chip physical channel */
			channel_size += sizeof(pac193x_single_channel);
			/* count how many enabled channels we have */
			attribute_count += ARRAY_SIZE(pac193x_single_channel);
			dev_info(&client->dev, ":%s: Channel %d active\n", __func__, cnt + 1);
		}
	}

	dyn_ch_struct = kzalloc(channel_size, GFP_KERNEL);
	if (!dyn_ch_struct)
		return -EINVAL;

	tmp_data = dyn_ch_struct;

	/* populate the dynamic channels and make all the adjustments */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (chip_info->active_channels[cnt]) {
			memcpy(tmp_data, pac193x_single_channel, sizeof(pac193x_single_channel));
			ch_sp = (struct iio_chan_spec *)tmp_data;
			ch_sp[IIO_EN].channel = cnt + 1;
			ch_sp[IIO_EN].scan_index = cnt;
			ch_sp[IIO_EN].address = cnt + PAC193X_VPOWER_ACC_1_ADDR;
			ch_sp[IIO_POW].channel = cnt + 1;
			ch_sp[IIO_POW].scan_index = cnt;
			ch_sp[IIO_POW].address = cnt + PAC193X_VPOWER_1_ADDR;
			ch_sp[IIO_VOLT].channel = cnt + 1;
			ch_sp[IIO_VOLT].scan_index = cnt;
			ch_sp[IIO_VOLT].address = cnt + PAC193X_VBUS_1_ADDR;
			ch_sp[IIO_CRT].channel = cnt + 1;
			ch_sp[IIO_CRT].scan_index = cnt;
			ch_sp[IIO_CRT].address = cnt + PAC193X_VSENSE_1_ADDR;
			ch_sp[IIO_VOLTAVG].channel = cnt + 1;
			ch_sp[IIO_VOLTAVG].scan_index = cnt;
			ch_sp[IIO_VOLTAVG].address = cnt + PAC193X_VBUS_AVG_1_ADDR;
			ch_sp[IIO_CRTAVG].channel = cnt + 1;
			ch_sp[IIO_CRTAVG].scan_index = cnt;
			ch_sp[IIO_CRTAVG].address = cnt + PAC193X_VSENSE_AVG_1_ADDR;

			/*
			 * now modify the parameters in all channels if the
			 * whole chip rail(channel) is bi-directional
			 */
			if (chip_info->bi_dir[cnt]) {
				ch_sp[IIO_EN].scan_type.sign = 's';
				ch_sp[IIO_EN].scan_type.realbits = PAC193X_ENERGY_S_RES;
				ch_sp[IIO_POW].scan_type.sign = 's';
				ch_sp[IIO_POW].scan_type.realbits = PAC193X_POWER_S_RES;
				ch_sp[IIO_VOLT].scan_type.sign = 's';
				ch_sp[IIO_VOLT].scan_type.realbits = PAC193X_VOLTAGE_S_RES;
				ch_sp[IIO_CRT].scan_type.sign = 's';
				ch_sp[IIO_CRT].scan_type.realbits = PAC193X_CURRENT_S_RES;
				ch_sp[IIO_VOLTAVG].scan_type.sign = 's';
				ch_sp[IIO_VOLTAVG].scan_type.realbits = PAC193X_VOLTAGE_S_RES;
				ch_sp[IIO_CRTAVG].scan_type.sign = 's';
				ch_sp[IIO_CRTAVG].scan_type.realbits = PAC193X_CURRENT_S_RES;
			}
			tmp_data += sizeof(pac193x_single_channel);
		}
	}

	/*
	 * send the updated dynamic channel structure information towards IIO
	 * prepare the required field for IIO class registration
	 */
	indio_dev->num_channels = attribute_count;
	indio_dev->channels = devm_kmemdup(&client->dev,
					   (const struct iio_chan_spec *)dyn_ch_struct,
					   channel_size, GFP_KERNEL);

	kfree(dyn_ch_struct);

	if (!indio_dev->channels)
		return -EINVAL;

	return 0;
}

static ssize_t reset_accumulators_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int ret, i;
	u8 refresh_cmd = PAC193X_REFRESH_REG_ADDR;

	ret = i2c_smbus_write_byte(chip_info->client, refresh_cmd);
	if (ret) {
		dev_err(&indio_dev->dev,
			"%s - cannot send 0x%02X\n",
			__func__, refresh_cmd);
	}

	for (i = 0 ; i < chip_info->phys_channels; i++)
		chip_info->chip_reg_data.energy_sec_acc[i] = 0;

	return count;
}

static IIO_DEVICE_ATTR(in_shunt_resistor_1, 0644, shunt_value_show, shunt_value_store, 0);
static IIO_DEVICE_ATTR(in_shunt_resistor_2, 0644, shunt_value_show, shunt_value_store, 0);
static IIO_DEVICE_ATTR(in_shunt_resistor_3, 0644, shunt_value_show, shunt_value_store, 0);
static IIO_DEVICE_ATTR(in_shunt_resistor_4, 0644, shunt_value_show, shunt_value_store, 0);

static IIO_DEVICE_ATTR(channel_name_1, 0444, channel_name_show, NULL, 0);
static IIO_DEVICE_ATTR(channel_name_2, 0444, channel_name_show, NULL, 0);
static IIO_DEVICE_ATTR(channel_name_3, 0444, channel_name_show, NULL, 0);
static IIO_DEVICE_ATTR(channel_name_4, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(reset_accumulators, 0200, NULL, reset_accumulators_store, 0);

static struct attribute *pac193x_all_attributes[] = {
	PAC193X_DEV_ATTR(in_shunt_resistor_1),
	PAC193X_DEV_ATTR(channel_name_1),
	PAC193X_DEV_ATTR(in_shunt_resistor_2),
	PAC193X_DEV_ATTR(channel_name_2),
	PAC193X_DEV_ATTR(in_shunt_resistor_3),
	PAC193X_DEV_ATTR(channel_name_3),
	PAC193X_DEV_ATTR(in_shunt_resistor_4),
	PAC193X_DEV_ATTR(channel_name_4),
	PAC193X_DEV_ATTR(reset_accumulators),
	NULL
};

static int pac193x_prep_custom_attributes(struct pac193x_chip_info *chip_info,
					  struct iio_dev *indio_dev)
{
	int i, j, active_channels_count = 0;
	struct attribute **pac193x_custom_attributes;
	struct attribute_group *pac193x_group;
	struct i2c_client *client = chip_info->client;

	for (i = 0 ; i < chip_info->phys_channels; i++)
		if (chip_info->active_channels[i])
			active_channels_count++;

	pac193x_group = devm_kzalloc(&client->dev, sizeof(*pac193x_group), GFP_KERNEL);

	pac193x_custom_attributes = devm_kzalloc(&client->dev,
						 (PAC193X_CUSTOM_ATTR_FOR_CHANNEL *
						 active_channels_count +
						 PAC193X_SHARED_DEVATTRS_COUNT)
						 * sizeof(*pac193x_group) + 1,
						 GFP_KERNEL);
	j = 0;

	for (i = 0 ; i < chip_info->phys_channels; i++) {
		if (chip_info->active_channels[i]) {
			pac193x_custom_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * j] =
			pac193x_all_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * i];
			pac193x_custom_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * j + 1] =
			pac193x_all_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * i + 1];
			j++;
		}
	}

	for (i = 0; i < PAC193X_SHARED_DEVATTRS_COUNT; i++)
		pac193x_custom_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL *
			active_channels_count + i] =
			pac193x_all_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL *
			chip_info->phys_channels + i];

	pac193x_group->attrs = pac193x_custom_attributes;
	chip_info->pac193x_info.attrs = pac193x_group;

	return 0;
}

static void pac193x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&client->dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);

	cancel_delayed_work_sync(&chip_info->work_chip_rfsh);
}

static const struct i2c_device_id pac193x_id[] = {
	{ "pac1931", PAC1931 },
	{ "pac1932", PAC1932 },
	{ "pac1933", PAC1933 },
	{ "pac1934", PAC1934 },
	{}
};

static int pac193x_probe(struct i2c_client *client)
{
	struct pac193x_chip_info *chip_info;
	struct iio_dev *indio_dev;
	const char *name = NULL;
	const struct of_device_id *match;
	int cnt, ret, dev_id = 0;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip_info));
	if (!indio_dev) {
		dev_err_probe(&client->dev, PTR_ERR(indio_dev),
			      "Can't allocate iio device\n");
		return -ENOMEM;
	}

	chip_info = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	chip_info->client = client;

	/*
	 * load default settings - all channels disabled,
	 * uni directional flow
	 */
	for (cnt = 0; cnt < PAC193X_MAX_NUM_CHANNELS; cnt++) {
		chip_info->active_channels[cnt] = false;
		chip_info->bi_dir[cnt] = false;
	}

	chip_info->crt_samp_spd_bitfield = PAC193X_SAMP_1024SPS;

	if (ACPI_HANDLE(&client->dev)) {
		pac193x_get_variant(chip_info);

		switch (chip_info->chip_variant) {
		case PAC1934_PID:
			client->dev.init_name = "pac1934";
			break;
		case PAC1933_PID:
			client->dev.init_name = "pac1933";
			break;
		case PAC1932_PID:
			client->dev.init_name = "pac1932";
			break;
		case PAC1931_PID:
			client->dev.init_name = "pac1931";
			break;
		default:
			return -EINVAL;
		}

		for (cnt = 0; cnt < PAC193X_MAX_NUM_CHANNELS; cnt++)
			chip_info->shunts[cnt] = SHUNT_UOHMS_DEFAULT;

		name = pac193x_match_acpi_device(client, chip_info);
	} else {
		dev_id = i2c_match_id(pac193x_id, client)->driver_data;

		chip_info->chip_variant = dev_id;
		chip_info->phys_channels = pac193x_chip_config[dev_id].phys_channels;

		/* identify the chip we have to deal with */
		ret = pac193x_chip_identify(chip_info);
		if (ret)
			return -EINVAL;

		/* check if we find the device within DT */
		if (!client->dev.of_node || (!of_get_next_child(client->dev.of_node, NULL)))
			return -EINVAL;

		match = of_match_node(pac193x_of_match, client->dev.of_node);
		if (match) {
			ret = pac193x_match_of_device(client, chip_info);
			if (!ret)
				name = match->compatible;
		}
	}

	if (!name) {
		dev_err_probe(&client->dev, PTR_ERR(indio_dev),
			      "parameter parsing returned an error\n");
		return -EINVAL;
	}

	mutex_init(&chip_info->lock);

	/*
	 * do now any chip specific initialization (e.g. read/write
	 * some registers), enable/disable certain channels, change the sampling
	 * rate to the requested value
	 */
	ret = pac193x_chip_configure(chip_info);
	if (ret < 0)
		goto fail;

	/* prepare the channel information */
	ret = pac193x_prep_iio_channels(chip_info, indio_dev);
	if (ret < 0)
		goto fail;

	ret = pac193x_prep_custom_attributes(chip_info, indio_dev);
	if (ret < 0) {
		dev_err_probe(&indio_dev->dev, ret,
			      "Can't configure custom attributes for PAC193X device\n");
		goto fail;
	}

	chip_info->pac193x_info.read_raw = pac193x_read_raw;
	chip_info->pac193x_info.read_avail = pac193x_read_avail;
	chip_info->pac193x_info.write_raw = pac193x_write_raw;

	indio_dev->info = &chip_info->pac193x_info;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/*
	 * read whatever has been accumulated in the chip so far
	 * and reset the accumulators
	 */
	ret = pac193x_reg_snapshot(chip_info, true, PAC193X_REFRESH_REG_ADDR,
				   PAC193X_MIN_UPDATE_WAIT_TIME_US);
	if (ret < 0)
		goto fail;

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
		dev_err_probe(&indio_dev->dev, ret,
			      "Can't register IIO device\n");
		goto fail;
	}

	return 0;

fail:
	cancel_delayed_work_sync(&chip_info->work_chip_rfsh);

	return ret;
}

MODULE_DEVICE_TABLE(i2c, pac193x_id);

MODULE_DEVICE_TABLE(of, pac193x_of_match);

static const struct acpi_device_id pac193x_acpi_match[] = {
	{"MCHP1930", 0},
	{ }
};
MODULE_DEVICE_TABLE(acpi, pac193x_acpi_match);

static struct i2c_driver pac193x_driver = {
	.driver	 = {
		.name = "pac193x",
		.of_match_table = pac193x_of_match,
		.acpi_match_table = ACPI_PTR(pac193x_acpi_match)
		},
	.probe_new = pac193x_probe,
	.remove = pac193x_remove,
	.id_table = pac193x_id,
};

module_i2c_driver(pac193x_driver);

MODULE_AUTHOR("Bogdan Bolocan <bogdan.bolocan@microchip.com>");
MODULE_AUTHOR("Victor Tudose");
MODULE_AUTHOR("Marius Cristea <marius.cristea@microchip.com>");
MODULE_DESCRIPTION("IIO driver for PAC193X Multi-Channel DC Power/Energy Monitor");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2.1");
