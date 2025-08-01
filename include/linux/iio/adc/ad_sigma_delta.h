/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Support code for Analog Devices Sigma-Delta ADCs
 *
 * Copyright 2012 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 */
#ifndef __AD_SIGMA_DELTA_H__
#define __AD_SIGMA_DELTA_H__

#include <linux/iio/iio.h>

enum ad_sigma_delta_mode {
	AD_SD_MODE_CONTINUOUS = 0,
	AD_SD_MODE_SINGLE = 1,
	AD_SD_MODE_IDLE = 2,
	AD_SD_MODE_POWERDOWN = 3,
};

/**
 * struct ad_sigma_delta_calib_data - Calibration data for Sigma Delta devices
 * @mode: Calibration mode.
 * @channel: Calibration channel.
 */
struct ad_sd_calib_data {
	unsigned int mode;
	unsigned int channel;
};

struct ad_sigma_delta;
struct device;
struct gpio_desc;
struct iio_dev;
struct spi_offload;
struct spi_offload_trigger;

/**
 * struct ad_sigma_delta_info - Sigma Delta driver specific callbacks and options
 * @set_channel: Will be called to select the current channel, may be NULL.
 * @append_status: Will be called to enable status append at the end of the sample, may be NULL.
 * @set_mode: Will be called to select the current mode, may be NULL.
 * @disable_all: Will be called to disable all channels, may be NULL.
 * @disable_one: Will be called to disable a single channel after
 *		ad_sigma_delta_single_conversion(), may be NULL.
 *		Usage of this callback expects iio_chan_spec.address to contain
 *		the value required for the driver to identify the channel.
 * @postprocess_sample: Is called for each sampled data word, can be used to
 *		modify or drop the sample data, it, may be NULL.
 * @has_registers: true if the device has writable and readable registers, false
 *		if there is just one read-only sample data shift register.
 * @has_named_irqs: Set to true if there is more than one IRQ line.
 * @supports_spi_offload: Set to true if the driver supports SPI offload. Often
 *		special considerations are needed for scan_type and other channel
 *		info, so individual drivers have to set this to let the core
 *		code know that it can use SPI offload if it is available.
 * @addr_shift: Shift of the register address in the communications register.
 * @read_mask: Mask for the communications register having the read bit set.
 * @status_ch_mask: Mask for the channel number stored in status register.
 * @data_reg: Address of the data register, if 0 the default address of 0x3 will
 *   be used.
 * @irq_flags: flags for the interrupt used by the triggered buffer
 * @num_slots: Number of sequencer slots
 * @num_resetclks: Number of SPI clk cycles with MOSI=1 to reset the chip.
 */
struct ad_sigma_delta_info {
	int (*set_channel)(struct ad_sigma_delta *, unsigned int channel);
	int (*append_status)(struct ad_sigma_delta *, bool append);
	int (*set_mode)(struct ad_sigma_delta *, enum ad_sigma_delta_mode mode);
	int (*disable_all)(struct ad_sigma_delta *);
	int (*disable_one)(struct ad_sigma_delta *, unsigned int chan);
	int (*postprocess_sample)(struct ad_sigma_delta *, unsigned int raw_sample);
	bool has_registers;
	bool has_named_irqs;
	bool supports_spi_offload;
	unsigned int addr_shift;
	unsigned int read_mask;
	unsigned int status_ch_mask;
	unsigned int data_reg;
	unsigned long irq_flags;
	unsigned int num_slots;
	unsigned int num_resetclks;
};

/**
 * struct ad_sigma_delta - Sigma Delta device struct
 * @spi: The spi device associated with the Sigma Delta device.
 * @trig: The IIO trigger associated with the Sigma Delta device.
 *
 * Most of the fields are private to the sigma delta library code and should not
 * be accessed by individual drivers.
 */
struct ad_sigma_delta {
	struct spi_device	*spi;
	struct iio_trigger	*trig;

/* private: */
	struct completion	completion;
	spinlock_t		irq_lock; /* protects .irq_dis and irq en/disable state */
	bool			irq_dis;

	bool			bus_locked;
	bool			keep_cs_asserted;

	u8			comm;

	const struct ad_sigma_delta_info *info;
	unsigned int		active_slots;
	unsigned int		current_slot;
	unsigned int		num_slots;
	struct gpio_desc	*rdy_gpiod;
	int			irq_line;
	bool			status_appended;
	/* map slots to channels in order to know what to expect from devices */
	unsigned int		*slots;
	struct spi_message	sample_msg;
	struct spi_transfer	sample_xfer[2];
	u8			*samples_buf;
	struct spi_offload	*offload;
	struct spi_offload_trigger *offload_trigger;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * 'tx_buf' is up to 32 bits.
	 * 'rx_buf' is up to 32 bits per sample + 64 bit timestamp,
	 * rounded to 16 bytes to take into account padding.
	 */
	u8				tx_buf[4] __aligned(IIO_DMA_MINALIGN);
	u8				rx_buf[16] __aligned(8);
	u8				sample_addr;
};

static inline bool ad_sigma_delta_has_spi_offload(struct ad_sigma_delta *sd)
{
	return sd->offload != NULL;
}

static inline int ad_sigma_delta_set_channel(struct ad_sigma_delta *sd,
	unsigned int channel)
{
	if (sd->info->set_channel)
		return sd->info->set_channel(sd, channel);

	return 0;
}

static inline int ad_sigma_delta_append_status(struct ad_sigma_delta *sd, bool append)
{
	int ret;

	if (sd->info->append_status) {
		ret = sd->info->append_status(sd, append);
		if (ret < 0)
			return ret;

		sd->status_appended = append;
	}

	return 0;
}

static inline int ad_sigma_delta_disable_all(struct ad_sigma_delta *sd)
{
	if (sd->info->disable_all)
		return sd->info->disable_all(sd);

	return 0;
}

static inline int ad_sigma_delta_disable_one(struct ad_sigma_delta *sd,
					     unsigned int chan)
{
	if (sd->info->disable_one)
		return sd->info->disable_one(sd, chan);

	return 0;
}

static inline int ad_sigma_delta_set_mode(struct ad_sigma_delta *sd,
	unsigned int mode)
{
	if (sd->info->set_mode)
		return sd->info->set_mode(sd, mode);

	return 0;
}

static inline int ad_sigma_delta_postprocess_sample(struct ad_sigma_delta *sd,
	unsigned int raw_sample)
{
	if (sd->info->postprocess_sample)
		return sd->info->postprocess_sample(sd, raw_sample);

	return 0;
}

void ad_sd_set_comm(struct ad_sigma_delta *sigma_delta, u8 comm);
int ad_sd_write_reg(struct ad_sigma_delta *sigma_delta, unsigned int reg,
	unsigned int size, unsigned int val);
int ad_sd_read_reg(struct ad_sigma_delta *sigma_delta, unsigned int reg,
	unsigned int size, unsigned int *val);

int ad_sd_reset(struct ad_sigma_delta *sigma_delta);

int ad_sigma_delta_single_conversion(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val);
int ad_sd_calibrate(struct ad_sigma_delta *sigma_delta,
	unsigned int mode, unsigned int channel);
int ad_sd_calibrate_all(struct ad_sigma_delta *sigma_delta,
	const struct ad_sd_calib_data *cd, unsigned int n);
int ad_sd_init(struct ad_sigma_delta *sigma_delta, struct iio_dev *indio_dev,
	struct spi_device *spi, const struct ad_sigma_delta_info *info);

int devm_ad_sd_setup_buffer_and_trigger(struct device *dev, struct iio_dev *indio_dev);

int ad_sd_validate_trigger(struct iio_dev *indio_dev, struct iio_trigger *trig);

#endif
