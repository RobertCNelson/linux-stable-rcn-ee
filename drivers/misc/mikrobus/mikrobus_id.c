// SPDX-License-Identifier: GPL-2.0-only
/*
 * mikrobus_id.c - w1 family ac mikroBUS ID EEPROM driver
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/delay.h>

#include <linux/w1.h>
#include <linux/nvmem-provider.h>

#define W1_EEPROM_MIKROBUS_ID	0xAC

#define W1_MIKROBUS_ID_EEPROM_SIZE	512
#define W1_MIKROBUS_ID_READ_EEPROM	0xF0
#define W1_MIKROBUS_ID_EEPROM_READ_RETRIES	10

static int w1_mikrobus_id_readblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 wrbuf[3];
	u8 cmp[W1_MIKROBUS_ID_EEPROM_SIZE];
	int tries = W1_MIKROBUS_ID_EEPROM_READ_RETRIES;

	do {
		wrbuf[0] = W1_MIKROBUS_ID_READ_EEPROM;
		wrbuf[1] = (count & 0x100) >> 8;
		wrbuf[1] = count & 0xFF;

		if (w1_reset_select_slave(sl))
			return -ENODEV;
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, buf, count);

		if (w1_reset_select_slave(sl))
			return -ENODEV;
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, cmp, count);

		if (!memcmp(cmp, buf, count))
			return 0;
	} while (--tries);

	dev_err(&sl->dev, "proof reading failed %d times\n",
			W1_MIKROBUS_ID_EEPROM_READ_RETRIES);
	return -EIO;
}

static int w1_mikrobus_id_nvmem_read(void *priv, unsigned int off, void *buf, size_t count)
{
	struct w1_slave *sl = priv;
	int ret;

	if (off > W1_MIKROBUS_ID_EEPROM_SIZE)
		return -EINVAL;

	if ((off + count) > W1_MIKROBUS_ID_EEPROM_SIZE)
		count = W1_MIKROBUS_ID_EEPROM_SIZE - off;

	mutex_lock(&sl->master->bus_mutex);
	ret = w1_mikrobus_id_readblock(sl, off, count, buf);
	mutex_unlock(&sl->master->bus_mutex);
	
	return ret;
}

static int w1_mikrobus_id_add_slave(struct w1_slave *sl)
{
	struct nvmem_device *nvmem;
	struct nvmem_config nvmem_cfg = {
		.dev = &sl->dev,
		.name = sl->master->bus_master->dev_id,
		.reg_read = w1_mikrobus_id_nvmem_read,
		.type = NVMEM_TYPE_OTP,
		.read_only = true,
		.word_size = 1,
		.size = W1_MIKROBUS_ID_EEPROM_SIZE,
		.priv = sl,
		.id = -1
	};

	nvmem = devm_nvmem_register(&sl->dev, &nvmem_cfg);
	return PTR_ERR_OR_ZERO(nvmem);
}

static const struct w1_family_ops w1_family_mikrobus_id_fops = {
	.add_slave		= w1_mikrobus_id_add_slave,
};

static struct w1_family w1_family_mikrobus_id = {
	.fid = W1_EEPROM_MIKROBUS_ID,
	.fops = &w1_family_mikrobus_id_fops,
};
module_w1_family(w1_family_mikrobus_id);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("w1 family ac driver for mikroBUS ID EEPROM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("w1-family-" __stringify(W1_EEPROM_MIKROBUS_ID));