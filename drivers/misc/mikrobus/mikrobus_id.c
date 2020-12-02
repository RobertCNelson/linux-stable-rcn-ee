// SPDX-License-Identifier: GPL-2.0-only
/*
 * mikrobus_id.c - w1 mikroBUS ID family EEPROM driver
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

#include "mikrobus_core.h"

#define W1_EEPROM_MIKROBUS_ID	0xAC

#define W1_MIKROBUS_ID_EEPROM_SIZE	512
#define W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE	32
#define W1_MIKROBUS_ID_READ_EEPROM	0xF0
#define W1_MIKROBUS_ID_EEPROM_READ_RETRIES	10
#define W1_MIKROBUS_ID_EEPROM_WRITE_SCRATCH	0x0F
#define W1_MIKROBUS_ID_EEPROM_READ_SCRATCH	0xAA
#define W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH	0x55
#define W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH_ES	0x40

#define W1_MIKROBUS_ID_EEPROM_TPROG_MS		40

static int w1_mikrobus_id_readblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 wrbuf[3];
	u8 cmp[W1_MIKROBUS_ID_EEPROM_SIZE];
	int tries = W1_MIKROBUS_ID_EEPROM_READ_RETRIES;

	do {
		wrbuf[0] = W1_MIKROBUS_ID_READ_EEPROM;
		wrbuf[1] = count >> 8;
		wrbuf[2] = count & 0xFF;

		w1_reset_select_slave(sl);
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, buf, count);

		w1_reset_select_slave(sl);
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, cmp, count);

		if (!memcmp(cmp, buf, count))
			return 0;
	} while (--tries);

	dev_err(&sl->dev, "proof reading failed %d times\n",
			W1_MIKROBUS_ID_EEPROM_READ_RETRIES);
	return -EIO;
}

static int w1_mikrobus_id_writeblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 wrbuf[4];
	u8 wrdata[W1_MIKROBUS_ID_EEPROM_SIZE];
	u8 scratchpad_verify[W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE + 3];
	u8 write_scratchpad_crc[2];
	u16 wraddr = 0;
	u16 len = W1_MIKROBUS_ID_EEPROM_SIZE;
	int k;

	memcpy(wrdata, buf, W1_MIKROBUS_ID_EEPROM_SIZE);

	while(len > 0) {
		wrbuf[0] = W1_MIKROBUS_ID_EEPROM_WRITE_SCRATCH;
		wrbuf[1] = (wraddr & 0x100) >> 8;
		wrbuf[2] = wraddr & 0xFF;

		/* write scratchpad */
		w1_reset_select_slave(sl);
		w1_write_block(sl->master, wrbuf, 3);
		/* write_block fails */
		// w1_write_block(sl->master, wrdata + wraddr, W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
		for(k = 0; k < W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE; k++){
			w1_write_8(sl->master, wrdata[wraddr + k]);
			udelay(50); //delay to fix slave going non-responsive
		}
		w1_read_block(sl->master, write_scratchpad_crc, 2);
		msleep(10); //delay to fix slave going non-responsive 
		/* verify scratchpad */
		w1_reset_select_slave(sl);
		w1_write_8(sl->master, W1_MIKROBUS_ID_EEPROM_READ_SCRATCH);
		/* read_block fails*/
		// w1_read_block(sl->master, scratchpad_verify, W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE + 3);
		for(k = 0; k < (W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE + 3); k++){
			scratchpad_verify[k] = w1_read_8(sl->master);
			udelay(50); //delay to fix slave going non-responsive
		}
		msleep(10); //delay to fix slave going non-responsive
		/* copy scratchpad */
		wrbuf[0] = W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH;
		wrbuf[1] = (wraddr & 0x100) >> 8;
		wrbuf[2] = wraddr & 0xFF;
		wrbuf[3] = W1_MIKROBUS_ID_EEPROM_COPY_SCRATCH_ES;
		w1_reset_select_slave(sl);
		w1_write_block(sl->master, wrbuf, 4);

		msleep(W1_MIKROBUS_ID_EEPROM_TPROG_MS);
		wraddr += W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE;
		len -= W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE;
	}
	return 0;
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

static int w1_mikrobus_id_nvmem_write(void *priv, unsigned int off, void *buf, size_t count)
{
	struct w1_slave *sl = priv;
	int ret;

	if (off > W1_MIKROBUS_ID_EEPROM_SIZE)
		return -EINVAL;

	if ((off + count) > W1_MIKROBUS_ID_EEPROM_SIZE)
		count = W1_MIKROBUS_ID_EEPROM_SIZE - off;

	mutex_lock(&sl->master->bus_mutex);
	ret = w1_mikrobus_id_writeblock(sl, off, count, buf);
	mutex_unlock(&sl->master->bus_mutex);
	
	return ret;
}

static int w1_mikrobus_id_add_slave(struct w1_slave *sl)
{
	struct nvmem_device *nvmem;
	struct mikrobus_port *port;
	struct nvmem_config nvmem_cfg = {
		.dev = &sl->dev,
		.reg_read = w1_mikrobus_id_nvmem_read,
		.reg_write = w1_mikrobus_id_nvmem_write,
		.type = NVMEM_TYPE_EEPROM,
		.read_only = false,
		.word_size = 1,
		.stride = 1,
		.size = W1_MIKROBUS_ID_EEPROM_SIZE,
		.priv = sl,
	};

	port = mikrobus_find_port_by_w1_master(sl->master);
	if(!port)
		return -ENODEV;

	set_bit(W1_ABORT_SEARCH, &sl->master->flags);
	nvmem_cfg.name = port->name;
	nvmem = devm_nvmem_register(&sl->dev, &nvmem_cfg);
	port->eeprom = nvmem;
	mikrobus_port_scan_eeprom(port);

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