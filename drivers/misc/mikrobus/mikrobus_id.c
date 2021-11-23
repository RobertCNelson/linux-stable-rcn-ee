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

#include <linux/mikrobus.h>

#define W1_EEPROM_MIKROBUS_SECONDARY_ID	0x4C
#define W1_EEPROM_MIKROBUS_ID	0x43
#define W1_MIKROBUS_ID_EEPROM_SIZE	 0x0A00
#define W1_MIKROBUS_ID_EEPROM_SECONDARY_SIZE	 0x0200
#define W1_MIKROBUS_ID_EEPROM_SECONDARY_PAGE_SIZE	32
#define W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE	32
#define W1_MIKROBUS_ID_EEPROM_VERIFY_SCRATCH_SIZE	35
#define W1_MIKROBUS_ID_READ_EEPROM	0xF0
#define W1_MIKROBUS_ID_READ_SECONDARY_EEPROM 0x69
#define W1_MIKROBUS_ID_RELEASE_SECONDARY_EEPROM 0xAA
#define W1_MIKROBUS_ID_EEPROM_READ_RETRIES	10
#define W1_MIKROBUS_ID_EEPROM_TPROG_MS		20
#define MIKROBUS_ID_USER_EEPROM_ADDR	0x0A0A

static int w1_mikrobus_id_readblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 wrbuf[3];
	u8 *cmp;
	int tries = W1_MIKROBUS_ID_EEPROM_READ_RETRIES;
	
	do {
		wrbuf[0] = W1_MIKROBUS_ID_READ_EEPROM;
		wrbuf[1] = off & 0xFF;
		wrbuf[2] = off >> 8;

		if (w1_reset_select_slave(sl))
				return -1;
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, buf, count);

		if (w1_reset_select_slave(sl))
				return -1;
		cmp = kzalloc(count, GFP_KERNEL);
		if (!cmp)
			return -ENOMEM;
		w1_write_block(sl->master, wrbuf, 3);
		w1_read_block(sl->master, cmp, count);
		if (!memcmp(cmp, buf, count)){
			kfree(cmp);
			return 0;
		}
	} while (--tries);

	dev_err(&sl->dev, "proof reading failed %d times\n",
			W1_MIKROBUS_ID_EEPROM_READ_RETRIES);
	kfree(cmp);
	return -EIO;
}

static int w1_mikrobus_id_readpage_secondary(struct w1_slave *sl, int pageaddr, char *buf)
{
	u8 crc_rdbuf[2];

	if (w1_reset_select_slave(sl))
				return -1;
	w1_write_8(sl->master, W1_MIKROBUS_ID_READ_SECONDARY_EEPROM);
	w1_write_8(sl->master, pageaddr);
	w1_read_block(sl->master, crc_rdbuf, 2);
	w1_write_8(sl->master, W1_MIKROBUS_ID_RELEASE_SECONDARY_EEPROM);
	msleep(10);
	w1_read_block(sl->master, crc_rdbuf, 1);
	w1_read_block(sl->master, buf, W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
	w1_read_block(sl->master, crc_rdbuf, 2);
	return 0;
}

static int w1_mikrobus_id_readbuf_secondary(struct w1_slave *sl, int count, char *buf)
{
	u8 pageaddr = 0;
	int iter, index, ret;
	int	len = count - (count % W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE);
	u8 temp_rdbuf[W1_MIKROBUS_ID_EEPROM_SECONDARY_PAGE_SIZE];

	while(len > 0) {			
			ret = w1_mikrobus_id_readpage_secondary(sl, pageaddr, buf + (W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE*pageaddr));
			pageaddr += 1;
			len -= W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE;
	}

	if(count % W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE){
			ret = w1_mikrobus_id_readpage_secondary(sl, pageaddr, temp_rdbuf);
			for(iter = W1_MIKROBUS_ID_EEPROM_SCRATCH_SIZE*pageaddr, index=0; iter < count; iter++, index++)
				buf[iter] = temp_rdbuf[index];
	}
	return ret;
}

static int w1_mikrobus_id_readblock_secondary(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 *cmp;
	int tries = W1_MIKROBUS_ID_EEPROM_READ_RETRIES;

	if(off == MIKROBUS_ID_USER_EEPROM_ADDR && count == 1) {
		buf[0] = 0;
		return 0;
	}

	do {	
		w1_mikrobus_id_readbuf_secondary(sl, count, buf);
		cmp = kzalloc(count, GFP_KERNEL);
		if (!cmp)
			return -ENOMEM;		
		w1_mikrobus_id_readbuf_secondary(sl, count, cmp);
		if (!memcmp(cmp, buf, count)){
			kfree(cmp);
			return 0;
		}
	} while (--tries);

	kfree(cmp);
	return -EINVAL;
}

static int w1_mikrobus_id_nvmem_read(void *priv, unsigned int off, void *buf, size_t count)
{
	struct w1_slave *sl = priv;
	int ret;

	mutex_lock(&sl->master->bus_mutex);
	if (sl->family->fid == W1_EEPROM_MIKROBUS_SECONDARY_ID)
		ret = w1_mikrobus_id_readblock_secondary(sl, off, count, buf);
	else
		ret = w1_mikrobus_id_readblock(sl, off, count, buf);
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
		.type = NVMEM_TYPE_EEPROM,
		.read_only = true,
		.word_size = 1,
		.stride = 1,
		.size = (sl->family->fid == W1_EEPROM_MIKROBUS_SECONDARY_ID) ? 
		 W1_MIKROBUS_ID_EEPROM_SECONDARY_SIZE: W1_MIKROBUS_ID_EEPROM_SIZE,
		.priv = sl,
	};

	port = mikrobus_find_port_by_w1_master(sl->master);
	if(!port)
		return -ENODEV;

	nvmem_cfg.name = port->name;
	nvmem = devm_nvmem_register(&sl->dev, &nvmem_cfg);
	port->eeprom = nvmem;
	mikrobus_port_scan_eeprom(port);

	return PTR_ERR_OR_ZERO(nvmem);
}

static struct w1_family_ops w1_family_mikrobus_id_fops = {
	.add_slave		= w1_mikrobus_id_add_slave,
};

static struct w1_family w1_family_mikrobus_id = {
	.fid = W1_EEPROM_MIKROBUS_ID,
	.fops = &w1_family_mikrobus_id_fops,
};

static struct w1_family w1_family_mikrobus_id_alternate = {
	.fid = W1_EEPROM_MIKROBUS_SECONDARY_ID,
	.fops = &w1_family_mikrobus_id_fops,
};

static int __init w1_mikrobusid_init(void)
{
	int err;

	err = w1_register_family(&w1_family_mikrobus_id);
	if (err)
		return err;

	err = w1_register_family(&w1_family_mikrobus_id_alternate);
	if (err)
		goto err_mikrobusidinit;


	return 0;

err_mikrobusidinit:
	w1_unregister_family(&w1_family_mikrobus_id);
	return err;
}

static void __exit w1_mikrobusid_exit(void)
{
	w1_unregister_family(&w1_family_mikrobus_id);
	w1_unregister_family(&w1_family_mikrobus_id_alternate);
}

module_init(w1_mikrobusid_init);
module_exit(w1_mikrobusid_exit);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("w1 family ac driver for mikroBUS ID EEPROM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("w1-family-" __stringify(W1_EEPROM_MIKROBUS_ID));
MODULE_ALIAS("w1-family-" __stringify(W1_EEPROM_MIKROBUS_SECONDARY_ID));
