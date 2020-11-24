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

#define W1_EEPROM_MIKROBUS_ID	0xAC

#define W1_FAC_EEPROM_SIZE		512
#define W1_FAC_SCRATCH_SIZE		32

#define W1_FAC_READ_EEPROM		0xF0
#define W1_FAC_WRITE_SCRATCH	0x0F
#define W1_FAC_READ_SCRATCH		0xAA
#define W1_FAC_COPY_SCRATCH		0x55
#define W1_FAC_COPY_SCRATCH_ES	0x40

#define W1_FAC_TPROG_MS		15

#define W1_FAC_READ_RETRIES		10
#define W1_FAC_READ_MAXLEN		512

static int w1_fac_readblock(struct w1_slave *sl, int off, int count, char *buf)
{
	u8 wrbuf[1];
	u8 cmp[W1_FAC_READ_MAXLEN];
	int tries = W1_FAC_READ_RETRIES;

	do {
		wrbuf[0] = W1_FAC_READ_EEPROM;

		if (w1_reset_select_slave(sl))
			return -1;

		w1_write_block(sl->master, wrbuf, 1);
		w1_read_block(sl->master, buf, count);

		if (w1_reset_select_slave(sl))
			return -1;

		w1_write_block(sl->master, wrbuf, 1);
		w1_read_block(sl->master, cmp, count);

		if (!memcmp(cmp, buf, count))
			return 0;
	} while (--tries);

	dev_err(&sl->dev, "proof reading failed %d times\n",
			W1_FAC_READ_RETRIES);

	return -1;
}

static ssize_t eeprom_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr, char *buf,
			   loff_t off, size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	
	if(count > W1_FAC_READ_MAXLEN)
		return -EINVAL;

	mutex_lock(&sl->master->bus_mutex);
	if (w1_fac_readblock(sl, off, count, buf) < 0)
		count = -EIO;
	mutex_unlock(&sl->master->bus_mutex);

	return count;
}

static int w1_fac_writescratchpad(struct w1_slave *sl, int addr, char *buf)
{
	u8 wrbuf[3];

	wrbuf[0] = W1_FAC_WRITE_SCRATCH;
	wrbuf[1] = addr >> 8;
	wrbuf[2] = addr & 0xFF;

	if (w1_reset_select_slave(sl))
		return -1;

	w1_write_block(sl->master, wrbuf, 3);
	w1_write_block(sl->master, buf, W1_FAC_SCRATCH_SIZE);

	return W1_FAC_SCRATCH_SIZE;
}

static int w1_fac_verifyscratchpad(struct w1_slave *sl, char *buf)
{
	u8 wrbuf[1];
	u8 cmp[W1_FAC_SCRATCH_SIZE + 2];

	wrbuf[0] = W1_FAC_READ_SCRATCH;

	if (w1_reset_select_slave(sl))
		return -1;

	w1_write_block(sl->master, wrbuf, 1);
	w1_read_block(sl->master, cmp, W1_FAC_SCRATCH_SIZE);

	if (!memcmp(cmp + 2, buf, W1_FAC_SCRATCH_SIZE))
		return 0;

	return -1;
}

static int w1_fac_copyscratchpad(struct w1_slave *sl, int addr)
{
	u8 wrbuf[4];

	wrbuf[0] = W1_FAC_COPY_SCRATCH;
	wrbuf[1] = addr >> 8;
	wrbuf[2] = addr & 0xFF;
	wrbuf[3] = W1_FAC_COPY_SCRATCH_ES;

	if (w1_reset_select_slave(sl))
		return -1;

	w1_write_block(sl->master, wrbuf, 4);

	/* Sleep for tprog ms to wait for the write to complete */
	msleep(W1_FAC_TPROG_MS);

	return 0;
}

static ssize_t eeprom_write(struct file *filp, struct kobject *kobj,
			    struct bin_attribute *bin_attr, char *buf,
			    loff_t off, size_t count)
{
	struct w1_slave *sl = kobj_to_w1_slave(kobj);
	u8 wrbuf[W1_FAC_EEPROM_SIZE];
	int addr, len;

	if(count > W1_FAC_EEPROM_SIZE)
		return -EINVAL;

	memcpy(wrbuf, buf, count);
	mutex_lock(&sl->master->bus_mutex);

	/* Can only write data in blocks of the size of the scratchpad */
	addr = 0;
	len = 512;
	while (len > 0) {
		if (w1_fac_writescratchpad(sl, addr, wrbuf) < 0) {
			count = -EIO;
			goto out_up;
		}
		if (w1_fac_verifyscratchpad(sl, wrbuf) < 0) {
			// count = -EIO;
			// goto out_up;
		}
		if (w1_fac_copyscratchpad(sl, addr) < 0) {
			// count = -EIO;
			// goto out_up;
		}
		wrbuf += W1_FAC_SCRATCH_SIZE;
		addr += W1_FAC_SCRATCH_SIZE;
		len -= W1_FAC_SCRATCH_SIZE;
	}

out_up:
	mutex_unlock(&sl->master->bus_mutex);

	return count;
}

static BIN_ATTR_RW(eeprom, W1_FAC_EEPROM_SIZE);

static struct bin_attribute *w1_fac_bin_attrs[] = {
	&bin_attr_eeprom,
	NULL,
};

static const struct attribute_group w1_fac_group = {
	.bin_attrs = w1_fac_bin_attrs,
};

static const struct attribute_group *w1_fac_groups[] = {
	&w1_fac_group,
	NULL,
};

static const struct w1_family_ops w1_fac_fops = {
	.groups		= w1_fac_groups,
};

static struct w1_family w1_family_ac = {
	.fid = W1_EEPROM_MIKROBUS_ID,
	.fops = &w1_fac_fops,
};
module_w1_family(w1_family_ac);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("w1 family ac driver for mikroBUS ID EEPROM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("w1-family-" __stringify(W1_EEPROM_MIKROBUS_ID));