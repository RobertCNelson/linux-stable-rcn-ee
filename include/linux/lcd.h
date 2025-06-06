/* SPDX-License-Identifier: GPL-2.0 */
/*
 * LCD Lowlevel Control Abstraction
 *
 * Copyright (C) 2003,2004 Hewlett-Packard Company
 *
 */

#ifndef _LINUX_LCD_H
#define _LINUX_LCD_H

#include <linux/device.h>
#include <linux/mutex.h>

#define LCD_POWER_ON			(0)
#define LCD_POWER_REDUCED		(1) // deprecated; don't use in new code
#define LCD_POWER_REDUCED_VSYNC_SUSPEND	(2) // deprecated; don't use in new code
#define LCD_POWER_OFF			(4)

/* Notes on locking:
 *
 * lcd_device->ops_lock is an internal backlight lock protecting the ops
 * field and no code outside the core should need to touch it.
 *
 * Access to set_power() is serialised by the update_lock mutex since
 * most drivers seem to need this and historically get it wrong.
 *
 * Most drivers don't need locking on their get_power() method.
 * If yours does, you need to implement it in the driver. You can use the
 * update_lock mutex if appropriate.
 *
 * Any other use of the locks below is probably wrong.
 */

struct lcd_device;

struct lcd_properties {
	/* The maximum value for contrast (read-only) */
	int max_contrast;
};

struct lcd_ops {
	/* Get the LCD panel power status (0: full on, 1..3: controller
	   power on, flat panel power off, 4: full off), see FB_BLANK_XXX */
	int (*get_power)(struct lcd_device *);
	/* Enable or disable power to the LCD (0: on; 4: off, see FB_BLANK_XXX) */
	int (*set_power)(struct lcd_device *, int power);
	/* Get the current contrast setting (0-max_contrast) */
	int (*get_contrast)(struct lcd_device *);
	/* Set LCD panel contrast */
        int (*set_contrast)(struct lcd_device *, int contrast);

	/*
	 * Set LCD panel mode (resolutions ...)
	 */
	int (*set_mode)(struct lcd_device *lcd, u32 xres, u32 yres);

	/*
	 * Check if the LCD controls the given display device. This
	 * operation is optional and if not implemented it is assumed that
	 * the display is always the one controlled by the LCD.
	 *
	 * RETURNS:
	 *
	 * If display_dev is NULL or display_dev matches the device controlled by
	 * the LCD, return true. Otherwise return false.
	 */
	bool (*controls_device)(struct lcd_device *lcd, struct device *display_device);
};

struct lcd_device {
	struct lcd_properties props;
	/* This protects the 'ops' field. If 'ops' is NULL, the driver that
	   registered this device has been unloaded, and if class_get_devdata()
	   points to something in the body of that driver, it is also invalid. */
	struct mutex ops_lock;
	/* If this is NULL, the backing module is unloaded */
	const struct lcd_ops *ops;
	/* Serialise access to set_power method */
	struct mutex update_lock;

	/**
	 * @entry: List entry of all registered lcd devices
	 */
	struct list_head entry;

	struct device dev;
};

struct lcd_platform_data {
	/* reset lcd panel device. */
	int (*reset)(struct lcd_device *ld);
	/* on or off to lcd panel. if 'enable' is 0 then
	   lcd power off and 1, lcd power on. */
	int (*power_on)(struct lcd_device *ld, int enable);

	/* it indicates whether lcd panel was enabled
	   from bootloader or not. */
	int lcd_enabled;
	/* it means delay for stable time when it becomes low to high
	   or high to low that is dependent on whether reset gpio is
	   low active or high active. */
	unsigned int reset_delay;
	/* stable time needing to become lcd power on. */
	unsigned int power_on_delay;
	/* stable time needing to become lcd power off. */
	unsigned int power_off_delay;

	/* it could be used for any purpose. */
	void *pdata;
};

static inline void lcd_set_power(struct lcd_device *ld, int power)
{
	mutex_lock(&ld->update_lock);
	if (ld->ops && ld->ops->set_power)
		ld->ops->set_power(ld, power);
	mutex_unlock(&ld->update_lock);
}

extern struct lcd_device *lcd_device_register(const char *name,
	struct device *parent, void *devdata, const struct lcd_ops *ops);
extern struct lcd_device *devm_lcd_device_register(struct device *dev,
	const char *name, struct device *parent,
	void *devdata, const struct lcd_ops *ops);
extern void lcd_device_unregister(struct lcd_device *ld);
extern void devm_lcd_device_unregister(struct device *dev,
	struct lcd_device *ld);

#if IS_REACHABLE(CONFIG_LCD_CLASS_DEVICE)
void lcd_notify_blank_all(struct device *display_dev, int power);
void lcd_notify_mode_change_all(struct device *display_dev,
				unsigned int width, unsigned int height);
#else
static inline void lcd_notify_blank_all(struct device *display_dev, int power)
{}

static inline void lcd_notify_mode_change_all(struct device *display_dev,
					      unsigned int width, unsigned int height)
{}
#endif

#define to_lcd_device(obj) container_of(obj, struct lcd_device, dev)

static inline void * lcd_get_data(struct lcd_device *ld_dev)
{
	return dev_get_drvdata(&ld_dev->dev);
}


#endif
