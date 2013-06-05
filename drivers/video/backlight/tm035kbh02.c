/*
 *
 * Copyright (C) 2010 Bticino S.p.a
 * Author: Davide Bonfanti <davide.bonfanti@bticino.it>
 *
 * Contributors:
 *     Raffaele Recalcati <raffaele.recalcati@bticino.it>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/vmalloc.h>
#include <linux/spi/spi.h>
#include <linux/tm035kbh02.h>
#include <linux/regulator/consumer.h>
#include <linux/lcd.h>
#include <linux/platform_device.h>
#include <asm/irq.h>

#define MAX_CONTR	0x1F
#define MAX_BRIGHT	0x7F
#define LCD_REFRESH_TIME 5000

struct tm035kbh02 lcd = {
	.is_suspended = 0,
	.disabled = 1,
	.spi = NULL,
	.model = -1,
	.reset_n = -1,
	.shutdown = -1,
	.HV_inversion = 0,
	.b_subcontrast = 2,
	.b_subcontr_min = 0,
	.b_subcontr_max = 3,
	.r_subcontrast = 2,
	.r_subcontr_min = 0,
	.r_subcontr_max = 3,
	.b_subbright = 0x20,
	.b_subbright_min = 0,
	.b_subbright_max = 0x3F,
	.r_subbright = 0x20,
	.r_subbright_min = 0,
	.r_subbright_max = 0x3F,
};

struct lcd_device *tm035kbh02_lcd_device;

static struct timer_list lcd_refresh_timer;

const u8 regs_tm035kbh02[][2] = {
/* Register address - Register Value - Register LSB Value */
	{0x01, 0x00 },
	{0x02, 0x03 },
	{0x03, 0xCC },
	{0x04, 0x46 },
	{0x05, 0x0D },
	{0x06, 0x00 },
	{0x07, 0x00 },
	{0x0A, 0x88 },
	{0x0E, 0x10 },
	{0x0F, 0x24 },
	{0x10, 0x04 },
	{0x11, 0x24 },
	{0x12, 0x24 },
	{0x00, 0x03 },
};

int  tm035kbh02_set_contrast(struct lcd_device *tm035kbh02_lcd_device,
			     int contrast)
{
	u8 loc_buf[2];

	if (lcd.disabled || !lcd.spi)
		return 0;
	if (contrast > MAX_CONTR || contrast < 0)
		return -EINVAL;
	loc_buf[0] = contrast;
	loc_buf[1] = 0x23;
	spi_write(lcd.spi, &loc_buf[0], 2);
	lcd.contrast = contrast;
	return 0;
}

int  tm035kbh02_get_contrast(struct lcd_device *tm035kbh02_lcd_device)
{
	return lcd.contrast;
}

int  tm035kbh02_set_brightness(struct lcd_device *tm035kbh02_lcd_device,
			       int brightness)
{
	u8 loc_buf[2];

	if (lcd.disabled || !lcd.spi)
		return 0;
	if (brightness > MAX_BRIGHT || brightness < 0)
		return -EINVAL;
	loc_buf[0] = brightness;
	loc_buf[1] = 0x27;
	spi_write(lcd.spi, &loc_buf[0], 2);
	lcd.brightness = brightness;
	return 0;
}

int  tm035kbh02_get_brightness(struct lcd_device *tm035kbh02_lcd_device)
{
	return lcd.brightness;
}

static int set_b_subcontrast(struct tm035kbh02 *lcd, unsigned long val)
{
	u8 loc_buf[2];

	if (lcd->disabled || !lcd->spi)
		return 0;

	loc_buf[0] = (val<<6) + ((lcd->r_subcontrast)<<2);
	loc_buf[1] = (0xb<<2) + 3;
	spi_write(lcd->spi, &loc_buf[0], 2);

	lcd->b_subcontrast = val;
	return 0;
}

static int set_r_subcontrast(struct tm035kbh02 *lcd, unsigned long val)
{
	u8 loc_buf[2];

	if (lcd->disabled || !lcd->spi)
		return 0;

	loc_buf[0] = (val<<2) + ((lcd->b_subcontrast)<<6);
	loc_buf[1] = (0xb<<2) + 3;
	spi_write(lcd->spi, &loc_buf[0], 2);

	lcd->r_subcontrast = val;
	return 0;
}

static int set_b_subbright(struct tm035kbh02 *lcd, unsigned long val)
{
	u8 loc_buf[2];

	if (lcd->disabled || !lcd->spi)
		return 0;

	loc_buf[0] = val;
	loc_buf[1] = (0xd<<2) + 3;
	spi_write(lcd->spi, &loc_buf[0], 2);

	lcd->b_subbright = val;
	return 0;
}

static int set_r_subbright(struct tm035kbh02 *lcd, unsigned long val)
{
	u8 loc_buf[2];

	if (lcd->disabled || !lcd->spi)
		return 0;

	loc_buf[0] = val;
	loc_buf[1] = (0xd<<2) + 3;
	spi_write(lcd->spi, &loc_buf[0], 2);

	lcd->r_subbright = val;
	return 0;
}


void tm035kbh02_disable(void)
{
	u8 loc_buf[2];
	if (lcd.disabled || !lcd.spi)
		return;
	loc_buf[0] = 0x01;
	loc_buf[1] = 0x03;
	spi_write(lcd.spi, &loc_buf[0], 2);

	mdelay(5);

	gpio_set_value(lcd.reset_n, 0);
	gpio_set_value(lcd.shutdown, 1);
	lcd.disabled = 1;
}

void tm035kbh02_enable(void)
{
	int i, len;
	u8 loc_buf[2];
	const u8 (*pnt)[2];

	if (lcd.model != TM035KBH02)
		return;
	pnt = regs_tm035kbh02;
	len = ARRAY_SIZE(regs_tm035kbh02);

	/* write data to LCD controller */
	if (lcd.spi) {
		/* Power-on sequence */
		if (lcd.disabled) {
			gpio_set_value(lcd.reset_n, 0);
			mdelay(1);
			gpio_set_value(lcd.reset_n, 1);
			mdelay(40);
			schedule();
		}
		for (i = 0; i < len; i++) {
			loc_buf[1] = (pnt[i][0]<<2) | 0x03 ;
			loc_buf[0] = pnt[i][1];
			spi_write(lcd.spi, &loc_buf[0], 2);
		}
		tm035kbh02_set_contrast(tm035kbh02_lcd_device, lcd.contrast);
		tm035kbh02_set_brightness(tm035kbh02_lcd_device,
					  lcd.brightness);
		set_b_subcontrast(&lcd, lcd.b_subcontrast);
		set_r_subcontrast(&lcd, lcd.r_subcontrast);
		set_b_subbright(&lcd, lcd.b_subbright);
		set_r_subbright(&lcd, lcd.r_subbright);
		lcd.disabled = 0;
	}
}

static void tm035kbh02_refr(struct work_struct *work)
{
	tm035kbh02_enable();
}

static int tm035kbh02_suspend(struct spi_device *spi, pm_message_t message)
{
	lcd.is_suspended = 1;
	tm035kbh02_disable();

	return 0;
}

static int tm035kbh02_resume(struct spi_device *spi)
{
	lcd.is_suspended = 0;
	tm035kbh02_enable();

	return 0;
}

static int tm035kbh02_power_changed(struct spi_device *spi,
					enum sys_power_state s)
{
	switch (s) {
	case SYS_PWR_GOOD:
		lcd.is_suspended = 0;
		tm035kbh02_enable();
		break;
	case SYS_PWR_FAILING:
		lcd.is_suspended = 1;
		tm035kbh02_disable();
		break;
	default:
		BUG();
	}

	return 0;
}

static void refresh_lcd_settings(unsigned long data)
{
	schedule_work(&lcd.work);
	lcd_refresh_timer.expires =
			jiffies + msecs_to_jiffies(lcd.time_refr_regs * 1000);
	add_timer(&lcd_refresh_timer);
}

int  tm035kbh02_set_power(struct lcd_device *tm035kbh02_lcd_device, int power)
{
	if (!power) {
		lcd.is_suspended = 0;
		tm035kbh02_enable();
		if (lcd.time_refr_regs) {
			if (timer_pending(&lcd_refresh_timer))
				return 0;
			init_timer(&lcd_refresh_timer);
			lcd_refresh_timer.function = refresh_lcd_settings;
			lcd_refresh_timer.expires =
				jiffies +
				msecs_to_jiffies(lcd.time_refr_regs * 1000);
			add_timer(&lcd_refresh_timer);
			tm035kbh02_set_contrast(tm035kbh02_lcd_device,
				lcd.contrast);
			tm035kbh02_set_brightness(tm035kbh02_lcd_device,
				lcd.brightness);
		}
	} else if (power == 4) {
		lcd.is_suspended = 1;
		tm035kbh02_disable();
		del_timer(&lcd_refresh_timer);
	}
	return 0;
}

int  tm035kbh02_get_power(struct lcd_device *tm035kbh02_lcd_device)
{
	if (lcd.is_suspended == 1)
		return 4;
	return 0;
}

struct lcd_ops tm035kbh02_lcd_ops = {
	.get_power = tm035kbh02_get_power,
	.set_power = tm035kbh02_set_power,
	.get_contrast = tm035kbh02_get_contrast,
	.set_contrast = tm035kbh02_set_contrast,
	.get_brightness = tm035kbh02_get_brightness,
	.set_brightness = tm035kbh02_set_brightness,
};

#define get_field(field) \
static ssize_t tm035kbh02_get_##field(struct device *dev, \
				struct device_attribute *attr, char *buf) \
{ \
	struct tm035kbh02 *lcd = \
			(struct tm035kbh02 *)dev_get_drvdata(dev); \
	return sprintf(buf, "%d\n", lcd->field); \
}

get_field(b_subcontrast)
get_field(r_subcontrast)
get_field(b_subbright)
get_field(r_subbright)
get_field(b_subcontr_min)
get_field(b_subcontr_max)
get_field(r_subcontr_min)
get_field(r_subcontr_max)
get_field(b_subbright_min)
get_field(b_subbright_max)
get_field(r_subbright_min)
get_field(r_subbright_max)

static ssize_t tm035kbh02_set_b_subcontrast(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct tm035kbh02 *lcd =
			(struct tm035kbh02 *)dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val > lcd->b_subcontr_max) || (val < lcd->b_subcontr_min))
		return -EINVAL;
	if (set_b_subcontrast(lcd, val))
		return -EINVAL;
	return count;
}

static ssize_t tm035kbh02_set_r_subcontrast(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct tm035kbh02 *lcd =
			(struct tm035kbh02 *)dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val > lcd->r_subcontr_max) || (val < lcd->r_subcontr_min))
		return -EINVAL;
	if (set_r_subcontrast(lcd, val))
		return -EINVAL;
	return count;
}

static ssize_t tm035kbh02_set_b_subbright(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct tm035kbh02 *lcd =
			(struct tm035kbh02 *)dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val > lcd->b_subbright_max) || (val < lcd->b_subbright_min))
		return -EINVAL;
	if (set_b_subbright(lcd, val))
		return -EINVAL;
	return count;
}

static ssize_t tm035kbh02_set_r_subbright(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct tm035kbh02 *lcd =
			(struct tm035kbh02 *)dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	if ((val > lcd->r_subbright_max) || (val < lcd->r_subbright_min))
		return -EINVAL;
	if (set_r_subbright(lcd, val))
		return -EINVAL;
	return count;
}

static DEVICE_ATTR(b_subcontrast, S_IRUGO | S_IWUSR,
		   tm035kbh02_get_b_subcontrast,
		   tm035kbh02_set_b_subcontrast);

static DEVICE_ATTR(r_subcontrast, S_IRUGO | S_IWUSR,
		   tm035kbh02_get_r_subcontrast,
		   tm035kbh02_set_r_subcontrast);

static DEVICE_ATTR(b_subbright, S_IRUGO | S_IWUSR,
		   tm035kbh02_get_b_subbright,
		   tm035kbh02_set_b_subbright);

static DEVICE_ATTR(r_subbright, S_IRUGO | S_IWUSR,
		   tm035kbh02_get_r_subbright,
		   tm035kbh02_set_r_subbright);

static DEVICE_ATTR(b_subcontr_min, S_IRUGO,
		   tm035kbh02_get_b_subcontr_min,
		   NULL);

static DEVICE_ATTR(b_subcontr_max, S_IRUGO,
		   tm035kbh02_get_b_subcontr_max,
		   NULL);

static DEVICE_ATTR(r_subcontr_min, S_IRUGO,
		   tm035kbh02_get_r_subcontr_min,
		   NULL);

static DEVICE_ATTR(r_subcontr_max, S_IRUGO,
		   tm035kbh02_get_r_subcontr_max,
		   NULL);

static DEVICE_ATTR(b_subbright_min, S_IRUGO,
		   tm035kbh02_get_b_subbright_min,
		   NULL);

static DEVICE_ATTR(b_subbright_max, S_IRUGO,
		   tm035kbh02_get_b_subbright_max,
		   NULL);

static DEVICE_ATTR(r_subbright_min, S_IRUGO,
		   tm035kbh02_get_r_subbright_min,
		   NULL);

static DEVICE_ATTR(r_subbright_max, S_IRUGO,
		   tm035kbh02_get_r_subbright_max,
		   NULL);

static struct attribute *tm035kbh02_attributes[] = {
	&dev_attr_b_subcontrast.attr,
	&dev_attr_r_subcontrast.attr,
	&dev_attr_b_subbright.attr,
	&dev_attr_r_subbright.attr,
	&dev_attr_b_subcontr_min.attr,
	&dev_attr_b_subcontr_max.attr,
	&dev_attr_r_subcontr_min.attr,
	&dev_attr_r_subcontr_max.attr,
	&dev_attr_b_subbright_min.attr,
	&dev_attr_b_subbright_max.attr,
	&dev_attr_r_subbright_min.attr,
	&dev_attr_r_subbright_max.attr,
	NULL
};

static const struct attribute_group tm035kbh02_attr_group = {
	.attrs = tm035kbh02_attributes,
};

static int __devinit tm035kbh02_spi_probe(struct spi_device *spi)
{
	struct tm035kbh02_platform_data *pdata = spi->dev.platform_data;
	int	err;
	struct class *cl;
	struct device *dev;

	if (!pdata) {
		/* if we don't know reset & shutdown GPIO can't drive LCD */
		dev_dbg(&spi->dev, "no platform data?\n");
		return -EINVAL;
	}

	if (pdata->check_presence) {
		if (!gpio_is_valid(pdata->lcd_present)) {
			dev_err(&spi->dev, "lcd_present gpio is not valid\n");
			return -EINVAL;
		}
		if (!gpio_get_value(pdata->lcd_present)) {
			dev_err(&spi->dev,
				"Attention: DISPLAY NOT PRESENT!!!\n");
			return -ENODEV;
		}
	}

	/* don't exceed max specified sample rate of 3MHz */
	if (spi->max_speed_hz > (3000000)) {
		dev_dbg(&spi->dev, "speed too high %d\n",
				spi->max_speed_hz);
		spi->max_speed_hz = 20000000;
	}
	spi->bits_per_word = 16;
	spi->mode = SPI_MODE_0;
	/* SPI_LSB_FIRST: this opt should used but DM365 works fine without */

	err = spi_setup(spi);
	if (err < 0)
		return err;

	lcd.spi = spi;
	lcd.reset_n = pdata->reset_n;
	lcd.shutdown = pdata->shutdown;
	lcd.model = pdata->model;
	lcd.HV_inversion = pdata->HV_inversion;
	lcd.time_refr_regs = pdata->time_refr_regs;
	lcd.contrast = 8;
	lcd.brightness = 0x40;

	tm035kbh02_lcd_device = lcd_device_register("tm035kbh02",
				&spi->dev, pdata, &tm035kbh02_lcd_ops);

	if (IS_ERR(tm035kbh02_lcd_device)) {
		err = PTR_ERR(tm035kbh02_lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return err;
	}
	tm035kbh02_lcd_device->props.max_contrast = MAX_CONTR;
	tm035kbh02_lcd_device->props.max_brightness = MAX_BRIGHT;

	INIT_WORK(&lcd.work, tm035kbh02_refr);


	cl = class_create(THIS_MODULE, "lcd_parameters");
	dev = device_create(cl, NULL, spi->dev.devt,
			NULL, "tm035kbh02");
	dev->platform_data = spi;
	sysfs_create_group(&dev->kobj, &tm035kbh02_attr_group);
	dev_set_drvdata(dev, &lcd);

	tm035kbh02_enable();

	return 0;
}

static int __devexit tm035kbh02_spi_remove(struct spi_device *spi)
{
	dev_dbg(&spi->dev, "unregistering Display\n");
	return 0;
}

static struct spi_driver tm035kbh02_driver = {
	.driver = {
		.name	= "tm035kbh02",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= tm035kbh02_spi_probe,
	.remove		= __devexit_p(tm035kbh02_spi_remove),
	.suspend	= tm035kbh02_suspend,
	.resume		= tm035kbh02_resume,
#ifdef CONFIG_PM_LOSS
	.power_changed	= tm035kbh02_power_changed,
#endif
};

static int __init tm035kbh02_init(void)
{
	return spi_register_driver(&tm035kbh02_driver);
}
module_init(tm035kbh02_init);

static void __exit tm035kbh02_exit(void)
{
	spi_unregister_driver(&tm035kbh02_driver);
}
module_exit(tm035kbh02_exit);

MODULE_DESCRIPTION("TIANMA tm035kbh02 LCD driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:tm035kbh02");
MODULE_AUTHOR("Davide Bonfanti <davide.bonfanti@bticino.it>");
