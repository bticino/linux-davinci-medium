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


#define LCD_REFRESH_TIME 5000

struct tm035kbh02 lcd = {
	.is_suspended = 0,
	.disabled = 1,
	.spi = NULL,
	.model = -1,
	.reset_n = -1,
	.shutdown = -1,
	.HV_inversion = 0,
};

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
	{0x0D, 0x20 },
	{0x0E, 0x10 },
	{0x0F, 0x24 },
	{0x10, 0x04 },
	{0x11, 0x24 },
	{0x12, 0x24 },
	{0x00, 0x03 },
};

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
		}
		for (i = 0; i < len; i++) {
			loc_buf[1] = (pnt[i][0]<<2) | 0x03 ;
			loc_buf[0] = pnt[i][1];
			spi_write(lcd.spi, &loc_buf[0], 2);
		}
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

int  tm035kbh02_set_contrast(struct lcd_device *tm035kbh02_lcd_device,
			     int contrast)
{
	u8 loc_buf[2];

	if (lcd.disabled || !lcd.spi)
		return 0;
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
			jiffies + msecs_to_jiffies(lcd.time_refr_regs * 1000);
			add_timer(&lcd_refresh_timer);
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

struct lcd_device *tm035kbh02_lcd_device;

static int __devinit tm035kbh02_spi_probe(struct spi_device *spi)
{
	struct tm035kbh02_platform_data *pdata = spi->dev.platform_data;
	int	err;

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
	tm035kbh02_lcd_device->props.max_contrast = 0x1F;
	tm035kbh02_lcd_device->props.max_brightness = 0x7F;

	INIT_WORK(&lcd.work, tm035kbh02_refr);
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
