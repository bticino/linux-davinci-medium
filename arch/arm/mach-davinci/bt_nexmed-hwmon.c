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
#include <linux/platform_device.h>
#include <linux/hwmon-sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/log2.h>

#include <mach/adc.h>

struct nexmed_hwmon {
	struct device *dev;
	struct platform_device *pdev;
	struct device *classdev;
};


#undef DEBUG

#define ADC_TEMP	0
#define ADC_HW_VERS	1
#define ADC_BOARD_L	2
#define ADC_BOARD_H	3
#define ADC_SPEED	4
#define ADC_LCD		5

static ssize_t show_name(struct device *dev, struct device_attribute
			*devattr, char *buf)
{
	return sprintf(buf, "Bticino Nextgne Medium Platform hwmon\n");
}

static const char *input_names[] = {
	[ADC_TEMP]    = "Temperature",
	[ADC_HW_VERS] = "Hw version",
	[ADC_BOARD_L]   = "board identification",
	[ADC_SPEED]   = "cpu speed",
	[ADC_LCD]    = "board configuration",
};

/* cpu frequencies */
int freqs[] = {270, 216, 300, 432, -1};

/* bticino boards */
char* boards[]={
	"BMX",
	"BASI",
	"DINGO",
	"OWL",
	"STORK",
	"JUMBO_D",
	"JUMBO_I",
	"ARGES",
	"LAGO",
	"AMICO_I",
};

/* calculating (2^(0,1*index))*100 */
int log_coeff[] = {
	100,
	107,
	115,
	123,
	132,
	141,
	152,
	162,
	174,
	187,
};

/* log2 first decimal digit approximation */
int log2_mul10(u32 dat)
{
	int log;
	int err1, err2, i;
	int app;

	log = ilog2(dat);
	app = 1 << log;

	for (i=0; i<10; i++)
	{
		err1 = dat * 100 - (app * log_coeff[i]);
		if (err1 < 0)
			break;
	}

	err2 = dat * 100 - (app * log_coeff[i-1]);
	if (err2 + err1 < 0)
		return(log * 10 + i);
	else
		return(log * 10 + i - 1);
}

/*
 * Tempperature approximation using a model
 * NTC epcos model B57331_V2223_J
 * B constant calculation formula
 *                            B: B Constant (K)
 *     lnR1–lnR2
 * B=  ---------------          T1: Arbitrary temperature (K)
 *    (1/T1)–(1/T2)
 *                            T2: Arbitrary temperature different from T1 (K)
 * R1: Zero-load resistance value at temperature T1(Ω)
 * R2: Zero-load resistance value at temperature T2(Ω)
 * Each temperature is measured in absolute temperature. 0°C=273.15K
 */
static ssize_t show_temp(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	u32 volatile ret;
	u32 ret1;
	u32 res;
	u32 temp;
	int channel = to_sensor_dev_attr(devattr)->index;
	struct davinci_adc_platform_data *adc_data= dev_get_drvdata(dev);

	ret = davinci_adc_read_oneshot(adc_data, channel);

	#ifdef DEBUG
		printk("ADC read = %d\n",ret);
	#endif
	ret = (ret * 1800) >> 10;
	#ifdef DEBUG
		printk("Calculating back %d mV\n", ret);
	#endif
	res = 10000 * ret / (1800 - ret);
	#ifdef DEBUG
		printk("Calculating back %d ohm\n", res);
	#endif
	ret = log2_mul10(res);
	#ifdef DEBUG
		printk("Calculating log2*10= %d\n", ret);
	#endif
	ret1 = 1341675 / ( ret * 207 / 10 + 1520) - 273;
	#ifdef DEBUG
		printk("Calculating temperature %d C\n", ret1);
	#endif

	return sprintf(buf, "%u\n", ret1);
}

/* thresholds using different resistors */
int lookup_resistors(int cnt)
{
	if (cnt > 948)
		return 9;
	if (cnt > 828)
		return 8;
	if (cnt > 720)
		return 7;
	if (cnt > 584)
		return 6;
	if (cnt > 440)
		return 5;
	if (cnt > 312)
		return 4;
	if (cnt > 216)
		return 3;
	if (cnt > 128)
		return 2;
	if (cnt > 44)
		return 1;
	return 0;
}

static ssize_t show_cfg(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int ret;
	int channel = to_sensor_dev_attr(devattr)->index;
	struct davinci_adc_platform_data *adc_data= dev_get_drvdata(dev);

	ret = davinci_adc_read_oneshot(adc_data, channel);
	return sprintf(buf, "%d\n", lookup_resistors(ret));
}

static ssize_t show_cfg1(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int ret;
	int channel = to_sensor_dev_attr(devattr)->index;
	struct davinci_adc_platform_data *adc_data= dev_get_drvdata(dev);


	ret = davinci_adc_read_oneshot(adc_data, channel);
	return sprintf(buf, "%dMHz\n", freqs[lookup_resistors(ret) % 5]);
}

static ssize_t show_cfg2(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	int ret, ret1;
	int channel = to_sensor_dev_attr(devattr)->index;
	struct davinci_adc_platform_data *adc_data= dev_get_drvdata(dev);

	ret = davinci_adc_read_oneshot(adc_data, channel);
	ret1 = davinci_adc_read_oneshot(adc_data, channel + 1);
	ret = lookup_resistors(ret);
	ret1 = lookup_resistors(ret1);
	ret += ret1 * 10;
	if (ret < ARRAY_SIZE(boards))
		return sprintf(buf, "%s BOARD\n", boards[ret]);
	else
		return sprintf(buf, "UNKNOWN BOARD\n");
	return sprintf(buf, "%d%d\n", lookup_resistors(ret1),
			lookup_resistors(ret));
}


static ssize_t show_label(struct device *dev, struct device_attribute
			*devattr, char *buf)
{
	int channel = to_sensor_dev_attr(devattr)->index;

	return sprintf(buf, "%s\n", input_names[channel]);
}

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, ADC_TEMP);
static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, show_label, NULL, ADC_TEMP);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_cfg, NULL, ADC_HW_VERS);
static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, show_label, NULL, ADC_HW_VERS);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_cfg2, NULL, ADC_BOARD_L);
static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, show_label, NULL, ADC_BOARD_L);
static SENSOR_DEVICE_ATTR(in4_input, S_IRUGO, show_cfg1, NULL, ADC_SPEED);
static SENSOR_DEVICE_ATTR(in4_label, S_IRUGO, show_label, NULL, ADC_SPEED);
static SENSOR_DEVICE_ATTR(in5_input, S_IRUGO, show_cfg, NULL, ADC_LCD);
static SENSOR_DEVICE_ATTR(in5_label, S_IRUGO, show_label, NULL, ADC_LCD);

static struct attribute *nexmed_hwmon_attr[] = {
	&dev_attr_name.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_label.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in5_label.dev_attr.attr,
	NULL,
};

static const struct attribute_group nexmed_hwmon_group = {
	.attrs = nexmed_hwmon_attr,
};

static int __init nexmed_hwmon_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev;
	struct davinci_adc_platform_data *adc_pl_data = pdev->dev.platform_data;

	/* Register sysfs hooks */
	ret = sysfs_create_group(&pdev->dev.kobj, &nexmed_hwmon_group);
	if (ret)
		return ret;

	dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		dev_err(&pdev->dev,
				"hwmon_device_register failed with %d.\n", ret);
		sysfs_remove_group(&pdev->dev.kobj, &nexmed_hwmon_group);
		return ret;
	}
	dev_set_drvdata(&pdev->dev, adc_pl_data);

	return 0;
}


static int __devexit nexmed_hwmon_remove(struct platform_device *pdev)
{
	struct nexmed_hwmon *nexmed_hwmon = platform_get_drvdata(pdev);

	hwmon_device_unregister(nexmed_hwmon->dev);

	sysfs_remove_group(&pdev->dev.kobj, &nexmed_hwmon_group);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


static struct platform_driver nexmed_hwmon_driver = {
	.probe		= nexmed_hwmon_probe,
	.remove         = __devexit_p(nexmed_hwmon_remove),
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "bt_nexmed_hwmon",
	},
};

static int __init nexmed_hwmon_init(void)
{
	return platform_driver_register(&nexmed_hwmon_driver);
}
module_init(nexmed_hwmon_init);

static void __exit nexmed_hwmon_exit(void)
{
	platform_driver_unregister(&nexmed_hwmon_driver);
}
module_exit(nexmed_hwmon_exit);

MODULE_DESCRIPTION("bt_nexmed_hwmon driver");
MODULE_AUTHOR("Davide Bonfanti");
MODULE_LICENSE("GPL");
