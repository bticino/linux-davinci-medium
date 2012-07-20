/*
 * mc44cc373- THS7303 Video Amplifier driver
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>

#include <media/davinci/videohd.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#include <linux/i2c/mc44cc373.h>
#include <linux/gpio.h>
#include "mc44cc373.h"
#include <media/v4l2-chip-ident.h>

#define DRIVER_VERSION  "1.0.0"

MODULE_DESCRIPTION("mc44cc373 Video Modulator Driver");
MODULE_AUTHOR("Davide Bonfanti, Simone Cianni");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-1");

struct mc44cc373 {
        struct v4l2_subdev sd;
        const struct mc44cc373_platform_data *pdata;
        int status;
};

static struct mc44cc373 *mc44cc373_data;

static inline struct mc44cc373 *to_state(struct v4l2_subdev *sd)
{
        return container_of(sd, struct mc44cc373, sd);
}

/* following function is used to set mc44cc373 */
static int mc44cc373_setvalue(struct v4l2_subdev *sd)
{

        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct mc44cc373 *t = to_state(sd);
        int ret = 0;
        u8 *buf = t->pdata->pars ;
        int len = t->pdata->num_par;

        //int i ;
        //for (i=0; i < t->pdata->num_par; i++)
        //        printk("\n Par_ %d=%x,%x \n", i, t->pdata->pars[i],buf[i]);

	//printk("%s: len=%d \n", __func__, len);

        ret = i2c_master_send(client, buf, len);
	if (ret != len) {
		v4l2_err(sd, "mc44cc373 write failed\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL(mc44cc373_setvalue);

static int mc44cc373_s_power(struct v4l2_subdev *sd, int on)
{
        struct mc44cc373 *t = to_state(sd);

        v4l2_dbg(1, debug, sd, "Switching modulator %s \n", on ? "on" : "off");

	gpio_set_value(t->pdata->power, on);

	if (on){
		mdelay(20);
		return mc44cc373_setvalue(sd);
	}
	return 0;
}

static int mc44cc373_s_std_output(struct v4l2_subdev *sd, v4l2_std_id std)
{
	return mc44cc373_s_power(sd,1);
}

int mc44cc373_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable == 1)
		v4l2_dbg(0, debug, sd, "Power Up Sequence \n");
	else
		v4l2_dbg(0, debug, sd, "Power Down Sequence \n");

	return mc44cc373_s_power(sd, enable);
}

static const struct v4l2_subdev_video_ops mc44cc373_video_ops = {
	.s_stream	= mc44cc373_s_stream,
	.s_std_output	= mc44cc373_s_std_output,
};

static const struct v4l2_subdev_core_ops mc44cc373_core_ops = {
	.s_power = mc44cc373_s_power,
};

static const struct v4l2_subdev_ops mc44cc373_ops = {
	.core	= &mc44cc373_core_ops,
	.video	= &mc44cc373_video_ops,
};
/*----------------------------------------------------------------------------*/

/*******************************************************************************
		Uso sysfs per mc44cc373

echo cat  /sys/class/mc44cc373/setting/enable
echo "on" >  /sys/class/mc44cc373/setting/enable
echo "off" >  /sys/class/mc44cc373/setting/enable

*******************************************************************************/

struct mc44cc373_device {
        struct module *owner;
        struct device class_dev;
	int status ;
};

static struct mc44cc373_device *davinci_mc44cc373_device;

#define to_mc44cc373_dev(cdev)	container_of(cdev, struct mc44cc373_device, \
				 class_dev)

static ssize_t enable_show(struct device *cdev, struct device_attribute *attr,
				 char *buf)
{
        char name[]= "mc44cc373_status: " ;
        struct mc44cc373_device *dev = to_mc44cc373_dev(cdev);
        int p;

	//printk("\n\n %s %d stato : %d \n\n", __func__, __LINE__, dev->status );

        p = sprintf(buf, name);
	if (dev->status)
		p += sprintf(buf + p, "on \n");
	else
		p += sprintf(buf + p, "off \n");

        return p;

}

static ssize_t enable_store(struct device *cdev, struct device_attribute *attr,
				const char *buf,size_t count)
{
        struct mc44cc373_device *dev = to_mc44cc373_dev(cdev);
        int ret;
	struct v4l2_subdev *sd;

        sd = &mc44cc373_data->sd;

        if (!buf || (count == 0))
                return 0;

        if (strncmp(buf, "on", 2) == 0)
                 dev->status = 1;
        else if (strncmp(buf, "off", 3) == 0)
                 dev->status = 0;
        else
                return -EINVAL;

        ret = mc44cc373_s_power(sd, dev->status);
        if (ret < 0)
                return ret;

        return count;
}

#define DECLARE_ATTR(_name, _mode, _show, _store)               \
{                                                               \
	.attr   = { .name = __stringify(_name), .mode = _mode,	\
		    .owner = THIS_MODULE },			\
	.show   = _show,                                        \
	.store  = _store,                                       \
}
static struct device_attribute mc44cc373_device_attributes[] = {
	DECLARE_ATTR(enable, S_IRWXUGO, enable_show, enable_store),
	DECLARE_ATTR(disable, S_IRWXUGO, NULL, NULL),
};

static void mc44cc373_class_release(struct device *cdev)
{
        struct mc44cc373_device *dev = to_mc44cc373_dev(cdev);

        if (dev != NULL)
                kfree(dev);
}

struct class mc44cc373_class = {
        .name = "mc44cc373",
        .owner  = THIS_MODULE,
        .dev_release = mc44cc373_class_release,
};

static void *create_sysfs_files(void)
{
	struct mc44cc373_device *dev;
	int ret;
	int i;

	dev = kzalloc(sizeof(struct mc44cc373_device), GFP_KERNEL);
	if (!dev)
		return NULL;

        dev->owner = THIS_MODULE;
        dev->class_dev.class = &mc44cc373_class;
        dev_set_name(&dev->class_dev, "setting");
        dev_set_drvdata(&dev->class_dev, dev);
        ret = device_register(&dev->class_dev);
        if (ret < 0) {
                printk(KERN_ERR "mc44cc373 : Error in device class register\n");
                kfree(dev);
                return NULL;
        }

	for (i = 0; i < ARRAY_SIZE(mc44cc373_device_attributes); i++) {
		ret = device_create_file(&dev->class_dev,
					       &mc44cc373_device_attributes[i]);
		if (ret < 0) {
			while (--i >= 0)
				device_remove_file(&dev->class_dev,
					 &mc44cc373_device_attributes[i]);
			dev_set_drvdata(&dev->class_dev, NULL);
			device_unregister(&dev->class_dev);
			return NULL;
		}
	}

	return dev;
}

static void remove_sysfs_files(struct mc44cc373_device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mc44cc373_device_attributes); i++)
		device_remove_file(&dev->class_dev,
					 &mc44cc373_device_attributes[i]);

	dev_set_drvdata(&dev->class_dev, NULL);
	device_unregister(&dev->class_dev);
}

/*----------------------------------------------------------------------------*/

/*******************************************************************************
			 i2c interface functions
*******************************************************************************/

//static struct mc44cc373 *mc44cc373_data;
static int mc44cc373_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;

	/* debug = 1; */

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

        if (!client->dev.platform_data) {
                v4l2_err(client, "No platform data!!\n");
                return -ENODEV;
        }

        mc44cc373_data = kzalloc(sizeof(struct  mc44cc373), GFP_KERNEL);
        if (!mc44cc373_data)
                return -ENOMEM;

	/* Copy board specific information here */
	mc44cc373_data->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, mc44cc373_data);

        /* Register with V4L2 layer as slave device */
        sd = &mc44cc373_data->sd;

	v4l2_i2c_subdev_init(sd, client, &mc44cc373_ops);

	{
	int i ;
	for (i=0; i < mc44cc373_data->pdata->num_par; i++)
		v4l2_dbg(1, debug, sd, "Par %d=%x", i, mc44cc373_data->pdata->pars[i]);

	v4l2_info(sd, "%s Video Modulator Driver registered (ver. %s)\n",
			sd->name, DRIVER_VERSION);
	}

	/* Power Down */
	mc44cc373_s_power(sd,0);

	return 0;
}

static int mc44cc373_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
        struct mc44cc373 *t = to_state(sd);

	v4l2_device_unregister_subdev(sd);
	kfree(t);

	/* Power Down */
	mc44cc373_s_power(sd,0);
	return 0;
}

static const struct i2c_device_id mc44cc373_id[] = {
	{"mc44cc373", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mc44cc373_id);

static struct i2c_driver mc44cc373_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "mc44cc373",
	},
	.probe		= mc44cc373_probe,
	.remove		= mc44cc373_remove,
	.id_table	= mc44cc373_id,
};

static int __init mc44cc373_init(void)
{
	/* Init SysFS Structure */
	class_register(&mc44cc373_class);
	davinci_mc44cc373_device = create_sysfs_files();
	if (!davinci_mc44cc373_device) {
		printk(KERN_ERR"Could not create mc44cc373 control sysfs");
		return -EINVAL;
	}

	/* Init Driver */
	return i2c_add_driver(&mc44cc373_driver);
}

static void __exit mc44cc373_exit(void)
{
	/* Remove SysFS Structure */
	remove_sysfs_files(davinci_mc44cc373_device);
	class_unregister(&mc44cc373_class);

	/* Remove Driver */
	i2c_del_driver(&mc44cc373_driver);
}

module_init(mc44cc373_init);
module_exit(mc44cc373_exit);

/* Enf of File ****************************************************************/

