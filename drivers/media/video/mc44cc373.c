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

MODULE_DESCRIPTION("mc44cc373 Video Modulator Driver");
MODULE_AUTHOR("Chaithrika U S, modify by SC");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-1");

struct mc44cc373 {
        struct v4l2_subdev sd;
        const struct mc44cc373_platform_data *pdata;
        int status;
};

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

static int mc44cc373_s_std_output(struct v4l2_subdev *sd)
{
	return mc44cc373_setvalue(sd);
}

static int mc44cc373_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct mc44cc373 *t = to_state(sd);

        //v4l2_dbg(1, debug, sd, "Switching %s the modulator\n", on ? "on" : "off");
        v4l_info(client, "Switching %s the modulator\n", on ? "on" : "off");
	gpio_set_value(t->pdata->power, on);

	if (on)
		return mc44cc373_setvalue(sd);
	return 0;
}

static const struct v4l2_subdev_video_ops mc44cc373_video_ops = {
	.s_std_output	= mc44cc373_s_std_output,
};

static const struct v4l2_subdev_core_ops mc44cc373_core_ops = {
	.s_power = mc44cc373_s_power,
};

static const struct v4l2_subdev_ops mc44cc373_ops = {
	.core	= &mc44cc373_core_ops,
	.video	= &mc44cc373_video_ops,
};

static int mc44cc373_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct mc44cc373 *data;
	struct v4l2_subdev *sd;
	//v4l2_std_id std_id = V4L2_STD_NTSC;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

        data = kzalloc(sizeof(struct  mc44cc373), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

	/* Copy board specific information here */
	data->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, data);

        /* Register with V4L2 layer as slave device */
        sd = &data->sd;

	//int i ;
	//for (i=0; i < data->pdata->num_par; i++)
	//	printk("Par %d=%x", i, data->pdata->pars[i]);

	v4l2_i2c_subdev_init(sd, client, &mc44cc373_ops);

	return mc44cc373_s_power(sd,1); /*on*/
}

static int mc44cc373_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(sd);

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
	return i2c_add_driver(&mc44cc373_driver);
}

static void __exit mc44cc373_exit(void)
{
	i2c_del_driver(&mc44cc373_driver);
}

module_init(mc44cc373_init);
module_exit(mc44cc373_exit);

