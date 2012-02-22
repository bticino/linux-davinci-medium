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

//static struct i2c_client *mc44cc373_client = NULL;

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
        u8 buf[] = { t->pdata->pars };
        int len = t->pdata->num_par;
	//u8 buf[] = {0xBA, 0x18, 0x26, 0xE8};

	printk("%s: len=%d \n", __func__, len);

        v4l2_dbg(1, debug, sd, "Switching ON the modulator\n");
//	gpio_set_value(t->pdata->power, 1);

	// mdelay(10);

        ret = i2c_master_send(client, buf, len);//ARRAY_SIZE(buf));


        v4l2_dbg(1, debug, sd, "Switching OFF the modulator\n");
//	gpio_set_value(t->pdata->power, 0);

        //ret = (ret == ARRAY_SIZE(buf)) ? 0 : ret;
        //v4l2_dbg(1, debug, sd, "Reading status byte\n");
        //ret = i2c_master_recv(client, &status, 1);
        //if (unlikely(ret != 1))
        //        dev_err(&client->dev, "wanted %d bytes, got %d\n",
        //                1, ret);
        //v4l2_dbg(1, debug, sd, "Status byte 0x%02X\n", status);
        //ret = 0;

        //switch (status & AFCWIN) {
       // case 1:
       //         *std = V4L2_STD_PAL;
       // break;
       // case 0:
       // default:
       // break;
       // }

       // return 0;
        //  u8 buf[] = {
        //        0, t->pdata->switching_mode, t->pdata->adjust_mode, t->pdata->data_mode,
        //};

	if (ret)
		v4l2_err(sd, "mc44cc373 write failed\n");
	return ret;
}
EXPORT_SYMBOL(mc44cc373_setvalue);

static int mc44cc373_s_std_output(struct v4l2_subdev *sd)
{
	return mc44cc373_setvalue(sd);
}

static int mc44cc373_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, 1111, 0); /*TODO*/
}

static const struct v4l2_subdev_video_ops mc44cc373_video_ops = {
	.s_std_output	= mc44cc373_s_std_output,
};

static const struct v4l2_subdev_core_ops mc44cc373_core_ops = {
	.g_chip_ident = mc44cc373_g_chip_ident,
};

static const struct v4l2_subdev_ops mc44cc373_ops = {
	.core	= &mc44cc373_core_ops,
	.video	= &mc44cc373_video_ops,
};

static int mc44cc373_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	//v4l2_std_id std_id = V4L2_STD_NTSC;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	sd = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (sd == NULL)
		return -ENOMEM;

	v4l2_i2c_subdev_init(sd, client, &mc44cc373_ops);

	//mc44cc373_client = client;

	return mc44cc373_s_std_output(sd);
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

