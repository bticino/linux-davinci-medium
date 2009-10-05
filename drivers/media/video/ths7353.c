/*
 * ths7353 - THS7353 Video Amplifier driver
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
 *
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

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>

#include <media/davinci/videohd.h>

MODULE_DESCRIPTION("TI THS7353 video amplifier driver");
MODULE_AUTHOR("Muralidharan Karicheri");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-1");

#define THS7353_CHANNEL_1	1
#define THS7353_CHANNEL_2	2
#define THS7353_CHANNEL_3	3
#define THS7353_DEF_LUMA_CHANNEL 2

/* all supported modes */
enum ths7353_filter_mode {
	THS_FILTER_MODE_480I_576I,
	THS_FILTER_MODE_480P_576P,
	THS_FILTER_MODE_720P_1080I,
	THS_FILTER_MODE_1080P
};

static int ths7353_luma_channel;

/* following function is used to set ths7353 */
static int ths7353_setvalue(struct v4l2_subdev *sd,
			    enum ths7353_filter_mode mode)
{
	u8 val = 0, input_bias_luma = 5, input_bias_chroma = 4, temp;
	struct i2c_client *client;
	int err = 0, disable = 0;
	int channel = 3;

	client = v4l2_get_subdevdata(sd);

	switch (mode) {
	case THS_FILTER_MODE_1080P:
		/* LPF - 5MHz */
		val = (3 << 6);
		/* LPF - bypass */
		val |= (3 << 3);
		break;
	case THS_FILTER_MODE_720P_1080I:
		/* LPF - 5MHz */
		val = (2 << 6);
		/* LPF - 35 MHz */
		val |= (2 << 3);
		break;
	case THS_FILTER_MODE_480P_576P:
		/* LPF - 2.5MHz */
		val = (1 << 6);
		/* LPF - 16 MHz */
		val |= (1 << 3);
		break;
	case THS_FILTER_MODE_480I_576I:
		/* LPF - 500 KHz, LPF - 9 MHz. Do nothing */
		break;
	default:
		/* disable all channels */
		disable = 1;
	}

	channel = ths7353_luma_channel;

	/* Setup channel 2 - Luma - Green */
	temp = val;
	if (!disable)
		val |= input_bias_luma;
	err = i2c_smbus_write_byte_data(client, channel, val);
	if (err)
		goto out;

	/* setup two chroma channels */
	if (!disable)
		temp |= input_bias_chroma;
	channel++;
	if (channel > THS7353_CHANNEL_3)
		channel = THS7353_CHANNEL_1;
	err = i2c_smbus_write_byte_data(client, channel, temp);
	if (err)
		goto out;

	channel++;
	if (channel > THS7353_CHANNEL_3)
		channel = THS7353_CHANNEL_1;
	err = i2c_smbus_write_byte_data(client, channel, temp);
	if (err)
		goto out;
	return 0;

out:
	v4l2_err(sd, "ths7353 write failed\n");
	return err;
}

static int ths7353_s_std_output(struct v4l2_subdev *sd, v4l2_std_id norm)
{
	if (norm & (V4L2_STD_ALL & ~V4L2_STD_SECAM))
		return ths7353_setvalue(sd, THS_FILTER_MODE_480I_576I);
	else if (norm & (V4L2_STD_525P_60 | V4L2_STD_625P_50))
		return ths7353_setvalue(sd, THS_FILTER_MODE_480P_576P);
	else if (norm & (V4L2_STD_720P_60 | V4L2_STD_720P_50 |
		 V4L2_STD_1080I_60 | V4L2_STD_1080I_50))
		return ths7353_setvalue(sd, THS_FILTER_MODE_720P_1080I);
	else if (norm & (V4L2_STD_1080P_60 | V4L2_STD_1080P_50))
		return ths7353_setvalue(sd, THS_FILTER_MODE_1080P);
	else
		return -EINVAL;
}

static int ths7353_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_THS7353, 0);
}

static const struct v4l2_subdev_video_ops ths7353_video_ops = {
	.s_std_output	= ths7353_s_std_output,
};

static const struct v4l2_subdev_core_ops ths7353_core_ops = {
	.g_chip_ident = ths7353_g_chip_ident,
};

static const struct v4l2_subdev_ops ths7353_ops = {
	.core	= &ths7353_core_ops,
	.video 	= &ths7353_video_ops,
};

static int ths7353_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	if (!client->dev.platform_data) {
		v4l_warn(client, "No platform data!!\n");
		ths7353_luma_channel = THS7353_DEF_LUMA_CHANNEL;
	} else {
		ths7353_luma_channel = (int)(client->dev.platform_data);
		if ((ths7353_luma_channel < THS7353_CHANNEL_1) ||
				(ths7353_luma_channel > THS7353_CHANNEL_3)) {
			v4l_warn(client, "Wring Luma Channel!!\n");
			ths7353_luma_channel = THS7353_DEF_LUMA_CHANNEL;
		}
	}

	sd = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (sd == NULL)
		return -ENOMEM;

	v4l2_i2c_subdev_init(sd, client, &ths7353_ops);

	return ths7353_setvalue(sd, THS_FILTER_MODE_480I_576I);
}

static int ths7353_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(sd);

	return 0;
}

static const struct i2c_device_id ths7353_id[] = {
	{"ths7353", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ths7353_id);

static struct i2c_driver ths7353_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ths7353",
	},
	.probe		= ths7353_probe,
	.remove		= ths7353_remove,
	.id_table	= ths7353_id,
};

static int __init ths7353_init(void)
{
	return i2c_add_driver(&ths7353_driver);
}

static void __exit ths7353_exit(void)
{
	i2c_del_driver(&ths7353_driver);
}

module_init(ths7353_init);
module_exit(ths7353_exit);

