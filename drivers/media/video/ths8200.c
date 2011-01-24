/*
 * ths8200 - THS8200 Video Encoder Driver
 *
 * The encoder hardware does not support SECAM.
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
#include <linux/videodev2.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <media/davinci/videohd.h>
#include <media/ths8200.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "ths8200_regs.h"

MODULE_DESCRIPTION("THS8200 video encoder driver");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-1");


struct ths8200_state {
	struct v4l2_subdev sd;
	u32 output;
	v4l2_std_id std;
};

enum ths8200_modes {
	THS8200_1080P_60 = 0,
	THS8200_1080I_60,
	THS8200_720P_60,
	THS8200_HDTV,
	THS8200_480I_60,
	THS8200_480P_60,
	THS8200_VESA_MASTER,
	THS8200_VESA_SLAVE,
	THS8200_576I_60,
	THS8200_SDTV
};

enum ths8200_input_intf {
	THS8200_INTF_30BIT = 0,
	THS8200_INTF_16BIT_RGB,
	THS8200_INTF_15BIT_RGB,
	THS8200_INTF_20BIT_YUV422,
	THS8200_INTF_10BIT_YUV422
};

struct ths8200_mode_info {
	v4l2_std_id std;
	u32 input_intf;
	enum ths8200_modes mode;
	u32 frame_size;
	u32 field_size;
	u32 pixels_per_line;
	u32 hs_in_delay;
	u32 negative_hsync_width;
	u32 positive_hsync_width;
	u32 hfront_porch;
	u32 hback_porch;
	u32 vfront_porch;
	u32 pulse_duration;
	u32 full_line_pulse_duration;
	u32 vback_porch;
	u32 dtg_spec_k1;
	u32 vs_in_delay;
};

static inline struct ths8200_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ths8200_state, sd);
}

static inline int ths8200_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return i2c_smbus_read_byte_data(client, reg);
}

static inline int ths8200_write(struct v4l2_subdev *sd, u8 reg, u8 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return i2c_smbus_write_byte_data(client, reg, value);
}

static const struct ths8200_mode_info mode_info[] = {
	/* 1080P Mode */
	{V4L2_STD_1080P_60, THS8200_INTF_20BIT_YUV422, THS8200_1080P_60,
		1125, 2047, 2200, 0x80,
		44, 44, 132, 88, 192, 0x00, 0x00, 88, 0x00, 0x03},
	/* SD Modes */
	{V4L2_STD_525_60, THS8200_INTF_10BIT_YUV422, THS8200_480I_60,
		525, 263, 858, 0x44,
		0x3E, 0x1E, 0x79, 0x14, 0x00, 0x16B, 0x31B, 0x11, 0x0A, 0x00},
	{V4L2_STD_625_50, THS8200_INTF_10BIT_YUV422, THS8200_576I_60,
		525, 312, 864, 0x40,
		0x3E, 0x20, 0x83, 0x58, 0xC0, 0x170, 0x31B, 0x0D, 0x0A, 0x02},
	/* 720P Mode */
	{V4L2_STD_720P_60, THS8200_INTF_20BIT_YUV422, THS8200_720P_60,
		750, 2047, 1650, 0xA0,
		0x2C, 0x2C, 0x84, 0x58, 0xC0, 0x00, 0x00, 0x58, 0x00, 0x05},
	{V4L2_STD_720P_50, THS8200_INTF_20BIT_YUV422, THS8200_720P_60,
		750, 2047, 1980, 0x1F0,
		0x2C, 0x2C, 0x84, 0x58, 0xC0, 0x00, 0x00, 0x58, 0x00, 0x20},
	/* 1080I Mode */
	{V4L2_STD_1080I_60, THS8200_INTF_20BIT_YUV422, THS8200_1080I_60,
		1125, 563, 2200, 0x8C,
		44, 44, 132, 88, 192, 0x00, 0x00, 88, 0x00, 0x03},
	{V4L2_STD_1080I_50, THS8200_INTF_20BIT_YUV422, THS8200_1080I_60,
		1125, 563, 2640, 0x240,
		44, 44, 132, 88, 192, 0x00, 0x00, 88, 0x00, 0x01},
	/* ED Modes */
	{V4L2_STD_525P_60, THS8200_INTF_20BIT_YUV422, THS8200_480P_60,
		525, 0x7FF, 0x35A, 0x40,
		0x3D, 0x20, 0x7A, 0x10, 0xC0, 0x190, 0x31B, 0x10, 0x00, 0x00}
};


#define ths8200_rset(a, b) ths8200_write(sd, a, b)
static int ths8200_setstd(struct v4l2_subdev *sd, v4l2_std_id std)
{
	int num_std;
	int err = -EINVAL;
	int i = 0, value;

	num_std = sizeof(mode_info) / sizeof(struct ths8200_mode_info);
	for (i = 0; i < num_std; i++) {
		if (std & mode_info[i].std) {
			err = 0;

			ths8200_write(sd, THS8200_CHIP_CTL, 0x01);

			value = ths8200_read(sd, THS8200_DATA_CNTL);
			value &= (~THS8200_DATA_CNTL_INTF_MASK);
			value |=  mode_info[i].input_intf;
			/* Use 3.01K resistor (FSADJ1) for 700mv swing */
			value |= (1 << 6);
			err |= ths8200_write(sd, THS8200_DATA_CNTL, value);

			value = ths8200_read(sd, THS8200_DTG1_MODE);
			value &= ~(THS8200_DTG1_MODE_MASK);
			value |= mode_info[i].mode;
			err |= ths8200_write(sd, THS8200_DTG1_MODE, value);

			/* Set the pixels per line */
			value = mode_info[i].pixels_per_line &
				THS8200_DTG1_TOT_PIXELS_LSB_MASK;
			err |= ths8200_write(sd,
					THS8200_DTG1_TOT_PIXELS_LSB, value);
			value = (mode_info[i].pixels_per_line >> 8) &
				THS8200_DTG1_TOT_PIXELS_MSB_MASK;
			err |= ths8200_write(sd,
					THS8200_DTG1_TOT_PIXELS_MSB, value);

			value = mode_info[i].pixels_per_line / 2;
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_G_LSB,
					value & THS8200_DTG1_SPEC_G_LSB_MASK);
			value = (value >> 8) & THS8200_DTG1_SPEC_G_MSB_MASK;
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_G_MSB,
					value);

			/* Set the frame size */
			value = mode_info[i].frame_size &
				THS8200_DTG1_FRAME_SZ_LSB_MASK;
			err |= ths8200_write(sd,
					THS8200_DTG1_FRAME_SZ_LSB, value);

			value = ths8200_read(sd,
					THS8200_DTG1_FRAME_FIELD_SZ_MSB);
			value &= ~(THS8200_DTG1_FRAME_SZ_MSB_MASK);
			value |= ((mode_info[i].frame_size >> 8) <<
					THS8200_DTG1_FRAME_SZ_MSB_SHIFT) &
				THS8200_DTG1_FRAME_SZ_MSB_MASK;
			err |= ths8200_write(sd,
					THS8200_DTG1_FRAME_FIELD_SZ_MSB,
					value);

			/* Set the field Size */
			value = mode_info[i].field_size &
				THS8200_DTG1_FIELD_SZ_LSB_MASK;
			err |= ths8200_write(sd,
					THS8200_DTG1_FIELD_SZ_LSB, value);

			value = ths8200_read(sd,
					THS8200_DTG1_FRAME_FIELD_SZ_MSB);
			value &= ~(THS8200_DTG1_FIELD_SZ_MSB_MASK);
			value |= ((mode_info[i].frame_size >> 8) <<
					THS8200_DTG1_FIELD_SZ_MSB_SHIFT) &
				THS8200_DTG1_FIELD_SZ_MSB_MASK;
			err |= ths8200_write(sd,
					THS8200_DTG1_FRAME_FIELD_SZ_MSB, value);

			value = mode_info[i].hs_in_delay &
				THS8200_DTG2_HS_IN_DLY_LSB_MASK;
			err |= ths8200_write(sd, THS8200_DTG2_HS_IN_DLY_LSB,
					value);
			value = (mode_info[i].hs_in_delay >> 8) &
				THS8200_DTG2_HS_IN_DLY_MSB_MASK;
			err |= ths8200_write(sd, THS8200_DTG2_HS_IN_DLY_MSB,
					value);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_A,
					mode_info[i].negative_hsync_width);
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_C,
					mode_info[i].positive_hsync_width);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_D_LSB,
					(mode_info[i].hfront_porch &
					 THS8200_DTG1_SPEC_D_LSB_MASK));
			value = ths8200_read(sd, THS8200_DTG1_SPEC_DEH_MSB);
			value &= 0x7F;
			value |= ((mode_info[i].hfront_porch & 0x100) >> 1);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_DEH_MSB,
					value);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_B,
					mode_info[i].hback_porch);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_E_LSB,
					(mode_info[i].vfront_porch &
					 THS8200_DTG1_SPEC_E_LSB_MASK));
			value = ths8200_read(sd, THS8200_DTG1_SPEC_DEH_MSB);
			value &= 0xBF;
			value |= ((mode_info[i].vfront_porch & 0x100) >> 2);
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_DEH_MSB,
					value);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_H_LSB,
					(mode_info[i].pulse_duration &
					 THS8200_DTG1_SPEC_H_LSB_MASK));
			value = ths8200_read(sd, THS8200_DTG1_SPEC_DEH_MSB);
			value &= 0xFC;
			value |= ((mode_info[i].pulse_duration >> 8) &
					THS8200_DTG1_SPEC_DEH_MSB_MASK);
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_DEH_MSB,
					value);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_I_LSB,
				(mode_info[i].full_line_pulse_duration &
					 THS8200_DTG1_SPEC_I_LSB_MASK));
			value = ths8200_read(sd, THS8200_DTG1_SPEC_I_MSB);
			value &= 0xF0;
			value |= ((mode_info[i].full_line_pulse_duration >>
					8) &
					THS8200_DTG1_SPEC_I_MSB_MASK);
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_I_MSB,
					value);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_K_LSB,
					(mode_info[i].vback_porch &
					 THS8200_DTG1_SPEC_K_LSB_MASK));
			value = ths8200_read(sd, THS8200_DTG1_SPEC_K_MSB);
			value &= 0xF8;
			value |= ((mode_info[i].vback_porch >> 8) &
					THS8200_DTG1_SPEC_K_MSB_MASK);
			err |= ths8200_write(sd, THS8200_DTG1_SPEC_K_MSB,
					value);

			err |= ths8200_write(sd, THS8200_DTG1_SPEC_K1,
					mode_info[i].dtg_spec_k1);

			err |= ths8200_write(sd, THS8200_DTG2_VS_IN_DLY_LSB,
					(mode_info[i].vs_in_delay &
					 THS8200_DTG2_VS_IN_DLY_LSB_MASK));
			err |= ths8200_write(sd, THS8200_DTG2_VS_IN_DLY_MSB,
					((mode_info[i].vs_in_delay >> 8) &
                                         THS8200_DTG2_VS_IN_DLY_MSB_MASK));
			ths8200_write(sd, THS8200_CHIP_CTL, 0x00);
			mdelay(10);
			ths8200_write(sd, THS8200_CHIP_CTL, 0x01);

			break;
		}
	}

	if (i == num_std)
		v4l2_err(sd, "Std not supported\n");
	if (err != 0)
		v4l2_err(sd, "Error setting std, write failed\n");

	return err;
}

static int ths8200_setoutput(struct v4l2_subdev *sd, u32 output_type)
{
	return 0;
}

static int ths8200_log_status(struct v4l2_subdev *sd)
{
	struct ths8200_state *state = to_state(sd);

	v4l2_info(sd, "Standard: %llx\n", (unsigned long long)state->std);
	v4l2_info(sd, "Output: %s\n", (state->output == 0) ? "Composite" :
			((state->output == 1) ? "Component" : "S-Video"));
	return 0;
}

static int ths8200_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	return 0;
}

static int ths8200_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return 0;
}

static int ths8200_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return 0;
}

static int ths8200_g_chip_ident(struct v4l2_subdev *sd,
				 struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_THS8200, 0);
}

static const struct v4l2_subdev_core_ops ths8200_core_ops = {
	.log_status	= ths8200_log_status,
	.g_chip_ident	= ths8200_g_chip_ident,
	.g_ctrl		= ths8200_g_ctrl,
	.s_ctrl		= ths8200_s_ctrl,
	.queryctrl	= ths8200_queryctrl,
	.s_std 		= ths8200_setstd,
};

static int ths8200_s_std_output(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct ths8200_state *state = to_state(sd);
	int err = 0;

	if (state->std == std)
		return 0;

	err = ths8200_setstd(sd, std);
	if (!err)
		state->std = std;

	return err;
}

static int ths8200_s_routing(struct v4l2_subdev *sd,
		u32 input, u32 output, u32 config)
{
	struct ths8200_state *state = to_state(sd);
	int err = 0;

	if (state->output == output)
		return 0;

	err = ths8200_setoutput(sd, output);
	if (!err)
		state->output = output;

	return err;
}

static const struct v4l2_subdev_video_ops ths8200_video_ops = {
	.s_std_output	= ths8200_s_std_output,
	.s_routing	= ths8200_s_routing,
};

static const struct v4l2_subdev_ops ths8200_ops = {
	.core	= &ths8200_core_ops,
	.video	= &ths8200_video_ops,
};

static int ths8200_initialize(struct v4l2_subdev *sd)
{
	struct ths8200_state *state = to_state(sd);
	int err = 0;
	u8 value;

	ths8200_write(sd, THS8200_CHIP_CTL, 0x01);

	/* Initialize THS8200 registers */
	value = ths8200_read(sd, THS8200_DTG1_MODE);
	value |= THS8200_DTG1_MODE_DTG_ON_MASK;
	err = ths8200_write(sd, THS8200_DTG1_MODE, value);

	value = ths8200_read(sd, THS8200_DTG2_CNTL);
	value |= THS8200_DTG2_CNTL_IGNORE_FID_MASK |
		THS8200_DTG2_CNTL_EMD_TIMING_MASK;
	value &= (~THS8200_DTG2_CNTL_RGB_MODE_MASK);
	err |= ths8200_write(sd, THS8200_DTG2_CNTL, value);

	value = ths8200_read(sd, THS8200_CSC_OFFS3);
	value |= (THS8200_CSC_OFFS3_CSC_BYPASS_MASK |
			THS8200_CSC_OFFS3_UNDER_OVERFLOW_MASK);
	err |= ths8200_write(sd, THS8200_CSC_OFFS3, value);

	err |= ths8200_write(sd, THS8200_CHIP_CTL, 0x00);
	mdelay(10);
	err |= ths8200_write(sd, THS8200_CHIP_CTL, 0x01);

	err = ths8200_setstd(sd, state->std);
	if (err < 0) {
		v4l2_err(sd, "Error setting std during init\n");
		return -EINVAL;
	}

	return err;
}

static int ths8200_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ths8200_state *state;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	state = kzalloc(sizeof(struct ths8200_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;


	state->output = THS8200_COMPONENT_ID;
	state->std = V4L2_STD_1080P_60;

	v4l2_i2c_subdev_init(&state->sd, client, &ths8200_ops);
	return ths8200_initialize(&state->sd);
}

static int ths8200_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

	return 0;
}

static const struct i2c_device_id ths8200_id[] = {
	{"ths8200", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ths8200_id);

static struct i2c_driver ths8200_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ths8200",
	},
	.probe		= ths8200_probe,
	.remove		= ths8200_remove,
	.id_table	= ths8200_id,
};

static __init int init_ths8200(void)
{
	return i2c_add_driver(&ths8200_driver);
}

static __exit void exit_ths8200(void)
{
	i2c_del_driver(&ths8200_driver);
}

module_init(init_ths8200);
module_exit(exit_ths8200);
