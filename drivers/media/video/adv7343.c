/*
 * adv7343 - ADV7343 Video Encoder Driver
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

#include <media/adv7343.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/davinci/videohd.h>
#include "adv7343_regs.h"

MODULE_DESCRIPTION("ADV7343 video encoder driver");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level 0-1");

struct adv7343_state {
	struct v4l2_subdev sd;
	int bright;
	int hue;
	int gain;
	u32 output;
	v4l2_std_id std;
};

static inline struct adv7343_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7343_state, sd);
}

static inline int adv7343_write(struct v4l2_subdev *sd, u8 reg, u8 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return i2c_smbus_write_byte_data(client, reg, value);
}

static inline int adv7343_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return i2c_smbus_read_byte_data(client, reg);
}

static const u8 adv7343_init_reg_val[] = {
	ADV7343_SOFT_RESET, ADV7343_SOFT_RESET_DEFAULT,
	ADV7343_POWER_MODE_REG, ADV7343_POWER_MODE_REG_DEFAULT,

	ADV7343_HD_MODE_REG1, ADV7343_HD_MODE_REG1_DEFAULT,
	ADV7343_HD_MODE_REG2, ADV7343_HD_MODE_REG2_DEFAULT,
	ADV7343_HD_MODE_REG3, ADV7343_HD_MODE_REG3_DEFAULT,
	ADV7343_HD_MODE_REG4, ADV7343_HD_MODE_REG4_DEFAULT,
	ADV7343_HD_MODE_REG5, ADV7343_HD_MODE_REG5_DEFAULT,
	ADV7343_HD_MODE_REG6, ADV7343_HD_MODE_REG6_DEFAULT,
	ADV7343_HD_MODE_REG7, ADV7343_HD_MODE_REG7_DEFAULT,

	ADV7343_SD_MODE_REG1, ADV7343_SD_MODE_REG1_DEFAULT,
	ADV7343_SD_MODE_REG2, ADV7343_SD_MODE_REG2_DEFAULT,
	ADV7343_SD_MODE_REG3, ADV7343_SD_MODE_REG3_DEFAULT,
	ADV7343_SD_MODE_REG4, ADV7343_SD_MODE_REG4_DEFAULT,
	ADV7343_SD_MODE_REG5, ADV7343_SD_MODE_REG5_DEFAULT,
	ADV7343_SD_MODE_REG6, ADV7343_SD_MODE_REG6_DEFAULT,
	ADV7343_SD_MODE_REG7, ADV7343_SD_MODE_REG7_DEFAULT,
	ADV7343_SD_MODE_REG8, ADV7343_SD_MODE_REG8_DEFAULT,

	ADV7343_SD_HUE_REG, ADV7343_SD_HUE_REG_DEFAULT,
	ADV7343_SD_CGMS_WSS0, ADV7343_SD_CGMS_WSS0_DEFAULT,
	ADV7343_SD_BRIGHTNESS_WSS, ADV7343_SD_BRIGHTNESS_WSS_DEFAULT,
};

/*
 * 			    2^32
 * FSC(reg) =  FSC (HZ) * --------
 *			  27000000
 */
struct adv7343_std_info adv7343_composite_std_info[ADV7343_COMPOSITE_NUM_STD] =
{
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 3,579,545.45 Hz */
		SD_STD_NTSC, 569408542, V4L2_STD_NTSC},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 4,433,618.75 Hz */
		SD_STD_PAL_BDGHI, 705268427, V4L2_STD_PAL},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 3,575,611.00 Hz */
		SD_STD_PAL_M, 568782678, V4L2_STD_PAL_M},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 3,582,056.00 */
		SD_STD_PAL_N, 569807903, V4L2_STD_PAL_Nc},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 4,433,618.75 Hz */
		SD_STD_PAL_N, 705268427, V4L2_STD_PAL_N},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 4,433,618.75 Hz */
		SD_STD_NTSC, 705268427, V4L2_STD_NTSC_443},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		/* FSC(Hz) = 4,433,618.75 Hz */
		SD_STD_PAL_M, 705268427, V4L2_STD_PAL_60}
};


struct adv7343_std_info adv7343_component_std_info[ADV7343_COMPONENT_NUM_STD] =
{
	{ ADV7343_HD_MODE_REG1, HD_720P_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_720P << STD_MODE_SHIFT),
		569408542, V4L2_STD_720P_60},
#if 0
	{ ADV7343_HD_MODE_REG1, HD_720P_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_720P_25 << STD_MODE_SHIFT),
		569408542, V4L2_STD_720P_25},
	{ ADV7343_HD_MODE_REG1, HD_720P_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_720P_30 << STD_MODE_SHIFT),
		569408542, V4L2_STD_720P_30},
#endif
	{ ADV7343_HD_MODE_REG1, HD_720P_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_720P_50 << STD_MODE_SHIFT),
		569408542, V4L2_STD_720P_50},
	{ ADV7343_HD_MODE_REG1, HD_1080I_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_1080I << STD_MODE_SHIFT),
		569408542, V4L2_STD_1080I_60},
	{ ADV7343_HD_MODE_REG1, HD_1080I_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_1080I_25fps << STD_MODE_SHIFT),
		569408542, V4L2_STD_1080I_50},
	{ ADV7343_HD_MODE_REG1, HD_720P_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_525P << STD_MODE_SHIFT),
		569408542, V4L2_STD_525P_60},
	{ ADV7343_HD_MODE_REG1, HD_720P_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_625P << STD_MODE_SHIFT),
		569408542, V4L2_STD_625P_50},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		SD_STD_NTSC, 569408542, V4L2_STD_525_60},
	{ ADV7343_SD_MODE_REG1, SD_INPUT_MODE, (~(SD_STD_MASK)),
		SD_STD_PAL_BDGHI, 569408542, V4L2_STD_625_50},
#if 0
	{ ADV7343_HD_MODE_REG1, HD_1080I_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_1080P_24 << STD_MODE_SHIFT),
		569408542, V4L2_STD_1080P_24},
	{ ADV7343_HD_MODE_REG1, HD_1080I_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_1080P_25 << STD_MODE_SHIFT),
		569408542, V4L2_STD_1080P_25},
	{ ADV7343_HD_MODE_REG1, HD_1080I_INPUT_MODE,
		(~(STD_MODE_MASK << STD_MODE_SHIFT)),
		(STD_MODE_1080P_30 << STD_MODE_SHIFT),
		569408542, V4L2_STD_1080P_30}
#endif
};

struct adv7343_output_info output_info[] = {
	{ADV7343_COMPOSITE_ID, ADV7343_COMPOSITE_NUM_STD,
		ADV7343_COMPOSITE_POWER_VALUE,
		adv7343_composite_std_info},
	{ADV7343_COMPONENT_ID, ADV7343_COMPONENT_NUM_STD,
		ADV7343_COMPONENT_POWER_VALUE,
		adv7343_component_std_info},
	{ADV7343_SVIDEO_ID, ADV7343_SVIDEO_NUM_STD,
		ADV7343_SVIDEO_POWER_VALUE,
		adv7343_composite_std_info}
};

static int adv7343_setstd(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct adv7343_state *state = to_state(sd);
	struct adv7343_std_info *std_info;
	int output_idx, num_std;
	char *fsc_ptr;
	u8 reg;
	int err = 0, val;
	int i = 0;

	output_idx = state->output;

	std_info =
		(struct adv7343_std_info *)(output_info[output_idx].std_info);
	num_std = output_info[output_idx].num_std;

	for (i = 0; i < num_std; i++) {
		if (std_info[i].stdid & std)
			break;
	}

	if (i == num_std) {
		v4l2_dbg(1, debug, sd,
				"Invalid std or std is not supported: %llx\n",
				(unsigned long long)std);
		return -EINVAL;
	}

	/* Set the standard */
	val = adv7343_read(sd, std_info[i].set_std_reg);
	if (val < 0)
		goto setstd_exit;

	val = val & std_info[i].standard_val2;
	val |= std_info[i].standard_val3;
	err = adv7343_write(sd, std_info[i].set_std_reg, val);
	if (err < 0)
		goto setstd_exit;

	/* Configure the input mode register */
	val = adv7343_read(sd, ADV7343_MODE_SELECT_REG);
	if (val < 0)
		goto setstd_exit;

	val = val & (~((u8) INPUT_MODE_MASK));
	val |= std_info[i].outputmode_val;
	err = adv7343_write(sd, ADV7343_MODE_SELECT_REG, val);
	if (err < 0)
		goto setstd_exit;

	/* Program the sub carrier frequency registers */
	fsc_ptr = (unsigned char *)&std_info[i].fsc_val;
	reg = ADV7343_FSC_REG0;
	for (i = 0; i < 4; i++, reg++, fsc_ptr++) {
		err = adv7343_write(sd, reg, *fsc_ptr);
		if (err < 0)
			goto setstd_exit;
	}

	/* Filter settings */
	val = adv7343_read(sd, ADV7343_SD_MODE_REG1);
	if (val < 0)
		goto setstd_exit;

	if (std & (V4L2_STD_NTSC | V4L2_STD_NTSC_443))
		val &= 0x03;
	else if (std & ~V4L2_STD_SECAM)
		val |= 0x04;

	err = adv7343_write(sd, ADV7343_SD_MODE_REG1, val);
	if (err < 0)
		goto setstd_exit;

	state->std = std;
	return 0;
setstd_exit:
	if (err != 0)
		v4l2_err(sd, "Error setting std, write failed\n");

	return err;
}

static int adv7343_setoutput(struct v4l2_subdev *sd, u32 output_type)
{
	struct adv7343_state *state = to_state(sd);
	int i, val;
	int err = 0;

	if (output_type > ADV7343_SVIDEO_ID) {
		v4l2_dbg(1, debug, sd,
			"Invalid output type or output type not supported:%d\n",
								output_type);
		return -EINVAL;
	}

	/* Enable Appropriate DAC */

	/* Enable PLL and disable sleep mode */
	val = 0x02;
	for (i = 0; i < ARRAY_SIZE(output_info); i++) {
		if (output_type == output_info[i].output_type) {
			val |= output_info[i].dac_enable;
			break;
		}
	}
	if (i == ARRAY_SIZE(output_info)) {
		v4l2_dbg(1, debug, sd,
			"Invalid output type or output type not supported:%d\n",
			output_type);
		return -EINVAL;
	}

	err = adv7343_write(sd, ADV7343_POWER_MODE_REG, val);
	if (err < 0)
		goto setoutput_exit;

	/* Enable YUV output */
	val = adv7343_read(sd, ADV7343_MODE_REG0);
	if (val < 0)
		goto setoutput_exit;
	val = val | YUV_OUTPUT_SELECT;
	err = adv7343_write(sd, ADV7343_MODE_REG0, val);
	if (err < 0)
		goto setoutput_exit;

	/* configure SD DAC Output 2 and SD DAC Output 1 bit to zero */
	val = adv7343_read(sd, ADV7343_SD_MODE_REG2);
	if (val < 0)
		goto setoutput_exit;
	val &= (SD_DAC_1_DI & SD_DAC_2_DI);
	err = adv7343_write(sd, ADV7343_SD_MODE_REG2, val);
	if (err < 0)
		goto setoutput_exit;

	/* configure ED/HD Color DAC Swap and ED/HD RGB Input Enable bit to
	 * zero */
	val = adv7343_read(sd, ADV7343_HD_MODE_REG6);
	if (val < 0)
		goto setoutput_exit;
	val &= (HD_RGB_INPUT_DI & HD_DAC_SWAP_DI);
	err = adv7343_write(sd, ADV7343_HD_MODE_REG6, val);
	if (err < 0)
		goto setoutput_exit;

	state->output = i;

setoutput_exit:
	if (err != 0)
		v4l2_err(sd, "Error setting output, write failed\n");

	return err;
}

static int adv7343_log_status(struct v4l2_subdev *sd)
{
	struct adv7343_state *state = to_state(sd);

	v4l2_info(sd, "Standard: %llx\n", (unsigned long long)state->std);
	v4l2_info(sd, "Output: %s\n", (state->output == 0) ? "Composite" :
			((state->output == 1) ? "Component" : "S-Video"));
	return 0;
}

static int adv7343_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	switch (qc->id) {
	case V4L2_CID_BRIGHTNESS:
		return v4l2_ctrl_query_fill(qc, ADV7343_BRIGHTNESS_MIN,
						ADV7343_BRIGHTNESS_MAX, 1,
						ADV7343_BRIGHTNESS_DEF);
	case V4L2_CID_HUE:
		return v4l2_ctrl_query_fill(qc, ADV7343_HUE_MIN,
						ADV7343_HUE_MAX, 1 ,
						ADV7343_HUE_DEF);
	case V4L2_CID_GAIN:
		return v4l2_ctrl_query_fill(qc, ADV7343_GAIN_MIN,
						ADV7343_GAIN_MAX, 1,
						ADV7343_GAIN_DEF);
	default:
		break;
	}

	return 0;
}

static int adv7343_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct adv7343_state *state = to_state(sd);
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		if (ctrl->value < ADV7343_BRIGHTNESS_MIN ||
					ctrl->value > ADV7343_BRIGHTNESS_MAX) {
			v4l2_dbg(1, debug, sd,
					"invalid brightness settings %d\n",
								ctrl->value);
			return -ERANGE;
		}

		state->bright = ctrl->value;
		err = adv7343_write(sd, ADV7343_SD_BRIGHTNESS_WSS,
					state->bright);
		break;

	case V4L2_CID_HUE:
		if (ctrl->value < ADV7343_HUE_MIN ||
					ctrl->value > ADV7343_HUE_MAX) {
			v4l2_dbg(1, debug, sd, "invalid hue settings %d\n",
								ctrl->value);
			return -ERANGE;
		}

		state->hue = ctrl->value;
		err = adv7343_write(sd, ADV7343_SD_HUE_REG, state->hue);
		break;

	case V4L2_CID_GAIN:
		if (ctrl->value < ADV7343_GAIN_MIN ||
					ctrl->value > ADV7343_GAIN_MAX) {
			v4l2_dbg(1, debug, sd, "invalid gain settings %d\n",
								ctrl->value);
			return -ERANGE;
		}

		if ((ctrl->value > POSITIVE_GAIN_MAX) &&
			(ctrl->value < NEGATIVE_GAIN_MIN)) {
			v4l2_dbg(1, debug, sd,
				"gain settings not within the specified range\n");
			return -ERANGE;
		}

		state->gain = ctrl->value;
		err = adv7343_write(sd, ADV7343_DAC2_OUTPUT_LEVEL, state->gain);
		break;

	default:
		return -EINVAL;
	}

	if (err < 0)
		v4l2_err(sd, "Failed to set the encoder controls\n");

	return err;
}

static int adv7343_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct adv7343_state *state = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = state->bright;
		break;

	case V4L2_CID_HUE:
		ctrl->value = state->hue;
		break;

	case V4L2_CID_GAIN:
		ctrl->value = state->gain;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int adv7343_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_ADV7343, 0);
}

static const struct v4l2_subdev_core_ops adv7343_core_ops = {
	.log_status	= adv7343_log_status,
	.g_chip_ident	= adv7343_g_chip_ident,
	.g_ctrl		= adv7343_g_ctrl,
	.s_ctrl		= adv7343_s_ctrl,
	.queryctrl	= adv7343_queryctrl,
};

static int adv7343_s_std_output(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct adv7343_state *state = to_state(sd);
	int err = 0;

	if (state->std == std)
		return 0;

	err = adv7343_setstd(sd, std);
	if (!err)
		state->std = std;

	return err;
}

static int adv7343_s_routing(struct v4l2_subdev *sd,
		u32 input, u32 output, u32 config)
{
	struct adv7343_state *state = to_state(sd);
	int err = 0;

	if (state->output == output)
		return 0;

	err = adv7343_setoutput(sd, output);
	if (!err)
		state->output = output;

	return err;
}

static const struct v4l2_subdev_video_ops adv7343_video_ops = {
	.s_std_output	= adv7343_s_std_output,
	.s_routing	= adv7343_s_routing,
};

static const struct v4l2_subdev_ops adv7343_ops = {
	.core	= &adv7343_core_ops,
	.video	= &adv7343_video_ops,
};

static int adv7343_initialize(struct v4l2_subdev *sd)
{
	struct adv7343_state *state = to_state(sd);
	int err = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(adv7343_init_reg_val); i += 2) {

		err = adv7343_write(sd, adv7343_init_reg_val[i],
					adv7343_init_reg_val[i+1]);
		if (err) {
			v4l2_err(sd, "Error initializing\n");
			return err;
		}
	}

	/* Configure for default video standard */
	err = adv7343_setoutput(sd, state->output);
	if (err < 0) {
		v4l2_err(sd, "Error setting output during init\n");
		return -EINVAL;
	}

	err = adv7343_setstd(sd, state->std);
	if (err < 0) {
		v4l2_err(sd, "Error setting std during init\n");
		return -EINVAL;
	}

	return err;
}

static int adv7343_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct adv7343_state *state;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	v4l_info(client, "chip found @ 0x%x (%s)\n",
			client->addr << 1, client->adapter->name);

	state = kzalloc(sizeof(struct adv7343_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	state->output = ADV7343_COMPOSITE_ID;
	state->std = V4L2_STD_NTSC;

	v4l2_i2c_subdev_init(&state->sd, client, &adv7343_ops);
	return adv7343_initialize(&state->sd);
}

static int adv7343_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

	return 0;
}

static const struct i2c_device_id adv7343_id[] = {
	{"adv7343", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7343_id);

static struct i2c_driver adv7343_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "adv7343",
	},
	.probe		= adv7343_probe,
	.remove		= adv7343_remove,
	.id_table	= adv7343_id,
};

static __init int init_adv7343(void)
{
	return i2c_add_driver(&adv7343_driver);
}

static __exit void exit_adv7343(void)
{
	i2c_del_driver(&adv7343_driver);
}

module_init(init_adv7343);
module_exit(exit_adv7343);
