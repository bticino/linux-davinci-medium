/*
 * Driver for MT9P031 CMOS Image Sensor from Micron, for TI Davinci platform
 *
 * Copyright (C) 2008, Guennadi Liakhovetski,
 * DENX Software Engineering <lg@denx.de>
 *
 * Heavily based on MT9T031 driver from Guennadi Liakhovetski
 * made changes to support TI Davinci platform
 * Copyright (C) 2010, Leopard Imaging, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/davinci/videohd.h>

/* mt9p031 i2c address 0x5d
 * The platform has to define i2c_board_info
 * and call i2c_register_board_info() */

/* mt9p031 selected register addresses */
#define MT9P031_CHIP_VERSION		0x00
#define MT9P031_ROW_START		0x01
#define MT9P031_COLUMN_START		0x02
#define MT9P031_WINDOW_HEIGHT		0x03
#define MT9P031_WINDOW_WIDTH		0x04
#define MT9P031_HORIZONTAL_BLANKING	0x05
#define MT9P031_VERTICAL_BLANKING	0x06
#define MT9P031_OUTPUT_CONTROL		0x07
#define MT9P031_SHUTTER_WIDTH_UPPER	0x08
#define MT9P031_SHUTTER_WIDTH		0x09
#define MT9P031_PIXEL_CLOCK_CONTROL	0x0a
#define MT9P031_FRAME_RESTART		0x0b
#define MT9P031_SHUTTER_DELAY		0x0c
#define MT9P031_RESET			0x0d
#define MT9P031_READ_MODE_1		0x1e
#define MT9P031_READ_MODE_2		0x20
#define MT9P031_READ_MODE_3		0x21
#define MT9P031_ROW_ADDRESS_MODE	0x22
#define MT9P031_COLUMN_ADDRESS_MODE	0x23
#define MT9P031_GLOBAL_GAIN		0x35
#define MT9P031_CHIP_ENABLE		0xF8

#define MT9P031_MAX_HEIGHT		1536
#define MT9P031_MAX_WIDTH		2048
#define MT9P031_MIN_HEIGHT		2
#define MT9P031_MIN_WIDTH		2
#define MT9P031_HORIZONTAL_BLANK	0
#define MT9P031_VERTICAL_BLANK	0
#define MT9P031_COLUMN_SKIP		32
#define MT9P031_ROW_SKIP		20
#define MT9P031_DEFAULT_WIDTH		1920
#define MT9P031_DEFAULT_HEIGHT		1080

#define V4L2_STD_MT9P031_STD_ALL  (V4L2_STD_525_60\
	|V4L2_STD_625_50|V4L2_STD_525P_60\
	|V4L2_STD_625P_50|V4L2_STD_720P_30\
   |V4L2_STD_720P_50|V4L2_STD_720P_60\
   |V4L2_STD_1080I_30|V4L2_STD_1080I_50\
	|V4L2_STD_1080I_60|V4L2_STD_1080P_30\
   |V4L2_STD_1080P_50|V4L2_STD_1080P_60)

#define MT9P031_BUS_PARAM	(SOCAM_PCLK_SAMPLE_RISING |	\
	SOCAM_PCLK_SAMPLE_FALLING | SOCAM_HSYNC_ACTIVE_HIGH |	\
	SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_DATA_ACTIVE_HIGH |	\
	SOCAM_MASTER | SOCAM_DATAWIDTH_10)

v4l2_std_id mt9p031_cur_std = V4L2_STD_1080P_30;
/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static const struct v4l2_fmtdesc mt9p031_formats[] = {
	{
		.index = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.description = "Bayer (sRGB) 10 bit",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
	},
};
static const unsigned int mt9p031_num_formats = ARRAY_SIZE(mt9p031_formats);

static const struct v4l2_queryctrl mt9p031_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Horizontally",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Gain",
		.minimum	= 0,
		.maximum	= 127,
		.step		= 1,
		.default_value	= 64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 1,
		.maximum	= 65535,
		.step		= 1,
		.default_value	= 255,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}
};
static const unsigned int mt9p031_num_controls = ARRAY_SIZE(mt9p031_controls);

struct mt9p031 {
	struct v4l2_subdev sd;
	int model;	/* V4L2_IDENT_MT9P031* codes from v4l2-chip-ident.h */
	unsigned char autoexposure;
	u16 xskip;
	u16 yskip;
	u32 width;
	u32 height;
	unsigned short x_min;           /* Camera capabilities */
	unsigned short y_min;
	unsigned short x_current;       /* Current window location */
	unsigned short y_current;
	unsigned short width_min;
	unsigned short width_max;
	unsigned short height_min;
	unsigned short height_max;
	unsigned short y_skip_top;      /* Lines to skip at the top */
	unsigned short gain;
	unsigned short exposure;
};

static inline struct mt9p031 *to_mt9p031(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9p031, sd);
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
	s32 data;

	data = i2c_smbus_read_word_data(client, reg);
	return data < 0 ? data : swab16(data);
}

static int reg_write(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	return i2c_smbus_write_word_data(client, reg, swab16(data));
}

static int reg_set(struct i2c_client *client, const u8 reg,
		   const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret | data);
}

static int reg_clear(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret & ~data);
}

static int set_shutter(struct v4l2_subdev *sd, const u32 data)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = reg_write(client, MT9P031_SHUTTER_WIDTH_UPPER, data >> 16);

	if (ret >= 0)
		ret = reg_write(client, MT9P031_SHUTTER_WIDTH, data & 0xffff);

	return ret;
}

static int get_shutter(struct v4l2_subdev *sd, u32 *data)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	ret = reg_read(client, MT9P031_SHUTTER_WIDTH_UPPER);
	*data = ret << 16;

	if (ret >= 0)
		ret = reg_read(client, MT9P031_SHUTTER_WIDTH);
	*data |= ret & 0xffff;

	return ret < 0 ? ret : 0;
}

static int mt9p031_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Disable chip output, synchronous option update */
	ret = reg_write(client, MT9P031_RESET, 1);
	if (ret >= 0)
		ret = reg_write(client, MT9P031_RESET, 0);
	if (ret >= 0)
		ret = reg_clear(client, MT9P031_OUTPUT_CONTROL, 2);

	return ret >= 0 ? 0 : -EIO;
}

static int mt9p031_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Switch to master "normal" mode */
	if (enable) {
		if (reg_set(client, MT9P031_OUTPUT_CONTROL, 2) < 0)
			return -EIO;
	} else {
	/* Never disable image sensor output */
	}
	return 0;
}

/* Round up minima and round down maxima */
static void recalculate_limits(struct mt9p031 *mt9p031,
			       u16 xskip, u16 yskip)
{
	mt9p031->x_min = (MT9P031_COLUMN_SKIP + xskip - 1) / xskip;
	mt9p031->y_min = (MT9P031_ROW_SKIP + yskip - 1) / yskip;
	mt9p031->width_min = (MT9P031_MIN_WIDTH + xskip - 1) / xskip;
	mt9p031->height_min = (MT9P031_MIN_HEIGHT + yskip - 1) / yskip;
	mt9p031->width_max = MT9P031_MAX_WIDTH / xskip;
	mt9p031->height_max = MT9P031_MAX_HEIGHT / yskip;
}

const struct v4l2_queryctrl *mt9p031_find_qctrl(u32 id)
{
	int i;

	for (i = 0; i < mt9p031_num_controls; i++) {
		if (mt9p031_controls[i].id == id)
			return &mt9p031_controls[i];
	}
	return NULL;
}

static int mt9p031_set_params(struct v4l2_subdev *sd,
			      struct v4l2_rect *rect, u16 xskip, u16 yskip)
{
	struct mt9p031 *mt9p031 = to_mt9p031(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int i, ret;
	u16 xbin, ybin, width, height, left, top;
   u16 hblank = MT9P031_HORIZONTAL_BLANK,
		 vblank = MT9P031_VERTICAL_BLANK;

unsigned int reg[][2] = {
/* PLL Control */
{ 0x0010, 0x0051},       	/*PLL_CONTROL */
{ 0x0011, 0x1C01},        	/*PLL Config1 = 0x1701*/
{ 0x0012, 0x0005},        	/*PLL Config2 = 0x5*/
{ 0x0010, 0x0053},        	/*PLL Control = 0x53*/
{ 0x0007, 0x1F82},        	/*(3) GREEN2_GAIN_REG = 0x8*/
{ 0x002B, 0x0008},       	/*(3) GREEN1_GAIN_REG = 0x8*/
{ 0x002C, 0x0012},       	/*(3) BLUE_GAIN_REG = 0x12*/
{ 0x002D, 0x000A},        	/*(3) RED_GAIN_REG = 0xA*/
{ 0x002E, 0x0008},        	/*(3) GREEN2_GAIN_REG = 0x8*/

/* [Subsample Optimization] */
{ 0x70, 0x5c},
{ 0x71, 0x5B00},
{ 0x72, 0x5900},
{ 0x73, 0x200},
{ 0x74, 0x200},
{ 0x75, 0x2800},
{ 0x76, 0x3E29},
{ 0x77, 0x3E29},
{ 0x78, 0x583F},
{ 0x79, 0x5B00},
{ 0x7A, 0x5A00},
{ 0x7B, 0x5900},
{ 0x7C, 0x5900},
{ 0x7E, 0x5900},
{ 0x7F, 0x5900},
{ 0x06, 0x0},
{ 0x29, 0x481},
{ 0x3E, 0x87},
{ 0x3F, 0x7},
{ 0x41, 0x3},
{ 0x48, 0x18},
{ 0x5F, 0x1C16},
{ 0x57, 0x7}
};

	if (mt9p031_cur_std == V4L2_STD_1080P_30
		|| mt9p031_cur_std == V4L2_STD_1080P_50
		|| mt9p031_cur_std == V4L2_STD_1080P_60
		|| mt9p031_cur_std == V4L2_STD_1080I_30
		|| mt9p031_cur_std == V4L2_STD_1080I_50
		|| mt9p031_cur_std == V4L2_STD_1080I_60){
			reg[0][1] = 0x51;
			reg[1][1] = 0x1801;
			reg[2][1] = 0x0002;
			reg[3][1] = 0x53;
			reg[4][1] = 0x1F8E;
	}

	for (i = 0; i < sizeof(reg)/(2*sizeof(int)); i++) {
		ret = reg_write(client, reg[i][0], reg[i][1]);
		if (ret < 0) {
			printk(KERN_INFO"-----mt9p031_set_params()..write reg error..\n");
			break;
		}
		mdelay(1);
	}

	/* Make sure we don't exceed sensor limits */
	if (rect->left + rect->width > mt9p031->width_max)
		rect->left =
		(mt9p031->width_max - rect->width) / 2 + mt9p031->x_min;

	if (rect->top + rect->height > mt9p031->height_max)
		rect->top =
		(mt9p031->height_max - rect->height) / 2 + mt9p031->y_min;

	width = rect->width;
	height = rect->height;
	left = rect->left;
	top = rect->top;

   xskip = 0;
   yskip = 0;
	xbin = min(xskip, (u16)3);
	ybin = min(yskip, (u16)3);

	v4l2_dbg(1, debug, sd, "xskip %u, width %u/%u, yskip %u,"
		"height %u/%u\n", xskip, width, rect->width, yskip,
		height, rect->height);

	/* Could just do roundup(rect->left, [xy]bin * 2);
		but this is cheaper */
	switch (xbin) {
	case 2:
		left = (left + 3) & ~3;
		break;
	case 3:
		left = roundup(left, 6);
	}

	switch (ybin) {
	case 2:
		top = (top + 3) & ~3;
		break;
	case 3:
		top = roundup(top, 6);
	}

	/* Disable register update, reconfigure atomically */
	ret = reg_set(client, MT9P031_OUTPUT_CONTROL, 1);
	if (ret < 0)
		return ret;


	/* Blanking and start values - default... */
	if (mt9p031_cur_std == V4L2_STD_720P_30
		|| mt9p031_cur_std == V4L2_STD_720P_50
		|| mt9p031_cur_std == V4L2_STD_720P_60){
			left = (2592-2560)/2+16-rect->left;
			top = (1944-1472)/2+54-rect->top;
			width = 2560+rect->left;
			height = 1472+rect->top;
			hblank = 0x245;
			vblank = 0x08;

		ret |= reg_write(client, MT9P031_ROW_ADDRESS_MODE, 0x0011);
		ret |= reg_write(client, MT9P031_COLUMN_ADDRESS_MODE, 0x0011);
	}
else {
		left = (2592-1920)/2+16-rect->left;
		top = (1944-1080)/2+54-rect->top;
		width = 1920+rect->left;
		height = 1080+rect->top;
		hblank = 0x01AE;
		vblank = 0x0;

		ret |= reg_write(client, MT9P031_ROW_ADDRESS_MODE, 0x0000);
		ret |= reg_write(client, MT9P031_COLUMN_ADDRESS_MODE, 0x0000);
	}

	ret |= reg_write(client, MT9P031_HORIZONTAL_BLANKING, hblank);
	ret |= reg_write(client, MT9P031_VERTICAL_BLANKING, vblank);
	 if (ret < 0)
		return ret;


#if 0
	if (yskip != mt9p031->yskip || xskip != mt9p031->xskip) {
		/* Binning, skipping */
		if (ret >= 0)
			ret = reg_write(client, MT9P031_COLUMN_ADDRESS_MODE,
					((xbin - 1) << 4) | (xskip - 1));
		if (ret >= 0)
			ret = reg_write(client, MT9P031_ROW_ADDRESS_MODE,
					((ybin - 1) << 4) | (yskip - 1));
	}
#endif
	v4l2_dbg(1, debug, sd, "new physical left %u, top %u\n", left, top);

	/* The caller provides a supported format, as guaranteed by
	 * icd->try_fmt_cap(), soc_camera_s_crop() and soc_camera_cropcap() */
	if (ret >= 0)
		ret = reg_write(client, MT9P031_COLUMN_START, left);
	if (ret >= 0)
		ret = reg_write(client, MT9P031_ROW_START, top);
	if (ret >= 0)
		ret = reg_write(client, MT9P031_WINDOW_WIDTH, width - 1);
	if (ret >= 0)
		ret = reg_write(client, MT9P031_WINDOW_HEIGHT, height - 1);

#if 0
	if (ret >= 0 && mt9p031->autoexposure) {
		ret = set_shutter(sd, exposure_time);
		if (ret >= 0) {
			const u32 shutter_max = MT9P031_MAX_HEIGHT + vblank;
			const struct v4l2_queryctrl *qctrl =
				mt9p031_find_qctrl(V4L2_CID_EXPOSURE);
			mt9p031->exposure = (shutter_max / 2 + (height +
					 mt9p031->y_skip_top + vblank - 1) *
					 (qctrl->maximum - qctrl->minimum)) /
				shutter_max + qctrl->minimum;
		}
	}
#endif

	/* Re-enable register update, commit all changes */
	if (ret >= 0) {
		ret = reg_clear(client, MT9P031_OUTPUT_CONTROL, 1);
		/* update the values */
		mt9p031->width	= rect->width,
		mt9p031->height	= rect->height,
		mt9p031->x_current = rect->left;
		mt9p031->y_current = rect->top;
	}
	return ret < 0 ? ret : 0;
}

static int mt9p031_get_fmt(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct mt9p031 *mt9p031 = to_mt9p031(sd);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width		= mt9p031->width;
	pix->height		= mt9p031->height;
	pix->pixelformat	= V4L2_PIX_FMT_SGRBG10;
	pix->field		= V4L2_FIELD_NONE;
	pix->colorspace		= V4L2_COLORSPACE_SRGB;

	return 0;
}
static int mt9p031_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_format *f)
{
	struct mt9p031 *mt9p031 = to_mt9p031(sd);
	int ret;
	u16 xskip, yskip;
	struct v4l2_rect rect = {
		.left	= mt9p031->x_current,
		.top	= mt9p031->y_current,
		.width	= f->fmt.pix.width,
		.height	= f->fmt.pix.height,
	};

	/*
	 * try_fmt has put rectangle within limits.
	 * S_FMT - use binning and skipping for scaling, recalculate
	 * limits, used for cropping
	 */
	/* Is this more optimal than just a division? */
	for (xskip = 8; xskip > 1; xskip--)
		if (rect.width * xskip <= MT9P031_MAX_WIDTH)
			break;

	for (yskip = 8; yskip > 1; yskip--)
		if (rect.height * yskip <= MT9P031_MAX_HEIGHT)
			break;

	recalculate_limits(mt9p031, xskip, yskip);

	ret = mt9p031_set_params(sd, &rect, xskip, yskip);
	if (!ret) {
		mt9p031->xskip = xskip;
		mt9p031->yskip = yskip;
	}
	return ret;
}

static int mt9p031_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->height < MT9P031_MIN_HEIGHT)
		pix->height = MT9P031_MIN_HEIGHT;
	if (pix->height > MT9P031_MAX_HEIGHT)
		pix->height = MT9P031_MAX_HEIGHT;
	if (pix->width < MT9P031_MIN_WIDTH)
		pix->width = MT9P031_MIN_WIDTH;
	if (pix->width > MT9P031_MAX_WIDTH)
		pix->width = MT9P031_MAX_WIDTH;

	pix->width &= ~0x01; /* has to be even */
	pix->height &= ~0x01; /* has to be even */
	return 0;
}

static int mt9p031_get_chip_id(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct mt9p031 *mt9p031 = to_mt9p031(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);;

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= mt9p031->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9p031_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);;
	struct mt9p031 *mt9p031 = to_mt9p031(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->val = reg_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int mt9p031_set_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (reg_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif


static int mt9p031_get_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9p031_set_control(struct v4l2_subdev *, struct v4l2_control *);
static int mt9p031_queryctrl(struct v4l2_subdev *, struct v4l2_queryctrl *);
static int mt9p031_querystd(struct v4l2_subdev *sd, v4l2_std_id *id);
static int mt9p031_set_standard(struct v4l2_subdev *sd, v4l2_std_id id);

static const struct v4l2_subdev_core_ops mt9p031_core_ops = {
	.g_chip_ident = mt9p031_get_chip_id,
	.init = mt9p031_init,
	.queryctrl = mt9p031_queryctrl,
	.g_ctrl	= mt9p031_get_control,
	.s_ctrl	= mt9p031_set_control,
    .s_std     =  mt9p031_set_standard,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.get_register = mt9p031_get_register,
	.set_register = mt9p031_set_register,
#endif
};

static const struct v4l2_subdev_video_ops mt9p031_video_ops = {
	.s_fmt = mt9p031_set_fmt,
    .g_fmt = mt9p031_get_fmt,
	.try_fmt = mt9p031_try_fmt,
	.querystd = mt9p031_querystd,
	.s_stream = mt9p031_s_stream,
};

static const struct v4l2_subdev_ops mt9p031_ops = {
	.core = &mt9p031_core_ops,
	.video = &mt9p031_video_ops,
};

static int mt9p031_queryctrl(struct v4l2_subdev *sd,
			    struct v4l2_queryctrl *qctrl)
{
	const struct v4l2_queryctrl *temp_qctrl;

	temp_qctrl = mt9p031_find_qctrl(qctrl->id);
	if (!temp_qctrl) {
		v4l2_err(sd, "control id %d not supported", qctrl->id);
		return -EINVAL;
	}
	memcpy(qctrl, temp_qctrl, sizeof(*qctrl));
	return 0;
}

static int mt9p031_get_control(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(sd);
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		data = reg_read(client, MT9P031_READ_MODE_2);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & 0x8000);
		break;
	case V4L2_CID_HFLIP:
		data = reg_read(client, MT9P031_READ_MODE_2);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & 0x4000);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ctrl->value = mt9p031->autoexposure;
		break;
	}
	return 0;
}

static int mt9p031_set_control(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct mt9p031 *mt9p031 = to_mt9p031(sd);
	const struct v4l2_queryctrl *qctrl = NULL;
	int data;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (NULL == ctrl)
		return -EINVAL;

	qctrl = mt9p031_find_qctrl(ctrl->id);
	if (!qctrl) {
		v4l2_err(sd, "control id %d not supported", ctrl->id);
		return -EINVAL;
	}

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		if (ctrl->value)
			data = reg_set(client, MT9P031_READ_MODE_2, 0x8000);
		else
			data = reg_clear(client, MT9P031_READ_MODE_2, 0x8000);
		if (data < 0)
			return -EIO;
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->value)
			data = reg_set(client, MT9P031_READ_MODE_2, 0x4000);
		else
			data = reg_clear(client, MT9P031_READ_MODE_2, 0x4000);
		if (data < 0)
			return -EIO;
		break;
	case V4L2_CID_GAIN:
		if (ctrl->value > qctrl->maximum \
			|| ctrl->value < qctrl->minimum)
			return -EINVAL;
		/* See Datasheet Table 7, Gain settings. */
		if (ctrl->value <= qctrl->default_value) {
			/* Pack it into 0..1 step 0.125, register values 0..8 */
			unsigned long range = qctrl->default_value \
						- qctrl->minimum;
			data = ((ctrl->value - qctrl->minimum) * 8 + range / 2)\
				/ range;

			v4l2_dbg(1, debug, sd, "Setting gain %d\n", data);
			data = reg_write(client, MT9P031_GLOBAL_GAIN, data);
			if (data < 0)
				return -EIO;
		} else {
			/* Pack it into 1.125..128 variable step,
				register values 9..0x7860 */
			/* assume qctrl->maximum - qctrl->default_value-1 > 0*/
			unsigned long range = qctrl->maximum \
						- qctrl->default_value - 1;
			/* calculated gain: map 65..127 to 9..1024 step 0.125 */
			unsigned long gain = ((ctrl->value \
						- qctrl->default_value - 1) *
					       1015 + range / 2) / range + 9;

			if (gain <= 32)
				/* calculated gain 9..32 -> 9..32 */
				data = gain;
			else if (gain <= 64)
				/* calculated gain 33..64 -> 0x51..0x60 */
				data = ((gain - 32) * 16 + 16) / 32 + 80;
			else
				/*
				 * calculated gain 65..1024 -> (1..120) << 8 +
				 * 0x60
				 */
				data = (((gain - 64 + 7) * 32) & 0xff00) | 0x60;

			v4l2_dbg(1, debug, sd, "Setting gain from 0x%x to"
				 "0x%x\n",
				 reg_read(client, MT9P031_GLOBAL_GAIN), data);

			data = reg_write(client, MT9P031_GLOBAL_GAIN, data);
			if (data < 0)
				return -EIO;
		}

		/* Success */
		mt9p031->gain = ctrl->value;
		break;
	case V4L2_CID_EXPOSURE:

printk(KERN_INFO"-----Exposure time = %x", ctrl->value);

		/* mt9p031 has maximum == default */
		if (ctrl->value > qctrl->maximum ||
		    ctrl->value < qctrl->minimum)
			return -EINVAL;
		else {
		   u32 shutter = ctrl->value;

			if (set_shutter(sd, shutter) < 0)
				return -EIO;
			mt9p031->exposure = ctrl->value;
			mt9p031->autoexposure = 0;
		}
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		if (ctrl->value) {
			const u16 vblank = MT9P031_VERTICAL_BLANK;
			const u32 shutter_max = MT9P031_MAX_HEIGHT + vblank;
			if (set_shutter(sd, mt9p031->height +
					mt9p031->y_skip_top + vblank) < 0)
				return -EIO;

			qctrl = mt9p031_find_qctrl(V4L2_CID_EXPOSURE);
			mt9p031->exposure =
				(shutter_max / 2 + (mt9p031->height +
				mt9p031->y_skip_top + vblank - 1) *
				(qctrl->maximum - qctrl->minimum)) /
				shutter_max + qctrl->minimum;
			mt9p031->autoexposure = 1;
		} else
			mt9p031->autoexposure = 0;
		break;
	}
	return 0;
}

/* Function querystd not supported by mt9p031 */
static int mt9p031_querystd(struct v4l2_subdev *sd, v4l2_std_id *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(sd);

	*id = V4L2_STD_MT9P031_STD_ALL;

	return 0;
}

/* Function set not supported by mt9p031 */
static int mt9p031_set_standard(struct v4l2_subdev *sd, v4l2_std_id id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(sd);

    mt9p031_cur_std = id;

	return 0;
}
/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int mt9p031_detect(struct i2c_client *client, int *model)
{
	s32 data;

	/* Enable the chip */
	data = reg_write(client, MT9P031_CHIP_ENABLE, 1);
	dev_dbg(&client->dev, "write: %d\n", data);

	/* Read out the chip version register */
	data = reg_read(client, MT9P031_CHIP_VERSION);

	switch (data) {
	case 0x1801:
		*model = V4L2_IDENT_MT9P031;
		break;
	default:
		dev_err(&client->dev,
			"No MT9P031 chip detected, register read %x\n", data);
		return -ENODEV;
	}

	dev_info(&client->dev, "Detected a MT9P031 chip ID %x\n", data);
	return 0;
}

static int mt9p031_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct mt9p031 *mt9p031;
	struct v4l2_subdev *sd;
	int pclk_pol;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&client->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "No platform data!!\n");
		return -ENODEV;
	}

	pclk_pol = (int)client->dev.platform_data;

	mt9p031 = kzalloc(sizeof(struct mt9p031), GFP_KERNEL);
	if (!mt9p031)
		return -ENOMEM;

	ret = mt9p031_detect(client, &mt9p031->model);
	if (ret)
		goto clean;

	mt9p031->x_min		= MT9P031_COLUMN_SKIP;
	mt9p031->y_min		= MT9P031_ROW_SKIP;
	mt9p031->width		= MT9P031_DEFAULT_WIDTH;
	mt9p031->height		= MT9P031_DEFAULT_WIDTH;
	mt9p031->x_current	= mt9p031->x_min;
	mt9p031->y_current	= mt9p031->y_min;
	mt9p031->width_min	= MT9P031_MIN_WIDTH;
	mt9p031->width_max	= MT9P031_MAX_WIDTH;
	mt9p031->height_min	= MT9P031_MIN_HEIGHT;
	mt9p031->height_max	= MT9P031_MAX_HEIGHT;
	mt9p031->y_skip_top	= 10;
	mt9p031->autoexposure = 1;
	mt9p031->xskip = 1;
	mt9p031->yskip = 1;

	/* Register with V4L2 layer as slave device */
	sd = &mt9p031->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9p031_ops);
	if (!pclk_pol)
		reg_clear(v4l2_get_subdevdata(sd),
			  MT9P031_PIXEL_CLOCK_CONTROL, 0x8000);
	else
		reg_set(v4l2_get_subdevdata(sd),
			MT9P031_PIXEL_CLOCK_CONTROL, 0x8000);

	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);
	return 0;

clean:
	kfree(mt9p031);
	return ret;
}

static int mt9p031_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9p031 *mt9p031 = to_mt9p031(sd);

	v4l2_device_unregister_subdev(sd);

	kfree(mt9p031);
	return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
	{ "mt9p031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

static struct i2c_driver mt9p031_i2c_driver = {
	.driver = {
		.name = "mt9p031",
	},
	.probe		= mt9p031_probe,
	.remove		= mt9p031_remove,
	.id_table	= mt9p031_id,
};

static int __init mt9p031_mod_init(void)
{
	return i2c_add_driver(&mt9p031_i2c_driver);
}

static void __exit mt9p031_mod_exit(void)
{
	i2c_del_driver(&mt9p031_i2c_driver);
}

module_init(mt9p031_mod_init);
module_exit(mt9p031_mod_exit);

MODULE_DESCRIPTION("Micron MT9P031 Camera driver");
MODULE_AUTHOR("Guennadi Liakhovetski <lg@denx.de>");
MODULE_LICENSE("GPL v2");
