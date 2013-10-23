/*
 * Driver for OV971x CMOS Image Sensor from Omnivision, for TI Davinci platform
 *
 * Copyright (C) 2011, Raffaele Recalcati, Davide Bonfanti,
 * bticino s.p.a. <raffaele.recalcati@bticino.it> <davide.bonfanti@bticino.it>
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
#include <media/ov971x.h>

/* ov9712 i2c address 0x30
 * The platform has to define i2c_board_info
 * and call i2c_register_board_info() */

#define OV971x_GAIN			0x00
#define OV971x_YAVG			0x2F
#define OV971x_AECH			0x16
#define OV971x_AECL			0x10
#define OV971x_WPT			0x24
#define OV971x_BPT			0x25

#define OV971x_REG03			0x03
#define OV971x_REG03_VSTART_LSB_SHIFT	0
#define OV971x_REG03_VSTART_LSB_MASK	(3 << OV971x_REG03_VSTART_LSB_SHIFT)
#define OV971x_REG03_AVSIZE_LSB_SHIFT	2
#define OV971x_REG03_AVSIZE_LSB_MASK	(3 << OV971x_REG03_AVSIZE_LSB_SHIFT)

#define OV971x_REG32			0x32
#define OV971x_REG32_HSTART_LSB_SHIFT	0
#define OV971x_REG32_HSTART_LSB_MASK	(7 << OV971x_REG32_HSTART_LSB_SHIFT)
#define OV971x_REG32_HSIZE_LSB_SHIFT	3
#define OV971x_REG32_HSIZE_LSB_MASK	(7 << OV971x_REG32_HSIZE_LSB_SHIFT)

#define OV971x_REG04			0x04
#define OV971x_REG04_MIRROR		BIT(7)
#define OV971x_REG04_FLIP		BIT(6)

#define OV971x_VSTART_MSB		0x19
#define OV971x_AVSIZE_MSB		0x1A
#define OV971x_AHSIZE_MSB		0x18
#define OV971x_HSTART_MSB		0x17

#define OV971X_PIDH			0x0A
#define OV971X_PIDL			0x0B

#define OV971x_COM2			0x09
#define OV971x_COM2_SLEEP		BIT(4)

#define OV971x_COM10			0x15
#define OV971x_COM10_VSYNC_POL		BIT(1)
#define OV971x_COM10_HSYNC_POL		BIT(0)

#define OV971X_VSTART			0x19

#define OV971x_DVP_CTRL_00		0xC2
#define OV971x_DVP_CTRL_00_PCLK_POL	BIT(2)

#define OV971x_REG4E			0x4E
#define OV971x_REG4F			0x4F
#define OV971x_REG50			0x50
#define OV971x_REG51			0x51

#define OV971x_REG5C			0x5C
#define OV971x_REG5C_PREDIV_SHIFT	5
#define OV971x_REG5C_PREDIV_MASK	(3 << OV971x_REG5C_PREDIV_SHIFT)
#define OV971x_REG5C_PLL_MULT_SHIFT	0
#define OV971x_REG5C_PLL_MULT_MASK	(0x1F << OV971x_REG5C_PLL_MULT_SHIFT)

#define OV971x_REG5D			0x5D
#define OV971x_REG5D_PLL_SEL_SHIFT	2
#define OV971x_REG5D_PLL_SEL_MASK	(3 << OV971x_REG5D_PLL_SEL_SHIFT)
#define OV971x_REG5D_OUT_DRIVE_SHIFT	4
#define OV971x_REG5D_OUT_DRIVE_MASK	(3 << OV971x_REG5D_OUT_DRIVE_SHIFT)

#define OV971x_CLK			0x11
#define OV971x_CLK_CLK_DIV_SHIFT	0
#define OV971x_CLK_CLK_DIV_MASK		(0x3F << OV971x_CLK_CLK_DIV_SHIFT)

#define OV971x_REG57			0x57
#define OV971x_REG57_VSIZE_LSB_SHIFT	0
#define OV971x_REG57_VSIZE_LSB_MASK	(3 << OV971x_REG57_VSIZE_LSB_SHIFT)
#define OV971x_REG57_HSIZE_LSB_SHIFT	2
#define OV971x_REG57_HSIZE_LSB_MASK	(7 << OV971x_REG57_HSIZE_LSB_SHIFT)

#define OV971x_VSIZE_MSB		0x58
#define OV971x_HSIZE_MSB		0x59

#define OV971x_DVP_CTRL_08		0xCA
#define OV971x_DVP_CTRL_08_VSYNC_MSB_SHIFT	6
#define OV971x_DVP_CTRL_08_VSYNC_MSB_MASK	(3 << OV971x_DVP_CTRL_08_VSYNC_MSB_SHIFT)

#define OV971x_VSYNC_LSB		0xC9
#define OV971x_HSYNC_START		0x30
#define OV971x_HSYNC_STOP		0x31

#define OV971x_BANDING_STEP_L		0x49
#define OV971x_BANDING_STEP_H		0x4A

/* Constants defining sensor capabilities */
#define OV971x_MAX_HEIGHT		720
#define OV971x_MAX_WIDTH		1280
#define OV971x_MIN_HEIGHT		100
#define OV971x_MIN_WIDTH		160
#define OV971x_HORIZONTAL_BLANK		0
#define OV971x_VERTICAL_BLANK		0
#define OV971x_COLUMN_SKIP		0
#define OV971x_ROW_SKIP			0
#define OV971x_DEFAULT_WIDTH		1280
#define OV971x_DEFAULT_HEIGHT		720
#define OV971x_BUS_PARAM	(SOCAM_PCLK_SAMPLE_RISING |	\
	SOCAM_PCLK_SAMPLE_FALLING | SOCAM_HSYNC_ACTIVE_HIGH |	\
	SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_DATA_ACTIVE_HIGH |	\
	SOCAM_MASTER | SOCAM_DATAWIDTH_10)

#define VPT 0x92

/* Debug functions */
static int debug = 0;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static const struct v4l2_fmtdesc ov971x_formats[] = {
	{
		.index = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.description = "Bayer (sRGB) 10 bit",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
	},
};
static const unsigned int ov971x_num_formats = ARRAY_SIZE(ov971x_formats);

static const struct v4l2_queryctrl ov971x_controls[] = {
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
		.default_value	= 127,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 30,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 96,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_PAN_RELATIVE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Pan, Relative",
		.minimum	= 0,
		.maximum	= 1280,
		.step		= 1,
		.default_value	= 10,
	}, {
		.id		= V4L2_CID_TILT_RELATIVE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Tilt, Relative",
		.minimum	= 0,
		.maximum	= 400,
		.step		= 1,
		.default_value	= 10,
	}, {
		.id		= V4L2_CID_PAN_RESET,
		.type		= V4L2_CTRL_TYPE_BUTTON,
		.name		= "Pan, Reset",
		.minimum	= 0,
		.maximum	= 0,
		.step		= 0,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_TILT_RESET,
		.type		= V4L2_CTRL_TYPE_BUTTON,
		.name		= "Tilt, Reset",
		.minimum	= 0,
		.maximum	= 0,
		.step		= 0,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_PAN_ABSOLUTE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Pan, Absolute",
		.minimum	= 320,
		.maximum	= 960,
		.step		= 1,
		.default_value	= 640,
	}, {
		.id		= V4L2_CID_TILT_ABSOLUTE,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Tilt, Absolute",
		.minimum	= 200,
		.maximum	= 600,
		.step		= 1,
		.default_value	= 400,
	}, {
		.id		= V4L2_CID_POWER_LINE_FREQUENCY,
		.type		= V4L2_CTRL_TYPE_MENU,
		.name		= "Light frequency filter",
		.minimum	= 0,
		.maximum	= 2,   /* 0: 0, 1: 50Hz, 2:60Hz */
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_EXPOSURE_WINDOWS_WEIGHT,
		.type		= V4L2_CTRL_TYPE_MENU,
		.minimum	= 0,
		.maximum	= 2,
		.step		= 1,
		.default_value	= 1,
		.name		= "Exposure Window weights"
	}
};
static const unsigned int ov971x_num_controls = ARRAY_SIZE(ov971x_controls);

struct ov971x {
	struct v4l2_subdev sd;
	int model;	/* V4L2_IDENT_OV971x* codes from v4l2-chip-ident.h */
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
	unsigned short current_height;
	unsigned short y_skip_top;      /* Lines to skip at the top */
	unsigned short gain;
	unsigned short exposure;
	u32 xclk_frequency;		/* freq sent to CMOS sensor */
	enum v4l2_power_line_frequency pwrline_fr;
};

static inline struct ov971x *to_ov971x(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov971x, sd);
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
	s32 data;

	data = i2c_smbus_read_byte_data(client, reg);
	return data;
}

static int reg_read_word(struct i2c_client *client, const u8 reg)
{
	s32 data;

	data = i2c_smbus_read_word_data(client, reg);
	return data < 0 ? data : swab16(data);
}

static int reg_write(struct i2c_client *client, const u8 reg,
		     const u8 data)
{
	return i2c_smbus_write_byte_data(client, reg, data);
}

static int reg_set(struct i2c_client *client, const u8 reg,
		   const u8 data)
{
	int ret;

	ret = reg_read(client, reg);
	if (ret < 0)
		return ret;
	return reg_write(client, reg, ret | data);
}

static int reg_clear(struct i2c_client *client, const u8 reg,
		     const u8 data)
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

	ret = reg_write(client, OV971x_WPT, (data < 256) ? data : 255);
	ret |= reg_write(client, OV971x_BPT, (data > 32) ? data - 16 : 10);
	return ret;
}

/*
static int get_shutter(struct v4l2_subdev *sd, u32 *data)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return reg_read(client, OV971x_WPT);
}
*/

static int ov971x_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Disable chip output, synchronous option update */
	if (reg_clear(client, OV971x_COM2, OV971x_COM2_SLEEP) < 0)
		return -EIO;
	return 0;
}

static int ov971x_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (enable) {
		if (reg_clear(client, OV971x_COM2, OV971x_COM2_SLEEP) < 0)
			return -EIO;
	} else {
		if (reg_set(client, OV971x_COM2, OV971x_COM2_SLEEP) < 0)
			return -EIO;
	}
	return 0;
}

const struct v4l2_queryctrl *ov971x_find_qctrl(u32 id)
{
	int i;

	for (i = 0; i < ov971x_num_controls; i++) {
		if (ov971x_controls[i].id == id)
			return &ov971x_controls[i];
	}
	return NULL;
}

const struct v4l2_queryctrl *ov971x_find_next_qctrl(u32 id)
{
	int i;

	for (i = 0; i < ov971x_num_controls; i++) {
		if (ov971x_controls[i].id > id)
			return &ov971x_controls[i];
	}
	return NULL;
}

typedef unsigned int pnt[2];

struct regs {
	unsigned int h_res;
	unsigned int v_res;
	unsigned int (*registers)[2];
	unsigned int size_regs;
	v4l2_std_id fmt_id;
};

static pnt reg_1280x720[] = {
	{0x12, 0x00},
	{0x09, 0x00},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x00},
	{0x3b, 0x00},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0xa2},
	{0x19, 0x01},
	{0x1a, 0xca},
	{0x03, 0x0a},
	{0x32, 0x07},
	{0x98, 0x00},
	{0x99, 0x00},
	{0x9a, 0x00},
	{0x57, 0x00},
	{0x58, 0xc8},
	{0x59, 0xa0},
	{0x4c, 0x13},
	{0x4b, 0x36},
	{0x3d, 0x3c},
	{0x3e, 0x03},
	{0xbd, 0xa0},
	{0xbe, 0xc8},
	{0x4e, 0x14},
	{0x4f, 0xFF},
	{0x50, 0xFF},
	{0x51, 0x41},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x13, 0x8D},
	{0x10, 0xB0},
	{0x14, 0x80},
	{0x41, 0x82},
	{0x22, 0x03},
};

static unsigned int reg_640x480[][2] = {
	{0x12, 0x80},
	{0x09, 0x10},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x00},
	{0x3b, 0x00},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0xA0}, /* 0xA2 */
	{0x19, 0x00}, /* 0x01 */
	{0x1a, 0x7a}, /* 0xCA */
	{0x03, 0x00}, /* 0x0A */
	{0x32, 0x07},
	{0x98, 0x00}, /* 0x40 */
	{0x99, 0x00}, /* 0xA0 */
	{0x9a, 0x00}, /* 0x01 */
	{0x57, 0x00},
	{0x58, 0x7a}, /* 0x78 */
	{0x59, 0xA0}, /* 0x50 */
	{0x4c, 0x13},
	{0x4b, 0x36},
	{0x3d, 0x3c},
	{0x3e, 0x03},
	{0xbd, 0x50},
	{0xbe, 0x78},
	{0x4e, 0x55},
	{0x4f, 0x55},
	{0x50, 0x55},
	{0x51, 0x55},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x13, 0xA5},
	{0x14, 0x40},
	{0x4a, 0x00},
	{0x49, 0xce},
	{0x22, 0x03},
	{0x09, 0x00},
};

static unsigned int reg_640x400[][2] = {
	{0x12, 0x80},
	{0x09, 0x10},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x40},
	{0x3b, 0x01},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0xA2},
	{0x19, 0x01},
	{0x1a, 0x65}, /* 0x64 */
	{0x03, 0x00}, /* 0x02 */
	{0x32, 0x07},
	{0x98, 0x00},
	{0x99, 0x00},
	{0x9a, 0x00},
	{0x57, 0x00},
	{0x58, 0x65}, /* 0x64 */
	{0x59, 0x50},
	{0x4b, 0x9a},
	{0x4c, 0x09},
	{0x3d, 0x9e},
	{0x3e, 0x01},
	{0xbd, 0x50},
	{0xbe, 0x64},
	{0x2c, 0x60},
	{0x23, 0x10},
	{0x4e, 0x55},
	{0x4f, 0x55},
	{0x50, 0x55},
	{0x51, 0x55},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x13, 0xA5},
	{0x14, 0x40},
	{0x4a, 0x00},
	{0x49, 0x67},
	{0x22, 0x03},
	{0x09, 0x00},
};

static unsigned int reg_320x240[][2] = {
	{0x12, 0x80},
	{0x09, 0x10},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x40},
	{0x3b, 0x01},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0x50}, /* 0xA2 */
	{0x19, 0x00}, /* 0x01 */
	{0x1a, 0x3d}, /* 0x64 */
	{0x03, 0x00}, /* 0x02 */
	{0x32, 0x07},
	{0x98, 0x00}, /* 0x40 */
	{0x99, 0x00}, /* 0x50 */
	{0x9a, 0x00}, /* 0x01 */
	{0x57, 0x00},
	{0x58, 0x3d}, /* 0x3c */
	{0x59, 0x50}, /* 0x28 */
	{0x4b, 0x9a},
	{0x4c, 0x09},
	{0x3d, 0x9e},
	{0x3e, 0x01},
	{0xbd, 0x28},
	{0xbe, 0x3c},
	{0x2c, 0x60},
	{0x23, 0x10},
	{0x4e, 0x55},
	{0x4f, 0x55},
	{0x50, 0x55},
	{0x51, 0x55},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x13, 0xA5},
	{0x14, 0x40},
	{0x4a, 0x00},
	{0x49, 0x67},
	{0x22, 0x03},
	{0x09, 0x00},
};

static unsigned int reg_352x288[][2] = {
	{0x12, 0x80},
	{0x09, 0x10},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x40},
	{0x3b, 0x01},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0xA2},
	{0x19, 0x01},
	{0x1a, 0x49}, /* 0x64 */
	{0x03, 0x02},
	{0x32, 0x07},
	{0x98, 0x20},
	{0x99, 0x38},
	{0x9a, 0x01},
	{0x57, 0x00},
	{0x58, 0x49}, /* 0x48 */
	{0x59, 0x2C},
	{0x4b, 0x9a},
	{0x4c, 0x09},
	{0x3d, 0x9e},
	{0x3e, 0x01},
	{0xbd, 0x2c},
	{0xbe, 0x48},
	{0x2c, 0x60},
	{0x23, 0x10},
	{0x4e, 0x55},
	{0x4f, 0x55},
	{0x50, 0x55},
	{0x51, 0x55},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x13, 0xA5},
	{0x14, 0x40},
	{0x4a, 0x00},
	{0x49, 0x67},
	{0x22, 0x03},
	{0x09, 0x00},
};

static unsigned int reg_176x144[][2] = {
	{0x12, 0x80},
	{0x09, 0x10},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x40},
	{0x3b, 0x01},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0xA2},
	{0x19, 0x01},
	{0x1a, 0x25}, /* 0x64 */
	{0x03, 0x02},
	{0x32, 0x07},
	{0x98, 0xD0},
	{0x99, 0x80},
	{0x9a, 0x01},
	{0x57, 0x00},
	{0x58, 0x25}, /* 0x24 */
	{0x59, 0x16},
	{0x4b, 0x9a},
	{0x4c, 0x09},
	{0x3d, 0x9e},
	{0x3e, 0x01},
	{0xbd, 0x16},
	{0xbe, 0x24},
	{0x2c, 0x60},
	{0x23, 0x10},
	{0x4e, 0x55},
	{0x4f, 0x55},
	{0x50, 0x55},
	{0x51, 0x55},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x13, 0xA5},
	{0x14, 0x40},
	{0x4a, 0x00},
	{0x49, 0x67},
	{0x22, 0x03},
	{0x09, 0x00},
};

static unsigned int reg_160x120[][2] = {
	{0x12, 0x80},
	{0x09, 0x10},
	{0x1e, 0x07},
	{0x5f, 0x18},
	{0x69, 0x04},
	{0x65, 0x2a},
	{0x68, 0x0a},
	{0x39, 0x28},
	{0x4d, 0x90},
	{0xc1, 0x80},
	{0x0c, 0x30},
	{0x6d, 0x02},
	{0x96, 0xf1},
	{0xbc, 0x68},
	{0x12, 0x40},
	{0x3b, 0x01},
	{0x97, 0x80},
	{0x17, 0x25},
	{0x18, 0xA2},
	{0x19, 0x01},
	{0x1a, 0x1f}, /* 0x64 */
	{0x03, 0x02},
	{0x32, 0x07},
	{0x98, 0xE0},
	{0x99, 0x8C},
	{0x9a, 0x01},
	{0x57, 0x00},
	{0x58, 0x1f}, /* 0x1e */
	{0x59, 0x14},
	{0x4b, 0x9a},
	{0x4c, 0x09},
	{0x3d, 0x9e},
	{0x3e, 0x01},
	{0xbd, 0x14},
	{0xbe, 0x1E},
	{0x2c, 0x60},
	{0x23, 0x10},
	{0x4e, 0x55},
	{0x4f, 0x55},
	{0x50, 0x55},
	{0x51, 0x55},
	{0x26, VPT},
	{0x2a, 0x98},
	{0x2b, 0x06},
	{0x2d, 0x00},
	{0x2e, 0x00},
	{0x14, 0x40},
	{0x4a, 0x00},
	{0x49, 0x67},
	{0x22, 0x03},
	{0x09, 0x00},
};

static struct regs formats[] = {
	{
		.h_res = 1280,
		.v_res = 720,
		.registers = reg_1280x720,
		.size_regs = ARRAY_SIZE(reg_1280x720),
		.fmt_id = V4L2_STD_720P_30,
	}, {
		.h_res = 640,
		.v_res = 480,
		.registers = reg_640x480,
		.size_regs = ARRAY_SIZE(reg_640x480),
		.fmt_id = V4L2_STD_640x400_30,
	}, {
		.h_res = 640,
		.v_res = 400,
		.registers = reg_640x400,
		.size_regs = ARRAY_SIZE(reg_640x400),
		.fmt_id = V4L2_STD_640x400_30,
	}, {
		.h_res = 320,
		.v_res = 240,
		.registers = reg_320x240,
		.size_regs = ARRAY_SIZE(reg_320x240),
		.fmt_id = V4L2_STD_320x240_30,
	}, {
		.h_res = 352,
		.v_res = 288,
		.registers = reg_352x288,
		.size_regs = ARRAY_SIZE(reg_352x288),
		.fmt_id = V4L2_STD_352x288_30,
	}, {
		.h_res = 176,
		.v_res = 144,
		.registers = reg_176x144,
		.size_regs = ARRAY_SIZE(reg_176x144),
		.fmt_id = V4L2_STD_176x144_30,
	}, {
		.h_res = 160,
		.v_res = 120,
		.registers = reg_160x120,
		.size_regs = ARRAY_SIZE(reg_160x120),
		.fmt_id = V4L2_STD_160x120_30,
	}, {
		.h_res = 640,
		.v_res = 480,
		.registers = reg_640x480,
		.size_regs = ARRAY_SIZE(reg_640x480),
		.fmt_id = V4L2_STD_640x480_24,
	}, {
		.h_res = 1280,
		.v_res = 720,
		.registers = reg_1280x720,
		.size_regs = ARRAY_SIZE(reg_1280x720),
		.fmt_id = V4L2_STD_720P_24,
	}
};

void ov971x_set_frequency(int freq_in, int freq_out, struct v4l2_subdev *sd)
{
	int pll_prediv, pll_mult, pll_div, sysclk_div, ret, cnt, tmp;
	int clk1, clk2;

	tmp = freq_out<<4;
	freq_out = tmp - (freq_out<<1);
	freq_out *= 1000;
	freq_out *= 100;

	if (freq_in >= 24000000)
		pll_prediv = 2;
	else if (freq_in >= 6000000)
		pll_prediv = 1;
	else
		pll_prediv = 0;
	clk1 = freq_in >> pll_prediv;
	pll_prediv <<= 1;

	clk2 = 80 * 1000 * 1000;
	/* setup PLL settings */
	if ((freq_out<<4) < clk2)
		sysclk_div = 2;
	else if ((freq_out<<3) < clk2)
		sysclk_div = 1;
	else
		sysclk_div = 0;
	for (pll_div = 0, tmp = 0; tmp < clk2; pll_div++) {
		tmp = 0;
		for (cnt = 0; cnt <= pll_div; cnt++)
			tmp += (freq_out<<sysclk_div);
	}
	sysclk_div <<= 1;
	clk2 = freq_out * pll_div;
	if (sysclk_div)
		clk2 *= sysclk_div;

	for (pll_mult = 1, tmp = 0; tmp < clk2; pll_mult++) {
		tmp = 0;
		for (cnt = 0; cnt <= pll_mult; cnt++)
			tmp += clk1;
	}

	ret = reg_read(v4l2_get_subdevdata(sd), OV971x_REG5C);
	ret &= ~OV971x_REG5C_PREDIV_MASK;
	if (pll_prediv == 2)
		ret |= 2 << OV971x_REG5C_PREDIV_SHIFT;
	else if (pll_prediv == 4)
		ret |= 3 << OV971x_REG5C_PREDIV_SHIFT;
	ret &= ~OV971x_REG5C_PLL_MULT_MASK;
	ret |= 32 - pll_mult;
	reg_write(v4l2_get_subdevdata(sd), OV971x_REG5C, ret);

	ret = reg_read(v4l2_get_subdevdata(sd), OV971x_REG5D);
	ret &= ~OV971x_REG5D_PLL_SEL_MASK;
	ret |= (pll_div - 1) << OV971x_REG5D_PLL_SEL_SHIFT;
	reg_write(v4l2_get_subdevdata(sd), OV971x_REG5D, ret);

	ret = reg_read(v4l2_get_subdevdata(sd), OV971x_CLK);
	ret &= ~OV971x_CLK_CLK_DIV_MASK;
	ret |= (sysclk_div / 2 - 1) << OV971x_CLK_CLK_DIV_SHIFT;
	reg_write(v4l2_get_subdevdata(sd), OV971x_CLK, ret);
}

static int ov971x_set_params(struct v4l2_subdev *sd,
			      struct v4l2_rect *rect)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int cnt1, cnt2;

	v4l2_dbg(1, debug, sd, "width %u, height %u\n",
			rect->width, rect->height);

	/* The caller provides a supported format, as guaranteed by
	 * icd->try_fmt_cap(), soc_camera_s_crop() and soc_camera_cropcap() */
	for (cnt1 = 0; cnt1 < ARRAY_SIZE(formats); cnt1++) {
		if ((formats[cnt1].h_res == rect->width) &&
			(formats[cnt1].v_res == rect->height)) {
			for (cnt2 = 0; cnt2 < formats[cnt1].size_regs; cnt2++) {
				reg_write(client,
					formats[cnt1].registers[cnt2][0],
					formats[cnt1].registers[cnt2][1]);
			}
			break;
		}
	}
	if (cnt1 == ARRAY_SIZE(formats)) {
		v4l2_dbg(3, debug, sd, "OV971X: FORMAT NOT AVAILABLE!!!!\n");
		return -EINVAL;
	}
	v4l2_dbg(3, debug, sd, "OV971X: FORMAT AVAILABLE!!!!\n");
	return 0;
}

static int ov971x_get_fmt(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct ov971x *ov971x = to_ov971x(sd);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width		= ov971x->width;
	pix->height		= ov971x->height;
	pix->pixelformat	= V4L2_PIX_FMT_SGRBG10;
	pix->field		= V4L2_FIELD_NONE;
	pix->colorspace		= V4L2_COLORSPACE_SRGB;

	return 0;
}

static int ov971x_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ARRAY_SIZE(ov971x_formats))
		return -EINVAL;

	fmt->flags = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strcpy(fmt->description, ov971x_formats[fmt->index].description);
	fmt->pixelformat = ov971x_formats[fmt->index].pixelformat;
	fmt->type = ov971x_formats[fmt->index].type;

	return 0;
}

static int ov971x_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_format *f)
{
	struct ov971x *ov971x = to_ov971x(sd);
	int ret, cnt1;
	struct v4l2_rect rect = {
		.left	= ov971x->x_current,
		.top	= ov971x->y_current,
		.width	= f->fmt.pix.width,
		.height	= f->fmt.pix.height,
	};

	for (cnt1 = 0; cnt1 < ARRAY_SIZE(formats); cnt1++) {
		if ((formats[cnt1].h_res == rect.width) &&
			(formats[cnt1].v_res == rect.height)) {
			break;
		}
	}
	if (cnt1 == ARRAY_SIZE(formats))
		return -EINVAL;
	ret = ov971x_set_params(sd, &rect);
	if (!ret) {
		ov971x->width = rect.width;
		ov971x->height = rect.height;
	}
	return ret;
}

static int ov971x_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_format *f)
{
	struct ov971x *ov971x = to_ov971x(sd);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (pix->height < OV971x_MIN_HEIGHT)
		pix->height = OV971x_MIN_HEIGHT;
	if (pix->height > OV971x_MAX_HEIGHT)
		pix->height = OV971x_MAX_HEIGHT;
	if (pix->width < OV971x_MIN_WIDTH)
		pix->width = OV971x_MIN_WIDTH;
	if (pix->width > OV971x_MAX_WIDTH)
		pix->width = OV971x_MAX_WIDTH;

	pix->pixelformat	= V4L2_PIX_FMT_SGRBG10;
	pix->field		= V4L2_FIELD_NONE;
	pix->colorspace		= V4L2_COLORSPACE_REC709;
	ov971x->current_height  = pix->height;
				/* V4L2_COLORSPACE_SRGB; */

	return 0;
}

int ov971x_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ov971x *ov971x = to_ov971x(sd);
	int light_freq, value;
	unsigned char tmp;

	if (param->parm.capture.timeperframe.numerator == 1 &&
	    param->parm.capture.timeperframe.denominator == 24)
		ov971x_set_frequency(ov971x->xclk_frequency, 24, sd);
	else
		ov971x_set_frequency(ov971x->xclk_frequency, 30, sd);

	if (ov971x->pwrline_fr == V4L2_CID_POWER_LINE_FREQUENCY_50HZ)
		light_freq = 50;
	else if (ov971x->pwrline_fr == V4L2_CID_POWER_LINE_FREQUENCY_60HZ)
		light_freq = 60;
	else {
		reg_write(v4l2_get_subdevdata(sd), OV971x_BANDING_STEP_L, 0);
		reg_write(v4l2_get_subdevdata(sd), OV971x_BANDING_STEP_H, 0);
		return 0;
	}
	value = param->parm.capture.timeperframe.denominator *
		 ov971x->current_height /
		 param->parm.capture.timeperframe.numerator /
		 light_freq;
	tmp = value & 0xFF;
	reg_write(v4l2_get_subdevdata(sd), OV971x_BANDING_STEP_L, tmp);
	tmp = (value & 0x300) >> 8;
	reg_write(v4l2_get_subdevdata(sd), OV971x_BANDING_STEP_H, tmp);
	return 0;
}

int ov971x_s_ext_ctrls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ctrls)
{
	unsigned int cnt, tot, tmp1, tmp2;
	unsigned char val[4];
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_ext_control *ec;

	tot = (ctrls->count < 16) ? ctrls->count : 16;
	for (cnt = 0; cnt < 4; cnt++)
		val[cnt] = reg_read(client, OV971x_REG4E + cnt);
	for (cnt = 0; cnt < tot; cnt++) {
		ec = (ctrls->controls) + cnt;
		if (ec->id != V4L2_CID_EXPOSURE_WINDOWS_WEIGHT)
			continue;
		tmp1 = cnt >> 2;
		tmp2 = (cnt & 0x3)<<1;
		val[tmp1] &= ~(3<<tmp2);
		val[tmp1] |= (ec->value & 0x3)<<tmp2;
	}
	for (cnt = 0; cnt < 4; cnt++)
		reg_write(client, OV971x_REG4E + cnt, val[cnt]);
	return 0;
}

static int ov971x_get_chip_id(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct ov971x *ov971x = to_ov971x(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);;

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= ov971x->model;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov971x_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);;

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->val = reg_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int ov971x_set_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (reg_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif

static int ov971x_get_control(struct v4l2_subdev *, struct v4l2_control *);
static int ov971x_set_control(struct v4l2_subdev *, struct v4l2_control *);
static int ov971x_queryctrl(struct v4l2_subdev *, struct v4l2_queryctrl *);
static int ov971x_querystd(struct v4l2_subdev *sd, v4l2_std_id *id);
static int ov971x_set_standard(struct v4l2_subdev *sd, v4l2_std_id id);

static const struct v4l2_subdev_core_ops ov971x_core_ops = {
	.g_chip_ident = ov971x_get_chip_id,
	.init = ov971x_init,
	.queryctrl = ov971x_queryctrl,
	.g_ctrl	= ov971x_get_control,
	.s_ctrl	= ov971x_set_control,
	.s_std = ov971x_set_standard,
	.s_ext_ctrls = ov971x_s_ext_ctrls,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov971x_get_register,
	.s_register = ov971x_set_register,
#endif
};

static const struct v4l2_subdev_video_ops ov971x_video_ops = {
	.s_fmt = ov971x_set_fmt,
	.g_fmt = ov971x_get_fmt,
	.try_fmt = ov971x_try_fmt,
	.querystd = ov971x_querystd,
	.s_stream = ov971x_s_stream,
	.enum_fmt = ov971x_enum_fmt,
	.s_parm = ov971x_s_parm,
};

static const struct v4l2_subdev_ops ov971x_ops = {
	.core = &ov971x_core_ops,
	.video = &ov971x_video_ops,
};


static int ov971x_queryctrl(struct v4l2_subdev *sd,
			    struct v4l2_queryctrl *qctrl)
{
	const struct v4l2_queryctrl *temp_qctrl;

	if (qctrl->id & V4L2_CTRL_FLAG_NEXT_CTRL)
		temp_qctrl =
			ov971x_find_next_qctrl(qctrl->id & V4L2_CTRL_ID_MASK);
	else
		temp_qctrl = ov971x_find_qctrl(qctrl->id);

	if (!temp_qctrl)
		return -EINVAL;
	memcpy(qctrl, temp_qctrl, sizeof(*qctrl));
	return 0;
}

static int ov971x_get_control(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct ov971x *ov971x = to_ov971x(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int data;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		data = reg_read(client, OV971x_REG04);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & OV971x_REG04_FLIP);
		break;
	case V4L2_CID_HFLIP:
		data = reg_read(client, OV971x_REG04);
		if (data < 0)
			return -EIO;
		ctrl->value = !!(data & OV971x_REG04_MIRROR);
		break;
	case V4L2_CID_EXPOSURE:
		ctrl->value = reg_read(client, OV971x_WPT);
		break;
	case V4L2_CID_GAIN:
		ctrl->value = reg_read(client, OV971x_GAIN) & 0x7F;
		break;
	case V4L2_CID_PAN_ABSOLUTE:
		data = reg_read(client, 0x9A);
		data &= 0x7;
		data <<= 8;
		data |= reg_read(client, 0x98);
		ctrl->value = data;
		break;
	case V4L2_CID_TILT_ABSOLUTE:
		data = reg_read(client, 0x9A);
		data &= 0x70;
		data <<= 4;
		data |= reg_read(client, 0x99);
		ctrl->value = data;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ctrl->value = ov971x->pwrline_fr;
		break;
	}
	return 0;
}

static int ov971x_set_control(struct v4l2_subdev *sd,
			       struct v4l2_control *ctrl)
{
	struct ov971x *ov971x = to_ov971x(sd);
	const struct v4l2_queryctrl *qctrl = NULL;
	unsigned long range;
	int data = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	static int vflip;

	if (NULL == ctrl)
		return -EINVAL;

	qctrl = ov971x_find_qctrl(ctrl->id);
	if (!qctrl) {
		v4l2_err(sd, "control id %d not supported", ctrl->id);
		return -EINVAL;
	}

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		if (ctrl->value && !vflip) {
			int tmp, tmp1, tmp2;
			data =
			reg_set(client, OV971x_REG04, OV971x_REG04_FLIP);
			tmp = (reg_read(client, OV971X_VSTART) << 2) |
			      (reg_read(client, OV971x_REG03) & 0x03);
			tmp1 = (reg_read(client, OV971x_AVSIZE_MSB) << 2) |
			       ((reg_read(client, OV971x_REG03) & 0x0C) >> 2);
			tmp2 = (reg_read(client, OV971x_VSIZE_MSB) << 2) |
			       ((reg_read(client, OV971x_REG57) & 0x03) >> 2);

			tmp1 -= tmp2 + 1;
			reg_write(client, OV971x_REG03,
				  (reg_read(client, OV971x_REG03) & 0xFC) |
				  ((tmp1 - tmp) & 0x3));
			reg_write(client, OV971X_VSTART, (tmp1 - tmp) >> 2);
			vflip = 1;
		} else if (vflip) {
			int tmp, tmp1, tmp2;
			data =
			reg_clear(client, OV971x_REG04, OV971x_REG04_FLIP);
			tmp = (reg_read(client, OV971X_VSTART) << 2) |
			      (reg_read(client, OV971x_REG03) & 0x03);
			tmp1 = (reg_read(client, OV971x_AVSIZE_MSB) << 2) |
			       ((reg_read(client, OV971x_REG03) & 0x0C) >> 2);
			tmp2 = (reg_read(client, OV971x_VSIZE_MSB) << 2) |
				((reg_read(client, OV971x_REG57) & 0x03) >> 2);

			tmp1 -= tmp2 - 1;
			reg_write(client, OV971x_REG03,
				  (reg_read(client, OV971x_REG03) & 0xFC) |
				  ((tmp1 - tmp) & 0x3));
			reg_write(client, OV971X_VSTART, (tmp1 - tmp) >> 2);
			vflip = 0;
		}
		if (data < 0)
			return -EIO;
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->value)
			data =
			reg_set(client, OV971x_REG04, OV971x_REG04_MIRROR);
		else
			data =
			reg_clear(client, OV971x_REG04, OV971x_REG04_MIRROR);
		if (data < 0)
			return -EIO;
		break;
	case V4L2_CID_GAIN:
		if (ctrl->value > qctrl->maximum \
			|| ctrl->value < qctrl->minimum)
			return -EINVAL;

		range = qctrl->default_value - qctrl->minimum;
		data = ((ctrl->value - qctrl->minimum) * 8 + range / 2)\
			/ range;

		v4l2_dbg(1, debug, sd, "Setting gain %d\n", data);
		/* data = reg_write(client, OV971x_GAIN, data); */
		if (data < 0)
			return -EIO;

		/* Success */
		ov971x->gain = ctrl->value;
		break;
	case V4L2_CID_EXPOSURE:
		/* ov971x has maximum == default */
		if (ctrl->value > qctrl->maximum ||
		    ctrl->value < qctrl->minimum)
			return -EINVAL;
		else {
			u32 shutter = ctrl->value;

			if (set_shutter(sd, shutter) < 0)
				return -EIO;
			ov971x->exposure = ctrl->value;
			ov971x->autoexposure = 0;
		}
		break;
	case V4L2_CID_PAN_RELATIVE:
		data = reg_read(client, 0x98);
		data += ctrl->value;
		reg_write(client, 0x98, data & 0xFF);
		if (data <= 0xFF)
			break;
		data = reg_read(client, 0x9A) + ((data & 0xF00) >> 8);
		reg_write(client, 0x9A, data);
		break;
	case V4L2_CID_TILT_RELATIVE:
		data = reg_read(client, 0x99);
		data += ctrl->value;
		reg_write(client, 0x99, data & 0xFF);
		if (data <= 0xFF)
			break;
		data = reg_read(client, 0x9A) + ((data & 0xF00) >> 4);
		reg_write(client, 0x9A, data);
		break;
	case V4L2_CID_PAN_ABSOLUTE:
		reg_write(client, 0x98, ctrl->value & 0xFF);
		data = reg_read(client, 0x9A);
		data &= 0xF0;
		data |= (ctrl->value >> 8);
		reg_write(client, 0x9A, data);
		break;
	case V4L2_CID_TILT_ABSOLUTE:
		reg_write(client, 0x99, ctrl->value & 0xFF);
		data = reg_read(client, 0x9A);
		data &= 0x0F;
		data |= ((ctrl->value & 0xF00) >> 4);
		reg_write(client, 0x9A, data);
		break;
	case V4L2_CID_PAN_RESET:
		reg_write(client, 0x98, 0x40);
		data = reg_read(client, 0x9A);
		data &= 0xF0;
		data |= 0x01;
		reg_write(client, 0x9A, data);
		break;
	case V4L2_CID_TILT_RESET:
		reg_write(client, 0x99, 0xa0);
		data = reg_read(client, 0x9A);
		data &= 0x0F;
		reg_write(client, 0x9A, data);
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ov971x->pwrline_fr = ctrl->value;
		break;
	case V4L2_CID_EXPOSURE_WINDOWS_WEIGHT:
		break;
	}
	return 0;
}

/* Function querystd not supported by ov971x */
static int ov971x_querystd(struct v4l2_subdev *sd, v4l2_std_id *id)
{
	*id = V4L2_STD_OV971x_STD_ALL;

	return 0;
}

/* Function set not supported by ov971x */
static int ov971x_set_standard(struct v4l2_subdev *sd, v4l2_std_id id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_info(&client->dev, "%s %llx\n", __func__, id);
	return 0;
}
/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int ov971x_detect(struct i2c_client *client, int *model)
{
	s32 data;

	/* Read out the chip version register */
	data = reg_read_word(client, OV971X_PIDH);

	switch (data) {
	case 0x9711:
		dev_info(&client->dev,
			 "Detected a OV971X chip. PID=%x\n", data);
		*model = V4L2_IDENT_OV9712;
		break;
	default:
		dev_err(&client->dev,
			"No OV971x chip detected, register read %x\n", data);
		return -ENODEV;
	}

	return 0;
}

/* read lux */
static ssize_t ov971x_show_lux_1(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	unsigned int val, wpt;
	struct i2c_client *client = to_i2c_client(dev);

	val = reg_read(client, OV971x_YAVG);
	wpt = reg_read(client, OV971x_WPT);

	return sprintf(buf, "%d\n", val * 100 / wpt);
}

static DEVICE_ATTR(light, S_IRUGO, ov971x_show_lux_1, NULL);

static struct attribute *ov971x_attributes_1[] = {
	&dev_attr_light.attr,
	NULL
};

static const struct attribute_group ov971x_attr_group_1 = {
	.attrs = ov971x_attributes_1,
};

static ssize_t ov971x_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct device *i2cdev = (struct device *)dev_get_drvdata(dev);
	return ov971x_show_lux_1(i2cdev, attr, buf);
}

static DEVICE_ATTR(light_lev, S_IRUGO, ov971x_show_lux, NULL);

static struct attribute *ov971x_attributes[] = {
	&dev_attr_light_lev.attr,
	NULL
};

static const struct attribute_group ov971x_attr_group = {
	.attrs = ov971x_attributes,
};

static int ov971x_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov971x *ov971x;
	struct ov971x_platform_data *ov971x_pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	int ret, cnt;
	struct class *light_class;
	struct device *dev;

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

	ov971x = kzalloc(sizeof(struct ov971x), GFP_KERNEL);
	if (!ov971x)
		return -ENOMEM;

	ret = ov971x_detect(client, &ov971x->model);
	if (ret)
		goto clean;

	ov971x->x_min		= OV971x_COLUMN_SKIP;
	ov971x->y_min		= OV971x_ROW_SKIP;
	ov971x->width		= OV971x_DEFAULT_WIDTH;
	ov971x->height		= OV971x_DEFAULT_HEIGHT;
	ov971x->x_current	= 0; /* ov971x->x_min; */
	ov971x->y_current	= 0; /* ov971x->y_min; */
	ov971x->width_min	= OV971x_MIN_WIDTH;
	ov971x->width_max	= OV971x_MAX_WIDTH;
	ov971x->height_min	= OV971x_MIN_HEIGHT;
	ov971x->height_max	= OV971x_MAX_HEIGHT;
	ov971x->y_skip_top	= 10;
	ov971x->autoexposure = 1;
	ov971x->xskip = 1;
	ov971x->yskip = 1;
	ov971x->pwrline_fr = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;

	/* Register with V4L2 layer as slave device */
	sd = &ov971x->sd;
	v4l2_i2c_subdev_init(sd, client, &ov971x_ops);
	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);

	if (ov971x_pdata->clk_polarity)
		reg_clear(v4l2_get_subdevdata(sd),
			OV971x_DVP_CTRL_00, OV971x_DVP_CTRL_00_PCLK_POL);
	else
		reg_set(v4l2_get_subdevdata(sd),
			OV971x_DVP_CTRL_00, OV971x_DVP_CTRL_00_PCLK_POL);

	if (ov971x_pdata->hs_polarity)
		reg_clear(v4l2_get_subdevdata(sd),
			OV971x_COM10, OV971x_COM10_HSYNC_POL);
	else
		reg_set(v4l2_get_subdevdata(sd),
			OV971x_COM10, OV971x_COM10_HSYNC_POL);

	if (ov971x_pdata->vs_polarity)
		reg_clear(v4l2_get_subdevdata(sd),
			OV971x_COM10, OV971x_COM10_VSYNC_POL);
	else
		reg_set(v4l2_get_subdevdata(sd),
			OV971x_COM10, OV971x_COM10_VSYNC_POL);

	ret = reg_read(v4l2_get_subdevdata(sd), OV971x_REG5D);
	ret &= ~OV971x_REG5D_OUT_DRIVE_MASK;
	ret |= (ov971x_pdata->out_drive_capability) <<
					OV971x_REG5D_OUT_DRIVE_SHIFT;
	reg_write(v4l2_get_subdevdata(sd), OV971x_REG5D, ret);

	ov971x->xclk_frequency = ov971x_pdata->xclk_frequency;
	ov971x_set_frequency(ov971x_pdata->xclk_frequency, 30, sd);

	for (cnt = 0; cnt < ARRAY_SIZE(reg_1280x720); cnt++)
		reg_write(v4l2_get_subdevdata(sd), reg_1280x720[cnt][0],
							reg_1280x720[cnt][1]);

	light_class = class_create(THIS_MODULE, "lighting_level");
	dev = device_create(light_class, NULL, client->dev.devt,
			    NULL, "ov9712");
	dev->platform_data = client;
	ret = sysfs_create_group(&client->dev.kobj, &ov971x_attr_group_1);
	ret = sysfs_create_group(&dev->kobj, &ov971x_attr_group);
	dev_set_drvdata(dev, &client->dev);
	return 0;

clean:
	kfree(ov971x);
	return ret;
}

static int ov971x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov971x *ov971x = to_ov971x(sd);

	v4l2_device_unregister_subdev(sd);

	kfree(ov971x);
	return 0;
}

static const struct i2c_device_id ov971x_id[] = {
	{ "ov971x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov971x_id);

static struct i2c_driver ov971x_i2c_driver = {
	.driver = {
		.name = "ov971x",
	},
	.probe		= ov971x_probe,
	.remove		= ov971x_remove,
	.id_table	= ov971x_id,
};

static int __init ov971x_mod_init(void)
{
	return i2c_add_driver(&ov971x_i2c_driver);
}

static void __exit ov971x_mod_exit(void)
{
	i2c_del_driver(&ov971x_i2c_driver);
}

module_init(ov971x_mod_init);
module_exit(ov971x_mod_exit);

MODULE_DESCRIPTION("Omnivision OV971x Camera driver");
MODULE_AUTHOR("Davide Bonfanti <davide.bonfanti@bticino.it>");
MODULE_LICENSE("GPL v2");
