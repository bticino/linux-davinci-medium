/*
 * tvp5150 - Texas Instruments TVP5150A/AM1 video decoder driver
 *
 * Copyright (c) 2005,2006 Mauro Carvalho Chehab (mchehab@infradead.org)
 * This code is placed under the terms of the GNU General Public License v2
 */

#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <media/v4l2-device.h>
#include <media/tvp5150.h>
#include <media/v4l2-i2c-drv.h>
#include <media/v4l2-chip-ident.h>
#include <linux/gpio.h>
#include <linux/i2c/tvp5150.h>

#include "tvp5150_reg.h"

MODULE_DESCRIPTION("Texas Instruments TVP5150A video decoder driver");
MODULE_AUTHOR("Mauro Carvalho Chehab");
MODULE_LICENSE("GPL");


static int debug;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/* enum tvp5150_std - enum for supported standards */
enum tvp5150_std {
	STD_NTSC_MJ = 0,
	STD_PAL_BDGHIN,
	STD_INVALID
};

/* Private macros for TVP */
#define LOCK_RETRY_COUNT                (5)
#define LOCK_RETRY_DELAY                (200)

/* supported controls */
static struct v4l2_queryctrl tvp5150_qctrl[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Brightness",
		.minimum = 0,
		.maximum = 255,
		.step = 1,
		.default_value = 128,
		.flags = 0,
	}, {
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 255,
		.step = 0x1,
		.default_value = 128,
		.flags = 0,
	}, {
		 .id = V4L2_CID_SATURATION,
		 .type = V4L2_CTRL_TYPE_INTEGER,
		 .name = "Saturation",
		 .minimum = 0,
		 .maximum = 255,
		 .step = 0x1,
		 .default_value = 128,
		 .flags = 0,
	}, {
		.id = V4L2_CID_HUE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Hue",
		.minimum = -128,
		.maximum = 127,
		.step = 0x1,
		.default_value = 0,
		.flags = 0,
	}
};

static int tvp5150_s_stream(struct v4l2_subdev *sd, int enable);
static int tvp5150_try_fmt_cap(struct v4l2_subdev *sd,
				struct v4l2_format *f);
/**
 * struct tvp5150_std_info - Structure to store standard informations
 * @width: Line width in pixels
 * @height:Number of active lines
 * @video_std: Value to write in TVP5150_VIDEO_STD register
 * @standard: v4l2 standard structure information
 */
struct tvp5150_std_info {
	unsigned long width;
	unsigned long height;
	u8 video_std;
	struct v4l2_standard standard;
};

/**
 * Supported standards -
 *
 * Currently supports two standards only, need to add support for rest of the
 * modes, like SECAM, etc...
 */
static struct tvp5150_std_info tvp5150_std_list[] = {
	/* Standard: STD_NTSC_MJ */
	[STD_NTSC_MJ] = {
	 .width = NTSC_NUM_ACTIVE_PIXELS,
	 .height = NTSC_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_NTSC_MJ_BIT,
	 .standard = {
		      .index = 0,
		      .id = V4L2_STD_NTSC,
		      .name = "NTSC",
		      .frameperiod = {1001, 30000},
		      .framelines = 525
		     },
	/* Standard: STD_PAL_BDGHIN */
	},
	[STD_PAL_BDGHIN] = {
	 .width = PAL_NUM_ACTIVE_PIXELS,
	 .height = PAL_NUM_ACTIVE_LINES,
	 .video_std = VIDEO_STD_PAL_BDGHIN_BIT,
	 .standard = {
		      .index = 1,
		      .id = V4L2_STD_PAL,
		      .name = "PAL",
		      .frameperiod = {1, 25},
		      .framelines = 625
		     },
	},
	/* Standard: need to add for additional standard */
};

struct tvp5150 {
	struct v4l2_subdev sd;
        const struct tvp5150_platform_data *pdata;

	int ver_maj;
	int ver_min;
	v4l2_std_id norm;	/* Current set standard */
	u32 input;
	u32 output;
	int enable;
	int bright;
	int contrast;
	int hue;
	int sat;
	int streaming;
	struct v4l2_pix_format pix;
	int num_fmts;
	const struct v4l2_fmtdesc *fmt_list;
	enum tvp5150_std current_std;
	int num_stds;
	struct tvp5150_std_info *std_list;
};

static inline struct tvp5150 *to_tvp5150(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tvp5150, sd);
}

static int tvp5150_read(struct v4l2_subdev *sd, unsigned char addr)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	unsigned char buffer[1];
	int rc;

	buffer[0] = addr;
	if (1 != (rc = i2c_master_send(c, buffer, 1)))
		v4l2_dbg(0, debug, sd, "i2c i/o error: rc == %d (should be 1)\n", rc);

	msleep(10);

	if (1 != (rc = i2c_master_recv(c, buffer, 1)))
		v4l2_dbg(0, debug, sd, "i2c i/o error: rc == %d (should be 1)\n", rc);

	v4l2_dbg(2, debug, sd, "tvp5150: read 0x%02x = 0x%02x\n", addr, buffer[0]);

	return (buffer[0]);
}

static inline void tvp5150_write(struct v4l2_subdev *sd, unsigned char addr,
				 unsigned char value)
{
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	unsigned char buffer[2];
	int rc;

	buffer[0] = addr;
	buffer[1] = value;
	v4l2_dbg(2, debug, sd, "tvp5150: writing 0x%02x 0x%02x\n", buffer[0], buffer[1]);
	if (2 != (rc = i2c_master_send(c, buffer, 2)))
		v4l2_dbg(0, debug, sd, "i2c i/o error: rc == %d (should be 2)\n", rc);
}

static void dump_reg_range(struct v4l2_subdev *sd, char *s, u8 init,
				const u8 end, int max_line)
{
	int i = 0;

	while (init != (u8)(end + 1)) {
		if ((i % max_line) == 0) {
			if (i > 0)
				printk("\n");
			printk("tvp5150: %s reg 0x%02x = ", s, init);
		}
		printk("%02x ", tvp5150_read(sd, init));

		init++;
		i++;
	}
	printk("\n");
}

static int tvp5150_log_status(struct v4l2_subdev *sd)
{
	printk("tvp5150: Video input source selection #1 = 0x%02x\n",
			tvp5150_read(sd, TVP5150_VD_IN_SRC_SEL_1));
	printk("tvp5150: Analog channel controls = 0x%02x\n",
			tvp5150_read(sd, TVP5150_ANAL_CHL_CTL));
	printk("tvp5150: Operation mode controls = 0x%02x\n",
			tvp5150_read(sd, TVP5150_OP_MODE_CTL));
	printk("tvp5150: Miscellaneous controls = 0x%02x\n",
			tvp5150_read(sd, TVP5150_MISC_CTL));
	printk("tvp5150: Autoswitch mask= 0x%02x\n",
			tvp5150_read(sd, TVP5150_AUTOSW_MSK));
	printk("tvp5150: Color killer threshold control = 0x%02x\n",
			tvp5150_read(sd, TVP5150_COLOR_KIL_THSH_CTL));
	printk("tvp5150: Luminance processing controls #1 #2 and #3 = %02x %02x %02x\n",
			tvp5150_read(sd, TVP5150_LUMA_PROC_CTL_1),
			tvp5150_read(sd, TVP5150_LUMA_PROC_CTL_2),
			tvp5150_read(sd, TVP5150_LUMA_PROC_CTL_3));
	printk("tvp5150: Brightness control = 0x%02x\n",
			tvp5150_read(sd, TVP5150_BRIGHT_CTL));
	printk("tvp5150: Color saturation control = 0x%02x\n",
			tvp5150_read(sd, TVP5150_SATURATION_CTL));
	printk("tvp5150: Hue control = 0x%02x\n",
			tvp5150_read(sd, TVP5150_HUE_CTL));
	printk("tvp5150: Contrast control = 0x%02x\n",
			tvp5150_read(sd, TVP5150_CONTRAST_CTL));
	printk("tvp5150: Outputs and data rates select = 0x%02x\n",
			tvp5150_read(sd, TVP5150_DATA_RATE_SEL));
	printk("tvp5150: Configuration shared pins = 0x%02x\n",
			tvp5150_read(sd, TVP5150_CONF_SHARED_PIN));
	printk("tvp5150: Active video cropping start = 0x%02x%02x\n",
			tvp5150_read(sd, TVP5150_ACT_VD_CROP_ST_MSB),
			tvp5150_read(sd, TVP5150_ACT_VD_CROP_ST_LSB));
	printk("tvp5150: Active video cropping stop  = 0x%02x%02x\n",
			tvp5150_read(sd, TVP5150_ACT_VD_CROP_STP_MSB),
			tvp5150_read(sd, TVP5150_ACT_VD_CROP_STP_LSB));
	printk("tvp5150: Genlock/RTC = 0x%02x\n",
			tvp5150_read(sd, TVP5150_GENLOCK));
	printk("tvp5150: Horizontal sync start = 0x%02x\n",
			tvp5150_read(sd, TVP5150_HORIZ_SYNC_START));
	printk("tvp5150: Vertical blanking start = 0x%02x\n",
			tvp5150_read(sd, TVP5150_VERT_BLANKING_START));
	printk("tvp5150: Vertical blanking stop = 0x%02x\n",
			tvp5150_read(sd, TVP5150_VERT_BLANKING_STOP));
	printk("tvp5150: Chrominance processing control #1 and #2 = %02x %02x\n",
			tvp5150_read(sd, TVP5150_CHROMA_PROC_CTL_1),
			tvp5150_read(sd, TVP5150_CHROMA_PROC_CTL_2));
	printk("tvp5150: Interrupt reset register B = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_RESET_REG_B));
	printk("tvp5150: Interrupt enable register B = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_ENABLE_REG_B));
	printk("tvp5150: Interrupt configuration register B = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INTT_CONFIG_REG_B));
	printk("tvp5150: Video standard = 0x%02x\n",
			tvp5150_read(sd, TVP5150_VIDEO_STD));
	printk("tvp5150: Chroma gain factor: Cb=0x%02x Cr=0x%02x\n",
			tvp5150_read(sd, TVP5150_CB_GAIN_FACT),
			tvp5150_read(sd, TVP5150_CR_GAIN_FACTOR));
	printk("tvp5150: Macrovision on counter = 0x%02x\n",
			tvp5150_read(sd, TVP5150_MACROVISION_ON_CTR));
	printk("tvp5150: Macrovision off counter = 0x%02x\n",
			tvp5150_read(sd, TVP5150_MACROVISION_OFF_CTR));
	printk("tvp5150: ITU-R BT.656.%d timing(TVP5150AM1 only)\n",
			(tvp5150_read(sd, TVP5150_REV_SELECT) & 1) ? 3 : 4);
	printk("tvp5150: Device ID = %02x%02x\n",
			tvp5150_read(sd, TVP5150_MSB_DEV_ID),
			tvp5150_read(sd, TVP5150_LSB_DEV_ID));
	printk("tvp5150: ROM version = (hex) %02x.%02x\n",
			tvp5150_read(sd, TVP5150_ROM_MAJOR_VER),
			tvp5150_read(sd, TVP5150_ROM_MINOR_VER));
	printk("tvp5150: Vertical line count = 0x%02x%02x\n",
			tvp5150_read(sd, TVP5150_VERT_LN_COUNT_MSB),
			tvp5150_read(sd, TVP5150_VERT_LN_COUNT_LSB));
	printk("tvp5150: Interrupt status register B = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_STATUS_REG_B));
	printk("tvp5150: Interrupt active register B = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_ACTIVE_REG_B));
	printk("tvp5150: Status regs #1 to #5 = %02x %02x %02x %02x %02x\n",
			tvp5150_read(sd, TVP5150_STATUS_REG_1),
			tvp5150_read(sd, TVP5150_STATUS_REG_2),
			tvp5150_read(sd, TVP5150_STATUS_REG_3),
			tvp5150_read(sd, TVP5150_STATUS_REG_4),
			tvp5150_read(sd, TVP5150_STATUS_REG_5));

	dump_reg_range(sd, "Teletext filter 1",   TVP5150_TELETEXT_FIL1_INI,
			TVP5150_TELETEXT_FIL1_END, 8);
	dump_reg_range(sd, "Teletext filter 2",   TVP5150_TELETEXT_FIL2_INI,
			TVP5150_TELETEXT_FIL2_END, 8);

	printk("tvp5150: Teletext filter enable = 0x%02x\n",
			tvp5150_read(sd, TVP5150_TELETEXT_FIL_ENA));
	printk("tvp5150: Interrupt status register A = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_STATUS_REG_A));
	printk("tvp5150: Interrupt enable register A = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_ENABLE_REG_A));
	printk("tvp5150: Interrupt configuration = 0x%02x\n",
			tvp5150_read(sd, TVP5150_INT_CONF));
	printk("tvp5150: VDP status register = 0x%02x\n",
			tvp5150_read(sd, TVP5150_VDP_STATUS_REG));
	printk("tvp5150: FIFO word count = 0x%02x\n",
			tvp5150_read(sd, TVP5150_FIFO_WORD_COUNT));
	printk("tvp5150: FIFO interrupt threshold = 0x%02x\n",
			tvp5150_read(sd, TVP5150_FIFO_INT_THRESHOLD));
	printk("tvp5150: FIFO reset = 0x%02x\n",
			tvp5150_read(sd, TVP5150_FIFO_RESET));
	printk("tvp5150: Line number interrupt = 0x%02x\n",
			tvp5150_read(sd, TVP5150_LINE_NUMBER_INT));
	printk("tvp5150: Pixel alignment register = 0x%02x%02x\n",
			tvp5150_read(sd, TVP5150_PIX_ALIGN_REG_HIGH),
			tvp5150_read(sd, TVP5150_PIX_ALIGN_REG_LOW));
	printk("tvp5150: FIFO output control = 0x%02x\n",
			tvp5150_read(sd, TVP5150_FIFO_OUT_CTRL));
	printk("tvp5150: Full field enable = 0x%02x\n",
			tvp5150_read(sd, TVP5150_FULL_FIELD_ENA));
	printk("tvp5150: Full field mode register = 0x%02x\n",
			tvp5150_read(sd, TVP5150_FULL_FIELD_MODE_REG));

	dump_reg_range(sd, "CC   data",   TVP5150_CC_DATA_INI,
			TVP5150_CC_DATA_END, 8);

	dump_reg_range(sd, "WSS  data",   TVP5150_WSS_DATA_INI,
			TVP5150_WSS_DATA_END, 8);

	dump_reg_range(sd, "VPS  data",   TVP5150_VPS_DATA_INI,
			TVP5150_VPS_DATA_END, 8);

	dump_reg_range(sd, "VITC data",   TVP5150_VITC_DATA_INI,
			TVP5150_VITC_DATA_END, 10);

	dump_reg_range(sd, "Line mode",   TVP5150_LINE_MODE_INI,
			TVP5150_LINE_MODE_END, 8);
	return 0;
}

/****************************************************************************
			Basic functions
 ****************************************************************************/

static inline void tvp5150_selmux(struct v4l2_subdev *sd)
{
	int opmode = 0;
	struct tvp5150 *decoder = to_tvp5150(sd);
	int input = 0;
	unsigned char val;

	if ((decoder->output & TVP5150_BLACK_SCREEN) || !decoder->enable)
		input = 8;

	switch (decoder->input) {
	case TVP5150_COMPOSITE1:
		input |= 2;
		/* fall through */
	case TVP5150_COMPOSITE0:
		break;
	case TVP5150_SVIDEO:
	default:
		input |= 1;
		break;
	}

	v4l2_dbg(1, debug, sd, "Selecting video route: route input=%i, output=%i "
			"=> tvp5150 input=%i, opmode=%i\n",
			decoder->input, decoder->output,
			input, opmode);

	tvp5150_write(sd, TVP5150_OP_MODE_CTL, opmode);
	tvp5150_write(sd, TVP5150_VD_IN_SRC_SEL_1, input);

	/* Svideo should enable YCrCb output and disable GPCL output
	 * For Composite and TV, it should be the reverse
	 */
	val = tvp5150_read(sd, TVP5150_MISC_CTL);
	if (decoder->input == TVP5150_SVIDEO)
		val = (val & ~0x40) | 0x10;
	else
		val = (val & ~0x10) | 0x40;
	tvp5150_write(sd, TVP5150_MISC_CTL, val);
};

struct i2c_reg_value {
	unsigned char reg;
	unsigned char value;
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_default[] = {
	{ /* 0x00 */
		TVP5150_VD_IN_SRC_SEL_1,0x00
	},
	{ /* 0x01 */
		TVP5150_ANAL_CHL_CTL,0x15
	},
	{ /* 0x02 */
		TVP5150_OP_MODE_CTL,0x00
	},
	{ /* 0x03 */
		TVP5150_MISC_CTL,0x01
	},
	{ /* 0x06 */
		TVP5150_COLOR_KIL_THSH_CTL,0x10
	},
	{ /* 0x07 */
		TVP5150_LUMA_PROC_CTL_1,0x60
	},
	{ /* 0x08 */
		TVP5150_LUMA_PROC_CTL_2,0x00
	},
	{ /* 0x09 */
		TVP5150_BRIGHT_CTL,0x80
	},
	{ /* 0x0a */
		TVP5150_SATURATION_CTL,0x80
	},
	{ /* 0x0b */
		TVP5150_HUE_CTL,0x00
	},
	{ /* 0x0c */
		TVP5150_CONTRAST_CTL,0x80
	},
	{ /* 0x0d */
		TVP5150_DATA_RATE_SEL,0x47
	},
	{ /* 0x0e */
		TVP5150_LUMA_PROC_CTL_3,0x00
	},
	{ /* 0x0f */
		TVP5150_CONF_SHARED_PIN,0x08
	},
	{ /* 0x11 */
		TVP5150_ACT_VD_CROP_ST_MSB,0x00
	},
	{ /* 0x12 */
		TVP5150_ACT_VD_CROP_ST_LSB,0x00
	},
	{ /* 0x13 */
		TVP5150_ACT_VD_CROP_STP_MSB,0x00
	},
	{ /* 0x14 */
		TVP5150_ACT_VD_CROP_STP_LSB,0x00
	},
	{ /* 0x15 */
		TVP5150_GENLOCK,0x01
	},
	{ /* 0x16 */
		TVP5150_HORIZ_SYNC_START,0x80
	},
	{ /* 0x18 */
		TVP5150_VERT_BLANKING_START,0x00
	},
	{ /* 0x19 */
		TVP5150_VERT_BLANKING_STOP,0x00
	},
	{ /* 0x1a */
		TVP5150_CHROMA_PROC_CTL_1,0x0c
	},
	{ /* 0x1b */
		TVP5150_CHROMA_PROC_CTL_2,0x14
	},
	{ /* 0x1c */
		TVP5150_INT_RESET_REG_B,0x00
	},
	{ /* 0x1d */
		TVP5150_INT_ENABLE_REG_B,0x00
	},
	{ /* 0x1e */
		TVP5150_INTT_CONFIG_REG_B,0x00
	},
	{ /* 0x28 */
		TVP5150_VIDEO_STD,0x00
	},
	{ /* 0x2e */
		TVP5150_MACROVISION_ON_CTR,0x0f
	},
	{ /* 0x2f */
		TVP5150_MACROVISION_OFF_CTR,0x01
	},
	{ /* 0xbb */
		TVP5150_TELETEXT_FIL_ENA,0x00
	},
	{ /* 0xc0 */
		TVP5150_INT_STATUS_REG_A,0x00
	},
	{ /* 0xc1 */
		TVP5150_INT_ENABLE_REG_A,0x00
	},
	{ /* 0xc2 */
		TVP5150_INT_CONF,0x04
	},
	{ /* 0xc8 */
		TVP5150_FIFO_INT_THRESHOLD,0x80
	},
	{ /* 0xc9 */
		TVP5150_FIFO_RESET,0x00
	},
	{ /* 0xca */
		TVP5150_LINE_NUMBER_INT,0x00
	},
	{ /* 0xcb */
		TVP5150_PIX_ALIGN_REG_LOW,0x4e
	},
	{ /* 0xcc */
		TVP5150_PIX_ALIGN_REG_HIGH,0x00
	},
	{ /* 0xcd */
		TVP5150_FIFO_OUT_CTRL,0x01
	},
	{ /* 0xcf */
		TVP5150_FULL_FIELD_ENA,0x00
	},
	{ /* 0xd0 */
		TVP5150_LINE_MODE_INI,0x00
	},
	{ /* 0xfc */
		TVP5150_FULL_FIELD_MODE_REG,0x7f
	},
	{ /* end of data */
		0xff,0xff
	}
};

/* Default values as sugested at TVP5150AM1 datasheet */
static const struct i2c_reg_value tvp5150_init_enable[] = {
	{
		TVP5150_CONF_SHARED_PIN, 2
	},{	/* Automatic offset and AGC enabled */
		TVP5150_ANAL_CHL_CTL, 0x15
	},{	/* Activate YCrCb output 0x9 or 0xd ? */
		TVP5150_MISC_CTL, 0x6f
	},{	/* Activates video std autodetection for all standards */
		TVP5150_AUTOSW_MSK, 0x0
	},{	/* Default format: 0x47. For 4:2:2: 0x40 */
		TVP5150_DATA_RATE_SEL, 0x47
	},{
		TVP5150_CHROMA_PROC_CTL_1, 0x0c
	},{
		TVP5150_CHROMA_PROC_CTL_2, 0x54
	},{	/* Non documented, but initialized on WinTV USB2 */
		0x27, 0x20
	},{
		0xff,0xff
	}
};

/* Default values as sugested at TVP5151 datasheet */
static const struct i2c_reg_value tvp5151_init_enable[] = {
	{	/* Activate YCrCb output 0x9 */
		TVP5150_VD_IN_SRC_SEL_1, 0x0
	},{	/* Default format: 0x47. For 4:2:2: 0x40 */
		TVP5150_MISC_CTL, 0x9
	},{	/* Default format: 0x47. For 4:2:2: 0x40 */
		TVP5150_DATA_RATE_SEL, 0x47
	},{
		0xff,0xff
	}
};

struct tvp5150_vbi_type {
	unsigned int vbi_type;
	unsigned int ini_line;
	unsigned int end_line;
	unsigned int by_field :1;
};

struct i2c_vbi_ram_value {
	u16 reg;
	struct tvp5150_vbi_type type;
	unsigned char values[16];
};

/* This struct have the values for each supported VBI Standard
 * by
 tvp5150_vbi_types should follow the same order as vbi_ram_default
 * value 0 means rom position 0x10, value 1 means rom position 0x30
 * and so on. There are 16 possible locations from 0 to 15.
 */

static struct i2c_vbi_ram_value vbi_ram_default[] =
{
	/* FIXME: Current api doesn't handle all VBI types, those not
	   yet supported are placed under #if 0 */
#if 0
	{0x010, /* Teletext, SECAM, WST System A */
		{V4L2_SLICED_TELETEXT_SECAM,6,23,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x26,
		  0xe6, 0xb4, 0x0e, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#endif
	{0x030, /* Teletext, PAL, WST System B */
		{V4L2_SLICED_TELETEXT_B,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x2b,
		  0xa6, 0x72, 0x10, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
#if 0
	{0x050, /* Teletext, PAL, WST System C */
		{V4L2_SLICED_TELETEXT_PAL_C,6,22,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0xa6, 0x98, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x070, /* Teletext, NTSC, WST System B */
		{V4L2_SLICED_TELETEXT_NTSC_B,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0x27, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x090, /* Tetetext, NTSC NABTS System C */
		{V4L2_SLICED_TELETEXT_NTSC_C,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xe7, 0x2e, 0x20, 0x22,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x15, 0x00 }
	},
	{0x0b0, /* Teletext, NTSC-J, NABTS System D */
		{V4L2_SLICED_TELETEXT_NTSC_D,10,21,1},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xa7, 0x2e, 0x20, 0x23,
		  0x69, 0x93, 0x0d, 0x00, 0x00, 0x00, 0x10, 0x00 }
	},
	{0x0d0, /* Closed Caption, PAL/SECAM */
		{V4L2_SLICED_CAPTION_625,22,22,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0xa6, 0x7b, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
#endif
	{0x0f0, /* Closed Caption, NTSC */
		{V4L2_SLICED_CAPTION_525,21,21,1},
		{ 0xaa, 0x2a, 0xff, 0x3f, 0x04, 0x51, 0x6e, 0x02,
		  0x69, 0x8c, 0x09, 0x00, 0x00, 0x00, 0x27, 0x00 }
	},
	{0x110, /* Wide Screen Signal, PAL/SECAM */
		{V4L2_SLICED_WSS_625,23,23,1},
		{ 0x5b, 0x55, 0xc5, 0xff, 0x00, 0x71, 0x6e, 0x42,
		  0xa6, 0xcd, 0x0f, 0x00, 0x00, 0x00, 0x3a, 0x00 }
	},
#if 0
	{0x130, /* Wide Screen Signal, NTSC C */
		{V4L2_SLICED_WSS_525,20,20,1},
		{ 0x38, 0x00, 0x3f, 0x00, 0x00, 0x71, 0x6e, 0x43,
		  0x69, 0x7c, 0x08, 0x00, 0x00, 0x00, 0x39, 0x00 }
	},
	{0x150, /* Vertical Interval Timecode (VITC), PAL/SECAM */
		{V4l2_SLICED_VITC_625,6,22,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0xa6, 0x85, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
	{0x170, /* Vertical Interval Timecode (VITC), NTSC */
		{V4l2_SLICED_VITC_525,10,20,0},
		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x6d, 0x49,
		  0x69, 0x94, 0x08, 0x00, 0x00, 0x00, 0x4c, 0x00 }
	},
#endif
	{0x190, /* Video Program System (VPS), PAL */
		{V4L2_SLICED_VPS,16,16,0},
		{ 0xaa, 0xaa, 0xff, 0xff, 0xba, 0xce, 0x2b, 0x0d,
		  0xa6, 0xda, 0x0b, 0x00, 0x00, 0x00, 0x60, 0x00 }
	},
	/* 0x1d0 User programmable */

	/* End of struct */
	{ (u16)-1 }
};

static int tvp5150_write_inittab(struct v4l2_subdev *sd,
				const struct i2c_reg_value *regs)
{
	while (regs->reg != 0xff) {
		tvp5150_write(sd, regs->reg, regs->value);
		regs++;
	}
	return 0;
}

static int tvp5150_vdp_init(struct v4l2_subdev *sd,
				const struct i2c_vbi_ram_value *regs)
{
	unsigned int i;

	/* Disable Full Field */
	tvp5150_write(sd, TVP5150_FULL_FIELD_ENA, 0);

	/* Before programming, Line mode should be at 0xff */
	for (i = TVP5150_LINE_MODE_INI; i <= TVP5150_LINE_MODE_END; i++)
		tvp5150_write(sd, i, 0xff);

	/* Load Ram Table */
	while (regs->reg != (u16)-1) {
		tvp5150_write(sd, TVP5150_CONF_RAM_ADDR_HIGH, regs->reg >> 8);
		tvp5150_write(sd, TVP5150_CONF_RAM_ADDR_LOW, regs->reg);

		for (i = 0; i < 16; i++)
			tvp5150_write(sd, TVP5150_VDP_CONF_RAM_DATA, regs->values[i]);

		regs++;
	}
	return 0;
}

/* Fills VBI capabilities based on i2c_vbi_ram_value struct */
static int tvp5150_g_sliced_vbi_cap(struct v4l2_subdev *sd,
				struct v4l2_sliced_vbi_cap *cap)
{
	const struct i2c_vbi_ram_value *regs = vbi_ram_default;
	int line;

	v4l2_dbg(1, debug, sd, "g_sliced_vbi_cap\n");
	memset(cap, 0, sizeof *cap);

	while (regs->reg != (u16)-1 ) {
		for (line=regs->type.ini_line;line<=regs->type.end_line;line++) {
			cap->service_lines[0][line] |= regs->type.vbi_type;
		}
		cap->service_set |= regs->type.vbi_type;

		regs++;
	}
	return 0;
}

/* Set vbi processing
 * type - one of tvp5150_vbi_types
 * line - line to gather data
 * fields: bit 0 field1, bit 1, field2
 * flags (default=0xf0) is a bitmask, were set means:
 *	bit 7: enable filtering null bytes on CC
 *	bit 6: send data also to FIFO
 *	bit 5: don't allow data with errors on FIFO
 *	bit 4: enable ECC when possible
 * pix_align = pix alignment:
 *	LSB = field1
 *	MSB = field2
 */
static int tvp5150_set_vbi(struct v4l2_subdev *sd,
			const struct i2c_vbi_ram_value *regs,
			unsigned int type,u8 flags, int line,
			const int fields)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	v4l2_std_id std = decoder->norm;
	u8 reg;
	int pos=0;

	if (std == V4L2_STD_ALL) {
		v4l2_err(sd, "VBI can't be configured without knowing number of lines\n");
		return 0;
	} else if (std & V4L2_STD_625_50) {
		/* Don't follow NTSC Line number convension */
		line += 3;
	}

	if (line<6||line>27)
		return 0;

	while (regs->reg != (u16)-1 ) {
		if ((type & regs->type.vbi_type) &&
		    (line>=regs->type.ini_line) &&
		    (line<=regs->type.end_line)) {
			type=regs->type.vbi_type;
			break;
		}

		regs++;
		pos++;
	}
	if (regs->reg == (u16)-1)
		return 0;

	type=pos | (flags & 0xf0);
	reg=((line-6)<<1)+TVP5150_LINE_MODE_INI;

	if (fields&1) {
		tvp5150_write(sd, reg, type);
	}

	if (fields&2) {
		tvp5150_write(sd, reg+1, type);
	}

	return type;
}

static int tvp5150_get_vbi(struct v4l2_subdev *sd,
			const struct i2c_vbi_ram_value *regs, int line)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	v4l2_std_id std = decoder->norm;
	u8 reg;
	int pos, type = 0;

	if (std == V4L2_STD_ALL) {
		v4l2_err(sd, "VBI can't be configured without knowing number of lines\n");
		return 0;
	} else if (std & V4L2_STD_625_50) {
		/* Don't follow NTSC Line number convension */
		line += 3;
	}

	if (line < 6 || line > 27)
		return 0;

	reg = ((line - 6) << 1) + TVP5150_LINE_MODE_INI;

	pos = tvp5150_read(sd, reg) & 0x0f;
	if (pos < 0x0f)
		type = regs[pos].type.vbi_type;

	pos = tvp5150_read(sd, reg + 1) & 0x0f;
	if (pos < 0x0f)
		type |= regs[pos].type.vbi_type;

	return type;
}

static int tvp5150_set_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	int fmt = 0;

	decoder->norm = std;

	/* First tests should be against specific std */

	if (std == V4L2_STD_ALL) {
		fmt = 0;	/* Autodetect mode */
	} else if (std & V4L2_STD_NTSC_443) {
		fmt = 0xa;
	} else if (std & V4L2_STD_PAL_M) {
		fmt = 0x6;
	} else if (std & (V4L2_STD_PAL_N | V4L2_STD_PAL_Nc)) {
		fmt = 0x8;
	} else {
		/* Then, test against generic ones */
		if (std & V4L2_STD_NTSC)
			fmt = 0x2;
		else if (std & V4L2_STD_PAL)
			fmt = 0x4;
		else if (std & V4L2_STD_SECAM)
			fmt = 0xc;
	}

	v4l2_dbg(1, debug, sd, "Set video std register to %d.\n", fmt);
	tvp5150_write(sd, TVP5150_VIDEO_STD, fmt);
	return 0;
}

static int tvp5150_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct tvp5150 *decoder = to_tvp5150(sd);

	if (decoder->norm == std)
		return 0;

	return tvp5150_set_std(sd, std);
}

static int tvp5150_reset(struct v4l2_subdev *sd, u32 val)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	u8 msb_id, lsb_id, msb_rom, lsb_rom;

	msb_id = tvp5150_read(sd, TVP5150_MSB_DEV_ID);
	lsb_id = tvp5150_read(sd, TVP5150_LSB_DEV_ID);
	msb_rom = tvp5150_read(sd, TVP5150_ROM_MAJOR_VER);
	lsb_rom = tvp5150_read(sd, TVP5150_ROM_MINOR_VER);

	if (msb_rom == 4 && lsb_rom == 0) { /* Is TVP5150AM1 */
		v4l2_info(sd, "tvp%02x%02xam1 detected.\n", msb_id, lsb_id);

		/* ITU-T BT.656.4 timing */
		tvp5150_write(sd, TVP5150_REV_SELECT, 0);
		/* Initializes TVP515X to its default values */
		tvp5150_write_inittab(sd, tvp5150_init_default);
	} else {
		if (msb_rom == TVP515X_CHIP_ID_MSB || lsb_rom == TVP5151_CHIP_ID_LSB) {
			/* Is TVP5151 */
			v4l2_info(sd, "tvp%02x%02x detected.\n", msb_id, lsb_id);
			v4l2_info(sd, "*** Rom ver is %d.%d\n", msb_rom, lsb_rom);
		} else {
			if (msb_rom == 3 || lsb_rom == 0x21) { /* Is TVP5150A */
				v4l2_info(sd, "tvp%02x%02xa detected.\n", msb_id, lsb_id);
			} else {
				v4l2_info(sd, "*** unknown tvp%02x%02x chip detected.\n",
					msb_id, lsb_id);
				v4l2_info(sd, "*** Rom ver is %d.%d\n", msb_rom, lsb_rom);
			}
		}
		tvp5150_write_inittab(sd, tvp5151_init_enable);
	}

	/* Initializes VDP registers */
	tvp5150_vdp_init(sd, vbi_ram_default);

	/* Selects decoder input */
	tvp5150_selmux(sd);


	/* Initialize image preferences */
	tvp5150_write(sd, TVP5150_BRIGHT_CTL, decoder->bright);
	tvp5150_write(sd, TVP5150_CONTRAST_CTL, decoder->contrast);
	tvp5150_write(sd, TVP5150_SATURATION_CTL, decoder->contrast);
	tvp5150_write(sd, TVP5150_HUE_CTL, decoder->hue);

	tvp5150_set_std(sd, decoder->norm);
	return 0;
};

static int tvp5150_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	v4l2_dbg(1, debug, sd, "g_ctrl called\n");

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = tvp5150_read(sd, TVP5150_BRIGHT_CTL);
		return 0;
	case V4L2_CID_CONTRAST:
		ctrl->value = tvp5150_read(sd, TVP5150_CONTRAST_CTL);
		return 0;
	case V4L2_CID_SATURATION:
		ctrl->value = tvp5150_read(sd, TVP5150_SATURATION_CTL);
		return 0;
	case V4L2_CID_HUE:
		ctrl->value = tvp5150_read(sd, TVP5150_HUE_CTL);
		return 0;
	}
	return -EINVAL;
}

static int tvp5150_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	u8 i, n;
	n = ARRAY_SIZE(tvp5150_qctrl);

	for (i = 0; i < n; i++) {
		if (ctrl->id != tvp5150_qctrl[i].id)
			continue;
		if (ctrl->value < tvp5150_qctrl[i].minimum ||
		    ctrl->value > tvp5150_qctrl[i].maximum)
			return -ERANGE;
		v4l2_dbg(1, debug, sd, "s_ctrl: id=%d, value=%d\n",
					ctrl->id, ctrl->value);
		break;
	}

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		tvp5150_write(sd, TVP5150_BRIGHT_CTL, ctrl->value);
		return 0;
	case V4L2_CID_CONTRAST:
		tvp5150_write(sd, TVP5150_CONTRAST_CTL, ctrl->value);
		return 0;
	case V4L2_CID_SATURATION:
		tvp5150_write(sd, TVP5150_SATURATION_CTL, ctrl->value);
		return 0;
	case V4L2_CID_HUE:
		tvp5150_write(sd, TVP5150_HUE_CTL, ctrl->value);
		return 0;
	}
	return -EINVAL;
}

/**
 * tvp5150_get_current_std() : Get the current standard detected by TVP5150
 * @sd: ptr to v4l2_subdev struct
 *
 * Get current standard detected by TVP5150, STD_INVALID if there is no
 * standard detected.
 */
static enum tvp5150_std tvp5150_get_current_std(struct v4l2_subdev *sd)
{
	u8 std, std_status;

	std = tvp5150_read(sd, TVP5150_VIDEO_STD);

	if ((std & VIDEO_STD_MASK) == VIDEO_STD_AUTO_SWITCH_BIT) {
		/* use the standard status register */
		std_status = tvp5150_read(sd, TVP5150_STATUS_REG_5);
	} else {

		/* use the standard register itself */
		std_status = std;
	}

	switch (std_status & VIDEO_STD_MASK) {
	case VIDEO_STD_NTSC_MJ_BIT:
		return STD_NTSC_MJ;
	case VIDEO_STD_PAL_BDGHIN_BIT:
		return STD_PAL_BDGHIN;
	default:
		return STD_INVALID;
	}

	return STD_INVALID;
}

/****************************************************************************
			I2C Command
 ****************************************************************************/

/**
 * tvp5150_detect() - Detect if an tvp5150 is present, and if so which revision.
 * @sd: pointer to standard V4L2 sub-device structure
 * @decoder: pointer to tvp5150_decoder structure
 *
 * A device is considered to be detected if the chip ID (LSB and MSB)
 * registers match the expected values.
 * Any value of the rom version register is accepted.
 * Returns ENODEV error number if no device is detected, or zero
 * if a device is detected.
 */
static int tvp5150_detect(struct v4l2_subdev *sd,
		struct tvp5150 *decoder)
{
	u8 chip_id_msb, chip_id_lsb, rom_ver_maj, rom_ver_min;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	chip_id_msb = tvp5150_read(sd, TVP5150_MSB_DEV_ID);
	chip_id_lsb = tvp5150_read(sd, TVP5150_LSB_DEV_ID);
	rom_ver_maj = tvp5150_read(sd, TVP5150_ROM_MAJOR_VER);
	rom_ver_min = tvp5150_read(sd, TVP5150_ROM_MINOR_VER);

	v4l2_dbg(1, debug, sd,
		 "chip id detected msb:0x%x lsb:0x%x rom version:0x%x.0x%x\n",
		 chip_id_msb, chip_id_lsb, rom_ver_maj, rom_ver_min);
	if ((chip_id_msb != TVP515X_CHIP_ID_MSB)
		|| ((chip_id_lsb != TVP5150_CHIP_ID_LSB)
		&& (chip_id_lsb != TVP5151_CHIP_ID_LSB))) {
		/* We didn't read the values we expected, so this must not be
		 * an TVP515x.
		 */
		v4l2_err(sd, "chip id mismatch msb:0x%x lsb:0x%x\n",
				chip_id_msb, chip_id_lsb);
		return -ENODEV;
	}

	decoder->ver_maj = rom_ver_maj;
	decoder->ver_min = rom_ver_min;

	v4l2_dbg(1, debug, sd, "%s (Version - 0x%.2x,0x%.2x) found at 0x%x (%s)\n",
			client->name, decoder->ver_maj, decoder->ver_min,
			client->addr << 1, client->adapter->name);
	return 0;
}

/**
 * tvp5150_querystd() - V4L2 decoder interface handler for querystd
 * @sd: pointer to standard V4L2 sub-device structure
 * @std_id: standard V4L2 std_id ioctl enum
 *
 * Returns the current standard detected by TVP5150. If no active input is
 * detected, returns -EINVAL
 */
static int tvp5150_querystd(struct v4l2_subdev *sd, v4l2_std_id *std_id)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	enum tvp5150_std current_std;
	enum tvp5150_input input_sel;
	u8 sync_lock_status, lock_mask;
	if (std_id == NULL)
		return -EINVAL;

//	tvp5150_write(sd, TVP5150_VIDEO_STD,VIDEO_STD_AUTO_SWITCH_BIT);
//
//	msleep(LOCK_RETRY_DELAY);
//
//        printk("%s -%d\n", __func__, __LINE__);
	/* get the current standard */
	current_std = tvp5150_get_current_std(sd);
	if (current_std == STD_INVALID)
		return -EINVAL;

	input_sel = decoder->input;

	switch (input_sel) {
	case TVP5150_COMPOSITE0:
	case TVP5150_COMPOSITE1:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case TVP5150_SVIDEO:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;
		/*Need to add other interfaces*/
	default:
		return -EINVAL;
	}
	/* check whether signal is locked */
	sync_lock_status = tvp5150_read(sd, TVP5150_STATUS_REG_1);
	if (lock_mask != (sync_lock_status & lock_mask))
		return -EINVAL;	/* No input detected */

	decoder->current_std = current_std;
	*std_id = decoder->std_list[current_std].standard.id;

	v4l2_dbg(1, debug, sd, "Current STD: %s",
			decoder->std_list[current_std].standard.name);
	return 0;
}

static int tvp5150_s_routing(struct v4l2_subdev *sd,
			     u32 input, u32 output, u32 config)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	enum tvp5150_input input_sel;
	enum tvp5150_output output_sel;
	enum tvp5150_std current_std = STD_INVALID;
	u8 sync_lock_status, lock_mask;
	int try_count = LOCK_RETRY_COUNT;

	decoder->input = input;
	decoder->output = output;
	tvp5150_selmux(sd);

	if ((input >= TVP5150_INPUT_INVALID) ||
			(output >= TVP5150_OUTPUT_INVALID))
		/* Index out of bound */
		return -EINVAL;

	/*
	 * For the sequence streamon -> streamoff and again s_input, most of
	 * the time, it fails to lock the signal, since streamoff puts TVP5150
	 * into power off state which leads to failure in sub-sequent s_input.
	 */
	tvp5150_s_stream(sd, 1);

	/*
	 * Since this api is goint to detect the input, it is required
	 * to set the standard in the auto switch mode
	 */
	tvp5150_write(sd, TVP5150_VIDEO_STD,
		VIDEO_STD_AUTO_SWITCH_BIT);

	msleep(LOCK_RETRY_DELAY);

	input_sel = input;
	output_sel = output;

	tvp5150_write(sd, TVP5150_VD_IN_SRC_SEL_1, input_sel);

#if 0
	/* a che serve? */

	output_sel |= tvp5150_read(sd,
			REG_OUTPUT_FORMATTER1) & 0x7;
	tvp5150_write(sd, REG_OUTPUT_FORMATTER1,
			output_sel);
	if (err)
		return err;

	decoder->tvp5150_regs[REG_INPUT_SEL].val = input_sel;
	decoder->tvp5150_regs[REG_OUTPUT_FORMATTER1].val = output_sel;

	/* Clear status */
	msleep(LOCK_RETRY_DELAY);
	err =
	    tvp5150_write(sd, REG_CLEAR_LOST_LOCK, 0x01);
	if (err)
		return err;
#endif

	switch (input_sel) {
	case TVP5150_COMPOSITE0:
	case TVP5150_COMPOSITE1:
		lock_mask = STATUS_CLR_SUBCAR_LOCK_BIT |
			STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;

	case TVP5150_SVIDEO:
		lock_mask = STATUS_HORZ_SYNC_LOCK_BIT |
			STATUS_VIRT_SYNC_LOCK_BIT;
		break;
	/* Need to add other interfaces*/
	default:
		return -EINVAL;
	}

	while (try_count-- > 0) {
		/* Allow decoder to sync up with new input */
		msleep(LOCK_RETRY_DELAY);

		/* get the current standard for future reference */
		current_std = tvp5150_get_current_std(sd);
		if (current_std == STD_INVALID)
			continue;

		sync_lock_status = tvp5150_read(sd,
				TVP5150_STATUS_REG_1);
		if (lock_mask == (sync_lock_status & lock_mask))
			/* Input detected */
			break;
	}

	if ((current_std == STD_INVALID) || (try_count < 0))
		return -EINVAL;

	decoder->current_std = current_std;
	decoder->input = input;
	decoder->output = output;

	v4l2_dbg(1, debug, sd, "Input set to: %d, std : %d",
			input_sel, current_std);

	return 0;
}

static int tvp5150_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	struct v4l2_sliced_vbi_format *svbi;
	struct v4l2_pix_format *pix;
	struct tvp5150 *decoder = to_tvp5150(sd);
	int i;
	int rval;

	/* capture */
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk("%s-%d V4L2_BUF_TYPE_VIDEO_CAPTURE\n", __func__, __LINE__);
		pix = &fmt->fmt.pix;
		rval = tvp5150_try_fmt_cap(sd, fmt);
		if (rval)
			return rval;
		decoder->pix = *pix;
		return 0;
	}

	/* raw vbi */
	if (fmt->type == V4L2_BUF_TYPE_VBI_CAPTURE) {
		/* this is for capturing 36 raw vbi lines
		   if there's a way to cut off the beginning 2 vbi lines
		   with the tvp5150 then the vbi line count could be lowered
		   to 17 lines/field again, although I couldn't find a register
		   which could do that cropping */
		if (fmt->fmt.vbi.sample_format == V4L2_PIX_FMT_GREY)
			tvp5150_write(sd, TVP5150_LUMA_PROC_CTL_1, 0x70);
		if (fmt->fmt.vbi.count[0] == 18 && fmt->fmt.vbi.count[1] == 18) {
			tvp5150_write(sd, TVP5150_VERT_BLANKING_START, 0x00);
			tvp5150_write(sd, TVP5150_VERT_BLANKING_STOP, 0x01);
		}
		return 0;
	}
	if (fmt->type != V4L2_BUF_TYPE_SLICED_VBI_CAPTURE)
		return -EINVAL;
	svbi = &fmt->fmt.sliced;
	if (svbi->service_set != 0) {
		for (i = 0; i <= 23; i++) {
			svbi->service_lines[1][i] = 0;
			svbi->service_lines[0][i] =
				tvp5150_set_vbi(sd, vbi_ram_default,
				       svbi->service_lines[0][i], 0xf0, i, 3);
		}
		/* Enables FIFO */
		tvp5150_write(sd, TVP5150_FIFO_OUT_CTRL, 1);
	} else {
		/* Disables FIFO*/
		tvp5150_write(sd, TVP5150_FIFO_OUT_CTRL, 0);

		/* Disable Full Field */
		tvp5150_write(sd, TVP5150_FULL_FIELD_ENA, 0);

		/* Disable Line modes */
		for (i = TVP5150_LINE_MODE_INI; i <= TVP5150_LINE_MODE_END; i++)
			tvp5150_write(sd, i, 0xff);
	}
	return 0;
}

static int tvp5150_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	struct v4l2_sliced_vbi_format *svbi;
	struct tvp5150 *decoder = to_tvp5150(sd);
	int i, mask = 0;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
		svbi = &fmt->fmt.sliced;
		memset(svbi, 0, sizeof(*svbi));

		for (i = 0; i <= 23; i++) {
			svbi->service_lines[0][i] =
				tvp5150_get_vbi(sd, vbi_ram_default, i);
			mask |= svbi->service_lines[0][i];
		}
		svbi->service_set = mask;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		fmt->fmt.pix = decoder->pix;

		v4l2_dbg(1, debug, sd, "Current FMT: bytesperline - %d"
			"Width - %d, Height - %d",
			decoder->pix.bytesperline,
			decoder->pix.width, decoder->pix.height);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static int tvp5150_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	int rev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	rev = tvp5150_read(sd, TVP5150_ROM_MAJOR_VER) << 8 |
	      tvp5150_read(sd, TVP5150_ROM_MINOR_VER);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_TVP5150,
					  rev);
}


#ifdef CONFIG_VIDEO_ADV_DEBUG
static int tvp5150_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	reg->val = tvp5150_read(sd, reg->reg & 0xff);
	reg->size = 1;
	return 0;
}

static int tvp5150_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	tvp5150_write(sd, reg->reg & 0xff, reg->val & 0xff);
	return 0;
}
#endif

static int tvp5150_g_tuner(struct v4l2_subdev *sd, struct v4l2_tuner *vt)
{
	int status = tvp5150_read(sd, 0x88);

	vt->signal = ((status & 0x04) && (status & 0x02)) ? 0xffff : 0x0;
	return 0;
}

static int tvp5150_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	v4l2_dbg(1, debug, sd, "queryctrl called\n");

	for (i = 0; i < ARRAY_SIZE(tvp5150_qctrl); i++)
		if (qc->id && qc->id == tvp5150_qctrl[i].id) {
			memcpy(qc, &(tvp5150_qctrl[i]),
			       sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}

/**
 * tvp5150_try_fmt_cap() - V4L2 decoder interface handler for try_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type. This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int
tvp5150_try_fmt_cap(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	int ifmt;
	struct v4l2_pix_format *pix;
	enum tvp5150_std current_std;

	if (f == NULL)
		return -EINVAL;
printk("%s-%d\n", __func__, __LINE__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix = &f->fmt.pix;

	/* Calculate height and width based on current standard */
	current_std = tvp5150_get_current_std(sd);
	if (current_std == STD_INVALID)
		return -EINVAL;
printk("%s-%d\n", __func__, __LINE__);

	decoder->current_std = current_std;
	pix->width = decoder->std_list[current_std].width;
	pix->height = decoder->std_list[current_std].height;

	for (ifmt = 0; ifmt < decoder->num_fmts; ifmt++) {
		if (pix->pixelformat ==
			decoder->fmt_list[ifmt].pixelformat)
			break;
	}

printk("%s-%d\n", __func__, __LINE__);
	if (ifmt == decoder->num_fmts)
		/* None of the format matched, select default */
		ifmt = 0;
	pix->pixelformat = decoder->fmt_list[ifmt].pixelformat;

	pix->field = V4L2_FIELD_INTERLACED;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->priv = 0;

printk("%s-%d\n", __func__, __LINE__);
	//v4l2_dbg(1, debug, sd, "Try FMT: pixelformat - %s, bytesperline - %d"
	printk("Try FMT: pixelformat - %s, bytesperline - %d"
			"Width - %d, Height - %d",
			decoder->fmt_list[ifmt].description, pix->bytesperline,
			pix->width, pix->height);

	return 0;
}

/**
 * tvp5150_g_parm() - V4L2 decoder interface handler for g_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the decoder's video CAPTURE parameters.
 */
static int
tvp5150_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	struct v4l2_captureparm *cparm;
	enum tvp5150_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* get the current standard */
	current_std = tvp5150_get_current_std(sd);
	if (current_std == STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	cparm = &a->parm.capture;
	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe =
		decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * tvp5150_s_parm() - V4L2 decoder interface handler for s_parm
 * @sd: pointer to standard V4L2 sub-device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the decoder to use the input parameters, if possible. If
 * not possible, returns the appropriate error code.
 */
static int
tvp5150_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	struct v4l2_fract *timeperframe;
	enum tvp5150_std current_std;

	if (a == NULL)
		return -EINVAL;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	timeperframe = &a->parm.capture.timeperframe;

	/* get the current standard */
	current_std = tvp5150_get_current_std(sd);
	if (current_std == STD_INVALID)
		return -EINVAL;

	decoder->current_std = current_std;

	*timeperframe =
	    decoder->std_list[current_std].standard.frameperiod;

	return 0;
}

/**
 * tvp5150_s_stream() - V4L2 decoder i/f handler for s_stream
 * @sd: pointer to standard V4L2 sub-device structure
 * @enable: streaming enable or disable
 *
 * Sets streaming to enable or disable, if possible.
 */
static int tvp5150_s_stream(struct v4l2_subdev *sd, int enable)
{
	int err = 0;
	struct tvp5150 *decoder = to_tvp5150(sd);

	if (decoder->streaming == enable)
		return 0;

	switch (enable) {
	case 0:
	{
		/* Power Down Sequence */
		tvp5150_write(sd, TVP5150_OP_MODE_CTL, 0x01);
		decoder->streaming = enable;

		/* putting the decoder in power down mode */
		gpio_set_value(decoder->pdata->pdn, 0);
		break;
	}
	case 1:
	{
		/* putting the decoder in operative mode */
		gpio_set_value(decoder->pdata->pdn, 1);
		mdelay(25);		/* manual declares for tvp5151 20msec+200usec */

		/* Power Up Sequence */
		tvp5150_write(sd, TVP5150_OP_MODE_CTL, 0x00);

		/* Detect if not already detected */
		err = tvp5150_detect(sd, decoder);
		if (err) {
			v4l2_err(sd, "Unable to detect decoder\n");
			return err;
		}

		/* TODO FIXING TVP5150 support */
		tvp5150_write_inittab(sd, tvp5151_init_enable);
		if (err) {
			v4l2_err(sd, "Unable to configure decoder\n");
			return err;
		}
		decoder->streaming = enable;
		break;
	}
	default:
		err = -ENODEV;
		break;
	}

	return err;
}



/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops tvp5150_core_ops = {
	.log_status = tvp5150_log_status,
	.g_ctrl = tvp5150_g_ctrl,
	.s_ctrl = tvp5150_s_ctrl,
	.queryctrl = tvp5150_queryctrl,
	.s_std = tvp5150_s_std,
	.reset = tvp5150_reset,
	.g_chip_ident = tvp5150_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tvp5150_g_register,
	.s_register = tvp5150_s_register,
#endif
};

static const struct v4l2_subdev_tuner_ops tvp5150_tuner_ops = {
	.g_tuner = tvp5150_g_tuner,
};

/**
 * List of image formats supported by TVP5150 decoder
 */
static const struct v4l2_fmtdesc tvp5150_fmt_list[] = {
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

/**
 * tvp5150_enum_fmt_cap() - V4L2 decoder interface handler for enum_fmt
 * @sd: pointer to standard V4L2 sub-device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl to enumerate supported formats
 */
static int
tvp5150_enum_fmt_cap(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmt)
{
	struct tvp5150 *decoder = to_tvp5150(sd);
	int index;

	if (fmt == NULL)
		return -EINVAL;

	index = fmt->index;
	if ((index >= decoder->num_fmts) || (index < 0))
		/* Index out of bound */
		return -EINVAL;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;
	memcpy(fmt, &decoder->fmt_list[index],
	sizeof(struct v4l2_fmtdesc));

	v4l2_dbg(1, debug, sd, "Current FMT: index - %d (%s)",
			decoder->fmt_list[index].index,
			decoder->fmt_list[index].description);
	return 0;
}

static const struct v4l2_subdev_video_ops tvp5150_video_ops = {
	.s_routing = tvp5150_s_routing,
	.querystd = tvp5150_querystd,
	.enum_fmt = tvp5150_enum_fmt_cap,
	.g_fmt = tvp5150_g_fmt,
	.try_fmt = tvp5150_try_fmt_cap,
	.s_fmt = tvp5150_s_fmt,
	.g_parm = tvp5150_g_parm,
	.s_parm = tvp5150_s_parm,
	.s_stream = tvp5150_s_stream,
	.g_sliced_vbi_cap = tvp5150_g_sliced_vbi_cap,
};

static const struct v4l2_subdev_ops tvp5150_ops = {
	.core = &tvp5150_core_ops,
	.tuner = &tvp5150_tuner_ops,
	.video = &tvp5150_video_ops,
};

static struct tvp5150 tvp5150_dev = {
	.streaming = 0,
	.norm = V4L2_STD_ALL,	/* Default is autodetect */
	.input = TVP5150_COMPOSITE0,
	.enable = 1,
	.bright = 128,
	.contrast = 128,
	.hue = 0,
	.sat = 128,

	.fmt_list = tvp5150_fmt_list,
	.num_fmts = ARRAY_SIZE(tvp5150_fmt_list),
        .pix = {
                /* Default to PAL 8-bit YUV 422 */
                .width = PAL_NUM_ACTIVE_PIXELS,
                .height = PAL_NUM_ACTIVE_LINES,
                .pixelformat = V4L2_PIX_FMT_UYVY,
                .field = V4L2_FIELD_INTERLACED,
                .bytesperline = PAL_NUM_ACTIVE_PIXELS * 2,
                .sizeimage =
                PAL_NUM_ACTIVE_PIXELS * 2 * PAL_NUM_ACTIVE_LINES,
                .colorspace = V4L2_COLORSPACE_SMPTE170M,
                },
        .current_std = STD_PAL_BDGHIN,
        .std_list = tvp5150_std_list,
        .num_stds = ARRAY_SIZE(tvp5150_std_list),

};

/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/

static int tvp5150_probe(struct i2c_client *c,
			 const struct i2c_device_id *id)
{
	struct tvp5150 *core;
	struct v4l2_subdev *sd;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(c->adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

        if (!c->dev.platform_data) {
                v4l2_err(c, "No platform data!!\n");
                return -ENODEV;
        }

	core = kzalloc(sizeof(struct tvp5150), GFP_KERNEL);
	if (!core) {
		return -ENOMEM;
	} else {

		/* Initialize the tvp5150_decoder with default configuration */
		*core = tvp5150_dev;

		/* Copy board specific information here */
		core->pdata = c->dev.platform_data;
		i2c_set_clientdata(c, core);
        }

	sd = &core->sd;
	v4l2_i2c_subdev_init(sd, c, &tvp5150_ops);
	v4l_info(c, "chip found @ 0x%02x (%s)\n",
		 c->addr << 1, c->adapter->name);

	core->norm = V4L2_STD_ALL;	/* Default is autodetect */
	core->input = TVP5150_COMPOSITE1;
	core->enable = 1;
	core->bright = 128;
	core->contrast = 128;
	core->hue = 0;
	core->sat = 128;

	/* putting the decoder in power down mode */
	gpio_set_value(core->pdata->pdn, 0);

	if (debug > 1)
		tvp5150_log_status(sd);
	return 0;
}

static int tvp5150_remove(struct i2c_client *c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(c);

	v4l2_dbg(1, debug, sd,
		"tvp5150.c: removing tvp5150 adapter on address 0x%x\n",
		c->addr << 1);

	v4l2_device_unregister_subdev(sd);
	kfree(to_tvp5150(sd));
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id tvp5150_id[] = {
	{ "tvp5150", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tvp5150_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = "tvp5150",
	.probe = tvp5150_probe,
	.remove = tvp5150_remove,
	.id_table = tvp5150_id,
};
