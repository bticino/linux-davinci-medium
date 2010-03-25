/*
 * Copyright (C) 2007-2009 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/davinci/videohd.h>
#include <media/tvp7002.h>

/* Module Name */
#define TVP7002_MODULE_NAME		"tvp7002"

/* Private macros for TVP */
#define I2C_RETRY_COUNT                 (5)
#define LOCK_RETRY_COUNT                (5)
#define LOCK_RETRY_DELAY                (200)

static int tvp7002_set_format_params(struct v4l2_subdev *sd,
				struct tvp7002_format_params *tvpformats);
static int tvp7002_setstd(struct v4l2_subdev *sd, v4l2_std_id id);

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

struct tvp7002_std_info {
	unsigned long width;
	unsigned long height;
	struct v4l2_standard standard;
};

struct i2c_reg_value {
	u8 reg;
	u32 val;
};

struct tvp7002_decoder {
	struct v4l2_subdev sd;
	const struct tvp7002_platform_data *pdata;

	int ver;
	int streaming;

	struct v4l2_pix_format pix;
	int num_fmts;
	const struct v4l2_fmtdesc *fmt_list;

	int current_std;
	/*
	int num_stds;
	struct tvp7002_std_info *std_list;
	*/
	/* Input and Output Routing parameters */
	u32 input;
	u32 output;
};

#if 0
/* Register default values (according to tvp7002 datasheet) */
static const struct i2c_reg_value tvp7002_init_default[] = {
	/* 0x00: read only */
	{ TVP7002_HPLL_FDBK_DIV_MSBS, 0x67 },
	{ TVP7002_HPLL_FDBK_DIV_LSBS, 0x20 },
	{ TVP7002_HPLL_CRTL, 0xa8 },
	{ TVP7002_HPLL_PHASE_SEL, 0x80 },
	{ TVP7002_CLAMP_START, 0x32 },
	{ TVP7002_CLAMP_W, 0x20 },
	{ TVP7002_HSYNC_OUT_W, 0x20 },
	{ TVP7002_B_FINE_GAIN, 0x00 },
	{ TVP7002_G_FINE_GAIN, 0x00 },
	{ TVP7002_R_FINE_GAIN, 0x00 },
	{ TVP7002_B_FINE_OFF_MSBS, 0x80 },
	{ TVP7002_G_FINE_OFF_MSBS, 0x80 },
	{ TVP7002_R_FINE_OFF_MSBS, 0x80 },
	{ TVP7002_SYNC_CTL_1, 0x5b },
	{ TVP7002_HPLL_AND_CLAMP_CTL, 0x2e },
	{ TVP7002_SYNC_ON_G_THRS, 0x5d },
	{ TVP7002_SYNC_SEPARATOR_THRS, 0x20 },
	{ TVP7002_HPLL_PRE_COAST, 0x00 },
	{ TVP7002_HPLL_POST_COAST, 0x00 },
	/* 0x14: read only */
	{ TVP7002_OUT_FORMATTER, 0x00 },
	{ TVP7002_MISC_CTL_1, 0x11 },
	{ TVP7002_MISC_CTL_2, 0x03 },
	{ TVP7002_MISC_CTL_3, 0x00 },
	{ TVP7002_IN_MUX_SEL_1, 0x00 },
	{ TVP7002_IN_MUX_SEL_2, 0xc2 },
	{ TVP7002_B_AND_G_COARSE_GAIN, 0x77 },
	{ TVP7002_R_COARSE_GAIN, 0x07 },
	{ TVP7002_COARSE_CLAMP_CTL, 0x00 },
	{ TVP7002_FINE_OFF_LSBS, 0x00 },
	{ TVP7002_B_COARSE_OFF, 0x10 },
	{ TVP7002_G_COARSE_OFF, 0x10 },
	{ TVP7002_R_COARSE_OFF, 0x10 },
	{ TVP7002_HSOUT_OUT_START, 0x0d	},
	{ TVP7002_MISC_CTL_4, 0x0d },
	/* 0x23: read only */
	/* 0x24: read only */
	/* 0x25: read only */
	{ TVP7002_AUTO_LVL_CTL_ENABLE, 0x80 },
	/* 0x27: read only */
	{ TVP7002_AUTO_LVL_CTL_FILTER, 0x53 },
	{ TVP7002_FINE_CLAMP_CTL, 0x07 },
	{ TVP7002_PWR_CTL, 0x00 },
	{ TVP7002_ADC_SETUP, 0x50 },
	{ TVP7002_COARSE_CLAMP_CTL, 0x00 },
	{ TVP7002_SOG_CLAMP, 0x80 },
	{ TVP7002_RGB_COARSE_CLAMP_CTL, 0x8c },
	{ TVP7002_SOG_COARSE_CLAMP_CTL, 0x04 },
	{ TVP7002_ALC_PLACEMENT, 0x5a },
	{ TVP7002_MVIS_STRIPPER_W, 0x03 },
	{ TVP7002_VSYNC_ALGN, 0x10 },
	{ TVP7002_SYNC_BYPASS, 0x00 },
	/* 0x37: read only */
	/* 0x38: read only */
	/* 0x39: read only */
	/* 0x3a: read only */
	/* 0x3b: read only */
	/* 0x3c: read only */
	{ TVP7002_L_LENGTH_TOL, 0x03 },
	{ TVP7002_VIDEO_BWTH_CTL, 0x00 },
	{ TVP7002_AVID_START_PIXEL_LSBS, 0x01 },
	{ TVP7002_AVID_START_PIXEL_MSBS, 0x2c },
	{ TVP7002_AVID_STOP_PIXEL_LSBS, 0x06 },
	{ TVP7002_AVID_STOP_PIXEL_MSBS, 0x2c },
	{ TVP7002_VBLK_F_0_START_L_OFF, 0x05 },
	{ TVP7002_VBLK_F_1_START_L_OFF, 0x05 },
	{ TVP7002_VBLK_F_0_DURATION, 0x1e },
	{ TVP7002_VBLK_F_1_DURATION, 0x1e },
	{ TVP7002_FBIT_F_0_START_L_OFF, 0x00 },
	{ TVP7002_FBIT_F_1_START_L_OFF, 0x00 },
	{ TVP7002_YUV_Y_G_COEF_LSBS, 0xe3 },
	{ TVP7002_YUV_Y_G_COEF_MSBS, 0x16 },
	{ TVP7002_YUV_Y_B_COEF_LSBS, 0x4f },
	{ TVP7002_YUV_Y_B_COEF_MSBS, 0x02 },
	{ TVP7002_YUV_Y_R_COEF_LSBS, 0xce },
	{ TVP7002_YUV_Y_R_COEF_MSBS, 0x06 },
	{ TVP7002_YUV_U_G_COEF_LSBS, 0xab },
	{ TVP7002_YUV_U_G_COEF_MSBS, 0xf3 },
	{ TVP7002_YUV_U_B_COEF_LSBS, 0x00 },
	{ TVP7002_YUV_U_B_COEF_MSBS, 0x10 },
	{ TVP7002_YUV_U_R_COEF_LSBS, 0x55 },
	{ TVP7002_YUV_U_R_COEF_MSBS, 0xfc },
	{ TVP7002_YUV_V_G_COEF_LSBS, 0x78 },
	{ TVP7002_YUV_V_G_COEF_MSBS, 0xf1 },
	{ TVP7002_YUV_V_B_COEF_LSBS, 0x88 },
	{ TVP7002_YUV_V_B_COEF_MSBS, 0xfe },
	{ TVP7002_YUV_V_R_COEF_LSBS, 0x00 },
	{ TVP7002_YUV_V_R_COEF_MSBS, 0x10 },
	{ 0x5c, 0x00 }	/* end of registers */
};
#endif

static const struct v4l2_fmtdesc tvp7002_fmt_list[] = {
	{
	 .index = 0,
	 .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	 .flags = 0,
	 .description = "8-bit UYVY 4:2:2 Format",
	 .pixelformat = V4L2_PIX_FMT_UYVY,
	},
};

static struct v4l2_standard tvp7002_standards[TVP7002_MAX_NO_STANDARDS] = {
	{
		.index = 0,
		.id = V4L2_STD_720P_60,
		.name = "720P-60",
		.frameperiod = {1, 60},
		.framelines = 720
	},
	{
		.index = 1,
		.id = V4L2_STD_1080I_60,
		.name = "1080I-30",
		.frameperiod = {1, 30},
		.framelines = 1080
	},
	{
		.index = 2,
		.id = V4L2_STD_1080I_50,
		.name = "1080I-25",
		.frameperiod = {1, 25},
		.framelines = 1080
	},
	{
		.index = 3,
		.id = V4L2_STD_720P_50,
		.name = "720P-50",
		.frameperiod = {1, 50},
		.framelines = 720
	},
	{
		.index = 4,
		.id = V4L2_STD_1080P_60,
		.name = "1080P-60",
		.frameperiod = {1, 60},
		.framelines = 1080
	},
	{
		.index = 5,
		.id = V4L2_STD_525P_60,
		.name = "480P-60",
		.frameperiod = {1, 60},
		.framelines = 525
	},
	{
		.index = 6,
		.id = V4L2_STD_625P_50,
		.name = "576P-50",
		.frameperiod = {1, 50},
		.framelines = 625
	},
};

static struct tvp7002_format_params
	tvp7002_formats[TVP7002_MAX_NO_STANDARDS] = {
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_720p,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_720p,
		.hpll_vco_control = VCO_CONTROL_720p,
		.hpll_cp_current = CP_CURRENT_720p,
		.hpll_phase_select = PHASE_SELECT_720p,
		.hpll_post_divider = POST_DIVIDER_720p,
		.hpll_control = HPLL_CONTROL_720p,
		.avid_start_msb = AVID_START_PIXEL_MSB_720p,
		.avid_start_lsb = AVID_START_PIXEL_LSB_720p,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_720p,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_720p,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_720p,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_720p,
		.vblk_f0_duration = VBLK_F0_DURATION_720p,
		.vblk_f1_duration = VBLK_F1_DURATION_720p,
		.alc_placement = TVP7002_HD_ALC_PLACEMENT,
		.clamp_start = TVP7002_HD_CLAMP_START,
		.clamp_width = TVP7002_HD_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_HD_PRE_COAST,
		.hpll_post_coast = TVP7002_HD_POST_COAST,
		.reserved = RESERVED_720p
	},
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_1080i,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_1080i,
		.hpll_vco_control = VCO_CONTROL_1080i,
		.hpll_cp_current = CP_CURRENT_1080i,
		.hpll_phase_select = PHASE_SELECT_1080i,
		.hpll_post_divider = POST_DIVIDER_1080i,
		.hpll_control = HPLL_CONTROL_1080i,
		.avid_start_msb = AVID_START_PIXEL_MSB_1080i,
		.avid_start_lsb = AVID_START_PIXEL_LSB_1080i,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_1080i,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_1080i,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_1080i,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_1080i,
		.vblk_f0_duration = VBLK_F0_DURATION_1080i,
		.vblk_f1_duration = VBLK_F1_DURATION_1080i,
		.alc_placement = TVP7002_HD_ALC_PLACEMENT,
		.clamp_start = TVP7002_HD_CLAMP_START,
		.clamp_width = TVP7002_HD_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_HD_PRE_COAST,
		.hpll_post_coast = TVP7002_HD_POST_COAST,
		.reserved = RESERVED_1080i
	},
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_1080i_50,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_1080i_50,
		.hpll_vco_control = VCO_CONTROL_1080i_50,
		.hpll_cp_current = CP_CURRENT_1080i_50,
		.hpll_phase_select = PHASE_SELECT_1080i_50,
		.hpll_post_divider = POST_DIVIDER_1080i_50,
		.hpll_control = HPLL_CONTROL_1080i_50,
		.avid_start_msb = AVID_START_PIXEL_MSB_1080i_50,
		.avid_start_lsb = AVID_START_PIXEL_LSB_1080i_50,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_1080i_50,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_1080i_50,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_1080i_50,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_1080i_50,
		.vblk_f0_duration = VBLK_F0_DURATION_1080i_50,
		.vblk_f1_duration = VBLK_F1_DURATION_1080i_50,
		.alc_placement = TVP7002_HD_ALC_PLACEMENT,
		.clamp_start = TVP7002_HD_CLAMP_START,
		.clamp_width = TVP7002_HD_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_HD_PRE_COAST,
		.hpll_post_coast = TVP7002_HD_POST_COAST,
		.reserved = RESERVED_1080i_50
	},
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_720p_50,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_720p_50,
		.hpll_vco_control = VCO_CONTROL_720p_50,
		.hpll_cp_current = CP_CURRENT_720p_50,
		.hpll_phase_select = PHASE_SELECT_720p_50,
		.hpll_post_divider = POST_DIVIDER_720p_50,
		.hpll_control = HPLL_CONTROL_720p_50,
		.avid_start_msb = AVID_START_PIXEL_MSB_720p_50,
		.avid_start_lsb = AVID_START_PIXEL_LSB_720p_50,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_720p_50,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_720p_50,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_720p_50,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_720p_50,
		.vblk_f0_duration = VBLK_F0_DURATION_720p_50,
		.vblk_f1_duration = VBLK_F1_DURATION_720p_50,
		.alc_placement = TVP7002_HD_ALC_PLACEMENT,
		.clamp_start = TVP7002_HD_CLAMP_START,
		.clamp_width = TVP7002_HD_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_HD_PRE_COAST,
		.hpll_post_coast = TVP7002_HD_POST_COAST,
		.reserved = RESERVED_720p
	},
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_1080p,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_1080p,
		.hpll_vco_control = VCO_CONTROL_1080p,
		.hpll_cp_current = CP_CURRENT_1080p,
		.hpll_phase_select = PHASE_SELECT_1080p,
		.hpll_post_divider = POST_DIVIDER_1080p,
		.hpll_control = HPLL_CONTROL_1080p,
		.avid_start_msb = AVID_START_PIXEL_MSB_1080p,
		.avid_start_lsb = AVID_START_PIXEL_LSB_1080p,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_1080p,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_1080p,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_1080p,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_1080p,
		.vblk_f0_duration = VBLK_F0_DURATION_1080p,
		.vblk_f1_duration = VBLK_F1_DURATION_1080p,
		.alc_placement = TVP7002_HD_ALC_PLACEMENT,
		.clamp_start = TVP7002_HD_CLAMP_START,
		.clamp_width = TVP7002_HD_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_HD_PRE_COAST,
		.hpll_post_coast = TVP7002_HD_POST_COAST,
		.reserved = RESERVED_720p
	},
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_480P,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_480P,
		.hpll_vco_control = VCO_CONTROL_480P,
		.hpll_cp_current = CP_CURRENT_480P,
		.hpll_phase_select = PHASE_SELECT_480P,
		.hpll_post_divider = POST_DIVIDER_480P,
		.hpll_control = HPLL_CONTROL_480P,
		.avid_start_msb = AVID_START_PIXEL_MSB_480P,
		.avid_start_lsb = AVID_START_PIXEL_LSB_480P,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_480P,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_480P,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_480P,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_480P,
		.vblk_f0_duration = VBLK_F0_DURATION_480P,
		.vblk_f1_duration = VBLK_F1_DURATION_480P,
		.alc_placement = TVP7002_ED_ALC_PLACEMENT,
		.clamp_start = TVP7002_ED_CLAMP_START,
		.clamp_width = TVP7002_ED_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_ED_PRE_COAST,
		.hpll_post_coast = TVP7002_ED_POST_COAST,
		.reserved = RESERVED_1080i_50
	},
	{
		.hpll_divider_msb = FEEDBACK_DIVIDER_MSB_576P,
		.hpll_divider_lsb = FEEDBACK_DIVIDER_LSB_576P,
		.hpll_vco_control = VCO_CONTROL_576P,
		.hpll_cp_current = CP_CURRENT_576P,
		.hpll_phase_select = PHASE_SELECT_576P,
		.hpll_post_divider = POST_DIVIDER_576P,
		.hpll_control = HPLL_CONTROL_576P,
		.avid_start_msb = AVID_START_PIXEL_MSB_576P,
		.avid_start_lsb = AVID_START_PIXEL_LSB_576P,
		.avid_stop_lsb = AVID_STOP_PIXEL_LSB_576P,
		.avid_stop_msb = AVID_STOP_PIXEL_MSB_576P,
		.vblk_start_f0_line_offset = VBLK_F0_START_LINE_OFFSET_576P,
		.vblk_start_f1_line_offset = VBLK_F1_START_LINE_OFFSET_576P,
		.vblk_f0_duration = VBLK_F0_DURATION_576P,
		.vblk_f1_duration = VBLK_F1_DURATION_576P,
		.alc_placement = TVP7002_ED_ALC_PLACEMENT,
		.clamp_start = TVP7002_ED_CLAMP_START,
		.clamp_width = TVP7002_ED_CLAMP_WIDTH,
		.hpll_pre_coast = TVP7002_ED_PRE_COAST,
		.hpll_post_coast = TVP7002_ED_POST_COAST,
		.reserved = RESERVED_1080i_50
	},
};

static struct tvp7002_config tvp7002_configuration[TVP7002_NUM_CHANNELS] = {
	{
		.no_of_inputs = TVP7002_MAX_NO_INPUTS,
		.input[0] = {
			.input_type = TVP7002_HD_INPUT,
			.input_info = {
				.index = 0,
				.name = "COMPONENT",
				.type = V4L2_INPUT_TYPE_CAMERA,
				.std = V4L2_STD_TVP7002_ALL
			},
			.no_of_standard = TVP7002_MAX_NO_STANDARDS,
			.standard = (struct v4l2_standard *)&tvp7002_standards,
			.def_std = V4L2_STD_720P_60,
			.format =
			   (struct tvp7002_format_params *)&tvp7002_formats,
			.no_of_controls = TVP7002_MAX_NO_CONTROLS,
			.controls = NULL
		},
		.def_params = {V4L2_STD_720P_60, 0,
				{1, 0xa, 0x6}, {0, 0, 0, 7, 7, 7},
				{0x80, 0x80, 0x80, 0, 0, 0, 0x10, 0x10, 0x10} }
	 }
};

#if 0
static struct param_ops params_ops = {
	.setparams = tvp7002_setparams,
	.getparams = tvp7002_getparams
};
static struct control_ops controls_ops = {
	.count = TVP7002_MAX_NO_CONTROLS,
	.queryctrl = tvp7002_querycontrol,
	.setcontrol = tvp7002_setcontrol,
	.getcontrol = tvp7002_getcontrol
};
static struct input_ops inputs_ops = {
	.count = TVP7002_MAX_NO_INPUTS,
	.enuminput = tvp7002_enuminput,
	.setinput = tvp7002_setinput,
	.getinput = tvp7002_getinput
};
static struct standard_ops standards_ops = {
	.count = TVP7002_MAX_NO_STANDARDS,
	.enumstd = tvp7002_enumstd,
	.setstd = tvp7002_setstd,
	.getstd = tvp7002_getstd,
	.querystd = tvp7002_querystd,
};
#endif

static inline struct tvp7002_decoder *to_decoder(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tvp7002_decoder, sd);
}

static int tvp7002_read_reg(struct v4l2_subdev *sd, u8 reg)
{
	int err, retry = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

read_again:

	err = i2c_smbus_read_byte_data(client, reg);
	if (err == -1) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l2_warn(sd, "Read: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto read_again;
		}
	}

	return err;
}

#if 0
static void dump_reg(struct v4l2_subdev *sd, u8 reg)
{
	u32 val;

	val = tvp7002_read_reg(sd, reg);
	v4l2_info(sd, "Reg(0x%.2X): 0x%.2X\n", reg, val);
}
#endif

static int tvp7002_write_reg(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	int err, retry = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

write_again:

	err = i2c_smbus_write_byte_data(client, reg, val);
	if (err) {
		if (retry <= I2C_RETRY_COUNT) {
			v4l2_warn(sd, "Write: retry ... %d\n", retry);
			retry++;
			msleep_interruptible(10);
			goto write_again;
		}
	}

	return err;
}

#if 0
static int tvp7002_write_regs(struct v4l2_subdev *sd,
			      const struct tvp7002_reg reglist[])
{
	int err;
	const struct tvp7002_reg *next = reglist;

	for (; next->token != TOK_TERM; next++) {
		if (next->token == TOK_DELAY) {
			msleep(next->val);
			continue;
		}

		if (next->token == TOK_SKIP)
			continue;

		err = tvp7002_write_reg(sd, next->reg, (u8) next->val);
		if (err) {
			v4l2_err(sd, "Write failed. Err[%d]\n", err);
			return err;
		}
	}
	return 0;
}
#endif

/* tvp7002_initialize :
 * This function will set the video format standard
 */
static int tvp7002_initialize(struct v4l2_subdev *sd)
{
	int err = 0;
	v4l2_std_id std;

	/* Reset the chip */
	err |= tvp7002_write_reg(sd, TVP7002_POWER_CONTROL,
				     0xff);
	msleep(20);

	err |= tvp7002_write_reg(sd, TVP7002_POWER_CONTROL,
				     0x00);

	msleep(20);
	err |= tvp7002_write_reg(sd, TVP7002_HPLL_DIVIDER_MSB,
				     TVP7002_HPLL_MSB_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_HPLL_DIVIDER_LSB,
				     TVP7002_HPLL_LSB_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_HPLL_CONTROL,
				     TVP7002_HPLL_CONTROL_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_HPLL_PHASE_SELECT,
				     TVP7002_HPLL_PHASE_SEL_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_CLAMP_START,
				     TVP7002_CLAMP_START_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_CLAMP_WIDTH,
				     TVP7002_CLAMP_WIDTH_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_HSYNC_OUTPUT_WIDTH,
				     TVP7002_HSYNC_OUTWIDTH_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_SYNC_CONTROL_1,
				     TVP7002_SYNC_CONTROL1_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_HPLL_CLAMP_CONTROL,
				     TVP7002_HPLL_CLAMP_CTRL_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_SYNC_ON_GREEN_THLD,
				     TVP7002_SYNC_GREEN_THLD_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_SYNC_SEPARATER_THLD,
				  TVP7002_SYNC_SEP_THLD_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_HPLL_PRE_COAST,
				  TVP7002_HPLL_PRE_COAST_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_HPLL_POST_COAST,
				  TVP7002_HPLL_POST_COAST_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_OUTPUT_FORMATTER,
				  TVP7002_OUTPUT_FORMATTER_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_MISC_CONTROL_1,
				  TVP7002_MISC_CONTROL1_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_MISC_CONTROL_3,
				  TVP7002_MISC_CONTROL3_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_MISC_CONTROL_2,
				  TVP7002_MISC_CONTROL2_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_INPUT_MUX_SELECT_1,
				  TVP7002_INPUT_MUX_SELECT1_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_INPUT_MUX_SELECT_2,
				  TVP7002_INPUT_MUX_SELECT2_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_HSOUT_OUTPUT_START,
				     TVP7002_HSOUT_OUTPUT_START_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_MISC_CONTROL_4,
				     TVP7002_MISC_CONTROL4_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_ALC_ENABLE,
				     TVP7002_ALC_ENABLE_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_ALC_FILTER,
				     TVP7002_ALC_FILTER_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_FINE_CLAMP_CONTROL,
				     TVP7002_FINE_CLAMP_CONTROL_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_ADC_SETUP,
				     TVP7002_ADC_SETUP_DEFAULT);
	err |= tvp7002_write_reg(sd,
				     TVP7002_COARSE_CLAMP_CONTROL,
				     TVP7002_COARSE_CLAMP_CONTROL_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_SOG_CLAMP,
				     TVP7002_SOG_CLAMP_DEFAULT);
	err |= tvp7002_write_reg(sd, TVP7002_ALC_PLACEMENT,
				     TVP7002_ALC_PLACEMENT_DEFAULT);
	err |=
	    tvp7002_write_reg(sd,
				  TVP7002_VIDEO_BANDWIDTH_CONTROL,
				  TVP7002_VIDEO_BANDWIDTH_CONTROL_DEFAULT);
	err |=
	    tvp7002_write_reg(sd, TVP7002_AVID_START_PIXEL_HIGH,
				  TVP7002_AVID_START_PIXEL_DEFAULT);

	if (err < 0) {
		err = -EINVAL;
		return err;
	} else {
		std = tvp7002_configuration[0].input[0].def_std;
		err |= tvp7002_setstd(sd, std);

		if (err < 0) {
			err = -EINVAL;
			return err;
		}
	}
	v4l2_dbg(1, debug, sd, "End of tvp7002_init.\n");
	return err;
}

#if 0
/* tvp7002_setcontrol : Function to set the control parameter
 */
static int tvp7002_setcontrol(struct v4l2_control *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	v4l2_dbg(1, debug, sd,
		"TVP7002 supports no control parameter to be set..\n");
	err = -EINVAL;
	return err;
}

/* tvp7002_getcontrol :
 * Function to get the control parameter
 */
static int tvp7002_getcontrol(struct v4l2_control *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	v4l2_dbg(1, debug, sd,
		"TVP7002 supports no control parameters..\n");
	err = -EINVAL;
	return err;
}

/* tvp7002_querycontrol :
 * Function to query control parameter
 */
static int tvp7002_querycontrol(struct v4l2_queryctrl *ctrl, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	v4l2_dbg(1, debug, sd,
		"TVP7002 has no control parameters to return..\n");
	err = -EINVAL;
	return err;
}

/* following function is used to set THS7353 */
static int ths7353_setvalue(void)
{
	int err = 0;
	u8 val[2];
	u16 ths7353_i2c_addr = 0x5C >> 1;
	val[1] = 0x95;

	val[0] = 0x01;
	val[1] = 0x94;
	err = davinci_i2c_write(2, val, ths7353_i2c_addr);
	val[0] = 0x02;
	val[1] = 0x95;
	err |= davinci_i2c_write(2, val, ths7353_i2c_addr);
	val[0] = 0x03;
	val[1] = 0x94;
	err |= davinci_i2c_write(2, val, ths7353_i2c_addr);
	if (err) {
		printk(KERN_ERR "error in configuring THS7353\n");
	}
	return err;
}
#endif

/* tvp7002_setstd :
 * Function to set the video standard
 */
static int tvp7002_setstd(struct v4l2_subdev *sd, v4l2_std_id id)
{
	int err = 0;
	struct tvp7002_format_params *tvp7002formats;
	int i = 0;
	struct v4l2_standard *standard;

	v4l2_dbg(1, debug, sd, "Start of tvp7002_setstd...\n");

	for (i = 0; i < tvp7002_configuration[0].input[0].
			no_of_standard; i++) {
		standard = &tvp7002_configuration[0].input[0].standard[i];
		if (standard->id & id)
			break;
	}
	if (i == tvp7002_configuration[0].input[0].no_of_standard) {
		v4l2_err(sd, "Invalid id...\n");
		return -EINVAL;
	}

	tvp7002formats =
		&tvp7002_configuration[0].input[0].format[i];

	err = tvp7002_set_format_params(sd, tvp7002formats);
	if (err < 0) {
		v4l2_err(sd, "Set standard failed\n");
		return err;
	}

	/* Lock the structure variable and assign std to the member
	   variable */
	/*tvp7002_channel_info[ch_id].params.std = *id;*/

	v4l2_dbg(1, debug, sd, "End of tvp7002 set standard...\n");
	return err;
}

#if 0
/* tvp7002_getstd :
 * Function to get the video standard
 */
static int tvp7002_getstd(v4l2_std_id * id, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	v4l2_dbg(1, debug, sd, "Starting getstd function.\n");
	if (id == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}

	/* Read the video standard */
	*id = tvp7002_channel_info[ch_id].params.std;
	v4l2_dbg(1, debug, sd, "End of getstd function.\n");
	return err;
}
#endif

static void tvp7002_dump_register(struct v4l2_subdev *sd)
{
	int i = 0;
	u8 val;

	for (i = 0; i <= 0x5B; i++) {
		val = tvp7002_read_reg(sd, i);
		v4l2_dbg(1, debug, sd, "reg %x, val = %x\n", i, val);
	}
}

/* tvp7002_querystd :
 * Function to return standard detected by decoder
 */
static int tvp7002_querystd(struct v4l2_subdev *sd, v4l2_std_id *id)
{
	int err = 0;
	int val, val1, val_t;

	v4l2_dbg(1, debug, sd, "Starting querystd function...\n");
	if (id == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}

	tvp7002_initialize(sd);

	mdelay(100);
	/* Query the standards */
	val = tvp7002_read_reg(sd, TVP7002_LINES_PER_FRAME_STATUS_LOW);
	if (val < 0) {
		v4l2_err(sd,
			"I2C read fails...Lines per frame low\n");
		return val;
	}
	val1 = val;
	val = tvp7002_read_reg(sd, TVP7002_LINES_PER_FRAME_STATUS_HIGH);
	if (val < 0) {
		v4l2_err(sd,
			"I2C read fails...Lines per frame high\n");
		return val;
	}
	val1 |= (val << LINES_PER_FRAME_MSB_SHIFT) & LINES_PER_FRAME_MSB_MASK;
	val = (val & VIDEO_DETECTION_MASK) >> VIDEO_DETECTION_SHIFT;

	val_t = tvp7002_read_reg(sd, TVP7002_CLOCK_PER_LINE_STATUS_LSB);

	tvp7002_dump_register(sd);
	v4l2_dbg(1, debug, sd,
		   "TVP2002 - clocks per line detected = %d\n", val_t);
	if ((INTERLACED_VIDEO == val)
	    && (TVP7002_LINES_1080_60 == val1) &&
		(val_t >= TVP7002_CPL_1080_60_LOWER &&
		val_t <= TVP7002_CPL_1080_60_UPPER))
		*id = V4L2_STD_1080I_60;
	else if ((INTERLACED_VIDEO == val)
		&& (TVP7002_LINES_1080_50 == val1) &&
		    (val_t >= TVP7002_CPL_1080_50_LOWER &&
		    val_t <= TVP7002_CPL_1080_50_UPPER))
		*id = V4L2_STD_1080I_50;
	else if ((PROGRESSIVE_VIDEO == val)
		   && (TVP7002_LINES_720 == val1) &&
		   (val_t >= TVP7002_CPL_720P_50_LOWER &&
		   val_t <= TVP7002_CPL_720P_50_UPPER))
		*id = V4L2_STD_720P_50;
	else if ((PROGRESSIVE_VIDEO == val)
		   && (TVP7002_LINES_720 == val1) &&
		   (val_t >= TVP7002_CPL_720P_60_LOWER &&
		   val_t <= TVP7002_CPL_720P_60_UPPER))
		*id = V4L2_STD_720P_60;
	else if ((PROGRESSIVE_VIDEO == val) &&
		 (TVP7002_LINES_1080_60 == val1) &&
		 (val_t >= TVP7002_CPL_1080P_60_LOWER &&
		  val_t <= TVP7002_CPL_1080P_60_UPPER))
		*id = V4L2_STD_1080P_60;
	else if ((PROGRESSIVE_VIDEO == val)
		   && (525 == val1))
		*id = V4L2_STD_525P_60;
	else if ((PROGRESSIVE_VIDEO == val)
		   && (625 == val1))
		*id = V4L2_STD_625P_50;
	else {
		v4l2_err(sd,
			"querystd, erorxxx, val = %x, val1 = %x\n", val, val1);
		return -EINVAL;
	}

	/*tvp7002_channel_info[ch_id].params.std = *id;*/
	err = tvp7002_setstd(sd, *id);
	v4l2_dbg(1, debug, sd, "End of querystd function.\n");
	return err;
}

#if 0
/* tvp7002_enumstd : Function to enumerate standards supported
 */
static int tvp7002_enumstd(struct v4l2_standard *std, void *dec)
{
	int index, index1;
	int err = 0;
	int ch_id;
	int input_idx, sumstd = 0;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	if (std == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}
	index = std->index;
	index1 = index;
	/* Check for valid value of index */
	for (input_idx = 0;
	     input_idx < tvp7002_configuration[0].no_of_inputs;
	     input_idx++) {
		sumstd += tvp7002_configuration[0].input[input_idx]
		    .no_of_standard;
		if (index < sumstd) {
			sumstd -= tvp7002_configuration[0]
			    .input[input_idx].no_of_standard;
			break;
		}
	}
	if (input_idx == tvp7002_configuration[0].no_of_inputs)
		return -EINVAL;
	index -= sumstd;

	memset(std, 0, sizeof(*std));

	memcpy(std, &tvp7002_configuration[0].input[input_idx].
	       standard[index], sizeof(struct v4l2_standard));
	std->index = index1;
	return err;
}

/* tvp7002_setinput :
 * Function to set the input
 */
static int tvp7002_setinput(int *index, void *dec)
{
	int err = 0;
	int ch_id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	v4l2_dbg(1, debug, sd, "Start of set input function.\n");

	/* check for null pointer */
	if (index == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}
	if ((*index >= tvp7002_configuration[0].no_of_inputs)
	    || (*index < 0)) {
		return -EINVAL;
	}
	if (*index == 0) {	/* HD */
		tvp7002_channel_info[ch_id].params.inputidx = *index;
	} else {
		v4l2_err(sd, "Invalid index.\n");
		return -EINVAL;
	}
	v4l2_dbg(1, debug, sd, "End of set input function.\n");
	return err;
}

/* tvp7002_getinput : Function to get the input
 */
static int tvp7002_getinput(int *index, void *dec)
{
	int err = 0;
	int ch_id;
	v4l2_std_id id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	v4l2_dbg(1, debug, sd, "Start of get input function.\n");

	/* check for null pointer */
	if (index == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}
	err |= tvp7002_querystd(&id, dec);
	if (err < 0) {
		return err;
	}
	*index = 0;
	*index = tvp7002_channel_info[ch_id].params.inputidx;
	v4l2_dbg(1, debug, sd, "End of get input function.\n");
	return err;
}

/* tvp7002_enuminput :
 * Function to enumerate the input
 */
static int tvp7002_enuminput(struct v4l2_input *input, void *dec)
{
	int err = 0;
	int index = 0;
	int ch_id;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer.\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;

	/* check for null pointer */
	if (input == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}

	/* Only one input is available */
	if (input->index >= tvp7002_configuration[0].no_of_inputs) {
		return -EINVAL;
	}
	index = input->index;
	memset(input, 0, sizeof(*input));
	input->index = index;
	memcpy(input,
	       &tvp7002_configuration[0].input[index].input_info,
	       sizeof(struct v4l2_input));
	return err;
}
#endif

/* tvp7002_set_format_params :
 * Function to set the format parameters
 */
static int tvp7002_set_format_params(struct v4l2_subdev *sd,
				struct tvp7002_format_params *tvpformats)
{
	int err = 0;
	unsigned char val;
	
	v4l2_dbg(1, debug, sd,
		"Tvp7002 set format params started...\n");
	if (tvpformats == NULL) {
		v4l2_err(sd, "NULL Pointer.\n");
		return -EINVAL;
	}

	/* Write the HPLL related registers */
	err = tvp7002_write_reg(sd, TVP7002_HPLL_DIVIDER_MSB,
				    tvpformats->hpll_divider_msb);
	if (err < 0) {
		v4l2_err(sd,
			"I2C write fails...Divider MSB\n");
		return err;
	}

	val = ((tvpformats->
		hpll_divider_lsb & HPLL_DIVIDER_LSB_MASK) <<
	       HPLL_DIVIDER_LSB_SHIFT);
	err =
	    tvp7002_write_reg(sd, TVP7002_HPLL_DIVIDER_LSB, val);
	if (err < 0) {
		v4l2_err(sd,
			"I2C write fails...Divider LSB.\n");
		return err;
	}
	err = tvp7002_write_reg(sd, TVP7002_HPLL_CONTROL,
				    tvpformats->hpll_control);
	err = tvp7002_write_reg(sd, TVP7002_AVID_START_PIXEL_LOW,
				    tvpformats->avid_start_lsb);
	err = tvp7002_write_reg(sd, TVP7002_AVID_START_PIXEL_HIGH,
				    tvpformats->avid_start_msb);
	err = tvp7002_write_reg(sd, TVP7002_AVID_STOP_PIXEL_LOW,
				    tvpformats->avid_stop_lsb);
	err =
	    tvp7002_write_reg(sd, TVP7002_AVID_STOP_PIXEL_HIGH,
				  tvpformats->avid_stop_msb);
	err =
	    tvp7002_write_reg(sd, TVP7002_VBLK_FIELD0_START_OFFSET,
				  tvpformats->vblk_start_f0_line_offset);
	err =
	    tvp7002_write_reg(sd, TVP7002_VBLK_FIELD1_START_OFFSET,
				  tvpformats->vblk_start_f1_line_offset);
	err =
	    tvp7002_write_reg(sd, TVP7002_VBLK_FIELD0_DURATION,
				  tvpformats->vblk_f0_duration);
	err = tvp7002_write_reg(sd, TVP7002_VBLK_FIELD1_DURATION,
				    tvpformats->vblk_f1_duration);

	err = tvp7002_write_reg(sd, TVP7002_ALC_PLACEMENT,
				    tvpformats->alc_placement);
	err = tvp7002_write_reg(sd, TVP7002_CLAMP_START,
				    tvpformats->clamp_start);
	err = tvp7002_write_reg(sd, TVP7002_CLAMP_WIDTH,
				    tvpformats->clamp_width);
	err = tvp7002_write_reg(sd, TVP7002_HPLL_PRE_COAST,
				    tvpformats->hpll_pre_coast);
	err = tvp7002_write_reg(sd, TVP7002_HPLL_POST_COAST,
				    tvpformats->hpll_post_coast);

	v4l2_dbg(1, debug, sd,
		"End of tvp7002 set format params...\n");
	return err;
}

static int
tvp7002_g_fmt_cap(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct tvp7002_decoder *decoder = to_decoder(sd);

	if (f == NULL)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		/* only capture is supported */
		return -EINVAL;

	f->fmt.pix = decoder->pix;

	v4l2_dbg(1, debug, sd, "Current FMT: bytesperline - %d"
			"Width - %d, Height - %d",
			decoder->pix.bytesperline,
			decoder->pix.width, decoder->pix.height);
	return 0;
}

#if 0
/* tvp7002_setparams : This function will set parameters for tvp7002
 */
static int tvp7002_setparams(void *params, void *dec)
{
	int err = 0;
	unsigned char val;
	int ch_id;
	struct i2c_client *ch_client;
	struct tvp7002_params tvp7002params;

	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	if (copy_from_user(&tvp7002params, (struct tvp7002_params *) params,
			   sizeof(tvp7002params))) {
		return -EFAULT;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	err |= tvp7002_set_input_mux(ch_id);
	if (err) {
		printk(KERN_ERR "Failed to set mux bit.\n");
		return -EINVAL;
	}

	ch_client = sd,client;
	v4l2_dbg(1, debug, sd,
		"Start of tvp7002 set params function.\n");

	/* check for null pointer */
	err |= tvp7002_setinput(&(tvp7002params.inputidx), dec);
	if (err < 0) {
		v4l2_dbg(1, debug, sd,
			"Set format parameters failed.\n");
		return err;
	}
	err |= tvp7002_setstd(&(tvp7002params.std), dec);
	if (err < 0) {
		v4l2_dbg(1, debug, sd,
			"Set format parameters failed.\n");
		return err;
	}

	/* set video format related parameters */
	err = tvp7002_set_format_params(sd, &tvp7002params.format);
	if (err < 0) {
		v4l2_dbg(1, debug, sd,
			"Set format parameters failed.\n");
		return err;
	}

	/* Write the gain information */
	err = tvp7002_write_reg(ch_client, TVP7002_BLUE_FINE_GAIN,
				    tvp7002params.gain.blue_fine_gain);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	err = tvp7002_write_reg(ch_client, TVP7002_GREEN_FINE_GAIN,
				    tvp7002params.gain.green_fine_gain);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	err = tvp7002_write_reg(ch_client, TVP7002_RED_FINE_GAIN,
				    tvp7002params.gain.red_fine_gain);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}

	/* Write coarse gain information */
	val = 0;
	val = tvp7002params.gain.blue_coarse_gain & BLUE_COARSE_GAIN_MASK;
	tvp7002params.gain.blue_coarse_gain = val;

	val |= ((tvp7002params.gain.green_coarse_gain &
		 GREEN_COARSE_GAIN_MASK) << GREEN_COARSE_GAIN_SHIFT);
	tvp7002params.gain.green_coarse_gain =
	    tvp7002params.gain.green_coarse_gain & GREEN_COARSE_GAIN_MASK;

	err = tvp7002_write_reg(ch_client,
				    TVP7002_BLUE_GREEN_COARSE_GAIN, val);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}

	val = tvp7002params.gain.red_coarse_gain & RED_COARSE_GAIN_MASK;
	tvp7002params.gain.red_coarse_gain = val;
	err = tvp7002_write_reg(ch_client, TVP7002_RED_COARSE_GAIN, val);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}

	/*Write the offset value in register */
	err = tvp7002_write_reg(ch_client,
				    TVP7002_BLUE_FINE_OFFSETMSB,
				    tvp7002params.offset.blue_fine_offset);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	err = tvp7002_write_reg(ch_client,
				    TVP7002_GREEN_FINE_OFFSETMSB,
				    tvp7002params.offset.green_fine_offset);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	err = tvp7002_write_reg(ch_client, TVP7002_RED_FINE_OFFSETMSB,
				    tvp7002params.offset.red_fine_offset);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	val = 0;
	val = tvp7002params.offset.blue_fine_offset_lsb & FINE_OFFSET_LSB_MASK;
	tvp7002params.offset.blue_fine_offset_lsb = val;

	val |= ((tvp7002params.
		 offset.green_fine_offset_lsb & FINE_OFFSET_LSB_MASK) <<
		FINE_OFFSET_LSB_SHIFT_GREEN);
	tvp7002params.offset.green_fine_offset_lsb =
	    (tvp7002params.offset.green_fine_offset_lsb & FINE_OFFSET_LSB_MASK);

	val |= ((tvp7002params.
		 offset.red_fine_offset_lsb & FINE_OFFSET_LSB_MASK) <<
		FINE_OFFSET_LSB_SHIFT_RED);
	tvp7002params.offset.red_fine_offset_lsb =
	    (tvp7002params.offset.red_fine_offset_lsb & FINE_OFFSET_LSB_MASK);

	err = tvp7002_write_reg(ch_client, TVP7002_FINE_OFFSET_LSBS, val);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	val = tvp7002params.offset.blue_coarse_offset & COARSE_OFFSET_MASK;
	tvp7002params.offset.blue_coarse_offset = val;

	err = tvp7002_write_reg(ch_client, TVP7002_BLUE_COARSE_OFFSET, val);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	val = tvp7002params.offset.green_coarse_offset & COARSE_OFFSET_MASK;
	tvp7002params.offset.green_coarse_offset = val;

	err =
	    tvp7002_write_reg(ch_client, TVP7002_GREEN_COARSE_OFFSET, val);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	val = tvp7002params.offset.red_coarse_offset & COARSE_OFFSET_MASK;
	tvp7002params.offset.red_coarse_offset = val;

	err = tvp7002_write_reg(ch_client, TVP7002_RED_COARSE_OFFSET, val);
	if (err < 0) {
		v4l2_err(sd, "I2C write fails...\n");
		return err;
	}
	if (tvp7002params.alc.alc_enable) {
		err =
		    tvp7002_write_reg(ch_client, TVP7002_ALC_ENABLE, 0x80);
	} else {
		err =
		    tvp7002_write_reg(ch_client, TVP7002_ALC_ENABLE, 0x00);
	}
	val = (tvp7002params.alc.vcoeff << TVP7002_ALC_VCOEFF_SHIFT) |
	    (tvp7002params.alc.hcoeff);
	err = tvp7002_write_reg(ch_client, TVP7002_ALC_FILTER, val);
	if (err < 0)
		return err;

	tvp7002_channel_info[ch_id].params = tvp7002params;
	v4l2_dbg(1, debug, sd, "End of configTVP7002...\n");
	return err;
}

/* tvp7002_getparams : This function will get parameters for tvp7002
 */
static int tvp7002_getparams(void *params, void *dec)
{
	int ch_id;
	int err = 0;
	struct tvp7002_params *tvp7002params = (struct tvp7002_params *) params;
	if (NULL == dec) {
		printk(KERN_ERR "NULL Pointer\n");
		return -EINVAL;
	}
	ch_id = ((struct decoder_device *)dec)->channel_id;
	v4l2_dbg(1, debug, sd, "Starting tvp7002_getparams\n");

	/* check for null pointer */
	if (tvp7002params == NULL) {
		v4l2_err(sd, "Null pointer\n");
		return -EINVAL;
	}
	if (copy_to_user(tvp7002params, &(tvp7002_channel_info[ch_id].params),
			 sizeof(*tvp7002params))) {
		return -EFAULT;
	}

	v4l2_dbg(1, debug, sd, "End of getparams TVP7002...\n");
	return err;
}
#endif

static int tvp7002_s_stream(struct v4l2_subdev *sd, int enable)
{
	int err = 0;
	struct tvp7002_decoder *decoder = to_decoder(sd);

	if (decoder->streaming == enable)
		return 0;

	switch (enable) {
	case 0:
	{
		err = tvp7002_write_reg(sd, TVP7002_MISC_CONTROL_2, 0x1);
		if (err) {
			v4l2_err(sd, "Unable to turn off decoder\n");
			return err;
		}
		decoder->streaming = enable;
		break;
	}
	case 1:
	{
		/* Power Up Sequence */
		err = tvp7002_write_reg(sd, TVP7002_MISC_CONTROL_2, 0x0);
		if (err) {
			v4l2_err(sd, "Unable to turn off decoder\n");
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

static const struct v4l2_subdev_core_ops tvp7002_core_ops = {
	.s_std = tvp7002_setstd,
};

static const struct v4l2_subdev_video_ops tvp7002_video_ops = {
	.querystd = tvp7002_querystd,
	.g_fmt = tvp7002_g_fmt_cap,
	.s_stream = tvp7002_s_stream,
};

static const struct v4l2_subdev_ops tvp7002_ops = {
	.core = &tvp7002_core_ops,
	.video = &tvp7002_video_ops,
};

static struct tvp7002_decoder tvp7002_dev = {
	.streaming = 0,

	.fmt_list = tvp7002_fmt_list,
	.num_fmts = ARRAY_SIZE(tvp7002_fmt_list),

	.pix = {
		/* Default to NTSC 8-bit YUV 422 */
		.width = HD_720_NUM_ACTIVE_PIXELS,
		.height = HD_720_NUM_ACTIVE_LINES,
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.field = V4L2_FIELD_NONE,
		.bytesperline = HD_720_NUM_ACTIVE_PIXELS * 2,
		.sizeimage =
		HD_720_NUM_ACTIVE_PIXELS * 2 * HD_720_NUM_ACTIVE_LINES,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		},

	.current_std = 0,
	/*
	.std_list = tvp7002_std_list,
	.num_stds = ARRAY_SIZE(tvp7002_std_list),
	*/
};

static int 
tvp7002_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tvp7002_decoder *decoder;
	struct v4l2_subdev *sd;
	/*int polarity;*/
	int err;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	if (!client->dev.platform_data) {
		v4l2_err(client, "No platform data!!\n");
		return -ENODEV;
	}

	decoder = kzalloc(sizeof(*decoder), GFP_KERNEL);
	if (!decoder)
		return -ENOMEM;

	/* Initialize the tvp7002_decoder with default configuration */
	*decoder = tvp7002_dev;

	/* Copy board specific information here */
	decoder->pdata = client->dev.platform_data;

	/* TODO: need to take the values from the platform data */
	/*decoder->pdata->clk_polarity
	decoder->pdata->hs_polarity
	decoder->pdata->vs_polarity
	decoder->pdata->fid_polarity*/

	/* Set default standard to auto */
	/*decoder->tvp7002_regs[REG_VIDEO_STD].val =
		VIDEO_STD_AUTO_SWITCH_BIT;*/

	/* Register with V4L2 layer as slave device */
	sd = &decoder->sd;
	v4l2_i2c_subdev_init(sd, client, &tvp7002_ops);

	err = tvp7002_initialize(sd);
	if (err < 0)
		return err;

	v4l2_info(sd, "%s decoder driver registered !!\n", sd->name);

	return 0;
}

static int tvp7002_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tvp7002_decoder *decoder = to_decoder(sd);

	v4l2_device_unregister_subdev(sd);
	kfree(decoder);
	return 0;
}

/* I2C Device ID table */
static const struct i2c_device_id tvp7002_id[] = {
	{ "tvp7002", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tvp7002_id);

/* I2C driver data */
static struct i2c_driver tvp7002_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tvp7002",
	},
	.probe = tvp7002_probe,
	.remove = tvp7002_remove,
	.id_table = tvp7002_id,
};

static int __init tvp7002_init(void)
{
	return i2c_add_driver(&tvp7002_driver);
}

static void __exit tvp7002_exit(void)
{
	i2c_del_driver(&tvp7002_driver);
}

module_init(tvp7002_init);
module_exit(tvp7002_exit);
MODULE_LICENSE("GPL");
