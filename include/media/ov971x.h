/* ov971x Camera
 *
 * Copyright (C) 2008 Renesas Solutions Corp.
 * Kuninori Morimoto <morimoto.kuninori@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __OV971X_H__
#define __OV971X_H__

#include <media/soc_camera.h>

/* for flags */
#define OV971X_FLAG_VFLIP     0x00000001 /* Vertical flip image */
#define OV971X_FLAG_HFLIP     0x00000002 /* Horizontal flip image */

/* Output drive capability */
enum ov971x_out_drive_cap {
	/* 1x */
	OV971x_DRIVE_CAP_X1,
	/* 2x */
	OV971x_DRIVE_CAP_X2,
	/* 3x */
	OV971x_DRIVE_CAP_X3,
	/* 4x */
	OV971x_DRIVE_CAP_X4,
};

/* clk polarity */
enum ov971x_clk_polarity {
	OV971x_CLK_POL_INVERTED,
	OV971x_CLK_POL_NORMAL,
};

/* clk polarity */
enum ov971x_hs_polarity {
	OV971x_HS_POL_POS,
	OV971x_HS_POL_NEG,
};

/* clk polarity */
enum ov971x_vs_polarity {
	OV971x_VS_POL_POS,
	OV971x_VS_POL_NEG,
};

/* Platform dependent data */
struct ov971x_platform_data {
	enum ov971x_clk_polarity clk_polarity;
	enum ov971x_hs_polarity hs_polarity;
	enum ov971x_vs_polarity vs_polarity;
	u32 xclk_frequency;
	enum ov971x_out_drive_cap out_drive_capability;
};

/*
 * for Edge ctrl
 *
 * strength also control Auto or Manual Edge Control Mode
 * see also OV971X_MANUAL_EDGE_CTRL
 */
struct ov971x_edge_ctrl {
	unsigned char strength;
	unsigned char threshold;
	unsigned char upper;
	unsigned char lower;
};

#define OV971X_MANUAL_EDGE_CTRL	0x80 /* un-used bit of strength */
#define EDGE_STRENGTH_MASK	0x1F
#define EDGE_THRESHOLD_MASK	0x0F
#define EDGE_UPPER_MASK		0xFF
#define EDGE_LOWER_MASK		0xFF

#define OV971X_AUTO_EDGECTRL(u, l)	\
{					\
	.upper = (u & EDGE_UPPER_MASK),	\
	.lower = (l & EDGE_LOWER_MASK),	\
}

#define OV971X_MANUAL_EDGECTRL(s, t)					\
{									\
	.strength  = (s & EDGE_STRENGTH_MASK) | OV971X_MANUAL_EDGE_CTRL,\
	.threshold = (t & EDGE_THRESHOLD_MASK),				\
}

/*
 * ov971x camera info
 */
struct ov971x_camera_info {
	unsigned long          buswidth;
	unsigned long          flags;
	struct soc_camera_link link;
	struct ov971x_edge_ctrl edgectrl;
};

#endif /* __OV971X_H__ */
