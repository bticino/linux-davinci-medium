/*
 * ths7303- THS7303 Video Amplifier driver
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

/* all supported modes */
enum ths7303_filter_mode {
	THS7303_FILTER_MODE_480I_576I,
	THS7303_FILTER_MODE_480P_576P,
	THS7303_FILTER_MODE_720P_1080I,
	THS7303_FILTER_MODE_1080P
};

int ths7303_setval(enum ths7303_filter_mode mode);

