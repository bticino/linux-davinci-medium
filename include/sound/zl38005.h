/*
 * linux/sound/wm8993.h -- Platform data for WM8993
 *
 * Copyright 2011 Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_ZL38005_H
#define __LINUX_SND_ZL38005_H

#define ZL38005_AEC_CTRL0	0x044A
#define ZL38005_SYSGAIN		0x044D
#define ZL38005_USRGAIN		0x046B

int zl38005_write(u16 addr, u16 val);
int zl38005_read(u16 addr, int *val);

#endif
