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

#include <sound/soc.h>

extern int zl38005_add_controls(unsigned int minor,
                                        struct snd_soc_codec *codec);

#endif
