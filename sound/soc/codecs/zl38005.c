/*
 * zl38005.c  --  ZL38005 ALSA Soc Audio driver
 *
 * Copyright 2011 Bticino S.p.A.
 *
 * Author: Raffaele Recalcati <raffaele.recalcati@bticino.it>
 *
 * Derived from the driver for max9877
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/zl38005.h>

#include "zl38005.h"
#include <sound/tlv.h>

/*
 * zl38005 register default settings
 */
static u16 zl38005_reg[] = {
	0x044a, 0x7e1c,
	0x044b, 0x1072,
	0x047b, 0x0072,
	0x0529, 0x7fff,
	0x044d, 0x8710,
	0x05e8, 0x0043,
	0x0451, 0x7fff,
	0x0450, 0xd3d3,
	0x044a, 0x7e1c,
	0x05ec, 0x0043,
	0x046b, 0x0037,
	0x0472, 0x7fff,
	0x0473, 0x7fff,
	0x0453, 0x7fff,
};

/*
 * Write to the zl38005 registers
 *
 */
static int zl38005_spi_write(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	int valt, i;

	if (invert)
		val = mask - val;

	if (zl38005_read(reg, &valt))
		return -EIO;

	if (((valt >> shift) & mask) == val)
		return 0;

	valt &= ~(mask << shift);
	valt |= val << shift;

	if (zl38005_write(reg, valt))
		return -EIO;

	for (i = 0; i < ARRAY_SIZE(zl38005_reg); i += 2)
		if (reg == zl38005_reg[i]) {
			zl38005_reg[i + 1] = valt;
			break;
		}

	return 0;
}

/*
 * Read zl38005 registers
 */
static int zl38005_spi_read(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	int i;

	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;
	int val;

	if (zl38005_read(reg, &val))
		return -1;

	ucontrol->value.integer.value[0] = ((val >> shift) & mask);

	if (invert)
		ucontrol->value.integer.value[0] =
			mask - ucontrol->value.integer.value[0];

	for (i = 0; i < ARRAY_SIZE(zl38005_reg); i += 2)
		if (reg == zl38005_reg[i]) {
			zl38005_reg[i + 1] = val;
			break;
		}

	return 0;
}

/*
 * Direct call to mute and un-mute MUTE_R bit
 * It should be better to implement using digital mute ops call
 * differentiating, as in wm8753_mute, between playback
 * and record active status.
 */
int zl38005_mute_r(int on)
{
	int val;
	if (zl38005_read(ZL38005_AEC_CTRL0, &val))
		return -EIO;
	if (((val >> 7) & 1) == on)
		return 0;
	val &= ~(1 << 7);
	val |= on << 7;
	if (zl38005_write(ZL38005_AEC_CTRL0, val))
		return -EIO;
	return 0;
}
EXPORT_SYMBOL_GPL(zl38005_mute_r);

static const struct snd_kcontrol_new zl38005_snd_controls[] = {
	SOC_SINGLE_EXT("ZL38005 UGAIN", ZL38005_USRGAIN , 0, 0xffff, 0,
			zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZL38005 SOUTGN", ZL38005_SYSGAIN , 8, 0xf, 0,
			zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZL38005 MUTE_R", ZL38005_AEC_CTRL0 , 7, 1, 0,
			zl38005_spi_read, zl38005_spi_write),
};

/* These functions are called from ASoC machine driver */
int zl38005_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, zl38005_snd_controls,
			ARRAY_SIZE(zl38005_snd_controls));
}
EXPORT_SYMBOL_GPL(zl38005_snd_controls);

/* This function is called from ASoC machine driver */
int zl38005_init(void)
{
	int i, ret = 0, valt;

	for (i = 0; i < ARRAY_SIZE(zl38005_reg); i += 2)
		if (zl38005_write(zl38005_reg[i], zl38005_reg[i+1]))
			ret = -1;
	for (i = 0; i < ARRAY_SIZE(zl38005_reg); i += 2) {
		zl38005_read(zl38005_reg[i], &valt);
		if (valt != zl38005_reg[i + 1])
			pr_info("ZL38005 ERR: REG=%04X VAL=%04X REREAD=%04X\n",
				 zl38005_reg[i], zl38005_reg[i + 1], valt);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(zl38005_init);

MODULE_DESCRIPTION("ASoC ZL38005 driver");
MODULE_AUTHOR("Raffaele Recalcati");
MODULE_LICENSE("GPL");
