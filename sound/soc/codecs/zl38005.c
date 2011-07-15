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
	u16 valt;

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

	return 0;
}

/*
 * Read zl38005 registers
 */
static int zl38005_spi_read(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;
	u16 val;

	if (zl38005_read(reg, &val))
		return -1;

	ucontrol->value.integer.value[0] = ((val >> shift) & mask);

	if (invert)
		ucontrol->value.integer.value[0] =
			mask - ucontrol->value.integer.value[0];
	return 0;
}

/*
 * ATTENZIONE ZARLINK REGISTRI A 16BIT, NON CAPISCO SE ASOC LI SUPPORTA
 * See SOC_SINGLE_EXT in sound/soc/codecs/max9877.c
 * e la max9877_write_regs per l'inizializzazione
 */
static const struct snd_kcontrol_new zl38005_snd_controls[] = {
	SOC_SINGLE_EXT("ZL PlayGain", ZL38005_USRGAIN , 0, 0xffff, 0,
			zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZL RecGain ", ZL38005_SYSGAIN , 8, 0xf, 0,
			zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZL PlayMute", ZL38005_AEC_CTRL0 , 7, 1, 0,
			zl38005_spi_read, zl38005_spi_write),
};

/* This function is called from ASoC machine driver */
int zl38005_add_controls(struct snd_soc_codec *codec)
{
	return snd_soc_add_controls(codec, zl38005_snd_controls,
			ARRAY_SIZE(zl38005_snd_controls));
}
EXPORT_SYMBOL_GPL(zl38005_snd_controls);

MODULE_DESCRIPTION("ASoC ZL38005 driver");
MODULE_AUTHOR("Raffaele Recalcati");
MODULE_LICENSE("GPL");
