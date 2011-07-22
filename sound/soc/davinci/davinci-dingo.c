/*
 * ASoC driver for BTICINO DINGO platform
 *
 * Author:      bticino
 * Copyright:   (C) 2010 bticino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/uda1334.h>

#include <mach/dm365.h>
#include <mach/dingo.h>
#include <mach/clock.h>
#include <linux/gpio.h>


#include <asm/dma.h>
#include <asm/mach-types.h>

#include <mach/asp.h>
#include <mach/edma.h>
#include <mach/mux.h>

#include "../codecs/uda1334.h"
#include "../codecs/cq93vc.h"
#include "davinci-pcm.h"
#include "davinci-pcm-copy.h"
#include "cq93vc_copy.h"
#include "davinci-i2s.h"
#include "davinci-vcif.h"

/*  Defines it if voice codec works with DMA. */
#undef VC_WORKING_WITH_DMA

#define AUDIO_FORMAT (\
	SND_SOC_DAIFMT_I2S | \
	SND_SOC_DAIFMT_NB_NF | \
	SND_SOC_DAIFMT_CBM_CFS)

/*	SND_SOC_DAIFMT_CBS_CFS  => CLK master & FRM master
	SND_SOC_DAIFMT_CBM_CFS  => CLK slave & FRM master */

static struct platform_device *dingo_snd_device[2];

static int dingo_hw_params_uda(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	unsigned sysclk;
	struct clk *clkout2;

	davinci_cfg_reg(DM365_EVT2_ASP_TX);
	davinci_cfg_reg(DM365_EVT3_ASP_RX);

	/* one can think to use codec_dai->id instead of 1 ... */
	clkout2 = clk_get(&(dingo_snd_device[1]->dev), "clkout2");

	printk(KERN_DEBUG "%s - %d rate_num:%d / rate_den=%d\n", __func__,
			__LINE__, params->rate_num, params->rate_den);

	sysclk = params->rate_num / params->rate_den * 256;

	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;


	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* on CLKS there's 256*fs -> 256/32 bits per sample => clk_div=8 */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, 8);
	if (ret < 0)
		return ret;

	printk(KERN_DEBUG "%s - %d setting sysclk=%d\n",
		__func__, __LINE__, sysclk);
	dm365_clkout2_set_rate(sysclk);
	return 0;
}

static int dingo_hw_params_cq93(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
#ifdef VC_WORKING_WITH_DMA
	/*
	 * Even if this settings are done in
	 * dm365.c, it assures that they are
	 * correct.
	 */
	davinci_cfg_reg(DM365_EVT2_VC_TX);
	davinci_cfg_reg(DM365_EVT3_VC_RX);
#endif
	return 0;
}

static struct snd_soc_ops dingo_ops_uda = {
	.hw_params = dingo_hw_params_uda,
};

static struct snd_soc_ops dingo_ops_cq93 = {
	.hw_params = dingo_hw_params_cq93,
};

void dingo_uda_power(int on)
{
	gpio_set_value(AUDIO_RESET, !on);
}

/* davinci-dingo machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Line In connected to (LINE1L | LINE1R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
};

/* Logic for a uda1334 as connected on a dingo */
static int dingo_uda1334_init(struct snd_soc_codec *codec)
{
#if 0
	/* Add dingo specific widgets */

	/* Set up dingo specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(codec, "Line In");

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Line Out");

	snd_soc_dapm_sync(codec);
#else
	printk(KERN_DEBUG "dapm for uda1334 not needed\n");
#endif

	return 0;
}

/* dingo digital audio interface glue - connects codec <--> CPU */

static struct snd_soc_dai_link dm365_dingo_dai[] = {
	{
		.name = "Voice Codec - CQ93VC",
		.stream_name = "CQ93",
		.cpu_dai = &davinci_vcif_dai,
		.codec_dai = &cq93vc_dai,
		.ops = &dingo_ops_cq93,
	},
	{
		.name = "UDA1334",
		.stream_name = "uda1334",
		.cpu_dai = &davinci_i2s_dai,
		.codec_dai = &uda1334_dai,
		.init = dingo_uda1334_init,
		.ops = &dingo_ops_uda,
	},
};

/* davinci dm365 dingo audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_dingo[] = {
	{
		.name = "DaVinci DINGO VOICE",
#ifdef VC_WORKING_WITH_DMA
		.platform = &davinci_soc_platform,
#else
		.platform = &davinci_soc_platform_copy,
#endif
		.dai_link = &dm365_dingo_dai[0],
		.num_links = 1,
	},
	{
		.name = "DaVinci DINGO STEREO OUT",
		.platform = &davinci_soc_platform,
		.dai_link = &dm365_dingo_dai[1],
		.num_links = 1,
	},
};

static struct uda1334_platform_data dingo_uda1334 = {
	.mute_gpio = AUDIO_MUTE,
	.deemp_gpio = AUDIO_DEEMP,
	.reset_gpio = AUDIO_RESET,
	.power = dingo_uda_power,
};

static struct davinci_pcm_copy_platform_data dingo_voice_data = {
	.buffer_size = 2048,
	.interrupt_margin_ps = 400000,
	.min_interrupt_interval_ps = 500000,
	.ops = &cq93vc_pcm_copy_ops,
};

/* dingo audio subsystem */
static struct snd_soc_device dm365_dingo_snd_devdata[] = {
	{
		.card = &dm365_snd_soc_card_dingo[0],
		.codec_dev = &soc_codec_dev_cq93vc,
		.codec_data = &dingo_voice_data,
	},
	{
		.card = &dm365_snd_soc_card_dingo[1],
		.codec_dev = &soc_codec_dev_uda1334,
		.codec_data = &dingo_uda1334,
	},
};

static int dingo_asoc_probe(struct platform_device *pdev)
{
	int ret;
	int id = pdev->id;


	if (!machine_is_dingo() || (id > 1))
		return -ENODEV;

	dingo_snd_device[id] = platform_device_alloc("soc-audio", id);
	if (!dingo_snd_device[id])
		return -ENOMEM;
	platform_set_drvdata(dingo_snd_device[id], &dm365_dingo_snd_devdata[id]);

	dm365_dingo_snd_devdata[id].dev = &dingo_snd_device[id]->dev;
	ret = platform_device_add(dingo_snd_device[id]);
	if (ret)
		platform_device_put(dingo_snd_device[id]);

	return ret;
}

static int __devexit dingo_asoc_remove(struct platform_device *pdev)
{
	int id = pdev->id;

	platform_device_unregister(dingo_snd_device[id]);
	return 0;
}

static struct platform_driver dingo_asoc_driver = {
	.probe = dingo_asoc_probe,
	.remove = __devexit_p(dingo_asoc_remove),
	.driver = {
		.name = "dingo-asoc",
		.owner = THIS_MODULE,
	},
};

static int __init dingo_soc_init(void)
{
	return platform_driver_register(&dingo_asoc_driver);
}

static void __exit dingo_soc_exit(void)
{
	platform_device_unregister(&dingo_snd_device);
}

module_init(dingo_soc_init);
module_exit(dingo_soc_exit);

MODULE_AUTHOR("Davide Bonfanti - bticino s.p.a.");
MODULE_DESCRIPTION("TI DAVINCI Dingo ASoC driver");
MODULE_LICENSE("GPL");
