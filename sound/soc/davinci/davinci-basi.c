/*
 * ASoC driver for BTICINO BASI platform
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
#include <sound/uda134x.h>

#include <mach/dm365.h>
#include <mach/basi.h>
#include <mach/clock.h>
#include <linux/gpio.h>


#include <asm/dma.h>
#include <asm/mach-types.h>

#include <mach/asp.h>
#include <mach/edma.h>
#include <mach/mux.h>

#include "../codecs/cq93vc.h"
#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-vcif.h"

static struct platform_device *basi_snd_device[1];

#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | \
		SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)
static int basi_hw_params_cq93(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	unsigned sysclk;

	/*
	 * Even if this settings are done in
	 * dm365.c, it assures that they are
	 * correct.
	 */
	davinci_cfg_reg(DM365_EVT2_VC_TX);
	davinci_cfg_reg(DM365_EVT3_VC_RX);

	/* ASP1 on DM355 EVM is clocked by an external oscillator */
	sysclk = 27000000;

	return 0;
}

static struct snd_soc_ops basi_ops_cq93 = {
	.hw_params = basi_hw_params_cq93,
};

/* basi digital audio interface glue - connects codec <--> CPU */

static struct snd_soc_dai_link dm365_basi_dai[] = {
	{
		.name = "Voice Codec - CQ93VC",
		.stream_name = "CQ93",
		.cpu_dai = &davinci_vcif_dai,
		.codec_dai = &cq93vc_dai,
		.ops = &basi_ops_cq93,
	},
};

/* davinci dm365 basi audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_basi[] = {
	{
		.name = "DaVinci BASI VOICE",
		.platform = &davinci_soc_platform,
		.dai_link = &dm365_basi_dai[0],
		.num_links = 1,
	},
};

/* basi audio subsystem */
static struct snd_soc_device dm365_basi_snd_devdata[] = {
	{
		.card = &dm365_snd_soc_card_basi[0],
		.codec_dev = &soc_codec_dev_cq93vc,
	},
};

static int basi_asoc_probe(struct platform_device *pdev)
{
	int ret;
	int id = pdev->id;

	if (!machine_is_basi() || (id > 1))
		return -ENODEV;

	basi_snd_device[id] = platform_device_alloc("soc-audio", id);
	if (!basi_snd_device[id])
		return -ENOMEM;

	platform_set_drvdata(basi_snd_device[id], &dm365_basi_snd_devdata[id]);
	dm365_basi_snd_devdata[id].dev = &basi_snd_device[id]->dev;

	ret = platform_device_add(basi_snd_device[id]);
	if (ret)
		platform_device_put(basi_snd_device[id]);

	return ret;
}

static int __devexit basi_asoc_remove(struct platform_device *pdev)
{
	int id = pdev->id;

	platform_device_unregister(basi_snd_device[id]);
	return 0;
}

static struct platform_driver basi_asoc_driver = {
	.probe = basi_asoc_probe,
	.remove = __devexit_p(basi_asoc_remove),
	.driver = {
		.name = "basi-asoc",
		.owner = THIS_MODULE,
	},
};

static int __init basi_asoc_init(void)
{
	return platform_driver_register(&basi_asoc_driver);
}

static void __exit basi_asoc_exit(void)
{
	platform_driver_unregister(&basi_asoc_driver);
}

module_init(basi_asoc_init);
module_exit(basi_asoc_exit);

MODULE_AUTHOR("bticino s.p.a.");
MODULE_DESCRIPTION("TI DAVINCI basi ASoC driver");
MODULE_LICENSE("GPL");
