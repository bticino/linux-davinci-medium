/*
 * ASoC driver for BTICINO OWL platform
 *
 * Author:      bticino
 * Copyright:   (C) 2010 bticino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG
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
#include <mach/owl.h>
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

static struct platform_device *owl_snd_device[1];

#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | \
		SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)
static int owl_hw_params_cq93(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params)
{
	/*
	 * Even if this settings are done in
	 * dm365.c, it assures that they are
	 * correct.
	 */
	davinci_cfg_reg(DM365_EVT2_VC_TX);
	davinci_cfg_reg(DM365_EVT3_VC_RX);

	return 0;
}

static struct snd_soc_ops owl_ops_cq93 = {
	.hw_params = owl_hw_params_cq93,
};

/* davinci-owl machine dapm widgets */
static const struct snd_soc_dapm_widget cq93_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Speakers out", NULL),
};

/* davinci-owl machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Speakers connected to SP (actually not connected !) */
	{ "Speakers out", NULL, "SP", },
	/* Line output connected to LO */
	{ "Line Out", NULL, "LINEO", },
	/* Microphone input connected to MIC */
	{ "MICIN", NULL, "Microphone" },
};

static int owl_cq93_init(struct snd_soc_codec *codec)
{
	pr_debug("owl_cq93_init(%p)\n", codec);
	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(codec, cq93_dapm_widgets,
				  ARRAY_SIZE(cq93_dapm_widgets));

	/* Set up davinci-owl specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(codec, "SP");

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "LINEO");
	snd_soc_dapm_enable_pin(codec, "MICIN");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* owl digital audio interface glue - connects codec <--> CPU */

static struct snd_soc_dai_link dm365_owl_dai[] = {
	{
		.name = "Voice Codec - CQ93VC",
		.stream_name = "CQ93",
		.cpu_dai = &davinci_vcif_dai,
		.codec_dai = &cq93vc_dai,
		.init = owl_cq93_init,
		.ops = &owl_ops_cq93,
	},
};

/* davinci dm365 owl audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_owl[] = {
	{
		.name = "DaVinci OWL VOICE",
		.platform = &davinci_soc_platform,
		.dai_link = &dm365_owl_dai[0],
		.num_links = 1,
	},
};

/* owl audio subsystem */
static struct snd_soc_device dm365_owl_snd_devdata[] = {
	{
		.card = &dm365_snd_soc_card_owl[0],
		.codec_dev = &soc_codec_dev_cq93vc,
	},
};

static int owl_asoc_probe(struct platform_device *pdev)
{
	int ret;
	int id = pdev->id;

	if (!machine_is_owl() || (id > 1))
		return -ENODEV;

	owl_snd_device[id] = platform_device_alloc("soc-audio", id);
	if (!owl_snd_device[id])
		return -ENOMEM;

	platform_set_drvdata(owl_snd_device[id], &dm365_owl_snd_devdata[id]);
	dm365_owl_snd_devdata[id].dev = &owl_snd_device[id]->dev;

	ret = platform_device_add(owl_snd_device[id]);
	if (ret)
		platform_device_put(owl_snd_device[id]);

	return ret;
}

static int __devexit owl_asoc_remove(struct platform_device *pdev)
{
	int id = pdev->id;

	platform_device_unregister(owl_snd_device[id]);
	return 0;
}

static struct platform_driver owl_asoc_driver = {
	.probe = owl_asoc_probe,
	.remove = __devexit_p(owl_asoc_remove),
	.driver = {
		.name = "owl-asoc",
		.owner = THIS_MODULE,
	},
};

static int __init owl_asoc_init(void)
{
	return platform_driver_register(&owl_asoc_driver);
}

static void __exit owl_asoc_exit(void)
{
	platform_driver_unregister(&owl_asoc_driver);
}

module_init(owl_asoc_init);
module_exit(owl_asoc_exit);

MODULE_AUTHOR("bticino s.p.a.");
MODULE_DESCRIPTION("TI DAVINCI owl ASoC driver");
MODULE_LICENSE("GPL");
