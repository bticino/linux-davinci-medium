/*
 * ASoC driver for BTICINO JUMBO_D platform
 *
 * Author:      bticino
 * Copyright:   (C) 2010 bticino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

//#define DEBUG
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
#include <mach/jumbo-d.h>
#include <mach/clock.h>
#include <linux/gpio.h>


#include <asm/dma.h>
#include <asm/mach-types.h>

#include <mach/asp.h>
#include <mach/edma.h>
#include <mach/mux.h>

#include "../codecs/uda1334.h"
#include "../codecs/cq93vc.h"
#include "../codecs/zl38005.h"
#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-vcif.h"

#include <sound/davinci_jumbo_asoc.h>
#include <linux/pm_loss.h>
#include <linux/delay.h>

/*  Defines it if voice codec works with DMA. */
//#undef VC_WORKING_WITH_DMA

static struct jumbo_asoc_platform_data jumbo_d_asoc_priv;
static struct uda1334_platform_data jumbo_d_uda1334_priv;

static struct platform_device *jumbo_d_snd_device[2];

#define AUDIO_FORMAT_CQ93 (SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)

/*	SND_SOC_DAIFMT_CBS_CFS  => CLK master & FRM master
	SND_SOC_DAIFMT_CBM_CFS  => CLK slave & FRM master */
#define AUDIO_FORMAT_UDA1334 ( SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFS)

static int jumbo_d_hw_params_cq93(struct snd_pcm_substream *substream,
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

static int jumbo_d_hw_params_uda(struct snd_pcm_substream *substream,
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
	clkout2 = clk_get(&(jumbo_d_snd_device[1]->dev), "clkout2");

	printk(KERN_DEBUG "%s - %d rate_num:%d / rate_den=%d\n", __func__,
			__LINE__, params->rate_num, params->rate_den);

	sysclk = params->rate_num / params->rate_den * 256;

	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT_UDA1334);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT_UDA1334);
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

static struct snd_soc_ops jumbo_d_ops_cq93 = {
	.hw_params = jumbo_d_hw_params_cq93,
};

static struct snd_soc_ops jumbo_d_ops_uda = {
	.hw_params = jumbo_d_hw_params_uda,
};

static int jumbo_d_line_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k, int event)
{
	zl38005_mute_r(SND_SOC_DAPM_EVENT_ON(event) ? 0 : 1);
	return 0;
};

static int jumbo_d_mic_event(struct snd_soc_dapm_widget *w,
			   struct snd_kcontrol *k, int event)
{
	zl38005_mute_r(SND_SOC_DAPM_EVENT_ON(event) ? 1 : 0);
	return 0;
};

static void ext_codec_power_work(struct work_struct *work)
{
	zl38005_init();
}

/* davinci-jumbo-d machine dapm widgets */
static const struct snd_soc_dapm_widget cq93_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Speakers out", NULL),
	SND_SOC_DAPM_LINE("Line Out", jumbo_d_line_event),
	SND_SOC_DAPM_MIC("Microphone", jumbo_d_mic_event),
};

/* davinci-jumbo-d machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map_c93[] = {
	/* Speakers connected to SP (actually not connected !) */
	{ "Speakers out", NULL, "SP", },
	/* Line output connected to LO */
	{ "Line Out", NULL, "LINEO", },
	/* Microphone input connected to MIC */
	{ "MICIN", NULL, "Microphone" },
};

/* davinci-jumbo-d machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map_uda[] = {
	/* Line Out connected to LLOUT, RLOUT */
        {"Line Out", NULL, "LLOUT"},
        {"Line Out", NULL, "RLOUT"},
        /* Line In connected to (LINE1L | LINE1R) */
        {"LINE1L", NULL, "Line In"},
        {"LINE1R", NULL, "Line In"},
};

/* Logic for a cq93 as connected on a jumbo_d */
static int jumbo_d_cq93_init(struct snd_soc_codec *codec)
{
	int err;

	pr_debug("jumbo_d_cq93_init(%p)\n", codec);
	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(codec, cq93_dapm_widgets,
				  ARRAY_SIZE(cq93_dapm_widgets));

	INIT_DELAYED_WORK(&jumbo_d_asoc_priv.delayed_work, ext_codec_power_work);
	jumbo_d_asoc_priv.ext_codec_power(1); /* zl38005 is the ext codec */
	mdelay(100); /* zl38005 startup need at least 83msec */
	zl38005_init();

	err = zl38005_add_controls(codec);
	if (err < 0)
		return err;

	/* Set up davinci-jumbo-d specific audio path audio_map_c93 */
	snd_soc_dapm_add_routes(codec, audio_map_c93, ARRAY_SIZE(audio_map_c93));

	/* not connected */
	snd_soc_dapm_disable_pin(codec, "SP");

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "LINEO");
	snd_soc_dapm_enable_pin(codec, "MICIN");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* Logic for a uda1334 as connected on a jumbo_d */
static int jumbo_d_uda1334_init(struct snd_soc_codec *codec)
{
#if 0
        /* Add jumbo specific widgets */

        /* Set up jumbo specific audio path audio_map */
        snd_soc_dapm_add_routes(codec, audio_map_uda, ARRAY_SIZE(audio_map_uda));

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

/* jumbo-d digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dm365_jumbo_d_dai[] = {
	{
		.name = "Voice Codec - CQ93VC",
		.stream_name = "CQ93",
		.cpu_dai = &davinci_vcif_dai,
		.codec_dai = &cq93vc_dai,
		.init = jumbo_d_cq93_init,
		.ops = &jumbo_d_ops_cq93,
	}, {
		.name = "UDA1334",
		.stream_name = "uda1334",
		.cpu_dai = &davinci_i2s_dai,
		.codec_dai = &uda1334_dai,
		.init = jumbo_d_uda1334_init,
		.ops = &jumbo_d_ops_uda,
	},
};

/* davinci dm365 jumbo-d audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_jumbo_d[] = {
	{
		.name = "DaVinci JUMBO_D VOICE",
		.platform = &davinci_soc_platform,
		.dai_link = &dm365_jumbo_d_dai[0],
		.num_links = 1,
	}, {
		.name = "DaVinci JUMBO_D STEREO OUT",
		.platform = &davinci_soc_platform,
		.dai_link = &dm365_jumbo_d_dai[1],
		.num_links = 1,
	},
};

/* jumbo-d audio subsystem */
static struct snd_soc_device dm365_jumbo_d_snd_devdata[] = {
	{
		.card = &dm365_snd_soc_card_jumbo_d[0],
		.codec_dev = &soc_codec_dev_cq93vc,
	//	.codec_data = &jumbo_d_voice_data,
	},
	{
		.card = &dm365_snd_soc_card_jumbo_d[1],
		.codec_dev = &soc_codec_dev_uda1334,
		.codec_data = &jumbo_d_uda1334_priv,
	},
};

static int jumbo_d_asoc_probe(struct platform_device *pdev)
{
	int ret;
	int id = pdev->id;

	struct jumbo_asoc_platform_data *pdata = pdev->dev.platform_data;
	struct uda1334_platform_data *pdata_1 = pdev->dev.platform_data;

	//printk("SIAMO QUIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII\n");
	if (!machine_is_jumbo_d() || (id > 1))
		return -ENODEV;

	if (id == 0)
	{
		if ((!pdata->ext_codec_power) || (!pdata->ext_circuit_power))
			return -EINVAL;

		memcpy(&jumbo_d_asoc_priv, pdata, sizeof(jumbo_d_asoc_priv));
	}

	if (id == 1)
	{
		//return 1;

		if (!pdata_1->power)
			return -EINVAL;
		memcpy(&jumbo_d_uda1334_priv, pdata_1, sizeof(jumbo_d_uda1334_priv));
	}

	jumbo_d_snd_device[id] = platform_device_alloc("soc-audio", id);
	if (!jumbo_d_snd_device[id])
		return -ENOMEM;

	platform_set_drvdata(jumbo_d_snd_device[id], &dm365_jumbo_d_snd_devdata[id]);
	dm365_jumbo_d_snd_devdata[id].dev = &jumbo_d_snd_device[id]->dev;

	ret = platform_device_add(jumbo_d_snd_device[id]);
	if (ret)
		platform_device_put(jumbo_d_snd_device[id]);

	return ret;
}

static int __devexit jumbo_d_asoc_remove(struct platform_device *pdev)
{
	int id = pdev->id;

	platform_device_unregister(jumbo_d_snd_device[id]);
	return 0;
}

#ifdef CONFIG_PM_LOSS
static int jumbo_d_asoc_power_changed(struct device *dev,
				 enum sys_power_state s)
{
	int ret = -EIO;

	switch (s) {
	case SYS_PWR_GOOD:
		jumbo_d_asoc_priv.ext_codec_power(1);
		jumbo_d_asoc_priv.ext_circuit_power(1);
		schedule_delayed_work(&jumbo_d_asoc_priv.delayed_work,
				      msecs_to_jiffies(100));
		jumbo_d_uda1334_priv.power(1);
		break;
	case SYS_PWR_FAILING:
		jumbo_d_asoc_priv.ext_codec_power(0);
		jumbo_d_asoc_priv.ext_circuit_power(0);
		ret = cancel_delayed_work(&jumbo_d_asoc_priv.delayed_work);
		jumbo_d_uda1334_priv.power(0);
		break;
	default:
		BUG();
		ret = -ENODEV;
	}
	return ret;
}

static struct dev_pm_ops jumbo_d_asoc_dev_pm_ops = {
	.power_changed = jumbo_d_asoc_power_changed,
};
#endif

static struct platform_driver jumbo_d_asoc_driver = {
	.probe = jumbo_d_asoc_probe,
	.remove = __devexit_p(jumbo_d_asoc_remove),
	.driver = {
		.name = "jumbo-d-asoc",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_LOSS
		.pm = &jumbo_d_asoc_dev_pm_ops,
#endif
	},
};

static int __init jumbo_d_asoc_init(void)
{
	return platform_driver_register(&jumbo_d_asoc_driver);
}

static void __exit jumbo_d_asoc_exit(void)
{
	platform_driver_unregister(&jumbo_d_asoc_driver);
}

module_init(jumbo_d_asoc_init);
module_exit(jumbo_d_asoc_exit);

MODULE_AUTHOR("bticino s.p.a.");
MODULE_DESCRIPTION("TI DAVINCI jumbo-d ASoC driver");
MODULE_LICENSE("GPL");
