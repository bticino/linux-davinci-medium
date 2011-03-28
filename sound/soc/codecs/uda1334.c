/*
 * uda1334.c  --  UDA1334 ALSA SoC Codec driver
 *
 * Author: Davide Bonfanti <davide.bonfanti@bticino.it>
 * Based on uda134x.c made by Zoltan Devai
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <sound/uda1334.h>

#include "uda1334.h"


/*
  ALSA SOC usually puts the device in standby mode when it's not used
  for sometime. If you define POWER_OFF_ON_STANDBY the driver will
  turn off the ADC/DAC when this callback is invoked and turn it back
  on when needed. Unfortunately this will result in a very light bump
  (it can be audible only with good earphones). If this bothers you
  just comment this line, you will have slightly higher power
  consumption . Please note that sending the L3 command for ADC is
  enough to make the bump, so it doesn't make difference if you
  completely take off power from the codec.
 */

#define UDA134X_RATES SNDRV_PCM_RATE_8000_48000
#define UDA134X_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S20_3LE)

struct uda1334_priv {
	int sysclk;
	int dai_fmt;
	int deemp;
	int running;
};

static int uda1334_mute(struct snd_soc_dai *dai, int mute)
{
	struct uda1334_platform_data *pd = (struct uda1334_platform_data*)dai->private_data;

	if (pd->mute_gpio >= 0)
		gpio_set_value(pd->mute_gpio, mute);

	return 0;
}

static int uda1334_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct uda1334_priv *uda1334 = codec->private_data;
	struct uda1334_platform_data *pd = codec->control_data;

	if (pd->reset_gpio >= 0)
		gpio_set_value(pd->reset_gpio, 0);
	if (pd->power)
		pd->power(1);

	uda1334->running = 0;
	return 0;
}

static void uda1334_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct uda1334_priv *uda1334 = codec->private_data;
	struct uda1334_platform_data *pd = codec->control_data;
	printk("%s - %d\n", __func__, __LINE__);

	if (pd->reset_gpio >= 0)
		gpio_set_value(pd->reset_gpio, 1);
	if (pd->power)
		pd->power(0);
	uda1334->running = 0;
}

static int uda1334_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	return 0;
}

static int uda1334_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct uda1334_priv *uda1334 = codec->private_data;

	pr_debug("%s clk_id: %d, freq: %u, dir: %d\n", __func__,
		 clk_id, freq, dir);

	/* Anything between 256fs*8Khz and 512fs*48Khz should be acceptable
	   because the codec is slave. Of course limitations of the clock
	   master (the IIS controller) apply.
	   We'll error out on set_hw_params if it's not OK */
	if ((freq >= (256 * 8000)) && (freq <= (512 * 48000))) {
		uda1334->sysclk = freq;
		return 0;
	}

	printk(KERN_ERR "%s unsupported sysclk\n", __func__);
	return -EINVAL;
}

static int uda1334_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int fmt)
{
	return 0;
}

static int snd_uda1334_deemp_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	return 0;
}

static int snd_uda1334_deemp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_device * socdev  = (struct snd_soc_device *) kcontrol->private_data;
	struct uda1334_priv *uda1334 = socdev->card->codec->private_data;
	ucontrol->value.integer.value[0] = uda1334->deemp;
	return 0;
}

static int snd_uda1334_deemp_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_device * socdev  = (struct snd_soc_device *) kcontrol->private_data;
	struct uda1334_priv *uda1334 = socdev->card->codec->private_data;
	struct uda1334_platform_data * pd = (struct uda1334_platform_data*) socdev->codec_data;

	uda1334->deemp = ucontrol->value.integer.value[0];

	gpio_set_value(pd->deemp_gpio, uda1334->deemp);
	return 0;
}

static struct snd_kcontrol_new snd_uda1334_deemp = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Deemphasis",
	.index = 0,
	.info = snd_uda1334_deemp_info,
	.get = snd_uda1334_deemp_get,
	.put = snd_uda1334_deemp_put,
	.access         = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0,
};

static int snd_uda1334_mute_info(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	return 0;
}

static int snd_uda1334_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_device * socdev  = (struct snd_soc_device *) kcontrol->private_data;
	struct uda1334_priv *uda1334 = socdev->card->codec->private_data;
	struct uda1334_platform_data * pd = (struct uda1334_platform_data*) socdev->codec_data;

	ucontrol->value.integer.value[0] = gpio_get_value(pd->mute_gpio);
	return 0;
}

static int snd_uda1334_mute_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_device * socdev  = (struct snd_soc_device *) kcontrol->private_data;
	struct uda1334_priv *uda1334 = socdev->card->codec->private_data;
	struct uda1334_platform_data * pd = (struct uda1334_platform_data*) socdev->codec_data;

	gpio_set_value(pd->mute_gpio, ucontrol->value.integer.value[0]);
	return 0;
}

static struct snd_kcontrol_new snd_uda1334_mute = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "Mute",
	.index = 0,
	.info = snd_uda1334_mute_info,
	.get = snd_uda1334_mute_get,
	.put = snd_uda1334_mute_put,
	.access         = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	.private_value = 0,
};

static struct snd_soc_dai_ops uda1334_dai_ops = {
	.startup	= uda1334_startup,
	.shutdown	= uda1334_shutdown,
	.hw_params	= uda1334_hw_params,
	.digital_mute	= uda1334_mute,
	.set_sysclk	= uda1334_set_dai_sysclk,
	.set_fmt	= uda1334_set_dai_fmt,
};

struct snd_soc_dai uda1334_dai = {
	.name = "UDA1334",
	/* playback capabilities */
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = UDA134X_RATES,
		.formats = UDA134X_FORMATS,
	},
	/* pcm operations */
	.ops = &uda1334_dai_ops,
};
EXPORT_SYMBOL(uda1334_dai);

static int uda1334_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	struct uda1334_priv *uda1334;
	void *codec_setup_data = socdev->codec_data;
	int ret = -ENOMEM;
	struct uda1334_platform_data *pd;

	printk(KERN_INFO "UDA1334 SoC Audio Codec\n");

	if (!codec_setup_data) {
		printk(KERN_ERR "UDA1334 SoC codec: "
		       "missing codec_data\n");
		return -ENODEV;
	}

	pd = codec_setup_data;

	socdev->card->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (socdev->card->codec == NULL)
		return ret;

	codec = socdev->card->codec;

	uda1334 = kzalloc(sizeof(struct uda1334_priv), GFP_KERNEL);
	if (uda1334 == NULL)
		goto priv_err;
	codec->private_data = uda1334;
	mutex_init(&codec->mutex);
	codec->name = "UDA1334";
	codec->owner = THIS_MODULE;
	codec->dai = &uda1334_dai;
	codec->num_dai = 1;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	uda1334_dai.private_data = pd;
	codec->control_data = codec_setup_data;

	if (pd->power)
		pd->power(0);

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "UDA1334: failed to register pcms\n");
		goto reg_err;
	}

	if (pd->deemp_gpio >= 0)
		ret = snd_ctl_add(codec->card, snd_ctl_new1(&snd_uda1334_deemp, socdev));
	if (pd->mute_gpio >= 0)
		ret = snd_ctl_add(codec->card, snd_ctl_new1(&snd_uda1334_mute, socdev));
	if (ret < 0) {
		printk(KERN_ERR "UDA1334: failed to register controls\n");
		goto reg_err;
	}

	return 0;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
reg_err:
	kfree(codec->private_data);
priv_err:
	kfree(codec);
	return ret;
}

/* power down chip */
static int uda1334_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	kfree(codec->private_data);
	kfree(codec->reg_cache);
	kfree(codec);

	return 0;
}

#if defined(CONFIG_PM)
static int uda1334_soc_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct uda1334_platform_data *pd = codec->control_data;

	if (pd->reset_gpio >= 0)
		gpio_set_value(pd->reset_gpio, 1);
	if (pd->power)
		pd->power(0);
	return 0;
}

static int uda1334_soc_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	struct uda1334_priv *uda1334 = codec->private_data;
	struct uda1334_platform_data *pd = codec->control_data;

	if (!uda1334->running)
		return 0
	if (pd->reset_gpio >= 0)
		gpio_set_value(pd->reset_gpio, 0);
	if (pd->power)
		pd->power(1);

	return 0;
}
#else
#define uda1334_soc_suspend NULL
#define uda1334_soc_resume NULL
#endif /* CONFIG_PM */

struct snd_soc_codec_device soc_codec_dev_uda1334 = {
	.probe =        uda1334_soc_probe,
	.remove =       uda1334_soc_remove,
	.suspend =      uda1334_soc_suspend,
	.resume =       uda1334_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_uda1334);

static int __init uda1334_init(void)
{
	return snd_soc_register_dai(&uda1334_dai);
}
module_init(uda1334_init);

static void __exit uda1334_exit(void)
{
	snd_soc_unregister_dai(&uda1334_dai);
}
module_exit(uda1334_exit);

MODULE_DESCRIPTION("UDA1334 ALSA soc codec driver");
MODULE_AUTHOR("Davide Bonfanti <davide.bonfanti@bticino.it>");
MODULE_LICENSE("GPL");
