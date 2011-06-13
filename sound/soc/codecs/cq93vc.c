/*
 * ALSA SoC CQ0093 Voice Codec Driver for DaVinci platforms
 *
 * Copyright (C) 2010 Texas Instruments, Inc
 *
 * Author: Miguel Aguilar <miguel.aguilar@ridgerun.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/mfd/davinci_voicecodec.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>


#include <mach/dm365.h>

#include "cq93vc.h"

static inline unsigned int cq93vc_read(struct snd_soc_codec *codec,
						unsigned int reg)
{
	struct davinci_vc *davinci_vc = codec->control_data;

	return readl(davinci_vc->base + reg);
}

static inline int cq93vc_write(struct snd_soc_codec *codec, unsigned int reg,
		       unsigned int value)
{
	struct davinci_vc *davinci_vc = codec->control_data;

	writel(value, davinci_vc->base + reg);

	return 0;
}


static const char *enbyp_txt[] = {
	"Enabled", "Bypass",
};

static const char *endis_txt[] = {
	"Disabled", "Enabled"
};

static const struct soc_enum hpf =
SOC_ENUM_SINGLE(DAVINCI_VC_REG04, 3, 2, enbyp_txt);

static const struct soc_enum nf =
SOC_ENUM_SINGLE(DAVINCI_VC_REG04, 1, 2, enbyp_txt);

static const char *gaa_txt[] = {
	"20dB", "26dB",
};

static const struct soc_enum gaa =
SOC_ENUM_SINGLE(DAVINCI_VC_REG05, 2, 2, gaa_txt);

static const DECLARE_TLV_DB_SCALE(dac_at_scale, -6300, 100, 1);

static const char *soft_mute_txt[] = {
	"OFF",
	"ON",
};

static const struct soc_enum soft_mute =
SOC_ENUM_SINGLE(DAVINCI_VC_REG09, 6, 2, soft_mute_txt);

static const struct soc_enum alc =
SOC_ENUM_SINGLE(DAVINCI_VC_REG06, 0, 2, endis_txt);

static const struct soc_enum zcaen =
SOC_ENUM_SINGLE(DAVINCI_VC_REG05, 4, 2, endis_txt);

static const struct soc_enum zcren =
SOC_ENUM_SINGLE(DAVINCI_VC_REG05, 3, 2, endis_txt);

static const struct snd_kcontrol_new cq93vc_snd_controls[] = {
	SOC_SINGLE("PGA Capture Volume", DAVINCI_VC_REG05, 0, 0x3, 0),
	SOC_SINGLE_TLV("Mono DAC Playback Volume", DAVINCI_VC_REG09, 0, 0x3f,
		       0, dac_at_scale),
	SOC_ENUM("High Pass Filter", hpf),
	SOC_ENUM("Notch Filter", nf),
	SOC_ENUM("Mic amplifier gain", gaa),
	SOC_ENUM("Soft mute", soft_mute),
	SOC_ENUM("ALC", alc),
	SOC_ENUM("ZC for ALC", zcaen),
	SOC_ENUM("ZC for MIC gain and PGA update", zcren),
};

static const struct snd_soc_dapm_widget cq93vc_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", DAVINCI_VC_REG12, 5, 0),
	SND_SOC_DAPM_ADC("ADC", "Capture", DAVINCI_VC_REG12, 4, 0),
	SND_SOC_DAPM_MICBIAS("Mic Bias", DAVINCI_VC_REG12, 2, 0),
	SND_SOC_DAPM_PGA("Mic Input", DAVINCI_VC_REG12, 3, 0, NULL, 0),
	SND_SOC_DAPM_INPUT("MICIN"),
	SND_SOC_DAPM_PGA("Spkr Output", DAVINCI_VC_REG12, 7, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("SP"),
	SND_SOC_DAPM_PGA("Line Output", DAVINCI_VC_REG12, 6, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("LINEO"),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Inputs */
	{"ADC", NULL, "Mic Input"},
	{ "Mic Input", NULL, "MICIN" },
	/* Outputs */
	{"Spkr Output", NULL, "DAC"},
	{"SP", NULL, "Spkr Output"},
	{"Line Output", NULL, "DAC"},
	{"LINEO", NULL, "Line Output"},
};

static int cq93vc_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	u8 reg = cq93vc_read(codec, DAVINCI_VC_REG09) & ~DAVINCI_VC_REG09_MUTE;

	if (mute)
		cq93vc_write(codec, DAVINCI_VC_REG09,
			     reg | DAVINCI_VC_REG09_MUTE);
	else
		cq93vc_write(codec, DAVINCI_VC_REG09, reg);

	return 0;
}

static int cq93vc_power(struct snd_soc_codec *codec, int power)
{

	u8 reg = cq93vc_read(codec, DAVINCI_VC_REG12) & ~DAVINCI_VC_REG12_PDSP;

	if (power)
		cq93vc_write(codec, DAVINCI_VC_REG12,
			     reg | DAVINCI_VC_REG12_PDSP);
	else
		cq93vc_write(codec, DAVINCI_VC_REG12, reg);

	return 0;
}

static int cq93vc_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct davinci_vc *davinci_vc = codec->control_data;

	switch (freq) {
	case 22579200:
	case 27000000:
	case 33868800:
		davinci_vc->cq93vc.sysclk = freq;
		return 0;
	}

	return -EINVAL;
}

static int cq93vc_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	unsigned int tmp;
	tmp = cq93vc_read(codec, DAVINCI_VC_REG12);
	switch (level) {
	case SND_SOC_BIAS_ON:
		pr_debug("SND_SOC_BIAS_ON\n");
		cq93vc_write(codec, DAVINCI_VC_REG12,
			     tmp |
			     (DAVINCI_VC_REG12_PDBS|DAVINCI_VC_REG12_PDCM));
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		pr_debug("SND_SOC_BIAS_STANDBY\n");
		cq93vc_write(codec, DAVINCI_VC_REG12,
			     tmp & ~DAVINCI_VC_REG12_PDBS);
		break;
	case SND_SOC_BIAS_OFF:
		/* force all power off */
		pr_debug("SND_SOC_BIAS_OFF\n");
		cq93vc_write(codec, DAVINCI_VC_REG12,
			     tmp & ~DAVINCI_VC_REG12_PDBS);
		break;
	}
	codec->bias_level = level;

	return 0;
}

#define CQ93VC_RATES	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000)
#define CQ93VC_FORMATS	(SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE)

static struct snd_soc_dai_ops cq93vc_dai_ops = {
	.digital_mute	= cq93vc_mute,
	.set_sysclk	= cq93vc_set_dai_sysclk,
};

struct snd_soc_dai cq93vc_dai = {
	.name = "CQ93VC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CQ93VC_RATES,
		.formats = CQ93VC_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CQ93VC_RATES,
		.formats = CQ93VC_FORMATS,},
	.ops = &cq93vc_dai_ops,
};
EXPORT_SYMBOL_GPL(cq93vc_dai);

static int cq93vc_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	cq93vc_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

static struct snd_soc_codec *cq93vc_codec;

static int cq93vc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct snd_soc_codec *codec;
	int ret;

	socdev->card->codec = cq93vc_codec;
	codec = socdev->card->codec;

	/* Register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(dev, "%s: failed to create pcms\n", pdev->name);
		return ret;
	}

	snd_soc_dapm_new_controls(codec, cq93vc_dapm_widgets,
				  ARRAY_SIZE(cq93vc_dapm_widgets));
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));
	snd_soc_dapm_new_widgets(codec);

	/* Set controls */
	snd_soc_add_controls(codec, cq93vc_snd_controls,
			     ARRAY_SIZE(cq93vc_snd_controls));

	/* Off, with power on */
	cq93vc_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	if (ret < 0) {
		printk(KERN_ERR "cq93vc: failed to register card\n");
		goto reset_err;
	}

	return 0;

reset_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return ret;
}

static int cq93vc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_cq93vc = {
	.probe = cq93vc_probe,
	.remove = cq93vc_remove,
	.resume = cq93vc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_cq93vc);

#ifdef CONFIG_PM_LOSS

static int cq93vc_power_changed(struct device *dev,
				enum sys_power_state s)
{
	struct davinci_vc *davinci_vc = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = davinci_vc->cq93vc.codec;
	switch (s) {
	case SYS_PWR_GOOD:
		cq93vc_power(codec, 1);
		break;
	case SYS_PWR_FAILING:
		cq93vc_power(codec, 0);
		break;
	default:
		BUG();
	}
	return 0;
}
#endif

static struct dev_pm_ops cq93vc_codec_pm_ops = {
#ifdef CONFIG_PM_LOSS
	.power_changed = cq93vc_power_changed,
#endif
};

static __init int cq93vc_codec_probe(struct platform_device *pdev)
{
	struct davinci_vc *davinci_vc = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL) {
		dev_dbg(davinci_vc->dev,
			"could not allocate memory for codec data\n");
		return -ENOMEM;
	}

	davinci_vc->cq93vc.codec = codec;

	cq93vc_dai.dev = &pdev->dev;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->dev = &pdev->dev;
	codec->name = "CQ93VC";
	codec->owner = THIS_MODULE;
	codec->read = cq93vc_read;
	codec->write = cq93vc_write;
	codec->set_bias_level = cq93vc_set_bias_level;
	codec->dai = &cq93vc_dai;
	codec->num_dai = 1;
	codec->control_data = davinci_vc;

	cq93vc_codec = codec;

	ret = snd_soc_register_codec(codec);
	if (ret) {
		dev_err(davinci_vc->dev, "failed to register codec\n");
		goto fail1;
	}

	ret = snd_soc_register_dai(&cq93vc_dai);
	if (ret) {
		dev_err(davinci_vc->dev, "could register dai\n");
		goto fail2;
	}
	return 0;

fail2:
	snd_soc_unregister_codec(codec);

fail1:
	kfree(codec);
	cq93vc_codec = NULL;

	return ret;
}

static int __devexit cq93vc_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	snd_soc_unregister_dai(&cq93vc_dai);
	snd_soc_unregister_codec(codec);

	cq93vc_codec = NULL;
	cq93vc_dai.dev = NULL;

	return 0;
}

static struct platform_driver cq93vc_codec_driver = {
	.driver = {
		   .name = "cq93vc",
		   .owner = THIS_MODULE,
		   .pm = &cq93vc_codec_pm_ops,
		   },
	.probe = cq93vc_codec_probe,
	.remove = __devexit_p(cq93vc_codec_remove),
};

static __init int cq93vc_init(void)
{
	return platform_driver_probe(&cq93vc_codec_driver, cq93vc_codec_probe);
}
module_init(cq93vc_init);

static __exit void cq93vc_exit(void)
{
	platform_driver_unregister(&cq93vc_codec_driver);
}
module_exit(cq93vc_exit);

MODULE_DESCRIPTION("Texas Instruments DaVinci ASoC CQ0093 Voice Codec Driver");
MODULE_AUTHOR("Miguel Aguilar");
MODULE_LICENSE("GPL");
