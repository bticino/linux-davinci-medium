/*
 * ASoC driver for BTICINO GEKKO platform
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
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <sound/soc-dai.h>
#include <sound/initval.h>


#include <mach/dm365.h>
#include <mach/gekko.h>
#include <mach/clock.h>
#include <linux/gpio.h>


#include <asm/dma.h>
#include <asm/mach-types.h>

#include <mach/asp.h>
#include <mach/edma.h>
#include <mach/mux.h>

#include "../codecs/cq93vc.h"
#include "../codecs/tpa2028d1.h"
#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-vcif.h"
#include <sound/davinci_gekko_asoc.h>
#include <linux/pm_loss.h>
#include <linux/delay.h>

static struct gekko_asoc_platform_data gekko_asoc_priv;
struct delayed_work gekko_audio_work;
int cnt_late;

struct  gekko_aud_data ;

static struct platform_device *gekko_snd_device[1];

enum gekko_audio_states {
	EN_AUDIO_REC_MIC = 0,
	EN_AUDIO_REC_SCS,
	EN_AUDIO_PLAY_LOC,
	EN_AUDIO_PLAY_LOC_SCS,
	EN_AUDIO_PASSTHROUGH,
	EN_AUDIO_RINGING,
	EN_AUDIO_PASSTHR_INT,
	EN_AUDIO_HALF_DUPL_IN,
	EN_AUDIO_OFF,
	EN_AUDIO_RINGING_LOC,
};

struct gekko_data {
	enum gekko_audio_states state;
};

char gekko_audio_enables[][8] = {
/*st/EN_5V_FON/EN_5V_A/MUTE_SPK/EN_CH_SUPP/poEN_T_COIL/EN_LOCAL_MIC/ATT_INTER*/
	{EN_AUDIO_REC_MIC,      1, 0, 1, 0, 0, 0, 0},
	{EN_AUDIO_REC_SCS,      1, 1, 0, 0, 0, 1, 0},
	{EN_AUDIO_PLAY_LOC,     0, 1, 1, 0, 1, 1, 0},
	{EN_AUDIO_PLAY_LOC_SCS, 1, 1, 0, 0, 0, 1, 0},
	{EN_AUDIO_PASSTHROUGH,  1, 1, 0, 0, 1, 0, 0},
	{EN_AUDIO_RINGING,      0, 1, 1, 1, 0, 1, 0},
	{EN_AUDIO_PASSTHR_INT,  1, 1, 0, 0, 1, 0, 1},
	{EN_AUDIO_HALF_DUPL_IN, 1, 1, 0, 0, 1, 1, 1},
	{EN_AUDIO_OFF,		0, 0, 1, 0, 0, 0, 0},
	{EN_AUDIO_RINGING_LOC,  0, 1, 1, 0, 0, 1, 0},
};

struct gekko_data my_data;

#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | \
		SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)

void gekko_snd_teardown(void)
{
	if (gpio_get_value(poEN_5V_A))
		tpa2028d1_shutdown();
	gpio_set_value(poEN_5V_FON, 0);
	mdelay(10);
	gpio_set_value(poEN_5V_A, 0);
	gpio_set_value(poMUTE_SPK, 1);
	gpio_set_value(poEN_T_COIL, 0);
	gpio_set_value(poEN_CH_SUPP, 0);
	gpio_set_value(poEN_LOCAL_MIC, 0);
	gpio_set_value(poATT_INTER, 0);
}

void gekko_audio_late_setup(struct work_struct *work)
{
	int cnt = cnt_late;

	if (!gekko_audio_enables[cnt][2] && gpio_get_value(poEN_5V_A))
		tpa2028d1_shutdown();
	gpio_set_value(poEN_5V_A, gekko_audio_enables[cnt][2]);
	if (gekko_audio_enables[cnt][2]) {
		mdelay(20); /* wait chip to turn on before setting registers */
		tpa2028d1_pwron();
	}

	gpio_set_value(poMUTE_SPK, gekko_audio_enables[cnt][3]);
	gpio_set_value(poEN_CH_SUPP, gekko_audio_enables[cnt][4]);
	gpio_set_value(poEN_T_COIL, gekko_audio_enables[cnt][5]);
	gpio_set_value(poEN_LOCAL_MIC, gekko_audio_enables[cnt][6]);
	gpio_set_value(poATT_INTER, gekko_audio_enables[cnt][7]);
}

void gekko_set_audio_state(void)
{
	int cnt;

	for (cnt = 0; cnt < ARRAY_SIZE(gekko_audio_enables); cnt++) {
		if (gekko_audio_enables[cnt][0] == my_data.state)
			break;
	}
	cnt_late = cnt;
	if (gekko_audio_enables[cnt][0] != my_data.state) {
		pr_debug("state %d not found", my_data.state);
		return;
	}

	if (my_data.state == EN_AUDIO_OFF) {
		gekko_snd_teardown();
		return;
	}
	gpio_set_value(poEN_5V_FON, gekko_audio_enables[cnt][1]);

	if (timer_pending(&gekko_audio_work.timer))
		cancel_delayed_work(&gekko_audio_work);
	INIT_DELAYED_WORK(&gekko_audio_work, gekko_audio_late_setup);
	if ((my_data.state == EN_AUDIO_PASSTHROUGH) ||
	    (my_data.state == EN_AUDIO_PASSTHR_INT))
		schedule_delayed_work(&gekko_audio_work,
					msecs_to_jiffies(500));
	else
		schedule_delayed_work(&gekko_audio_work,
					msecs_to_jiffies(10));
}

void gekko_snd_shutdown(struct snd_pcm_substream *substream)
{
	gekko_snd_teardown();
	my_data.state = EN_AUDIO_OFF;
}

static int gekko_snd_hw_params(struct snd_pcm_substream *substream,
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

static struct snd_soc_ops gekko_snd_ops = {
	.hw_params = gekko_snd_hw_params,
	.shutdown = gekko_snd_shutdown,
};

/* davinci-gekko machine dapm widgets */
static const struct snd_soc_dapm_widget cq93_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Speakers out", NULL),
};

/* davinci-gekko machine connections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Speakers connected to SP (actually not connected !) */
	{ "Speakers out", NULL, "SP", },
	{ "TPA2028D1", NULL, "SP", },
	{ "Gekko_Enables", NULL, "SP", },
};

static const struct snd_soc_dapm_widget gekko_dapm_widgets[] = {
				SND_SOC_DAPM_OUTPUT("Gekko_Enables"),
};

static const char *gekko_aud_states_txt[] = {"Rec Mic", "Rec SCS", "Play Loc",
					     "Play Loc Scs", "Passthrough",
					     "Ringing", "Passthrough Intercom",
					     "Half Duplex IN", "OFF",
					     "Ringing Loc"};

static int gekko_set(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	if (my_data.state == ucontrol->value.enumerated.item[0])
		return 0;
	my_data.state = ucontrol->value.enumerated.item[0];

	gekko_set_audio_state();
	return 0;
}

static int gekko_get(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = my_data.state;
	return 0;
}

static const struct soc_enum gekko_aud_states =
	SOC_ENUM_SINGLE(0, 0, 10,  gekko_aud_states_txt);

static const struct snd_kcontrol_new gekko_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "Gekko_Aud_St",
		.info = snd_soc_info_enum_double,
		.get = gekko_get,
		.put = gekko_set,
		.private_value = (unsigned long)&gekko_aud_states,
	},
};

static int gekko_snd_init(struct snd_soc_codec *codec)
{
	int err;

	pr_debug("gekko_cq93_init(%p)\n", codec);
	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(codec, cq93_dapm_widgets,
				  ARRAY_SIZE(cq93_dapm_widgets));

	tpa2028d1_add_controls(codec);

	snd_soc_dapm_new_controls(codec, gekko_dapm_widgets,
				  ARRAY_SIZE(gekko_dapm_widgets));

	err = snd_soc_add_controls(codec, gekko_controls,
				 ARRAY_SIZE(gekko_controls));
	if (err)
		printk(KERN_ERR "ERROR adding controls %d\n", err);

	my_data.state = EN_AUDIO_OFF;
	gekko_set_audio_state();
	/* Set up davinci-amico-i specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(codec, "SP");

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Spkr Output");
	snd_soc_dapm_enable_pin(codec, "MICIN");

	snd_soc_dapm_sync(codec);

	return 0;
}

/* amico-i digital audio interface glue - connects codec <--> CPU */

static struct snd_soc_dai_link dm365_gekko_dai[] = {
	{
		.name = "Voice Codec - CQ93VC",
		.stream_name = "CQ93",
		.cpu_dai = &davinci_vcif_dai,
		.codec_dai = &cq93vc_dai,
		.init = gekko_snd_init,
		.ops = &gekko_snd_ops,
	},
};

/* davinci dm365 amico-i audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_gekko[] = {
	{
		.name = "DaVinci GEKKO VOICE",
		.platform = &davinci_soc_platform,
		.dai_link = &dm365_gekko_dai[0],
		.num_links = 1,
	},
};

/* amico-i audio subsystem */
static struct snd_soc_device dm365_gekko_snd_devdata[] = {
	{
		.card = &dm365_snd_soc_card_gekko[0],
		.codec_dev = &soc_codec_dev_cq93vc,
	},
};

static int gekko_asoc_probe(struct platform_device *pdev)
{
	int ret;
	int id = pdev->id;
	struct gekko_asoc_platform_data *pdata = pdev->dev.platform_data;
	if (!machine_is_gekko() || (id > 1))
		return -ENODEV;

	if (!pdata->ext_circuit_power)
		return -EINVAL;

	memcpy(&gekko_asoc_priv, pdata, sizeof(gekko_asoc_priv));

	gekko_snd_device[id] = platform_device_alloc("soc-audio", id);
	if (!gekko_snd_device[id])
		return -ENOMEM;
	platform_set_drvdata(gekko_snd_device[id],
			&dm365_gekko_snd_devdata[id]);
	dm365_gekko_snd_devdata[id].dev = &gekko_snd_device[id]->dev;

	ret = platform_device_add(gekko_snd_device[id]);
	if (ret)
		platform_device_put(gekko_snd_device[id]);

	return ret;
}

static int __devexit gekko_asoc_remove(struct platform_device *pdev)
{
	int id = pdev->id;

	platform_device_unregister(gekko_snd_device[id]);
	return 0;
}

#ifdef CONFIG_PM_LOSS
static int gekko_asoc_power_changed(struct device *dev,
				 enum sys_power_state s)
{
	int ret = -EIO;

	switch (s) {
	case SYS_PWR_GOOD:
		gekko_asoc_priv.ext_circuit_power(1);
		gekko_audio_late_setup(&gekko_asoc_priv.delayed_work.work);
		break;
	case SYS_PWR_FAILING:
		gekko_asoc_priv.ext_circuit_power(0);
		ret = cancel_delayed_work(&gekko_asoc_priv.delayed_work);
		break;
	default:
		BUG();
		ret = -ENODEV;
	}
	return ret;
}

static struct dev_pm_ops gekko_asoc_dev_pm_ops = {
	.power_changed = gekko_asoc_power_changed,
};
#endif

static struct platform_driver gekko_asoc_driver = {
	.probe = gekko_asoc_probe,
	.remove = __devexit_p(gekko_asoc_remove),
	.driver = {
		.name = "gekko-asoc",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_LOSS
		.pm = &gekko_asoc_dev_pm_ops,
#endif
	},
};

static int __init gekko_asoc_init(void)
{
	return platform_driver_register(&gekko_asoc_driver);
}

static void __exit gekko_asoc_exit(void)
{
	platform_driver_unregister(&gekko_asoc_driver);
}

module_init(gekko_asoc_init);
module_exit(gekko_asoc_exit);

MODULE_AUTHOR("bticino s.p.a.");
MODULE_DESCRIPTION("TI DAVINCI gekko ASoC driver");
MODULE_LICENSE("GPL");
