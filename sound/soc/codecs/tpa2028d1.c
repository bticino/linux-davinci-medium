/*
 * ALSA SoC Texas Instruments TPA2016D2  amplifier driver
 *
 * Copyright (C) Bticino S.p.A.
 *
 * Author: Davide Bonfanti <davide.bonfanti@bticino.it>
 *
 * Derived from TPA6130a2 driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <linux/delay.h>

#include "tpa2028d1.h"

static struct i2c_client *tpa2028d1_client;

static u8 tpa2028d1_regs[] = {1, /* address of first register */
			      0xC2, /* function control */
			      0x00, /* agc attack control */
			      0x00, /* agc release control */
			      0x00, /* agc hold time control */
			      0x0F, /* fixed gain control */
			      0x9F, /* agc control 1 */
			      0xC0, /* agc control 2 */ };
#define FUNCTION_CTRL	0x01
#define SWS		(1<<5)
#define ENABLE		(1<<6)
#define NG_EN		(1<<0)
#define AGC_ATTACK_CTRL	0x02
#define AGC_REL_CTRL	0x03
#define AGC_HOLD_T_CTRL	0x04
#define FIXED_GAIN_CTRL 0x05
#define AGC_CTRL_1	0x06
#define AGC_CTRL_2	0x07

/* This struct is used to save the context */
struct tpa2028d1_data {
	struct mutex mutex;
	unsigned char regs[7];
	unsigned int shutdown_state;
};

static int tpa2028d1_i2c_read(int reg)
{
	struct tpa2028d1_data *data;
	int val;

	BUG_ON(tpa2028d1_client == NULL);
	data = i2c_get_clientdata(tpa2028d1_client);

	/* If shutdown, return the cached value */
	if (!data->shutdown_state) {
		val = i2c_smbus_read_byte_data(tpa2028d1_client, reg);
		if (val < 0)
			dev_err(&tpa2028d1_client->dev, "Read failed\n");
		else
			data->regs[reg] = val;
	} else
		val = data->regs[reg];

	return val;
}

static void tpa2028d1_write_regs(void)
{
	unsigned int i;
	unsigned int cnt = ARRAY_SIZE(tpa2028d1_regs);
	u8 data[ARRAY_SIZE(tpa2028d1_regs)];
	struct	tpa2028d1_data *i2c_data;

	BUG_ON(tpa2028d1_client == NULL);
	i2c_data = i2c_get_clientdata(tpa2028d1_client);
	if (i2c_data->shutdown_state)
		return;

	for (i = 0; i < cnt; i++)
		data[i] = tpa2028d1_regs[i];

	mutex_lock(&i2c_data->mutex);
	if (i2c_master_send(tpa2028d1_client, data, cnt) != cnt)
		dev_err(&tpa2028d1_client->dev, "i2c write failed\n");
	mutex_unlock(&i2c_data->mutex);
}

static int tpa2028d1_get_reg(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;

	if (reg == FIXED_GAIN_CTRL)
		ucontrol->value.integer.value[0] = tpa2028d1_regs[reg];
	else {
		ucontrol->value.integer.value[0] =
					(tpa2028d1_regs[reg] >> shift) & mask;
		if (invert)
			ucontrol->value.integer.value[0] =
				mask - ucontrol->value.integer.value[0];
	}

	return 0;
}

static int tpa2028d1_set_reg(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);

	if (reg == FIXED_GAIN_CTRL)
		tpa2028d1_regs[reg] = val;
	else {
		if (invert)
			val = mask - val;

		if (((tpa2028d1_regs[reg] >> shift) & mask) == val)
			return 0;

		tpa2028d1_regs[reg] &= ~(mask << shift);
		tpa2028d1_regs[reg] |= val << shift;
	}
	tpa2028d1_write_regs();

	return 1;
}

static int tpa2028d1_initialize(void)
{
	struct tpa2028d1_data *data;
	int ret = 0;

	BUG_ON(tpa2028d1_client == NULL);
	data = i2c_get_clientdata(tpa2028d1_client);

	tpa2028d1_write_regs();
	return ret;
}

void tpa2028d1_pwron(void)
{
	struct	tpa2028d1_data *data;

	BUG_ON(tpa2028d1_client == NULL);
	data = i2c_get_clientdata(tpa2028d1_client);

	data->shutdown_state = 0;
	tpa2028d1_regs[1] &= (~SWS);
	tpa2028d1_initialize();
}
EXPORT_SYMBOL_GPL(tpa2028d1_pwron);

void tpa2028d1_shutdown(void)
{
	struct	tpa2028d1_data *data;
	u8	val[2];

	BUG_ON(tpa2028d1_client == NULL);
	data = i2c_get_clientdata(tpa2028d1_client);

	mutex_lock(&data->mutex);
	val[0] = FIXED_GAIN_CTRL;
	val[1] = 0;
	if (i2c_master_send(tpa2028d1_client, val, 2) != 2)
		dev_err(&tpa2028d1_client->dev, "i2c write failed\n");
	val[0] = FUNCTION_CTRL;
	val[1] = tpa2028d1_regs[1] | SWS;
	if (i2c_master_send(tpa2028d1_client, val, 2) != 2)
		dev_err(&tpa2028d1_client->dev, "i2c write failed\n");
	mutex_unlock(&data->mutex);
	tpa2028d1_regs[1] |= SWS;
	tpa2028d1_regs[5] = 0;
	data->shutdown_state = 1;
}
EXPORT_SYMBOL_GPL(tpa2028d1_shutdown);

static const struct snd_soc_dapm_widget tpa2028d1_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("TPA2028D1", NULL),
};

static const unsigned int tpa2028d1_gain[] = {
	TLV_DB_RANGE_HEAD(1),
	0, 30, TLV_DB_MINMAX_ITEM(0, 3000),
};

static const struct snd_kcontrol_new tpa2028d1_controls[] = {
	SOC_SINGLE_EXT_TLV("TPA2028D1 Volume",
			   5, 0, 31, 0, tpa2028d1_get_reg,
			   tpa2028d1_set_reg, tpa2028d1_gain),
};

int tpa2028d1_add_controls(struct snd_soc_codec *codec)
{
	struct	tpa2028d1_data *data;

	if (tpa2028d1_client == NULL)
		return -ENODEV;
	data = i2c_get_clientdata(tpa2028d1_client);

	snd_soc_dapm_new_controls(codec, tpa2028d1_dapm_widgets,
				ARRAY_SIZE(tpa2028d1_dapm_widgets));
	return snd_soc_add_controls(codec, tpa2028d1_controls,
						ARRAY_SIZE(tpa2028d1_controls));

}
EXPORT_SYMBOL_GPL(tpa2028d1_add_controls);

static int __devinit tpa2028d1_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct device *dev;
	struct tpa2028d1_data *data;

	dev = &client->dev;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Can not allocate memory\n");
		return -ENOMEM;
	}

	tpa2028d1_client = client;
	i2c_set_clientdata(tpa2028d1_client, data);
	mutex_init(&data->mutex);

	return 0;
}

static int __devexit tpa2028d1_remove(struct i2c_client *client)
{
	struct tpa2028d1_data *data = i2c_get_clientdata(client);

	kfree(data);
	tpa2028d1_client = NULL;

	return 0;
}

static const struct i2c_device_id tpa2028d1_id[] = {
	{ "tpa2028d1", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tpa2028d1_id);

static struct i2c_driver tpa2028d1_i2c_driver = {
	.driver = {
		.name = "tpa2028d1",
		.owner = THIS_MODULE,
	},
	.probe = tpa2028d1_probe,
	.remove = __devexit_p(tpa2028d1_remove),
	.id_table = tpa2028d1_id,
};

static int __init tpa2028d1_init(void)
{
	return i2c_add_driver(&tpa2028d1_i2c_driver);
}

static void __exit tpa2028d1_exit(void)
{
	i2c_del_driver(&tpa2028d1_i2c_driver);
}

MODULE_AUTHOR("Davide Bonfanti");
MODULE_DESCRIPTION("TPA2028D1 amplifier driver");
MODULE_LICENSE("GPL");

module_init(tpa2028d1_init);
module_exit(tpa2028d1_exit);
