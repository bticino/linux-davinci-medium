/*
 * drivers/i2c/input/keyboard
 *
 * Copyright (C) 2014 Bticino
 *
 * Written by Massimiliano Iuliano <massimiliano.iuliano@bticino.it>
 *            Davide Bonfanti <davide.bonfanti@bticino.it>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
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
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/backlight.h>
#include <linux/i2c/i2c_kb_touch.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/pm.h>
#include <linux/pm_loss.h>

#define I2C_KB_TOUCH_FIFO_LEN   5
#define I2C_KB_TOUCH_LED_NUM	6

char *name_led[] = {
	"led_ans_machine",
	"led_exc_call",
	"led_memo",
	"led_lock",
	"led_vct_red",
	"led_vct_green",
};

struct per_led {
	struct work_struct	work;
	enum led_brightness     brightness;
	struct led_classdev     led_cdev;
	char                    name[32];
	int			led_num;
	struct i2c_client	*client;
};

struct i2c_kb_touch_chip {
	struct mutex		lock;
	struct i2c_client	*client;
	unsigned                keys_down;
	bool			kp_enabled;
	bool			pm_suspend;
	struct backlight_device *bled;

	struct per_led		leds[6];
	struct work_struct	work_btn;
	struct delayed_work	reset_work;
	struct input_dev	*idev;
	unsigned short          keymap[6];
};

static int	genericbl_intensity;
int		dato_leds;
static struct	backlight_device *generic_backlight_device;

#define I2C_KB_TOUCH_MAX_DATA	8
#define DEFAULT_BRIGHTNESS	0
#define MAX_BRIGHTNESS		65

#define I2C_KB_TOUCH_CMD_LED	0x01
#define	I2C_KB_TOUCH_CMD_PWM    0x02
#define I2C_KB_TOUCH_CMD_INIT	0x03

static int i2c_kb_touch_read(struct i2c_kb_touch_chip *lm, u8 *buf, int len)
{
	int ret;

	ret = i2c_master_recv(lm->client, buf, len);
	if (unlikely(ret != len))
		dev_err(&lm->client->dev, "wanted %d bytes, got %d\n",
			len, ret);

	return ret;
}

static inline int i2c_kb_touch_ispress(u8 event)
{
	return (event & 0x01) ? 1 : 0;
}

static void process_keys(struct i2c_kb_touch_chip *lm)
{
	u8 key_fifo[I2C_KB_TOUCH_FIFO_LEN + 1];
	int ret;
	int i = 0;

	ret = i2c_kb_touch_read(lm, key_fifo, I2C_KB_TOUCH_FIFO_LEN);

	if (ret < 0) {
		dev_err(&lm->client->dev, "Failed reading fifo \n");
		return;
	}
	key_fifo[ret] = 0xff;

	if (key_fifo[0] == 0xff) {
		schedule_delayed_work(&lm->reset_work,  msecs_to_jiffies(3000));

		for (i = 0; i < 4 ; i++) {
			u8 key = i+1;
			int isdown = 0;
			unsigned short keycode = lm->keymap[key];
			input_event(lm->idev, EV_KEY, KEY_1 + key - 1, isdown);
			input_report_key(lm->idev, keycode, isdown);
			input_sync(lm->idev);
		}

		return;
	} else {
		while ((key_fifo[i]) != 0xff) {
			u8 key = i+1;
			int isdown = i2c_kb_touch_ispress(key_fifo[i]);
			unsigned short keycode = lm->keymap[key];

			dev_vdbg(&lm->client->dev, "key 0x%02x %s\n",
				 key, isdown ? "down" : "up");

			if (lm->kp_enabled) {

				input_event(lm->idev, EV_KEY, KEY_1 + key - 1,
					isdown);
				input_report_key(lm->idev, keycode, isdown);
				input_sync(lm->idev);
			}

			if (isdown)
				lm->keys_down++;
			else
				lm->keys_down--;
			i++;
		}
	}
}

static void i2c_kb_touch_work(struct work_struct *work)
{

	struct i2c_kb_touch_chip *lm;

	lm = container_of(work, struct i2c_kb_touch_chip, work_btn);
	process_keys(lm);

}

static irqreturn_t i2c_kb_touch_irq(int irq, void *data)
{
	struct i2c_kb_touch_chip *lm = data;

	schedule_work(&lm->work_btn);
	return IRQ_HANDLED;
}

static void i2c_kb_touch_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct per_led *ledm;

	ledm = container_of(led_cdev, struct per_led, led_cdev);
	ledm->brightness = value;
	schedule_work(&ledm->work);
}

static void i2c_kb_touch_reset_work(struct delayed_work *work)
{

	u8 data[3];
	int ret, len;
	struct i2c_kb_touch_chip *lm;

	lm = container_of(work, struct i2c_kb_touch_chip, reset_work);

	data[0] = I2C_KB_TOUCH_CMD_LED;
	data[2] = (u8)(dato_leds&(0x00ff));
	data[1] = (u8)(((dato_leds&(0xff00))>>8));
	len = 3;
	ret = i2c_master_send(lm->client, data, len);

	data[0] = I2C_KB_TOUCH_CMD_PWM;
	data[2] = (u8)(genericbl_intensity);
	data[1] = 0x00;
	len = 3;
	ret = i2c_master_send(lm->client, data, len);

}

static void i2c_kb_touch_led_work(struct work_struct *work)
{

	struct per_led *ledm;
	u8 ls_led;
	u8 data[3];
	int ret, len;
	int reset_word = 0x03;
	int half_word = 0x01;
	int set_word = 0x02;

	ledm = container_of(work, struct per_led, work);
	ls_led = ledm->led_num;

	switch (ledm->brightness) {

	case LED_FULL:
		data[0] = I2C_KB_TOUCH_CMD_LED;
		dato_leds = (dato_leds & (~(reset_word<<(2*(ls_led+2)))))|
			((set_word<<(2*(ls_led+2))));
		data[2] = (u8)(dato_leds&(0x00ff));
		data[1] = (u8)(((dato_leds&(0xff00))>>8));
		len = 3;
		ret = i2c_master_send(ledm->client, data, len);
		break;
	case LED_OFF:
		data[0] = I2C_KB_TOUCH_CMD_LED;
		dato_leds = (dato_leds & (~(reset_word<<(2*(ls_led+2)))));
		data[2] = (u8)(dato_leds&(0x00ff));
		data[1] = (u8)(((dato_leds&(0xff00))>>8));
		len = 3;
		ret = i2c_master_send(ledm->client, data, len);
		break;
	case LED_HALF:
		data[0] = I2C_KB_TOUCH_CMD_LED;
		dato_leds = (dato_leds & (~(reset_word<<(2*(ls_led+2)))))|
			((half_word<<(2*(ls_led+2))));
		data[2] = (u8)(dato_leds&(0x00ff));
		data[1] = (u8)(((dato_leds&(0xff00))>>8));
		len = 3;
		ret = i2c_master_send(ledm->client, data, len);
		break;
	default:
		data[0] = I2C_KB_TOUCH_CMD_LED;
		data[2] = (u8)(dato_leds&(0x00ff));
		data[1] = (u8)(((dato_leds&(0xff00))>>8));
		len = 3;
		ret = i2c_master_send(ledm->client, data, len);
		break;
	}
}

static struct i2c_driver i2c_kb_touch_i2c_driver;

static ssize_t i2c_kb_touch_show_disable(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_kb_touch_chip *lm = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", !lm->kp_enabled);
}

static ssize_t i2c_kb_touch_set_disable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct i2c_kb_touch_chip *lm = dev_get_drvdata(dev);
	int ret;
	unsigned long i;

	ret = strict_strtoul(buf, 10, &i);
	mutex_lock(&lm->lock);
	lm->kp_enabled = !i;
	mutex_unlock(&lm->lock);
	return count;
}

static DEVICE_ATTR(disable_kp, 0644, i2c_kb_touch_show_disable,
			i2c_kb_touch_set_disable);

static int genericbl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	u8 data[3];
	int ret, len;
	struct i2c_kb_touch_chip *lm_h = bl_get_data(bd);
	struct i2c_client *client = lm_h->client;

	genericbl_intensity = intensity;
	data[0] = I2C_KB_TOUCH_CMD_PWM;
	data[2] = (u8)(intensity);
	data[1] = 0x00;
	len = 3;
	ret = i2c_master_send(client, data, len);
	return 0;
}

static int genericbl_get_intensity(struct backlight_device *bd)
{
	return genericbl_intensity;
}

static struct backlight_ops genericbl_ops = {

	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = genericbl_get_intensity,
	.update_status  = genericbl_send_intensity,

};

static int __devinit i2c_kb_touch_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct input_dev *idev;
	struct i2c_kb_touch_chip *lm;
	int i;
	int err, len, ret;
	struct backlight_properties props;
	u8 data[3];
	const char *name = "i2c_kb_touch-bl";
	struct backlight_device *bd;

	lm = kzalloc(sizeof *lm , GFP_KERNEL);
	idev = input_allocate_device();
	i2c_set_clientdata(client, lm);
	lm->client = client;
	lm->idev = idev;
	INIT_WORK(&lm->work_btn, i2c_kb_touch_work);
	INIT_DELAYED_WORK(&lm->reset_work, i2c_kb_touch_reset_work);

	for (i = 0; i < I2C_KB_TOUCH_LED_NUM; i++) {
		snprintf(lm->leds[i].name,
		sizeof(lm->leds[i].name), "%s",
			name_led[i]);

		lm->leds[i].client = client;
		lm->leds[i].led_num = i;
		lm->leds[i].led_cdev.name = lm->leds[i].name;
		lm->leds[i].led_cdev.brightness_set = i2c_kb_touch_led_set;

		INIT_WORK(&lm->leds[i].work, i2c_kb_touch_led_work);

		err = led_classdev_register(&client->dev,
						&lm->leds[i].led_cdev);
	}

	dato_leds = 0;
	data[0] = I2C_KB_TOUCH_CMD_LED;
	data[2] = (u8)(0);
	data[1] = (u8)(0);
	len = 3;
	ret = i2c_master_send(client, data, len);
	mdelay(80);
	memset(&props, 0, sizeof(props));
	props.max_brightness = MAX_BRIGHTNESS;
	props.power = FB_BLANK_UNBLANK;
	props.brightness = DEFAULT_BRIGHTNESS;
	bd = backlight_device_register(
				name,
				&client->dev, lm, &genericbl_ops);
	bd->props.brightness = DEFAULT_BRIGHTNESS;

	lm->bled = bd;
	lm->bled->props.max_brightness = MAX_BRIGHTNESS;
	lm->bled->props.power = FB_BLANK_UNBLANK;
	lm->bled->props.brightness = DEFAULT_BRIGHTNESS;

	backlight_update_status(lm->bled);
	genericbl_intensity = DEFAULT_BRIGHTNESS;
	generic_backlight_device = lm->bled;

	data[0] = I2C_KB_TOUCH_CMD_PWM;
	data[2] = (u8)(genericbl_intensity);
	data[1] = 0x00;
	len = 3;
	ret = i2c_master_send(client, data, len);
	mdelay(80);

	data[0] = I2C_KB_TOUCH_CMD_INIT;
	data[2] = 0x01;
	data[1] = 0x00;
	len = 3;
	ret = i2c_master_send(client, data, len);
	mdelay(80);

	lm->kp_enabled = true;
	idev->name = "I2C_KB_TOUCH keypad";
	input_set_capability(idev, EV_KEY, KEY_1);
	input_set_capability(idev, EV_KEY, KEY_2);
	input_set_capability(idev, EV_KEY, KEY_3);
	input_set_capability(idev, EV_KEY, KEY_4);

	err = input_register_device(idev);
	if (err) {
		dev_dbg(&client->dev, "error registering input device\n");
		goto fail3;
	}

	err = request_irq(client->irq, i2c_kb_touch_irq,
			  IRQF_TRIGGER_FALLING | IRQF_DISABLED,
			  "i2c_kb_touch", lm);
	if (err) {
		dev_err(&client->dev, "could not get IRQ %d\n", client->irq);
		goto fail4;
	}

	device_init_wakeup(&client->dev, 1);
	enable_irq_wake(client->irq);

	return 0;

fail4:
	input_unregister_device(idev);
	idev = NULL;
fail3:
	device_remove_file(&client->dev, &dev_attr_disable_kp);

	input_free_device(idev);
	kfree(lm);
	return err;
}

static int __devexit i2c_kb_touch_remove(struct i2c_client *client)
{
	struct i2c_kb_touch_chip *lm = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < I2C_KB_TOUCH_LED_NUM; i++) {
		led_classdev_unregister(&lm->leds[i].led_cdev);
		cancel_work_sync(&lm->leds[i].work);
	}
	backlight_device_unregister(lm->bled);

	disable_irq_wake(client->irq);
	free_irq(client->irq, lm);
	cancel_work_sync(&lm->work_btn);

	input_unregister_device(lm->idev);
	device_remove_file(&lm->client->dev, &dev_attr_disable_kp);
	kfree(lm);

	return 0;
}

#ifdef CONFIG_PM
/*
 * We don't need to explicitly suspend the chip, as it already switches off
 * when there's no activity.
 */
static int i2c_kb_touch_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct i2c_kb_touch_chip *lm = i2c_get_clientdata(client);

	set_irq_wake(client->irq, 0);
	disable_irq(client->irq);

	mutex_lock(&lm->lock);
	lm->pm_suspend = true;
	mutex_unlock(&lm->lock);

	return 0;
}

static int i2c_kb_touch_resume(struct i2c_client *client)
{
	struct i2c_kb_touch_chip *lm = i2c_get_clientdata(client);

	mutex_lock(&lm->lock);
	lm->pm_suspend = false;
	mutex_unlock(&lm->lock);

	enable_irq(client->irq);
	set_irq_wake(client->irq, 1);

	return 0;
}
#else
#define i2c_kb_touch_suspend	NULL
#define i2c_kb_touch_resume	NULL
#endif

static const struct i2c_device_id i2c_kb_touch_id[] = {
	{ "i2c_kb_touch", 0 },
	{ }
};


#ifdef CONFIG_PM_LOSS

static int gekko_I2c_kb_touch_power_changed(struct i2c_client *client,
						enum sys_power_state s)
{
	u8 data[3];
	int len, ret;

	switch (s) {
	case SYS_PWR_GOOD:
		data[0] = I2C_KB_TOUCH_CMD_PWM;
		data[2] = (u8)(genericbl_intensity);
		data[1] = 0x00;
		len = 3;
		ret = i2c_master_send(client, data, len);
		break;
	case SYS_PWR_FAILING:
		data[0] = I2C_KB_TOUCH_CMD_PWM;
		data[2] = DEFAULT_BRIGHTNESS;
		data[1] = 0x00;
		len = 3;
		ret = i2c_master_send(client, data, len);
		break;
	default:
		BUG();
	}

	return 0;
}

#endif

static struct i2c_driver i2c_kb_touch_i2c_driver = {
	.driver = {
		.name	= "i2c_kb_touch",
	},
	.probe		= i2c_kb_touch_probe,
	.remove		= __devexit_p(i2c_kb_touch_remove),
	.suspend	= i2c_kb_touch_suspend,
	.resume		= i2c_kb_touch_resume,
	.id_table	= i2c_kb_touch_id,
#ifdef CONFIG_PM_LOSS
	.power_changed	= gekko_I2c_kb_touch_power_changed,
#endif
};
MODULE_DEVICE_TABLE(i2c, i2c_kb_touch_id);

static int __init i2c_kb_touch_init(void)
{
	return i2c_add_driver(&i2c_kb_touch_i2c_driver);
}

static void __exit i2c_kb_touch_exit(void)
{

	i2c_del_driver(&i2c_kb_touch_i2c_driver);
}
module_init(i2c_kb_touch_init);
module_exit(i2c_kb_touch_exit);

MODULE_AUTHOR("Massimiliano Iuliano <massimiliano.iuliano@.bticino.it>");
MODULE_DESCRIPTION("I2C_KB_TOUCH keypad driver");
MODULE_LICENSE("GPL");
