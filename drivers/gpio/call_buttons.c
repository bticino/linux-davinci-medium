/*
 * Copyright (C) 2013 Shidean, Legrand.
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * This Driver allows to read button number  using a GPIO as interrupt
 * that signals when data is available. It also provides a character device
 * driver interface for userspace controlling.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <mach/amico.h>

/* For registration of charatcer device */
#define DEVCOUNT 1
#define DRIVER_NAME "call_buttons"
#define CALL_BUTTONS_DEVICE_NAME "call_buttons"

/* Key status */
#define KEY_STATUS_DOWN			0
#define KEY_STATUS_UP			1
#define KEY_STATUS_DOWNX		2
#define KEY_STATUS_LPRESS		3

/* Key set */
#define GPIO_DEBOUNCE_TO		10
#define KEY_LDOWN_TIMER			100
#define LPRESS_TIMES			20

/* Key code*/
#define CALL_IU_CODE		0x27
#define CALL_MC_CODE		0x0c
#define CALL_IP_CODE		0x28	/* means read IP address */

/* Key number  */
#define IU_CALL			0
#define MC_CALL			1

/*
*	code: short key code
*	locde: long key code
*/
struct irq_call_key {
	int gpio;
	int irq;
	int status;
	int code;
	int lcode;
	struct timer_list key_timer;
};

static int buttons_major;
static struct cdev c_dev;
static struct class *buttons_class;
static const char module_name[] = "call_buttons";

static struct irq_call_key call_key[] = {
	{	/* Call_IU */
		.gpio = piGPIO_INTn,
		.irq = IRQ_DM365_GPIO0_3,
		.code = CALL_IU_CODE,
		.lcode = CALL_IP_CODE,
	} , { /* Call MC */
		.gpio = piPENIRQn,
		.irq = IRQ_DM365_GPIO0_5,
		.code = CALL_MC_CODE,
	},
};

/* For interrupt procedure */
static DECLARE_WAIT_QUEUE_HEAD(buttons_waitq);
static int lpress;
static int gpio_num;
static volatile int gpio_int;

/*
 * ISR
 */
static irqreturn_t buttons_int_handler(int irq, void *dev_id)
{
	int i, irqs;
	int button;

	/* Get irqs for call buttons */
	for (i = 0, irqs = 0, button = 0; i < ARRAY_SIZE(call_key); i++) {
		if (!gpio_get_value(call_key[i].gpio)) {
			button = i;
			irqs++;
		}
	}

	/* Only support one call button irq at same time */
	if (irqs == 1) {
		gpio_num = button;
		disable_irq(IRQ_DM365_GPIO0);
		mod_timer(&call_key[gpio_num].key_timer, jiffies
			+ msecs_to_jiffies(GPIO_DEBOUNCE_TO));
	}

	return IRQ_RETVAL(IRQ_HANDLED);
}

/*
 * Char device stuff
 */
static int buttons_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int buttons_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t buttons_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char key_code;

	if (!count)
		return 0;

	/*if gpio_int=0, sleep here*/
	wait_event_interruptible(buttons_waitq, gpio_int);
	/*running to this line, gpio_int is 1*/
	gpio_int = 0;

	if (lpress == 1) {
		lpress = 0;
		key_code = call_key[gpio_num].lcode;
		enable_irq(IRQ_DM365_GPIO0);
	} else {
		key_code = call_key[gpio_num].code;
	}

	copy_to_user(buf, &key_code, 1);

	return 1;
}

static unsigned int buttons_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &buttons_waitq, wait);

	if (gpio_int)
		mask = (POLLIN | POLLRDNORM);

	return mask;
}

static const struct file_operations buttons_fops = {
	.owner	= THIS_MODULE,
	.open	= buttons_open,
	.release	= buttons_release,
	.read	= buttons_read,
	.poll		= buttons_poll,
};

/*
 * callback
 */
static void buttons_callback(unsigned long data)
{
	static int times;
	int key_num = data;

	if (!gpio_get_value(call_key[key_num].gpio)) {
		/* Long Press IU */
		if ((key_num == IU_CALL) && (times++ > LPRESS_TIMES)) {
			times = 0;
			lpress = 1;
			call_key[key_num].status = KEY_STATUS_LPRESS;
			gpio_int = 1;
			wake_up_interruptible(&buttons_waitq);
			return;
		}

		call_key[key_num].status = KEY_STATUS_DOWN;
		mod_timer(&call_key[key_num].key_timer,
			jiffies + msecs_to_jiffies(KEY_LDOWN_TIMER));
	} else {
		/* Short Press */
		switch (call_key[key_num].status) {
		case KEY_STATUS_DOWN:
			gpio_int = 1;
			wake_up_interruptible(&buttons_waitq);
			break;
		default:
			gpio_int = 0;
		}

		times = 0;
		call_key[key_num].status = KEY_STATUS_UP;
		enable_irq(IRQ_DM365_GPIO0);
	}
}

static int call_buttons_init(void)
{
	int i;
	int result;

	for (i = 0; i < ARRAY_SIZE(call_key); i++) {
		result = request_threaded_irq(call_key[i].irq,
			buttons_int_handler, NULL, 0, "buttons_irq",
			(void *)gpio_int);
		if (result < 0) {
			printk(KERN_ERR "[buttons_driver] unable to get IRQ %d.\n",
				result);
			goto err;
		}

		printk(KERN_INFO "[buttons_driver] get IRQ %d successful!\n",
			call_key[i].irq);
		call_key[i].status = KEY_STATUS_UP;
		call_key[i].key_timer.function = buttons_callback;
		call_key[i].key_timer.data = i;
		init_timer(&call_key[i].key_timer);
	}

	return 0;
err:
	for (i -= 1; i >= 0; i--) {
		free_irq(call_key[i].irq, (void *)gpio_int);
		del_timer(&call_key[i].key_timer);
	}

	return result;
}

static void call_buttons_del(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(call_key); i++) {
		del_timer(&call_key[i].key_timer);
		free_irq(call_key[i].irq, (void *)gpio_int);
	}
}

/*
 * Module stuff
 */
static int __init buttons_drv_init(void)
{
	struct device *dev;
	dev_t dev_id;
	int result;
	int minor = 0;

	/***** character driver stuff *****/
	result = alloc_chrdev_region(&dev_id, 0, DEVCOUNT,
		CALL_BUTTONS_DEVICE_NAME);
	if (result < 0) {
		printk(KERN_ERR "[buttons_driver] can't alloc device number!\n");
		return -ENODEV;
	}

	buttons_major = MAJOR(dev_id);
	register_chrdev(buttons_major, DRIVER_NAME, &buttons_fops);

	buttons_class = class_create(THIS_MODULE, module_name);
	if (!buttons_class) {
		printk(KERN_ERR "[buttons_driver] class_create failed!\n");
		goto fail1;
	}

	cdev_init(&c_dev, &buttons_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops   = &buttons_fops;

	result = cdev_add(&c_dev, dev_id, DEVCOUNT);
	if (result) {
		printk(KERN_ERR "[buttons_driver] Error %d adding buttons",
			result);
		goto fail2;
	}

	dev = device_create(buttons_class, NULL, MKDEV(buttons_major, minor),
		NULL, "%s%d", module_name, minor);
	result = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (result) {
		printk(KERN_ERR "[buttons_driver] device_create failed!\n");
		goto fail2;
	}

	result = call_buttons_init();
	if (result) {
		printk(KERN_ERR "[buttons_driver] call init failed!\n");
		goto fail3;
	}

	return 0;

fail3:
	device_destroy(buttons_class, MKDEV(buttons_major, minor));
fail2:
	class_destroy(buttons_class);
fail1:
	unregister_chrdev(buttons_major, DRIVER_NAME);

	return result;
}
module_init(buttons_drv_init);

static void __exit buttons_drv_exit(void)
{
	call_buttons_del();
	device_destroy(buttons_class, MKDEV(buttons_major, 0));
	class_destroy(buttons_class);
	unregister_chrdev_region(buttons_major, DEVCOUNT);
	cdev_del(&c_dev);
}
module_exit(buttons_drv_exit);

/* Module information */
MODULE_DESCRIPTION("call buttons device driver with GPIO signal as interrupt");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0.2");

/* End of File*/
