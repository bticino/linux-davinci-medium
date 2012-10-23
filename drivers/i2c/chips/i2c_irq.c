/*
 * Copyright (C) 2012 Shidean, Legrand.
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
 * This Driver allows to read from I2C channel using a GPIO as interrupt
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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <mach/amico.h>
#include <linux/i2c/i2c_irq.h>

#define I2C_IRQ_DEVICE_NAME "i2c_irq"

/* For registration of charatcer device */
#define DEVCOUNT 1
#define DRIVER_NAME "i2c_irq"
static int i2c_major;
static struct cdev c_dev;
static struct class *i2c_class;

static const char module_name[] = "i2c_irq";

struct _i2c_data {
	struct i2c_client *client;
	struct i2c_irq_platform_data *pdata;
};
static struct _i2c_data i2c_data;

/* For interrupt procedure */
static DECLARE_WAIT_QUEUE_HEAD(i2c_waitq);
static volatile int gpio_int;/* = 0;*/
static int gpio_num;

/*
 * ISR
 */
static irqreturn_t i2c_int_handler(int irq, void *dev_id)
{
	if (!gpio_get_value(gpio_num)) {
		gpio_int = 1;
		wake_up_interruptible(&i2c_waitq);
	}
	return IRQ_RETVAL(IRQ_HANDLED);
}

/*
 * Char device stuff
 */

static int i2c_open(struct inode *inode, struct file *file)
{
	struct _i2c_data *data = &i2c_data;
	file->private_data = data;

	return 0;
}

static int i2c_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t i2c_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct _i2c_data *data = file->private_data;
	struct i2c_client *client = data->client;

	int ret = 0;
	char *kbuf;

	if (!count)
		return 0;

	/*if gpio_int=0, sleep here*/
	wait_event_interruptible(i2c_waitq, gpio_int);
	/*running to this line, gpio_int is 1*/
	gpio_int = 0;

	kbuf = kzalloc(count, GFP_KERNEL);
	if (kbuf == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(client, kbuf, count);
	if (ret < 0) {
		kfree(kbuf);
		return ret;
	}

	if (copy_to_user(buf, kbuf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	kfree(kbuf);
	return ret;
}

static unsigned int i2c_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &i2c_waitq, wait);

	if (gpio_int)
		mask = (POLLIN | POLLRDNORM);

	return mask;
}

static ssize_t i2c_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct _i2c_data *data = file->private_data;
	struct i2c_client *client = data->client;

	int ret = 0;
	char *kbuf;

	if (!count)
		return 0;

	kbuf = kzalloc(count, GFP_KERNEL);
	if (kbuf == NULL)
		return -ENOMEM;

	if (copy_from_user(kbuf, buf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	ret = i2c_master_send(client, kbuf, count);
	if (ret < 0) {
		printk(KERN_ERR "[i2c driver] i2c write error\n");
		kfree(kbuf);
		return -EIO;
	}

	kfree(kbuf);
	return ret;
}

static const struct file_operations i2c_fops = {
	.owner    = THIS_MODULE,
	.open     = i2c_open,
	.release  = i2c_release,
	.read     = i2c_read,
	.write    = i2c_write,
	.poll     = i2c_poll,
};

/*
 * I2C device driver stuff
 */

static int __devinit i2c_irq_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret = 0;
	struct _i2c_data *data = &i2c_data;

	if (client->dev.platform_data) {
		data->client = client;
		data->pdata = \
			(struct at24_platform_data *)client->dev.platform_data;
	} else {
		printk(KERN_ERR "[i2c driver] unable to find device.\n");
		return -ENODEV;
	}

	gpio_num = data->pdata->gpio;

	ret = request_threaded_irq(data->pdata->irq, i2c_int_handler,
		NULL, 0, "i2c_irq", (void *)gpio_int);
	if (ret < 0) {
		printk(KERN_ERR "[i2c driver] unable to get IRQ %d.\n", ret);
		free_irq(data->pdata->irq, (void *)gpio_int);
		return -EBUSY;
	} else
		printk(KERN_INFO "[i2c driver] get IRQ %d successful!\n", \
			data->pdata->irq);

	i2c_set_clientdata(client, data);

	printk(KERN_INFO "[i2c driver] i2c_irq proble done\n");

	return 0;
}

static int __devexit i2c_irq_remove(struct i2c_client *client)
{
	struct _i2c_data *data = &i2c_data;
	free_irq(data->pdata->irq, (void *)gpio_int);
	return 0;
}


static const struct i2c_device_id i2c_irq_id_table[] = {
	{"pic16f1939", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, i2c_irq_id_table);

static struct i2c_driver i2c_irq_driver = {
	.driver = {
		.name = "i2c_irq_driver",
		.owner = THIS_MODULE,
	},
	.probe = i2c_irq_probe,
	.remove = i2c_irq_remove,
	.id_table = i2c_irq_id_table,
};

/*
 * Module stuff
 */

static int __init i2c_drv_init(void)
{
	struct device *dev;
	dev_t dev_id;
	int result;
	int minor = 0;

	/***** character driver stuff *****/
	result = alloc_chrdev_region(&dev_id, 0, DEVCOUNT, I2C_IRQ_DEVICE_NAME);
	if (result < 0) {
		printk(KERN_ERR "[i2c driver] can't alloc device number!\n");
		return -ENODEV;
	}

	i2c_major = MAJOR(dev_id);
	register_chrdev(i2c_major, DRIVER_NAME, &i2c_fops);

	i2c_class = class_create(THIS_MODULE, module_name);
	if (!i2c_class) {
		printk(KERN_ERR "[i2c driver] class_create failed!\n");
		cdev_del(&c_dev);
		unregister_chrdev(i2c_major, DRIVER_NAME);
	}

	cdev_init(&c_dev, &i2c_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops   = &i2c_fops;

	result = cdev_add(&c_dev, dev_id, DEVCOUNT);
	if (result) {
		printk(KERN_ERR "[i2c driver] Error %d adding i2c", result);
		cdev_del(&c_dev);
		unregister_chrdev_region(i2c_major, DEVCOUNT);
		class_destroy(i2c_class);
		return result;
	}

	dev = device_create(i2c_class, NULL, MKDEV(i2c_major, minor), NULL,
		"%s%d", module_name, minor);
	result = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (result) {
		printk(KERN_ERR "[i2c driver] device_create failed!\n");
		cdev_del(&c_dev);
		unregister_chrdev_region(i2c_major, DEVCOUNT);
		class_destroy(i2c_class);
		return result;
	}

	return i2c_add_driver(&i2c_irq_driver);
}
module_init(i2c_drv_init);

static void __exit i2c_drv_exit(void)
{
	int minor = 0;
	struct _i2c_data *data = &i2c_data;

	i2c_del_driver(&i2c_irq_driver);
	if (data->client != NULL)
		i2c_unregister_device(data->client);

	device_destroy(i2c_class, MKDEV(i2c_major, minor));
	cdev_del(&c_dev);
	unregister_chrdev_region(i2c_major, DEVCOUNT);
	class_destroy(i2c_class);
}
module_exit(i2c_drv_exit);

/* Module information */
MODULE_DESCRIPTION("I2C device driver with GPIO signal as interrupt");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0.1");

/* End of File*/
