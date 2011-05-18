/*
 * Copyright (C) 2011 Bticino S.p.A.
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
 * Zarlink zl38005 (Acoustic Echo Canceller)
 * Some not public information, delivered by Zarlink,
 * has been used to implement the following code.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include "zl38005.h"

MODULE_DESCRIPTION("zl38005 codec driver");
MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_LICENSE("GPL");

#ifdef DEBUG
#define deb(args...)      printk(KERN_DEBUG "%s: " __func__, ##args);
#else
#define deb(args...)
#endif

/* ZL38005 driver private data */
struct zl38005 {
	dev_t			dev;
	struct spi_device	*spi;
	struct list_head	device_entry;
};

/* For registeration of charatcer device */
static struct cdev c_dev;

int module_usage_count;

struct zl38005 zl38005_data;

static DEFINE_MUTEX(zl38005_list_lock);
static LIST_HEAD(zl38005_list);

struct zl38005_ioctl_par {
	unsigned int		addr;
	unsigned int		data;
};

/* ioctl() calls that are permitted to the /dev/zl38005 interface. */
#define ZL_MAGIC 'G'
#define ZL_RD_REG             _IOR(ZL_MAGIC, 0x09, struct zl38005_ioctl_par)
#define ZL_WR_REG             _IOR(ZL_MAGIC, 0x0a, struct zl38005_ioctl_par)

static struct zl38005_ioctl_par zl38005_ioctl_par;

static int zl38005_rd_ctrl_reg(struct device *dev, u8 *val)
{
	int err;
	u8 cmd;
	u8 rx_buf[3];
	struct zl38005 *zl38005 = dev_get_drvdata(dev);

	cmd = (CMD_VALID | CMD_READ | CMD_LEN_16_BIT | CMD_TYPE_CTRL);
	deb("%s TX cmd=%X\n", cmd);
	err = spi_write_then_read(zl38005->spi, &cmd, sizeof cmd,
                        rx_buf, 2);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	*val = rx_buf[1];
	return 0;
}

static int zl38005_wr_ctrl_reg(struct device *dev, u8 val, u8 write)
{
	struct spi_message msg;
	u8 buf[2];
	struct spi_transfer xfer = {
		.len = 2,
		.tx_buf = buf,
	};
	int err;
	u8 cmd = (CMD_VALID | CMD_WRITE | CMD_LEN_8_BIT | CMD_TYPE_CTRL);
	struct zl38005 *zl38005 = dev_get_drvdata(dev);

	spi_message_init(&msg);
	buf[0] = cmd;
	buf[1] = val | (write == 1 ? CTRL_OP_WR : CTRL_OP_RD) | CTRL_WIDTH_16;
	deb("buf[0]=%02X\n", buf[0]);
	deb("buf[1]=%02X\n", buf[1]);
	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

static int zl38005_wr_addr(struct device *dev, u16 val)
{
	struct spi_message msg;
	u8 buf[3];
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = buf,
	};
	int err;
	u8 cmd = (CMD_VALID | CMD_WRITE | CMD_LEN_16_BIT | CMD_TYPE_ADDR_WORD);
	struct zl38005 *zl38005 = dev_get_drvdata(dev);

	spi_message_init(&msg);
	buf[0] = cmd;
	buf[1] = ((val >> 8) & 0xff);
	buf[2] = (val & 0xff);
	deb("buf[0]=%02X\n", buf[0]);
	deb("buf[1]=%02X\n", buf[1]);
	deb("buf[2]=%02X\n", buf[2]);
	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

static int zl38005_wr_data(struct device *dev, u16 val)
{
	struct spi_message msg;
	u8 buf[5];
	struct spi_transfer xfer = {
		.len = 5,
		.tx_buf = buf,
	};
	int err;
	u8 cmd = (CMD_VALID | CMD_WRITE | CMD_LEN_32_BIT | CMD_TYPE_WR_DATA);
	struct zl38005 *zl38005 = dev_get_drvdata(dev);

	spi_message_init(&msg);
	buf[0] = cmd;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = ((val >> 8) & 0xff);
	buf[4] = (val & 0xff);
	deb("buf[0]=%02X\n", buf[0]);
	deb("buf[1]=%02X\n", buf[1]);
	deb("buf[2]=%02X\n", buf[2]);
	deb("buf[3]=%02X\n", buf[3]);
	deb("buf[4]=%02X\n", buf[4]);
	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

static int zl38005_rd_data(struct device *dev, char *pval)
{
	int err;
	u8 cmd = (CMD_VALID | CMD_READ | CMD_LEN_32_BIT | CMD_TYPE_RD_DATA);
	u8 rx_buf[5];
	struct zl38005 *zl38005 = dev_get_drvdata(dev);

	deb("tx cmd=%X\n", cmd);
	err = spi_write_then_read(zl38005->spi, &cmd, sizeof cmd,
                        rx_buf, sizeof rx_buf);
	if (err < 0) {
		dev_err(dev, "spi_write_then_read failed with %d\n", err);
		return err;
	}
	deb("rx_buf[0]=%X\n", rx_buf[0]);
	deb("rx_buf[1]=%X\n", rx_buf[1]);
	deb("rx_buf[2]=%X\n", rx_buf[2]);
	deb("rx_buf[3]=%X\n", rx_buf[3]);
	deb("rx_buf[4]=%X\n", rx_buf[4]);
	memcpy(pval, &rx_buf[3], 2);
	return 0;
}

static int zl38005_wr_reg(struct device *dev, u16 addr,
			u16 val)
{
	int t = 0;
	u8 ctrl_val = CTRL_PENDING;
	u8 v;

	/* See if there is a pending operation */
	while (t++ < COUNT) {
		if (!(zl38005_rd_ctrl_reg(dev, &ctrl_val))) {
			v = ctrl_val & CTRL_PENDING;
			if (v == 0)
				break;
		}
		schedule();
	}
	if (t == COUNT) {
		printk(KERN_ERR "zl38005: TIMEOUT for pending operations\n");
		return -1;
	}

	/* Write Address */
	if (zl38005_wr_addr(dev, addr))
		return -ENODEV;

	/* Write Data */
	if (zl38005_wr_data(dev, val))
		return -ENODEV;

	/* Writing Ctrl reg */
	if (zl38005_wr_ctrl_reg(dev, CTRL_PENDING, 1))
		return -ENODEV;

	return 0;
}

static int zl38005_rd_reg(struct device *dev, u16 addr,
			u8 *pval)
{
	int t = 0;
	u8 ctrl_val = CTRL_PENDING;
	u8 v;

	/* See if there is a pending operation */
	while (t++ < COUNT) {
		if (!(zl38005_rd_ctrl_reg(dev, &ctrl_val))) {
			v = ctrl_val & CTRL_PENDING;
			if (v == 0)
				break;
		}
		schedule();
	}
	if (t == COUNT)
		return -1;

	/* Write Address */
	if (zl38005_wr_addr(dev, addr))
		return -ENODEV;

	/* Writing Ctrl reg */
	if (zl38005_wr_ctrl_reg(dev, CTRL_PENDING, 0))
		return -ENODEV;

	/* See if there is a pending operation */
	while (t++ < COUNT) {
		if (!(zl38005_rd_ctrl_reg(dev, &ctrl_val))) {
			v = ctrl_val & CTRL_PENDING;
			if (v == 0)
				break;
		}
		schedule();
	}
	if (t == COUNT)
		return -1;

	/* Read Data */
	if (zl38005_rd_data(dev, pval))
		return -ENODEV;

	return 0;
}

static int zl38005_spi_port_test(struct device *dev)
{
	int t = 0;
	u8 buf[2];

	zl38005_wr_reg(dev, REG_0403, REG_0403_ACK);
	while (t++ < COUNT) {
		zl38005_wr_reg(dev, REG_0402, TX_START);
		if (!(zl38005_rd_reg(dev, REG_0403, buf)) && (buf[0] == RX_START_OK))
			break;
	}

	if (t < COUNT)
		return 0;
	else
		return -1;
}

static int zl38005_open(struct inode *inode, struct file *filp)
{
	if (module_usage_count) {
		printk(KERN_ERR "zl38005 device alrady opened\n");
		return -EBUSY;
	}
	module_usage_count++;
	return 0;
}

static int zl38005_close(struct inode *inode, struct file *filp)
{
	if (module_usage_count) {
		module_usage_count--;
	}
	return 0;
}

static int zl38005_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	struct zl38005		*zl38005 = &zl38005_data;
	int ret;
	u8 buf[2];

	switch (cmd) {
	case ZL_WR_REG:
		if (copy_from_user(&zl38005_ioctl_par, (struct zl38005_ioctl_par *)arg, sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		deb("ioctl wr ADDR=%X\n", zl38005_ioctl_par.addr);
		deb("ioctl wr DATA=%X\n", zl38005_ioctl_par.data);
		ret = zl38005_wr_reg(&zl38005->spi->dev, zl38005_ioctl_par.addr, zl38005_ioctl_par.data);
	break;
	case ZL_RD_REG:
		if (copy_from_user(&zl38005_ioctl_par, (struct zl38005_ioctl_par *)arg, sizeof(struct zl38005_ioctl_par))) {
			return -EFAULT;
		}
		deb("ioctl wr ADDR=%X\n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, (u16)zl38005_ioctl_par.addr, buf);
		zl38005_ioctl_par.data = (buf[0] << 8) | buf[1];
		if (!ret) {
			deb("ioctl rd DATA=%X\n", zl38005_ioctl_par.data);
			if (copy_to_user((struct zl38005_ioctl_par *)arg, &zl38005_ioctl_par, sizeof(struct zl38005_ioctl_par)))
                                return -EFAULT;
		} else
			return -EAGAIN;
	break;
	default:
		printk(KERN_DEBUG "ioctl: Invalid Command Value");
		ret = -EINVAL;
	}
	return ret;
}

static const struct file_operations zl38005_fops = {
	.owner =	THIS_MODULE,
	.open =		zl38005_open,
	.release =	zl38005_close,
	.ioctl =	zl38005_ioctl,
};

/*-------------------------------------------------------------------------*/

static struct class *zl38005_class;

static int zl38005_spi_probe(struct spi_device *spi)
{
	struct zl38005 *zl38005 = &zl38005_data;
	int err;

	dev_dbg(&spi->dev, "probing zl38005 spi device\n");

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		return err;

	/* Initialize the driver data */
	zl38005->spi = spi;
	spi_set_drvdata(spi, zl38005);

#if 0
	/* Trying to see if zl38005 is live */
	if (zl38005_spi_port_test(&spi->dev)) {
		dev_err(&spi->dev, "ZL38005 spi port test NOT passed\n");
	}

	dev_info(&spi->dev, "ZL38005 spi port test passed\n");
#endif
	return 0;
}

static int zl38005_spi_remove(struct spi_device *spi)
{
	device_destroy(zl38005_class, zl38005_data.dev);
	class_destroy(zl38005_class);
	kfree(spi_get_drvdata(spi));
	return 0;
}

static struct spi_driver zl38005_spi = {
	.driver = {
		.name = "zl38005",
		.owner = THIS_MODULE,
	},
	.probe = zl38005_spi_probe,
	.remove = zl38005_spi_remove,
};

static int __init zl38005_init(void)
{
	struct zl38005 *zl38005 = &zl38005_data;
	int result;

	result = alloc_chrdev_region(&zl38005->dev, 0, 1, "zl38005");
	if (result < 0) {
		printk(KERN_ERR "\nzl38005: Module intialization failed.\
                could not register character device");
                return -ENODEV;
        }

	/* Initialize of character device */
	cdev_init(&c_dev, &zl38005_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &zl38005_fops;

	/* addding character device */
	result = cdev_add(&c_dev, zl38005->dev, 1);
	if (result) {
		printk("NOTICE \nzl38005:Error %d adding zl38005\
				..error no:", result);
		unregister_chrdev_region(zl38005->dev, 1);
	}

	/* registration of character device */
	register_chrdev(MAJOR(zl38005->dev), DRIVER_NAME, &zl38005_fops);

	zl38005_class = class_create(THIS_MODULE, "zl38005");

	if (!zl38005_class) {
		cdev_del(&c_dev);
		unregister_chrdev(MAJOR(zl38005->dev), DRIVER_NAME);
	}

	device_create(zl38005_class, NULL, zl38005->dev, NULL, "zl38005");

	result = spi_register_driver(&zl38005_spi);
	if (result < 0) {
		class_destroy(zl38005_class);
		unregister_chrdev(MAJOR(zl38005->dev), zl38005_spi.driver.name);
	}
	return result;

}

module_init(zl38005_init);

static void __exit zl38005_exit(void)
{
	spi_unregister_driver(&zl38005_spi);
}
module_exit(zl38005_exit);

/* Module information */
MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_DESCRIPTION("ZL38005 acoustic echo canceller");
MODULE_LICENSE("GPLv2");
MODULE_VERSION(DRIVER_VERSION);

