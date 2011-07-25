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

/* Includes ------------------------------------------------------------------*/

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

#ifdef DEBUG

#define ZL38005 "zl38005:"
#define pr_debug_zl1(a, args...) \
printk(KERN_INFO ZL38005 "%s %d: "s, __func__, __LINE__, ##args);

#if DEBUG > 1
#define pr_debug_zl2(a, args...) \
printk(KERN_INFO ZL38005 "%s %d: "s, __func__, __LINE__, ##args);
#else
#define pr_debug_zl2(a, args...)
#endif

#else
#define pr_debug_zl1(a, args...)
#define pr_debug_zl2(a, args...)
#endif

MODULE_DESCRIPTION("zl38005 codec driver");
MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_LICENSE("GPL");

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
#define ZL_RD_DATA_REG		_IOR(ZL_MAGIC, 0x09, struct zl38005_ioctl_par)
#define ZL_WR_DATA_REG		_IOR(ZL_MAGIC, 0x0a, struct zl38005_ioctl_par)
#define ZL_RD_INSTR_REG		_IOR(ZL_MAGIC, 0x0b, struct zl38005_ioctl_par)
#define ZL_WR_INSTR_REG		_IOR(ZL_MAGIC, 0x0c, struct zl38005_ioctl_par)

static struct zl38005_ioctl_par zl38005_ioctl_par;

void zl_delay(void){
	udelay(125);
}

/* Cmd : Read 16bit Controll Register */
static int zl38005_rd_ctrl_reg(struct device *dev)
{
	int err;
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	u8 cmd;
	u8 rx_buf[3];           /* dummy,Byte_HI,Byte_LO */
	u16 rd_value = -1 ;
	int t = 0;

	cmd = CMD_RD_CTRL_BYTE ;

	/* See if there is a pending operation */
	while ((t < COUNT) && ((rd_value & CTRL_PENDING) == 0)) {
		err = spi_write_then_read(zl38005->spi, &cmd, sizeof cmd, \
					rx_buf, sizeof rx_buf);
		if (err < 0) {
			dev_err(dev, "spi_sync failed with %d\n", err);
			return err;
		}
		rd_value  = (rx_buf[1]<<8) | rx_buf[2] ; /* Byte_HI, Byte_LO */
		pr_debug_zl1("TX cmd = %02X\n", cmd);
		pr_debug_zl1("RX rd_value = %04X \n", rd_value);

		/* suspend and after wait a little bit */
		schedule();
		zl_delay();
		t++;
	}
	if (t == COUNT) {
		printk(KERN_ERR "zl38005: TIMEOUT for pending operations\n");
		return -1;
	}
	return 0;
}

/* Cmd : Write 16bit Controll Register */
static int zl38005_wr_ctrl_reg(struct device *dev, u16 Ctlr_Byte)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	struct spi_message msg;
	u8 buf[3];
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = buf,
	};
	int err;

	spi_message_init(&msg);
	buf[0] = CMD_WR_CTRL_BYTE ;
	buf[1] = (Ctlr_Byte >> 8) & 0xFF ;
	buf[2] = Ctlr_Byte & 0xFF ;

	pr_debug_zl1("TX cmd       = %02X \n", buf[0]);
	pr_debug_zl1("TX ctrl_byte = %02X %02X \n", buf[1], buf[2]);

	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

/* Cmd : Write 16bit address */
static int zl38005_wr_addr(struct device *dev, u16 adr)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	struct spi_message msg;
	u8 buf[3];
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = buf,
	};
	int err;

	spi_message_init(&msg);
	buf[0] = CMD_WR_ADR;
	buf[1] = ((adr >> 8) & 0xff);
	buf[2] = (adr & 0xff);
	pr_debug_zl1("TX cmd = %02X\n", buf[0]);
	pr_debug_zl1("TX adr = %02X %02X \n", buf[1], buf[2]);

	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

/* Read 32bit data */
static int zl38005_rd_data(struct device *dev, int *pval)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	int err;
	u8 cmd;
	u8 rx_buf[5]; /* Buf Rx , 1byte dummy + 4byte data */

	cmd = CMD_RD_DATA_32 ;
	err = spi_write_then_read(zl38005->spi, &cmd, sizeof cmd, rx_buf, \
				sizeof rx_buf);
	if (err < 0) {
		dev_err(dev, "spi_write_then_read failed with %d\n", err);
		return err;
	}

	pr_debug_zl1("TX cmd    = %02X \n", cmd);
	pr_debug_zl1("RX valore = %02X %02X %02X %02X \n", rx_buf[1], \
			rx_buf[2], rx_buf[3], rx_buf[4]);
	*pval = (rx_buf[1] << 24) | (rx_buf[2] << 16) | (rx_buf[3] << 8) \
		| rx_buf[4];

	pr_debug_zl2("LEGGO   = %08X \n", *pval);

	return 0;
}

/* Write 32bit data */
static int zl38005_wr_data(struct device *dev, u32 val)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	struct spi_message msg;
	u8 buf[5];
	struct spi_transfer xfer = {
		.len = 5,
		.tx_buf = buf,
	};
	int err;

	spi_message_init(&msg);

	buf[0] = CMD_WR_DATA_32 ;
	buf[1] = ((val >> 24) & 0xff);	/* byte Hi */
	buf[2] = ((val >> 16) & 0xff);
	buf[3] = ((val >> 8)  & 0xff);
	buf[4] = (val & 0xff);	/* byte Lo */

	pr_debug_zl1("TX cmd    = %02X \n", buf[0]);
	pr_debug_zl1("TX valore = %02X %02X %02X %02X \n", buf[1], buf[2], \
					buf[3], buf[4]);
	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

/*
 *
 * High Level Funcion
 *
 */

/* Read Reg from Instruction Ram (IMEM) or Data Ram (DMEM) */
static int zl38005_rd_reg(struct device *dev, u16 addr, int *pval, u8 mem)
{
	u16 CtrlByte = 0 ;

	CtrlByte = CTLR_RD_DMEM ;
	if (mem)
		CtrlByte = CTLR_RD_IMEM ;

	/* See if there is a pending operation */
	if (zl38005_rd_ctrl_reg(dev))
		return -ENODEV;

	/* Write Address */
	if (zl38005_wr_addr(dev, addr))
		return -ENODEV;

	/* Writing Ctrl reg */
	if (zl38005_wr_ctrl_reg(dev, CtrlByte))
		return -ENODEV;

	/* See if there is a pending operation */
	if (zl38005_rd_ctrl_reg(dev))
		return -ENODEV;

	/* Read Data */
	if (zl38005_rd_data(dev, pval))
		return -ENODEV;

	return 0;
}

int zl38005_read(u16 addr, u16 *val)
{
	struct zl38005          *zl38005 = &zl38005_data;
	pr_debug_zl2("addr=%04X\n", addr);
	if (zl38005_rd_reg(&zl38005->spi->dev, addr, (int *)val, DMEM))
		return -ENODEV;
	pr_debug_zl2("val=%04X\n", *val);
	return 0;
}
EXPORT_SYMBOL(zl38005_read);

/* Write Reg in Instruction Ram (IMEM) or Data Ram (DMEM) */
static int zl38005_wr_reg(struct device *dev, u16 addr, u32 val, u8 mem)
{
	u16 CtrlByte = 0 ;

	CtrlByte = CTLR_WR_DMEM ;
	if (mem)
		CtrlByte = CTLR_WR_IMEM ;

	/* See if there is a pending operation */
	if (zl38005_rd_ctrl_reg(dev))
		return -ENODEV;

	/* Write Address */
	if (zl38005_wr_addr(dev, addr))
		return -ENODEV;

	/* Write Data */
	if (zl38005_wr_data(dev, val))
		return -ENODEV;

	/* Writing Ctrl reg */
	if (zl38005_wr_ctrl_reg(dev, CtrlByte))
		return -ENODEV;

	return 0;
}

int zl38005_write(u16 addr, u16 val)
{
	struct zl38005		*zl38005 = &zl38005_data;

	return zl38005_wr_reg(&zl38005->spi->dev, addr, val, DMEM);
}
EXPORT_SYMBOL(zl38005_write);

/*
 *
 * Driver Funcion
 *
 */
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

	switch (cmd) {
	case ZL_WR_DATA_REG:
		if (copy_from_user(&zl38005_ioctl_par, \
			(struct zl38005_ioctl_par *)arg,\
			 sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		pr_debug_zl1("ioctl wr DATA = %X \n", zl38005_ioctl_par.data);
		ret = zl38005_wr_reg(&zl38005->spi->dev, \
			zl38005_ioctl_par.addr, zl38005_ioctl_par.data, DMEM);
	break;
	case ZL_RD_DATA_REG:
		if (copy_from_user(&zl38005_ioctl_par, \
			(struct zl38005_ioctl_par *)arg,\
			 sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, \
		 (u16)zl38005_ioctl_par.addr, &zl38005_ioctl_par.data, DMEM);
		if (!ret) {
			pr_debug_zl1("ioctl rd DATA = %X \n ", \
				zl38005_ioctl_par.data);
			if (copy_to_user((struct zl38005_ioctl_par *)arg,\
			 &zl38005_ioctl_par, sizeof(struct zl38005_ioctl_par)))
				return -EFAULT;
		} else
			return -EAGAIN;
	break;
	case ZL_WR_INSTR_REG:
		if (copy_from_user(&zl38005_ioctl_par, (struct zl38005_ioctl_par *)arg, sizeof(struct zl38005_ioctl_par)))
		return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		pr_debug_zl1("ioctl wr DATA = %X \n", zl38005_ioctl_par.data);
		ret = zl38005_wr_reg(&zl38005->spi->dev, zl38005_ioctl_par.addr, zl38005_ioctl_par.data, IMEM);
	break;
	case ZL_RD_INSTR_REG:
		if (copy_from_user(&zl38005_ioctl_par, (struct zl38005_ioctl_par *)arg, sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, (u16)zl38005_ioctl_par.addr,  &zl38005_ioctl_par.data, IMEM);
		if (!ret) {
			pr_debug_zl1("ioctl rd DATA=%X\n", zl38005_ioctl_par.data);
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

