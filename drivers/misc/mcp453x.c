/*
 * mcp453x.c - mcp453x Potentiometer driver
 *
 * Copyright (C) 2012 Bticino S.p.A.
 *
 * Derived from
 * apds9802als.c - apds9802  ALS Driver
 *
 * Copyright (C) 2009 Intel Corp
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/mcp453x.h>

#define DRIVER_NAME "mcp453x"

#define VOLATILE_WIPER0	0x00
#define VOLATILE_TCON	0x04
#define STATUS		0x05

#define MCP_WRITE	0x0
#define MCP_READ	(0x3 << 2)

#define VAL_H_MASK	0x1

/* data are 9 bits with one bit in the address register */
#define WRITE_CONV_H(r, t) \
	((r << 4) | MCP_WRITE | ((t >> 8) & VAL_H_MASK))
#define WRITE_CONV_L(t) (t & 0x00ff)
#define READ_ADDR_CONV(t) \
	((t << 4) | MCP_READ)
/* swapping bytes */
#define READ_CONV(t) \
	(((t & VAL_H_MASK) << 8) | ((t & 0xFF00) >> 8))

struct mcp_data {
	struct mutex mutex;
	struct mcp453x_platform_data *pdata;
};

static ssize_t volatile_wiper0_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp_data *data = i2c_get_clientdata(client);
	int ret_val;

	mutex_lock(&data->mutex);
	ret_val = i2c_smbus_read_word_data(client,
					   READ_ADDR_CONV(VOLATILE_WIPER0));
	if (ret_val < 0)
		goto failed;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "0x%x\n", READ_CONV(ret_val));
failed:
	mutex_unlock(&data->mutex);
	return ret_val;
}

static ssize_t volatile_wiper0_store(struct device *dev,
		struct device_attribute *attr, const  char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp_data *data = i2c_get_clientdata(client);
	int ret_val;
	unsigned long val;
	u8 reg_val_h, val_l;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&data->mutex);
	reg_val_h = WRITE_CONV_H(VOLATILE_WIPER0, val);
	val_l = WRITE_CONV_L(val);
	ret_val = i2c_smbus_write_byte_data(client, reg_val_h, val_l);
	if (ret_val >= 0) {
		mutex_unlock(&data->mutex);
		return count;
	}
	mutex_unlock(&data->mutex);

	return ret_val;
}

static ssize_t volatile_tcon_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp_data *data = i2c_get_clientdata(client);
	int ret_val;

	mutex_lock(&data->mutex);
	ret_val = i2c_smbus_read_word_data(client,
					   READ_ADDR_CONV(VOLATILE_TCON));
	if (ret_val < 0)
		goto failed;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "0x%x\n", READ_CONV(ret_val));
failed:
	mutex_unlock(&data->mutex);
	return ret_val;
}

static ssize_t volatile_tcon_store(struct device *dev,
		struct device_attribute *attr, const  char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp_data *data = i2c_get_clientdata(client);
	int ret_val;
	unsigned long val;
	u8 reg_val_h, val_l;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&data->mutex);
	reg_val_h = WRITE_CONV_H(VOLATILE_TCON, val);
	val_l = WRITE_CONV_L(val);
	ret_val = i2c_smbus_write_byte_data(client, reg_val_h, val_l);
	if (ret_val >= 0) {
		mutex_unlock(&data->mutex);
		return count;
	}
	mutex_unlock(&data->mutex);

	return ret_val;
}

static ssize_t status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mcp_data *data = i2c_get_clientdata(client);
	int ret_val;

	mutex_lock(&data->mutex);
	ret_val = i2c_smbus_read_word_data(client, READ_ADDR_CONV(STATUS));
	if (ret_val < 0)
		goto failed;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "0x%x\n", READ_CONV(ret_val));

failed:
	mutex_unlock(&data->mutex);
	return ret_val;
}

static DEVICE_ATTR(volatile_wiper0, S_IRUGO | S_IWUSR,
	volatile_wiper0_show, volatile_wiper0_store);
static DEVICE_ATTR(volatile_tcon, S_IRUGO | S_IWUSR,
	volatile_tcon_show, volatile_tcon_store);
static DEVICE_ATTR(status, S_IRUGO,
	status_show, NULL);

static struct attribute *sysfs_attrs_mcp[] = {
	&dev_attr_volatile_wiper0.attr,
	&dev_attr_volatile_tcon.attr,
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group m_mcp_gr = {
	.name = "mcp453x",
	.attrs = sysfs_attrs_mcp,
};

static int mcp_set_default_config(struct i2c_client *client,
				  struct mcp453x_platform_data *pdata)
{
	int ret_val;
	u8 reg_val_h, val_l;

	dev_info(&client->dev, "Setting volatile_wiper0 to %x\n",
			      pdata->volatile_wiper0);
	reg_val_h = WRITE_CONV_H(VOLATILE_WIPER0, pdata->volatile_wiper0);
	val_l = WRITE_CONV_L(pdata->volatile_wiper0);
	ret_val = i2c_smbus_write_word_data(client, reg_val_h, val_l);
	if (ret_val < 0) {
		dev_err(&client->dev, "failing during write\n");
		return ret_val;
	}

	dev_info(&client->dev, "Setting volatile_tcon to %x\n",
			      pdata->volatile_tcon);
	reg_val_h = WRITE_CONV_H(VOLATILE_TCON, pdata->volatile_tcon);
	val_l = WRITE_CONV_L(pdata->volatile_tcon);
	ret_val = i2c_smbus_write_word_data(client, reg_val_h, val_l);
	if (ret_val < 0) {
		dev_err(&client->dev, "failing during write\n");
		return ret_val;
	}

	return ret_val;
}

static int __devinit mcp453x_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct device *dev;
	struct mcp_data *data;
	int res;

	dev = &client->dev;
	if (!client->dev.platform_data) {
		dev_err(dev, "Platform data not set\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct mcp_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	data->pdata = client->dev.platform_data;

	if (!data->pdata->volatile_wiper0 ||
	    !data->pdata->volatile_tcon) {
		dev_err(dev, "Platform data default values not set\n");
		return -ENODEV;
	}

	i2c_set_clientdata(client, data);

	res = sysfs_create_group(&client->dev.kobj, &m_mcp_gr);
	if (res) {
		dev_err(&client->dev, "device create file failed\n");
		goto mcp_error1;
	}
	dev_info(&client->dev, "MCP453x chip found\n");
	mcp_set_default_config(client, data->pdata);
	mutex_init(&data->mutex);

	return res;
mcp_error1:
	kfree(data);
	return res;
}

static int __devexit mcp453x_remove(struct i2c_client *client)
{
	struct mcp_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &m_mcp_gr);
	kfree(data);
	return 0;
}

#define mcp453x_suspend NULL
#define mcp453x_resume NULL
#define MCP453x_PM_OPS NULL

static struct i2c_device_id mcp453x_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mcp453x_id);

static struct i2c_driver mcp453x_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = mcp453x_probe,
	.remove = __devexit_p(mcp453x_remove),
	.suspend = mcp453x_suspend,
	.resume = mcp453x_resume,
	.id_table = mcp453x_id,
};

static int __init mcp453x_init(void)
{
	return i2c_add_driver(&mcp453x_driver);
}

static void  __exit mcp453x_exit(void)
{
	i2c_del_driver(&mcp453x_driver);
}
module_init(mcp453x_init);
module_exit(mcp453x_exit);

MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>,"
	      "Simone Cianni <simone.cianni@bticino.it>");
MODULE_DESCRIPTION("mcp453x potentiometer driver");
MODULE_LICENSE("GPL v2");
