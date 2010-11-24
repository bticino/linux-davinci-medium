/*
 * Driver for the TDA9885 chip
 *
 * Copyright (c) 2010 Rodolfo Giometti <giometti@linux.it>
 * Copyright (c) 2010 cino S.p.A. <info@bticino.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/i2c/tda9885.h>

#define DRIVER_VERSION	"1.0.0"

/*
 * Initialization function
 */

static int tda9885_init_client(struct i2c_client *client,
			struct tda9885_platform_data *pdata)
{
	u8 buf[] = {
		0, pdata->switching_mode, pdata->adjust_mode, pdata->data_mode,
	};
	int ret;

	/*
	 * This chip is very simple, just write first the base address
	 * and then all registers settings.
	 */
	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		return ret;
	if (ret != ARRAY_SIZE(buf))
		return -EIO;

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static int __devinit tda9885_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tda9885_platform_data *pdata = client->dev.platform_data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	/* No private data for this driver */
	i2c_set_clientdata(client, NULL);

	/* Check platform data */
	if (!pdata)
		dev_warn(&client->dev, "no default init data");
	else {
		dev_info(&client->dev, "default switching mode is %02x\n",
				pdata->switching_mode);
		dev_info(&client->dev, "default adjust mode is %02x\n",
				pdata->adjust_mode);
		dev_info(&client->dev, "default data mode is %02x\n",
				pdata->data_mode);

		/* Initialize the TDA9885 chip */
		err = tda9885_init_client(client, pdata);
		if (err)
			goto exit;
	}

	/*
	 * This driver doesn't need any settings once it's initialized,
	 * so no sysfs hooks are needed to manage it...
	 */
	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

exit:
	return err;
}

static int __devexit tda9885_remove(struct i2c_client *client)
{
	/* Nop! =:-o */

	return 0;
}

static const struct i2c_device_id tda9885_id[] = {
	{ "tda9885", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tda9885_id);

static struct i2c_driver tda9885_driver = {
	.driver = {
		.name   = "tda9885",
		.owner  = THIS_MODULE,
	},
	.probe = tda9885_probe,
	.remove = __devexit_p(tda9885_remove),
	.id_table = tda9885_id,
};

static int __init tda9885_init(void)
{
	return i2c_add_driver(&tda9885_driver);
}

static void __exit tda9885_exit(void)
{
	i2c_del_driver(&tda9885_driver);
}

/* Module information */
MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("TDA9885 IF-PPL demodulator driver");
MODULE_LICENSE("GPLv2");
MODULE_VERSION(DRIVER_VERSION);

module_init(tda9885_init);
module_exit(tda9885_exit);
