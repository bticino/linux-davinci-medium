/*
 * TI DaVinci DM646x board specific setup
 *
 * Author: Nageswari Srinivasan
 * This file adds support for CDCE949, on board PLL on DM6467 EVM
 * CDCE949 can output 5 different clock frequecies depending upon
 * the configuration. In DM6467 EVM, CDCE949 output is used by
 * VPIF display, Audio (McASP0, McASP1) and TSIF
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <mach/dm646x.h>
#include <mach/cputype.h>
#include <mach/clock.h>
#include "clock.h"
#include "mux.h"

struct i2c_client *cdce_client;
static DEFINE_MUTEX(cdce_mutex);

/* Structure to hold the register values for different frequencies */

struct cdce_cfg_val {
	unsigned char reg_val[5];
};

static const struct cdce_cfg_val freq_13500 = {
	.reg_val	= {8, 0x00, 0x04, 0x02, 0x08},
};

static const struct cdce_cfg_val freq_16875 = {
	.reg_val	= {8, 0x9f, 0xB0, 0x02, 0x89},
};


static const struct cdce_cfg_val freq_27000 = {
	.reg_val	= {4, 0x00, 0x40, 0x02, 0x08},
};


static const struct cdce_cfg_val freq_54000 = {
	.reg_val	= {4, 0xFF, 0x80, 0x02, 0x07},
};

static const struct cdce_cfg_val freq_81000 = {
	.reg_val	= {2, 0xBF, 0xA0, 0x03, 0x0A},
};
static const struct cdce_cfg_val freq_74250 = {
	.reg_val	= {2, 0xBF, 0xA0, 0x03, 0x0A},
};

static const struct cdce_cfg_val freq_148000 = {
	.reg_val	= {2, 0xBF, 0xA0, 0x03, 0x0A},
};


struct cdce_freq_table {
	struct cdce_cfg_val	*cfg_data;
	unsigned long frequency;
};

#define TABLE(freq)	\
{			\
	.cfg_data	= (struct cdce_cfg_val *) &freq_##freq,	\
	.frequency	= freq,	\
}

static struct cdce_freq_table freq_table[] =  {
TABLE(13500),
TABLE(16875),
TABLE(27000),
TABLE(54000),
TABLE(81000),
TABLE(74250),
TABLE(148000),
};


/*
 * Device specific clocks
 */
#define CDCE949_REF_CLK		27000000


/*	depending on the name of the clock,
 *	call the corresponding set of registers
 *	of CDCE 949 for setting the required clock frequency
 */


static int cdce_set_rate(struct clk *clk, unsigned long rate)
{
	int array_size, i;
	int err = 0;
	struct cdce_cfg_val *freq_val = NULL;
	struct i2c_client *client;

	client = cdce_client;

/* Find the index value for the frequency divider and
 *  multiplier from the Table */

	array_size	= ARRAY_SIZE(freq_table);
	rate = rate/1000;

	for (i = 0; i < array_size; i++) {
		if (freq_table[i].frequency == rate) {
			freq_val = freq_table[i].cfg_data;
			break;
		}
	}
/* Check whether the cline adapter is installed */

	if (IS_ERR(cdce_client))
		return -ENODEV;

	if (freq_val == NULL)
		return -EINVAL;

	mutex_lock(&cdce_mutex);
	if (!(strcmp(clk->name, "cdce_vpif"))) {
		if (rate == 148000) {
			err |= i2c_smbus_write_byte_data(client,
						0x13 | 0x80, 0x00);
/* Set PLL1_0 Multiplier */
			err |= i2c_smbus_write_byte_data(client,
						0x18 | 0x80, 0xaf);
			err |= i2c_smbus_write_byte_data(client,
						0x19 | 0x80, 0x50);
			err |= i2c_smbus_write_byte_data(client,
						0x1a | 0x80, 0x02);
			err |= i2c_smbus_write_byte_data(client,
						0x1b | 0x80, 0xc9);
/* Set PLL1_11 Multiplier */
			err |= i2c_smbus_write_byte_data(client,
					0x1c | 0x80, 0x00);
			err |= i2c_smbus_write_byte_data(client,
					0x1d | 0x80, 0x40);
			err |= i2c_smbus_write_byte_data(client,
					 0x1e | 0x80, 0x02);
			err |= i2c_smbus_write_byte_data(client,
					0x1f | 0x80, 0xc9);
/* Output state selection */
			err |= i2c_smbus_write_byte_data(client,
					0x15 | 0x80, 0x00);
			err |= i2c_smbus_write_byte_data(client,
					0x14 | 0x80, 0xef);
/* Switch MUX to PLL1 output */
			err |= i2c_smbus_write_byte_data(client,
					0x14 | 0x80, 0x6f);
			err |= i2c_smbus_write_byte_data(client,
					0x16 | 0x80, 0x06);
/* Set P2DIV Divider, P3DIV and Input crystal */
			err |= i2c_smbus_write_byte_data(client,
					0x17 | 0x80, 0x06);

			err |= i2c_smbus_write_byte_data(client,
					0x01 | 0x80, 0x00);
			err |= i2c_smbus_write_byte_data(client,
					0x05 | 0x80, (9 << 3));
			err |= i2c_smbus_write_byte_data(client,
					0x02 | 0x80, 0x80);
/* Enable and Disable PLL */
			err |= i2c_smbus_write_byte_data(client,
					0x02 | 0x80, 0xbc);
			err |= i2c_smbus_write_byte_data(client,
					0x03 | 0x80, 0x01);
	} else if (rate == 74250) {
		err |= i2c_smbus_write_byte_data(client,
				0x13 | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
				0x18 | 0x80, 0xaf);
		err |= i2c_smbus_write_byte_data(client,
					0x19 | 0x80, 0x50);
		err |= i2c_smbus_write_byte_data(client,
					0x1a | 0x80, 0x02);
		err |= i2c_smbus_write_byte_data(client,
					0x1b | 0x80, 0xc9);

		err |= i2c_smbus_write_byte_data(client,
					0x1c | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x1d | 0x80, 0x40);
		err |= i2c_smbus_write_byte_data(client,
					0x1e | 0x80, 0x02);
		err |= i2c_smbus_write_byte_data(client,
					0x1f | 0x80, 0xc9);

/* Output state selection */

		err |= i2c_smbus_write_byte_data(client,
					0x15 | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x14 | 0x80, 0xef);
		err |= i2c_smbus_write_byte_data(client,
					0x14 | 0x80, 0x6f);
		err |= i2c_smbus_write_byte_data(client,
					0x16 | 0x80, 0x06);
		err |= i2c_smbus_write_byte_data(client,
					0x17 | 0x80, 0x06);

		err |= i2c_smbus_write_byte_data(client,
					0x01 | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x05 | 0x80, ( 9 << 3 ));
		err |= i2c_smbus_write_byte_data(client,
					0x02 | 0x80, 0x80);
		err |= i2c_smbus_write_byte_data(client,
					0x02 | 0x80, 0xbc);
		err |= i2c_smbus_write_byte_data(client,
					0x03 | 0x80, 0x02);

	} else if (rate == 27000) {
		err |= i2c_smbus_write_byte_data(client,
					0x13 | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x18 | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x19 | 0x80, 0x40);
		err |= i2c_smbus_write_byte_data(client,
					0x1a | 0x80, 0x02);
		err |= i2c_smbus_write_byte_data(client,
					0x1b | 0x80, 0x08);

		err |= i2c_smbus_write_byte_data(client,
					0x1c | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x1d | 0x80, 0x40);
		err |= i2c_smbus_write_byte_data(client,
					0x1e | 0x80, 0x02);
		err |= i2c_smbus_write_byte_data(client,
					0x1f | 0x80, 0x08);

		err |= i2c_smbus_write_byte_data(client,
					0x15 | 0x80, 0x02);
		err |= i2c_smbus_write_byte_data(client,
					0x14 | 0x80, 0xed);
		err |= i2c_smbus_write_byte_data(client,
					0x16 | 0x80, 0x01);
		err |= i2c_smbus_write_byte_data(client,
					0x17 | 0x80, 0x01);

		err |= i2c_smbus_write_byte_data(client,
					0x01 | 0x80, 0x00);
		err |= i2c_smbus_write_byte_data(client,
					0x05 | 0x80, 0x50);
		err |= i2c_smbus_write_byte_data(client,
					0x02 | 0x80, 0xb4);
		err |= i2c_smbus_write_byte_data(client,
					0x03 | 0x80, 0x01);
	}
	mutex_unlock(&cdce_mutex);

	if (err)
		return -EINVAL;
	else
		clk->rate = (freq_table[i].frequency * 1000);

	}


	else if (strcmp(clk->name, "cdce_audio"))	{

/*	TO DO - Need to add corresponding functions for
 *	McASP1 clock settings on EVM
 */
	}
/*	TO DO - Need to add corresponding functions for
 *	McASP1 clock settings on EVM
 */
	else if (strcmp(clk->name, "cdce_mcasp1")) {
		return 0;
	}
/*	Clock setting for TSIF.
 *	Currently TSIF0 and TSIF1 will be using the same frequency for
 *	TSIF_OUT if they select the same clock source as external
 */

	else if (strcmp(clk->name, "cdce_crgo_vc1")) {

	}
/* Update the clock rate to take new clock value */

	clk->rate = freq_table[i].frequency * 1000;
	return 0;
}

static int cdce949_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	cdce_client = client;
	return 0;

}

static int cdce949_remove(struct i2c_client *client)
{
	client = NULL;
	return 0;
}
static const struct i2c_device_id cdce949_id[] = {
	{"cdce949", 0},
	{},
};




static struct i2c_driver cdce949_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "cdce949",
	},
	.probe		= cdce949_probe,
	.id_table	= cdce949_id,
	.remove		= cdce949_remove,
};





static struct clk ref_clk = {
	.name = "cdce_ref_clk",
	.rate = CDCE949_REF_CLK,
};


static struct clk cdce_vpif_clk = {
	.name = "cdce_vpif",
	.parent = &ref_clk,
	.rate = CDCE949_REF_CLK,
	.set_rate = cdce_set_rate,
};

static struct clk cdce_audio_clk = {
	.name = "cdce_audio",
	.parent = &ref_clk,
	.rate = CDCE949_REF_CLK,
	.set_rate = cdce_set_rate,
};

static struct clk cdce_mcasp1_clk = {
	.name = "cdce_mcasp1",
	.parent = &ref_clk,
	.rate = CDCE949_REF_CLK,
	.set_rate = cdce_set_rate,
};

static struct clk cdce_crg0_vc1_clk = {
	.name = "cdce_crg0_vc1",
	.parent = &ref_clk,
	.rate = CDCE949_REF_CLK,
	.set_rate = cdce_set_rate,
};

struct davinci_clk cdce_clks[] = {
	CLK(NULL, "cdce_ref_clk", &ref_clk),
	CLK(NULL, "cdce_vpif", &cdce_vpif_clk),
	CLK(NULL, "cdce_audio", &cdce_audio_clk),
	CLK(NULL, "cdce_mcasp1", &cdce_mcasp1_clk),
	CLK(NULL, "cdce_crg0_vc1", &cdce_crg0_vc1_clk),
	CLK(NULL, NULL, NULL),
};

static int __init cdce949_init(void)
{
	struct davinci_clk *c;
	struct clk *clk;

	for (c = cdce_clks; c->lk.clk; c++) {
		clk = c->lk.clk;
		clkdev_add(&c->lk);
		clk_register(clk);
	}

	return i2c_add_driver(&cdce949_driver);
}

static void __exit cdce949_exit(void)
{
	i2c_del_driver(&cdce949_driver);
}


module_init(cdce949_init);
module_exit(cdce949_exit);
