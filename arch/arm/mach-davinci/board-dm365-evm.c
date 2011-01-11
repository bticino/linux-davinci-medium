/*
 * TI DaVinci DM365 EVM board support
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/tsc2004.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mmc.h>
#include <mach/mux.h>
#include <mach/nand.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <mach/cputype.h>
#include <linux/videodev2.h>
#include <media/tvp514x.h>
#include <media/tvp7002.h>
#include <media/davinci/videohd.h>



/* have_imager() - Check if we have support for imager interface */
static inline int have_imager(void)
{
#if defined(CONFIG_SOC_CAMERA_MT9P031) || \
    defined(CONFIG_SOC_CAMERA_MT9P031_MODULE)
	return 1;
#else
	return 0;
#endif
}

static inline int have_tvp7002(void)
{
#ifdef CONFIG_VIDEO_TVP7002
	return 1;
#else
	return 0;
#endif
}


#define DM365_ASYNC_EMIF_CONTROL_BASE	0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE	0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE	0x04000000

#define DM365_EVM_PHY_MASK		(0x2)
#define DM365_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

/*
 * A MAX-II CPLD is used for various board control functions.
 */
#define CPLD_OFFSET(a13a8,a2a1)		(((a13a8) << 10) + ((a2a1) << 3))

#define CPLD_VERSION	CPLD_OFFSET(0,0)	/* r/o */
#define CPLD_TEST	CPLD_OFFSET(0,1)
#define CPLD_LEDS	CPLD_OFFSET(0,2)
#define CPLD_MUX	CPLD_OFFSET(0,3)
#define CPLD_SWITCH	CPLD_OFFSET(1,0)	/* r/o */
#define CPLD_POWER	CPLD_OFFSET(1,1)
#define CPLD_VIDEO	CPLD_OFFSET(1,2)
#define CPLD_CARDSTAT	CPLD_OFFSET(1,3)	/* r/o */

#define CPLD_DILC_OUT	CPLD_OFFSET(2,0)
#define CPLD_DILC_IN	CPLD_OFFSET(2,1)	/* r/o */

#define CPLD_IMG_DIR0	CPLD_OFFSET(2,2)
#define CPLD_IMG_MUX0	CPLD_OFFSET(2,3)
#define CPLD_IMG_MUX1	CPLD_OFFSET(3,0)
#define CPLD_IMG_DIR1	CPLD_OFFSET(3,1)
#define CPLD_IMG_MUX2	CPLD_OFFSET(3,2)
#define CPLD_IMG_MUX3	CPLD_OFFSET(3,3)
#define CPLD_IMG_DIR2	CPLD_OFFSET(4,0)
#define CPLD_IMG_MUX4	CPLD_OFFSET(4,1)
#define CPLD_IMG_MUX5	CPLD_OFFSET(4,2)

#define CPLD_RESETS	CPLD_OFFSET(4,3)

#define CPLD_TS_CFG	CPLD_OFFSET(5,0)

#define CPLD_CCD_DIR1	CPLD_OFFSET(0x3e,0)
#define CPLD_CCD_IO1	CPLD_OFFSET(0x3e,1)
#define CPLD_CCD_DIR2	CPLD_OFFSET(0x3e,2)
#define CPLD_CCD_IO2	CPLD_OFFSET(0x3e,3)
#define CPLD_CCD_DIR3	CPLD_OFFSET(0x3f,0)
#define CPLD_CCD_IO3	CPLD_OFFSET(0x3f,1)

#define CPLD_VIDEO_INPUT_MUX_MASK	0x7
#define CPLD_VIDEO_INPUT_MUX_TVP7002	0x1
#define CPLD_VIDEO_INPUT_MUX_IMAGER	0x2
#define CPLD_VIDEO_INPUT_MUX_TVP5146	0x5

static void __iomem *cpld;

static struct tvp514x_platform_data tvp5146_pdata = {
       .clk_polarity = 0,
       .hs_polarity = 1,
       .vs_polarity = 1
};

/* tvp7002 platform data, used during reset and probe operations */
static struct tvp7002_platform_data tvp7002_pdata = {
       .clk_polarity = 0,
       .hs_polarity = 0,
       .vs_polarity = 0,
       .fid_polarity = 0,
};

/* NOTE:  this is geared for the standard config, with a socketed
 * 2 GByte Micron NAND (MT29F16G08FAA) using 128KB sectors.  If you
 * swap chips, maybe with a different block size, partitioning may
 * need to be changed.
 */
/*define NAND_BLOCK_SIZE		SZ_128K*/

/* For Samsung 4K NAND (K9KAG08U0M) with 256K sectors */
/*#define NAND_BLOCK_SIZE		SZ_256K*/

/* For Micron 4K NAND with 512K sectors */
#define NAND_BLOCK_SIZE		SZ_512K

static struct mtd_partition davinci_nand_partitions[] = {
	{
		/* UBL (a few copies) plus U-Boot */
		.name		= "bootloader",
		.offset		= 0,
		.size		= 30 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	}, {
		/* U-Boot environment */
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * NAND_BLOCK_SIZE,
		.mask_flags	= 0,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "filesystem1",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_512M,
		.mask_flags	= 0,
	}, {
		.name		= "filesystem2",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_chipsel		= BIT(14),
	.parts			= davinci_nand_partitions,
	.nr_parts		= ARRAY_SIZE(davinci_nand_partitions),
	.ecc_mode		= NAND_ECC_HW,
	.options		= NAND_USE_FLASH_BBT,
	.ecc_bits		= 4,
};

static struct resource davinci_nand_resources[] = {
	{
		.start		= DM365_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DM365_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= DM365_ASYNC_EMIF_CONTROL_BASE,
		.end		= DM365_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device davinci_nand_device = {
	.name			= "davinci_nand",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(davinci_nand_resources),
	.resource		= davinci_nand_resources,
	.dev			= {
		.platform_data	= &davinci_nand_data,
	},
};

static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = davinci_get_mac_addr,
	.context	= (void *)0x7f00,
};

static struct snd_platform_data dm365_evm_snd_data = {
	.eventq_no = EVENTQ_3,
};

/*
 * TSC 2004 Support
 */
#define TSC2004_GPIO_IRQ_PIN		GPIO_TO_PIN(0, 0)
#define DM365_EVM_TSC2004_ADDRESS	0x49

static int tsc2004_init_irq(void)
{
	int ret = 0;
	u8 val;

	ret = gpio_request(TSC2004_GPIO_IRQ_PIN, "tsc2004-irq");
	if (ret < 0) {
		pr_warning("%s: failed to TSC2004 IRQ GPIO: %d\n",
				__func__, ret);
		return ret;
	}

	gpio_direction_input(TSC2004_GPIO_IRQ_PIN);

	val = __raw_readb(cpld + CPLD_TS_CFG);
	val |= 0x2;
	__raw_writeb(val, cpld + CPLD_TS_CFG);

	return ret;
}

static void tsc2004_exit_irq(void)
{
	gpio_free(TSC2004_GPIO_IRQ_PIN);
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(TSC2004_GPIO_IRQ_PIN) ? 0 : 1;
}

struct tsc2004_platform_data dm365evm_tsc2004data = {
	.model = 2004,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2004_get_irq_level,
	.init_platform_hw = tsc2004_init_irq,
	.exit_platform_hw = tsc2004_exit_irq,
};

static struct i2c_board_info i2c_info[] = {
	{
		I2C_BOARD_INFO("dm365evm_keys", 0x25),
	},
	{
		I2C_BOARD_INFO("24c256", 0x50),
		.platform_data	= &eeprom_info,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("ths7303", 0x2c),
	},
	{
		I2C_BOARD_INFO("PCA9543A", 0x73),
	},
};

static struct i2c_board_info __initdata dm365_evm_tsc2004_dev = {
	I2C_BOARD_INFO("tsc2004", DM365_EVM_TSC2004_ADDRESS),
	.platform_data  = &dm365evm_tsc2004data,
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq	= 400	/* kHz */,
	.bus_delay	= 0	/* usec */,
};

static int dm365evm_keyscan_enable(struct device *dev)
{
	return davinci_cfg_reg(DM365_KEYSCAN);
}

static unsigned short dm365evm_keymap[] = {
	KEY_KP2,
	KEY_LEFT,
	KEY_EXIT,
	KEY_DOWN,
	KEY_ENTER,
	KEY_UP,
	KEY_KP1,
	KEY_RIGHT,
	KEY_MENU,
	KEY_RECORD,
	KEY_REWIND,
	KEY_KPMINUS,
	KEY_STOP,
	KEY_FASTFORWARD,
	KEY_KPPLUS,
	KEY_PLAYPAUSE,
	0
};

static struct davinci_ks_platform_data dm365evm_ks_data = {
	.device_enable	= dm365evm_keyscan_enable,
	.keymap		= dm365evm_keymap,
	.keymapsize	= ARRAY_SIZE(dm365evm_keymap),
	.rep		= 1,
	/* Scan period = strobe + interval */
	.strobe		= 0x5,
	.interval	= 0x2,
	.matrix_type	= DAVINCI_KEYSCAN_MATRIX_4X4,
};

static int cpld_mmc_get_ro(int module)
{
	if (!cpld)
		return -ENXIO;

	/* high == card's write protect switch active */
	return !!(__raw_readb(cpld + CPLD_CARDSTAT) & BIT(module ? 5 : 1));
}


static struct i2c_client *pca9543a;

static int pca9543a_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	printk("pca9543a_probe\n");
	pca9543a = client;
	return 0;
}

static int pca9543a_remove(struct i2c_client *client)
{
	pca9543a = NULL;
	return 0;
}

static const struct i2c_device_id pca9543a_ids[] = {
	{ "PCA9543A", 0, },
	{ /* end of list */ },
};

/* This is for i2c driver for the MT9P031 header i2c switch */
static struct i2c_driver pca9543a_driver = {
	.driver.name	= "PCA9543A",
	.id_table	= pca9543a_ids,
	.probe		= pca9543a_probe,
	.remove		= pca9543a_remove,
};

/**
 * dm365evm_reset_imager() - reset the image sensor
 * @en: enable/disable flag
 */
static void dm365evm_reset_imager(int rst)
{
	u8 val;
	
   val = __raw_readb(cpld + CPLD_POWER) | BIT(3) | BIT(11) | BIT(19) | BIT(27);
	__raw_writeb(val, (cpld + CPLD_POWER));

   val = __raw_readb(cpld + CPLD_MUX) | BIT(6) | BIT(14) | BIT(22) | BIT(30);
	__raw_writeb(val, (cpld + CPLD_MUX));

	/* Reset bit6 of CPLD_IMG_DIR2 */
	val = __raw_readb(cpld + CPLD_IMG_DIR2) & ~BIT(6);
	__raw_writeb(val, (cpld + CPLD_IMG_DIR2));	

	/* Set bit5 of CPLD_IMG_MUX5 */
	val = __raw_readb(cpld + CPLD_IMG_MUX5) | BIT(5);
	__raw_writeb(val, (cpld + CPLD_IMG_MUX5));	

	/* Reset bit 0 of CPLD_IMG_MUX5 */
	val = __raw_readb(cpld + CPLD_IMG_MUX5) & ~BIT(0);
	__raw_writeb(val, (cpld + CPLD_IMG_MUX5));	

	/**
	 * Configure GPIO40 to be output and high. This has dependency on MMC1
	 */
	davinci_cfg_reg(DM365_GPIO40);
	gpio_request(40, "sensor_reset");
	if (rst)
		gpio_direction_output(40, 1);
	else
		gpio_direction_output(40, 0);
}

/**
 * dm365evm_enable_pca9543a() - Enable/Disable I2C switch PCA9543A for sensor
 * @en: enable/disable flag
 */
static int dm365evm_enable_pca9543a(int en)
{
	static char val = 1;
	int status;
	struct i2c_msg msg = {
			.flags = 0,
			.len = 1,
			.buf = &val,
		};

	printk("dm365evm_enable_pca9543a\n");	
	if (!en)
		val = 0;

	if (!pca9543a)
		return -ENXIO;

	msg.addr = pca9543a->addr;
	/* turn i2c switch, pca9543a, on/off */
	status = i2c_transfer(pca9543a->adapter, &msg, 1);
	printk("dm365evm_enable_pca9543a, status = %d\n", status);	
	return status;
	return 0;
}

#define V4L2_STD_MT9P031_STD_ALL  (V4L2_STD_525_60\
	|V4L2_STD_625_50|V4L2_STD_525P_60\
	|V4L2_STD_625P_50|V4L2_STD_720P_30\
   |V4L2_STD_720P_50|V4L2_STD_720P_60\
   |V4L2_STD_1080I_30|V4L2_STD_1080I_50\
	|V4L2_STD_1080I_60|V4L2_STD_1080P_30\
   |V4L2_STD_1080P_50|V4L2_STD_1080P_60)

/* Input available at the mt9p031 */
static struct v4l2_input mt9p031_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
      .std = V4L2_STD_MT9P031_STD_ALL,
	}
};

#define TVP514X_STD_ALL        (V4L2_STD_NTSC | V4L2_STD_PAL)
/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
	{
		.index = 1,
		.name = "S-Video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP514X_STD_ALL,
	},
};

#define TVP7002_STD_ALL        (V4L2_STD_525P_60   | V4L2_STD_625P_50 	|\
				V4L2_STD_NTSC      | V4L2_STD_PAL   	|\
				V4L2_STD_720P_50   | V4L2_STD_720P_60 	|\
				V4L2_STD_1080I_50  | V4L2_STD_1080I_60 	|\
				V4L2_STD_1080P_50  | V4L2_STD_1080P_60)

/* Inputs available at the TVP7002 */
static struct v4l2_input tvp7002_inputs[] = {
	{
		.index = 0,
		.name = "Component",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = TVP7002_STD_ALL,
	},
};

/*
 * this is the route info for connecting each input to decoder
 * ouput that goes to vpfe. There is a one to one correspondence
 * with tvp5146_inputs
 */
static struct vpfe_route tvp5146_routes[] = {
	{
		.input = INPUT_CVBS_VI2B,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
{
		.input = INPUT_SVIDEO_VI2C_VI1C,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name = "tvp5146",
		.grp_id = VPFE_SUBDEV_TVP5146,
		.num_inputs = ARRAY_SIZE(tvp5146_inputs),
		.inputs = tvp5146_inputs,
		.routes = tvp5146_routes,
		.can_route = 1,
		.ccdc_if_params = {
			.if_type = VPFE_BT656,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5d),
			.platform_data = &tvp5146_pdata,
		},
	},
	{
		.module_name = "tvp7002",
		.grp_id = VPFE_SUBDEV_TVP7002,
		.num_inputs = ARRAY_SIZE(tvp7002_inputs),
		.inputs = tvp7002_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_BT1120,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("tvp7002", 0x5c),
			.platform_data = &tvp7002_pdata,
		},
	},
	{
		.module_name = "ths7353",
		.grp_id = VPFE_SUBDEV_TVP7002,
		.board_info = {
			I2C_BOARD_INFO("ths7353", 0x2e),
		},
	},
	{
		.module_name = "mt9p031",
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_MT9P031,
		.num_inputs = ARRAY_SIZE(mt9p031_inputs),
		.inputs = mt9p031_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_RAW_BAYER,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {
			I2C_BOARD_INFO("mt9p031", 0x5d),
			/* this is for PCLK rising edge */
			.platform_data = (void *)1,
		},
	}
};

/* Set the input mux for TVP7002/TVP5146/MTxxxx sensors */
static int dm365evm_setup_video_input(enum vpfe_subdev_id id)
{
	const char *label;
	u8 mux, resets;

	mux = __raw_readb(cpld + CPLD_MUX);
	mux &= ~CPLD_VIDEO_INPUT_MUX_MASK;
	resets = __raw_readb(cpld + CPLD_RESETS);
	switch (id) {
		case VPFE_SUBDEV_TVP5146:
			mux |= CPLD_VIDEO_INPUT_MUX_TVP5146;
			resets &= ~BIT(0);
			label = "tvp5146 SD";
			dm365evm_reset_imager(0);
			break;
		case VPFE_SUBDEV_MT9P031:
			mux |= CPLD_VIDEO_INPUT_MUX_IMAGER;
			resets |= BIT(0); /* Put TVP5146 in reset */
			label = "HD imager";

			dm365evm_reset_imager(1);
			/* Switch on pca9543a i2c switch */
			if (have_imager())
				dm365evm_enable_pca9543a(1);
			break;
		case VPFE_SUBDEV_TVP7002:
			resets &= ~BIT(2);
			mux |= CPLD_VIDEO_INPUT_MUX_TVP7002;
			label = "tvp7002 HD";
			break;
		default:
			return 0;
	}
	__raw_writeb(mux, cpld + CPLD_MUX);
	__raw_writeb(resets, cpld + CPLD_RESETS);

	pr_info("EVM: switch to %s video input\n", label);
	return 0;
}

static struct vpfe_config vpfe_cfg = {
       .setup_input = dm365evm_setup_video_input,
       .num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
       .sub_devs = vpfe_sub_devs,
       .card_name = "DM365 EVM",
       .ccdc = "DM365 ISIF",
       .num_clocks = 1,
       .clocks = {"vpss_master"},
};

static struct davinci_mmc_config dm365evm_mmc_config = {
	.get_ro		= cpld_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void dm365evm_emac_configure(void)
{
	/*
	 * EMAC pins are multiplexed with GPIO and UART
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 125 - 127
	 */
	davinci_cfg_reg(DM365_EMAC_TX_EN);
	davinci_cfg_reg(DM365_EMAC_TX_CLK);
	davinci_cfg_reg(DM365_EMAC_COL);
	davinci_cfg_reg(DM365_EMAC_TXD3);
	davinci_cfg_reg(DM365_EMAC_TXD2);
	davinci_cfg_reg(DM365_EMAC_TXD1);
	davinci_cfg_reg(DM365_EMAC_TXD0);
	davinci_cfg_reg(DM365_EMAC_RXD3);
	davinci_cfg_reg(DM365_EMAC_RXD2);
	davinci_cfg_reg(DM365_EMAC_RXD1);
	davinci_cfg_reg(DM365_EMAC_RXD0);
	davinci_cfg_reg(DM365_EMAC_RX_CLK);
	davinci_cfg_reg(DM365_EMAC_RX_DV);
	davinci_cfg_reg(DM365_EMAC_RX_ER);
	davinci_cfg_reg(DM365_EMAC_CRS);
	davinci_cfg_reg(DM365_EMAC_MDIO);
	davinci_cfg_reg(DM365_EMAC_MDCLK);

	/*
	 * EMAC interrupts are multiplexed with GPIO interrupts
	 * Details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 133 - 134
	 */
	davinci_cfg_reg(DM365_INT_EMAC_RXTHRESH);
	davinci_cfg_reg(DM365_INT_EMAC_RXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_TXPULSE);
	davinci_cfg_reg(DM365_INT_EMAC_MISCPULSE);
}

static void dm365evm_mmc_configure(void)
{
	/*
	 * MMC/SD pins are multiplexed with GPIO and EMIF
	 * Further details are available at the DM365 ARM
	 * Subsystem Users Guide(sprufg5.pdf) pages 118, 128 - 131
	 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);
}

static void dm365evm_usb_configure(void)
{
	davinci_cfg_reg(DM365_GPIO33);
	gpio_request(33, "usb");
	gpio_direction_output(33, 1);
	setup_usb(500, 8);
}

static void __init evm_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	if (have_imager())
		i2c_add_driver(&pca9543a_driver);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
}

static struct platform_device *dm365_evm_nand_devices[] __initdata = {
	&davinci_nand_device,
};

static inline int have_leds(void)
{
#ifdef CONFIG_LEDS_CLASS
	return 1;
#else
	return 0;
#endif
}

struct cpld_led {
	struct led_classdev	cdev;
	u8			mask;
};

static const struct {
	const char *name;
	const char *trigger;
} cpld_leds[] = {
	{ "dm365evm::ds2", },
	{ "dm365evm::ds3", },
	{ "dm365evm::ds4", },
	{ "dm365evm::ds5", },
	{ "dm365evm::ds6", "nand-disk", },
	{ "dm365evm::ds7", "mmc1", },
	{ "dm365evm::ds8", "mmc0", },
	{ "dm365evm::ds9", "heartbeat", },
};

static void cpld_led_set(struct led_classdev *cdev, enum led_brightness b)
{
	struct cpld_led *led = container_of(cdev, struct cpld_led, cdev);
	u8 reg = __raw_readb(cpld + CPLD_LEDS);

	if (b != LED_OFF)
		reg &= ~led->mask;
	else
		reg |= led->mask;
	__raw_writeb(reg, cpld + CPLD_LEDS);
}

static enum led_brightness cpld_led_get(struct led_classdev *cdev)
{
	struct cpld_led *led = container_of(cdev, struct cpld_led, cdev);
	u8 reg = __raw_readb(cpld + CPLD_LEDS);

	return (reg & led->mask) ? LED_OFF : LED_FULL;
}

static int __init cpld_leds_init(void)
{
	int	i;

	if (!have_leds() ||  !cpld)
		return 0;

	/* setup LEDs */
	__raw_writeb(0xff, cpld + CPLD_LEDS);
	for (i = 0; i < ARRAY_SIZE(cpld_leds); i++) {
		struct cpld_led *led;

		led = kzalloc(sizeof(*led), GFP_KERNEL);
		if (!led)
			break;

		led->cdev.name = cpld_leds[i].name;
		led->cdev.brightness_set = cpld_led_set;
		led->cdev.brightness_get = cpld_led_get;
		led->cdev.default_trigger = cpld_leds[i].trigger;
		led->mask = BIT(i);

		if (led_classdev_register(NULL, &led->cdev) < 0) {
			kfree(led);
			break;
		}
	}

	return 0;
}
/* run after subsys_initcall() for LEDs */
fs_initcall(cpld_leds_init);


static void __init evm_init_cpld(void)
{
	u8 mux, resets;
	const char *label;
	struct clk *aemif_clk;
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	/* Make sure we can configure the CPLD through CS1.  Then
	 * leave it on for later access to MMC and LED registers.
	 */
	aemif_clk = clk_get(NULL, "aemif");
	if (IS_ERR(aemif_clk))
		return;
	clk_enable(aemif_clk);

	if (request_mem_region(DM365_ASYNC_EMIF_DATA_CE1_BASE, SECTION_SIZE,
			"cpld") == NULL)
		goto fail;
	cpld = ioremap(DM365_ASYNC_EMIF_DATA_CE1_BASE, SECTION_SIZE);
	if (!cpld) {
		release_mem_region(DM365_ASYNC_EMIF_DATA_CE1_BASE,
				SECTION_SIZE);
fail:
		pr_err("ERROR: can't map CPLD\n");
		clk_disable(aemif_clk);
		return;
	}

	/* External muxing for some signals */
	mux = 0;

	/* Read CPLD version number */
	soc_info->cpld_version = __raw_readb(cpld + CPLD_VERSION);

	/* Read SW5 to set up NAND + keypad _or_ OneNAND (sync read).
	 * NOTE:  SW4 bus width setting must match!
	 */
	if ((__raw_readb(cpld + CPLD_SWITCH) & BIT(5)) == 0) {
		/* external keypad mux */
		mux |= BIT(7);

		platform_add_devices(dm365_evm_nand_devices,
				ARRAY_SIZE(dm365_evm_nand_devices));
	} else {
		/* no OneNAND support yet */
	}

	/* Leave external chips in reset when unused. */
	resets = BIT(3) | BIT(2) | BIT(1) | BIT(0);

	/* ... and ENET ... */
	dm365evm_emac_configure();
	soc_info->emac_pdata->phy_mask = DM365_EVM_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = DM365_EVM_MDIO_FREQUENCY;
	resets &= ~BIT(3);

	/* ... and AIC33 */
	resets &= ~BIT(1);

	/* Static video input config with SN74CBT16214 1-of-3 mux:
	 *  - port b1 == tvp7002 (mux lowbits == 1 or 6)
	 *  - port b2 == imager (mux lowbits == 2 or 7)
	 *  - port b3 == tvp5146 (mux lowbits == 5)
	 *
	 * Runtime switching could work too, with limitations.
	 */
	if (have_imager()) {
		label = "HD imager";
		mux |= CPLD_VIDEO_INPUT_MUX_IMAGER;
		/* externally mux MMC1 to imager */
		mux |= BIT(6);
		dm365evm_reset_imager(1);

	} else {

		/* we can use MMC1 ... */
		dm365evm_mmc_configure();
		davinci_setup_mmc(1, &dm365evm_mmc_config);

		if (have_tvp7002()) {
			mux |= CPLD_VIDEO_INPUT_MUX_TVP7002;
			resets &= ~BIT(2);
			label = "tvp7002 HD";
		} else {
			/* default to tvp5146 */
			mux |= CPLD_VIDEO_INPUT_MUX_TVP5146;
			resets &= ~BIT(0);
			label = "tvp5146 SD";
			dm365evm_reset_imager(0);
		}
	}
	__raw_writeb(mux, cpld + CPLD_MUX);
	__raw_writeb(resets, cpld + CPLD_RESETS);

	pr_info("EVM: %s video input\n", label);

	/* REVISIT export switches: NTSC/PAL (SW5.6), EXTRA1 (SW5.2), etc */
}

void enable_lcd(void)
{
	/* Turn on LCD backlight for DM368 */
	if (cpu_is_davinci_dm368()) {
		davinci_cfg_reg(DM365_GPIO80);

		/* Configure 9.25MHz clock to LCD */
		__raw_writeb(0x80, cpld + CPLD_RESETS);

		/* CPLD_CONN_GIO17 is level high */
		__raw_writeb(0xff, cpld + CPLD_CCD_IO1);

		/* CPLD_CONN_GIO17 is an output */
		__raw_writeb(0xfb, cpld + CPLD_CCD_DIR1);
	}
}
EXPORT_SYMBOL(enable_lcd);

void enable_hd_clk(void)
{
	u8 resets;

	resets = __raw_readb(cpld + CPLD_RESETS);
	if (cpu_is_davinci_dm368()) {
		davinci_cfg_reg(DM365_GPIO80);
		resets |= BIT(7) | BIT(6) | BIT(5);
	}

	__raw_writeb(resets, cpld + CPLD_RESETS);
}
EXPORT_SYMBOL(enable_hd_clk);

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

static void __init dm365_evm_map_io(void)
{
	/* setup input configuration for VPFE input devices */
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init();
}

static struct spi_eeprom at25640 = {
	.byte_len	= SZ_64K / 8,
	.name		= "at25640",
	.page_size	= 32,
	.flags		= EE_ADDR2,
};

static struct spi_board_info dm365_evm_spi_info[] __initconst = {
	{
		.modalias	= "at25",
		.platform_data	= &at25640,
		.max_speed_hz	= 20 * 1000 * 1000,	/* at 3v3 */
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

static void __init dm365_init_tsc2004(void)
{
	dm365_evm_tsc2004_dev.irq = gpio_to_irq(TSC2004_GPIO_IRQ_PIN);
	i2c_register_board_info(1, &dm365_evm_tsc2004_dev, 1);
}

static __init void dm365_evm_init(void)
{
	evm_init_i2c();
	davinci_serial_init(&uart_config);

	dm365evm_emac_configure();
	dm365evm_usb_configure();

	davinci_setup_mmc(0, &dm365evm_mmc_config);

	/* maybe setup mmc1/etc ... _after_ mmc0 */
	evm_init_cpld();

	dm365_init_asp(&dm365_evm_snd_data);
	dm365_init_rtc();
	dm365_init_ks(&dm365evm_ks_data);

	dm365_init_spi0(BIT(0), dm365_evm_spi_info,
			ARRAY_SIZE(dm365_evm_spi_info));

	dm365_init_tsc2004();
}

static __init void dm365_evm_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DAVINCI_DM365_EVM, "DaVinci DM36x EVM")
	.phys_io	= IO_PHYS,
	.io_pg_offst	= (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params	= (0x80000100),
	.map_io		= dm365_evm_map_io,
	.init_irq	= dm365_evm_irq_init,
	.timer		= &davinci_timer,
	.init_machine	= dm365_evm_init,
MACHINE_END

