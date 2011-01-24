/*
 * TI DaVinci DM646X EVM board
 *
 * Derived from: arch/arm/mach-davinci/board-evm.c
 * Copyright (C) 2006 Texas Instruments.
 *
 * (C) 2007-2008, MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/root_dev.h>
#include <linux/dma-mapping.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pcf857x.h>
#include <linux/etherdevice.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>

#include <media/tvp514x.h>
#include <media/tvp7002.h>
#include <media/davinci/videohd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/dm646x.h>
#include <mach/common.h>
#include <mach/mux.h>
#include <mach/psc.h>
#include <mach/serial.h>
#include <mach/i2c.h>
#include <mach/nand.h>
#include <mach/mmc.h>
#include <mach/emac.h>
#include <mach/aemif.h>

#include <mach/clock.h>
#include "clock.h"


#if defined(CONFIG_PCI)
#define HAS_PCI 1
#else
#define HAS_PCI 0
#endif

#if defined(CONFIG_MTD_NAND_DAVINCI) || \
	defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#define HAS_NAND 1
#else
#define HAS_NAND 0
#endif

#if defined(CONFIG_BLK_DEV_PALMCHIP_BK3710) || \
    defined(CONFIG_BLK_DEV_PALMCHIP_BK3710_MODULE)
#define HAS_ATA 1
#else
#define HAS_ATA 0
#endif

#if defined(CONFIG_MTD_NAND_DAVINCI) || \
    defined(CONFIG_MTD_NAND_DAVINCI_MODULE)
#define HAS_NAND 1
#else
#define HAS_NAND 0
#endif


#define DAVINCI_ASYNC_EMIF_CONTROL_BASE		0x20008000
#define DAVINCI_ASYNC_EMIF_DATA_CE0_BASE	0x42000000

#define NAND_BLOCK_SIZE		SZ_128K

/* CPLD Register 0 bits to control ATA */
#define DM646X_EVM_ATA_RST		BIT(0)
#define DM646X_EVM_ATA_PWD		BIT(1)
#define DM646X_EVM_USB_VBUS		BIT(7)
#define DM646X_EVM_CIR_UART             BIT(5)

#define DM646X_EVM_PHY_MASK		(0x2)
#define DM646X_EVM_MDIO_FREQUENCY	(2200000) /* PHY bus frequency */

#define VIDCLKCTL_OFFSET	(DAVINCI_SYSTEM_MODULE_BASE + 0x38)
#define VSCLKDIS_OFFSET		(DAVINCI_SYSTEM_MODULE_BASE + 0x6c)
#define VCH2CLK_MASK		(BIT_MASK(10) | BIT_MASK(9) | BIT_MASK(8))
#define VCH2CLK_SYSCLK8		(BIT(9))
#define VCH2CLK_AUXCLK		(BIT(9) | BIT(8))
#define VCH2CLK_VP_CLKIN2	(BIT(9) | BIT(10))

#define VCH3CLK_MASK		(BIT_MASK(14) | BIT_MASK(13) | BIT_MASK(12))
#define VCH3CLK_SYSCLK8		(BIT(13))
#define VCH3CLK_AUXCLK		(BIT(12) | BIT(13))
#define VCH3CLK_VP_CLKIN2	(BIT(13) | BIT(14))

#define VIDCH2CLK		(BIT(10))
#define VIDCH3CLK		(BIT(11))
#define VIDCH1CLK		(BIT(4))

#define VIDCH1CLKSRCCH1		(BIT(4))
#define VIDCH1CLKSRCCH0		(~BIT(4))

#define TVP7002_INPUT		(BIT(4))
#define TVP5147_INPUT		(~BIT(4))
#define VPIF_INPUT_ONE_CHANNEL	(BIT(5))
#define VPIF_INPUT_TWO_CHANNEL	(~BIT(5))
#define TVP5147_CH0		"tvp514x-0"
#define TVP5147_CH1		"tvp514x-1"

static void __iomem *vpif_vidclkctl_reg;
static void __iomem *vpif_vsclkdis_reg;
/* spin lock for updating above registers */
static spinlock_t vpif_reg_lock;


static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0),
};

/* Note: The partitioning is driven by combination of UBL and U-Boot. For
 * example, in the layout below, U-Boot puts environment in block 0
 * and UBL can be in blocks 1-5 while U-Boot resides after UBL blocks.
 */
static struct mtd_partition davinci_nand_partitions[] = {
	{
		/* U-Boot environment */
		.name		= "params",
		.offset		= 0,
		.size		= 1 * NAND_BLOCK_SIZE,
		.mask_flags	= 0,
	}, {
		/* UBL, U-Boot */
		.name		= "bootloader",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 10 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M,
		.mask_flags	= 0,
	}, {
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0,
	}
};

static struct davinci_aemif_timing evm_nandflash_timing = {
	.wsetup		= 29,
	.wstrobe	= 24,
	.whold		= 14,
	.rsetup		= 19,
	.rstrobe	= 33,
	.rhold		= 0,
	.ta		= 29,
};

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_cle 		= 0x80000,
	.mask_ale 		= 0x40000,
	.parts			= davinci_nand_partitions,
	.nr_parts		= ARRAY_SIZE(davinci_nand_partitions),
	.ecc_mode		= NAND_ECC_HW,
	.options		= 0,
	.timing			= &evm_nandflash_timing,
};

static struct resource davinci_nand_resources[] = {
	{
		.start		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE,
		.end		= DAVINCI_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags		= IORESOURCE_MEM,
	}, {
		.start		= DAVINCI_ASYNC_EMIF_CONTROL_BASE,
		.end		= DAVINCI_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
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

/* Setup DM6467 for PCI mode. In case of default EVM, the CPLD along with FET
 * switches takes care of switching to PCI Boot mode when the EVM is put in
 * the PCI slot and this funtion needs to do nothing. While in case when the
 * CPLD code/ Hardware is reworked (not to do autoswitch), the code below
 * handles mux configurations to switch to PCI (Host) mode  and takes care of
 * driving RST# over PCI Bus.
 *
 * Note: This function relies on h/w setting of PCIEN to distinguish between
 * modified and unmodified EVM and might not work in case s/w (e.g., bootloader)
 * is manipulating PCIEN after booting.
 */
static void dm646xevm_pci_setup(void)
{
	void __iomem *base = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE);

	/* Skip this if PCIEN is already set in  PINMUX0 */
	if (!((__raw_readl(base + PINMUX0)) & (1<<2))) {
		/* Power up the I/O cells for PCI interface */
		__raw_writel(__raw_readl(base + DM64XX_VDD3P3V_PWDN)
				& ~(3<<16), base + DM64XX_VDD3P3V_PWDN);

		davinci_cfg_reg(DM646X_HPI32EN);

		/* Drive GPIO[13] High to avoid reset when PCI is
		 * enabled
		 */
		if (gpio_request(13, "RST#") != 0) {
			pr_err("Request for GPIO13 failed.\n");
			return;
		}

		gpio_direction_output(13, 1);

		/* Ensure AUDCK1 is disabled to control GPIO[2] */
		davinci_cfg_reg(DM646X_AUDCK1);

		davinci_cfg_reg(DM646X_PCIEN);

		/* Drive GPIO[2] high to take the PCI bus out of reset
		 * (drive RST#) and select B2 of the FET mux on EVM to
		 * deselect NAND and switch to PCI Bus
		 */
		if (gpio_request(2, "PCIRST#") != 0) {
			pr_err("Request for GPIO2 failed.\n");
			return;
		}
		gpio_direction_output(2, 1);
	} else {
		pr_info("PCI_EN is already asserted.\n");
	}
}
static struct i2c_client *cpld_reg0_client;

/* CPLD Register 0 Client: used for I/O Control */
static int cpld_reg0_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	u8 data;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &data,
		},
	};

	cpld_reg0_client = client;

	/* Clear UART CIR to enable cir operation. */
		i2c_transfer(client->adapter, msg, 1);
		data &= ~(DM646X_EVM_CIR_UART);
		i2c_transfer(client->adapter, msg + 1, 1);

	if (HAS_ATA && !HAS_PCI) {
		/* Clear ATA_RSTn and ATA_PWD bits to enable ATA operation. */
		i2c_transfer(client->adapter, msg, 1);
		data &= ~(DM646X_EVM_ATA_RST | DM646X_EVM_ATA_PWD);
		i2c_transfer(client->adapter, msg + 1, 1);
	}

	setup_usb(500, 8);

	return 0;
}

void usb_vbus_control(u8 on)
{
	u8 data;
	struct i2c_msg msg[2] = {
		{
			.addr = cpld_reg0_client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &data,
		},
		{
			.addr = cpld_reg0_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &data,
		},
	};

	i2c_transfer(cpld_reg0_client->adapter, msg, 1);
	if (on)
		data |= DM646X_EVM_USB_VBUS;
	else
		data &= ~DM646X_EVM_USB_VBUS;

	i2c_transfer(cpld_reg0_client->adapter, msg + 1, 1);
}
EXPORT_SYMBOL(usb_vbus_control);

static const struct i2c_device_id cpld_reg_ids[] = {
	{ "cpld_reg0", 0, },
	{ },
};

static struct i2c_driver dm6467evm_cpld_driver = {
	.driver.name	= "cpld_reg0",
	.id_table	= cpld_reg_ids,
	.probe		= cpld_reg0_probe,
};

/* LEDS */

static struct gpio_led evm_leds[] = {
	{ .name = "DS1", .active_low = 1, },
	{ .name = "DS2", .active_low = 1, },
	{ .name = "DS3", .active_low = 1, },
	{ .name = "DS4", .active_low = 1, },
};

static const struct gpio_led_platform_data evm_led_data = {
	.num_leds = ARRAY_SIZE(evm_leds),
	.leds     = evm_leds,
};

static struct platform_device *evm_led_dev;

static int evm_led_setup(struct i2c_client *client, int gpio,
			unsigned int ngpio, void *c)
{
	struct gpio_led *leds = evm_leds;
	int status;

	while (ngpio--) {
		leds->gpio = gpio++;
		leds++;
	};

	evm_led_dev = platform_device_alloc("leds-gpio", 0);
	platform_device_add_data(evm_led_dev, &evm_led_data,
				sizeof(evm_led_data));

	evm_led_dev->dev.parent = &client->dev;
	status = platform_device_add(evm_led_dev);
	if (status < 0) {
		platform_device_put(evm_led_dev);
		evm_led_dev = NULL;
	}
	return status;
}

static int evm_led_teardown(struct i2c_client *client, int gpio,
				unsigned ngpio, void *c)
{
	if (evm_led_dev) {
		platform_device_unregister(evm_led_dev);
		evm_led_dev = NULL;
	}
	return 0;
}

static int evm_sw_gpio[4] = { -EINVAL, -EINVAL, -EINVAL, -EINVAL };

static int evm_sw_setup(struct i2c_client *client, int gpio,
			unsigned ngpio, void *c)
{
	int status;
	int i;
	char label[10];

	for (i = 0; i < 4; ++i) {
		snprintf(label, 10, "user_sw%d", i);
		status = gpio_request(gpio, label);
		if (status)
			goto out_free;
		evm_sw_gpio[i] = gpio++;

		status = gpio_direction_input(evm_sw_gpio[i]);
		if (status) {
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
			goto out_free;
		}

		status = gpio_export(evm_sw_gpio[i], 0);
		if (status) {
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
			goto out_free;
		}
	}
	return status;
out_free:
	for (i = 0; i < 4; ++i) {
		if (evm_sw_gpio[i] != -EINVAL) {
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
		}
	}
	return status;
}

static int evm_sw_teardown(struct i2c_client *client, int gpio,
			unsigned ngpio, void *c)
{
	int i;

	for (i = 0; i < 4; ++i) {
		if (evm_sw_gpio[i] != -EINVAL) {
			gpio_unexport(evm_sw_gpio[i]);
			gpio_free(evm_sw_gpio[i]);
			evm_sw_gpio[i] = -EINVAL;
		}
	}
	return 0;
}

static int evm_pcf_setup(struct i2c_client *client, int gpio,
			unsigned int ngpio, void *c)
{
	int status;

	if (ngpio < 8)
		return -EINVAL;

	status = evm_sw_setup(client, gpio, 4, c);
	if (status)
		return status;

	return evm_led_setup(client, gpio+4, 4, c);
}

static int evm_pcf_teardown(struct i2c_client *client, int gpio,
			unsigned int ngpio, void *c)
{
	BUG_ON(ngpio < 8);

	evm_sw_teardown(client, gpio, 4, c);
	evm_led_teardown(client, gpio+4, 4, c);

	return 0;
}

static struct pcf857x_platform_data pcf_data = {
	.gpio_base	= DAVINCI_N_GPIO+1,
	.setup		= evm_pcf_setup,
	.teardown	= evm_pcf_teardown,
};

/* Most of this EEPROM is unused, but U-Boot uses some data:
 *  - 0x7f00, 6 bytes Ethernet Address
 *  - ... newer boards may have more
 */

static struct at24_platform_data eeprom_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = 64,
	.flags          = AT24_FLAG_ADDR16,
	.setup          = davinci_get_mac_addr,
	.context	= (void *)0x7f00,
};

static u8 dm646x_iis_serializer_direction[] = {
       TX_MODE, RX_MODE, INACTIVE_MODE, INACTIVE_MODE,
};

static u8 dm646x_dit_serializer_direction[] = {
       TX_MODE,
};

static struct snd_platform_data dm646x_evm_snd_data[] = {
	{
		.tx_dma_offset  = 0x400,
		.rx_dma_offset  = 0x400,
		.op_mode        = DAVINCI_MCASP_IIS_MODE,
		.num_serializer = ARRAY_SIZE(dm646x_iis_serializer_direction),
		.tdm_slots      = 2,
		.serial_dir     = dm646x_iis_serializer_direction,
		.eventq_no      = EVENTQ_0,
	},
	{
		.tx_dma_offset  = 0x400,
		.rx_dma_offset  = 0,
		.op_mode        = DAVINCI_MCASP_DIT_MODE,
		.num_serializer = ARRAY_SIZE(dm646x_dit_serializer_direction),
		.tdm_slots      = 32,
		.serial_dir     = dm646x_dit_serializer_direction,
		.eventq_no      = EVENTQ_0,
	},
};

static struct i2c_client *cpld_client;

static int cpld_video_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	cpld_client = client;
	return 0;
}

static int __devexit cpld_video_remove(struct i2c_client *client)
{
	cpld_client = NULL;
	return 0;
}

static const struct i2c_device_id cpld_video_id[] = {
	{ "cpld_video", 0 },
	{ }
};

static struct i2c_driver cpld_video_driver = {
	.driver = {
		.name	= "cpld_video",
	},
	.probe		= cpld_video_probe,
	.remove		= cpld_video_remove,
	.id_table	= cpld_video_id,
};

static void evm_init_cpld(void)
{
	i2c_add_driver(&cpld_video_driver);
}

static struct i2c_board_info __initdata i2c_info[] =  {
	{
		I2C_BOARD_INFO("24c256", 0x50),
		.platform_data  = &eeprom_info,
	},
	{
		I2C_BOARD_INFO("pcf8574a", 0x38),
		.platform_data	= &pcf_data,
	},
	{
		I2C_BOARD_INFO("cpld_reg0", 0x3a),
	},
	{
		I2C_BOARD_INFO("tlv320aic33", 0x18),
	},
	{
		I2C_BOARD_INFO("cpld_video", 0x3b),
	},
	{
		I2C_BOARD_INFO("cdce949", 0x6C),
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq       = 100 /* kHz */,
	.bus_delay      = 0 /* usec */,
};

static int set_vpif_clock(int mux_mode, int hd)
{
	unsigned long flags;
	unsigned int value;
	int val = 0;
	int err = 0;

	if (!vpif_vidclkctl_reg || !vpif_vsclkdis_reg || !cpld_client)
		return -ENXIO;

	/* disable the clock */
	spin_lock_irqsave(&vpif_reg_lock, flags);
	value = __raw_readl(vpif_vsclkdis_reg);
	value |= (VIDCH3CLK | VIDCH2CLK);
	__raw_writel(value, vpif_vsclkdis_reg);
	spin_unlock_irqrestore(&vpif_reg_lock, flags);

	val = i2c_smbus_read_byte(cpld_client);
	if (val < 0)
		return val;

	if (mux_mode == 1)
		val &= ~0x40;
	else
		val |= 0x40;

	err = i2c_smbus_write_byte(cpld_client, val);
	if (err)
		return err;

	value = __raw_readl(vpif_vidclkctl_reg);
	value &= ~(VCH2CLK_MASK);
	value &= ~(VCH3CLK_MASK);

	/* Here it is assumed that channel3 will use same
	   clock as channel-2. If channel3 operates in SD
	   mode, this needs to be changed */
	value |= (VCH2CLK_VP_CLKIN2 | VCH3CLK_VP_CLKIN2);

	__raw_writel(value, vpif_vidclkctl_reg);

	spin_lock_irqsave(&vpif_reg_lock, flags);
	value = __raw_readl(vpif_vsclkdis_reg);
	/* enable the clock */
	value &= ~(VIDCH3CLK | VIDCH2CLK);
	__raw_writel(value, vpif_vsclkdis_reg);
	spin_unlock_irqrestore(&vpif_reg_lock, flags);

	return 0;
}

static struct vpif_subdev_info dm646x_vpif_subdev[] = {
#if defined(CONFIG_VIDEO_ADV7343) || defined(CONFIG_VIDEO_ADV7343_MODULE)
	{
		.name	= "adv7343",
		.board_info = {
			I2C_BOARD_INFO("adv7343", 0x2a),
		},
	},
#endif
	{
		.name	= "ths7303",
		.board_info = {
			I2C_BOARD_INFO("ths7303", 0x2c),
		},
	},
#if defined(CONFIG_VIDEO_THS8200) || defined(CONFIG_VIDEO_THS8200_MODULE)
	{
		.name	= "ths8200",
		.board_info = {
			I2C_BOARD_INFO("ths8200", 0x20),
		},
	},
#endif
};

static const char *output[] = {
	"Composite",
	"Component",
	"S-Video",
};

static struct vpif_display_config dm646x_vpif_display_config = {
	.set_clock	= set_vpif_clock,
	.subdevinfo	= dm646x_vpif_subdev,
	.subdev_count	= ARRAY_SIZE(dm646x_vpif_subdev),
	.output		= output,
	.output_count	= ARRAY_SIZE(output),
	.card_name	= "DM646x EVM",
};

/**
 * setup_vpif_input_path()
 * @channel: channel id (0 - CH0, 1 - CH1)
 * @sub_dev_name: ptr sub device name
 *
 * This will set vpif input to capture data from tvp514x or
 * tvp7002.
 */
static int setup_vpif_input_path(int channel, const char *sub_dev_name)
{
	int err = 0;
	int val;

	/* for channel 1, we don't do anything */
	if (channel != 0)
		return 0;

	if (!cpld_client)
		return -ENXIO;

	val = i2c_smbus_read_byte(cpld_client);
	if (val < 0)
		return val;

	if (!strcmp(sub_dev_name, TVP5147_CH0) ||
	    !strcmp(sub_dev_name, TVP5147_CH1))
		val &= TVP5147_INPUT;
	else
		val |= TVP7002_INPUT;

	err = i2c_smbus_write_byte(cpld_client, val);
	if (err)
		return err;
	return 0;
}

/**
 * setup_vpif_input_channel_mode()
 * @mux_mode:  mux mode. 0 - 1 channel or (1) - 2 channel
 *
 * This will setup input mode to one channel (TVP7002) or 2 channel (TVP5147)
 */
static int setup_vpif_input_channel_mode(int mux_mode)
{
	unsigned long flags;
	int err = 0;
	int val;
	u32 value, value1;

	if (!vpif_vsclkdis_reg || !cpld_client)
		return -ENXIO;

	val = i2c_smbus_read_byte(cpld_client);
	if (val < 0)
		return val;

	spin_lock_irqsave(&vpif_reg_lock, flags);
	value = __raw_readl(vpif_vsclkdis_reg);
	value1 = __raw_readl(vpif_vidclkctl_reg);
	if (mux_mode) {
		val &= VPIF_INPUT_TWO_CHANNEL;
		value |= VIDCH1CLK;
		value1 |= VIDCH1CLKSRCCH1;
	} else {
		val |= VPIF_INPUT_ONE_CHANNEL;
		value &= ~VIDCH1CLK;
		value1 &= VIDCH1CLKSRCCH0;
	}
	__raw_writel(value, vpif_vsclkdis_reg);
	__raw_writel(value1, vpif_vidclkctl_reg);
	spin_unlock_irqrestore(&vpif_reg_lock, flags);

	err = i2c_smbus_write_byte(cpld_client, val);
	if (err)
		return err;

	return 0;
}

#ifndef CONFIG_VIDEO_TVP7002
static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity = 0,
	.hs_polarity = 1,
	.vs_polarity = 1
};
#endif

struct tvp7002_platform_data tvp7002_pdata = {
       .clk_polarity = 1,
       .hs_polarity = 1,
       .vs_polarity = 1,
       .fid_polarity = 1,
};

#define TVP514X_STD_ALL (V4L2_STD_NTSC | V4L2_STD_PAL)
#define TVP7002_STD_ALL        (V4L2_STD_525P_60   | V4L2_STD_625P_50 	|\
				V4L2_STD_525_60    | V4L2_STD_625_50   |\
				V4L2_STD_720P_50   | V4L2_STD_720P_60 	|\
				V4L2_STD_1080I_50  | V4L2_STD_1080I_60 	|\
				V4L2_STD_1080P_50  | V4L2_STD_1080P_60)

static struct vpif_subdev_info vpif_capture_sdev_info[] = {
#ifndef CONFIG_VIDEO_TVP7002
	{
		.name	= TVP5147_CH0,
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5d),
			.platform_data = &tvp5146_pdata,
		},
		.input = INPUT_CVBS_VI2B,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.can_route = 1,
		.vpif_if = {
			.if_type = VPIF_IF_BT656,
			.hd_pol = 1,
			.vd_pol = 1,
			.fid_pol = 0,
		},
	},
	{
		.name	= TVP5147_CH1,
		.board_info = {
			I2C_BOARD_INFO("tvp5146", 0x5c),
			.platform_data = &tvp5146_pdata,
		},
		.input = INPUT_SVIDEO_VI2C_VI1C,
		.output = OUTPUT_10BIT_422_EMBEDDED_SYNC,
		.can_route = 1,
		.vpif_if = {
			.if_type = VPIF_IF_BT656,
			.hd_pol = 1,
			.vd_pol = 1,
			.fid_pol = 0,
		},
	},
#else
	{
		.name = "ths7353",
		.board_info = {
			I2C_BOARD_INFO("ths7353", 0x2e),
			/* From driver point of view, we need only
			   channel number for Luma*/
			.platform_data = (void *)
					CONFIG_VIDEO_THS7353_LUMA_CHANNEL,
		},
	},
	{
		.name = "tvp7002",
		.board_info = {
			I2C_BOARD_INFO("tvp7002", 0x5d),
			.platform_data = &tvp7002_pdata,
		},
		.vpif_if = {
			.if_type = VPIF_IF_BT1120,
			.hd_pol = 1,
			.vd_pol = 1,
			.fid_pol = 0,
		},
	},
#endif
};

static const struct vpif_input dm6467_ch0_inputs[] = {
	{
		.input = {
			.index = 0,
			.name = "Composite",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP514X_STD_ALL,
		},
		.subdev_name = TVP5147_CH0,
	},
	{
		.input = {
			.index = 1,
			.name = "Component",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP7002_STD_ALL,
		},
		.subdev_name = "tvp7002",
	},
};

static const struct vpif_input dm6467_ch1_inputs[] = {
       {
		.input = {
			.index = 0,
			.name = "S-Video",
			.type = V4L2_INPUT_TYPE_CAMERA,
			.std = TVP514X_STD_ALL,
		},
		.subdev_name = TVP5147_CH1,
	},
};

static struct vpif_capture_config dm646x_vpif_capture_cfg = {
	.setup_input_path = setup_vpif_input_path,
	.setup_input_channel_mode = setup_vpif_input_channel_mode,
	.subdev_info = vpif_capture_sdev_info,
	.subdev_count = ARRAY_SIZE(vpif_capture_sdev_info),
	.card_name	= "DM646x EVM",
	.chan_config[0] = {
		.inputs = dm6467_ch0_inputs,
		.input_count = ARRAY_SIZE(dm6467_ch0_inputs),
	},
	.chan_config[1] = {
		.inputs = dm6467_ch1_inputs,
		.input_count = ARRAY_SIZE(dm6467_ch1_inputs),
	},
};

static void __init evm_init_video(void)
{
	vpif_vidclkctl_reg = ioremap(VIDCLKCTL_OFFSET, 4);
	vpif_vsclkdis_reg = ioremap(VSCLKDIS_OFFSET, 4);
	if (!vpif_vidclkctl_reg || !vpif_vsclkdis_reg) {
		pr_err("Can't map VPIF VIDCLKCTL or VSCLKDIS registers\n");
		return;
	}
	spin_lock_init(&vpif_reg_lock);

	dm646x_setup_vpif(&dm646x_vpif_display_config,
			  &dm646x_vpif_capture_cfg);
}

static void __init evm_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_add_driver(&dm6467evm_cpld_driver);
	i2c_register_board_info(1, i2c_info, ARRAY_SIZE(i2c_info));
	evm_init_cpld();
	evm_init_video();
}

static void __init davinci_map_io(void)
{
	dm646x_init();
}

static struct spi_eeprom at25640a = {
	.byte_len	= SZ_64K / 2,
	.name		= "at25640a",
	.page_size	= 64,
	.flags		= EE_ADDR2,
};

static struct spi_board_info dm646x_evm_spi_info[] __initconst = {
	{
		.modalias	= "at25",
		.platform_data	= &at25640a,
		.max_speed_hz	= 10 * 1000 * 1000,	/* at 3v3 */
		.bus_num	= 0,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};

static __init void evm_init(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	evm_init_i2c();
	davinci_serial_init(&uart_config);
	dm646x_init_mcasp0(&dm646x_evm_snd_data[0]);
	dm646x_init_mcasp1(&dm646x_evm_snd_data[1]);
	dm646x_init_cir_device();

	if (HAS_PCI) {
		if (HAS_ATA)
			pr_warning("WARNING: both PCI and IDE are "
					"enabled, but they share some pins.\n"
					"\tDisable PCI for IDE support.\n");
		if (HAS_NAND)
			pr_warning("WARNING: both PCI and NAND are "
					"enabled, but they share AEMIF pins.\n"
					"\tDisable PCI for NAND support.\n");
		dm646xevm_pci_setup();
	} else {

		if (HAS_ATA)
			dm646x_init_ide();
		if (HAS_NAND)
			platform_device_register(&davinci_nand_device);
	}

	soc_info->emac_pdata->phy_mask = DM646X_EVM_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = DM646X_EVM_MDIO_FREQUENCY;

	dm646x_init_spi0(dm646x_evm_spi_info, ARRAY_SIZE(dm646x_evm_spi_info));
}

static __init void davinci_dm646x_evm_irq_init(void)
{
	davinci_irq_init();
}

MACHINE_START(DAVINCI_DM6467_EVM, "DaVinci DM646x EVM")
	.phys_io      = IO_PHYS,
	.io_pg_offst  = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params  = (0x80000100),
	.map_io       = davinci_map_io,
	.init_irq     = davinci_dm646x_evm_irq_init,
	.timer        = &davinci_timer,
	.init_machine = evm_init,
MACHINE_END



