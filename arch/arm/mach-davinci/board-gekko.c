/*
 * BTicino S.p.A. gekko platform support based on evm-dm365 board
 *
 * Massimiliano Iuliano, Davide Bonfanti
 * Copyright (C) 2014 , BTicino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/mc44cc373.h>
#include <linux/i2c/i2c_kb_touch.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/spi/zl38005.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/serial_8250.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/videodev2.h>
#include <linux/i2c/tda9885.h>
#include <linux/i2c/tvp5150.h>
#include <linux/i2c/i2c_irq.h>
#include <linux/pm_loss.h>
#include <linux/irq_gpio.h>
#include <linux/gpio_keys.h>
#include <media/soc_camera.h>
#include <sound/davinci_gekko_asoc.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/mux.h>
#include <mach/hardware.h>
#include <mach/dm365.h>
#include <mach/nand.h>
#include <mach/psc.h>
#include <mach/common.h>
#include <mach/i2c.h>
#include <mach/serial.h>
#include <mach/mux.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <mach/usb.h>
#include <media/soc_camera.h>
#include <media/tvp5150.h>
#include <mach/aemif.h>
#include <mach/sram.h>
#include <mach/gekko.h>
#include <video/davincifb.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/tsc2005.h>
#include <media/davinci/vid_encoder_types.h>
#include <media/davinci/davinci_enc_mngr.h>
#include <media/davinci/videohd.h>
#include <media/ov971x.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define GEKKO_PHY_MASK              (0x2)
#define GEKKO_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
#define HW_IN_CLOCKOUT2_UDA_I2S
#define NAND_BLOCK_SIZE         SZ_128K
#define DAVINCI_BOARD_MAX_NR_UARTS 3

struct work_struct late_init_work;
static struct timer_list startup_timer;
#define TIME_TO_LATE_INIT 1500

/* PWM Definitions - not found elsewhere */
#define DM365_PWM0_CONTROL_BASE	0x01C22000

/* registers */
#define PWM_CFG			(0x08)
#define PWM_START		(0x0C)
#define PWM_PER			(0x14)
#define PWM_PH1D		(0x18)
/* Bit position */
#define MODE			(0)
#define P1OUT			(4)
#define INACTOUT		(5)
#define INTEN			(6)
/* modes */
#define PWM_DISABLED		0
#define PWM_ONESHOT		1
#define PWM_CONTINUOS		2

#define OV971X_XCLK		24

static int gekko_debug = 1;
module_param(gekko_debug, int, 0644);
MODULE_PARM_DESC(gekko_debug, "Debug level 0-1");

extern unsigned int system_rev;
extern int lookup_resistors(int cnt);


void wp_set(int enable)
{
	gpio_direction_output(poE2_WPn, enable);
}

static struct at24_platform_data at24_info = {
	.byte_len = (256 * 1024) / 8,
	.page_size = 64,
	.flags = AT24_FLAG_ADDR16,
	.setup = davinci_get_mac_addr,
	.context = (void *)0x19e,       /* where it gets the mac-address */
	.wpset = wp_set,
};

static struct mtd_partition gekko_nand_partitions[] = {
	{
		/* U-Boot */
		.name           = "u-boot",
		.offset         = 0,
		.size           = 12 * NAND_BLOCK_SIZE, /* 1,536 MByte*/
		.mask_flags     = 0,
	}, {
		/* U-Boot environment */
		.name           = "u-boot e",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * NAND_BLOCK_SIZE, /* 256 kByte*/
		.mask_flags     = 0,
	}, {	/* Recovery copy of kernel */
		.name           = "kernel_c",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 6 * SZ_1M, /* 6 MByte*/
		.mask_flags     = 0,
	}, {	/* Recovery copy of rootfs */
		.name           = "rootfs_c",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 22 * SZ_1M, /* 22 MByte */
		.mask_flags     = 0,
	}, {	/* Primary copy of kernel */
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 6 * SZ_1M, /* 6 MByte */
		.mask_flags     = 0,
	}, {	/* Primary copy of rootfs */
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 60 * SZ_1M, /* 60 MByte */
		.mask_flags     = 0,
	}, {	/* Configurations and extras */
		.name           = "extras",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
		.mask_flags     = 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct davinci_aemif_timing gekko_nandflash_timing = {
	.wsetup		= 5,
	.wstrobe	= 10,
	.whold		= 5,
	.rsetup		= 5,
	.rstrobe	= 10,
	.rhold		= 5,
	.ta		= 10,
};

static struct davinci_nand_pdata davinci_nand_data = {
	.mask_chipsel	= 0, /* BIT(14), enable a second nand flash */
	.parts		= gekko_nand_partitions,
	.nr_parts	= ARRAY_SIZE(gekko_nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.options	= NAND_USE_FLASH_BBT,
	.ecc_bits	= 4,
	.timing		= &gekko_nandflash_timing,
};

static struct resource davinci_nand_resources[] = {
	{
		.start          = DM365_ASYNC_EMIF_DATA_CE0_BASE,
		.end            = DM365_ASYNC_EMIF_DATA_CE0_BASE + SZ_32M - 1,
		.flags          = IORESOURCE_MEM,
	}, {
		.start          = DM365_ASYNC_EMIF_CONTROL_BASE,
		.end            = DM365_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
		.flags          = IORESOURCE_MEM,
	},
};

static struct platform_device davinci_nand_device = {
	.name                   = "davinci_nand",
	.id                     = 0,
	.num_resources          = ARRAY_SIZE(davinci_nand_resources),
	.resource               = davinci_nand_resources,
	.dev                    = {
		.platform_data  = &davinci_nand_data,
	},
};

/* [VideoIn] TDA9885 : Video Demolutator */
static struct tda9885_platform_data tda9885_defaults = {
	.switching_mode = 0xf2,
	.adjust_mode = 0xd0,
	.data_mode = 0x0b,
	.power = poENABLE_VIDEO_IN,

};

void gekko_tvp510x_reset(void)
{
	printk(KERN_DEBUG "Reset tvp510x PalDecoder \n");

	/* Active-low reset. RESETB can be used only when PDN = 1 */
	gpio_direction_output(poPDEC_PWRDNn, 1);
	mdelay(1);

	gpio_direction_output(poPDEC_RESETn, 0);
	mdelay(1); /* 500 nsec is enough */
	gpio_direction_output(poPDEC_RESETn, 1);
	mdelay(10); /* ensure detect device after reset */
}

/* [VideoIn] TVP5150 : Pal Decoder */
static struct tvp5150_platform_data tvp5150_pdata = {
	.pdn = poPDEC_PWRDNn,
	.resetb = &gekko_tvp510x_reset,
};

/* Inputs available at the TDA9885 */
static struct v4l2_input tda9885_inputs[] = {
	{
		.index = 0,
		.name = "SCS Modulated video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_PAL,
	},
};

/* Inputs available at the TVP5150 */
static struct v4l2_input tvp5151_inputs[] = {
	{
		.index = 0,
		.name = "Composite",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_PAL,
	},
};

/*
 * this is the route info for connecting each input to decoder
 * ouput that goes to vpfe. There is a one to one correspondence
 * with tvp5151_inputs
 */
static struct vpfe_route tvp5151_routes[] = {
	{
		.input = TVP5150_COMPOSITE0,
		.output = TVP5150_NORMAL,
	},
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.module_name = "tda9885",
		.grp_id = VPFE_SUBDEV_TVP5150,
		.num_inputs = 1,
		.inputs = tda9885_inputs,
		.can_route = 1,
		.board_info = { /* Video Demodulator */
				I2C_BOARD_INFO("tda9885", 0x43),
				.platform_data = &tda9885_defaults,
		},
	},
	{
		.module_name = "tvp5150",
		.grp_id = VPFE_SUBDEV_TVP5150,
		.num_inputs = ARRAY_SIZE(tvp5151_inputs),
		.inputs = tvp5151_inputs,
		.routes = tvp5151_routes,
		.can_route = 1,
		.ccdc_if_params = {
			.if_type = VPFE_BT656,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {	/* Pal Decoder */
			I2C_BOARD_INFO("tvp5150", 0x5d),
			.platform_data = &tvp5150_pdata,
		},
	},
};

/* choose tvp5151 or ov971x enter work status
 * argument: 0 for tvp5151 work, 1 for ov971x work
 */
static void gekko_switch_video_channel(int channel)
{
	if (channel == 0)
		gpio_direction_output(poPDEC_PWRDNn, 1);

	else if (channel == 1)
		gpio_direction_output(poPDEC_PWRDNn, 0);

}

/* Set : video_input */
static int gekko_setup_video_input(enum vpfe_subdev_id id)
{
	/*Nothing todo*/
	return 0;
}

static struct vpfe_config vpfe_cfg = {
	.setup_input = gekko_setup_video_input,
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "DM365 GEKKO",
	.ccdc = "DM365 ISIF",
	.num_clocks = 1,
	.clocks = {"vpss_master"},
};

void gekko_phy_power(int on)
{
	gpio_set_value(poENET_RESETn, on);
	if (!on)
		/* must be at least 10ms due to board hw */
		msleep(10);
	else
		/* 80msec to rise to 2V, with 40msec margin */
		msleep(120);
}

static void gekko_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = GEKKO_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = GEKKO_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = gekko_phy_power;

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

static void gekko_lcd_configure(void)
{
	davinci_cfg_reg(DM365_VOUT_COUTL_EN);
	davinci_cfg_reg(DM365_VOUT_COUTH_EN);
	davinci_cfg_reg(DM365_VOUT_HVSYNC);
	davinci_cfg_reg(DM365_VOUT_OE);
	davinci_cfg_reg(DM365_VCLK);
	davinci_cfg_reg(DM365_VOUT_B0);
	davinci_cfg_reg(DM365_VOUT_B1);
	davinci_cfg_reg(DM365_VOUT_B2);
	davinci_cfg_reg(DM365_VOUT_R0);
	davinci_cfg_reg(DM365_VOUT_R1);
	davinci_cfg_reg(DM365_VOUT_R2);
	davinci_cfg_reg(DM365_VOUT_G0);
	davinci_cfg_reg(DM365_VOUT_G1);
}

static u64 davinci_fb_dma_mask = DMA_BIT_MASK(32);

static struct davincifb_platform_data gekko_fb_platform_data = {
	.invert_field = 0,
};

static struct resource davincifb_resources[] = {
	{
		.start          = IRQ_VLCDINT,
		.end            = IRQ_VLCDINT,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = IRQ_VDINT0,
		.end            = IRQ_VDINT0,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = IRQ_VDINT1,
		.end            = IRQ_VDINT1,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = IRQ_VDINT2,
		.end            = IRQ_VDINT2,
		.flags          = IORESOURCE_IRQ,
	},
	{
		.start          = 0x01c70000,
		.end            = 0x01c70000 + 0xff,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = 0x01c70200,
		.end            = 0x01c70200 + 0xff,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = 0x01c71C00,
		.end            = 0x01c71C00 + 0x1ff,
		.flags          = IORESOURCE_MEM,
	},
	{
		.start          = 0x01c71E00,
		.end            = 0x01c71E00 + 0x1ff,
		.flags          = IORESOURCE_MEM,
	},
};

static struct platform_device davinci_fb_device = {
	.name           = "davincifb",
	.id             = -1,
	.dev = {
		.dma_mask               = &davinci_fb_dma_mask,
		.coherent_dma_mask      = DMA_BIT_MASK(32),
		.platform_data          = &gekko_fb_platform_data,
	},
	.num_resources = ARRAY_SIZE(davincifb_resources),
	.resource      = davincifb_resources,
};

int gekko_get_pendown_state(void)
{
	return !gpio_get_value(piPENIRQn);
}

static void gekko_tsc2005_set_reset(bool level)
{
	if (level)
		gpio_set_value(poLCD_GPIO, 1);
	else
		gpio_set_value(poLCD_GPIO, 0);
}

static struct tsc2005_platform_data gekko_tsc2005_info = {

	.ts_pressure_max        = 2048,
	.ts_pressure_fudge      = 2,
	.ts_x_max               = 4096,
	.ts_x_fudge             = 4,
	.ts_y_max               = 4096,
	.ts_y_fudge             = 7,
	.ts_x_plate_ohm         = 700,
	.esd_timeout_ms         = 8000,
	.bit_x_word             = 8,
	.set_reset              = gekko_tsc2005_set_reset,
};

static struct spi_board_info gekko_spi3_info[] = {
	{
		.modalias = "tsc2005",
		.platform_data = &gekko_tsc2005_info,
		.max_speed_hz = 5000000, /* Max 25MHz for the chip*/
		.bus_num = 3,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.irq = IRQ_DM365_GPIO0_5,
	},
};

static struct davinci_adc_platform_data gekko_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device gekko_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &gekko_adc_data,
		},
	.id = 0,
};

static void pinmux_check(void)
{
	void __iomem *pinmux_reg[] = {
		IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x0),
		IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x4),
		IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x8),
		IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0xC),
		IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x10)
	};
	int pinmux[5], i;

		for (i = 0; i < 5; i++) {
			pinmux[i] = __raw_readl(pinmux_reg[i]);
		printk(KERN_INFO "pinmux%d=%X\n", i, pinmux[i]);
	}
}

static struct irq_on_gpio gekko_irq_on_gpio0[] = {
	{
		.gpio = piPOWER_FAILn,
		.irq = IRQ_DM365_GPIO0_0,
		.type = EDGE,
		.mode = GPIO_EDGE_RISING,
	}, {
		.gpio = piINT_I2Cn,
		.irq = IRQ_DM365_GPIO0_1,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piPOWER_FAILn,
		.irq = IRQ_DM365_GPIO0_2,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,

	}, {
		.gpio = piFLOOR_CALL,
		.irq = IRQ_DM365_GPIO0_4,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piTMK_INTn,
		.irq = IRQ_DM365_GPIO0_3,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piPENIRQn,
		.irq = IRQ_DM365_GPIO0_5,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	},
};

static struct irq_gpio_platform_data gekko_irq_gpio_platform_data = {
	.gpio_list = gekko_irq_on_gpio0,
	.len = ARRAY_SIZE(gekko_irq_on_gpio0),
	.gpio_common = 0,
	.irq_gpio = IRQ_DM365_GPIO0,
};

static struct platform_device gekko_irq_gpio_device = {
	.name			= "irq_gpio",
	.id			= 0,
	.num_resources		= 0,
	.resource		= NULL,
	.dev			= {
		.platform_data	= &gekko_irq_gpio_platform_data,
	},
};

static struct gpio_keys_button gekko_gpio_buttons[] = {
	{
		.code                   = KEY_5,
		.gpio                   = piFLOOR_CALL,
		.desc                   = "floor call",
		.type			= EV_KEY,
		.wakeup                 = 0,
		.interrupt		= IRQ_DM365_GPIO0_4,
		.active_low		= 1,
		.debounce_interval      = 100,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons        = gekko_gpio_buttons,
	.nbuttons       = ARRAY_SIZE(gekko_gpio_buttons),
};

static struct platform_device gekko_keys_gpio = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &gpio_key_info,
	},
};

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t gekko_powerfail_stop(int irq, void *dev_id);

static irqreturn_t gekko_powerfail_quick_check_start(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(IRQ_DM365_GPIO0_2);
	desc->chip->unmask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t gekko_powerfail_start(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(IRQ_DM365_GPIO0_2);
	desc->chip->unmask(IRQ_DM365_GPIO0_0);

	if (powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 1;
	pm_loss_power_changed(SYS_PWR_FAILING);

	/* PowerFail situation - START: power is going away */
	return IRQ_HANDLED;
}

static irqreturn_t gekko_powerfail_quick_check_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t gekko_powerfail_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);
	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_HANDLED;
}

enum gekko_pwrfail_prio {
	GEKKO_PWR_FAIL_PRIO_0,
	GEKKO_PWR_FAIL_MIN_PRIO = GEKKO_PWR_FAIL_PRIO_0,
	GEKKO_PWR_FAIL_PRIO_1,
	GEKKO_PWR_FAIL_PRIO_2,
	GEKKO_PWR_FAIL_PRIO_3,
	GEKKO_PWR_FAIL_MAX_PRIO = GEKKO_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item gekko_pm_loss_policy_items[] = {
	{
		.bus_name = "platform",
		.bus_priority = GEKKO_PWR_FAIL_PRIO_2,
	},
	{
		.bus_name = "i2c",
		.bus_priority = GEKKO_PWR_FAIL_PRIO_1,
	}

};

#define GEKKO_POLICY_NAME "gekko-default"

struct pm_loss_default_policy_table gekko_pm_loss_policy_table = {
	.name = GEKKO_POLICY_NAME,
	.items = gekko_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(gekko_pm_loss_policy_items),
};

static void gekko_powerfail_configure(struct work_struct *work)
{
	int stat;
	struct pm_loss_policy *p;
	struct irq_desc *desc;

	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    gekko_powerfail_quick_check_start,
				    gekko_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    gekko_powerfail_quick_check_stop,
				    gekko_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	desc = irq_to_desc(IRQ_DM365_GPIO0_0);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	p = pm_loss_setup_default_policy(&gekko_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(GEKKO_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       GEKKO_POLICY_NAME);

	printk(KERN_ERR "%s OK\n", __func__);
}

int platform_pm_loss_power_changed(struct device *dev,
				   enum sys_power_state s)
{
	int ret = 0;

	/* Calling platform bus pm_loss functions */
	pr_debug_pm_loss("platform_pm_loss_power_changed(%d)\n", s);

	if (dev->driver && dev->driver->pm &&
		dev->driver->pm->power_changed)
		ret = dev->driver->pm->power_changed(dev, s);
	return ret;
}

#else /* !CONFIG_PM_LOSS */
#define gekko_powerfail_configure()
#endif

/*
 *	Macro : gpio_configure_out
 */
int gpio_configure_out(int DM365_Pin, int Name, int DefVualue, char *str)
{
	int status;
	davinci_cfg_reg(DM365_Pin);
	status = gpio_request(Name, str);
	if (status) {
		printk(KERN_ERR "%s: Failed to request GPIO %s \
				 det: %d\n", __func__, str, status);
		return status;
	}
	gpio_direction_output(Name, DefVualue);
	return status;
}

/*
 *	Macro : gpio_configure_in
 */
int gpio_configure_in(int DM365_Pin, int Name, char *str)
{
	int status;
	davinci_cfg_reg(DM365_Pin);
	status = gpio_request(Name, str);
	if (status) {
		printk(KERN_ERR "%s: Failed to request GPIO %s \
				 det: %d\n", __func__, str, status);
		return status;
	}
	gpio_direction_input(Name);
	return status;
}

/*
 *	gekko_gpio_configure
 */
static void gekko_gpio_configure(void)
{
	void __iomem *pupdctl0 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x78);
	void __iomem *pupdctl1 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x7c);

	/* Configure (disable) pull down control */
	__raw_writel(0, pupdctl0);
	__raw_writel(0x40000, pupdctl1); /* EM_WAIT active pull-up */

	dm365_setup_debounce(1, 0, 4096);

	/*  -- Configure Input ---------------------------------------------- */
	gpio_request(0, "GIO0"); /* pi_INTERRUPT */
	gpio_direction_input(0);
	gpio_configure_in(DM365_GPIO50, piPOWER_FAILn, "piPOWER_FAILn");
	gpio_configure_in(DM365_GPIO35, piINT_I2Cn, "piINT_I2Cn");
	gpio_configure_in(DM365_GPIO64_57, piTMK_INTn, "piTMK_INTn (RTC)");
	gpio_configure_in(DM365_GPIO97, piPENIRQn, "piPENIRQn");
	gpio_configure_in(DM365_GPIO31, piMaster_Slave, "piMaster_Slave");
	gpio_configure_in(DM365_GPIO64_57, piPower_Aux, "piPower_Aux");
	gpio_configure_in(DM365_GPIO46, 46, "46");
	/* -- Configure Output -----------------------------------------------*/
	gpio_configure_out(DM365_GPIO96, poLCD_GPIO, 1, "poLCD_GPIO");
	gpio_configure_out(DM365_GPIO36, poNAND_WPn, 1,
			"po_NAND_WriteProtect_n,");
	gpio_configure_out(DM365_GPIO43, poEN_LOCAL_MIC, 0,
			"Local microphone control");
	gpio_configure_out(DM365_GPIO42, poMUTE_SPK, 0,
			"Local mute speaker");
	gpio_configure_out(DM365_GPIO37, poEN_CH_SUPP , 0,
			"Enable ch suppl");
	gpio_configure_out(DM365_GPIO44, poENET_RESETn, 1, "poENET_RESETn");

	gekko_phy_power(0);
	gekko_phy_power(1);

	gpio_configure_out(DM365_GPIO41, poATT_INTER, 0, "poATT_INTER");
	gpio_configure_out(DM365_GPIO64_57, poEN_LCD_3_3V, 1,
			"Enable LCD 3.3V");
	gpio_configure_out(DM365_GPIO64_57, poEN_VGL, 0,
			"Enable LCD VGL");
	gpio_configure_out(DM365_GPIO64_57, poEN_VGH, 0,
			"Enable LCD VGL");
	gpio_configure_out(DM365_GPIO78_73, poEN_AVDD, 0,
			"Enable LCD AVDD");
	gpio_configure_out(DM365_GPIO26, poEN_LCD_POWER, 0,
			"Enable LCD Power");
	gpio_configure_out(DM365_GPIO72, poVideo_Mode, 1,
			"Video Bus Mode");
	gpio_configure_out(DM365_GPIO40, poENABLE_VIDEO_IN, 1,
			"Enable video demodulator");
	gpio_configure_out(DM365_GPIO64_57, poE2_WPn, 0,
			"EEprom write protect");
	gpio_configure_out(DM365_GPIO64_57, poEN_T_COIL, 0,
			"Enable T_Coil");

	gpio_configure_out(DM365_GPIO68, poVIDEO_BUS_RES, 1,
			"video bus reset");
	gpio_configure_out(DM365_GPIO64_57, poEN_DITHER, 0,
			"enable dithering");

	gpio_configure_out(DM365_GPIO64_57, poEN_5V_A, 0,
			"enable 5V_A");
	gpio_configure_out(DM365_GPIO99, poEN_5V_FON, 0,
			"enable 5V_FON");
	/* choose tvp5151 initiation first */
	gpio_configure_out(DM365_GPIO78_73, poSN74CBT_S1, 1,
			"SN74CBT16214 switch video channel");
	gpio_configure_out(DM365_GPIO39, poPDEC_PWRDNn, 1,
			"Pal-Decoder power down");
	gpio_configure_out(DM365_GPIO38, poPDEC_RESETn, 0,	/* RESET */
			"PAL decoder Reset");

	gpio_configure_out(DM365_GPIO78_73, poSPK_PWR, 0,
			"Speaker power control");
	gpio_configure_out(DM365_GPIO78_73, poEN_LCD_5V, 1, "Enable LCD 5V");
	gpio_configure_out(DM365_GPIO101, poTOUCH_CSn, 1,
			"Touch screen chip select");
	gpio_configure_out(DM365_GPIO48, poPIC_RESETn, 1,
			"PIC AV and PIC AI reset");
	gpio_configure_out(DM365_GPIO99, poMCU_RESETn, 1,
			"keyboard mcu reset");

	gpio_configure_out(DM365_GPIO98, poEN_VCOM_LCD, 0,
			"Lcd VCOM");

	/* -- Export For Debug -----------------------------------------------*/
	if (gekko_debug) {
		gpio_export(piPOWER_FAILn, 0);
		gpio_export(piINT_I2Cn, 0);
		gpio_export(piTMK_INTn, 0);
		gpio_export(piFLOOR_CALL, 0);
		gpio_export(piPENIRQn, 0);
		gpio_export(piMaster_Slave, 0);
		gpio_export(piPower_Aux, 0);
		gpio_export(poLCD_GPIO, 0);
		gpio_export(poEN_5V_A, 0);
		gpio_export(poEN_5V_FON, 0);
		gpio_export(poATT_INTER, 0);
		gpio_export(poNAND_WPn, 0);
		gpio_export(poEN_LOCAL_MIC, 0);
		gpio_export(poMUTE_SPK, 0);
		gpio_export(poEN_CH_SUPP, 0);
		gpio_export(poEN_T_COIL, 0);
		gpio_export(poVideo_Mode, 0);
		gpio_export(poENET_RESETn, 0);
		gpio_export(poBOOT_FL_WPn, 0); /* danger */
		gpio_export(poEN_LCD_3_3V, 0);
		gpio_export(poEN_VGL, 0);
		gpio_export(poEN_VGH, 0);
		gpio_export(poEN_VCOM_LCD, 0);
		gpio_export(poEN_AVDD, 0);
		gpio_export(poEN_LCD_POWER, 0);
		gpio_export(poTOUCH_CSn, 0);
		gpio_export(poVIDEO_BUS_RES, 0);
		gpio_export(poEN_DITHER, 0);
		gpio_export(poENABLE_VIDEO_IN, 0);
		gpio_export(poE2_WPn, 0);
		gpio_export(poPDEC_PWRDNn, 0);
		gpio_export(poPDEC_RESETn, 0);
		gpio_export(poSPK_PWR, 0);
		gpio_export(poEN_LCD_5V, 0);
		gpio_export(poSN74CBT_S1, 0);
		gpio_export(poPIC_RESETn, 0);
		gpio_export(poMCU_RESETn, 0);
	}
}

static void gekko_usb_configure(void)
{
	pr_notice("Launching setup_usb\n");
	setup_usb(500, 8);
}

/* sound power control,power manage */
void gekko_en_audio_power(int value)
{
	/*NOTHING TO DO*/
}

static struct gekko_asoc_platform_data gekko_asoc_info = {
	.ext_circuit_power = gekko_en_audio_power,
};

static struct platform_device gekko_asoc_device[] = {
	{
		.name = "gekko-asoc", /*Internal Voice codec + Zarlink*/
		.id = 0,
		.dev = {
			.platform_data  = &gekko_asoc_info,
		},
	},
};

static struct snd_platform_data dm365_gekko_snd_data[] = {
		{
			.asp_chan_q = EVENTQ_3,
			.ram_chan_q = EVENTQ_3,
		},
};

/* I2C 7bit Adr */
static struct i2c_board_info __initdata gekko_i2c_info[] = {
	{	/* RTC */
		I2C_BOARD_INFO("pcf8563", 0x51),
		.irq = IRQ_DM365_GPIO0_3,
	},
	{	/* EEprom */
		I2C_BOARD_INFO("24c256", 0x53),
		.platform_data = &at24_info,
	},
	{
		I2C_BOARD_INFO("i2c_kb_touch", 0x41),
		.irq = IRQ_DM365_GPIO0_1,
	},
	{
		I2C_BOARD_INFO("tpa2028d1", 0x58),
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq = 100 /* kHz */ ,
	.bus_delay = 0 /* usec */ ,
};

static void __init gekko_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, gekko_i2c_info, ARRAY_SIZE(gekko_i2c_info));
}

static struct platform_device *gekko_devices[] __initdata = {
	&davinci_nand_device,
	&gekko_asoc_device[0],
	&gekko_hwmon_device,
	&gekko_irq_gpio_device,
	&davinci_fb_device,
	&gekko_keys_gpio,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1),
};

static void __init gekko_map_io(void)
{
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init_isif(DM365_ISIF_8BIT);
	dm365_init();
}

#ifdef ENABLING_SPI_FLASH
static struct spi_eeprom at25640 = {
	.byte_len = SZ_64K / 8,
	.name = "at25640",
	.page_size = 32,
	.flags = EE_ADDR2,
};
#endif


static void gekko_late_init(unsigned long data)
{
	void __iomem *adc_mem;
	u32 regval, index;

	del_timer(&startup_timer);

	/* setting /proc/cpuinfo hardware_version information */
	index = 1;

	adc_mem = ioremap(DM365_ADCIF_BASE, SZ_1K);
	__raw_writel(1 << index, adc_mem + CHSEL);
	regval = ADCTL_SCNIEN | ADCTL_SCNFLG;
	__raw_writel(regval, adc_mem + ADCTL);
	regval |= ADCTL_START;
	__raw_writel(regval, adc_mem + ADCTL);
	do { } while (__raw_readl(adc_mem + ADCTL) & ADCTL_START);
	regval = __raw_readl(adc_mem + AD_DAT(index));

	system_rev = lookup_resistors(regval);
	/* system_serial_low & system_serial_high can also be set here*/

	if (gpio_get_value(piPower_Aux)) {
		printk("non c'e' power_aux\n");
		INIT_WORK(&late_init_work, gekko_powerfail_configure);
		schedule_work(&late_init_work);
	}
	/* uart for expansion */
	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
}

static __init void gekko_init(void)
{

	pr_warning("Board_Init: START\n");
	if (gekko_debug > 1)
		pinmux_check();

	gekko_gpio_configure();
	gekko_tvp510x_reset();
	/* gekko choose tvp5151 work default*/
	gekko_switch_video_channel(0);
	/* 2 usart for pic */
	davinci_serial_init(&uart_config);
	mdelay(1);
	/* I2C */
	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	gekko_init_i2c();
	gekko_emac_configure();
	gekko_lcd_configure();
	/* SPI3 for touchscreen and Zarlink*/
	dm365_init_spi3(BIT(0), gekko_spi3_info, ARRAY_SIZE(gekko_spi3_info));
	gekko_usb_configure();
	/* output 24Mhz for ov971x mclk*/
	davinci_cfg_reg(DM365_GPIO31);
	dm365_init_vc(&dm365_gekko_snd_data[0]);
	/* dm365_init_asp(&dm365_dingo_snd_data[1]);*/
	platform_add_devices(gekko_devices, ARRAY_SIZE(gekko_devices));
	dm365_init_rtc();
	dm365_init_adc(&gekko_adc_data);
	gpio_direction_output(poPDEC_PWRDNn, 0);
	init_timer(&startup_timer);
	startup_timer.function = gekko_late_init;
	startup_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_LATE_INIT);
	add_timer(&startup_timer);
	if (gekko_debug)
		pinmux_check();
	pr_warning("Board_Init: DONE\n");
}

MACHINE_START(GEKKO, "Bticino gekko board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = gekko_map_io,
	.init_irq = davinci_irq_init,
	.timer = &davinci_timer,
	.init_machine = gekko_init,
MACHINE_END

/* End Of File -------------------------------------------------------------- */
