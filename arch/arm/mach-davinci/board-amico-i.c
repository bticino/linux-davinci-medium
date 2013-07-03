/*
 * BTicino S.p.A. amico platform support based on evm-dm365 board
 *
 * Simone Cianni, Davide Bonfanti
 * Copyright (C) 2012 , BTicino S.p.A.
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

#include <media/soc_camera.h>
#include <sound/davinci_amico_asoc.h>

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
#include <mach/mmc.h>
#include <mach/mux.h>
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <mach/usb.h>
#include <media/soc_camera.h>
#include <media/tvp5150.h>
#include <mach/aemif.h>

#include <mach/amico.h>
#include <video/davincifb.h>
#include <linux/spi/ads7846.h>
#include <media/davinci/vid_encoder_types.h>
#include <media/davinci/davinci_enc_mngr.h>
#include <media/davinci/videohd.h>
#include <media/ov971x.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define AMICO_PHY_MASK              (0x2)
#define AMICO_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
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

static int amico_debug = 1;
module_param(amico_debug, int, 0644);
MODULE_PARM_DESC(amico_debug, "Debug level 0-1");

extern unsigned int system_rev;
extern int lookup_resistors(int cnt);

static void amico_bl_set_intensity(int level);
/*
 * wp_set: set/unset the at24 eeprom write protect
 */
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

static struct mtd_partition amico_nand_partitions[] = {
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
		.size           = SZ_8M, /* 8 MByte*/
		.mask_flags     = 0,
	}, {	/* Recovery copy of rootfs */
		.name           = "rootfs_c",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40 * SZ_1M, /* 40 MByte */
		.mask_flags     = 0,
	}, {	/* Primary copy of kernel */
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_8M, /* 8 MByte */
		.mask_flags     = 0,
	}, {	/* Primary copy of rootfs */
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_128M + 60 * SZ_1M, /* 188 MByte */
		.mask_flags     = 0,
	}, {	/* Configurations and extras */
		.name           = "extras",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL, /* Circa 66 MByte */
		.mask_flags     = 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct davinci_aemif_timing amico_nandflash_timing = {
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
	.parts		= amico_nand_partitions,
	.nr_parts	= ARRAY_SIZE(amico_nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.options	= NAND_USE_FLASH_BBT,
	.ecc_bits	= 4,
	.timing		= &amico_nandflash_timing,
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

void amico_tvp510x_reset(void)
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
	.resetb = &amico_tvp510x_reset,
};

/* ov9712 platform data, used during reset and probe operations */
static struct ov971x_platform_data ov971x_pdata = {
	.clk_polarity = OV971x_CLK_POL_NORMAL,
	.hs_polarity = OV971x_HS_POL_POS,
	.vs_polarity = OV971x_VS_POL_POS,
	.xclk_frequency = OV971X_XCLK * 1000 * 1000,
	.out_drive_capability = OV971x_DRIVE_CAP_X1,
};

/*
#define V4L2_STD_OV971X_STD_ALL (v4l2_std_id)(V4L2_STD_720P_30 \
		| V4L2_STD_640x400_30 | V4L2_STD_320x240_30  \
		| V4L2_STD_160x120_30)
*/

/* Input available at the ov971x */
static struct v4l2_input ov971x_inputs[] = {
	{
		.index = 0,
		.name = "Camera",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_OV971x_STD_ALL,
	}
};

#if 0
/* [VideoOut] mc44cc373 Video Modulator */
static u8 mc44cc373_pars[] = {0xBA, 0x18, 0x26, 0xE8};
static struct mc44cc373_platform_data mc44cc373_pdata = {
	.num_par = 4,
	.pars = mc44cc373_pars,
	.power = po_ENABLE_VIDEO_OUT,
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
#endif

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
	},	{
		.module_name = "ov971x",
		.is_camera = 1,
		.grp_id = VPFE_SUBDEV_OV971X,
		.num_inputs = ARRAY_SIZE(ov971x_inputs),
		.inputs = ov971x_inputs,
		.ccdc_if_params = {
			.if_type = VPFE_RAW_BAYER,
			.hdpol = VPFE_PINPOL_POSITIVE,
			.vdpol = VPFE_PINPOL_POSITIVE,
		},
		.board_info = {	/* Pal Decoder */
			I2C_BOARD_INFO("ov971x", 0x30),
			.platform_data = &ov971x_pdata,
		},
	},
};

/* choose tvp5151 or ov971x enter work status
 * argument: 0 for tvp5151 work, 1 for ov971x work
 */
static void amico_switch_video_channel(int channel)
{
	if (channel == 0) {
		gpio_direction_output(poPDEC_PWRDNn, 1);
		gpio_direction_output(poSN74CBT_S0, 0);
	} else if (channel == 1)	{
		gpio_direction_output(poPDEC_PWRDNn, 0);
		gpio_direction_output(poSN74CBT_S0, 1);
	}
}

/* Set : video_input */
static int amico_setup_video_input(enum vpfe_subdev_id id)
{
	/*Nothing todo*/
	return 0;
}

static struct vpfe_config vpfe_cfg = {
	.setup_input = amico_setup_video_input,
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "DM365 AMICO",
	.ccdc = "DM365 ISIF",
	.num_clocks = 1,
	.clocks = {"vpss_master"},
};

void amico_phy_power(int on)
{
	gpio_set_value(poENET_RESETn, on);
	udelay(100);
}

static void amico_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = AMICO_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = AMICO_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = amico_phy_power;

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

static void amico_lcd_configure(void)
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

static struct davincifb_platform_data amico_fb_platform_data = {
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
		.platform_data          = &amico_fb_platform_data,
	},
	.num_resources = ARRAY_SIZE(davincifb_resources),
	.resource      = davincifb_resources,
};

int amico_get_pendown_state(void)
{
	return !gpio_get_value(piPENIRQn);
}

static struct ads7846_platform_data amico_ads7846_info = {
	.model = 7846,
	.x_max = 0x0fff,
	.y_max = 0x0fff,
	.x_plate_ohms = 240,
	.y_plate_ohms = 700,
	.keep_vref_on = 1,
	.settle_delay_usecs = 500,
	.penirq_recheck_delay_usecs = 100,

	.debounce_tol = 50,
	.debounce_max = 20,
	.debounce_rep = 3,
	.sampling_period = 20 * 1000 * 1000,
	.pressure_max = 1024,

	.vref_mv = 3300,
	.get_pendown_state = amico_get_pendown_state,
};



static struct zl38005_platform_data zl_config[] = {
	{
		.minor = 0,
		.reset_gpio = poZARLINK_RESET,
	},
};

static struct spi_board_info amico_spi3_info[] = {
	{	/* dm365 spi work between 600000 and 50000000 */
		.modalias       = "zl38005",
		.max_speed_hz   = 2 * 1000 * 1000,
		.bus_num        = 3,
		.controller_data = (void *)poZARLINK_CS,
		/* .chip_select	= poZARLINK_CS, cosi non va... */
		.chip_select 	= 0,
		.mode           = SPI_MODE_0,
		.platform_data  = &zl_config,
	},
	{
		.modalias = "ads7846",
		.platform_data = &amico_ads7846_info,
		.max_speed_hz = 2 * 1000 * 1000, /* Max 2.5MHz for the chip*/
		.bus_num = 3,
		.controller_data = (void *)poTOUCH_CSn,
		.chip_select = 1,
		.mode = SPI_MODE_0,
		.irq = IRQ_DM365_GPIO0_5,
	},
};

static struct davinci_adc_platform_data amico_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device amico_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &amico_adc_data,
		},
	.id = 0,
};

#if 0
static struct gpio_led amico_gpio_led[] = {
	{
		.name                   = "LED_SYSTEM",
		.gpio                   = poPOWER_LED_YELLOWn,
		.active_low             = 1,
		.default_trigger        = "none",
		.default_state          = LEDS_GPIO_DEFSTATE_ON,
	},
};

static struct gpio_led_platform_data amico_gpio_led_info = {
	.num_leds = 1,
	.leds = amico_gpio_led,
};

static struct platform_device amico_leds_gpio_device = {
	.name = "leds-gpio",
	.dev = {
		.platform_data = &amico_gpio_led_info,
		},
	.id = 0,
};

static void __init amico_led_init(void)
{
	davinci_cfg_reg(DM365_GPIO82);
	platform_device_register(&amico_leds_gpio_device);
}
#endif

static struct generic_bl_info amico_bl_machinfo = {
	.max_intensity = 0xff,
	.default_intensity = 0xf0,
	.limit_mask = 0xff,
	.set_bl_intensity = amico_bl_set_intensity,
	.name = "amico_backlight",
};

static struct resource gpio_resources[] = {
	{
		.start          = 0x01c67000,
		.end            = 0x01c67000 + 0x7ff,
		.flags          = IORESOURCE_MEM,
	},
};

static struct platform_device amico_bl_device = {
	.name   = "generic-bl",
	.dev    = {
		.platform_data  = &amico_bl_machinfo,
	},
	.id             = -1,
	.num_resources  = ARRAY_SIZE(gpio_resources),
	.resource       = gpio_resources,
};

static void amico_bl_set_intensity(int level)
{
	static void __iomem *base;
	static int bl_is_on;
	int tmp;

	if (!base)
		base = ioremap(DM365_PWM0_CONTROL_BASE, SZ_1K - 1);

	pr_info("set backlight lev:%d\n", level);

	level = ~level & 0xff; /* 0:max, 255:min; get opposite*/

	if (!level) {
		__raw_writeb(PWM_DISABLED << MODE, base + PWM_CFG);
		bl_is_on = 0;
		return;
	}
	if (!bl_is_on) {
		struct clk *pwm0_clk;

		davinci_cfg_reg(DM365_GPIO23);
		gpio_request(poBL_PWM, "Backlight");
		gpio_direction_output(poBL_PWM, 1);
		pwm0_clk = clk_get(NULL, "pwm0");
		if (IS_ERR(pwm0_clk)) {
			pr_err("couldn't get PWM0 CLOCK!\n");
			return;
		}
		if (clk_enable(pwm0_clk))
			pr_err("couldn't enable PWM0 CLOCK!\n");
		clk_put(pwm0_clk);
		tmp = PWM_CONTINUOS << MODE | 1 << P1OUT;
		__raw_writel(tmp, base + PWM_CFG);
		__raw_writel(0X3FF, base + PWM_PER);
		__raw_writel(level<<2, base + PWM_PH1D);
		__raw_writel(1, base + PWM_START);
		bl_is_on = 1;

		/* wait the soft start completion */
		udelay(50);
		davinci_cfg_reg(DM365_PWM0_G23);
	} else
		__raw_writel(level<<2, base + PWM_PH1D);

	return ;
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
#if 0
static struct plat_serial8250_port amico_dm365_serial_platform_data[] = {
	{
		.mapbase        = DM365_ASYNC_EMIF_DATA_CE1_BASE,
		.irq            = IRQ_DM365_GPIO0_3,
		.type           = PORT_16550A,
		.flags          = UPF_BOOT_AUTOCONF | UPF_IOREMAP
				| UART_CONFIG_TYPE | UPF_SKIP_TEST
				| UPF_FIXED_TYPE,
		.iotype         = UPIO_MEM,
		.regshift       = 0,
		.uartclk        = 8000000,
	},
	{
		.mapbase        = DM365_ASYNC_EMIF_DATA_CE1_BASE + 0x80,
		.irq            = IRQ_DM365_GPIO0_1,
		.type           = PORT_16550A,
		.flags          = UPF_BOOT_AUTOCONF | UPF_IOREMAP
				| UART_CONFIG_TYPE | UPF_SKIP_TEST
				| UPF_FIXED_TYPE,
		.iotype         = UPIO_MEM,
		.regshift       = 0,
		.uartclk        = 8000000,
	},
	{
		.flags          = 0
	},
};

static struct resource davinci_asyn_resources[] = {
	{
		.start          = DM365_ASYNC_EMIF_DATA_CE1_BASE,
		.end            = DM365_ASYNC_EMIF_DATA_CE1_BASE + SZ_32M - 1,
		.flags          = IORESOURCE_MEM,
	}, {
		.start          = DM365_ASYNC_EMIF_CONTROL_BASE,
		.end            = DM365_ASYNC_EMIF_CONTROL_BASE + SZ_4K - 1,
		.flags          = IORESOURCE_MEM,
	},
};

static struct platform_device amico_dm365_serial_device = {
	.name                   = "serial8250",
	.id                     = PLAT8250_DEV_PLATFORM1,
	.resource               = davinci_asyn_resources,
	.dev                    = {
		.platform_data  = amico_dm365_serial_platform_data,
	},
};

static void amico_uart_configure(void)
{
	struct clk *aemif_clk;
	struct davinci_aemif_timing t;
	void __iomem *base;
	int ret;
	base = ioremap(DM365_ASYNC_EMIF_CONTROL_BASE, SZ_4K - 1);

	aemif_clk = clk_get(NULL, "aemif");
	if (IS_ERR(aemif_clk)) {
		printk(KERN_ERR "couldn't get AEMIF CLOCK!!! \
				- ttyS2 won't work\n");
		return;
	}
	clk_enable(aemif_clk);
	clk_put(aemif_clk);

	/* set external bus 8-bits wide */
	ret = __raw_readb(base + A1CR_OFFSET + 1 * 4);
	ret &= ~1;
	__raw_writeb(ret, base + A1CR_OFFSET + 1 * 4);

	/* All timings in nanoseconds */
	t.wsetup = 15;
	t.wstrobe = 40;
	t.whold = 20;
	t.rsetup = 40;
	t.rstrobe = 40;
	t.rhold = 20;
	t.ta = 50;
	ret = davinci_aemif_setup_timing(&t, base, 1);
	if (ret) {
		printk(KERN_ERR  "failed setting aemif timings \
				- ttyS2 won't work");
		return;
	}

	/* setting proper pinmux for AEMIF */
	davinci_cfg_reg(DM365_AEMIF_AR2);
	davinci_cfg_reg(DM365_AEMIF_CE1);
	davinci_cfg_reg(DM365_EM_WE_OE);

	davinci_cfg_reg(DM365_GPIO51);
	gpio_direction_output(poRES_EXTUART, 1);
    mdelay(10);
	gpio_direction_output(poRES_EXTUART, 0);

	set_irq_type(gpio_to_irq(0), IRQ_TYPE_EDGE_BOTH);
	platform_device_register(&amico_dm365_serial_device);
}
#endif
static struct irq_on_gpio amico_irq_on_gpio0[] = {
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
	},
#if 0
	{
		.gpio = piINT_UART_An,
		.irq = IRQ_DM365_GPIO0_3,
		.type = LEVEL,
		.mode = GPIO_EDGE_FALLING,
	},
#endif
	{
		.gpio = piTMK_INTn,
		.irq = IRQ_DM365_GPIO0_4,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piPENIRQn,
		.irq = IRQ_DM365_GPIO0_5,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	},
};

static struct irq_gpio_platform_data amico_irq_gpio_platform_data = {
	.gpio_list = amico_irq_on_gpio0,
	.len = ARRAY_SIZE(amico_irq_on_gpio0),
	.gpio_common = 0,
	.irq_gpio = IRQ_DM365_GPIO0,
};

static struct platform_device amico_irq_gpio_device = {
	.name			= "irq_gpio",
	.id			= 0,
	.num_resources		= 0,
	.resource		= NULL,
	.dev			= {
		.platform_data	= &amico_irq_gpio_platform_data,
	},
};

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t amico_powerfail_stop(int irq, void *dev_id);

static irqreturn_t amico_powerfail_quick_check_start(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(IRQ_DM365_GPIO0_2);
	desc->chip->unmask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t amico_powerfail_start(int irq, void *dev_id)
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

static irqreturn_t amico_powerfail_quick_check_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t amico_powerfail_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);
	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_HANDLED;
}

enum amico_pwrfail_prio {
	AMICO_PWR_FAIL_PRIO_0,
	AMICO_PWR_FAIL_MIN_PRIO = AMICO_PWR_FAIL_PRIO_0,
	AMICO_PWR_FAIL_PRIO_1,
	AMICO_PWR_FAIL_PRIO_2,
	AMICO_PWR_FAIL_PRIO_3,
	AMICO_PWR_FAIL_MAX_PRIO = AMICO_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item amico_pm_loss_policy_items[] = {
	{
		.bus_name = "mmc",
		.bus_priority = AMICO_PWR_FAIL_PRIO_1,
	},
	{
		.bus_name = "platform",
		.bus_priority = AMICO_PWR_FAIL_PRIO_2,
	},
};

#define AMICO_POLICY_NAME "amico-default"

struct pm_loss_default_policy_table amico_pm_loss_policy_table = {
	.name = AMICO_POLICY_NAME,
	.items = amico_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(amico_pm_loss_policy_items),
};

static void amico_powerfail_configure(struct work_struct *work)
{
	int stat;
	struct pm_loss_policy *p;
	struct irq_desc *desc;

	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    amico_powerfail_quick_check_start,
				    amico_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    amico_powerfail_quick_check_stop,
				    amico_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	desc = irq_to_desc(IRQ_DM365_GPIO0_0);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	p = pm_loss_setup_default_policy(&amico_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(AMICO_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       AMICO_POLICY_NAME);

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
#define amico_powerfail_configure()
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
 *	amico_gpio_configure
 */
static void amico_gpio_configure(void)
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
	gpio_configure_in(DM365_GPIO64_57, piPENIRQn, "piPENIRQn");

	gpio_configure_in(DM365_GPIO96, piSD_DETECTn, "SD detec");
	gpio_configure_in(DM365_GPIO64_57, piTOUCH_BUSY, "piTOUCH_BUSY");

	/* -- Configure Output -----------------------------------------------*/
	gpio_configure_out(DM365_GPIO41, poAUDIO_SEP_ZL, 0, "AUDIO SEP ZALINK");
	gpio_configure_out(DM365_GPIO36, poNAND_WPn, 1,
			"po_NAND_WriteProtect_n,");
	gpio_configure_out(DM365_GPIO43, poEN_LOCAL_MIC, 0,
			"Local microphone control");
	gpio_configure_out(DM365_GPIO44, poENET_RESETn, 1, "poENET_RESETn");
	gpio_configure_out(DM365_GPIO45, poBOOT_FL_WPn, 1,
			"Protect SPI ChipSelect");
	gpio_configure_out(DM365_GPIO64_57, poEN_LCD_3_3V, 1,
			"Enable LCD 3.3V");
	gpio_configure_out(DM365_GPIO64_57, poENABLE_VIDEO_IN, 1,
			"Enable video demodulator");
	gpio_configure_out(DM365_GPIO64_57, poAUDIO_DM365_ZL, 0,
			"AUDIO DM365 ZALINK");
	gpio_configure_out(DM365_GPIO64_57, poE2_WPn, 0,
			"EEprom write protect");

	/* choose tvp5151 initiation first */
	gpio_configure_out(DM365_GPIO78_73, poSN74CBT_S1, 1,
				"SN74CBT16214 switch video channel");
	/*poSN74CBT_S0:0:tvp5151, 1:ov971x*/
	gpio_configure_out(DM365_GPIO78_73, poSN74CBT_S0, 0,
				"SN74CBT16214 switch video channel");

	/* tvp5150: 0 for powerdown, ov971x: 1 for powerdown */
	gpio_configure_out(DM365_GPIO39, poPDEC_PWRDNn, 1,
			"Pal-Decoder power down");
	gpio_configure_out(DM365_GPIO38, poPDEC_RESETn, 0,	/* RESET */
			"PAL decoder Reset");

	gpio_configure_out(DM365_GPIO78_73, poSPK_PWR, 0,
			"Speaker power control");
	gpio_configure_out(DM365_GPIO78_73, poEN_LCD_5V, 1, "Enable LCD 5V");
	gpio_configure_out(DM365_GPIO78_73, poLCD_GAMMA_CTRL, 1,
			"Lcd gamma control");
	gpio_configure_out(DM365_GPIO78_73, poEN_BACKLIGHT, 1,
			"Lcd backlight control");
	gpio_configure_out(DM365_GPIO51, poTOUCH_CSn, 1,
			"Touch screen chip select");

	gpio_configure_out(DM365_GPIO40, poZARLINK_PWR, 0,
			"Zarlink power enable");
	davinci_cfg_reg(DM365_GPIO78_73);/*ZL_RESET: Configure into the driver*/
	davinci_cfg_reg(DM365_GPIO101);  /*ZL_CS: Configure into the driver*/
	gpio_configure_out(DM365_GPIO99, poPIC_RESETn, 1,
			"PIC AV and PIC AI reset");
	gpio_configure_out(DM365_GPIO98, poWATCHDOG, 0,
			"Emulate feed watchdog, output to mcu");

	/* -- Export For Debug -----------------------------------------------*/
	if (amico_debug) {
		gpio_export(piPOWER_FAILn, 0);
		gpio_export(piINT_I2Cn, 0);
		gpio_export(piTMK_INTn, 0);
		gpio_export(piPENIRQn, 0);
		gpio_export(piSD_DETECTn, 0);
		gpio_export(piTOUCH_BUSY, 0);

		gpio_export(poAUDIO_SEP_ZL, 0);
		gpio_export(poNAND_WPn, 0);
		gpio_export(poEN_LOCAL_MIC, 0);
		gpio_export(poENET_RESETn, 0);
		gpio_export(poBOOT_FL_WPn, 0); /* danger */
		gpio_export(poEN_LCD_3_3V, 0);
		gpio_export(poENABLE_VIDEO_IN, 0);
		gpio_export(poAUDIO_DM365_ZL, 0);
		gpio_export(poE2_WPn, 0);
		gpio_export(poPDEC_PWRDNn, 0);
		gpio_export(poPDEC_RESETn, 0);
		gpio_export(poSPK_PWR, 0);
		gpio_export(poLCD_GAMMA_CTRL, 0);
		gpio_export(poEN_BACKLIGHT, 0);
		gpio_export(poEN_LCD_5V, 0);
		gpio_export(poSN74CBT_S1, 0);
		gpio_export(poSN74CBT_S0, 0);
		gpio_export(poPIC_RESETn, 0);
		gpio_export(poWATCHDOG, 0);
		gpio_export(poZARLINK_PWR, 0);
	}
}

static int amico_mmc_get_ro(int module)
{
	/* high == card's write protect switch active*/
	return 0;
}

static int amico_mmc1_get_cd(int module)
{
	/* low == card present*/
	return gpio_get_value(piSD_DETECTn);
}

static struct davinci_mmc_config amico_mmc_config[] = {
	{
		/* .get_cd is not defined since it seems it useless... the MMC
		* controller detects anyway the card! O_o
		*/
		.get_ro		= amico_mmc_get_ro,
		.wires		= 4,
		.max_freq	= 50000000,
		.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
		.version	= MMC_CTLR_VERSION_2,
	}, {
		/*.get_cd		= amico_mmc1_get_cd,*/
		.get_ro		= amico_mmc_get_ro,
		.wires		= 4,
		.max_freq	= 50000000,
		.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
		.version	= MMC_CTLR_VERSION_2,
	},
};

static void amico_mmc0_configure(void)
{
	/* MMC/SD 0 */
	davinci_cfg_reg(DM365_MMCSD0);

	davinci_setup_mmc(0, &amico_mmc_config[0]);

	return;
}

static void amico_usb_configure(void)
{
	pr_notice("Launching setup_usb\n");
	setup_usb(500, 8);
}

/* sound power control,power manage */
void amico_en_audio_power(int value)
{
	gpio_set_value(poSPK_PWR, value);
	gpio_set_value(poEN_LOCAL_MIC, value);
}

static struct amico_asoc_platform_data amico_asoc_info = {
	.ext_circuit_power = amico_en_audio_power,
};

static struct platform_device amico_asoc_device[] = {
	{
		.name = "amico-i-asoc", /*Internal Voice codec + Zarlink*/
		.id = 0,
		.dev = {
			.platform_data  = &amico_asoc_info,
		},
	},
};

static struct snd_platform_data dm365_amico_snd_data[] = {
		{
			.asp_chan_q = EVENTQ_3,
			.ram_chan_q = EVENTQ_2,
		},
#if 0
		{
			.asp_chan_q = EVENTQ_3,
			.i2s_accurate_sck = 1,
			.clk_input_pin = MCBSP_CLKS,
		},
#endif
};

/* gpio info for i2c_irq module */
static struct i2c_irq_platform_data i2c_irq_info = {
	.gpio = piINT_I2Cn,
	.irq = IRQ_DM365_GPIO0_1,
};

/* I2C 7bit Adr */
static struct i2c_board_info __initdata amico_i2c_info[] = {
	{	/* RTC */
		I2C_BOARD_INFO("pcf8563", 0x51),
		.irq = IRQ_DM365_GPIO0_4,
	},
	{	/* EEprom */
		I2C_BOARD_INFO("24c256", 0x53),
		.platform_data = &at24_info,
	},
	{	/* SDA PIC */
		I2C_BOARD_INFO("pic16f1939", 0x57),
		.platform_data = &i2c_irq_info,
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq = 400 /* kHz */ ,
	.bus_delay = 0 /* usec */ ,
};

static void __init amico_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, amico_i2c_info, ARRAY_SIZE(amico_i2c_info));
}

static struct platform_device *amico_devices[] __initdata = {
	&davinci_nand_device,
	&amico_asoc_device[0],
	&amico_hwmon_device,
	&amico_irq_gpio_device,
	&davinci_fb_device,
	&amico_bl_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1),
};

static void __init amico_map_io(void)
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

static struct spi_board_info amico_spi_info[] __initconst = {
#ifdef ENABLING_SPI_FLASH
	/*
	 * DANGEROUS, because spi flash contains bubl bootloader
	 */
	{
		.modalias       = "at25",
		.platform_data  = &at25640,
		.max_speed_hz   = 20 * 1000 * 1000, /* at 3v3 */
		.bus_num        = 0,
		.chip_select    = 0,
		.mode           = SPI_MODE_0,
	},
#endif
};

static void amico_late_init(unsigned long data)
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

	INIT_WORK(&late_init_work, amico_powerfail_configure);
	schedule_work(&late_init_work);

	/* uart for expansion */
	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
}

static __init void amico_init(void)
{

	pr_warning("Board_Init: START\n");
	if (amico_debug > 1)
		pinmux_check();

	amico_gpio_configure();
	amico_tvp510x_reset();
	/* amico_i choose tvp5151 work default*/
	amico_switch_video_channel(0);

	/* 2 usart for pic */
	davinci_serial_init(&uart_config);
	mdelay(1);

	/* I2C */
	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	amico_init_i2c();

	amico_emac_configure();
	amico_lcd_configure();

	/* SPI3 for touchscreen and Zarlink*/
	dm365_init_spi3(0, amico_spi3_info, ARRAY_SIZE(amico_spi3_info));

	amico_mmc0_configure();
	amico_usb_configure();

	/* output 24Mhz for ov971x mclk*/
	dm365_clkout2_set_rate(OV971X_XCLK * 1000 * 1000);
	davinci_cfg_reg(DM365_CLKOUT2);

	dm365_init_vc(&dm365_amico_snd_data[0]);
	/* dm365_init_asp(&dm365_dingo_snd_data[1]);*/
	platform_add_devices(amico_devices, ARRAY_SIZE(amico_devices));

	dm365_init_rtc();
	dm365_init_adc(&amico_adc_data);

	init_timer(&startup_timer);
	startup_timer.function = amico_late_init;
	startup_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_LATE_INIT);
	add_timer(&startup_timer);

	if (amico_debug)
		pinmux_check();
	pr_warning("Board_Init: DONE\n");
}

MACHINE_START(AMICO_I, "Shidean amico-i board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = amico_map_io,
	.init_irq = davinci_irq_init,
	.timer = &davinci_timer,
	.init_machine = amico_init,
MACHINE_END

/* End Of File -------------------------------------------------------------- */
