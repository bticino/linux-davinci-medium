/*
 * BTicino S.p.A. jumbo platform support based on evm-dm365 board
 *
 * Simone Cianni, Davide Bonfanti
 * Copyright (C) 2013 , BTicino S.p.A.
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
#include <linux/pm_loss.h>
#include <linux/irq_gpio.h>

#include <media/soc_camera.h>
#include <sound/davinci_jumbo_asoc.h>

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

#include <mach/jumbo-d.h>
#include <mach/aemif.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define JUMBO_PHY_MASK              (0x2)
#define JUMBO_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
#define HW_IN_CLOCKOUT2_UDA_I2S

#define NAND_BLOCK_SIZE         SZ_128K

#define DAVINCI_BOARD_MAX_NR_UARTS 3

struct work_struct late_init_work;
static struct timer_list startup_timer;
#define TIME_TO_LATE_INIT 1500

static int jumbo_debug = 1;
module_param(jumbo_debug, int, 0644);
MODULE_PARM_DESC(jumbo_debug, "Debug level 0-1");

extern unsigned int system_rev;
extern int lookup_resistors(int cnt);

/* wp_set: set/unset the at24 eeprom write protect */
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

static struct mtd_partition jumbo_nand_partitions[] = {
	{
		/* U-Boot */
		.name           = "u-boot",
		.offset         = 0,
		.size           = 12 * NAND_BLOCK_SIZE, /* 1,536 MByte*/
		.mask_flags     = MTD_WRITEABLE, /* force read-only */
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
		.size           = SZ_128M + 10 * SZ_1M, /* 138 MByte */
		.mask_flags     = 0,
	}, {	/* Configurations and extras */
		.name           = "extras",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL, /* Circa 66 MByte */
		.mask_flags     = 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct davinci_aemif_timing jumbo_nandflash_timing = {
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
	.parts		= jumbo_nand_partitions,
	.nr_parts	= ARRAY_SIZE(jumbo_nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.options	= NAND_USE_FLASH_BBT,
	.ecc_bits	= 4,
	.timing		= &jumbo_nandflash_timing,
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

void jumbo_tvp510x_reset(void)
{
	printk(KERN_DEBUG "Reset tvp510x PalDecoder \n");

	/* Active-low reset. RESETB can be used only when PDN = 1 */
	gpio_direction_output(poPDEC_PWRDNn, 1);
	mdelay(1);

	gpio_direction_output(poPDEC_RESETn, 0);
	mdelay(1); /* 500 nsec is enough */
	gpio_direction_output(poPDEC_RESETn, 1);
}

/* [VideoIn] TVP5150 : Pal Decoder */
static struct tvp5150_platform_data tvp5150_pdata = {
	.pdn = poPDEC_PWRDNn,
	.resetb = &jumbo_tvp510x_reset,
};

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

/* Inputs available at the TVP5150 */
static struct v4l2_input tvp5151_inputs[] = {
	{
		.index = 0,
		.name = "SCS Composite",
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
		.board_info = {	/* Video Demodulator */
			I2C_BOARD_INFO("tda9885", 0x43),
			.platform_data = &tda9885_defaults,
		},
	},	{
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

/* Set : video_input */
static int jumbo_setup_video_input(enum vpfe_subdev_id id)
{
	/*Nothing todo*/
	return 0;
}

static struct vpfe_config vpfe_cfg = {
	.setup_input = jumbo_setup_video_input,
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "DM365 JUMBO",
	.ccdc = "DM365 ISIF",
	.num_clocks = 1,
	.clocks = {"vpss_master"},
};

void jumbo_phy_power(int on)
{
	printk(KERN_DEBUG "Reset eth controller \n");
	gpio_set_value(poENET_RESETn, !on);
	mdelay(2); /* Measured by oscilloscope : must be at least 1,6ms */
	gpio_set_value(poENET_RESETn, on);
}

static void jumbo_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = JUMBO_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = JUMBO_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = jumbo_phy_power;

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

static struct davinci_adc_platform_data jumbo_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device jumbo_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &jumbo_adc_data,
		},
	.id = 0,
};

static struct gpio_led jumbo_gpio_led[] = {
	{
		.name                   = "LED_SYSTEM",
		.gpio                   = poPOWER_LED_YELLOWn,
		.active_low             = 1,
		.default_trigger        = "none",
		.default_state          = LEDS_GPIO_DEFSTATE_ON,
	},
};

static struct gpio_led_platform_data jumbo_gpio_led_info = {
	.num_leds = 1,
	.leds = jumbo_gpio_led,
};

static struct platform_device jumbo_leds_gpio_device = {
	.name = "leds-gpio",
	.dev = {
		.platform_data = &jumbo_gpio_led_info,
		},
	.id = 0,
};

static void __init jumbo_led_init(void)
{
	davinci_cfg_reg(DM365_GPIO82);
	platform_device_register(&jumbo_leds_gpio_device);
}

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
		printk(KERN_DEBUG "pinmux%d=%X\n", i, pinmux[i]);
	}
}

static struct plat_serial8250_port jumbo_dm365_serial_platform_data[] = {
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

static struct platform_device jumbo_dm365_serial_device = {
	.name                   = "serial8250",
	.id                     = PLAT8250_DEV_PLATFORM1,
	.resource               = davinci_asyn_resources,
	.dev                    = {
		.platform_data  = jumbo_dm365_serial_platform_data,
	},
};

static void jumbo_uart_configure(void)
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
	platform_device_register(&jumbo_dm365_serial_device);
}

static struct irq_on_gpio jumbo_irq_on_gpio0 [] = {
	{
		.gpio = piPOWER_FAILn,
		.irq = IRQ_DM365_GPIO0_0,
		.type = EDGE,
		.mode = GPIO_EDGE_RISING,
	}, {
		.gpio = piINT_UART_Bn,
		.irq = IRQ_DM365_GPIO0_1,
		.type = LEVEL,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piPOWER_FAILn,
		.irq = IRQ_DM365_GPIO0_2,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piINT_UART_An,
		.irq = IRQ_DM365_GPIO0_3,
		.type = LEVEL,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piTMK_INTn,
		.irq = IRQ_DM365_GPIO0_4,
		.type = LEVEL,
		.mode = GPIO_EDGE_FALLING,
	}, {
		.gpio = piGPIO_INTn,
		.irq = IRQ_DM365_GPIO0_5,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	},
};

static struct irq_gpio_platform_data jumbo_irq_gpio_platform_data = {
	.gpio_list = jumbo_irq_on_gpio0,
	.len = ARRAY_SIZE(jumbo_irq_on_gpio0),
	.gpio_common = 0,
	.irq_gpio = IRQ_DM365_GPIO0,
};

static struct platform_device jumbo_irq_gpio_device = {
	.name			= "irq_gpio",
	.id			= 0,
	.num_resources		= 0,
	.resource		= NULL,
	.dev			= {
		.platform_data	= &jumbo_irq_gpio_platform_data,
	},
};

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t jumbo_powerfail_stop(int irq, void *dev_id);

static irqreturn_t jumbo_powerfail_quick_check_start(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(IRQ_DM365_GPIO0_2);
	desc->chip->unmask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t jumbo_powerfail_start(int irq, void *dev_id)
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

static irqreturn_t jumbo_powerfail_quick_check_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t jumbo_powerfail_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);
	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_HANDLED;
}

enum jumbo_pwrfail_prio {
	JUMBO_PWR_FAIL_PRIO_0,
	JUMBO_PWR_FAIL_MIN_PRIO = JUMBO_PWR_FAIL_PRIO_0,
	JUMBO_PWR_FAIL_PRIO_1,
	JUMBO_PWR_FAIL_PRIO_2,
	JUMBO_PWR_FAIL_PRIO_3,
	JUMBO_PWR_FAIL_MAX_PRIO = JUMBO_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item jumbo_pm_loss_policy_items[] = {
	{
		.bus_name = "mmc",
		.bus_priority = JUMBO_PWR_FAIL_PRIO_1,
	},
	{
		.bus_name = "platform",
		.bus_priority = JUMBO_PWR_FAIL_PRIO_2,
	},
};

#define JUMBO_POLICY_NAME "jumbo-default"

struct pm_loss_default_policy_table jumbo_pm_loss_policy_table = {
	.name = JUMBO_POLICY_NAME,
	.items = jumbo_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(jumbo_pm_loss_policy_items),
};

static void jumbo_powerfail_configure(struct work_struct *work)
{
	int stat;
	struct pm_loss_policy *p;
	struct irq_desc *desc;

	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    jumbo_powerfail_quick_check_start,
				    jumbo_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    jumbo_powerfail_quick_check_stop,
				    jumbo_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	desc = irq_to_desc(IRQ_DM365_GPIO0_0);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	p = pm_loss_setup_default_policy(&jumbo_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(JUMBO_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       JUMBO_POLICY_NAME);

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
#define jumbo_powerfail_configure()
#endif

/* gpio_configure in/out */
int inline gpio_configure_out (int DM365_Pin,int Name,int DefVualue, char* str)
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

int inline gpio_configure_in (int DM365_Pin,int Name, char* str)
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

static void jumbo_gpio_configure(void)
{
	void __iomem *pupdctl0 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x78);
	void __iomem *pupdctl1 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x7c);

	/* Configure (disable) pull down control */
	__raw_writel(0, pupdctl0);
	__raw_writel(0x40000, pupdctl1); /* EM_WAIT active pull-up */

	/*  -- Configure Digital Board--------------------------------------- */
	dm365_setup_debounce(1, 0, 2048);
	gpio_request(0, "GIO0"); /* pi_INTERRUPT */
	gpio_direction_input(0);
	gpio_configure_in(DM365_GPIO50, piPOWER_FAILn, "piPOWER_FAILn");
	gpio_configure_in(DM365_GPIO64_57, piINT_UART_An, "piINT_UART_An");
	gpio_configure_in(DM365_GPIO64_57, piTMK_INTn, "piTMK_INTn (RTC)");
	gpio_configure_in(DM365_GPIO26, piINT_UART_Bn, "piINT_UART_Bn");
	gpio_configure_in(DM365_GPIO101, piOCn, "Over Current VBUS");
	gpio_configure_in(DM365_GPIO96, piSD_DETECTn, "SD detec");
	gpio_configure_in(DM365_GPIO28, piRESET_CONF, "piRESET_CONF");

	gpio_configure_out(DM365_GPIO44, poENET_RESETn, 1, "poENET_RESETn");
	gpio_configure_out(DM365_GPIO64_57, poEMMC_RESETn, 1, "eMMC reset(n)");
	gpio_configure_out(DM365_GPIO90, po_AUDIO_RESET, 1, "po_AUDIO_RESET");
	gpio_configure_out(DM365_GPIO91, po_AUDIO_DEEMP, 0, "po_AUDIO_DEEMP");
	gpio_configure_out(DM365_GPIO92, po_AUDIO_MUTE, 0, "po_AUDIO_MUTE");
	gpio_configure_out(DM365_GPIO51, poRES_EXTUART, 0, "poRES_EXT_UART");
	gpio_configure_out(DM365_GPIO45, poBOOT_FL_WPn, 1, "Protect SPI CS");
	gpio_configure_out(DM365_GPIO80, poE2_WPn, 0, "EEprom write protect");
	gpio_configure_out(DM365_GPIO99, poPIC_RESETn, 1, "Reset PIC AV & AI");
	gpio_configure_out(DM365_GPIO86, po_NAND_WPn, 1, "po_NAND_WProtect_n,");
	gpio_configure_out(DM365_GPIO33, po_EN_SW_USB, 0, "po_EN_SW_USB");
	gpio_configure_out(DM365_GPIO27, po_DISCHARGE, 0,
		"Discharge for Configuration Recovery");
	gpio_configure_out(DM365_GPIO103, poPDEC_PWRDNn, 1, "PalDec nPDown");
	gpio_configure_out(DM365_GPIO102, poPDEC_RESETn, 0, "PAL decoder nRST");

	/* -- Scheda Base ----------------------------------------------------*/
	gpio_configure_in(DM365_GPIO100, piGPIO_INTn, "piGPIO_INTn");

	gpio_configure_out(DM365_GPIO68, po_EN_FONICA, 1, "po_EN_FONICA");
	davinci_cfg_reg(DM365_GPIO64_57); /* ZL_CS: Configure into the driver */
	davinci_cfg_reg(DM365_GPIO72); /* ZL_RESET: Configure into the driver */

	gpio_configure_out(DM365_GPIO64_57, po_ENABLE_VIDEO_OUT, 0,
		"EN_VIDEO_OUT");

	/* INTERF 2F/IP */
	gpio_configure_out(DM365_GPIO64_57, poRL_VIDEO, 0, /* EN_HFREQ */
		"Enable Rele' Video SCS");

	/* INTERF D45/IP */
	gpio_configure_out(DM365_GPIO88, pi_GPIO_2, 0, "Rele1");
	gpio_configure_out(DM365_GPIO89, pi_GPIO_1, 0, "Rele2");
	gpio_configure_out(DM365_GPIO64_57, poENABLE_VIDEO_IN, 0,
		"Rele3 + EN PowerVideo IN");

	/* -- Export For Debug -----------------------------------------------*/

	if (jumbo_debug) {
		gpio_export(piPOWER_FAILn, 0);
		gpio_export(piINT_UART_An, 0);
		gpio_export(piTMK_INTn, 0);
		gpio_export(piRESET_CONF, 0);
		gpio_export(poRL_VIDEO, 0);
		gpio_export(poENABLE_VIDEO_IN, 0);
		gpio_export(poEMMC_RESETn, 0);
		gpio_export(piINT_UART_Bn, 0);
		gpio_export(piGPIO_INTn, 0);
		gpio_export(poENET_RESETn, 0);
		gpio_export(poRES_EXTUART, 0);
		gpio_export(poBOOT_FL_WPn, 0); /* danger */
		gpio_export(poPDEC_PWRDNn, 0);
		gpio_export(poPDEC_RESETn, 0);
		gpio_export(poE2_WPn, 0);
		gpio_export(poPIC_RESETn, 0);
		gpio_export(piOCn, 0);
		gpio_export(piSD_DETECTn, 0);
		gpio_export(po_ENABLE_VIDEO_OUT, 0);
		gpio_export(po_EN_FONICA, 0);
		gpio_export(po_NAND_WPn, 0);
		gpio_export(pi_GPIO_2, 0);
		gpio_export(pi_GPIO_1, 0);
		gpio_export(po_AUDIO_RESET, 0);
		gpio_export(po_AUDIO_DEEMP, 0);
		gpio_export(po_AUDIO_MUTE, 0);
		gpio_export(po_EN_SW_USB, 0);
		gpio_export(po_DISCHARGE, 0);
	}
}

static void jumbo_usb_configure(void)
{
	pr_notice("Launching setup_usb\n");
	setup_usb(500, 8);
}

void inline jumbo_en_audio_power(int value)
{
	gpio_set_value(po_EN_FONICA, value);
}

static struct jumbo_asoc_platform_data jumbo_asoc_info = {
	.ext_circuit_power = jumbo_en_audio_power,
};

static struct platform_device jumbo_asoc_device[] = {
	{
		.name = "jumbo-i-asoc", /*Internal Voice codec + Zarlink*/
		.id = 0,
		.dev = {
			.platform_data  = &jumbo_asoc_info,
		},
	},
};

static struct snd_platform_data dm365_jumbo_snd_data[] = {
	{
		.asp_chan_q = EVENTQ_3,
		.ram_chan_q = EVENTQ_2,
	},
};

/* I2C 7bit Adr */
static struct i2c_board_info __initdata jumbo_i2c_info[] = {
	{	/* RTC */
		I2C_BOARD_INFO("pcf8563", 0x51),
		.irq = IRQ_DM365_GPIO0_4,
	}, {	/* EEprom */
		I2C_BOARD_INFO("24c256", 0x53),
		.platform_data = &at24_info,
	}, {	/*Video Modulator*/
		I2C_BOARD_INFO("mc44cc373", 0x65),
		.platform_data = &mc44cc373_pdata,
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq = 400 /* kHz */ ,
	.bus_delay = 0 /* usec */ ,
};

static void __init jumbo_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, jumbo_i2c_info, ARRAY_SIZE(jumbo_i2c_info));
}

static struct platform_device *jumbo_devices[] __initdata = {
	&davinci_nand_device,
	&jumbo_asoc_device[0],
	&jumbo_hwmon_device,
	&jumbo_irq_gpio_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3),
};

static void __init jumbo_map_io(void)
{
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init_isif(DM365_ISIF_8BIT);
	dm365_init();
}

#ifdef  ENABLING_SPI_FLASH
static struct spi_eeprom at25640 = {
	.byte_len = SZ_64K / 8,
	.name = "at25640",
	.page_size = 32,
	.flags = EE_ADDR2,
};
#endif

static struct zl38005_platform_data zl_config[] = {
	{
		.minor = 0,
		.reset_gpio = poZARLINK_RESET,
	},
};

static struct spi_board_info jumbo_spi_info[] __initconst = {
#ifdef  ENABLING_SPI_FLASH
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
	{	/* dm365 spi work between 600000 and 50000000 */
		.modalias       = "zl38005",
		.max_speed_hz   = 2 * 1000 * 1000,
		.bus_num        = 0,
		.controller_data = poZARLINK_CS,
		/* .chip_select	= poZARLINK_CS, cosi non va... */
		.mode           = SPI_MODE_0,
		.platform_data  = &zl_config,
	},

};

static void jumbo_late_init(unsigned long data)
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

	INIT_WORK(&late_init_work, jumbo_powerfail_configure);
	schedule_work(&late_init_work);

	/* uart for expansion */
	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
}

static __init void jumbo_init(void)
{
	pr_warning("Board_Init: START\n");
	if (jumbo_debug>1)
		pinmux_check();

	jumbo_gpio_configure();
	jumbo_tvp510x_reset();
	jumbo_led_init();

	/* 2 usart for pic */
	davinci_serial_init(&uart_config);
	mdelay(1);

	/* I2C */
	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	jumbo_init_i2c();

	jumbo_emac_configure();

	/* SPI for Zarlink*/
	dm365_init_spi0(0, jumbo_spi_info, ARRAY_SIZE(jumbo_spi_info));

	jumbo_usb_configure();
	jumbo_uart_configure();

	dm365_init_vc(&dm365_jumbo_snd_data[0]);
	platform_add_devices(jumbo_devices, ARRAY_SIZE(jumbo_devices));

	dm365_init_rtc();
	dm365_init_adc(&jumbo_adc_data);

	init_timer(&startup_timer);
	startup_timer.function = jumbo_late_init;
	startup_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_LATE_INIT);
	add_timer(&startup_timer);

	if (jumbo_debug)
		pinmux_check();
	pr_warning("Board_Init: DONE\n");
}

MACHINE_START(JUMBO_I, "BTicino Jumbo-i board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = jumbo_map_io,
	.init_irq = davinci_irq_init,
	.timer = &davinci_timer,
	.init_machine = jumbo_init,
MACHINE_END

/* End Of File -------------------------------------------------------------- */
