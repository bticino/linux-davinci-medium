/*
 * BTicino S.p.A. jumbo platform support
 * based on evm-dm365 board
 *
 * Copyright (C) 2012  BTicino S.p.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/i2c/at24.h>
#include <linux/leds.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/serial_8250.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/pm_loss.h>

#include <media/soc_camera.h>

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
#include <mach/keyscan.h>
#include <mach/gpio.h>
#include <mach/usb.h>
#include <linux/videodev2.h>
#include <media/soc_camera.h>
#include <media/tvp5150.h>
#include <linux/i2c/tda9885.h>
#include <linux/i2c/tvp5150.h>
#include <mach/aemif.h>
#include <sound/davinci_jumbo_asoc.h>

#include <mach/jumbo-d.h>
#include <mach/aemif.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define JUMBO_PHY_MASK              (0x2)
#define JUMBO_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
#define HW_IN_CLOCKOUT2_UDA_I2S

#define DAVINCI_BOARD_MAX_NR_UARTS 3

static int jumbo_debug = 1;
module_param(jumbo_debug, int, 0644);
MODULE_PARM_DESC(jumbo_debug, "Debug level 0-1");

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

static struct tda9885_platform_data tda9885_defaults = {
	.switching_mode = 0xf2,
	.adjust_mode = 0xd0,
	.data_mode = 0x0b,
	.power = poENABLE_VIDEO_IN,
};

static struct tvp5150_platform_data tvp5150_pdata = {
	.pdn = poPDEC_PWRDNn,
	.resetb = poPDEC_RESETn,
};

/* Inputs available at the TVP5146 */
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

/* Inputs available at the TDA9885 */
static struct v4l2_input tda9885_inputs[] = {
	{
		.index = 0,
		.name = "SCS Modulated video",
		.type = V4L2_INPUT_TYPE_CAMERA,
		.std = V4L2_STD_PAL,
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
		.board_info = {
			I2C_BOARD_INFO("tvp5150", 0x5d),
			.platform_data = &tvp5150_pdata,
		},
	},
	{
		.module_name = "tda9885",
		.grp_id = VPFE_SUBDEV_TVP5150,
		.num_inputs = 1,
		.inputs = tda9885_inputs,
		.can_route = 1,
		.board_info = {
			I2C_BOARD_INFO("tda9885", 0x43),
			.platform_data = &tda9885_defaults,
		},
	},

};

static int jumbo_setup_video_input(enum vpfe_subdev_id id)
{
	/*TODO ?*/
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
	gpio_set_value(poENET_RESETn, on);
	udelay(100);
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
	.leds = &jumbo_gpio_led,
	.num_leds = 1,
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
		printk("pinmux%d=%X\n",i,pinmux[i]);
	}
}

static struct plat_serial8250_port jumbo_dm365_serial_platform_data[] = {
	{
		.mapbase        = DM365_ASYNC_EMIF_DATA_CE1_BASE,
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

static void jumbo_uart2_configure(void)
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
	gpio_direction_output(poRES_EXTUART, 0);

	set_irq_type(gpio_to_irq(0), IRQ_TYPE_EDGE_BOTH);
	platform_device_register(&jumbo_dm365_serial_device);
}

static struct irq_on_gpio0 {
	unsigned int gpio;
	unsigned int irq;
};

static struct irq_on_gpio0 jumbo_irq_on_gpio0 [] = {
	{
		.gpio = piINT_UART_Bn,
		.irq = IRQ_DM365_GPIO0_1,
	}, {
		.gpio = piPOWER_FAILn,
		.irq = IRQ_DM365_GPIO0_2,
	}, {
                .gpio = piINT_UART_An,
                .irq = IRQ_DM365_GPIO0_3,
        }, {
                .gpio = piTMK_INTn,
                .irq = IRQ_DM365_GPIO0_4,
        }, {
                .gpio = piGPIO_INTn,
                .irq = IRQ_DM365_GPIO0_5,
        },
};

static unsigned long jumbo_gpio0_irq_enabled;

static void jumbo_mask_irq_gpio0(unsigned int irq)
{
	int jumbo_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	jumbo_gpio0_irq_enabled &= ~(1 << jumbo_gpio_irq);
}

static void jumbo_unmask_irq_gpio0(unsigned int irq)
{
	int jumbo_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	jumbo_gpio0_irq_enabled |= (1 << jumbo_gpio_irq);
}

static struct irq_chip jumbo_gpio0_irq_chip = {
	.name           = "DM365_GPIO0",
	.ack            = jumbo_mask_irq_gpio0,
	.mask           = jumbo_mask_irq_gpio0,
	.unmask         = jumbo_unmask_irq_gpio0,
};

static void jumbo_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cnt;

	if ((jumbo_gpio0_irq_enabled & 0x01) && (gpio_get_value(0))) {
		/* enabled interrupt when lines are all high */
		generic_handle_irq(IRQ_DM365_GPIO0_0);
	}
	else if (!gpio_get_value(0)) {
		for (cnt = 0; cnt < ARRAY_SIZE(jumbo_irq_on_gpio0); cnt++) {
			if (!gpio_get_value(jumbo_irq_on_gpio0[cnt].gpio)
			&& (jumbo_gpio0_irq_enabled &
		    (1<<(jumbo_irq_on_gpio0[cnt].irq - IRQ_DM365_GPIO0_0)))){
			generic_handle_irq(jumbo_irq_on_gpio0[cnt].irq);
			}
		}
	}
	desc->chip->ack(irq);
}

static void __init jumbo_gpio_init_irq(void)
{
	int irq;

	davinci_irq_init();
	/* setup extra Jumbo irqs on GPIO0*/
	for(irq = IRQ_DM365_GPIO0_0; irq <= IRQ_DM365_GPIO0_5; irq++) {
		set_irq_chip(irq, &jumbo_gpio0_irq_chip);
		set_irq_handler(irq, handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(IRQ_DM365_GPIO0, IRQ_TYPE_EDGE_BOTH);
	set_irq_chained_handler(IRQ_DM365_GPIO0, jumbo_gpio_irq_handler);
}

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t jumbo_powerfail_stop(int irq, void *dev_id);

static irqreturn_t jumbo_powerfail_quick_check_start(int irq, void *dev_id)
{
	jumbo_mask_irq_gpio0(IRQ_DM365_GPIO0_2);
	jumbo_unmask_irq_gpio0(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t jumbo_powerfail_start(int irq, void *dev_id)
{
	if (powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 1;
	pm_loss_power_changed(SYS_PWR_FAILING);
	return IRQ_HANDLED;
}

static irqreturn_t jumbo_powerfail_quick_check_stop(int irq, void *dev_id)
{
	jumbo_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	jumbo_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t jumbo_powerfail_stop(int irq, void *dev_id)
{
	if (!powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);
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

static void jumbo_powerfail_configure(void)
{
	int stat;
	struct pm_loss_policy *p;
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
	jumbo_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	p = pm_loss_setup_default_policy(&jumbo_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(JUMBO_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       JUMBO_POLICY_NAME);
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

/*
 *	Macro : gpio_configure_out
 */
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

/*
 *	Macro : gpio_configure_in
 */
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

/*
 *	jumbo_gpio_configure
 */
static void jumbo_gpio_configure(void)
{
	int status;
	void __iomem *pupdctl0 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x78);
	void __iomem *pupdctl1 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x7c);

	/*
	 * Configure (disable) pull down control because can give problems:
	 * for example it is needed to disable "CIN[7:0] and PCLK Pull down
	 * enable" in order to read correctly I2CSEL TVP5151 input.
	 */
	__raw_writel(0, pupdctl0);
	__raw_writel(0, pupdctl1);

	/*  -- Configure Input ---------------------------------------------- */
	gpio_request(0, "GIO0"); /* pi_INTERRUPT */
	gpio_direction_input(0);

	gpio_configure_in (DM365_GPIO50, piPOWER_FAILn, "piPOWER_FAILn");
	gpio_configure_in (DM365_GPIO64_57, piINT_UART_An, "piINT_UART_An");
	gpio_configure_in (DM365_GPIO64_57, piTMK_INTn, "piTMK_INTn");
	gpio_configure_in (DM365_GPIO26, piINT_UART_Bn, "piINT_UART_Bn");
	gpio_configure_in (DM365_GPIO100, piGPIO_INTn, "piGPIO_INTn");
	gpio_configure_in (DM365_GPIO101, piOCn, "Over Current VBUS");
	gpio_configure_in (DM365_GPIO96, piSD_DETECTn, "SD detec");
	gpio_configure_in (DM365_GPIO88, pi_GPIO_2, "pi_GPIO_2");
	gpio_configure_in (DM365_GPIO89, pi_GPIO_1, "pi_GPIO_1");

	/* -- Configure Output -----------------------------------------------*/

	gpio_configure_out (DM365_GPIO64_57, poEN_MOD_DIFF_SONORA, 0,
				"Audio modulator Enable on external connector");
	/* enabled, to allow i2c attach */
	gpio_configure_out (DM365_GPIO64_57, poENABLE_VIDEO_IN, 1,
				"Enable video demodulator");
	gpio_configure_out (DM365_GPIO64_57, poZARLINK_CS, 1,
				"Zarlink chip select");
	gpio_configure_out (DM365_GPIO64_57, poEMMC_RESETn, 1, "eMMC reset(n)");
	gpio_configure_out (DM365_GPIO44, poENET_RESETn, 1, "poENET_RESETn");
	gpio_configure_out (DM365_GPIO51, poRES_EXTUART, 0, "poRES_EXTUART");
	gpio_configure_out (DM365_GPIO45, poBOOT_FL_WPn, 1, /* TODO Verify*/
				"Protecting SPI chip select");
	gpio_configure_out (DM365_GPIO103, poPDEC_PWRDNn, 1,
				"Pal-Decoder power down");
	gpio_configure_out (DM365_GPIO102, poPDEC_RESETn, 0,
				"PAL decoder Reset");
        /*
	 * The I2CSEL tvp5151 input is sampled when its resetb input is down,
         * assigning the i2c address.
	 */
        mdelay(10);
        gpio_direction_output(poPDEC_RESETn, 1);

	gpio_configure_out (DM365_GPIO80, poE2_WPn, 0, "EEprom write protect");
	gpio_configure_out (DM365_GPIO72, poZARLINK_RESET, 0, "Zarlink reset");
	gpio_configure_out (DM365_GPIO99, poPIC_RESETn, 0,
				"PIC AV and PIC AI reset");
	gpio_configure_out (DM365_GPIO28, poRESET_CONFn, 0, "poRESET_CONFn");
	gpio_configure_out (DM365_GPIO64_57, po_ENABLE_VIDEO_OUT, 0,
				"po_ENABLE_VIDEO_OUT");
	gpio_configure_out (DM365_GPIO68, po_EN_FONICA, 0, "po_EN_FONICA");
	gpio_configure_out (DM365_GPIO86, po_NAND_WPn, 0, "po_NAND_WPn,");
	gpio_configure_out (DM365_GPIO90, po_AUDIO_RESET, 1, "po_AUDIO_RESET");
	gpio_configure_out (DM365_GPIO91, po_AUDIO_DEEMP, 0, "po_AUDIO_DEEMP");
	gpio_configure_out (DM365_GPIO92, po_AUDIO_MUTE, 0, "po_AUDIO_MUTE");
	gpio_configure_out (DM365_GPIO97, po_EN_SW_USB, 0, "po_EN_SW_USB");
	gpio_configure_out (DM365_GPIO98, po_EN_PWR_USB, 0, "po_EN_PWR_USB");

	/* -- Export For Debug -----------------------------------------------*/

	if (jumbo_debug) {
		gpio_export(piPOWER_FAILn, 0);
		gpio_export(piINT_UART_An, 0);
		gpio_export(piTMK_INTn, 0);
		gpio_export(poEN_MOD_DIFF_SONORA, 0);
		gpio_export(poENABLE_VIDEO_IN, 0);
		gpio_export(poZARLINK_CS, 0);
		gpio_export(poEMMC_RESETn, 0);
		gpio_export(piINT_UART_Bn, 0);
		gpio_export(piGPIO_INTn, 0);
		gpio_export(poENET_RESETn, 0);
		gpio_export(poRES_EXTUART, 0);
		gpio_export(poBOOT_FL_WPn, 0); /* danger */
		gpio_export(poPDEC_PWRDNn, 0);
		gpio_export(poPDEC_RESETn, 0);
		gpio_export(poE2_WPn, 0);
		gpio_export(poZARLINK_RESET, 0);
		gpio_export(poPIC_RESETn, 0);
		gpio_export(piOCn, 0);
		gpio_export(piSD_DETECTn, 0);
		gpio_export(poRESET_CONFn, 0);
		gpio_export(po_ENABLE_VIDEO_OUT, 0);
		gpio_export(po_EN_FONICA, 0);
		gpio_export(po_NAND_WPn, 0);
		gpio_export(pi_GPIO_2, 0);
		gpio_export(pi_GPIO_1, 0);
		gpio_export(po_AUDIO_RESET, 0);
		gpio_export(po_AUDIO_DEEMP, 0);
		gpio_export(po_AUDIO_MUTE, 0);
		gpio_export(po_EN_SW_USB, 0);
		gpio_export(po_EN_PWR_USB, 0);
	}
}

/*
static int jumbo_mmc_get_ro(int module)
{
	return gpio_get_value( "pin non presente" );
}

static struct davinci_mmc_config jumbo_mmc_config = {
	// .get_cd is not defined since it seems it useless... the MMC
	// controller detects anyway the card! O_o

	.get_ro		= jumbo_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void jumbo_mmc_configure(void)
{
	int ret;

	davinci_cfg_reg(DM365_MMCSD0);
	davinci_cfg_reg(DM365_GPIO43); // Not connected to the eMMC

	ret = gpio_request( pin non presente, "MMC WP");
	if (ret)
		pr_warning("jumbo: cannot request GPIO %d!\n", "pin non presente"  );
	gpio_direction_input( "pin non presente" );
	davinci_setup_mmc(0, &jumbo_mmc_config);

	return;
}
*/

static void jumbo_usb_configure(void)
{
	pr_notice("Launching setup_usb\n");
	setup_usb(500, 8);
}

void jumbo_en_audio_power(int on)
{
	gpio_set_value(poEN_MOD_DIFF_SONORA, on);
}

void jumbo_zarlink_reset(int on)
{
	gpio_set_value(poZARLINK_RESET, on);
}

static struct jumbo_asoc_platform_data jumbo_asoc_info = {
	.ext_codec_power = jumbo_zarlink_reset,
	.ext_circuit_power = jumbo_en_audio_power,
};

static struct platform_device jumbo_asoc_device[] = {
	{
		.name = "jumbo-d-asoc",
		.id = 0,
		.dev = {
			.platform_data  = &jumbo_asoc_info,
		},
	},
};

static struct i2c_board_info __initdata jumbo_i2c_info[] = {
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
	},
	{
		I2C_BOARD_INFO("24c256", 0x53),
		.platform_data = &at24_info,
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
	&jumbo_asoc_device[0],
	&jumbo_hwmon_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1) | (1 << 2),
};

static void __init jumbo_map_io(void)
{
	dm365_set_vpfe_config(&vpfe_cfg);
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

static struct spi_board_info jumbo_spi_info[] __initconst = {
	/*
	 * DANGEROUS, because spi flash contains bubl bootloader
	 */
#ifdef  ENABLING_SPI_FLASH
	{
		.modalias       = "at25",
		.platform_data  = &at25640,
		.max_speed_hz   = 20 * 1000 * 1000,     /* at 3v3 */
		.bus_num        = 0,
		.chip_select    = 0,
		.mode           = SPI_MODE_0,
	},
#endif
	{
		.modalias       = "zl38005",
		.max_speed_hz   = 2 * 1000 * 1000, /* dm365 spi work between 600000 and 50000000 */
		.bus_num        = 0,
		.controller_data = poZARLINK_CS,
		.mode           = SPI_MODE_0,
	},

};

static struct snd_platform_data dm365_jumbo_snd_data;


static __init void jumbo_init(void)
{
	if (jumbo_debug>1)
		pinmux_check();
	jumbo_gpio_configure();
//	jumbo_led_init();
	davinci_cfg_reg(DM365_UART1_RXD_34); /* pic uart */
	davinci_cfg_reg(DM365_UART1_TXD_25);
	davinci_serial_init(&uart_config);
	mdelay(1);
	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	jumbo_init_i2c();
	jumbo_emac_configure();

//	dm365_init_spi0(0, jumbo_spi_info, ARRAY_SIZE(jumbo_spi_info));

//	- funzione eliminata - jumbo_mmc_configure();
//	jumbo_usb_configure();
//	jumbo_uart2_configure();
//	jumbo_powerfail_configure();
//	dm365_init_vc(&dm365_jumbo_snd_data);
//	platform_add_devices(jumbo_devices, ARRAY_SIZE(jumbo_devices));
//	dm365_init_rtc();
//	dm365_init_adc(&jumbo_adc_data);

	if (jumbo_debug)
		pinmux_check();
}

MACHINE_START(JUMBO_D, "BTicino Jumbo-d board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = jumbo_map_io,
	.init_irq = jumbo_gpio_init_irq,
	.timer = &davinci_timer,
	.init_machine = jumbo_init,
MACHINE_END
