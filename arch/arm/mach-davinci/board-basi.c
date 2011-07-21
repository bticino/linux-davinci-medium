/*
 * BTicino S.p.A. basi platform support
 * based on evm-dm365 board
 *
 * Copyright (C) 2010 raffaele.recalcati@bticino.it
 *                    davide.bonfanti@bticino.it
 *               BTicino S.p.A.
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

#include <mach/basi.h>
#include <mach/aemif.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define BASI_PHY_MASK              (0x2)
#define BASI_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
#define HW_IN_CLOCKOUT2_UDA_I2S

#define DAVINCI_BOARD_MAX_NR_UARTS 3

static int basi_debug = 1;
module_param(basi_debug, int, 0644);
MODULE_PARM_DESC(basi_debug, "Debug level 0-1");

/*
wp_set: set/unset the at24 eeprom write protect
*/
void wp_set(int enable)
{
	gpio_direction_output(E2_WPn, enable);
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
	.power = ABIL_DEM_VIDEO,
};

static struct tvp5150_platform_data tvp5150_pdata = {
	.pdn = PDEC_PWRDNn,
	.resetb = PDEC_RESETn,
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

static int basi_setup_video_input(enum vpfe_subdev_id id)
{
	return 0;
}

static struct vpfe_config vpfe_cfg = {
	.setup_input = basi_setup_video_input,
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "DM365 BASI",
	.ccdc = "DM365 ISIF",
	.num_clocks = 1,
	.clocks = {"vpss_master"},
};

void basi_phy_power(int on)
{
	gpio_set_value(ENET_RESETn, on);
}

static void basi_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = BASI_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = BASI_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = basi_phy_power;

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

static struct davinci_adc_platform_data basi_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device basi_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &basi_adc_data,
		},
	.id = 0,
};

static struct gpio_led basi_gpio_led[] = {
	{
		.name                   = "LED_SYSTEM",
		.gpio                   = LED_1,
		.active_low             = 1,
		.default_trigger        = "none",
		.default_state          = LEDS_GPIO_DEFSTATE_ON,
	},
};

static struct gpio_led_platform_data basi_gpio_led_info = {
	.leds = &basi_gpio_led,
	.num_leds = 1,
};

static struct platform_device basi_leds_gpio_device = {
	.name = "leds-gpio",
	.dev = {
		.platform_data = &basi_gpio_led_info,
		},
	.id = 0,
};

static void __init basi_led_init(void)
{
	davinci_cfg_reg(DM365_GPIO40);
	platform_device_register(&basi_leds_gpio_device);
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

static struct plat_serial8250_port basi_dm365_serial_platform_data[] = {
	{
		.mapbase        = DM365_ASYNC_EMIF_DATA_CE1_BASE,
		.irq            = IRQ_DM365_GPIO0_1,
		.type           = PORT_16550A,
		.flags          = UPF_BOOT_AUTOCONF | UPF_IOREMAP | UART_CONFIG_TYPE | UPF_SKIP_TEST | UPF_FIXED_TYPE,
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

static struct platform_device basi_dm365_serial_device = {
	.name                   = "serial8250",
	.id                     = PLAT8250_DEV_PLATFORM1,
	.resource               = davinci_asyn_resources,
	.dev                    = {
		.platform_data  = basi_dm365_serial_platform_data,
	},
};

static void basi_uart2_configure(void)
{
	struct clk *aemif_clk;
	struct davinci_aemif_timing t;
	void __iomem *base;
	int ret;
	base = ioremap(DM365_ASYNC_EMIF_CONTROL_BASE, SZ_4K - 1);

	aemif_clk = clk_get(NULL, "aemif");
	if (IS_ERR(aemif_clk)) {
		printk(KERN_ERR "couldn't get AEMIF CLOCK!!! - ttyS2 won't work\n");
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
		printk(KERN_ERR  "failed setting aemif timings - ttyS2 won't work");
		return;
	}

	/* setting proper pinmux for AEMIF */
	davinci_cfg_reg(DM365_AEMIF_AR2);
	davinci_cfg_reg(DM365_AEMIF_CE1);
	davinci_cfg_reg(DM365_EM_WE_OE);
	davinci_cfg_reg(DM365_GPIO51);
	gpio_direction_output(RES_EXTUART, 0);

	set_irq_type(gpio_to_irq(0), IRQ_TYPE_EDGE_BOTH);
	platform_device_register(&basi_dm365_serial_device);
}

static struct irq_on_gpio0 {
	unsigned int gpio;
	unsigned int irq;
};

static struct irq_on_gpio0 basi_irq_on_gpio0 [] = {
	{
		.gpio = INT_UART,
		.irq = IRQ_DM365_GPIO0_1,
	}, {
		.gpio = POWER_FAIL,
		.irq = IRQ_DM365_GPIO0_2,
	},
};

static unsigned long basi_gpio0_irq_enabled;

static void basi_mask_irq_gpio0(unsigned int irq)
{
	int basi_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	basi_gpio0_irq_enabled &= ~(1 << basi_gpio_irq);
}

static void basi_unmask_irq_gpio0(unsigned int irq)
{
	int basi_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	basi_gpio0_irq_enabled |= (1 << basi_gpio_irq);
}

static struct irq_chip basi_gpio0_irq_chip = {
	.name           = "DM365_GPIO0",
	.ack            = basi_mask_irq_gpio0,
	.mask           = basi_mask_irq_gpio0,
	.unmask         = basi_unmask_irq_gpio0,
};

static void basi_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cnt;

	if ((basi_gpio0_irq_enabled & 0x01) && (gpio_get_value(0))) {
		/* enabled interrupt when lines are all high */
		generic_handle_irq(IRQ_DM365_GPIO0_0);
	}
	else if (!gpio_get_value(0)) {
		for (cnt = 0; cnt < ARRAY_SIZE(basi_irq_on_gpio0); cnt++) {
			if (!gpio_get_value(basi_irq_on_gpio0[cnt].gpio) && (basi_gpio0_irq_enabled &
			    (1<<(basi_irq_on_gpio0[cnt].irq - IRQ_DM365_GPIO0_0)))){
				generic_handle_irq(basi_irq_on_gpio0[cnt].irq);
			}
		}
	}
	desc->chip->ack(irq);
}

static void __init basi_gpio_init_irq(void)
{
	int irq;

	davinci_irq_init();
	/* setup extra Basi irqs on GPIO0*/
	for(irq = IRQ_DM365_GPIO0_0; irq <= IRQ_DM365_GPIO0_2; irq++) {
		set_irq_chip(irq, &basi_gpio0_irq_chip);
		set_irq_handler(irq, handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(IRQ_DM365_GPIO0, IRQ_TYPE_EDGE_BOTH);
	set_irq_chained_handler(IRQ_DM365_GPIO0, basi_gpio_irq_handler);
}

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t basi_powerfail_stop(int irq, void *dev_id);

static irqreturn_t basi_powerfail_quick_check_start(int irq, void *dev_id)
{
	basi_mask_irq_gpio0(IRQ_DM365_GPIO0_2);
	basi_unmask_irq_gpio0(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t basi_powerfail_start(int irq, void *dev_id)
{
	if (powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 1;
	pm_loss_power_changed(SYS_PWR_FAILING);
	return IRQ_HANDLED;
}

static irqreturn_t basi_powerfail_quick_check_stop(int irq, void *dev_id)
{
	basi_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	basi_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t basi_powerfail_stop(int irq, void *dev_id)
{
	if (!powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);
	return IRQ_HANDLED;
}

enum basi_pwrfail_prio {
	BASI_PWR_FAIL_PRIO_0,
	BASI_PWR_FAIL_MIN_PRIO = BASI_PWR_FAIL_PRIO_0,
	BASI_PWR_FAIL_PRIO_1,
	BASI_PWR_FAIL_PRIO_2,
	BASI_PWR_FAIL_PRIO_3,
	BASI_PWR_FAIL_MAX_PRIO = BASI_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item basi_pm_loss_policy_items[] = {
	{
		.bus_name = "mmc",
		.bus_priority = BASI_PWR_FAIL_PRIO_1,
	},
	{
		.bus_name = "platform",
		.bus_priority = BASI_PWR_FAIL_PRIO_2,
	},
};

#define BASI_POLICY_NAME "basi-default"

struct pm_loss_default_policy_table basi_pm_loss_policy_table = {
	.name = BASI_POLICY_NAME,
	.items = basi_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(basi_pm_loss_policy_items),
};

static void basi_powerfail_configure(void)
{
	int stat;
	struct pm_loss_policy *p;
	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    basi_powerfail_quick_check_start,
				    basi_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    basi_powerfail_quick_check_stop,
				    basi_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	basi_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	p = pm_loss_setup_default_policy(&basi_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(BASI_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       BASI_POLICY_NAME);
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
#define basi_powerfail_configure()
#endif

static void basi_gpio_configure(void)
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

	gpio_request(0, "GIO0");
	gpio_direction_input(0);

	davinci_cfg_reg(DM365_GPIO45);
	status = gpio_request(BOOT_FL_WP, "Protecting SPI chip select");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO: Protecting SPI \
		                 chip select: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(BOOT_FL_WP, 0);

	davinci_cfg_reg(DM365_GPIO36);
	status = gpio_request(EN_AUDIO, "Audio modulator Enable on external connector");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO: Audio modulator Enable on \
		                 external connector: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(EN_AUDIO, 0);

	davinci_cfg_reg(DM365_GPIO98);
	status = gpio_request(ABIL_DEM_VIDEO, "Enable video demodulator");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO: Enable video \
				 demodulator: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(ABIL_DEM_VIDEO, 1); /* enabled, to allow i2c attach */

	davinci_cfg_reg(DM365_GPIO37);
	status = gpio_request(PDEC_PWRDNn, "Pal-Decoder power down");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO: Pal-Decoder \
			         power down: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(PDEC_PWRDNn, 1);

	davinci_cfg_reg(DM365_GPIO26);
	status = gpio_request(PDEC_RESETn, "PAL decoder Reset");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO PAL \
					   decoder reset: %d\n", __func__,
					    status);
		return;
	}
	gpio_direction_output(PDEC_RESETn, 1);

	davinci_cfg_reg(DM365_GPIO97);
	status = gpio_request(E2_WPn, "Eeprom write protect");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO eeprom \
				      write protect: %d\n", __func__,
				      status);
		return;
	}
	gpio_direction_output(E2_WPn, 0);

	davinci_cfg_reg(DM365_GPIO100);
	status = gpio_request(REG_ERR_AVn, "Reset pic SCS AV");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO reset pic \
			         scs av: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(REG_ERR_AVn, 1);

	davinci_cfg_reg(DM365_GPIO99);
	status = gpio_request(EN_RTC, "RTC power enable");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO enable \
				 external rtc powering: %d\n", __func__,
				 status);
		return;
	}
	gpio_direction_output(EN_RTC, 1); /* enabled */

	davinci_cfg_reg(DM365_GPIO35);
	status = gpio_request(ZARLINK_CS, "Zarlink chip select");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO Zarlink \
				 chip select: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(ZARLINK_CS, 1);

	davinci_cfg_reg(DM365_GPIO38);
	status = gpio_request(PIC_RESET_N, "PIC AI reset");
	if (status) {
		printk(KERN_ERR "%s: failed to request PIC AI \
				 reset: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(PIC_RESET_N, 0);

	davinci_cfg_reg(DM365_GPIO39);
	status = gpio_request(OC2_VBUS_USB, "Over Current VBUS");
	if (status) {
		printk(KERN_ERR "%s: failed to request Over Current \
				 VBUS: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(OC2_VBUS_USB);

	davinci_cfg_reg(DM365_GPIO41);
	status = gpio_request(SD_CARD_DET, "Always true eMMC det");
	if (status) {
		printk(KERN_ERR "%s: failed to request always true \
				 eMMC det: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(SD_CARD_DET);

	davinci_cfg_reg(DM365_GPIO42);
	status = gpio_request(EMMC_RESET, "eMMC reset");
	if (status) {
		printk(KERN_ERR "%s: failed to request eMMC \
				 reset %d\n", __func__, status);
		return;
	}
	gpio_direction_output(EMMC_RESET, 1);

	davinci_cfg_reg(DM365_GPIO44);
	status = gpio_request(ENET_RESETn, "ENET RESET");
	if (status) {
		printk(KERN_ERR "%s: failed to request ENET \
				 RESET %d\n", __func__, status);
		return;
	}
	gpio_direction_output(ENET_RESETn, 1);

/*
 *      TODO
 * 	davinci_cfg_reg(DM365_GPIO69); see PINMUX2 .. difficult
 */

	davinci_cfg_reg(DM365_GPIO50);
	status = gpio_request(POWER_FAIL, "POWER_FAIL");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO POWER \
				 FAIL: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(POWER_FAIL);

	davinci_cfg_reg(DM365_GPIO51);
	status = gpio_request(RES_EXTUART, "RES_EXTUART");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO RES \
				 EXTUART: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(RES_EXTUART, 0);

	davinci_cfg_reg(DM365_GPIO96);
	status = gpio_request(PB_IN, "External Push Button");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO push button \
				 in: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(PB_IN);

	if (basi_debug) {
		gpio_export(BOOT_FL_WP, 0); /* danger */
		gpio_export(EN_AUDIO, 0);
		gpio_export(ABIL_DEM_VIDEO, 0);
		gpio_export(PDEC_PWRDNn, 0);
		gpio_export(PDEC_RESETn, 0);
		gpio_export(PIC_RESET_N, 0);
		gpio_export(E2_WPn, 0);
		gpio_export(REG_ERR_AVn, 0);
		gpio_export(EN_RTC, 0);
		gpio_export(ZARLINK_CS, 0);
/*
 * ZARLINK_RESET todo
 */
		gpio_export(PB_IN, 0);
		gpio_export(OC2_VBUS_USB, 0);
		gpio_export(SD_CARD_DET, 0);
		gpio_export(POWER_FAIL, 0);
		gpio_export(RES_EXTUART, 0);
		gpio_export(EMMC_RESET, 0);
	}
}

static int basi_mmc_get_ro(int module)
{
	return gpio_get_value(SD_WR_PR);
}

static struct davinci_mmc_config basi_mmc_config = {
	/* .get_cd is not defined since it seems it useless... the MMC
	 * controller detects anyway the card! O_o
	 */
	.get_ro		= basi_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void basi_mmc_configure(void)
{
	int ret;

	davinci_cfg_reg(DM365_MMCSD0);
	davinci_cfg_reg(DM365_GPIO43); /* Not connected to the eMMC */

	ret = gpio_request(SD_WR_PR, "MMC WP");
	if (ret)
		pr_warning("basi: cannot request GPIO %d!\n", SD_WR_PR);
	gpio_direction_input(SD_WR_PR);
	davinci_setup_mmc(0, &basi_mmc_config);

	return;
}

static void basi_usb_configure(void)
{
	pr_notice("Launching setup_usb\n");
	setup_usb(500, 8);
}

static struct platform_device basi_asoc_device[] = {
	{
		.name = "basi-asoc",
		.id = 0,
	},
};

static struct i2c_board_info __initdata basi_i2c_info[] = {
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

static void __init basi_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, basi_i2c_info, ARRAY_SIZE(basi_i2c_info));
}

static struct platform_device *basi_devices[] __initdata = {
	&basi_asoc_device[0],
	&basi_hwmon_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1) | (1 << 2),
};

static void __init basi_map_io(void)
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

static struct spi_board_info basi_spi_info[] __initconst = {
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
		.controller_data = ZARLINK_CS,
		.mode           = SPI_MODE_0,
	},

};

static struct snd_platform_data dm365_basi_snd_data;


static __init void basi_init(void)
{
	if (basi_debug>1)
		pinmux_check();
	basi_gpio_configure();
	basi_led_init();
	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
	davinci_serial_init(&uart_config);
	mdelay(1);
	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	basi_init_i2c();
	basi_emac_configure();

	dm365_init_spi0(0, basi_spi_info, ARRAY_SIZE(basi_spi_info));

	basi_mmc_configure();
	basi_usb_configure();
	basi_uart2_configure();
	basi_powerfail_configure();
	dm365_init_vc(&dm365_basi_snd_data);
	platform_add_devices(basi_devices, ARRAY_SIZE(basi_devices));
	dm365_init_rtc();
	dm365_init_adc(&basi_adc_data);

	if (basi_debug)
		pinmux_check();
}

MACHINE_START(BASI, "BTicino BASI board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = basi_map_io,
	.init_irq = basi_gpio_init_irq,
	.timer = &davinci_timer,
	.init_machine = basi_init,
MACHINE_END
