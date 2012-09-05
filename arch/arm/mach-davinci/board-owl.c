/*
 * BTicino S.p.A. owl platform support
 * based on evm-dm365 board
 *
 * Copyright (C) 2011 davide .bonfanti@bticino.it
 *                    raffaele.recalcati@bticino.it
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
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/eeprom.h>
#include <linux/delay.h>
#include <linux/backlight.h>
#include <linux/tm035kbh02.h>
#include <linux/serial_8250.h>
#include <linux/spi/ads7846.h>
#include <linux/irq.h>
#include <linux/phy.h>

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
#include <mach/adc.h>
#include <media/soc_camera.h>
#include <mach/aemif.h>
#include <video/davincifb.h>
#include <linux/pm_loss.h>
#include <linux/videodev2.h>
#include <media/davinci/videohd.h>
#include <media/davinci/dm365_ccdc.h>
#include <media/ov971x.h>

#include <mach/owl.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define DM365_EVM_PHY_MASK              (0x2)
#define DM365_EVM_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
#define HW_IN_CLOCKOUT2_UDA_I2S

#define NAND_BLOCK_SIZE         SZ_128K

#define OWL_PHY_MASK              (0x2)
#define OWL_MDIO_FREQUENCY        (2200000) /* PHY bus frequency */

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

#define POWER_FAIL_DEBOUNCE_TO 50
#define TIME_TO_LATE_INIT 1500

#define OV971X_XCLK		24

static int owl_debug = 1;
module_param(owl_debug, int, 0644);
MODULE_PARM_DESC(dowl_debug, "Debug level 0-1");

/*
wp_set: set/unset the at24 eeprom write protect
*/
void wp_set(int enable)
{
	gpio_direction_output(E2_WP, enable);
}

static struct at24_platform_data at24_info = {
	.byte_len = (256 * 1024) / 8,
	.page_size = 64,
	.flags = AT24_FLAG_ADDR16,
	.setup = davinci_get_mac_addr,
	.context = (void *)0x19e,	/* where it gets the mac-address */
	.wpset = wp_set,
};

static struct timer_list pf_debounce_timer, startup_timer;

static struct mtd_partition owl_nand_partitions[] = {
	{
		/* U-Boot */
		.name           = "u-boot",
		.offset         = 0,
		.size           = 12 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE, /* force read-only */
	}, {
		/* U-Boot environment */
		.name           = "u-boot e",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * NAND_BLOCK_SIZE,
		.mask_flags     = 0,
	}, {	/* Recovery copy of kernel */
		.name           = "kernel_c",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_8M,
		.mask_flags     = 0,
	}, {	/* Recovery copy of rootfs */
		.name           = "rootfs_c",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 20 * SZ_1M,
		.mask_flags     = 0,
	}, {	/* Primary copy of kernel */
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_8M,
		.mask_flags     = 0,
	}, {	/* Primary copy of rootfs */
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = SZ_70M,
		.mask_flags     = 0,
	}, {	/* Configurations and extras */
		.name           = "extras",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
		.mask_flags     = 0,
	}
	/* two blocks with bad block table (and mirror) at the end */
};

static struct davinci_aemif_timing owl_nandflash_timing = {
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
	.parts		= owl_nand_partitions,
	.nr_parts	= ARRAY_SIZE(owl_nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.options	= NAND_USE_FLASH_BBT,
	.ecc_bits	= 4,
	.timing		= &owl_nandflash_timing,
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

void owl_phy_power(int on)
{
	gpio_set_value(ENET_RESETn, on);
}

static void owl_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = OWL_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = OWL_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = owl_phy_power;

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

static struct davinci_adc_platform_data owl_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device owl_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &owl_adc_data,
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
		printk(KERN_ERR "pinmux%d=%X\n", i, pinmux[i]);
	}
}

struct irq_on_gpio0 {
	unsigned int gpio;
	unsigned int irq;
};

static struct irq_on_gpio0 owl_irq_on_gpio0[] = {
	{
		.gpio = POWER_FAIL,
		.irq = IRQ_DM365_GPIO0_2,
	}, {
		.gpio = TMK_INTn,
		.irq = IRQ_DM365_GPIO0_3,
	},
};

static unsigned long owl_gpio0_irq_enabled;

static void owl_mask_irq_gpio0(unsigned int irq)
{
	int owl_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	owl_gpio0_irq_enabled &= ~(1 << owl_gpio_irq);
}

static void owl_unmask_irq_gpio0(unsigned int irq)
{
	int owl_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	owl_gpio0_irq_enabled |= (1 << owl_gpio_irq);
}

static int owl_set_wake_irq_gpio0(unsigned int irq, unsigned int on)
{
	return 0;
}

static struct irq_chip owl_gpio0_irq_chip = {
	.name		= "DM365_GPIO0",
	.ack		= owl_mask_irq_gpio0,
	.mask		= owl_mask_irq_gpio0,
	.unmask		= owl_unmask_irq_gpio0,
	.disable	= owl_mask_irq_gpio0,
	.enable		= owl_unmask_irq_gpio0,
	.bus_sync_unlock = owl_unmask_irq_gpio0,
	.bus_lock	= owl_mask_irq_gpio0,
	.set_wake	= owl_set_wake_irq_gpio0,
};

static void owl_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cnt;

	desc->chip->ack(irq);
	if ((owl_gpio0_irq_enabled & 0x01) && (gpio_get_value(0))) {
		/* enabled interrupt when lines are all high */
		generic_handle_irq(IRQ_DM365_GPIO0_0);
	} else if (!gpio_get_value(0)) {
		udelay(50);
		for (cnt = 0; cnt < ARRAY_SIZE(owl_irq_on_gpio0); cnt++) {
			if (!gpio_get_value(owl_irq_on_gpio0[cnt].gpio) &&
			   (owl_gpio0_irq_enabled & (1<<
			   (owl_irq_on_gpio0[cnt].irq - IRQ_DM365_GPIO0_0)))
			   )
				generic_handle_irq(owl_irq_on_gpio0[cnt].irq);
		}
	}
	desc->chip->unmask(irq);
}

static void __init owl_gpio_init_irq(void)
{
	int irq;

	davinci_irq_init();
	/* setup extra Basi irqs on GPIO0*/
	for (irq = IRQ_DM365_GPIO0_0;
	    irq <= (IRQ_DM365_GPIO0_0 + ARRAY_SIZE(owl_irq_on_gpio0)); irq++) {
		set_irq_chip(irq, &owl_gpio0_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	set_irq_type(IRQ_DM365_GPIO0, IRQ_TYPE_EDGE_BOTH);
	set_irq_chained_handler(IRQ_DM365_GPIO0, owl_gpio_irq_handler);
}

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t owl_powerfail_stop(int irq, void *dev_id);

static irqreturn_t owl_powerfail_quick_check_start(int irq, void *dev_id)
{
	owl_mask_irq_gpio0(IRQ_DM365_GPIO0_2);
	owl_unmask_irq_gpio0(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t owl_powerfail_start(int irq, void *dev_id)
{
	owl_mask_irq_gpio0(IRQ_DM365_GPIO0_2);
	owl_unmask_irq_gpio0(IRQ_DM365_GPIO0_0);
	if (powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 1;
	pm_loss_power_changed(SYS_PWR_FAILING);

	/* PowerFail situation - START: power is going away */

	return IRQ_HANDLED;
}

static irqreturn_t owl_powerfail_quick_check_stop(int irq, void *dev_id)
{
	owl_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	owl_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static void debounce(unsigned long data)
{
	del_timer(&pf_debounce_timer);
	if (!gpio_get_value(POWER_FAIL))
		return;

	if (!powerfail_status)
		return;
	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);

	owl_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	owl_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);

	/* PowerFail situation - STOP: power is back */
}

static irqreturn_t owl_powerfail_stop(int irq, void *dev_id)
{
	/* PowerFail situation - STOP: power is coming back */
	init_timer(&pf_debounce_timer);
	pf_debounce_timer.function = debounce;
	pf_debounce_timer.expires =
			jiffies + msecs_to_jiffies(POWER_FAIL_DEBOUNCE_TO);
	add_timer(&pf_debounce_timer);

	return IRQ_HANDLED;
}

enum owl_pwrfail_prio {
	OWL_PWR_FAIL_PRIO_0,
	OWL_PWR_FAIL_MIN_PRIO = OWL_PWR_FAIL_PRIO_0,
	OWL_PWR_FAIL_PRIO_1,
	OWL_PWR_FAIL_PRIO_2,
	OWL_PWR_FAIL_PRIO_3,
	OWL_PWR_FAIL_MAX_PRIO = OWL_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item owl_pm_loss_policy_items[] = {
	{
		.bus_name = "platform",
		.bus_priority = OWL_PWR_FAIL_PRIO_1,
	},
};

#define OWL_POLICY_NAME "owl-default"

struct pm_loss_default_policy_table owl_pm_loss_policy_table = {
	.name = OWL_POLICY_NAME,
	.items = owl_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(owl_pm_loss_policy_items),
};

static void owl_powerfail_configure(void)
{
	int stat;
	struct pm_loss_policy *p;
	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    owl_powerfail_quick_check_start,
				    owl_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    owl_powerfail_quick_check_stop,
				    owl_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	owl_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	p = pm_loss_setup_default_policy(&owl_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(OWL_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       OWL_POLICY_NAME);

	printk(KERN_ERR "%s OK\n", __func__);
}

int platform_pm_loss_power_changed(struct device *dev, enum sys_power_state s)
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
#define owl_powerfail_configure()
#endif

static void owl_gpio_configure(void)
{
	int status;
	void __iomem *pupdctl0 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x78);
	void __iomem *pupdctl1 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x7c);

	/* Configure (disable) pull down control */
	__raw_writel(0x80000000, pupdctl0);
	/* active pull-down EM_WAIT + YIN[7:0] */
	__raw_writel(0xC0040000, pupdctl1);

	gpio_request(0, "GIO0");
	gpio_direction_input(0);

	davinci_cfg_reg(DM365_GPIO26);
	status = gpio_request(TMK_INTn, "Timer keeper interrupt");
	if (status) {
		pr_err("%s: failed to request GPIO: Timer Keeper " \
				"interrupt: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(TMK_INTn);

	davinci_cfg_reg(DM365_GPIO40);
	status = gpio_request(E2_WP, "EEPROM Write protect");
	if (status) {
		pr_err("%s: failed to request GPIO: EEPROM Write " \
				"protect: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(E2_WP, 1);

	davinci_cfg_reg(DM365_GPIO44);
	status = gpio_request(ENET_RESETn, "ENET RESET");
	if (status) {
		pr_err("%s: failed to request ENET " \
				"RESET %d\n", __func__, status);
		return;
	}
	gpio_direction_output(ENET_RESETn, 1);

	davinci_cfg_reg(DM365_GPIO50);
	status = gpio_request(POWER_FAIL, "Early advise of power down");
	if (status) {
		pr_err("%s: failed to request GPIO: early advise of " \
				"power down: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(POWER_FAIL);

	davinci_cfg_reg(DM365_GPIO80);
	status = gpio_request(CMOS_PWDN, "CMOS Power Down");
	if (status) {
		pr_err("%s: failed to request GPIO: CMOS pwdow: %d\n",
							__func__, status);
		return;
	}
	gpio_direction_output(CMOS_PWDN, 0);

	mdelay(1);

	davinci_cfg_reg(DM365_GPIO48);
	status = gpio_request(CMOS_RSTBn, "CMOS Reset");
	if (status) {
		pr_err("%s: failed to request GPIO: CMOS reset: %d\n",
							 __func__, status);
		return;
	}
	gpio_direction_output(CMOS_RSTBn, 1);
	mdelay(1);

	if (owl_debug) {
		gpio_export(BOOT_FL_WP, 0); /* danger */
		gpio_export(E2_WP, 0);
		gpio_export(TMK_INTn, 0);
		gpio_export(POWER_FAIL, 0);
		gpio_export(23, 0);
		gpio_export(CMOS_PWDN, 0);
		gpio_export(CMOS_RSTBn, 0);
	}
}

static struct platform_device owl_asoc_device[] = {
	{
		.name = "owl-asoc",
		.id = 0,
	},
};

static struct i2c_board_info __initdata owl_i2c_info[] = {
	{
		I2C_BOARD_INFO("pcf8563", 0x51),
		.irq = IRQ_DM365_GPIO0_3,
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

static void __init owl_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, owl_i2c_info, ARRAY_SIZE(owl_i2c_info));
}

static struct platform_device *owl_devices[] __initdata = {
	&davinci_nand_device,
	&owl_asoc_device[0],
	&owl_hwmon_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1), /* | (1 << 2), */
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
		/* .std = V4L2_STD_OV971X_STD_ALL, */
	}
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
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
		.board_info = {
			I2C_BOARD_INFO("ov971x", 0x30),
			.platform_data = &ov971x_pdata,
		},
	},
};

static int owl_setup_video_input(enum vpfe_subdev_id id)
{
	return 0;
}

static struct vpfe_config vpfe_cfg = {
	.setup_input = owl_setup_video_input,
	.num_subdevs = ARRAY_SIZE(vpfe_sub_devs),
	.sub_devs = vpfe_sub_devs,
	.card_name = "DM365 OWL",
	.ccdc = "DM365 ISIF",
	.num_clocks = 1,
	.clocks = {"vpss_master"},
};

static void __init owl_map_io(void)
{
	dm365_set_vpfe_config(&vpfe_cfg);
	dm365_init_isif(DM365_ISIF_10BIT);
	dm365_init();
}

static struct snd_platform_data dm365_owl_snd_data[] = {
	{
		.asp_chan_q = EVENTQ_3,
		.ram_chan_q = EVENTQ_2,
		.i2s_accurate_sck = 1,
		.clk_input_pin = MCBSP_CLKS,
	},
};

struct device my_device;

static void owl_late_init(unsigned long data)
{
	int status;

	del_timer(&startup_timer);
	davinci_cfg_reg(DM365_GPIO45);
	status = gpio_request(BOOT_FL_WP, "Protecting SPI0 chip select");
	if (status) {
		pr_err("%s: fail GPIO request: Protecting SPI0" \
				" chip select: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(BOOT_FL_WP, 1);

	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
/*
	gpio_direction_output(DEBUG_GPIO1, 1);
	mdelay(2);
	gpio_direction_output(DEBUG_GPIO1, 0);
	gpio_direction_output(DEBUG_GPIO1, 1);
	mdelay(2);
	gpio_direction_output(DEBUG_GPIO1, 0);
*/
}

static __init void owl_init(void)
{
	int ret;

	pr_warning("Owl_init: START\n");
	if (owl_debug > 1)
		pinmux_check();
	owl_gpio_configure();

	ret = dm365_clkout2_set_rate(OV971X_XCLK * 1000 * 1000);

	gpio_direction_output(CMOS_RSTBn, 0);
	mdelay(1);
	gpio_direction_output(CMOS_RSTBn, 1);
	mdelay(100);

	davinci_serial_init(&uart_config);
	pr_warning("owl_init: starting ...\n");

	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	owl_init_i2c();
	owl_emac_configure();

	owl_powerfail_configure();
	davinci_cfg_reg(DM365_CLKOUT2);
	dm365_init_vc(&dm365_owl_snd_data[0]);

	platform_add_devices(owl_devices, ARRAY_SIZE(owl_devices));

	dm365_init_rtc();
	dm365_init_adc(&owl_adc_data);

	init_timer(&startup_timer);
	startup_timer.function = owl_late_init;
	startup_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_LATE_INIT);
	add_timer(&startup_timer);

	if (owl_debug > 1)
		pinmux_check();
	pr_warning("owl_init: DONE\n");
}

MACHINE_START(OWL, "bticino OWL board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = owl_map_io,
	.init_irq = owl_gpio_init_irq,
	/* .init_irq = davinci_irq_init, */
	.timer = &davinci_timer,
	.init_machine = owl_init,
MACHINE_END
