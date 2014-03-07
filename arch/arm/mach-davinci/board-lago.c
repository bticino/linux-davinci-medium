/*
 * BTicino S.p.A. lago platform support based on evm-dm365 board
 *
 * raffaele.recalcati@bticino.it, davide.bonfanti@bticino.it ,
 * simone.cianni@bticino.it
 *
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
#include <mach/bt_nexmed-hwmon.h>
#include <linux/videodev2.h>
#include <media/soc_camera.h>
#include <media/tvp5150.h>
#include <linux/i2c/tda9885.h>
#include <linux/i2c/tvp5150.h>
#include <mach/aemif.h>
#include <linux/irq_gpio.h>

#include <mach/lago.h>
#include <mach/aemif.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define LAGO_PHY_MASK              (0x2)
#define LAGO_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */

struct work_struct late_init_work;
static struct timer_list startup_timer;
#define TIME_TO_LATE_INIT 1500

static int lago_debug = 1;
module_param(lago_debug, int, 0644);
MODULE_PARM_DESC(lago_debug, "Debug level 0-1");

extern unsigned int system_rev;
extern int lookup_resistors(int cnt);

/* wp_set: set/unset the at24 eeprom write protect */
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

void lago_phy_power(int on)
{
	printk(KERN_DEBUG "Reset eth controller \n");
	gpio_set_value(ENET_RESETn, !on);
	mdelay(2); /* Measured by oscilloscope : must be at least 1,6ms */
	gpio_set_value(ENET_RESETn, on);
}

static void lago_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = LAGO_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = LAGO_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = lago_phy_power;

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

static struct davinci_adc_platform_data lago_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device lago_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &lago_adc_data,
		},
	.id = 0,
};

static struct gpio_led lago_gpio_led[] = {
	{
		.name                   = "LED_SYSTEM",
		.gpio                   = LED_1,
		.active_low             = 1,
		.default_trigger        = "none",
		.default_state          = LEDS_GPIO_DEFSTATE_ON,
	},
};

static struct gpio_led_platform_data lago_gpio_led_info = {
	.num_leds = 1,
	.leds = &lago_gpio_led,
};

static struct platform_device lago_leds_gpio_device = {
	.name = "leds-gpio",
	.dev = {
		.platform_data = &lago_gpio_led_info,
		},
	.id = 0,
};

static void __init lago_led_init(void)
{
	davinci_cfg_reg(DM365_GPIO40);
	platform_device_register(&lago_leds_gpio_device);
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

static struct irq_on_gpio lago_irq_on_gpio0[] = {
	{
		.gpio = POWER_FAIL,
		.irq = IRQ_DM365_GPIO0_0,
		.type = EDGE,
		.mode = GPIO_EDGE_RISING,
	}, {
		.gpio = POWER_FAIL,
		.irq = IRQ_DM365_GPIO0_2,
		.type = EDGE,
		.mode = GPIO_EDGE_FALLING,
	},/* {
		.gpio = TMK_INTn,
		.irq = IRQ_DM365_GPIO0_4,
		.type = LEVEL,
		.mode = GPIO_EDGE_FALLING,
	},*/
};

static struct irq_gpio_platform_data lago_irq_gpio_platform_data = {
	.gpio_list = lago_irq_on_gpio0,
	.len = ARRAY_SIZE(lago_irq_on_gpio0),
	.gpio_common = 0,
	.irq_gpio = IRQ_DM365_GPIO0,
};

static struct platform_device lago_irq_gpio_device = {
	.name			= "irq_gpio",
	.id			= 0,
	.num_resources		= 0,
	.resource		= NULL,
	.dev			= {
		.platform_data	= &lago_irq_gpio_platform_data,
	},
};

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t lago_powerfail_stop(int irq, void *dev_id);

static irqreturn_t lago_powerfail_quick_check_start(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->mask(IRQ_DM365_GPIO0_2);
	desc->chip->unmask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t lago_powerfail_start(int irq, void *dev_id)
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

static irqreturn_t lago_powerfail_quick_check_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t lago_powerfail_stop(int irq, void *dev_id)
{
	struct irq_desc *desc = irq_to_desc(irq);

	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);
	desc->chip->unmask(IRQ_DM365_GPIO0_2);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - STOP: power is coming back */
	return IRQ_HANDLED;
}

enum lago_pwrfail_prio {
	LAGO_PWR_FAIL_PRIO_0,
	LAGO_PWR_FAIL_MIN_PRIO = LAGO_PWR_FAIL_PRIO_0,
	LAGO_PWR_FAIL_PRIO_1,
	LAGO_PWR_FAIL_PRIO_2,
	LAGO_PWR_FAIL_PRIO_3,
	LAGO_PWR_FAIL_MAX_PRIO = LAGO_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item lago_pm_loss_policy_items[] = {
	{
		.bus_name = "mmc",
		.bus_priority = LAGO_PWR_FAIL_PRIO_1,
	},
	{
		.bus_name = "platform",
		.bus_priority = LAGO_PWR_FAIL_PRIO_2,
	},
};

#define LAGO_POLICY_NAME "lago-default"

struct pm_loss_default_policy_table lago_pm_loss_policy_table = {
	.name = LAGO_POLICY_NAME,
	.items = lago_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(lago_pm_loss_policy_items),
};

static void lago_powerfail_configure(struct work_struct *work)
{
	int stat;
	struct pm_loss_policy *p;
	struct irq_desc *desc;

	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    lago_powerfail_quick_check_start,
				    lago_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    lago_powerfail_quick_check_stop,
				    lago_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	desc = irq_to_desc(IRQ_DM365_GPIO0_0);
	desc->chip->mask(IRQ_DM365_GPIO0_0);

	p = pm_loss_setup_default_policy(&lago_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(LAGO_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       LAGO_POLICY_NAME);

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
#define lago_powerfail_configure()
#endif

/* gpio_configure in/out */
inline int gpio_configure_out(int DM365_Pin, int Name, int DefVualue, \
				char *str)
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

inline int gpio_configure_in(int DM365_Pin, int Name, char *str)
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

static void lago_gpio_configure(void)
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
	gpio_configure_in(DM365_GPIO50, POWER_FAIL, "Advise of power down");
	gpio_configure_in(DM365_GPIO39, OC2_VBUS_USB, "Over Current VBUS");
	gpio_configure_in(DM365_GPIO41, SD_CARD_DET, "Always true eMMC det");
	gpio_configure_in(DM365_GPIO96, PB_IN, "External Push Button");

	/* -- Configure Output -----------------------------------------------*/

	gpio_configure_out(DM365_GPIO45, BOOT_FL_WP, 0,
			"Protect SPI ChipSelect");
	gpio_configure_out(DM365_GPIO97, E2_WPn, 0, "EEprom write protect");
	gpio_configure_out(DM365_GPIO100, REG_ERR_AVn, 1, "Reset pic SCS AV");
	gpio_configure_out(DM365_GPIO99, EN_RTC, 1, "RTC power enable");
	gpio_configure_out(DM365_GPIO42, EMMC_RESET, 1, "eMMC reset");
	gpio_configure_out(DM365_GPIO44, ENET_RESETn, 1, "ENET RESET");

	/* -- Export For Debug -----------------------------------------------*/

	if (lago_debug) {
		gpio_export(BOOT_FL_WP, 0); /* danger */
		gpio_export(PIC_RESET_N, 0);
		gpio_export(E2_WPn, 0);
		gpio_export(REG_ERR_AVn, 0);
		gpio_export(EN_RTC, 0);
		gpio_export(PB_IN, 0);
		gpio_export(OC2_VBUS_USB, 0);
		gpio_export(SD_CARD_DET, 0);
		gpio_export(POWER_FAIL, 0);
		gpio_export(EMMC_RESET, 0);
	}
}

#if 0
static int jumbo_mmc_get_ro(int module)
{
	/* high == card's write protect switch active */
	return 0;
}

static int jumbo_mmc1_get_cd(int module)
{
	/* low == card present */
	return gpio_get_value(piSD_DETECTn);
}

static struct davinci_mmc_config jumbo_mmc_config[] = {
	{
		/*/ .get_cd is not defined since it seems it useless... the MMC
		 controller detects anyway the card! O_o */
		.get_ro		= jumbo_mmc_get_ro,
		.wires		= 4,
		.max_freq	= 50000000,
		.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
		.version	= MMC_CTLR_VERSION_2,
	}, {
		/*.get_cd		= jumbo_mmc1_get_cd, */
		.get_ro		= jumbo_mmc_get_ro,
		.wires		= 4,
		.max_freq	= 50000000,
		.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
		.version	= MMC_CTLR_VERSION_2,
	},
};

static void jumbo_mmc0_configure(void)
{
	/* MMC/SD 0 */
	davinci_cfg_reg(DM365_MMCSD0);

	davinci_setup_mmc(0, &jumbo_mmc_config[0]);

	return;
}

static void jumbo_mmc1_sd1_configure(void)
{
	/* MMC/SD 1 */
	davinci_cfg_reg(DM365_SD1_CLK);
	davinci_cfg_reg(DM365_SD1_CMD);
	davinci_cfg_reg(DM365_SD1_DATA3);
	davinci_cfg_reg(DM365_SD1_DATA2);
	davinci_cfg_reg(DM365_SD1_DATA1);
	davinci_cfg_reg(DM365_SD1_DATA0);

	davinci_setup_mmc(1, &jumbo_mmc_config[1]);

	return;
}
#endif

static int lago_mmc_get_ro(int module)
{
	return gpio_get_value(SD_WR_PR);
}

static struct davinci_mmc_config lago_mmc_config = {
	/* .get_cd is not defined since it seems it useless... the MMC
	 * controller detects anyway the card! O_o
	 */
	.get_ro		= lago_mmc_get_ro,
	.wires		= 4,
	.max_freq	= 50000000,
	.caps		= MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED,
	.version	= MMC_CTLR_VERSION_2,
};

static void lago_mmc_configure(void)
{
	int ret;

	davinci_cfg_reg(DM365_MMCSD0);
	davinci_cfg_reg(DM365_GPIO43); /* Not connected to the eMMC */

	ret = gpio_request(SD_WR_PR, "MMC WP");
	if (ret)
		pr_warning("lago: cannot request GPIO %d!\n", SD_WR_PR);
	gpio_direction_input(SD_WR_PR);
	davinci_setup_mmc(0, &lago_mmc_config);

	return;
}

static void lago_usb_configure(void)
{
	pr_notice("Launching setup_usb\n");
	setup_usb(500, 8);
}

/* I2C 7bit Adr */
static struct i2c_board_info __initdata lago_i2c_info[] = {
	{	/* RTC */
		I2C_BOARD_INFO("pcf8563", 0x51),
		/* .irq = IRQ_DM365_GPIO0_4, */
	}, {	/* EEprom */
		I2C_BOARD_INFO("24c256", 0x53),
		.platform_data = &at24_info,
	},
};

static struct davinci_i2c_platform_data i2c_pdata = {
	.bus_freq = 400 /* kHz */ ,
	.bus_delay = 0 /* usec */ ,
};

static void __init lago_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, lago_i2c_info, ARRAY_SIZE(lago_i2c_info));
}

static struct platform_device *lago_devices[] __initdata = {
	&lago_hwmon_device,
	&lago_irq_gpio_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1),
};

static void __init lago_map_io(void)
{
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

static struct spi_board_info lago_spi_info[] __initconst = {
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

int read_hw_vers()
{
	void __iomem *adc_mem;
	u32 regval, index;

	index = 1;
	adc_mem = ioremap(DM365_ADCIF_BASE, SZ_1K);
	__raw_writel(1 << index, adc_mem + CHSEL);
	regval = ADCTL_SCNIEN | ADCTL_SCNFLG;
	__raw_writel(regval, adc_mem + ADCTL);
	regval |= ADCTL_START;
	__raw_writel(regval, adc_mem + ADCTL);
	do { } while (__raw_readl(adc_mem + ADCTL) & ADCTL_START);
	regval = __raw_readl(adc_mem + AD_DAT(index));
	return lookup_resistors(regval);
}

static void lago_late_init(unsigned long data)
{
	int status;

	del_timer(&startup_timer);

	/* setting /proc/cpuinfo hardware_version information */
	system_rev = read_hw_vers();

	/* system_serial_low & system_serial_high can also be set here*/
	INIT_WORK(&late_init_work, lago_powerfail_configure);
	schedule_work(&late_init_work);

	/* uart for expansion */
	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
	/* gpio_direction_input(LONG_RESET_BUTTON); */
}

static __init void lago_init(void)
{
	pr_warning("Board_Init: START\n");
	if (lago_debug > 1)
		pinmux_check();

	lago_gpio_configure();
	lago_led_init();

	/* usart for pic */
	davinci_serial_init(&uart_config);
	mdelay(1);
	pr_warning("starting ...\n");

	/* I2C */
	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	lago_init_i2c();

	lago_emac_configure();

	/* SPI */
	dm365_init_spi0(0, lago_spi_info, ARRAY_SIZE(lago_spi_info));

	lago_mmc_configure();

	lago_usb_configure();

	platform_add_devices(lago_devices, ARRAY_SIZE(lago_devices));

	dm365_init_rtc();
	dm365_init_adc(&lago_adc_data);

	init_timer(&startup_timer);
	startup_timer.function = lago_late_init;
	startup_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_LATE_INIT);
	add_timer(&startup_timer);

	if (lago_debug > 1)
		pinmux_check();
	pr_warning("Init: DONE\n");
}

MACHINE_START(LAGO, "BTicino LAGO board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = lago_map_io,
	.init_irq = davinci_irq_init,
	.timer = &davinci_timer,
	.init_machine = lago_init,
MACHINE_END

/* End Of File -------------------------------------------------------------- */
