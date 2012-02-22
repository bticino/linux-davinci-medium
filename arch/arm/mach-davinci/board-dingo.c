/*
 * BTicino S.p.A. dingo platform support
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
#include <mach/adc.h>
#include <video/davincifb.h>
#include <linux/pm_loss.h>

#include <mach/dingo.h>

#include <media/davinci/vid_encoder_types.h>
#include <media/davinci/davinci_enc_mngr.h>

#define DM365_ASYNC_EMIF_CONTROL_BASE   0x01d10000
#define DM365_ASYNC_EMIF_DATA_CE0_BASE  0x02000000
#define DM365_ASYNC_EMIF_DATA_CE1_BASE  0x04000000
#define DM365_EVM_PHY_MASK              (0x2)
#define DM365_EVM_MDIO_FREQUENCY        (2200000)	/* PHY bus frequency */
#define HW_IN_CLOCKOUT2_UDA_I2S

#define NAND_BLOCK_SIZE         SZ_128K

#define DINGO_PHY_MASK              (0x2)
#define DINGO_MDIO_FREQUENCY        (2200000) /* PHY bus frequency */

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
#define GPIO_DEBOUNCE_TO 10
#define TIME_TO_LATE_INIT 1500

static int dingo_debug = 1;
module_param(dingo_debug, int, 0644);
MODULE_PARM_DESC(dingo_debug, "Debug level 0-1");

extern unsigned int system_rev;
extern int lookup_resistors(int cnt);

static void dingo_bl_set_intensity(int level);

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

static struct mtd_partition dingo_nand_partitions[] = {
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

static struct davinci_aemif_timing dingo_nandflash_timing = {
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
	.parts		= dingo_nand_partitions,
	.nr_parts	= ARRAY_SIZE(dingo_nand_partitions),
	.ecc_mode	= NAND_ECC_HW,
	.options	= NAND_USE_FLASH_BBT,
	.ecc_bits	= 4,
	.timing		= &dingo_nandflash_timing,
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

void dingo_phy_power(int on)
{
	gpio_set_value(ENET_RESETn, on);
	udelay(100);
}

static void dingo_emac_configure(void)
{
	struct davinci_soc_info *soc_info = &davinci_soc_info;

	soc_info->emac_pdata->phy_mask = DINGO_PHY_MASK;
	soc_info->emac_pdata->mdio_max_freq = DINGO_MDIO_FREQUENCY;
	soc_info->emac_pdata->power = dingo_phy_power;

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

static void dingo_lcd_configure(void)
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

static struct davinci_adc_platform_data dingo_adc_data = {
	.adc_configuration = ADC_CH0_EANBLE | ADC_CH1_EANBLE | ADC_CH2_EANBLE |
			     ADC_CH3_EANBLE | ADC_CH4_EANBLE | ADC_CH5_EANBLE |
			     ADC_CH0_ONESHOOT | ADC_CH1_ONESHOOT |
			     ADC_CH2_ONESHOOT | ADC_CH3_ONESHOOT |
			     ADC_CH4_ONESHOOT | ADC_CH5_ONESHOOT,
};

static struct platform_device dingo_hwmon_device = {
	.name = "bt_nexmed_hwmon",
	.dev = {
		.platform_data = &dingo_adc_data,
		},
	.id = 0,
};

static u64 davinci_fb_dma_mask = DMA_BIT_MASK(32);

static struct davincifb_platform_data dingo_fb_platform_data = {
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
		.platform_data          = &dingo_fb_platform_data,
	},
	.num_resources = ARRAY_SIZE(davincifb_resources),
	.resource      = davincifb_resources,
};

static struct generic_bl_info dingo_bl_machinfo = {
	.max_intensity = 0xff,
	.default_intensity = 0xF0,
	.limit_mask = 0xff,
	.set_bl_intensity = dingo_bl_set_intensity,
	.name = "dingo_backlight",
};

static struct resource gpio_resources[] = {
	{
		.start          = 0x01c67000,
		.end            = 0x01c67000 + 0x7ff,
		.flags          = IORESOURCE_MEM,
	},
};

static struct platform_device dingo_bl_device = {
	.name   = "generic-bl",
	.dev    = {
		.platform_data  = &dingo_bl_machinfo,
	},
	.id             = -1,
	.num_resources  = ARRAY_SIZE(gpio_resources),
	.resource       = gpio_resources,
};

static void dingo_bl_set_intensity(int level)
{
	static void __iomem *base;
	static int bl_is_on;
	int tmp;

	if (!base)
		base = ioremap(DM365_PWM0_CONTROL_BASE, SZ_1K - 1);

	pr_info("set backlight lev:%d\n", level);

	level = level & 0xff;

	if (!level) {
		__raw_writeb(PWM_DISABLED << MODE, base + PWM_CFG);
		bl_is_on = 0;
		return;
	}
	if (!bl_is_on) {
		struct clk *pwm0_clk;

		davinci_cfg_reg(DM365_GPIO23);
		gpio_request(EN_RETROILL, "Backlight");
		gpio_direction_output(EN_RETROILL, 1);
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
		printk(KERN_ERR "pinmux%d=%X\n", i, pinmux[i]);
	}
}

#if 0
static struct plat_serial8250_port dingo_dm365_serial_platform_data[] = {
	{
		.mapbase        = DM365_ASYNC_EMIF_DATA_CE1_BASE,
		.irq            = IRQ_DM365_GPIO0_1,
		.type           = PORT_16550A,
		.flags          = UPF_BOOT_AUTOCONF | UPF_IOREMAP |
				  UART_CONFIG_TYPE | UPF_SKIP_TEST |
				  UPF_FIXED_TYPE,
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

static struct platform_device dingo_dm365_serial_device = {
	.name                   = "serial8250",
	.id                     = PLAT8250_DEV_PLATFORM1,
	.resource               = davinci_asyn_resources,
	.dev                    = {
		.platform_data  = dingo_dm365_serial_platform_data,
	},
};

static void dingo_uart2_configure(void)
{
	struct clk *aemif_clk;
	struct davinci_aemif_timing t;
	void __iomem *base;
	int ret;
	base = ioremap(DM365_ASYNC_EMIF_CONTROL_BASE, SZ_4K - 1);

	aemif_clk = clk_get(NULL, "aemif");
	if (IS_ERR(aemif_clk)) {
		pr_err("couldn't get AEMIF CLOCK!!! - ttyS2 won't work\n");
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
		pr_err("failed setting aemif timings - ttyS2 won't work");
		return;
	}

	/* setting proper pinmux for AEMIF */
	davinci_cfg_reg(DM365_AEMIF_AR2);
	davinci_cfg_reg(DM365_AEMIF_CE1);
	davinci_cfg_reg(DM365_EM_WE_OE);
	davinci_cfg_reg(DM365_GPIO51);
	gpio_direction_output(RES_EXTUART, 0);

	set_irq_type(gpio_to_irq(0), IRQ_TYPE_EDGE_BOTH);
	platform_device_register(&dingo_dm365_serial_device);
}
#endif

struct irq_on_gpio0 {
	unsigned int gpio;
	unsigned int irq;
};

static struct irq_on_gpio0 dingo_irq_on_gpio0[] = {
	{
		.gpio = INT_UART,
		.irq = IRQ_DM365_GPIO0_1,
	}, {
		.gpio = POWER_FAIL,
		.irq = IRQ_DM365_GPIO0_2,
	}, {
		.gpio = TMK_INTn,
		.irq = IRQ_DM365_GPIO0_3,
	}, {
		.gpio = PENIRQn,
		.irq = IRQ_DM365_GPIO0_4,
	},
};

static unsigned long dingo_gpio0_irq_enabled;

static void dingo_mask_irq_gpio0(unsigned int irq)
{
	int dingo_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	dingo_gpio0_irq_enabled &= ~(1 << dingo_gpio_irq);
}

static void dingo_unmask_irq_gpio0(unsigned int irq)
{
	int dingo_gpio_irq = (irq - IRQ_DM365_GPIO0_0);
	dingo_gpio0_irq_enabled |= (1 << dingo_gpio_irq);
}

static int dingo_set_wake_irq_gpio0(unsigned int irq, unsigned int on)
{
	return 0;
}

static struct irq_chip dingo_gpio0_irq_chip = {
	.name		= "DM365_GPIO0",
	.ack		= dingo_mask_irq_gpio0,
	.mask		= dingo_mask_irq_gpio0,
	.unmask		= dingo_unmask_irq_gpio0,
	.disable	= dingo_mask_irq_gpio0,
	.enable		= dingo_unmask_irq_gpio0,
	.bus_sync_unlock = dingo_unmask_irq_gpio0,
	.bus_lock	= dingo_mask_irq_gpio0,
	.set_wake	= dingo_set_wake_irq_gpio0,
};

static struct manage_gpio_interrupts {
	struct work_struct work;
	unsigned int pending_interrupts;
} manage_gpio_interrupts;

static struct manage_gpio_interrupts manage_gpio_int;

static void debounce_gpio_int(unsigned long data)
{
	gpio_set_value(DEBUG_GPIO1, 1);
	udelay(50);
	gpio_set_value(DEBUG_GPIO1, 0);
	unsigned int shift, cnt;

	if (gpio_get_value(0)) {
		del_timer(&pf_debounce_timer);
		manage_gpio_int.pending_interrupts = 0;
		return;
	}
	for (cnt = 0; cnt < ARRAY_SIZE(dingo_irq_on_gpio0); cnt++) {
		shift = 1<<(dingo_irq_on_gpio0[cnt].irq - IRQ_DM365_GPIO0_0);
		if (!gpio_get_value(dingo_irq_on_gpio0[cnt].gpio) &&
		   (dingo_gpio0_irq_enabled & shift) &&
		   !(manage_gpio_int.pending_interrupts & shift)) {
			manage_gpio_int.pending_interrupts |= shift;
			generic_handle_irq(dingo_irq_on_gpio0[cnt].irq);
		}
		if (gpio_get_value(dingo_irq_on_gpio0[cnt].gpio) &&
		   (dingo_gpio0_irq_enabled & shift) &&
		   (manage_gpio_int.pending_interrupts & shift)
		   ) {
			manage_gpio_int.pending_interrupts &= ~shift;
		}
	}
	mod_timer(&pf_debounce_timer, jiffies +
		msecs_to_jiffies(GPIO_DEBOUNCE_TO));
}

static void debounce_gpio_interrupts(struct work_struct *work)
{
	del_timer(&pf_debounce_timer);
	init_timer(&pf_debounce_timer);
	pf_debounce_timer.function = debounce_gpio_int;
	pf_debounce_timer.expires =
			jiffies + msecs_to_jiffies(GPIO_DEBOUNCE_TO);
	add_timer(&pf_debounce_timer);

}

static void dingo_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cnt;
	unsigned int shift;

	desc->chip->ack(irq);
	udelay(50);
	if (gpio_get_value(0)) {
		del_timer(&pf_debounce_timer);
		manage_gpio_int.pending_interrupts = 0;
		if (dingo_gpio0_irq_enabled & 0x01)
			generic_handle_irq(IRQ_DM365_GPIO0_0);
	} else {
		for (cnt = 0; cnt < ARRAY_SIZE(dingo_irq_on_gpio0); cnt++) {
			shift =
			   1<<(dingo_irq_on_gpio0[cnt].irq - IRQ_DM365_GPIO0_0);
			if (!gpio_get_value(dingo_irq_on_gpio0[cnt].gpio) &&
			   (dingo_gpio0_irq_enabled & shift)) {
				generic_handle_irq(dingo_irq_on_gpio0[cnt].irq);
				manage_gpio_int.pending_interrupts |= shift;
				schedule_work(&manage_gpio_int.work);
			} else
				manage_gpio_int.pending_interrupts &= ~shift;
		}
	}
	desc->chip->unmask(irq);
}

static void __init dingo_gpio_init_irq(void)
{
	int irq;

	davinci_irq_init();
	/* setup extra Basi irqs on GPIO0*/
	for (irq = IRQ_DM365_GPIO0_0;
	    irq <= IRQ_DM365_GPIO0_0 + ARRAY_SIZE(dingo_irq_on_gpio0); irq++) {
		set_irq_chip(irq, &dingo_gpio0_irq_chip);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	INIT_WORK(&manage_gpio_int.work, debounce_gpio_interrupts);
	set_irq_type(IRQ_DM365_GPIO0, IRQ_TYPE_EDGE_BOTH);
	set_irq_chained_handler(IRQ_DM365_GPIO0, dingo_gpio_irq_handler);
}

/* Power failure stuff */
#ifdef CONFIG_PM_LOSS

static int powerfail_status;

static irqreturn_t dingo_powerfail_stop(int irq, void *dev_id);

static irqreturn_t dingo_powerfail_quick_check_start(int irq, void *dev_id)
{
	dingo_mask_irq_gpio0(IRQ_DM365_GPIO0_2);
	dingo_unmask_irq_gpio0(IRQ_DM365_GPIO0_0);

	/* PowerFail situation - START: power is going away */
	return IRQ_WAKE_THREAD;
}

static irqreturn_t dingo_powerfail_start(int irq, void *dev_id)
{
	dingo_mask_irq_gpio0(IRQ_DM365_GPIO0_2);
	dingo_unmask_irq_gpio0(IRQ_DM365_GPIO0_0);
	if (powerfail_status)
		return IRQ_HANDLED;
	powerfail_status = 1;
	pm_loss_power_changed(SYS_PWR_FAILING);

	/* PowerFail situation - START: power is going away */

	return IRQ_HANDLED;
}

static irqreturn_t dingo_powerfail_quick_check_stop(int irq, void *dev_id)
{
	dingo_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	dingo_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);

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

	dingo_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	dingo_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);

	/* PowerFail situation - STOP: power is back */
}

static irqreturn_t dingo_powerfail_stop(int irq, void *dev_id)
{
	dingo_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	dingo_unmask_irq_gpio0(IRQ_DM365_GPIO0_2);
	powerfail_status = 0;
	pm_loss_power_changed(SYS_PWR_GOOD);

	return IRQ_HANDLED;
	/* PowerFail situation - STOP: power is coming back */
	init_timer(&pf_debounce_timer);
	pf_debounce_timer.function = debounce;
	pf_debounce_timer.expires =
			jiffies + msecs_to_jiffies(POWER_FAIL_DEBOUNCE_TO);
	add_timer(&pf_debounce_timer);

	return IRQ_HANDLED;
}

enum dingo_pwrfail_prio {
	DINGO_PWR_FAIL_PRIO_0,
	DINGO_PWR_FAIL_MIN_PRIO = DINGO_PWR_FAIL_PRIO_0,
	DINGO_PWR_FAIL_PRIO_1,
	DINGO_PWR_FAIL_PRIO_2,
	DINGO_PWR_FAIL_PRIO_3,
	DINGO_PWR_FAIL_MAX_PRIO = DINGO_PWR_FAIL_PRIO_3,
};

struct pm_loss_default_policy_item dingo_pm_loss_policy_items[] = {
	{
		.bus_name = "platform",
		.bus_priority = DINGO_PWR_FAIL_PRIO_1,
	},
};

#define DINGO_POLICY_NAME "dingo-default"

struct pm_loss_default_policy_table dingo_pm_loss_policy_table = {
	.name = DINGO_POLICY_NAME,
	.items = dingo_pm_loss_policy_items,
	.nitems = ARRAY_SIZE(dingo_pm_loss_policy_items),
};

static void dingo_powerfail_configure(void)
{
	int stat;
	struct pm_loss_policy *p;
	stat = request_threaded_irq(IRQ_DM365_GPIO0_2,
				    dingo_powerfail_quick_check_start,
				    dingo_powerfail_start,
				    0,
				    "pwrfail-on", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-on) "
		       "failed\n", IRQ_DM365_GPIO0_2);
	stat = request_threaded_irq(IRQ_DM365_GPIO0_0,
				    dingo_powerfail_quick_check_stop,
				    dingo_powerfail_stop, 0,
				    "pwrfail-off", NULL);
	if (stat < 0)
		printk(KERN_ERR "request_threaded_irq for IRQ%d (pwrfail-off) "
		       "failed\n", IRQ_DM365_GPIO0_0);
	dingo_mask_irq_gpio0(IRQ_DM365_GPIO0_0);
	p = pm_loss_setup_default_policy(&dingo_pm_loss_policy_table);

	if (!p)
		printk(KERN_ERR "Could not register pwr change policy\n");

	if (pm_loss_set_policy(DINGO_POLICY_NAME) < 0)
		printk(KERN_ERR "Could not set %s power loss policy\n",
		       DINGO_POLICY_NAME);

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
#define dingo_powerfail_configure()
#endif

static void dingo_gpio_configure(void)
{
	int status;
	void __iomem *pupdctl0 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x78);
	void __iomem *pupdctl1 = IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE + 0x7c);

	/* Configure (disable) pull down control */
	__raw_writel(0, pupdctl0);
	__raw_writel(0x40000, pupdctl1); /* EM_WAIT active pull-up */

	gpio_request(0, "GIO0");
	gpio_direction_input(0);

	davinci_cfg_reg(DM365_GPIO35);
	status = gpio_request(EN_AUDIO, "Audio Enable to external connector");
	if (status) {
		pr_err("%s: fail GPIO request: Audio Enable to"\
				" extern conn: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(EN_AUDIO, 0);

	davinci_cfg_reg(DM365_GPIO95);
	status = gpio_request(AUDIO_MUTE, "DEBUG GPIO on TP30");
	if (status) {
		pr_err("%s: fail GPIO request: Audio Mute %d\n",
				 __func__, status);
		return;
	}
	gpio_direction_output(AUDIO_MUTE, 0);

	davinci_cfg_reg(DM365_GPIO94);
	status = gpio_request(DEBUG_GPIO1, "DEBUG GPIO on TP31");
	if (status) {
		pr_err("%s: fail GPIO request: DEBUG GPIO on "\
				" TP31: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(DEBUG_GPIO1, 0);

	davinci_cfg_reg(DM365_GPIO96);
	status = gpio_request(LCD_RESETn, "Reset of LCD");
	if (status) {
		pr_err("%s: Reset of LCD: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(LCD_RESETn, 1);

	davinci_cfg_reg(DM365_GPIO99);
	status = gpio_request(PIC_RESET_N, "SCS PIC Reset");
	if (status) {
		pr_err("%s: fail GPIO request: SCS PIC Reset" \
					 " : %d\n", __func__, status);
		return;
	}
	gpio_direction_output(PIC_RESET_N, 1);

	davinci_cfg_reg(DM365_GPIO98);
	status = gpio_request(LCD_SHTDWNn, "LCD Shutdown");
	if (status) {
		pr_err("%s: failed to request GPIO: LCD Shutdown" \
				"  : %d\n", __func__, status);
		return;
	}
	gpio_direction_output(LCD_SHTDWNn, 1);

	davinci_cfg_reg(DM365_GPIO97);
	status = gpio_request(PENIRQn, "Resistive touch interface irq signal");
	if (status) {
		pr_err("%s: failed to request GPIO: Resistive touch " \
				"interface irq signal: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(PENIRQn);

#if 0
	davinci_cfg_reg(DM365_GPIO101);
	status = gpio_request(TOUCH_CSn, "Touch-Intf chip select for SPI bus");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO: Touch interface \
				 chip select for SPI bus: %d\n", __func__,
				 status);
		return;
	}
	gpio_direction_output(TOUCH_CSn, 1); /* enabled */

	davinci_cfg_reg(DM365_GPIO23);
	status = gpio_request(EN_RETROILL, "Enable LCD backlght");
	if (status) {
		printk(KERN_ERR "%s: failed to request GPIO: Enable LCD \
				 backlight: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(EN_RETROILL, 1);

#endif
	davinci_cfg_reg(DM365_GPIO26);
	status = gpio_request(TMK_INTn, "Timer keeper interrupt");
	if (status) {
		pr_err("%s: failed to request GPIO: Timer Keeper " \
				"interrupt: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(TMK_INTn);

	davinci_cfg_reg(DM365_GPIO36);
	status = gpio_request(LCD_GPIO, "GIO routed on LCD connector");
	if (status) {
		pr_err("%s: failed to request GPIO: GIO routed on " \
				"LCD Connector: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(LCD_GPIO);

	davinci_cfg_reg(DM365_AEMIF_AR0);
	status = gpio_request(AUDIO_DEEMP, "Audio Deemphasis");
	if (status) {
		pr_err("%s: failed to request GPIO: Audio Deemphasis " \
				": %d\n", __func__, status);
		return;
	}
	gpio_direction_output(AUDIO_DEEMP, 1);

	/* davinci_cfg_reg(DM365_GPIO69); */
	status = gpio_request(AUDIO_RESET, "Audio Reset");
	if (status) {
		pr_err("%s: failed to request GPIO: Audio Reset " \
				": %d\n", __func__, status);
		return;
	}
	gpio_direction_output(AUDIO_RESET, 1);

	davinci_cfg_reg(DM365_GPIO40);
	status = gpio_request(E2_WP, "EEPROM Write protect");
	if (status) {
		pr_err("%s: failed to request GPIO: EEPROM Write " \
				"protect: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(E2_WP, 1);

	davinci_cfg_reg(DM365_GPIO38);
	status = gpio_request(EN_RTC, "Enable external RTC power supply");
	if (status) {
		pr_err("%s: failed to request GPIO: Enable external " \
				"RTC power supply %d\n", __func__, status);
		return;
	}
	gpio_direction_output(EN_RTC, 1);

	davinci_cfg_reg(DM365_GPIO44);
	status = gpio_request(ENET_RESETn, "ENET RESET");
	if (status) {
		pr_err("%s: failed to request ENET " \
				"RESET %d\n", __func__, status);
		return;
	}
	gpio_direction_output(ENET_RESETn, 1);

	/* davinci_cfg_reg(DM365_GPIO74); */
	status = gpio_request(TOUCH_BUSY, "Touch interface busy");
	if (status) {
		pr_err("%s: failed to request GPIO: Touch interface " \
				"busy: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(TOUCH_BUSY);

	davinci_cfg_reg(DM365_GPIO68);
	status = gpio_request(LCD_CSn, "LCD Chip Select for SPI bus");
	if (status) {
		pr_err("%s: failed to request GPIO: LCD Chip Select " \
				"for SPI bus: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(LCD_CSn, 1);

	davinci_cfg_reg(DM365_GPIO50);
	status = gpio_request(POWER_FAIL, "Early advise of power down");
	if (status) {
		pr_err("%s: failed to request GPIO: early advise of " \
				"power down: %d\n", __func__, status);
		return;
	}
	gpio_direction_input(POWER_FAIL);


	davinci_cfg_reg(DM365_GPIO39);
	status = gpio_request(OC2_VBUS_USB, "OC2_VBUS_USB");
	if (status) {
		pr_err("%s: failed to request GPIO: OC2_VBUS_USB " \
				": %d\n", __func__, status);
		return;
	}
	gpio_direction_input(OC2_VBUS_USB);

	if (dingo_debug) {
		gpio_export(BOOT_FL_WP, 0); /* danger */
		gpio_export(EN_AUDIO, 0);
		gpio_export(DEBUG_GPIO1, 0);
		gpio_export(LCD_RESETn, 0);
		gpio_export(LCD_SHTDWNn, 0);
		gpio_export(PIC_RESET_N, 0);
		gpio_export(E2_WP, 0);
		gpio_export(PENIRQn, 0);
		gpio_export(EN_RTC, 0);
		gpio_export(TOUCH_CSn, 0);
		gpio_export(OC2_VBUS_USB, 0);
		gpio_export(TMK_INTn, 0);
		gpio_export(POWER_FAIL, 0);
		gpio_export(LCD_GPIO, 0);
		gpio_export(AUDIO_DEEMP, 0);
		gpio_export(AUDIO_RESET, 0);
		gpio_export(AUDIO_MUTE, 0);
		gpio_export(TOUCH_BUSY, 0);
		gpio_export(LCD_CSn, 0);
		gpio_export(23, 0);
	}
}

static void dingo_usb_configure(void)
{
	/* pr_notice("Launching setup_usb\n"); */
	setup_usb(500, 8);
}

static struct platform_device dingo_asoc_device[] = {
	{
		.name = "dingo-asoc",
		.id = 0,
	},
	{
		.name = "dingo-asoc",
		.id = 1,
	},
};

static struct i2c_board_info __initdata dingo_i2c_info[] = {
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

static void __init dingo_init_i2c(void)
{
	davinci_init_i2c(&i2c_pdata);
	i2c_register_board_info(1, dingo_i2c_info, ARRAY_SIZE(dingo_i2c_info));
}

static struct platform_device *dingo_devices[] __initdata = {
	&davinci_nand_device,
	&dingo_bl_device,
	&dingo_asoc_device[0],
	&dingo_asoc_device[1],
	&dingo_hwmon_device,
	&davinci_fb_device,
};

static struct davinci_uart_config uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1), /* | (1 << 2), */
};

static void __init dingo_map_io(void)
{
	dm365_init();
}

static struct tm035kbh02_platform_data dingo_tm035kbh02_info = {
	.reset_n = LCD_RESETn,
	.shutdown = LCD_SHTDWNn,
	.check_presence = 0,
	.lcd_present = -1,
	.model = TM035KBH02,
	.HV_inversion = -1,
	.time_refr_regs = 10,
};

int dingo_get_pendown_state(void)
{
	return !gpio_get_value(PENIRQn);
}

static struct ads7846_platform_data dingo_ads7846_info = {
	.model = 7846,
	.x_max = 0x0fff,
	.y_max = 0x0fff,
	.x_plate_ohms = 600,
	.y_plate_ohms = 350,
	.keep_vref_on = 1,
	.settle_delay_usecs = 500,
	.penirq_recheck_delay_usecs = 100,

	.debounce_tol = 50,
	.debounce_max = 20,
	.debounce_rep = 3,
	.sampling_period = 50 * 1000 * 1000,
	.pressure_max = 1024,

	.vref_mv = 3300,
	.get_pendown_state = dingo_get_pendown_state,
};

static struct spi_board_info dingo_spi3_info[] = {
	{
		.modalias = "ads7846",
		.platform_data = &dingo_ads7846_info,
		.max_speed_hz = 2000000, /* Max 2.5MHz for the chip*/
		.bus_num = 3,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.irq = IRQ_DM365_GPIO0_4,
	},
	{
		.modalias   = "tm035kbh02",
		.platform_data  = &dingo_tm035kbh02_info,
		.controller_data = (void *)LCD_CSn, /* bitbang chip select */
		.max_speed_hz   = 1000000, /* Max 3.215 MHz */
		.bus_num = 3,
		.chip_select = 1,
		.mode = SPI_MODE_0,
	},
};

static struct snd_platform_data dm365_dingo_snd_data[] = {
	{
	},
	{
		.eventq_no = EVENTQ_3,
		.i2s_accurate_sck = 1,
		.clk_input_pin = MCBSP_CLKS,
	},
};

struct device my_device;

static void dingo_late_init(unsigned long data)
{
	void __iomem *adc_mem;
	int status;
	u32 regval, index;

	del_timer(&startup_timer);
	davinci_cfg_reg(DM365_GPIO45);
	status = gpio_request(BOOT_FL_WP, "Protecting SPI0 chip select");
	if (status) {
		pr_err("%s: fail GPIO request: Protecting SPI0" \
				" chip select: %d\n", __func__, status);
		return;
	}
	gpio_direction_output(BOOT_FL_WP, 1);

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

	davinci_cfg_reg(DM365_UART1_RXD_34);
	davinci_cfg_reg(DM365_UART1_TXD_25);
	gpio_direction_output(DEBUG_GPIO1, 1);
	mdelay(2);
	gpio_direction_output(DEBUG_GPIO1, 0);
	gpio_direction_output(DEBUG_GPIO1, 1);
	mdelay(2);
	gpio_direction_output(DEBUG_GPIO1, 0);
}

static __init void dingo_init(void)
{
	pr_warning("Dingo_init: START\n");
	if (dingo_debug > 1)
		pinmux_check();
	dingo_gpio_configure();

	davinci_serial_init(&uart_config);
	pr_warning("dingo_init: starting ...\n");

	davinci_cfg_reg(DM365_I2C_SDA);
	davinci_cfg_reg(DM365_I2C_SCL);
	dingo_init_i2c();
	dingo_emac_configure();
	dingo_lcd_configure();

	dm365_init_spi3(BIT(0), dingo_spi3_info, ARRAY_SIZE(dingo_spi3_info));

	dingo_usb_configure();
	/* dingo_uart2_configure(); */

	dingo_powerfail_configure();
	davinci_cfg_reg(DM365_CLKOUT2);
	dm365_init_vc(&dm365_dingo_snd_data[0]);
	dm365_init_asp(&dm365_dingo_snd_data[1]);
	davinci_cfg_reg(DM365_GPIO44);

	platform_add_devices(dingo_devices, ARRAY_SIZE(dingo_devices));

	dm365_init_rtc();
	dm365_init_adc(&dingo_adc_data);

	init_timer(&startup_timer);
	startup_timer.function = dingo_late_init;
	startup_timer.expires =
			jiffies + msecs_to_jiffies(TIME_TO_LATE_INIT);
	add_timer(&startup_timer);

	if (dingo_debug > 1)
		pinmux_check();
	pr_warning("dingo_init: DONE\n");
}

MACHINE_START(DINGO, "bticino DINGO board")
	.phys_io = IO_PHYS,
	.io_pg_offst = (__IO_ADDRESS(IO_PHYS) >> 18) & 0xfffc,
	.boot_params = (0x80000100),
	.map_io = dingo_map_io,
	.init_irq = dingo_gpio_init_irq,
	/* .init_irq = davinci_irq_init, */
	.timer = &davinci_timer,
	.init_machine = dingo_init,
MACHINE_END
