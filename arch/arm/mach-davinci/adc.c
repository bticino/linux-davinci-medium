/*
 *
 * Copyright (C) 2010 Bticino S.p.a
 * Author: Davide Bonfanti <davide.bonfanti@bticino.it>
 *
 * Contributors:
 *     Raffaele Recalcati <raffaele.recalcati@bticino.it>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/adc.h>

#define INTMUX 0x18

u32 davinci_adc_read_oneshot(struct davinci_adc_platform_data *adc_data, u32 index)
{
	static void __iomem *adc_mem = NULL;
	u32 regval;

	if (!adc_mem){
		adc_mem = ioremap(DM365_ADCIF_BASE, SZ_1K);
	}
	__raw_writel(1 << index, adc_mem + CHSEL);
	regval = ADCTL_SCNIEN | ADCTL_SCNFLG;
	__raw_writel(regval, adc_mem + ADCTL);
	regval |= ADCTL_START;
	__raw_writel(regval, adc_mem + ADCTL);
	try_wait_for_completion(&adc_data->done);

	wait_for_completion_timeout(&adc_data->done, msecs_to_jiffies(50));
	regval = __raw_readl(adc_mem + AD_DAT(index));
	return regval;
}
EXPORT_SYMBOL_GPL(davinci_adc_read_oneshot);


static irqreturn_t davinci_adc_isr(int irq, void *data)
{
	struct davinci_adc_platform_data *adc_data = data;

	complete(&adc_data->done);

	return IRQ_HANDLED;
}

static int __devinit davinci_adc_probe(struct platform_device *pdev)
{
	struct davinci_adc_platform_data *adc_pl_data = pdev->dev.platform_data;
	int ret;
	static void __iomem *arm_sys_mem;
	u32 regval;

	arm_sys_mem = ioremap(DAVINCI_SYSTEM_MODULE_BASE, SZ_1K);
	__raw_writel(__raw_readl(arm_sys_mem + INTMUX) | 1<<21, arm_sys_mem + INTMUX);
	iounmap(arm_sys_mem);
	init_completion(&adc_pl_data->done);
	dev_set_drvdata(&pdev->dev, adc_pl_data);

	if (request_irq(IRQ_DM365_ADCINT, davinci_adc_isr, IRQF_SHARED, "davinci-adc",
		adc_pl_data)) {
		printk(KERN_ERR "ADC: ERROR retrieving interrupt service routine\n");
		return -EPERM;
	}
	return 0;
}

static int __devexit davinci_adc_remove(struct platform_device *pdev)
{
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static struct platform_driver davinci_adc_driver = {
	.driver = {
		.name = "davinci_adc",
		.owner = THIS_MODULE,
	},
	.remove = __devexit_p(davinci_adc_remove),
};

static int __init davinci_adc_init(void)
{
	return platform_driver_probe(&davinci_adc_driver, davinci_adc_probe);
}
module_init(davinci_adc_init);

static void __exit davinci_adc_exit(void)
{
	platform_driver_unregister(&davinci_adc_driver);
}
module_exit(davinci_adc_exit);

MODULE_AUTHOR("Davide Bonfanti");
MODULE_DESCRIPTION("Texas Instruments DaVinci ADC Core Interface");
MODULE_LICENSE("GPL");

