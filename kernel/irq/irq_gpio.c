/*
 * Copyright (C) 2012 davide.bonfanti@bticino.it
 *               BTicino S.p.A.
 *
 * handler to manage gpio interrupts using a single gpio having
 * interrupts capabilities.
 *
 *    gpio_1      ______
 * ------------->|      |
 *    gpio_2     |      |
 * ------------->|      | gpio_interrupt
 *    ...        |  OR  |------------------->
 * ------------->|      |
 *    gpio_n     |      |
 * ------------->|_____ |
 *
 *  gpio_interrupt     ______
 * ------------------>|      |
 *    gpio_1          |      |
 * ------------------>|      |
 *    ...             |  CPU |
 * ------------------>|      |
 *    gpio_n          |      |
 * ------------------>|_____ |
 *
 *
 * When the gpio_interrupt occurs, all the input sources are watched
 * in order to discover the event occurred and the proper event (interrupt) is
 * dispatched (handle_irq).
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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/pm_loss.h>
#include <mach/gpio.h>
#include <linux/irq_gpio.h>
#include <linux/platform_device.h>

#define GPIO_DEBOUNCE_TO 10

struct irqgpio {
	struct irq_on_gpio *gpio_list;
	int len;
	int gpio_common;
	int irq_gpio;
	struct work_struct work;
	unsigned int pending_interrupts;
	struct timer_list pf_debounce_timer;
	unsigned long gpio_irq_enabled;
	unsigned int state;
};

static void mask_irq_gpio(unsigned int irq)
{
	int cnt;
	struct irqgpio *irqgpio = get_irq_data(irq);
	struct irq_on_gpio *pres_irq = irqgpio->gpio_list;

	for (cnt = 0; cnt < irqgpio->len; cnt++, pres_irq++) {
		if (pres_irq->irq == irq)
			break;
	}
	if (cnt < irqgpio->len)
		irqgpio->gpio_irq_enabled &= ~(1<<cnt);
}

static void unmask_irq_gpio(unsigned int irq)
{
	int cnt;
	struct irqgpio *irqgpio = get_irq_data(irq);
	struct irq_on_gpio *pres_irq = irqgpio->gpio_list;

	for (cnt = 0; cnt < irqgpio->len; cnt++, pres_irq++) {
		if (pres_irq->irq == irq)
			break;
	}
	if (cnt < irqgpio->len)
		irqgpio->gpio_irq_enabled |= (1<<cnt);
}

static struct irq_chip gpio_irq_chip = {
	.name		= "MULTI_GPIO_IRQ",
	.ack		= mask_irq_gpio,
	.mask		= mask_irq_gpio,
	.unmask		= unmask_irq_gpio,
};

static inline void test_io(struct irqgpio *irqgpio)
{
	unsigned int shift, cnt;
	struct irq_on_gpio *pres_irq = irqgpio->gpio_list;

	for (cnt = 0; cnt < irqgpio->len; cnt++, pres_irq++) {
		shift = 1<<cnt;
		if (!gpio_get_value(pres_irq->gpio)) {
			if ((irqgpio->gpio_irq_enabled & shift)
				&& (pres_irq->mode & GPIO_EDGE_FALLING)) {
				if ((pres_irq->type == LEVEL) ||
					!(irqgpio->pending_interrupts & shift))
					generic_handle_irq(pres_irq->irq);
			}
			irqgpio->pending_interrupts |= shift;
		} else if ((irqgpio->gpio_irq_enabled & shift) &&
			   (irqgpio->pending_interrupts & shift))
			irqgpio->pending_interrupts &= ~shift;
		else if ((irqgpio->gpio_irq_enabled & shift) &&
			 (pres_irq->mode & GPIO_EDGE_RISING))
				generic_handle_irq(pres_irq->irq);
	}
	if (gpio_get_value(irqgpio->gpio_common))
		mod_timer(&irqgpio->pf_debounce_timer, 0);
}

static void debounce_gpio_int(unsigned long data)
{
	struct irqgpio *irqgpio = (struct irqgpio *) data;
	if (gpio_get_value(irqgpio->gpio_common)) {
		irqgpio->pending_interrupts = 0;
		return;
	}

	test_io(irqgpio);
	mod_timer(&irqgpio->pf_debounce_timer, jiffies +
		msecs_to_jiffies(GPIO_DEBOUNCE_TO));
}

static void debounce_gpio_interrupts(struct work_struct *work)
{
	struct irqgpio *irqgpio = container_of(work, struct irqgpio, work);

	if (gpio_get_value(irqgpio->gpio_common))
		return;
	test_io(irqgpio);
	mod_timer(&irqgpio->pf_debounce_timer,
		jiffies + msecs_to_jiffies(GPIO_DEBOUNCE_TO));
}

static void gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	unsigned int cnt;
	unsigned int shift;
	struct irqgpio *irqgpio = get_irq_data(irq);
	struct irq_on_gpio *pres_irq;

	desc->chip->ack(irq);
	desc->chip->mask(irq);

	mod_timer(&irqgpio->pf_debounce_timer, 0);
	if (gpio_get_value(irqgpio->gpio_common)) {
		desc->chip->unmask(irq);
		return;
	}
	pres_irq = irqgpio->gpio_list;

	irqgpio->pending_interrupts = 0;
	for (cnt = 0; cnt < irqgpio->len; cnt++, pres_irq++) {
		shift = 1<<cnt;
		if (!gpio_get_value(pres_irq->gpio) &&
		   (irqgpio->gpio_irq_enabled & shift) &&
		   (pres_irq->mode & GPIO_EDGE_FALLING)) {
			generic_handle_irq(pres_irq->irq);
			irqgpio->pending_interrupts |= shift;
			if (!test_bit(WORK_STRUCT_PENDING,
					work_data_bits(&irqgpio->work)))
				schedule_work(&irqgpio->work);
			break;
		} else
			irqgpio->pending_interrupts &= ~shift;
	}
	desc->chip->unmask(irq);
}

static int __devinit irqgpio_probe(struct platform_device *dev)
{
	struct irq_gpio_platform_data *pdata = dev->dev.platform_data;
	struct irqgpio *irqgpio;
	struct irq_on_gpio *pres_irq;
	int cnt, irq;

	irqgpio = kzalloc(sizeof *irqgpio, GFP_KERNEL);
	if (!irqgpio)
		return -ENOMEM;

	platform_set_drvdata(dev, irqgpio);
	irqgpio->gpio_list = pdata->gpio_list;
	irqgpio->len = pdata->len;
	irqgpio->gpio_common = pdata->gpio_common;
	irqgpio->irq_gpio = pdata->irq_gpio;
	irqgpio->gpio_irq_enabled = 0;
	irqgpio->pending_interrupts = 0;
	irqgpio->state = 1;

	pres_irq = irqgpio->gpio_list;
	for (cnt = 0; cnt < irqgpio->len; cnt++, pres_irq++) {
		irq = pres_irq->irq;
		set_irq_chip(irq, &gpio_irq_chip);
		set_irq_data(irq, irqgpio);
		set_irq_handler(irq, handle_level_irq);
		set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
	}

	INIT_WORK(&irqgpio->work, debounce_gpio_interrupts);
	init_timer(&irqgpio->pf_debounce_timer);
	irqgpio->pf_debounce_timer.function = debounce_gpio_int;
	irqgpio->pf_debounce_timer.expires = 0;
	irqgpio->pf_debounce_timer.data = (unsigned long)irqgpio;
	set_irq_type(irqgpio->irq_gpio, IRQ_TYPE_EDGE_FALLING);
	set_irq_data(irqgpio->irq_gpio, irqgpio);
	set_irq_chained_handler(irqgpio->irq_gpio, gpio_irq_handler);
	return 0;
}

static int __devexit irqgpio_remove(struct platform_device *dev)
{
	struct irqgpio *irqgpio = platform_get_drvdata(dev);
	struct irq_desc *desc = irq_to_desc(irqgpio->irq_gpio);

	desc->chip->disable(irqgpio->irq_gpio);
	del_timer(&irqgpio->pf_debounce_timer);
	return 0;
}

static struct platform_driver irqgpio_platform_driver = {
	.driver = {
		.name   = "irq_gpio",
		.owner  = THIS_MODULE,
	},
	.probe          = irqgpio_probe,
	.remove         = irqgpio_remove,
};

static int __init irqgpio_init(void)
{
	int retval = 0;

	retval = platform_driver_register(&irqgpio_platform_driver);
	return retval;
}

static void __exit irqgpio_exit(void)
{
	platform_driver_unregister(&irqgpio_platform_driver);
}

module_init(irqgpio_init);
module_exit(irqgpio_exit);

MODULE_DESCRIPTION("multi gpio interrupts");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Davide Bonfanti");
