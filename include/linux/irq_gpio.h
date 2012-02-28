
#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H
/*void __init basi_gpio_init_irq(void);*/

enum irq_on_gpio_type {
	LEVEL,
	EDGE,
};

enum irq_on_gpio_mode {
	GPIO_EDGE_RISING = 1<<0,
	GPIO_EDGE_FALLING = 1<<1,
};

struct irq_on_gpio {
	unsigned int gpio;
	unsigned int irq;
	enum irq_on_gpio_type type;
	enum irq_on_gpio_mode mode;
};

struct irq_gpio_platform_data {
	struct irq_on_gpio *gpio_list;
	int len;
	int gpio_common;
	int irq_gpio;
};

#endif
