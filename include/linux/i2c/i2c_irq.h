#ifndef __LINUX_I2C_IRQ_H
#define __LINUX_I2C_IRQ_H

struct i2c_irq_platform_data {
	unsigned int gpio;	/* gpio used for mux */
	unsigned int irq;	/* interrupt number */
};

#endif /* __LINUX_I2C_IRQ_H */

