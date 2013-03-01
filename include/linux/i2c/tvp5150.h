#ifndef __LINUX_I2C_TVP5150_H
#define __LINUX_I2C_TVP5150_H

struct tvp5150_platform_data {
	int pdn;
	void (*resetb)(void);
};

#endif /* __LINUX_I2C_TVP5150_H */
