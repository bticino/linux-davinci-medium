#ifndef __LINUX_I2C_MC44CC373_H
#define __LINUX_I2C_MC44CC373_H

struct mc44cc373_platform_data {
        u8 num_par;
	u8 *pars;
        int power;
};

#endif /* __LINUX_I2C_MC44CC373_H */

