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

#ifndef __TM035KBH02_H
#define __TM035KBH02_H

struct tm035kbh02_platform_data{
	int model;	/* see enum downwords */
	int reset_n;	/* GPIO for Reset connection */
	int shutdown;	/* GPIO for Shutdown connection */
	int check_presence; /* 1: check Displ presence - 0: don't check */
	int lcd_present; /* GPIO where to read lcd presence */
	int HV_inversion;
};

/*void tm035kbh02_enable(void);
void tm035kbh02_disable(void);
int tm035kbh02_is_suspended(void);
*/

/* Available part numbers supported
 * TO add a new device
 *   - add here
 *   - add control inside probe-function
 *   - add registers & their usage
 */
enum part_number {
	TM035KBH02,
};


struct tm035kbh02{
	int model; /* see enum downwords */
	int disabled;
	int is_suspended;
	struct spi_device *spi;
	int reset_n; /* GPIO for Reset connection */
	int shutdown; /* GPIO for Shutdown connection */
	int HV_inversion; /* Horizontal&vertical inversion */
	int contrast;
};


#endif
