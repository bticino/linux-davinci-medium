/*
 *  Author:     Davide Bonfanti
 *  Created:    Aug 27, 2011
 *  Copyright:  Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_OWL_H
#define ASM_ARCH_OWL_H

/*
 * Protecting SPI chip select
 */
#define BOOT_FL_WP		45

/*
 * DEBUG GPIO on TP30
 */
#define DEBUG_GPIO1		94

/*
 * Ethernet Reset
 */
#define ENET_RESETn		44

/*
 * Timer keeper interrupt
 */
#define TMK_INTn		85

/*
 * EEPROM Write protect
 */
#define E2_WP			40

/*
 * Early Advise of occurring power down
 */
#define POWER_FAIL		50

/*
 * CMOS Reset
 */
#define CMOS_RSTBn		48

/*
 * CMOS Shutdown
 */
#define CMOS_PWDN		80


#endif /* ASM_ARCH_OWL_H */
