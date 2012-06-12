/*
 *  Author:     Raffaele Recalcati
 *  Created:    Nov 16, 2010
 *  Copyright:  Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_LAGO_H
#define ASM_ARCH_LAGO_H

/*
 * Protecting SPI chip select
 */
#define BOOT_FL_WP		45

/*
 * PIC_RESET_N
 */
#define PIC_RESET_N		38

/*
 * OC2_VBUS_USB
 */
#define OC2_VBUS_USB		39

/*
 * LED_1
 */
#define LED_1			40

/*
 * SD_CARD_DET
 */
#define SD_CARD_DET		41

/*
 * EEPROM wite protect
 */
#define E2_WPn			97

/*
 * RESET pic SCS AV
 */
#define REG_ERR_AVn		100

/*
 * SD card write protect
 */
#define SD_WR_PR		43

/*
 * Enable external RTC powering - Reset for echo canceller
 */
#define EN_RTC			99

/*
 * Power fail
 */
#define POWER_FAIL		50

/*
 * Push button in
 */
#define PB_IN			96

/*
 * eMMC reset
 */
#define EMMC_RESET		42

/*
 * Eth phy reset
 */
#define ENET_RESETn		44

#endif /* ASM_ARCH_LAGO_H */

