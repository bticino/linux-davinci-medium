/*
 *  Author:     Raffaele Recalcati
 *  Created:    Nov 16, 2010
 *  Copyright:  Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_BASI_H
#define ASM_ARCH_BASI_H

/*
 * Protecting SPI chip select  
 */
#define BOOT_FL_WP		45

/*
 * Audio modulator Enable on external connector
 */
#define EN_AUDIO		36

/*
 * Enable video demodulator
 */
#define ABIL_DEM_VIDEO		98

/*
 * Pal-Decoder power down
 */
#define PDEC_PWRDNn		37

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
 * Pal-decoder reset
 */
#define PDEC_RESETn		26

/*
 * EEPROM wite protect
 */
#define E2_WPn			97

/*
 * Reset PIC on the bottom board
 */
#define PIC_RESET		38

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
 * Zarlink chip select
 */
#define ZARLINK_CS		35

/*
 * Power fail
 */
#define POWER_FAIL		50

/*
 * Zarlink reset
 */
#define ZARLINK_RESET		69

/*
 * Push button in
 */
#define PB_IN			96

/*
 * eMMC reset
 */
#define EMMC_RESET		42

/*
 * RES_EXTUART
 */
#define RES_EXTUART		51


/*
 * Eth phy reset
 */
#define ENET_RESETn		44

#endif /* ASM_ARCH_BASI_H */

