/*
 *  Author:     Davide Bonfanti
 *  Created:    Feb 4, 2011
 *  Copyright:  Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_DINGO_H
#define ASM_ARCH_DINGO_H

/*
 * Protecting SPI chip select
 */
#define BOOT_FL_WP		45

/*
 * Audio Enable towards external connector
 */
#define EN_AUDIO		35

/*
 * DEBUG GPIO on TP30
 */
#define DEBUG_GPIO1		95

/*
 * DEBUG GPIO on TP31
 */
#define DEBUG_GPIO2		94

/*
 * Reset of LCD (Active Low)
 */
#define LCD_RESETn		96

/*
 * PIC_RESET_N
 */
#define PIC_RESET_N		99

/*
 * LCD Shutdown
 */
#define LCD_SHTDWNn		98

/*
 * Resistive touch interface irq signal
 */
#define PENIRQn			97

/*
 * Touch-Interface chip select for SPI control bus -- used as peripheral!
 */
#define TOUCH_CSn		101

/*
 * Enable LCD backlght
 */
#define EN_RETROILL		23

/*
 * Ethernet Reset
 */
#define ENET_RESETn		44

/*
 * Timer keeper interrupt
 */
#define TMK_INTn		26

/*
 * GIO routed on LCD connector for possible future connection
 */
#define LCD_GPIO		36

/*
 * Audio Deemphasis
 */
#define AUDIO_DEEMP		70

/*
 * Audio Reset
 */
#define AUDIO_RESET		69

/*
 * EEPROM Write protect
 */
#define E2_WP			40

/*
 * Enable external RTC power supply
 */
#define EN_RTC			38

/*
 * Touch interface busy
 */
#define TOUCH_BUSY		74

/*
 * LCD Chip Select for SPI connection
 */
#define LCD_CSn			68

/*
 * Early Advise of occurring power down
 */
#define POWER_FAIL		50

/*
 * OC2_VBUS_USB
 */
#define OC2_VBUS_USB		39

/*
 * ------
 * Additional definitions for possible expansions
 */

/*
 * Reset External UART Interface
 */
#define RES_EXTUART		51

/*
 * Interrupt signalling for external UART
 */
#define INT_UART		72

/*
 * GPIO routed on zif connector
 */
#define ExpansionGPIO		43

/*
 * Transmission/Receive for 485 interface (connected on ZIF connector)
 */
#define RX_TX485		42

#endif /* ASM_ARCH_DINGO_H */
