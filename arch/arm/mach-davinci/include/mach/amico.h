/*
 *  Created:    Aug 22, 2012
 *  Copyright:  Shidean.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_AMICO_H
#define ASM_ARCH_AMICO_H

/* -- IRQ (GPIO) ------------------------------------------------------------ */
#define piINTERRUPT		0	/* Interrup Pin */
#define piPOWER_FAILn	50	/* IRQ_PowerFail */
#define piINT_I2Cn		35	/* I2C interrupt between DM365 and MCU*/
#define piTMK_INTn		62	/* IRQ_RTC */
#define piPENIRQn		64	/* IRQ_TouchScreen */
#define piGPIO_INTn		63	/* GPIO interrupt for the future*/

/* -- GPIO ------------------------------------------------------------------ */
#define poBL_PWM			23  /* Backlight pwm control */
#define poAUDIO_SEP_ZL		41	/* AUDIO SEP ZALINK */
#define poNAND_WPn			36	/* Nand flash write protect */
#define poENET_RESETn		44	/* ETH phy reset */
#define poBOOT_FL_WPn		45	/* Protecting SPI chip select */
#define poENABLE_VIDEO_IN	58	/* Enable DEM_VIDEO */
#define poPDEC_PWRDNn		39	/* PalDecoder PWRDown */
#define poPDEC_RESETn       38     /* PAL decoder Reset */
#define poZARLINK_CS		101	/* Zarlink chip select */
#define poZARLINK_RESET		76	/* Zarlink reset */
#define poZARLINK_PWR		40	/* Zarlink power enable */
#define poPIC_RESETn		99	/* PIC AV & AI reset */
#define piSD_DETECTn		96	/* Detect MicroSD*/
#define po_NAND_WPn	36	/* Nand flash write protect*/
#define poWATCHDOG	98	/* Emulate feed watchdog,output mcu*/
#define poEN_LOCAL_MIC		43	/* Enable local microphone */
#define poEN_CAMERA_LED		42	/* Enable compensation LED for camera */
#define poEN_LCD_3_3V	    57	/* Enable LCD 3.3V */
#define piTOUCH_BUSY		59	/* Touch screen busy */
#define poAUDIO_DM365_ZL	60	/* AUDIO DM365 ZALINK */
#define poE2_WPn		61	/* E2prom Write Protect */
#define poSPK_PWR		74	/* Speaker power control,H:enable */
#define poEN_BACKLIGHT		78	/* Lcd backlight enable */
#define poLCD_GAMMA_CTRL	77	/* Lcd gamma control */
#define poTOUCH_CSn		51	/* Touch screen chip select */
#define poEN_LCD_5V		75	/* Enable LCD 5v */
#define poSN74CBT_S1		65	/* SN74CBT16214 switch video channel */
#define poSN74CBT_S0		66	/* */
#if 0
#define piButton1	72	/* use for future,work in polling mode */
#define piButton2	71	/* */
#define piButton3	70	/* */
#define piButton4	69	/* */
#define piButton5	68	/* */
#define poEnable3	67	/* use for future */
#define poEnable4	73	/* */
#define poEnable5	37	/* */
#endif

/* Configurator Resistence */
/*
#define OSCFG			81
#define AECFG[0]		73
#define AECFG[1]		74
#define AECFG[2]		75
#define BTSEL[0]		76
#define BTSEL[1]		77
#define BTSEL[2]		78
*/

/* -- Peripheral ------------------------------------------------------------ */
/*
#define UART0_TX		18
#define UART0_RX		19
#define UART1_TX		25
#define UART1_RX		34
#define SD_D0			38 non gpio ma eemc bus
#define SD_D1	`		39
#define SD_D2			40
#define SD_D3			41
#define SD_CMD			42
#define SD_CLK			43
#define I2C_SCL			20
#define I2C_SDA			21
#define SPI0_TXD		22
#define SPI0_RXD		23
#define SPI0_CLK		24
#define IIS_FS			47
#define IIS_CLK			48
#define IIS_DATAQ		49
*/

/*
#define ENET_MDC		1
#define ENET_MDIO		2
#define ENET_CRS		3
#define ENET_RXER		4
#define ENET_RXDV		5
#define ENET_RXC		6
#define ENET_RXD0		7
#define ENET_RXD1		8
#define ENET_RXD2		9
#define ENET_RXD3		10
#define ENET_COL		15
#define ENET_TXC_U		16
#define ENET_TXEN_U		17
*/

/*
#define ASYN_CE1_U_N		55
#define ASYN_CE0_U_N		56
#define ASYN_RE_U_N		53
#define ASYN_WE_U_N		54
#define EM_BA0			65
#define EM_BA1			66
#define ADR[0]			67
#define ADR[5]			70
#define CODEC_SYS_CLK		37
#define CPU_ASYN_D16_U	52
*/

/*
#define PAL_DEC_FIELD		93
#define PAL_DEC_VS		94
#define PAL_DEC_HS		95
*/

#endif /* ASM_ARCH_AMICO_H */

/* End Of File. */
