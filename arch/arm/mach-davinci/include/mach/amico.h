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
#define piPOWER_FAILn		50	/* IRQ_PowerFail */
#define piINT_I2Cn		35	/* I2C interrupt between DM365 and MCU*/

/* -- GPIO ------------------------------------------------------------------ */
#define poBL_PWM			25  /* Backlight control */
#define poAUDIO_SEP_ZL		34	/* AUDIO SEP ZALINK */
#define poNAND_WPn			36	/* Nand flash write protect */
#define poENET_RESETn		44	/* ETH phy reset */
#define poRES_EXTUART		51	/* Extend uart reset */
#define poBOOT_FL_WPn		45	/* Protecting SPI chip select */
#define poENABLE_VIDEO_IN	58	/* Enable DEM_VIDEO */
#define poPDEC_PWRDNn		38	/* PalDecoder PWRDown */
#define poPDEC_RESETn       39     /* PAL decoder Reset */
#define poZARLINK_CS		101	/* Zarlink chip select */
#define poZARLINK_RESET		78	/* Zarlink reset */
#define poPIC_RESETn		99	/* PIC AV & AI reset */
#define piSD_DETECTn		96	/* Detect MicroSD*/
#define po_NAND_WPn	36	/* Nand flash write protect*/
#define poWATCHDOG	98	/* Emulate feed watchdog,output mcu*/
#define piIN_MUX_READ		40	/* In mux read*/
#define poEN_LOCAL_MIC		43	/* Enable local microphone */
#define poEN_LCD_3_3V	    57	/* Enable LCD 3.3V */
#define piTOUCH_BUSY		59	/* Touch screen busy */
#define poAUDIO_DM365_ZL	60	/* AUDIO DM365 ZALINK */
#define poE2_WPn		61	/* E2prom Write Protect */
#define poSEL_MUX_2		62	/* 74hc4051, 1 of 4, interrupt scan */
#define poSEL_MUX_1             63	/* */
#define poSEL_MUX_0             64	/* */
#define poSPK_PWR		68	/* Speaker power control,H:enable */
#define poEN_BACKLIGHT		72	/* Lcd backlight control */
#define poLCD_GAMMA_CTRL	73	/* Lcd gamma control */
#define poTOUCH_CSn		74	/* Touch screen chip select */
#define poEN_LCD_5V		75	/* Enable LCD 5v */
#define poSN74CBT_S1		76	/* SN74CBT16214 switch video channel */
#define poSN74CBT_S0		77	/* */

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
