/*
 *  Created:    Jan 24, 2012
 *  Copyright:  Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_JUMBO_COMMON_H
#define ASM_ARCH_JUMBO_COMMON_H

/* -- IRQ (GPIO) ------------------------------------------------------------ */
#define pi_INTERRUPT		0	/* Interrup Pin */
#define piPOWER_FAILn		50	/* IRQ_PowerFail */
#define piINT_UART_An		58	/* IRQ_USART_A */
#define piINT_UART_Bn		26	/* IRQ_USART_B */
#define piTMK_INTn		64	/* IRQ_RTC */
#define piGPIO_INTn		100	/* IRQ_General GPIO "For Future Use" */

/* -- GPIO ------------------------------------------------------------------ */

#define poENET_RESETn		44	/* */
#define poRES_EXTUART		51	/* */
#define poBOOT_FL_WPn		45	/* Protecting SPI chip select */
/* #define poEN_MOD_DIFF_SONORA	55 */ /*TODO */	/* EN_Audio */
#define poENABLE_VIDEO_IN	62	/* Enable DEM_VIDEO */
#define poPDEC_PWRDNn		103	/* PalDecoder PWRDown */
#define poPDEC_RESETn		102	/* PAL decoder Reset */
#define poE2_WPn		80	/* E2prom Write Protect */
#define poZARLINK_CS		60	/* Zarlink chip select */
#define poZARLINK_RESET		72	/* Zarlink reset */
#define poPIC_RESETn		99	/* PIC AV & AI reset */
#define piOCn			101	/* Over Current */
#define piSD_DETECTn		96	/* Detect MicroSD*/
#define poEMMC_RESETn		59	/* eMMC reset*/
#define poPOWER_LED_YELLOWn	82	/* LED */
#define poRESET_CONFn		28	/* NON COLLEGATO TODO */
#define po_ENABLE_VIDEO_OUT	61	/* */
#define po_EN_FONICA		68	/* */
#define po_NAND_WPn		86	/* */
#define pi_GPIO_2		88	/* Future use */
#define pi_GPIO_1		89	/* Future use */
#define po_AUDIO_RESET		90
#define po_AUDIO_DEEMP		91
#define po_AUDIO_MUTE		92
#define po_EN_SW_USB		97
#define po_EN_PWR_USB		98

/* Configurator Resistence */
//#define OSCFG			81
//#define AECFG[0]		73
//#define AECFG[1]		74
//#define AECFG[2]		75
//#define BTSEL[0]		76
//#define BTSEL[1]		77
//#define BTSEL[2]		78

/* -- Peripheral ------------------------------------------------------------ */

//#define UART0_TX		18
//#define UART0_RX		19
//#define UART1_TX		25
//#define UART1_RX		34
//#define SD_D0			38 non gpio ma eemc bus
//#define SD_D1	`		39
//#define SD_D2			40
//#define SD_D3			41
//#define SD_CMD		42
//#define SD_CLK		43
//#define I2C_SCL		20
//#define I2C_SDA		21
//#define SPI0_TXD		22
//#define SPI0_RXD		23
//#define SPI0_CLK		24
//#define IIS_FS		47
//#define IIS_CLK		48
//#define IIS_DATAQ		49
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
//#define ASYN_CE1_U_N		55
//#define ASYN_CE0_U_N		56
//#define ASYN_RE_U_N		53
//#define ASYN_WE_U_N		54
//#define EM_BA0		65
//#define EM_BA1		66
//#define ADR[0]		67
//#define ADR[5]		70
//#define CODEC_SYS_CLK		37
//#define CPU_ASYN_D16_U	52

//#define PAL_DEC_FIELD		93
//#define PAL_DEC_VS		94
//#define PAL_DEC_HS		95

#endif /* ASM_ARCH_JUMBO_COMMON_H */

/* End Of File. */
