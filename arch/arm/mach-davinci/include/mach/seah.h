/*
 *  Created:    April 15, 2014
 *  Copyright:  Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_SEAH_H
#define ASM_ARCH_SEAH_H

/* -- IRQ (GPIO) ------------------------------------------------------------ */
#define pi_INTERRUPT		0	/* Interrup Pin */
#define piPOWER_FAILn		50	/* IRQ_PowerFail */
#define piRESET_CONF		42	/* Configuration Recovery */

/* -- GPIO ------------------------------------------------------------------ */
#define ENET_RESETn		44	/* */
#define poBOOT_FL_WPn		45	/* Protecting SPI chip select */
#define poE2_WPn		61	/* E2prom Write Protect */
#define poPIC_RESETn		99	/* poMCU_RESETn */
#define po_NAND_WPn		36	/* */

#define	po_LED_RED		38	/* LED */
#define	po_LED_GREEN		39
#define	po_SPEED_LEDn		40
#define	po_LINK_LEDn		41
#define po_DISCHARGE		43	/* Discharge Configuration Recovery */

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
#define I2C_SCL			20
#define I2C_SDA			21
#define SPI0_TXD		22
#define SPI0_RXD		23
#define SPI0_CLK		24
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
#define ENET_TXD0		11
#define ENET_TXD1		12
#define ENET_TXD2		13
#define ENET_TXD3		14
#define ENET_COL		15
#define ENET_TXC_U		16
#define ENET_TXEN_U		17
*/

/*
#define ASYN_CE0_U_N		56
#define ASYN_RE_U_N		53
#define ASYN_WE_U_N		54
#define CPU_ASYN_D16_U		52
*/

#endif /* ASM_ARCH_SEAH_H */
