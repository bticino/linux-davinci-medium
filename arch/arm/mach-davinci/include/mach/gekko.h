/*
 *  Created:    Aug 22, 2014
 *  Copyright:  Bticino.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef ASM_ARCH_GEKKO_H
#define ASM_ARCH_GEKKO_H

/* -- IRQ (GPIO) ------------------------------------------------------------ */
#define piINTERRUPT		0	/* Interrup Pin */
#define piPOWER_FAILn	50	/* IRQ_PowerFail */
#define piINT_I2Cn		35	/* I2C interrupt between DM365 and MCU*/
#define piTMK_INTn		62	/* IRQ_RTC */
#define piPENIRQn		97	/* IRQ_TouchScreen */
#define	piMaster_Slave		31	/* GPIO Master_Slave */
#define	piPower_Aux		47	/* GPIO Power Aux */
#define piFLOOR_CALL		63	/* GPIO floor call */
/* -- GPIO ------------------------------------------------------------------ */
#define poNAND_WPn              36      /* Nand flash write protect */
#define poENET_RESETn           44      /* ETH phy reset */
#define poBOOT_FL_WPn           45      /* Protecting SPI chip select */
#define poENABLE_VIDEO_IN       40      /* Enable DEM_VIDEO */
#define poPDEC_PWRDNn           39      /* PalDecoder PWRDown */
#define poPDEC_RESETn		38     /* PAL decoder Reset */
#define poEN_CH_SUPP		37	/* Enable ch supp */
#define poPIC_RESETn            48      /* PIC AV & AI reset */
#define poMCU_RESETn            99      /* PIC AV & AI reset */
#define poLCD_GPIO		96      /* NRESET TOUCH*/
#define po_NAND_WPn		36      /* Nand flash write protect*/
#define poEN_LOCAL_MIC          43      /* Enable local microphone */
#define poMUTE_SPK		42      /* Enable mute speaker */
#define poATT_INTER		41	/* Enable Att intercom */
#define poEN_LCD_3_3V		57	/* Enable LCD 3.3V */
#define poEN_VGL		59	/* ENABLE LCD VGL */
#define poEN_VGH                58	/* ENABLE LCD VGH */
#define poEN_VCOM_LCD		98	/* enable LCD VCOM */
#define poEN_AVDD		66	/* enable AVDD */
#define poEN_LCD_POWER		26	/* enable LCD power */
#define poEN_DITHER		55	/* enable dithering */
#define poVIDEO_BUS_RES		68      /* video bus reset */
#define poEN_5V_A		60      /* enable 5V_A */
#define poEN_5V_FON		67      /* enable 5V_FON */
#define poE2_WPn		61      /* E2prom Write Protect */
#define poVideo_Mode		72	/* Video Bus mode */
#define poSPK_PWR		74      /* Speaker power control,H:enable */
#define poTOUCH_CSn		101      /* Touch screen chip select */
#define poEN_LCD_5V		75      /* Enable LCD 5v */
#define poSN74CBT_S1		65      /* SN74CBT16214 switch video channel */
#define poEN_T_COIL		64      /* enable tcoil */

#endif /* ASM_ARCH_GEKKO_H */

/* End Of File. */
