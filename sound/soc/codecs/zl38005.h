/*
 * zl38005.h  --  ZL38005 Soc Audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ZL38005_H
#define __ZL38005_H
#define DRIVER_NAME	"zl38005"

/* Command Byte */
#define CMD_RD_CTRL_BYTE        (CMD_VALID | CMD_READ | CMD_LEN_16_BIT \
				| CMD_TYPE_CTRL)
#define CMD_WR_CTRL_BYTE        (CMD_VALID | CMD_WRITE | CMD_LEN_16_BIT \
				| CMD_TYPE_CTRL)

#define CMD_RD_ADR              (CMD_VALID | CMD_READ | CMD_LEN_16_BIT \
				| CMD_TYPE_ADDR_WORD)
#define CMD_WR_ADR              (CMD_VALID | CMD_WRITE | CMD_LEN_16_BIT \
				| CMD_TYPE_ADDR_WORD)

#define CMD_RD_DATA_16          (CMD_VALID | CMD_READ | CMD_LEN_16_BIT \
				| CMD_TYPE_RD_DATA)
#define CMD_WR_DATA_16          (CMD_VALID | CMD_WRITE | CMD_LEN_16_BIT \
				| CMD_TYPE_WR_DATA)

#define CMD_RD_DATA_24          (CMD_VALID | CMD_READ | CMD_LEN_24_BIT \
				| CMD_TYPE_RD_DATA)
#define CMD_WR_DATA_24          (CMD_VALID | CMD_WRITE | CMD_LEN_24_BIT \
				| CMD_TYPE_WR_DATA)

#define CMD_RD_DATA_32          (CMD_VALID | CMD_READ | CMD_LEN_32_BIT \
				| CMD_TYPE_RD_DATA)
#define CMD_WR_DATA_32          (CMD_VALID | CMD_WRITE | CMD_LEN_32_BIT \
				| CMD_TYPE_WR_DATA)

/* Control Byte */
#define CTLR_RD_DMEM            (CTRL_DMEM | CTRL_PENDING | CTRL_WIDTH_16 \
				| CTRL_OP_RD)
#define CTLR_WR_DMEM            (CTRL_DMEM | CTRL_PENDING | CTRL_WIDTH_16 \
				| CTRL_OP_WR)

#define CTLR_RD_IMEM            (CTRL_IMEM | CTRL_PENDING | CTRL_WIDTH_24 \
				| CTRL_OP_RD)
#define CTLR_WR_IMEM            (CTRL_IMEM | CTRL_PENDING | CTRL_WIDTH_24 \
				| CTRL_OP_WR)

/* Command Byte */
#define CMD_VALID	(2<<6)
#define CMD_READ	(0<<5)
#define CMD_WRITE	(1<<5)
#define CMD_LEN_8_BIT	(0<<3)
#define CMD_LEN_16_BIT	(1<<3)
#define CMD_LEN_24_BIT	(2<<3)
#define CMD_LEN_32_BIT	(3<<3)
#define CMD_TYPE_CTRL	(1<<0)
#define CMD_TYPE_ADDR_WORD	(2<<0)
#define CMD_TYPE_RD_DATA	(3<<0)
#define CMD_TYPE_WR_DATA	(4<<0)

/* Controll Register Function */
#define IMEM		1    /* 1-> Instruction Ram */
#define DMEM		0    /* 0-> Data Ram */
#define CTRL_IMEM	(1<<8)
#define CTRL_DMEM	(0<<8)
#define CTRL_OP_RD	(0<<0)
#define CTRL_OP_WR	(1<<0)
#define CTRL_WIDTH_8	(0<<1)
#define CTRL_WIDTH_16	(1<<1)
#define CTRL_WIDTH_24	(2<<1)
#define CTRL_WIDTH_32	(3<<1)
#define CTRL_PENDING	(1<<3) /* 1-> Start/pending , 0-> Access Complete */

#define RETRY_COUNT	10

/* ZL Register */
#define BOOT_CTRL0	0x0400
#define BOOT_CTRL1	0x0401
#define HOST_RQ		0x0402
#define TARGET_ACK	0x0403

/*Boot Loader Flag */
#define LD_CFG_END	0x0800 /* 0b_xxxx_1000_0000_xxx0 */
#define LD_IMG_END	0x0400 /* 0b_xxxx_0100_0000_xxx0 */
#define GO		0x0200 /* 0b_xxxx_0010_0000_xxx0 */
#define S_IMG		0x0100 /* 0b_xxxx_0001_0000_xxx0 */
#define LD_CFG_F	0x0080 /* 0b_xxxx_0000_1000_xxx0 */
#define LD_IMG_F	0x0040 /* 0b_xxxx_0000_0100_xxx0 */
#define LD_CFG		0x0020 /* 0b_xxxx_0000_0010_xxx0 */
#define LD_IMG		0x0010 /* 0b_xxxx_0000_0001_xxx0 */

#define BOOT_MODE	0x0000
#define USER_MODE	0x0001

#define	BLCNRT		0x0800 /* 0b_xxxx_10xx_xx0x_xxx0 */
#define	S_CFG		0x0001 /* 0b_xxxx_00xx_xx0x_xxx1 */

#define	TX_START	0x7E01

#define	NOT_DONE	0x0000
#define	COMM_START_ACK	0x0001
#define	COMM_OK		0x0002
#define	COMM_FAIL	0x0003
#define	RX_START	0x5E01

#define ZL38005_RX_BUF_SIZE	4
#define ZL38005_TX_BUF_SIZE	4

typedef union
{
	unsigned short Buff[16];
	struct{
		unsigned short MagicWord;
		unsigned short FlashSpeed;
		unsigned short FWR_Ver;
		unsigned short FlashADD_CfgRecord_LO;
		unsigned short FlashADD_CfgRecord_HI;
		unsigned short Reserved1;
		unsigned short Reserved2;
		unsigned short Reserved3;
		unsigned short FlashADD_SecondFwrImg_LO;
		unsigned short FlashADD_SecondFwrImg_HI;
		unsigned short IMEM_StartAddress;
		unsigned short IMEM_Size;
		unsigned short DMEM_StartAddress;
		unsigned short DMEM_Size;
		unsigned short IMEM_Cks;
		unsigned short DMEM_Cks;
	} FIH ; /* ZL Fwr Image Header */
}t_FwrImageHeader;

typedef union
{
	unsigned short Buff[4];
	struct{
		unsigned short MagicWord;
		unsigned short StartAddress;
		unsigned short Size;
		unsigned short Cks;
	} CFGRecord ; /* ZL Fwr Image Header */
}t_CFGR;

#define ZL38005_AEC_CTRL0       0x044A
#define ZL38005_AEC_CTRL1       0x044B
#define ZL38005_SYSGAIN         0x044D
#define ZL38005_USRGAIN         0x046B

#endif /* __ZL38005_H */
