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

#define COUNT		5

/* Reg.. */
#define REG_0402	0x0402
#define REG_0403	0x0403
#define	TX_START	0x7e01
#define	RX_START_OK	0x5e01
#define REG_0403_ACK	0 // x1

#define ZL38005_RX_BUF_SIZE	4
#define ZL38005_TX_BUF_SIZE	4

#endif /* __ZL38005_H */
