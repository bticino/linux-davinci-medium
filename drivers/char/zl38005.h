#ifndef __ZL38005_H
#define __ZL38005_H
#define DRIVER_NAME	"zl38005"


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

#define CTRL_OP_RD	(0<<0)
#define CTRL_OP_WR	(1<<0)
#define CTRL_WIDTH_8	(0<<1)
#define CTRL_WIDTH_16	(1<<1)
#define CTRL_WIDTH_24	(2<<1)
#define CTRL_WIDTH_32	(3<<1)
#define CTRL_PENDING	(1<<3)
#define CTRL_MEMORY_SEL	(1<<8)

#define REG_0402	0x0402
#define REG_0403	0x0403
#define	TX_START	0x7e01
#define	RX_START_OK	0x5e01
#define REG_0403_ACK	0 // x1

#define COUNT		10

#define ZL38005_RX_BUF_SIZE	4
#define ZL38005_TX_BUF_SIZE	4

#endif /* __ZL38005_H */
