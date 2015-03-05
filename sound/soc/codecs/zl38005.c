/*
 * Copyright (C) 2012 Bticino S.p.A.
 *
 * This program is free software you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not,write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 * Zarlink zl38005 (Acoustic Echo Canceller)
 * Some not public information, delivered by Zarlink,
 * has been used to implement the following code.
 *
 * TODO List: Porting of zl_hardwareReset ??? (Command Reverded)
 *
 *          2010 : R.Recalcati,S.Ciani	rev.1.0
 * December 2012 : S.Cianni		rev.2.0
 *
 */

/* Includes ------------------------------------------------------------------*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/spi/zl38005.h>
#include <linux/gpio.h>
#include <sound/soc.h>

#include "zl38005.h"
#include <asm/mach-types.h>


static const char module_name[] = "zl38005";

#define ZL38005 "zl38005:"

#ifdef DEBUG
#define pr_debug_zl1(s, args...) \
	printk(KERN_DEBUG "%s %d: "s, __func__, __LINE__, ##args);
	/* printk(KERN_INFO ZL38005 "%s %d: "s, __func__, __LINE__, ##args); */
#else
#define pr_debug_zl1(s, args...)
#endif

#ifdef DEBUG_2
#define pr_debug_zl2(s, args...) \
	printk(KERN_DEBUG "%s %d: "s, __func__, __LINE__, ##args);
	/* printk(KERN_INFO ZL38005 "%s %d: "s, __func__, __LINE__, ##args); */
#else
#define pr_debug_zl2(s, args...)
#endif

#ifdef DEBUG_3
#define pr_debug_zl3(s, args...) \
	printk(KERN_DEBUG "%s %d: "s, __func__, __LINE__, ##args);
	/* printk(KERN_INFO ZL38005 "%s %d: "s, __func__, __LINE__, ##args); */
#else
#define pr_debug_zl3(s, args...)
#endif

/*
 * zl38005 register default settings
 */
struct zl38005_regs_s {
	u16 reg;
	u16 val;
};

static struct zl38005_regs_s zl38005_default_reg[] = {
	/* dummy registers */
	{ 0x0000, 0x0000 },		/* on/off status */

	/* real registers */
	{ 0x044a, 0 },
	{ 0x044b, 0 },
	{ 0x047b, 0 },
	{ 0x0529, 0 },
	{ 0x044d, 0 },
	{ 0x05e8, 0 },
	{ 0x0451, 0 },
	{ 0x0450, 0 },
	{ 0x05ec, 0 },
	{ 0x046b, 0 },
	{ 0x0472, 0 },
	{ 0x0473, 0 },
	{ 0x0453, 0 },

	/* terminator */
	{ 0xffff, 0xffff }
};

/* ZL38005 driver private data */
struct zl38005 {
	dev_t dev;
	int minor;
	struct spi_device *spi;
	struct list_head device_entry;
	char cs_gpio_label[15]; /*chip_select_label*/
	int reset_gpio;
	char reset_gpio_label[10];
	atomic_t usage;

	struct zl38005_regs_s reg_cache[0];	/* must be the last one! */
};

/* For registeration of charatcer device */
static struct cdev c_dev;

#define DEVCOUNT 1

static int zl38005_major;
static struct zl38005 *zl38005_table[DEVCOUNT];

static DEFINE_MUTEX(zl38005_list_lock);
static LIST_HEAD(zl38005_list);

struct zl38005_ioctl_par {
	unsigned int		addr;
	unsigned int		value;
};

/* ioctl() calls that are permitted to the /dev/zl38005 interface.
 * nota: sono tutte _IOR , esistono anche le _IOW _IOWR */
#define ZL_MAGIC 'G'
#define ZL_RD_DATA_REG		_IOR(ZL_MAGIC, 0x09, struct zl38005_ioctl_par)
#define ZL_WR_DATA_REG		_IOR(ZL_MAGIC, 0x0a, struct zl38005_ioctl_par)
#define ZL_RD_INSTR_REG		_IOR(ZL_MAGIC, 0x0b, struct zl38005_ioctl_par)
#define ZL_WR_INSTR_REG		_IOR(ZL_MAGIC, 0x0c, struct zl38005_ioctl_par)
#define ZL_UPDATE		_IOR(ZL_MAGIC, 0x0d, struct file_data)
#define ZL_UPLOAD		_IOR(ZL_MAGIC, 0x0e, struct file_data)
#define ZL_SAVE_CFGR		_IOR(ZL_MAGIC, 0x0f, int)
#define ZL_READ_CURRENT_CFGR	_IOR(ZL_MAGIC, 0x10, struct file_data)

static struct zl38005_ioctl_par zl38005_ioctl_par;
static struct file_data {
	t_FwrImageHeader	* FWR_Header;
	unsigned int		* IMEM_DataBuffer;
	unsigned short		* DMEM_DataBuffer;
	t_CFGR			* CFG_Header;
	unsigned short		* CFG_DataBuffer;
} file_data;

static void zl_delay(void){
	udelay(500);
}

static int zl_checkConnection(struct device *dev);

/* Cmd : Read 16bit Controll Register */
static int zl38005_rd_ctrl_reg(struct device *dev)
{
	int err;
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	u8 cmd;
	u8 rx_buf[3];           /* dummy,Byte_HI,Byte_LO */
	u16 rd_value = -1 ;
	int t = 0;

	cmd = CMD_RD_CTRL_BYTE ;

	/* See if there is a pending operation */
	while (t < RETRY_COUNT) {
		err = spi_write_then_read(zl38005->spi, &cmd, sizeof cmd, \
					rx_buf, sizeof rx_buf);
		if (err < 0) {
			dev_err(dev, "spi_sync failed with %d\n", err);
			return err;
		}
		rd_value  = (rx_buf[1]<<8) | rx_buf[2] ; /* Byte_HI, Byte_LO */
		pr_debug_zl1("TX cmd = %02X\n", cmd);
		pr_debug_zl1("RX rd_value = %04X \n", rd_value);

		if ((rd_value & CTRL_PENDING) == 0)
			break;
		else {
			zl_delay();
			/* suspend and after wait a little bit */
			schedule();
		}
		t++;
	}
	if (t == RETRY_COUNT) {
		pr_debug_zl2("zl38005: TIMEOUT for pending operations %d\n", t);
		/*printk(KERN_ERR "zl38005: TIMEOUT for pending
					* operations\n");*/
		return -1;
	}
	return 0;
}

/* Cmd : Write 16bit Controll Register */
static int zl38005_wr_ctrl_reg(struct device *dev, u16 Ctlr_Byte)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	struct spi_message msg;
	u8 buf[3];
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = buf,
	};
	int err;

	spi_message_init(&msg);
	buf[0] = CMD_WR_CTRL_BYTE ;
	buf[1] = (Ctlr_Byte >> 8) & 0xFF ;
	buf[2] = Ctlr_Byte & 0xFF ;

	pr_debug_zl1("TX cmd       = %02X \n", buf[0]);
	pr_debug_zl1("TX ctrl_byte = %02X %02X \n", buf[1], buf[2]);

	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

/* Cmd : Write 16bit address */
static int zl38005_wr_addr(struct device *dev, u16 adr)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	struct spi_message msg;
	u8 buf[3];
	struct spi_transfer xfer = {
		.len = 3,
		.tx_buf = buf,
	};
	int err;

	spi_message_init(&msg);
	buf[0] = CMD_WR_ADR;
	buf[1] = ((adr >> 8) & 0xff);
	buf[2] = (adr & 0xff);
	pr_debug_zl1("TX cmd = %02X\n", buf[0]);
	pr_debug_zl1("TX adr = %02X %02X \n", buf[1], buf[2]);

	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

/* Read 32bit data */
static int zl38005_rd_data(struct device *dev, int *pval)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	int err;
	u8 cmd;
	u8 rx_buf[5]; /* Buf Rx , 1byte dummy + 4byte data */

	cmd = CMD_RD_DATA_32 ;
	err = spi_write_then_read(zl38005->spi, &cmd, sizeof cmd, rx_buf, \
				sizeof rx_buf);
	if (err < 0) {
		dev_err(dev, "spi_write_then_read failed with %d\n", err);
		return err;
	}

	pr_debug_zl1("TX cmd    = %02X \n", cmd);
	pr_debug_zl1("RX valore = %02X %02X %02X %02X \n", rx_buf[1], \
			rx_buf[2], rx_buf[3], rx_buf[4]);
	*pval = (rx_buf[1] << 24) | (rx_buf[2] << 16) | (rx_buf[3] << 8) \
		| rx_buf[4];

	return 0;
}

/* Write 32bit data */
static int zl38005_wr_data(struct device *dev, u32 val)
{
	struct zl38005 *zl38005 = dev_get_drvdata(dev);
	struct spi_message msg;
	u8 buf[5];
	struct spi_transfer xfer = {
		.len = 5,
		.tx_buf = buf,
	};
	int err;

	spi_message_init(&msg);

	buf[0] = CMD_WR_DATA_32 ;
	buf[1] = ((val >> 24) & 0xff);	/* byte Hi */
	buf[2] = ((val >> 16) & 0xff);
	buf[3] = ((val >> 8)  & 0xff);
	buf[4] = (val & 0xff);	/* byte Lo */

	pr_debug_zl1("TX cmd    = %02X \n", buf[0]);
	pr_debug_zl1("TX valore = %02X %02X %02X %02X \n", buf[1], buf[2], \
					buf[3], buf[4]);
	spi_message_add_tail(&xfer, &msg);
	err = spi_sync(zl38005->spi, &msg);
	if (err < 0) {
		dev_err(dev, "spi_sync failed with %d\n", err);
		return err;
	}
	return 0;
}

/*
 * High Level Funcion
 */

/* Read Reg from Instruction Ram (IMEM) or Data Ram (DMEM) */
static int zl38005_rd_reg(struct device *dev, u16 addr, int *pval, u8 mem)
{
	u16 CtrlByte = 0 ;

	CtrlByte = CTLR_RD_DMEM ;
	if (mem)
		CtrlByte = CTLR_RD_IMEM ;

	/* See if there is a pending operation */
	if (zl38005_rd_ctrl_reg(dev))
		return -ENODEV;
	/* Write Address */
	if (zl38005_wr_addr(dev, addr))
		return -ENODEV;

	/* Writing Ctrl reg */
	if (zl38005_wr_ctrl_reg(dev, CtrlByte))
		return -ENODEV;

	/* See if there is a pending operation */
	if (zl38005_rd_ctrl_reg(dev))
		return -ENODEV;

	/* Read Data */
	if (zl38005_rd_data(dev, pval))
		return -ENODEV;

	pr_debug_zl3("ADD = %08X   LEGGO = %08X \n",addr, *pval);

	return 0;
}

/* Write Reg in Instruction Ram (IMEM) or Data Ram (DMEM) */
static int zl38005_wr_reg(struct device *dev, u16 addr, u32 val, u8 mem)
{
	u16 CtrlByte = 0 ;

	CtrlByte = CTLR_WR_DMEM ;
	if (mem)
		CtrlByte = CTLR_WR_IMEM ;

	/* See if there is a pending operation */
	if (zl38005_rd_ctrl_reg(dev))
		return -ENODEV;

	/* Write Address */
	if (zl38005_wr_addr(dev, addr))
		return -ENODEV;

	/* Write Data */
	if (zl38005_wr_data(dev, val))
		return -ENODEV;

	/* Writing Ctrl reg */
	if (zl38005_wr_ctrl_reg(dev, CtrlByte))
		return -ENODEV;

	pr_debug_zl3("ADD = %08X   SCRIVO = %08X \n",addr, val);

	return 0;
}

/*
 * Fwr Update Functions
 */

/*
 * ____ Slave ZL SPI Port Test _________________________________________________
 */
const int ImageHeaderReg[16] = {
	0x0404, 0x040F, 0x0415, 0x0410, 0x0411, 0x0417, 0x0418, 0x0419,
	0x041A, 0x041B, 0x0405, 0x0406, 0x0407, 0x0408, 0x0409, 0x040A };

const int CFGRecordHeaderReg[16] = { 0x040B, 0x040C, 0x040D, 0x040E };

/* Check connection (SPI)
* write    : HOST_RQ    0x0402 == TX_START 0x7E01
* and wait : TARGET_ACK	0x0403 == RX_START 0x5E01
*/
static int zl_checkConnection(struct device *dev)
{
	int Ack = -1;
	u16 Retry = 0;

	/* Start Tx and verifi Ack */
	while (Ack != RX_START) {
		zl38005_wr_reg(dev, TARGET_ACK, 0x0000, DMEM);  /* Clear Ack */
		zl38005_wr_reg(dev, HOST_RQ, TX_START, DMEM);
		zl_delay();
		zl38005_rd_reg(dev, TARGET_ACK, &Ack, DMEM);
		/* pr_debug_zl2("Ack = %X \n", Ack); */
		Retry++;
		if (Retry == RETRY_COUNT){
			pr_debug_zl2("Connection Fail, Retry = %d \n", Retry);
			return -1;
		}
	}
	return 0;
}

/* Mode = 0 -> goto BOOT_MODE , Mode = 1 -> goto USER_MODE */
static int zl_SwitchModeTo(struct device *dev, int mode)
{
	int ret = -1;

	/* Set the STOP bit in register 0x0401 and to go to boot mode */
	if (mode == BOOT_MODE) {
		zl38005_wr_reg(dev, 0x0401, 0x0800, DMEM); /*BLCNRT = 1*/
		ret = 0;
	}
	/* Set the GO bit in register 0x0400 and to go to User mode */
	if (mode == USER_MODE) {
		zl38005_wr_reg(dev, 0x0401, 0x0000, DMEM); /* BLCNRT = 0 */
		zl38005_wr_reg(dev, 0x0400, 0x0200, DMEM); /* GO = 1 */
		ret = 0;
	}

	return ret;
}

/* Verify_OPER MODE */
static int zl_Verify_OPER_MODE_is(struct device *dev, int mode)
{
	int Value = -1;
	int Retry = 0;

	/* BOOT_MODE :  OPER_MODE reg 0x4000 == 0b_xxxx_xxxx_xxxx_xxx0 */
	/* USER_MODE :  OPER_MODE reg 0x4000 == 0b_xxxx_xxxx_xxxx_xxx1 */

	/* Verify OPER_MODE */
	while ((Value & 0x0001) != mode) {
		zl38005_rd_reg(dev, BOOT_CTRL0, &Value, DMEM); /*rd: 0x0400*/
		mdelay(20);
		Retry++;

		if (Retry == RETRY_COUNT) {
			pr_debug_zl2("Failure to enter OPER_MODE=0x%4X, \
				Retry = %d \n", mode, Retry);
			return -1;
		}
	}

	if (Retry > 1) {
		if (mode == 1)
			pr_debug_zl2(" User Mode, in Retry= %d \n", Retry--);
		else
			pr_debug_zl2(" Boot Mode, in Retry= %d \n", Retry--);
	}

	return 0;
}

/*Cmd to save CFG Register, save on GO=1, if you need also save IMG take GO=0 */
static int zl_SaveCFG(struct device *dev, int go)
{
	/*
	* Initiates a configuration record save to flash operation.
	* In boot mode so just flip the bit config record will actually
	* be saved when the GO bit is set
	* BLCNRT = 0 , S_CFG =1
	*/
	zl38005_wr_reg(dev, 0x0401, 0x0001, DMEM); /* S_CFG = 1*/

	/* Set Stop bit to Go to User mode */
	if (go == 1) /*Execute*/
		zl38005_wr_reg(dev, 0x0400, 0x0200, DMEM);   /*GO ( Save)*/

	return 0;
}

/*Cmd to save FWR Image */
static int zl_SaveIMG(struct device *dev)
{
	/*
	* Save image to flash (actually occurs when GO bit is set) &
	* Set GO Bit to running Fwr (0b_xxxx_0011_0000_xxx0)
	*/
	zl38005_wr_reg(dev, 0x0400, 0x0300, DMEM);   /*SaveFWR & GO*/

	return 0;
}

/*Cmd to save FWR Image & Cfg Rerister */
static int zl_SaveALL(struct device *dev)
{
	/*
	* Initiates a configuration record save to flash operation.
	* In boot mode so just flip the bit config record will actually
	* be saved when the GO bit is set
	*/
	zl38005_wr_reg(dev, 0x0401, 0x0001, DMEM); /* S_CFG = 1*/

	/*
	* Save image to flash (actually occurs when GO bit is set) &
	* Set GO Bit to running Fwr (0b_xxxx_0011_0000_xxx0)
	*/
	zl38005_wr_reg(dev, 0x0400, 0x0300, DMEM);   /*SaveFWR & GO*/

	return 0;
}

static int zl_WaitBootCommand(struct device *dev)
{
	int ret = -1;
	int Retry = 0 ;
	int Ack = NOT_DONE ; /* 0x0000*/

	/* Wait Done */
	while (Ack == NOT_DONE) { /* ..Wait.. */
		zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
		mdelay(10); /*20*/
		Retry++;
		if (Ack == COMM_START_ACK) {
			ret = COMM_START_ACK;
			break;
		}
		if (Ack == COMM_OK) {
			ret = COMM_OK;
			break;
		}
		if (Ack == COMM_FAIL) {
			pr_debug_zl2("Fail");
			ret = COMM_FAIL;
			break;
		}
		if (Retry == RETRY_COUNT) {
			pr_debug_zl2("Boot_command Fail cause Retry = %d \n",
				 Retry);
			ret = -1;
			break;
		}
	}
	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);
	return ret;
}

/* Load FWR Image From Flash SPI */
static int zl_LoadIMG(struct device *dev)
{
	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Host ask to load the Firmware Image Header, Instruction Memory Data
	 * and Image Data from FLASH.
	 * This function can only be initiated when the firmware is in Boot Mode
	 * This control bit will return to zero automatically.
	 * Once initiated, the status of this function is indicated by the
	 * Target Acknowledge Register (address: 0x0403) to be either
	 * Not Done, COMM_OK or COMM_FAIL */
	zl38005_wr_reg(dev, 0x0400, LD_IMG_F, DMEM);   /*LD_IMG_F = 1*/

	/* Wait Done */
	if (zl_WaitBootCommand(dev) == COMM_OK)
		return 0;
	else
		return -1;

}

/* Load CFG Register From Flash SPI */
static int zl_LoadCFG(struct device *dev)
{
	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Host ask to load the Configuration Header and Record from FLASH.
	 * This function can only be initiated when the firmware is in Boot Mode
	 * This control bit will return to zero automatically.
	 * Once initiated, the status of this function is indicated by the
	 * Target Acknowledge Register (address: 0x0403) to be either
	 * Not Done, COMM_OK or COMM_FAIL */

	zl38005_wr_reg(dev, 0x0400, LD_CFG_F, DMEM);   /*LD_CFG_F =1*/

	/* Wait Done and clear Ack */
	if (zl_WaitBootCommand(dev) == COMM_OK)
		return 0;
	else
		return -1;

}

/* Update a romcode : IMGHeader,IMEM,DMEM */
static int zl_UpdateIMG(struct device *dev)
{
	u32 ZL_Write_Add = 0;
	int i = 0 ;

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* host to indicate to the Boot Loader that a new Firmware Image Header,
	 * Instruction Memory Data and Image Data.
	 * This function must be initiated when the firmware is not running.
	 * Once initiated, the status of this function is indicated by the
	 * Target Acknowledge Register (address: 0x0403) to be either Not Done
	 * or COMM_START_ACK. After the COMM_START_ACK state is reached the host
	 * may proceed with loading the Firmware */
	zl38005_wr_reg(dev, 0x0400, LD_IMG, DMEM);   /*LD_IMG = 1*/

	/* Wait Done and clear Ack */
	if (zl_WaitBootCommand(dev) != COMM_START_ACK)
		return -1;

	/* clear the BOOT_FAIL bit (isn't automagically
	 * cleared, we have to do it) */
	zl38005_wr_reg(dev, 0x0412, 0x0000, DMEM);

	/* ____1. Write Fwr Image Header ____ */
	for (i = 0; i < 16; i++) {
		if (zl38005_wr_reg(dev, ImageHeaderReg[i], \
				file_data.FWR_Header->Buff[i], DMEM)){
			pr_debug_zl2("Write Fwr Image Header Error \n");
			return -1;
		}
	}

	/* ____2. Write Fwr Image Instruction Memory Data ____ */
	ZL_Write_Add = file_data.FWR_Header->FIH.IMEM_StartAddress;
	for (i = 0; i < file_data.FWR_Header->FIH.IMEM_Size ; i++) {
		if (zl38005_wr_reg(dev, ZL_Write_Add, \
			file_data.IMEM_DataBuffer[i], IMEM)){
			pr_debug_zl2(" Write Fwr IMEM Error \n");
			return -1;
		}
		ZL_Write_Add += 1;
	}

	/* ____ 3. Write Fwr Image  Data Memory Data ____ */
	ZL_Write_Add = file_data.FWR_Header->FIH.DMEM_StartAddress;
	for (i = 0; i < file_data.FWR_Header->FIH.DMEM_Size ; i++) {
		if (zl38005_wr_reg(dev, ZL_Write_Add, \
			file_data.DMEM_DataBuffer[i], DMEM)){
			pr_debug_zl2(" Write Fwr DMEM Error \n");
			return -1;
		}
		ZL_Write_Add += 1;
	}

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Host to indicate to the Boot Loader that software have finished,
	 * and ask the Boot Loader to do chesk the CRC and shows the result.
	 This control bit will return to zero automatically. */
	zl38005_wr_reg(dev, 0x0400, LD_IMG_END, DMEM);   /*LD_IMG_END =1*/

	/* Wait Done and clear Ack */
	if (zl_WaitBootCommand(dev) == COMM_OK)
		return 0;
	else
		return -1;

}

/* Update a configuration record */
static int zl_UpdateCFG(struct device *dev)
{
	int i = 0 ;
	u32 ZL_Write_Add = 0;

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* host to indicate to the Boot Loader that a new Configuration Header
	 * and Record will be loaded.
	 * This function must be initiated when the firmware is not running.
	 * Once initiated, the status of this function is indicated by the
	 * Target Acknowledge Register (address: 0x0403) to be either Not Done
	 * or COMM_START_ACK. After the COMM_START_ACK state is reached the
	 * host may proceed with loading the Firmware */
	zl38005_wr_reg(dev, 0x0400, LD_CFG, DMEM);   /*LD_CFG = 1*/

	/* Wait Done and clear Ack */
	if (zl_WaitBootCommand(dev) != COMM_START_ACK)
		return -1;

	/* ____1. Write Control Registers Header ____ */
	for (i = 0; i < 4; i++)
		if (zl38005_wr_reg(dev, CFGRecordHeaderReg[i], \
			file_data.CFG_Header->Buff[i], DMEM)) {
			pr_debug_zl2("Write Control Registers Error \n");
			return -1;
	}

	/* ____2. Write Write CFG Record Data ____ */
	ZL_Write_Add = file_data.CFG_Header->CFGRecord.StartAddress;
	for (i = 0; i < file_data.CFG_Header->CFGRecord.Size ; i++) {
		if (zl38005_wr_reg(dev, ZL_Write_Add, \
			file_data.CFG_DataBuffer[i], DMEM)) {
			pr_debug_zl2(" Write Write CFG Record Data Error \n");
			return -1;
		}
		ZL_Write_Add += 1;
	}

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Host to indicate to the Boot Loader that software have finished,
	 * and ask the Boot Loader to do chesk the CRC and shows the result.
	 This control bit will return to zero automatically. */
	zl38005_wr_reg(dev, 0x0400, LD_CFG_END, DMEM);   /*LD_CFG_END =1*/

	/* Wait Done and clear Ack */
	if (zl_WaitBootCommand(dev) == COMM_OK)
		return 0;
	else
		return -1;

}

/* Complete FWR Update Procedure */
static int zl_UpdateFWR(struct device *dev)
{
		pr_debug_zl2("Fwr_update Procedure:\n");
		pr_debug_zl2("Check SPI Connection \n");
		if (zl_checkConnection(dev) == -1)
			return -1;

		pr_debug_zl2("Goto Boot Mode \n");
		if (zl_SwitchModeTo(dev, BOOT_MODE) == -1)
			return -1;

		pr_debug_zl2("Update_FWR_IMAGE \n");
		if (zl_UpdateIMG(dev) == -1)
			return -1;

		pr_debug_zl2("Update_CFGRegister \n");
		if (zl_UpdateCFG(dev) == -1)
			return -1;

		pr_debug_zl2("Save to Flash \n");
		if (zl_SaveALL(dev) == -1)
			return -1;

		pr_debug_zl2("Host_Verify Op mode = UserMode \n");
		if (zl_Verify_OPER_MODE_is(dev, USER_MODE) == -1)
			return -1;

		pr_debug_zl2("END \n");
		return 0;
}

/* Upload (Dump) RomCode Firmware (IMEM & DMEM) */
static int Zl_Upload_FWR_IMAGE(struct device *dev)
{
	u16 i = 0;
	int value = 0;
	u32 ZL_Read_Add = 0;

	/* ____1. Read Fwr Image Header ____ */
	for (i = 0; i < 16; i++) {
		if (zl38005_rd_reg(dev, ImageHeaderReg[i], &value, DMEM) == -1)
			return -1;
		file_data.FWR_Header->Buff[i] = value;
		/*printk("%4X \n",data);*/
	}

	/* ____2. Read Fwr Image Instruction Memory Data ____ */
	ZL_Read_Add = file_data.FWR_Header->FIH.IMEM_StartAddress;
	for (i = 0; i < file_data.FWR_Header->FIH.IMEM_Size ; i++) {
		if (zl38005_rd_reg(dev, ZL_Read_Add, &value, IMEM) == -1)
			return -1;
		file_data.IMEM_DataBuffer[i] = value;
		ZL_Read_Add += 1;
	}

	/* ____ 3. Read Fwr Image  Data Memory Data ____ */
	ZL_Read_Add = file_data.FWR_Header->FIH.DMEM_StartAddress;
	for (i = 0; i < file_data.FWR_Header->FIH.DMEM_Size ; i++) {
		if (zl38005_rd_reg(dev, ZL_Read_Add, &value, DMEM) == -1)
			return -1;
		file_data.DMEM_DataBuffer[i] = value;
		ZL_Read_Add += 1;
	}
	return 0;
}

/* Upload (dump) configuration record */
static int zl_Upload_CFGRegister(struct device *dev)
{
	u16 i = 0;
	int value = 0;
	u32 ZL_Read_Add = 0;

	/* ____1. Read Control Registers Header ____ */
	for (i = 0; i < 4; i++) {
		if (zl38005_rd_reg(dev, CFGRecordHeaderReg[i], &value, DMEM) \
		 == -1)
			return -1;
		file_data.CFG_Header->Buff[i] = value;
		/*printk("%4X \n",data);*/
	}

	/* ____ 2. Read CFG Record Data ____ */
	ZL_Read_Add = file_data.CFG_Header->CFGRecord.StartAddress;
	for (i = 0; i < file_data.CFG_Header->CFGRecord.Size ; i++) {
		if (zl38005_rd_reg(dev, ZL_Read_Add, &value, DMEM) == -1)
			return -1;
		file_data.CFG_DataBuffer[i] = value;
		/*printk("%4X \n",data);*/
		ZL_Read_Add += 1;
	}
	return 0;
}

/* Upload fwr+cfg from ram or from flash (fwr in stop!) */
#define FROM_RAM	0
#define FROM_FLASH	1
static int zl_UpLoadIMG(struct device *dev, int from)
{
	pr_debug_zl2("Fwr_Upload Procedure:\n");
	pr_debug_zl2("Check SPI Connection \n");
	if (zl_checkConnection(dev) == -1)
		return -1;

	pr_debug_zl2("Goto Boot Mode \n");
	if (zl_SwitchModeTo(dev, BOOT_MODE) == -1)
		return -1;

	if (from == FROM_FLASH) {
		/* Load FWR Image From Flash SPI */
		pr_debug_zl2("Load FWR Image & CFGRecord From Flash SPI \n");
		if (zl_LoadIMG(dev) == -1)
			return -1;
		if (zl_LoadCFG(dev) == -1)
			return -1;
	}

	/* Load FWR Image From (DSP) Ram */
	pr_debug_zl2("Upload FWR Image \n");
	if (Zl_Upload_FWR_IMAGE(dev) == -1)
			return -1;

	pr_debug_zl2("Upload CFGRecord \n");
	if (zl_Upload_CFGRegister(dev) == -1)
			return -1;

	pr_debug_zl2("Come back to User Mode \n");
	if (zl_SwitchModeTo(dev, USER_MODE) == -1)
		return -1;

	pr_debug_zl2("Host_Verify Op mode = UserMode \n");
	if (zl_Verify_OPER_MODE_is(dev, USER_MODE) == -1)
		return -1;

	pr_debug_zl2("END \n");
	return 0;
}

#if 0
static int zl_UpLoadCFG(struct device *dev)i
{
}
static int zl_DownLoadCFG_Only(struct device *dev)
{
}
static int zl_UpLoadFWR(struct device *dev)
{
}
#endif

/*
Comandi Riservato .... TODO Lo faccio ??????????
Comandi Riservato .... TODO Lo faccio ??????????
Comandi Riservato .... TODO Lo faccio ??????????
*/
/* Reset the device using the MAU
* Parameters:
* reset_mode = ZL_ROM : Resets the device and reload from FLASH.
* reset_mode = ZL_RAM : Resets the device and runs from RAM
*/
#if 0
void zl_hardwareReset(UInt16T reset_mode)
{
	/*send command*/
	zl_writeSPI(0xA0); /*10,1,00,000 - write, 8 bits, MAU Reset register*/
	if (reset_mode == ZL_ROM) {
		/*tell MAU control register to reset from ROM*/
		zl_writeSPI(0x01); /* RAM/ROM bit low, reset high */
	} else {
		/*tell MAU control register to reset from RAM*/
		zl_writeSPI(0x05); /* RAM/ROM bit high, reset high */
	}
}
#endif

/*
 * SoC stuff
 */

static void zl38005_reg_cache_write(struct zl38005_regs_s *reg_cache,
					u16 reg, u16 val)
{
	int i;

	for (i = 0; reg_cache[i].reg != 0xffff; i++)
		if (reg == reg_cache[i].reg) {
			reg_cache[i].val = val;
			return;
		}
	BUG();
}

static u16 zl38005_reg_cache_read(struct zl38005_regs_s *reg_cache, u16 reg)
{
	int i;

	for (i = 0; reg_cache[i].reg != 0xffff; i++)
		if (reg == reg_cache[i].reg)
			return reg_cache[i].val;
	BUG();
}

static int zl38005_is_on(struct zl38005_regs_s *reg_cache)
{
	return !!zl38005_reg_cache_read(reg_cache, 0x0000);
}

static int zl38005_reg_is_dummy(u16 reg)
{
	return reg <= 0x0000;
}

static int zl38005_sync_cache(struct spi_device *spi,
				struct zl38005_regs_s *reg_cache)
{
	int i;
	int ret;

	i = 0;
	do {
		msleep(60);
		ret = zl_checkConnection(&spi->dev);
	} while (ret & (i++ < 10));

	if (i == 10)
		return -EIO;

	for (i = 0; reg_cache[i].reg != 0xffff; i++) {
		if (zl38005_reg_is_dummy(reg_cache[i].reg))
			continue;

		ret = zl38005_wr_reg(&spi->dev, reg_cache[i].reg,
				reg_cache[i].val, DMEM);
#if 0
		printk(KERN_DEBUG "%s, sync reg %4X , value %4X ",
				__func__, reg_cache[i].reg, reg_cache[i].val);
#endif
		if (ret)
			return -EIO;
	}

	return 0;
}

static int zl38005_init_cache(struct zl38005 *zl38005,
				struct zl38005_regs_s *reg_cache)
{

	struct spi_device *spi;

	int i;
	int ret;

	spi = zl38005->spi;

	i = 0;
	do {
		msleep(60);
		ret = zl_checkConnection(&spi->dev);
	} while (ret & (i++ < 10));

	if (i == 10)
		return -EIO;

	for (i = 0; reg_cache[i].reg != 0xffff; i++) {
		if (zl38005_reg_is_dummy(reg_cache[i].reg))
			continue;
		{
		int valt = 0;
		ret = zl38005_rd_reg(&spi->dev, reg_cache[i].reg, &valt, DMEM);
		reg_cache[i].val = valt;
		}
#if 0
		printk(KERN_DEBUG "%s, init reg %4X , value %4X ",
				__func__, reg_cache[i].reg, reg_cache[i].val);
#endif
		if (ret)
			return -EIO;
	}

	return 0;
}

static int zl38005_spi_write(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
	(struct soc_mixer_control *) kcontrol->private_value;
	struct zl38005 *zl38005;
	struct spi_device *spi;
	unsigned int reg = mc->reg;
	unsigned int shift = mc->shift;
	unsigned int mask = mc->max;
	unsigned int invert = mc->invert;
	unsigned int val = (ucontrol->value.integer.value[0] & mask);
	int minor, valt;
	int ret;

	/* Dirty hack to get proper spi device... */
	minor = kcontrol->id.name[2] - '0';
	BUG_ON(minor >= DEVCOUNT);
	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);
	spi = zl38005->spi;

        if (invert)
                val = mask - val;

	if (zl38005_is_on(zl38005->reg_cache) && !zl38005_reg_is_dummy(reg)) {
		ret = zl38005_rd_reg(&spi->dev, reg, &valt, DMEM);
		if (ret)
			return -EIO;
	} else
		valt = zl38005_reg_cache_read(zl38005->reg_cache, reg);

        valt &= ~(mask << shift);
        valt |= val << shift;

	zl38005_reg_cache_write(zl38005->reg_cache, reg, valt);

	switch (reg) {
	case 0x0000:
		if (valt == 1) {
			/*printk(KERN_DEBUG "Zarlink Control:Zarlik UnReset");*/
			gpio_set_value(zl38005->reset_gpio, 1);
			if (!machine_is_basi()) {
				ret = zl38005_sync_cache(spi, zl38005->reg_cache);
				if (ret)
					return ret;
			} else {
				/*printk(KERN_DEBUG "Basi Board Don't use zalink cache!!");*/
			}
		} else {
			/*printk(KERN_DEBUG "Zarlink Control : Zarlik Reset");*/
			gpio_set_value(zl38005->reset_gpio, 0);
		}
		break;

	default:
		zl38005_wr_reg(&spi->dev, reg, valt, DMEM);
		/*printk(KERN_DEBUG "%s (%d) : Zarlink Control write reg %4X, value %4X",
			__func__, __LINE__, reg, valt);*/
	}
	return 1;
}

static int zl38005_spi_read(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc =
                (struct soc_mixer_control *) kcontrol->private_value;
	struct zl38005 *zl38005;
	struct spi_device *spi;
        unsigned int reg = mc->reg;
        unsigned int shift = mc->shift;
        unsigned int mask = mc->max;
        unsigned int invert = mc->invert;
        int minor, val;
        int ret;

	/* Dirty hack to get proper spi device... */
	minor = kcontrol->id.name[2] - '0';
	BUG_ON(minor >= DEVCOUNT);
	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);
	spi = zl38005->spi;

	if (zl38005_is_on(zl38005->reg_cache) && !zl38005_reg_is_dummy(reg)) {
		ret = zl38005_rd_reg(&spi->dev, reg, &val, DMEM);
		if (ret)
			return -EIO;
	} else
		val = zl38005_reg_cache_read(zl38005->reg_cache, reg);

        ucontrol->value.integer.value[0] = ((val >> shift) & mask);

        if (invert)
                ucontrol->value.integer.value[0] =
                        mask - ucontrol->value.integer.value[0];

	return 0;
}

static const struct snd_kcontrol_new zl38005_controls[] = {
	/* Controll compatible with ZLS38501 v3.2.07 */
	SOC_SINGLE_EXT("ZLx AEC Bypass", ZL38005_AEC_CTRL0, 4, 1, 0,
					zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZLx Mute Rout", ZL38005_AEC_CTRL0, 7, 1, 0,
					zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZLx Mute Sout", ZL38005_AEC_CTRL0, 8, 1, 0,
					zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZLx Adaptation Disable", ZL38005_AEC_CTRL1, 1, 1, 0,
					zl38005_spi_read, zl38005_spi_write),

	SOC_SINGLE_EXT("ZLx Sin Gain control", ZL38005_SYSGAIN, 0, 0xff, 0,
					zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_EXT("ZLx Sout Gain control", ZL38005_SYSGAIN, 8, 0xf, 0,
					zl38005_spi_read, zl38005_spi_write),

	SOC_SINGLE_EXT("ZLx Rout Gain control", ZL38005_USRGAIN, 0, 0x7f, 0,
					zl38005_spi_read, zl38005_spi_write),

	SOC_SINGLE_EXT("ZLx-ON", 0x0000, 0, 1, 0,
			zl38005_spi_read, zl38005_spi_write),
};

/* This function is called from ASoC machine driver */
int zl38005_add_controls(unsigned int minor, struct snd_soc_codec *codec)
{
	struct zl38005 *zl38005;
	int i;
	struct soc_mixer_control *p;

	if (minor >= DEVCOUNT || !zl38005_table[minor])
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	for (i = 0; i < ARRAY_SIZE(zl38005_controls); i++) {
		p = (struct soc_mixer_control *)
				zl38005_controls[i].private_value;
		zl38005_controls[i].name[2] = '0' + minor;
	}

        return snd_soc_add_controls(codec, zl38005_controls,
                        ARRAY_SIZE(zl38005_controls));
}
EXPORT_SYMBOL_GPL(zl38005_add_controls);

/*
 * Char device stuff
 */

static int zl38005_open(struct inode *inode, struct file *filp)
{
	struct zl38005 *zl38005;
	int minor = MINOR(inode->i_rdev);

	if (minor >= DEVCOUNT || !zl38005_table[minor])
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	/* Only open once */
	if (atomic_inc_return(&zl38005->usage) > 1) {
		printk(KERN_ERR "at least one zl38005 device already opened\n");
                atomic_dec(&zl38005->usage);
                return -EBUSY;
        }

	return 0;
}

static int zl38005_close(struct inode *inode, struct file *filp)
{
	struct zl38005 *zl38005;
	int minor = MINOR(inode->i_rdev);

	if (minor >= DEVCOUNT || !zl38005_table[minor])
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	atomic_dec(&zl38005->usage);

	return 0;
}

static long zl38005_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int minor = iminor(f->f_path.dentry->d_inode);
	struct zl38005 *zl38005;
	int ret;
	long now, deadline;
	int msecs;

	/* Misure Time of Operation*/
	msecs = 0;
	now = jiffies;

	if (minor >= DEVCOUNT || !zl38005_table[minor])
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	/* IOCTL cmd */
	switch (cmd) {
	/* Write DMEM */
	case ZL_WR_DATA_REG:
		if (copy_from_user(&zl38005_ioctl_par, \
			(struct zl38005_ioctl_par *)arg,\
			 sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		pr_debug_zl1("ioctl wr DATA = %X \n", zl38005_ioctl_par.value);
		ret = zl38005_wr_reg(&zl38005->spi->dev, \
			zl38005_ioctl_par.addr, zl38005_ioctl_par.value, DMEM);
	break;
	/* Read DMEM */
	case ZL_RD_DATA_REG:
		if (copy_from_user(&zl38005_ioctl_par, \
			(struct zl38005_ioctl_par *)arg,\
			 sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, \
		 (u16)zl38005_ioctl_par.addr, &zl38005_ioctl_par.value, DMEM);
		if (!ret) {
			pr_debug_zl1("ioctl rd DATA = %X \n ", \
				zl38005_ioctl_par.value);
			if (copy_to_user((struct zl38005_ioctl_par *)arg,\
			 &zl38005_ioctl_par, sizeof(struct zl38005_ioctl_par)))
				return -EFAULT;
		} else
			return -EAGAIN;
	break;
	/* Write IMEM */
	case ZL_WR_INSTR_REG :
		if (copy_from_user(&zl38005_ioctl_par,
				   (struct zl38005_ioctl_par *)arg,
				   sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		pr_debug_zl1("ioctl wr DATA = %X \n", zl38005_ioctl_par.value);
		ret = zl38005_wr_reg(&zl38005->spi->dev,
				     zl38005_ioctl_par.addr,
				     zl38005_ioctl_par.value, IMEM);
	break;
	/* Read IMEM */
	case ZL_RD_INSTR_REG :
		if (copy_from_user(&zl38005_ioctl_par,
				   (struct zl38005_ioctl_par *)arg,
				   sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, \
			(u16)zl38005_ioctl_par.addr, &zl38005_ioctl_par.value,\
			 IMEM);
		if (!ret) {
			pr_debug_zl1("ioctl rd DATA = %X\n", \
					zl38005_ioctl_par.value);
			if (copy_to_user((struct zl38005_ioctl_par *)arg, \
				&zl38005_ioctl_par, \
				sizeof(struct zl38005_ioctl_par)))
				return -EFAULT;
		} else
			return -EAGAIN;
	break;
	/* Complete FWR UPDATE */
	case ZL_UPDATE:
		if (copy_from_user(&file_data, (struct file_data *)arg,\
				sizeof(struct file_data)))
			return -EFAULT;

	/*
	pr_debug_zl1();   //printk("%s %d \n",__func__, __LINE__);
	printk("FWR_Header        : %04x \n", file_data.FWR_Header->Buff[0]);
	printk("IMEM_DataBuffer   : %06x \n", file_data.IMEM_DataBuffer[0]);
	printk("DMEM_DataBuffer   : %04x \n", file_data.DMEM_DataBuffer[0]);
	printk("CFG_Header        : %04x \n", file_data.CFG_Header->Buff[0]);
	printk("CFG_DataBuffer    : %04x \n", file_data.CFG_DataBuffer[0]);
	pr_debug_zl1();   //printk("%s %d \n",__func__, __LINE__);
	*/

		ret = zl_UpdateFWR(&zl38005->spi->dev);
	break;
	/* Complete FWR UPLOAD (take data from SPI FLASH)*/
	case ZL_UPLOAD:
	{
		ret = -1;
		if (copy_from_user(&file_data, (struct file_data *)arg,\
				sizeof(struct file_data)))
			return -EFAULT;

		/*From RAM*/
		/*ret = zl_UpLoadIMG(&zl38005->spi->dev, FROM_RAM);*/
		/*From Flash SPI*/
		ret = zl_UpLoadIMG(&zl38005->spi->dev, FROM_FLASH);
	}
	break;
	/* Save Current Confiruration Register to flash */
	case ZL_SAVE_CFGR:
	{
		ret = -1;
		pr_debug_zl2("Save Current Confiruration Register to flash:\n");
		pr_debug_zl2("Check SPI Connection \n");
		if (zl_checkConnection(&zl38005->spi->dev) == -1)
			return -1;

		pr_debug_zl2("Goto Boot Mode \n");
		if (zl_SwitchModeTo(&zl38005->spi->dev, BOOT_MODE) == -1)
			return -1;

		pr_debug_zl2("Save Confiruration \n");
		zl_SaveCFG(&zl38005->spi->dev, 1);

		pr_debug_zl2("Come back to User Mode \n");
		if (zl_SwitchModeTo(&zl38005->spi->dev, USER_MODE) == -1)
			return -1;

		pr_debug_zl2("Host_Verify Op mode = UserMode \n");
		if (zl_Verify_OPER_MODE_is(&zl38005->spi->dev, USER_MODE) == -1)
			return -1;
		ret = 0;
		pr_debug_zl2("END \n");
	}
	break;
	/* Read Current Config (CFGRegister) from Ram */
	case ZL_READ_CURRENT_CFGR:
	{
		ret = -1;
		if (copy_from_user(&file_data, (struct file_data *)arg,\
				sizeof(struct file_data)))
			return -EFAULT;

		ret = -1;
		pr_debug_zl2("Read Current Config (CFGRegister) from Ram:\n");
		pr_debug_zl2("Check SPI Connection \n");
		if (zl_checkConnection(&zl38005->spi->dev) == -1)
			return -1;

		pr_debug_zl2("Goto Boot Mode \n");
		if (zl_SwitchModeTo(&zl38005->spi->dev, BOOT_MODE) == -1)
			return -1;

		pr_debug_zl2("Upload CFGRecord \n");
		if (zl_Upload_CFGRegister(&zl38005->spi->dev) == -1)
				return -1;

		pr_debug_zl2("Come back to User Mode \n");
		if (zl_SwitchModeTo(&zl38005->spi->dev, USER_MODE) == -1)
			return -1;

		pr_debug_zl2("Host_Verify Op mode = UserMode \n");
		if (zl_Verify_OPER_MODE_is(&zl38005->spi->dev, USER_MODE) == -1)
			return -1;

		pr_debug_zl2("END \n");
		ret = 0;
		return 0;
	}
	break;
	/* Invalid Command */
	default:
		printk(KERN_DEBUG "ioctl: Invalid Command Value \n");
		ret = -EINVAL;
	}

	/* Print time of Operation*/
	deadline = jiffies;
	msecs = jiffies_to_msecs(deadline - now);
	pr_debug_zl2("Time of IOCTL Operation = %dmsec, (%dsec) \n", \
		msecs, msecs/1000);

	return ret;
}

static const struct file_operations zl38005_fops = {
	.owner =	THIS_MODULE,
	.open =		zl38005_open,
	.release =	zl38005_close,
	.unlocked_ioctl = zl38005_ioctl,
};

/*
 * SPI stuff
 */

static struct class *zl38005_class;

static int zl38005_spi_probe(struct spi_device *spi)
{
	struct device *dev;
	struct zl38005 *zl38005;
	int status, err;
	struct zl38005_platform_data    *pdata = spi->dev.platform_data;
	int minor;

	dev_dbg(&spi->dev, "probing zl38005 spi device\n");
	/*printk(KERN_DEBUG "Probing zl38005 \n");*/

	minor = pdata->minor;
	if (minor >= DEVCOUNT)
		return -ENODEV;

	/* Allocate driver data */
	zl38005 = kzalloc(sizeof(*zl38005) +
			ARRAY_SIZE(zl38005_default_reg) *
				sizeof(struct zl38005_regs_s),
			GFP_KERNEL);
	if (!zl38005) {
		err = -ENOMEM;
		goto error;
	}

	zl38005->minor = minor;
	atomic_set(&zl38005->usage, 0);

	/* Initialize driver data */
	zl38005->reset_gpio = pdata->reset_gpio;

	sprintf(zl38005->reset_gpio_label, "ZL%d reset", minor);
	if (zl38005->reset_gpio) {
		if (gpio_request(zl38005->reset_gpio, zl38005->reset_gpio_label) < 0)
			printk(KERN_ERR "can't get %s \n",
				 zl38005->reset_gpio_label);
		gpio_direction_output(zl38005->reset_gpio, 0);
		gpio_export(zl38005->reset_gpio, 0);
	}

	sprintf(zl38005->cs_gpio_label, "ZL%d chip selet", minor);
	if (gpio_request((int)spi->controller_data, zl38005->cs_gpio_label) < 0)
		printk(KERN_ERR "can't get %s \n", zl38005->cs_gpio_label);
	gpio_direction_output((int)spi->controller_data, 1);

	/*printk(KERN_DEBUG "UnReset zl38005 \n");*/
	gpio_set_value(zl38005->reset_gpio, 1);

	memcpy(zl38005->reg_cache, zl38005_default_reg,
		ARRAY_SIZE(zl38005_default_reg) *
			sizeof(struct zl38005_regs_s));

	dev = device_create(zl38005_class, NULL,
			    MKDEV(zl38005_major, minor),
			    NULL, "%s%d", module_name, minor);
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	err = spi_setup(spi);
	if (err < 0)
		goto error;
	zl38005->spi = spi;
	spi_set_drvdata(spi, zl38005);

	/* Save device data for the char device! */
	zl38005_table[minor] = zl38005;

	zl38005_init_cache(zl38005,  zl38005->reg_cache);

	return 0;

error:
	kfree(zl38005);

	return err;
}

static int zl38005_spi_remove(struct spi_device *spi)
{
	struct zl38005 *zl38005 = spi_get_drvdata(spi);
	int minor = zl38005->minor;

	device_destroy(zl38005_class, MKDEV(zl38005_major, minor));
	kfree(zl38005);

	return 0;
}

static struct spi_driver zl38005_spi = {
	.driver = {
		.name = "zl38005",
		.owner = THIS_MODULE,
	},
	.probe = zl38005_spi_probe,
	.remove = zl38005_spi_remove,
};

/*
 * Module stuff
 */

static int __init zl38005_init(void)
{
	dev_t dev_id;
	int result;

	result = alloc_chrdev_region(&dev_id, 0, DEVCOUNT, "zl38005");
	if (result < 0) {
		printk(KERN_ERR "\nzl38005: Module intialization failed.\
                could not register character device");
                return -ENODEV;
        }

	zl38005_major = MAJOR(dev_id);

	/* registration of character device */
	register_chrdev(zl38005_major, DRIVER_NAME, &zl38005_fops);

	zl38005_class = class_create(THIS_MODULE, module_name);

	if (!zl38005_class) {
		cdev_del(&c_dev);
		unregister_chrdev(zl38005_major, DRIVER_NAME);
	}

	result = spi_register_driver(&zl38005_spi);
	if (result < 0) {
		class_destroy(zl38005_class);
		unregister_chrdev(zl38005_major, zl38005_spi.driver.name);
	}

	/* Initialize of character device */
	cdev_init(&c_dev, &zl38005_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &zl38005_fops;

	/* addding character device */
	result = cdev_add(&c_dev, dev_id, DEVCOUNT);
	if (result) {
		printk("NOTICE\nzl38005:Error %d adding zl38005\
				..error no:", result);
		unregister_chrdev_region(zl38005_major, DEVCOUNT);
	}

	return result;
}

module_init(zl38005_init);

static void __exit zl38005_exit(void)
{
	spi_unregister_driver(&zl38005_spi);
	cdev_del(&c_dev);
	unregister_chrdev_region(zl38005_major, DEVCOUNT);
	class_destroy(zl38005_class);
}
module_exit(zl38005_exit);

/* Module information */
MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_DESCRIPTION("ZL38005 acoustic echo canceller");
MODULE_LICENSE("GPLv2");
MODULE_VERSION(DRIVER_VERSION);

/* End of File*/
