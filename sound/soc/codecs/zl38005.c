/*
 * Copyright (C) 2011 Bticino S.p.A.
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
#include <linux/atomic.h>
#include <linux/smp_lock.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/spi/zl38005.h>
#include <linux/gpio.h>
#include <sound/soc.h>

#include "zl38005.h"

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

/* ZL38005 driver private data */
struct zl38005 {
	dev_t			dev;
	struct spi_device	*spi;
	struct list_head	device_entry;
	int			chip_select_gpio;
	char			chip_select_gpio_label[15];
	int			reset_gpio;
	char			reset_gpio_label[10];
	atomic_t		usage;
};

/* For registeration of charatcer device */
static struct cdev c_dev;

#define DEVCOUNT 2

static int zl38005_major;
static struct zl38005 *zl38005_table[DEVCOUNT];

static DEFINE_MUTEX(zl38005_list_lock);
static LIST_HEAD(zl38005_list);

struct zl38005_ioctl_par {
	unsigned int		addr;
	unsigned int		data;
};

/* ioctl() calls that are permitted to the /dev/zl38005 interface. */
#define ZL_MAGIC 'G'
#define ZL_RD_DATA_REG		_IOR(ZL_MAGIC, 0x09, struct zl38005_ioctl_par)
#define ZL_WR_DATA_REG		_IOR(ZL_MAGIC, 0x0a, struct zl38005_ioctl_par)
#define ZL_RD_INSTR_REG		_IOR(ZL_MAGIC, 0x0b, struct zl38005_ioctl_par)
#define ZL_WR_INSTR_REG		_IOR(ZL_MAGIC, 0x0c, struct zl38005_ioctl_par)
#define ZL_UPDATE		_IOR(ZL_MAGIC, 0x0d, struct file_data)

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
 *
 * High Level Funcion
 *
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
static int zl_checkConnection(struct device *dev)
{
	int Ack = -1;
	u16 Retry = 0;

	/* Start Tx and verifi Ack */
	while (Ack != RX_START_OK) {
		zl38005_wr_reg(dev, REG_0403, 0x0000, DMEM);  /* Clear Ack */
		zl38005_wr_reg(dev, REG_0402, TX_START, DMEM);
		zl_delay();
		zl38005_rd_reg(dev, REG_0403, &Ack, DMEM);
		/* pr_debug_zl2("Ack = %X \n", Ack); */
		Retry++;
		if (Retry == RETRY_COUNT){
			pr_debug_zl2("%s-%d: Conn Fail , Retry = %d \n", __FILE__, __LINE__, Retry);
			return -1;
		}
	}
	return 0;
}

/*
 *  set the STOP bit in register 0x0401 and to go back to boot mode
 */
static int zl_go_to_boot_mode(struct device *dev)
{
	int Data_Rx = 0;
	u16 Data = 0;

	/* Set Stop bit to Go to boot mode */
	zl38005_rd_reg(dev, 0x0401, &Data_Rx, DMEM); /* Read BootCtrl_1 */
	Data = Data_Rx | 0x0800 ;
	zl38005_wr_reg(dev, 0x0401, Data , DMEM);

	return 0;
}
/*----------------------------------------------------------------------------*/

const int ImageHeaderReg[16] = {
	0x0404, 0x040F, 0x0415, 0x0410, 0x0411, 0x0417, 0x0418, 0x0419,
	0x041A, 0x041B, 0x0405, 0x0406, 0x0407, 0x0408, 0x0409, 0x040A };

const int CFGRecordHeaderReg[16] = { 0x040B, 0x040C, 0x040D, 0x040E };

/*
 * __ Upload a romcode ________________________________________________________
 */
static int Zl_Upload_FWR_IMAGE(struct device *dev)
{
	int Retry = 0 ;
	int Ack = -1 ;
	int i = 0 ;
	u32 ZL_Write_Add = 0;

	/* clear status register */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Set LD_IMG to request load image and to tell
	 * that we want to boot a romcode */
	zl38005_wr_reg(dev, 0x0400, 0x0010, DMEM); /* 0b_xxxx_0000_0001_xxx0 */

	/*  wait for COM START ACK */
	Retry = 0 ;
	Ack = -1 ;
	while ( Ack != 0x0001){
		zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
		Retry++;
		if (Retry == RETRY_COUNT){
			pr_debug_zl2("Set LD_IMG Fail , Retry = %d \n", Retry);
			return -1;
		}
	}

	 /* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* clear the BOOT_FAIL bit (isn't automagically
	 * cleared, we have to do it) */
	zl38005_wr_reg(dev, 0x0412, 0x0000, DMEM);

	/* ____1. Write Fwr Image Header ____ */
	for(i=0; i<16; i++){
		if (zl38005_wr_reg(dev, ImageHeaderReg[i], \
				file_data.FWR_Header->Buff[i], DMEM)){
			pr_debug_zl2("Write Fwr Image Header Error \n");
			return -1;
		}
	}

	/* ____2. Write Fwr Image Instruction Memory Data ____ */
	ZL_Write_Add = file_data.FWR_Header->FIH.IMEM_StartAddress;
	for(i=0; i < file_data.FWR_Header->FIH.IMEM_Size ; i++){
		if (zl38005_wr_reg(dev, ZL_Write_Add, \
			file_data.IMEM_DataBuffer[i], IMEM)){
			pr_debug_zl2(" Write Fwr IMEM Error \n");
			return -1;
		}
		ZL_Write_Add +=1;
	}

	/* ____ 3. Write Fwr Image  Data Memory Data ____ */
	ZL_Write_Add = file_data.FWR_Header->FIH.DMEM_StartAddress;
	for(i=0; i < file_data.FWR_Header->FIH.DMEM_Size ; i++)
	{
		if (zl38005_wr_reg(dev, ZL_Write_Add, \
			file_data.DMEM_DataBuffer[i], DMEM)){
			pr_debug_zl2(" Write Fwr DMEM Error \n");
			return -1;
		}
		ZL_Write_Add +=1;
	}


	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Set LD_IMG_END : stop upload and check to make sure it uploaded ok */
	zl38005_wr_reg(dev, 0x0400, 0x0400, DMEM); /* 0b_xxxx_0100_0000_xxx0 */

	/* pr_debug_zl2(" WaitDone + Verifica Comm_OK == 2") */;

	/* Wait Done */
	Ack = 0;
	Retry = 0;
	while ( Ack == 0x0000){ /* NOT_DONE == 0? (Yes..Wait) */
		zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
		mdelay(20);
		Retry++;
		if (Retry == RETRY_COUNT){
			pr_debug_zl2("Set LD_IMG Fail , Retry = %d \n", Retry);
			return -1;
		}
	}

	if (Retry > 1)
		pr_debug_zl2(" WaitDone + Verifica Comm_OK == 2 : \
				ACK = %x, in Retry= %d \n", Ack, Retry--);

	/* Verify COMM_OK (== 2) */
	zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
	if (Ack == 0x0002) {
		/* clear status register (Target Ack) */
		zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);
		return 0;
	} else {
		pr_debug_zl2("Set COMM_OK : Fail , bootload romcode failed!");
		return -1;
	}
}
/*----------------------------------------------------------------------------*/

/*
 * __ Upload a configuration record ____________________________________________
 */
static int zl_Update_CFGRegister(struct device *dev)
{
	int Retry = 0 ;
	int Ack = -1 ;
	int i = 0 ;
	u32 ZL_Write_Add = 0;

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Set  LD_CONFIG ( 0b_xxxx_0000_0010_xxx0 )to request load image and
	 * to tell that we want to boot a romcode */
	zl38005_wr_reg(dev, 0x0400, 0x0020, DMEM); /* set LD_CONFIG */

	/*  wait for COM START ACK */
	Retry = 0 ;
	Ack = -1 ;
	while ( Ack != 0x0001){
		zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
		Retry++;
		if (Retry == RETRY_COUNT){
			pr_debug_zl2("Set LD_CONFIG Fail,Retry = %d \n", Retry);
			return -1;
		}
	}

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* ____1. Write Control Registers Header ____ */
	for(i=0; i<4; i++)
		if (zl38005_wr_reg(dev, CFGRecordHeaderReg[i], \
			file_data.CFG_Header->Buff[i], DMEM)) {
			pr_debug_zl2("Write Control Registers Error \n");
			return -1;
	}

	/* ____2. Write Write CFG Record Data ____ */
	ZL_Write_Add = file_data.CFG_Header->CFGRecord.StartAddress;
	for(i=0; i < file_data.CFG_Header->CFGRecord.Size ; i++){
		if (zl38005_wr_reg(dev, ZL_Write_Add, \
			file_data.CFG_DataBuffer[i], DMEM)) {
			pr_debug_zl2(" Write Write CFG Record Data Error \n");
			return -1;
		}
		ZL_Write_Add +=1;
	}

	/* clear status register (Target Ack) */
	zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);

	/* Set LD_CFG_END (0b_xxxx_1000_0000_xxx0)
	*stop upload and check to make sure it uploaded ok */
	zl38005_wr_reg(dev, 0x0400, 0x0800, DMEM); /* Set LD_CFG_END : */

	/* pr_debug_zl2(" WaitDone + Verifica Comm_OK == 2"); */

	/* Wait Done */
	Ack = 0;
	Retry = 0;
	while ( Ack == 0x0000){ /* NOT_DONE == 0? (Yes..Wait) */
		zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
		mdelay(20);
		Retry++;
		if (Retry == RETRY_COUNT){
			pr_debug_zl2("Set CFG_END Fail,Retry = %d \n", Retry);
			return -1;
		}
	}

	if (Retry > 1)
		pr_debug_zl2(" WaitDone + Verifica Comm_OK == 2 : \
		ACK = %x, in Retry= %d \n", Ack, Retry--);

	/* Verify COMM_OK (== 2) */
	zl38005_rd_reg(dev, 0x0403, &Ack, DMEM);
	if (Ack == 0x0002) {
		/* clear status register (Target Ack) */
		zl38005_wr_reg(dev, 0x0403, 0x0000, DMEM);
		return 0;
	} else{
		pr_debug_zl2("Set COMM_OK : Fail , bootload romcode failed!");
		return -1;
	}
}
/*----------------------------------------------------------------------------*/

/*
* __ Save To Flash & Set GO _____________________________________________
*
* Sets the bit which initiates a firmware save to flash when the device
* moves from a STOPPED state to a RUNNING state (by using the GO bit)
*/
static int zl38005_Save_To_Flash(struct device *dev)
{
	/*
	* Initiates a configuration record save to flash operation.
	* In boot mode so just flip the bit config record will actually
	* be saved when the GO bit is set
	*/
	zl38005_wr_reg(dev, 0x0401, 0x0001, DMEM); /* Save Conf : S_CFG = 1 */

	/*
	* Save image to flash (actually occurs when GO bit is set) &
	* Set GO Bit to running Fwr (0b_xxxx_0011_0000_xxx0)
	*/
	zl38005_wr_reg(dev, 0x0400, 0x0300, DMEM);   /*SaveFWR & GO*/

	return 0;
}
/*----------------------------------------------------------------------------*/

/*
* __ Verify_OPER_MODE __________________________________________________________
*/
static int zl38005_Host_Verify_OPER_MODE(struct device *dev)
{
	int Data = -1;
	int Retry = 0;

	/* Verify OPER_MODE reg 0x4000 == 0b_xxxx_xxxx_xxxx_xxx1 */
	while ((Data & 0x0001) != 0x0001) {
		zl38005_rd_reg(dev, 0x0400, &Data, DMEM);
		mdelay(20);
		Retry++;

		if (Retry == RETRY_COUNT) {
			pr_debug_zl2("Failure to enter in Normal \
			Operation Mode, Retry = %d \n", Retry);
			return -1;
		}
	}

	if (Retry > 1)
		pr_debug_zl2(" Operation Mode, in Retry= %d \n", Retry--);

	return 0;
}
/*----------------------------------------------------------------------------*/

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
/*
void zl_hardwareReset(UInt16T reset_mode)
{
	// send command
	zl_writeSPI(0xA0); // 10,1,00,000 - write, 8 bits, MAU Reset register
	if (reset_mode==ZL_ROM){
		// tell MAU control register to reset from ROM
		zl_writeSPI(0x01); // RAM/ROM bit low, reset high
	} else {
		// tell MAU control register to reset from RAM
		zl_writeSPI(0x05); // RAM/ROM bit high, reset high
	}
}
*/
/*----------------------------------------------------------------------------*/

/*
 * SoC stuff
 */

struct soc_mixer_zcontrol {
	struct spi_device *spi;
        int min, max, platform_max;
        unsigned int reg, rreg, shift, rshift, invert;
};

static int zl38005_spi_write(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_zcontrol *mc =
                (struct soc_mixer_zcontrol *) kcontrol->private_value;

	struct spi_device *spi = mc->spi;
        unsigned int reg = mc->reg;
        unsigned int shift = mc->shift;
        unsigned int mask = mc->max;
        unsigned int invert = mc->invert;
        unsigned int val = (ucontrol->value.integer.value[0] & mask);
        int valt;
	int ret;

	BUG_ON(!spi);

        if (invert)
                val = mask - val;

	ret = zl38005_rd_reg(&spi->dev, reg, &valt, DMEM);
        if (ret)
                return -EIO;

        if (((valt >> shift) & mask) == val)
                return 0;

        valt &= ~(mask << shift);
        valt |= val << shift;

	return zl38005_wr_reg(&spi->dev, reg, valt, DMEM);
}

static int zl38005_spi_read(struct snd_kcontrol *kcontrol,
                struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_zcontrol *mc =
                (struct soc_mixer_zcontrol *) kcontrol->private_value;

	struct spi_device *spi = mc->spi;
        unsigned int reg = mc->reg;
        unsigned int shift = mc->shift;
        unsigned int mask = mc->max;
        unsigned int invert = mc->invert;
        int val;
	int ret;

	BUG_ON(!spi);

	ret = zl38005_rd_reg(&spi->dev, reg, &val, DMEM);
        if (ret)
                return -EIO;

        ucontrol->value.integer.value[0] = ((val >> shift) & mask);

        if (invert)
                ucontrol->value.integer.value[0] =
                        mask - ucontrol->value.integer.value[0];

	return 0;
}

#define SOC_SINGLE_ZVALUE(xreg, xshift, xmax, xinvert) \
        ((unsigned long)&(struct soc_mixer_zcontrol) \
        {.spi = NULL, \
	.reg = xreg, .shift = xshift, .rshift = xshift, .max = xmax, \
        .platform_max = xmax, .invert = xinvert})

#define SOC_SINGLE_ZEXT(xname, xreg, xshift, xmax, xinvert,\
         xhandler_get, xhandler_put) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
        .info = snd_soc_info_volsw, \
        .get = xhandler_get, .put = xhandler_put, \
        .private_value = SOC_SINGLE_ZVALUE(xreg, xshift, xmax, xinvert) }

static const struct snd_kcontrol_new zl38005_controls[] = {
        SOC_SINGLE_ZEXT("ZLx-UGAIN", ZL38005_USRGAIN , 0, 0xffff, 0,
                        zl38005_spi_read, zl38005_spi_write),
        SOC_SINGLE_ZEXT("ZLx-SOUTGN", ZL38005_SYSGAIN , 8, 0xf, 0,
                        zl38005_spi_read, zl38005_spi_write),
        SOC_SINGLE_ZEXT("ZLx-MUTE_R", ZL38005_AEC_CTRL0 , 7, 1, 0,
                        zl38005_spi_read, zl38005_spi_write),
	SOC_SINGLE_ZEXT("ZLx-INJDIS", ZL38005_AEC_CTRL1, 6, 1, 0,
                        zl38005_spi_read, zl38005_spi_write),
};

/* This function is called from ASoC machine driver */
int zl38005_add_controls(unsigned int minor, struct snd_soc_codec *codec)
{
	struct zl38005 *zl38005;
	int i;
	struct soc_mixer_zcontrol *p;

	if (minor >= DEVCOUNT)
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	for (i = 0; i < ARRAY_SIZE(zl38005_controls); i++) {
		p = (struct soc_mixer_zcontrol *)
				zl38005_controls[i].private_value;
		p->spi = zl38005->spi;
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
	int err;

	if (minor >= DEVCOUNT)
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	/* Only open once */
	if (atomic_inc_return(&zl38005->usage) > 1) {
		printk(KERN_ERR "at least one zl38005 device already opened\n");
                atomic_dec(&zl38005->usage);
                return -EBUSY;
        }

	zl38005->spi->chip_select_gpio = zl38005->chip_select_gpio;
	err = spi_setup(zl38005->spi);
	if (err < 0)
		return err;

	return 0;
}

static int zl38005_close(struct inode *inode, struct file *filp)
{
	struct zl38005 *zl38005;
	int minor = MINOR(inode->i_rdev);

	if (minor >= DEVCOUNT)
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

	if (minor >= DEVCOUNT)
		return -ENODEV;

	zl38005 = zl38005_table[minor];
	BUG_ON(!zl38005);

	switch (cmd) {
	case ZL_WR_DATA_REG:
		if (copy_from_user(&zl38005_ioctl_par, \
			(struct zl38005_ioctl_par *)arg,\
			 sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		pr_debug_zl1("ioctl wr DATA = %X \n", zl38005_ioctl_par.data);
		ret = zl38005_wr_reg(&zl38005->spi->dev, \
			zl38005_ioctl_par.addr, zl38005_ioctl_par.data, DMEM);
	break;
	case ZL_RD_DATA_REG:
		if (copy_from_user(&zl38005_ioctl_par, \
			(struct zl38005_ioctl_par *)arg,\
			 sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, \
		 (u16)zl38005_ioctl_par.addr, &zl38005_ioctl_par.data, DMEM);
		if (!ret) {
			pr_debug_zl1("ioctl rd DATA = %X \n ", \
				zl38005_ioctl_par.data);
			if (copy_to_user((struct zl38005_ioctl_par *)arg,\
			 &zl38005_ioctl_par, sizeof(struct zl38005_ioctl_par)))
				return -EFAULT;
		} else
			return -EAGAIN;
	break;
	case ZL_WR_INSTR_REG :
		if (copy_from_user(&zl38005_ioctl_par,
				   (struct zl38005_ioctl_par *)arg,
				   sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		pr_debug_zl1("ioctl wr DATA = %X \n", zl38005_ioctl_par.data);
		ret = zl38005_wr_reg(&zl38005->spi->dev,
				     zl38005_ioctl_par.addr,
				     zl38005_ioctl_par.data, IMEM);
	break;
	case ZL_RD_INSTR_REG :
		if (copy_from_user(&zl38005_ioctl_par,
				   (struct zl38005_ioctl_par *)arg,
				   sizeof(struct zl38005_ioctl_par)))
			return -EFAULT;
		pr_debug_zl1("ioctl wr ADDR = %X \n", zl38005_ioctl_par.addr);
		ret = zl38005_rd_reg(&zl38005->spi->dev, \
			(u16)zl38005_ioctl_par.addr, &zl38005_ioctl_par.data,\
			 IMEM);
		if (!ret) {
			pr_debug_zl1("ioctl rd DATA = %X\n", \
					zl38005_ioctl_par.data);
			if (copy_to_user((struct zl38005_ioctl_par *)arg, \
				&zl38005_ioctl_par, \
				sizeof(struct zl38005_ioctl_par)))
				return -EFAULT;
		} else
		return -EAGAIN;
	break;
	case ZL_UPDATE:
		if (copy_from_user(&file_data, (struct file_data *)arg,\
				sizeof(struct file_data)))
			return -EFAULT;

	/*
	pr_debug_zl1();   //printk("%s %d \n", __func__, __LINE__ );
	printk("FWR_Header        : %04x \n", file_data.FWR_Header->Buff[0]);
	printk("IMEM_DataBuffer   : %06x \n", file_data.IMEM_DataBuffer[0]);
	printk("DMEM_DataBuffer   : %04x \n", file_data.DMEM_DataBuffer[0]);
	printk("CFG_Header        : %04x \n", file_data.CFG_Header->Buff[0]);
	printk("CFG_DataBuffer    : %04x \n", file_data.CFG_DataBuffer[0]);
	pr_debug_zl1();   //printk("%s %d \n", __func__, __LINE__ );
	*/

		ret = 0;

		/* __Fwr Update Procedure:__ */
		pr_debug_zl2("Fwr_update \n");

		/* __Slave ZL SPI Port Test__ */
		pr_debug_zl2("checkConnection \n");
		ret = zl_checkConnection(&zl38005->spi->dev);
		if (ret < 0)
			return -1;

		pr_debug_zl2("Goto Boot Mode \n");
		ret = zl_go_to_boot_mode(&zl38005->spi->dev);
		if (ret < 0)
			return -1;

		/* __Update Firmware__ */
		pr_debug_zl2("Upload_FWR_IMAGE \n");
		ret = Zl_Upload_FWR_IMAGE (&zl38005->spi->dev);
		if (ret < 0)
			return -1;

		/* __Update CFGRegister__ */
		pr_debug_zl2("Update_CFGRegister \n");
		ret = zl_Update_CFGRegister (&zl38005->spi->dev);
		if (ret < 0)
			return -1;

		/* __Save to Flash__ */
		pr_debug_zl2("Save to Flash \n");
		ret = zl38005_Save_To_Flash (&zl38005->spi->dev);
		if (ret < 0)
			return -1;

		/* __Host_Verify Op mode__ */
		pr_debug_zl2("_Host_Verify Op mode \n");
		ret = zl38005_Host_Verify_OPER_MODE(&zl38005->spi->dev);

		if (ret < 0)
			return -1;

		pr_debug_zl2("END \n");
			return 0;
	break;
	default:
		printk(KERN_DEBUG "ioctl: Invalid Command Value");
		ret = -EINVAL;
	}
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

	for (minor = 0; minor < DEVCOUNT; minor++) {
		/* Allocate driver data */
		zl38005 = kzalloc(sizeof(*zl38005), GFP_KERNEL);
		if (!zl38005)
			return -ENOMEM;
		atomic_set(&zl38005->usage, 0);

		/* Initialize driver data */
		zl38005->chip_select_gpio = pdata[minor].chip_select_gpio;
		zl38005->reset_gpio = pdata[minor].reset_gpio;
		sprintf(zl38005->reset_gpio_label, "ZL%d reset", minor);
		if (zl38005->reset_gpio) {
			if (gpio_request(zl38005->reset_gpio, zl38005->reset_gpio_label) < 0)
				printk(KERN_ERR "can't get %s reset\n", zl38005->reset_gpio_label);
			gpio_direction_output(zl38005->reset_gpio, 1);
			gpio_export(zl38005->reset_gpio, 0);
		}
		sprintf(zl38005->chip_select_gpio_label,
			"ZL%d chip selet", minor);
		if (zl38005->chip_select_gpio) {
			if (gpio_request(zl38005->chip_select_gpio,
					 zl38005->chip_select_gpio_label) < 0)
				printk(KERN_ERR "can't get %s reset\n",
				       zl38005->reset_gpio_label);
			gpio_direction_output(zl38005->chip_select_gpio, 1);
		}
		if (zl38005_table[minor])
			continue;
		zl38005_table[minor] = zl38005;
		dev = device_create(zl38005_class, NULL,
				    MKDEV(zl38005_major, minor),
				    NULL, "%s%d", module_name, minor);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

		spi->bits_per_word = 8;
		spi->mode = SPI_MODE_0;
		err = spi_setup(spi);
		if (err < 0)
			return err;
		zl38005->spi = spi;
		spi_set_drvdata(spi, zl38005);
	}

	return 0;
}

static int zl38005_spi_remove(struct spi_device *spi)
{
	int minor;

	for (minor = 0; minor < DEVCOUNT; minor++)
		device_destroy(zl38005_class, MKDEV(zl38005_major, minor));

	class_destroy(zl38005_class);
	kfree(spi_get_drvdata(spi));
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
}
module_exit(zl38005_exit);

/* Module information */
MODULE_AUTHOR("Raffaele Recalcati <raffaele.recalcati@bticino.it>");
MODULE_DESCRIPTION("ZL38005 acoustic echo canceller");
MODULE_LICENSE("GPLv2");
MODULE_VERSION(DRIVER_VERSION);

/* End of File*/
