/*
 * dm646x_pci_targetdrv.c
 *  Description:
 *  DM6467 PCI Target driver.
 *
 *
 * Copyright (C) 2009, Texas Instruments, Incorporated
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */


/*******************************************************************************
 *	LOCAL INCLUDE FILES
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>	/**< debug */
#include <linux/device.h>	/**< debug */
#include <linux/slab.h> 	/**< kmalloc() */
#include <linux/fs.h> 		/**< everything\ldots{} */
#include <linux/types.h> 	/**< size_t */
#include <linux/fcntl.h>	/**< O_ACCMODE */
#include <linux/uaccess.h>
#include <linux/ioport.h>	/**< For request_region, check_region etc */
#include <linux/io.h>             /**< For ioremap_nocache */
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <asm/irq.h>
#include "dm646x_pci_targetdrv.h"


/*******************************************************************************
 *	LOCAL DEFINES
 */
#define DM646X_PCI_MODNAME      ("DM646x PCI Boot")
#define DM646X_DEV_COUNT        (1)

/* U-Boot size is limited to 512KB */
#define DM646X_UBOOT_MAX_SIZE   (0x80000)

/* Script/Communication Area is limited to 512KB */
#define DM646X_COMM_MAX_SIZE    (0x80000)

/* Linux image area is limited to 7MB */
#define DM646X_LINUX_MAX_SIZE   (0x700000)

/**
 *    KERN_EMERG > KERN_ALERT > KERN_CRIT >
 *    KERN_ERR > KERN_WARNING > KERN_INFO > KERN_DEBUG
 */
#define LOG_LEVEL               KERN_ALERT

/*
 * These macros provide the virtual address of PCI and device register at
 * specified offset respectively. Use reg_virt initialized from MMR BAR base
 * virtual mapping.
 */
#define PCI_REGV(reg)           ((reg_virt + DM6467PCI_PCIREG_BASE \
					- DM6467PCI_MMRREG_BASE) + reg)
#define DEV_REGV(reg)           ((reg_virt + DM6467PCI_DEVREG_BASE \
					- DM6467PCI_MMRREG_BASE) + reg)


/*******************************************************************************
 *	ENUMERATED DATATYPES
 */
enum E_PCI_BAR {
	  E_PCI_BAR_TCM_RAM	= 0 	/* 0x10010000 */
	, E_PCI_BAR_EMIF_REGS		/* 0x20000000 */
	, E_PCI_BAR_CHIP_MMR		/* 0x01C00000 */
	, E_PCI_BAR_L2_RAM		/* 0x11810000 */
	, E_PCI_BAR_DDR2_A	    	/* 0x80000000 */
	, E_PCI_BAR_DDR2_B	    	/* 0x80800000 */
	, E_PCI_BAR_MAX		    	/* max 6 in number */
} EPciBar;
/*******************************************************************************
 *	LOCAL FUNCTION PROTOTYPES
 */
static int dm646x_pci_find_device(void);
static int dm646x_pci_read_BAR(void);
static int dm646x_pci_remap_BAR(u32 bar_number, u32 new_window);
static void dm646x_pci_set_master(void);

static ssize_t dm646x_pci_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long arg);
static int dm646x_pci_mmap(struct file *filp, struct vm_area_struct *vma);
static int __init dm646x_pci_init(void);
static void __exit dm646x_pci_cleanup(void);

/*******************************************************************************
 *	GLOBAL VARIABLES
 */
struct pci_dev *dm646x_pci_dev;
u32  shrd_intr_var;

u32  tcm_base        = 0, tcm_len         = 0;
u32  emif_reg_base   = 0, emif_reg_len    = 0;
u32  reg_base        = 0, reg_len         = 0;
u32  l2_ram_base     = 0, l2_ram_len      = 0;
u32  ddr2a_base      = 0, ddr2a_len       = 0;
u32  ddr2b_base      = 0, ddr2b_len       = 0;

u32	*tcm_virt;
u32	*emif_virt;
u32	reg_virt;
u32	*ddr2a_virt;

u32	*ddr2b_virt;
u32  *comm_virt;
u32  *linux_virt;

static int dm646x_pci_major;	    	                /**< Major number */
static struct cdev dm646x_pci_cdev;                     /**< CDev object */
static struct class *dm646x_pci_class;
static dev_t dm646x_dev_id;

/**
 *	Structure that declares the usual file
 *	access functions
 */
static const struct file_operations dm646x_pci_fops = {
	.owner =    THIS_MODULE,
	.mmap  =    dm646x_pci_mmap,
	.ioctl =    dm646x_pci_ioctl,
};

/* =============================================================================
 *  @func   dm646x_pci_find_device
 *
 *  @desc   This function locates DM6467 PCI cards on system.
 *
 *  @modif  None.
 *  ============================================================================
 */
static int dm646x_pci_find_device(void)
{
	struct pci_dev *dev = NULL;

	while (NULL !=
		(dev = pci_get_device(PCI_TI_VENDOR, PCI_TI_DEVICE, dev))) {
		pr_info("dm646xpci: Found DM646x PCI device at 0x%08x\n",
				(int)dev);
		if ((dev->class >> 8) == PCI_CLASS_BRIDGE_HOST) {
			pr_warning("dm646xpci: skipping DM646x PCI Host...");
			continue;
		}
		dm646x_pci_dev = dev;
		return 0;
	}
	pr_err("dm646xpci: No DM646x PCI Target found on bus\n");
	return -1;
}

/* =============================================================================
 *  @func   dm646x_pci_remap_BAR
 *
 *  @desc   This function changes the current mapping of the BAR registers
 *
 *  @modif  None.
 *  ============================================================================
 */
static int dm646x_pci_remap_BAR(u32 bar_number, u32 new_window)
{
	/* switch on the requested BAR */
	switch (bar_number) {
	case E_PCI_BAR_TCM_RAM:
	{
		dev_info(&dm646x_pci_dev->dev,
				"Original BAR-0 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(0))));
		__raw_writel(new_window, PCI_REGV(PCIBARTRL(0)));
		dev_info(&dm646x_pci_dev->dev,
				"New BAR-0 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(0))));
		break;
	}
	case E_PCI_BAR_EMIF_REGS:
	{
		dev_info(&dm646x_pci_dev->dev,
				"Original BAR-1 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(1))));
		__raw_writel(new_window, PCI_REGV(PCIBARTRL(1)));
		dev_info(&dm646x_pci_dev->dev,
				"New BAR-1 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(1))));
		break;
	}
	case E_PCI_BAR_CHIP_MMR:
	{
		dev_info(&dm646x_pci_dev->dev,
				"Original BAR-2 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(2))));
		__raw_writel(new_window, PCI_REGV(PCIBARTRL(2)));
		dev_info(&dm646x_pci_dev->dev,
				"New BAR-2 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(2))));
		break;
	}
	case E_PCI_BAR_L2_RAM:
	{
		dev_info(&dm646x_pci_dev->dev,
				"Original BAR-3 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(3))));
		__raw_writel(new_window, PCI_REGV(PCIBARTRL(3)));
		dev_info(&dm646x_pci_dev->dev,
				"New BAR-3 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(3))));
		break;
	}
	case E_PCI_BAR_DDR2_A:
	{
		dev_info(&dm646x_pci_dev->dev,
				"Original BAR-4 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(4))));
		__raw_writel(new_window, PCI_REGV(PCIBARTRL(4)));
		dev_info(&dm646x_pci_dev->dev,
				"New BAR-4 value =\t0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(4))));
		break;
	}
	case E_PCI_BAR_DDR2_B:
	{
		dev_info(&dm646x_pci_dev->dev,
				"Original BAR-5 value\t= 0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(5))));
		__raw_writel(new_window, PCI_REGV(PCIBARTRL(5)));
		dev_info(&dm646x_pci_dev->dev,
				"New BAR-5 value =\t0x%08x\n",
				(int)__raw_readl(PCI_REGV(PCIBARTRL(5))));
		break;
	}
	default:
	{
		dev_err(&dm646x_pci_dev->dev, "Invalid BAR number\n");
		return -1;
	}
	} /* switch(bar_number) */
	return 0;
}

/* =============================================================================
 *  @func   dm646x_pci_read_BAR
 *
 *  @desc   This function reads config.
 *
 *  @modif  None.
 *  ============================================================================
 */
static int dm646x_pci_read_BAR(void)
{
	int index;
	u32  bar_start[E_PCI_BAR_MAX];
	u32  bar_len[E_PCI_BAR_MAX];
	u32  bar_flags[E_PCI_BAR_MAX];

	dev_info(&dm646x_pci_dev->dev, "BAR Configuration - \n\t   "
			"Start\t|\tLength\t|\tFlags\n");
	for (index = E_PCI_BAR_TCM_RAM; index < E_PCI_BAR_DDR2_A; index++) {
		bar_start[index] = pci_resource_start(dm646x_pci_dev, index);
		bar_len[index] = pci_resource_len(dm646x_pci_dev, index);
		bar_flags[index] = pci_resource_flags(dm646x_pci_dev, index);

		if (bar_flags[index] & IORESOURCE_IO) {
			dev_err(&dm646x_pci_dev->dev,
				"This driver does not support PCI IO.\n");
			return -1;
		}

		dev_info(&dm646x_pci_dev->dev, "\t0x%08x\t|\t%d\t|\t0x%08x\n",
				(int)bar_start[index], (int)bar_len[index],
				(int)bar_flags[index]);
	}


	/* ---------------------------------------------------------------------
	* Maintain information on L2 RAM  memory region
	* ---------------------------------------------------------------------
	*/
	l2_ram_base = bar_start[E_PCI_BAR_L2_RAM];
	l2_ram_len  = bar_len[E_PCI_BAR_L2_RAM];


	/* ---------------------------------------------------------------------
	* Map the ARM TCM RAM  memory region
	* ---------------------------------------------------------------------
	*/
	tcm_base = bar_start[E_PCI_BAR_TCM_RAM];

	/* Map the memory region. */
	request_mem_region(tcm_base, bar_len[E_PCI_BAR_TCM_RAM], "DV-HD");

	tcm_virt = ioremap_nocache(bar_start[E_PCI_BAR_TCM_RAM],
					bar_len[E_PCI_BAR_TCM_RAM]);
	if (!tcm_virt) {
		dev_err(&dm646x_pci_dev->dev, "ARM TCM RAM memory could "
				"not be remapped\n");
		return -1;
	}
	dev_info(&dm646x_pci_dev->dev, "ARM TCM RAM memory mapped to 0x%08x\n",
			(int)tcm_virt);
	tcm_len = bar_len[E_PCI_BAR_TCM_RAM];

	/* ---------------------------------------------------------------------
	* Map the EMIF Memory Controller registers memory region
	* ---------------------------------------------------------------------
	*/
	emif_reg_base = bar_start[E_PCI_BAR_EMIF_REGS];

	/* Map the memory region. */
	request_mem_region(emif_reg_base,
				bar_len[E_PCI_BAR_EMIF_REGS], "DV-HD");

	emif_virt = ioremap_nocache(bar_start[E_PCI_BAR_EMIF_REGS],
					bar_len[E_PCI_BAR_EMIF_REGS]);
	if (!emif_virt) {
		dev_err(&dm646x_pci_dev->dev,
				"DDR2 area could not be remapped\n");
		return -1;
	}
	dev_info(&dm646x_pci_dev->dev, "DDR2 Memory mapped to 0x%08x\n",
			(int)emif_virt);
	emif_reg_len = bar_len[E_PCI_BAR_EMIF_REGS];

	/* ---------------------------------------------------------------------
	* Map the REG memory region
	* ---------------------------------------------------------------------
	*/
	reg_base = bar_start[E_PCI_BAR_CHIP_MMR] ;

	/* Map the memory region. */
	request_mem_region(reg_base, bar_len[E_PCI_BAR_CHIP_MMR], "DV-HD");

	reg_virt = (u32)ioremap_nocache(bar_start[E_PCI_BAR_CHIP_MMR],
					bar_len[E_PCI_BAR_CHIP_MMR]) ;
	if (!reg_virt) {
		dev_err(&dm646x_pci_dev->dev,
				"MMR registers could not be mapped\n");
		return -1;
	}
	dev_info(&dm646x_pci_dev->dev, "MMR registers mapped to 0x%08x:%#x\n",
			(int)reg_virt, bar_start[E_PCI_BAR_CHIP_MMR]);
	reg_len = bar_len[E_PCI_BAR_CHIP_MMR];


	/* ---------------------------------------------------------------------
	* Change the mapping for the DDR2 BAR registers
	* ---------------------------------------------------------------------
	*/
	if (0 != dm646x_pci_remap_BAR(E_PCI_BAR_DDR2_A, 0x82000000)) {
		dev_err(&dm646x_pci_dev->dev,
				"Remapping of BAR4 for DDR2-A failed\n");
		return -1;
	}

	if (0 != dm646x_pci_remap_BAR(E_PCI_BAR_DDR2_B, 0x82800000)) {
		dev_err(&dm646x_pci_dev->dev,
				"Remapping of BAR4 for DDR2-B failed\n");
		return -1;
	}

	for (index = E_PCI_BAR_DDR2_A; index < E_PCI_BAR_MAX; index++) {
		bar_start[index] = pci_resource_start(dm646x_pci_dev, index);
		bar_len[index] = pci_resource_len(dm646x_pci_dev, index);
		bar_flags[index] = pci_resource_flags(dm646x_pci_dev, index);
		dev_info(&dm646x_pci_dev->dev, "\t0x%08x\t|\t%d\t|\t0x%08x\n",
				(int)bar_start[index], (int)bar_len[index],
				(int)bar_flags[index]);
	}

	/* ---------------------------------------------------------------------
	* Map the DDR2-A memory region
	* ---------------------------------------------------------------------
	*/
	ddr2a_base = bar_start[E_PCI_BAR_DDR2_A] ;

	/* Map the memory region. */
	request_mem_region(ddr2a_base, bar_len[E_PCI_BAR_DDR2_A], "DV-HD");

	ddr2a_virt = ioremap_nocache(bar_start[E_PCI_BAR_DDR2_A],
					DM646X_UBOOT_MAX_SIZE);
	if (!ddr2a_virt) {
		dev_err(&dm646x_pci_dev->dev,
				"DDR2 memory could not be mapped\n");
		return -1;
	}

	comm_virt = ioremap_nocache(bar_start[E_PCI_BAR_DDR2_A]
					+ DM646X_UBOOT_MAX_SIZE,
					DM646X_COMM_MAX_SIZE);
	if (!comm_virt) {
		dev_err(&dm646x_pci_dev->dev,
				"Communication/Script area failed to remap\n");
		return -1;
	}

	linux_virt = ioremap_nocache((bar_start[E_PCI_BAR_DDR2_A]
			+ DM646X_UBOOT_MAX_SIZE + DM646X_COMM_MAX_SIZE),
			bar_len[E_PCI_BAR_DDR2_A] - DM646X_UBOOT_MAX_SIZE
			- DM646X_COMM_MAX_SIZE);
	if (!linux_virt) {
		dev_err(&dm646x_pci_dev->dev,
				"Error: Linux memory could not be remapped\n");
		return -1;
	}
	dev_info(&dm646x_pci_dev->dev, "DDR2 memory mapped to 0x%08x\n",
			(int)ddr2a_virt);
	dev_info(&dm646x_pci_dev->dev,
			"Communication/Script area mapped to 0x%08x\n",
			(int)comm_virt);
	dev_info(&dm646x_pci_dev->dev, "Linux area mapped to 0x%08x\n",
			(int)linux_virt);

	ddr2a_len = bar_len[E_PCI_BAR_DDR2_A];

	/* ---------------------------------------------------------------------
	* Map the DDR2-B memory region
	* ---------------------------------------------------------------------
	*/
	ddr2b_base = bar_start[E_PCI_BAR_DDR2_B];

	/* Map the memory region. */
	request_mem_region(ddr2b_base, bar_len[E_PCI_BAR_DDR2_B], "DV-HD");

	ddr2b_virt = ioremap_nocache(bar_start[E_PCI_BAR_DDR2_B],
			bar_len[E_PCI_BAR_DDR2_B] - DM646X_COMM_MAX_SIZE);
	if (!ddr2b_virt) {
		dev_err(&dm646x_pci_dev->dev,
				"DDR2 memory could not be remapped\n");
		return -1;
	}
	dev_info(&dm646x_pci_dev->dev, "DDR2-B memory mapped to 0x%08x\n",
			(int)ddr2b_virt);
	ddr2b_len = bar_len[E_PCI_BAR_DDR2_B];

	return 0;
}

/* =============================================================================
 *  @func   dm646x_pci_set_master
 *
 *  @desc   This function makes the given device to be master.
 *
 *  @modif  None.
 *  ============================================================================
 */
static void dm646x_pci_set_master(void)
{
	s32   ret_val ;
	u16  cmd_val ;
	struct pci_dev *dev = dm646x_pci_dev;

	/*
	 *  Set the desired PCI dev to be master, this internally sets the
	 *  latency timer.
	 */
	pci_set_master(dev);
	pci_write_config_byte(dev, PCI_LATENCY_TIMER, 0x80);

	/* Add support memory write invalidate */
	ret_val = pci_set_mwi(dev);

	pci_read_config_word(dev, PCI_COMMAND, (u16 *) &cmd_val);

	/* and set the master bit in command register. */
	cmd_val |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_SERR);

	pci_write_config_word(dev, PCI_COMMAND, cmd_val);
}

/* =============================================================================
 *  @func  dm646x_pci_ioctl
 *
 *  @desc   This function provides the interface to the application code
 *	        to transfer data to PCI mapped memory spaces. It will be used
 *          to initially transfer the UBL code followed by the u-boot.bin
 *	        file. This code largely depends on the handshaking done using
 *	        the BootComplete flag.
 *
 *  @modif  None.
 *  ============================================================================
 */
ssize_t dm646x_pci_ioctl(struct inode *inode, struct file *filp,
				unsigned int cmd, unsigned long arg)
{
	int ret_val = 0 ;

	switch (cmd) {
	/**
	*  1.  Run the UBL code on the ARM TCM IRAM
	*/
	case DM646X_PCI_RUN_TCM_IMG:
	{
		/* Copy the start location to predefined IRAM location */
		tcm_virt[0x7E80/4] = 0x20;

		/* Set the boot complete flag, BC flag */
		__raw_writel(__raw_readl(DEV_REGV(BOOTSTAT)) | 0x1,
					DEV_REGV(BOOTSTAT));

		/* Wait until BC bit is cleared by the UBL */
		while ((__raw_readl(DEV_REGV(BOOTSTAT)) & 0x1) == 0x1)
			udelay(1000);

		dev_info(&dm646x_pci_dev->dev,
				"\nUBL completed its job; BC is cleared;"
				" DDR2 is initialized\n");

		return 0;
	}

	/**
	*  2.  Remap BAR to a new location
	*/
	case DM646X_PCI_REMAP_BAR:
	{
		struct DM646x_Remap_Bar_Args *args
				= (struct DM646x_Remap_Bar_Args *) arg;
		dev_info(&dm646x_pci_dev->dev, "Remapping BAR%d to %#x...\n",
				(int)args->bar_number, (int)args->new_window);
		if (0 != dm646x_pci_remap_BAR(args->bar_number,
					args->new_window)) {
			dev_err(&dm646x_pci_dev->dev,
				"Error: Remapping of BAR%d to 0x%08x failed\n",
				(int)args->bar_number,
				(int)args->new_window);
			return -1;
		}
		return 0;
	}

	/**
	*  3.  Indicate to the UBL where to find the U-Boot start location
	*      and boot the image
	*/
	case DM646X_PCI_BOOT_IMAGE:
	{
		/* Copy the start location of U-boot to last IRAM location */
		tcm_virt[0x7FFC/4] = (unsigned int)arg;

		/* Set the boot complete flag, BC flag */
		__raw_writel(__raw_readl(DEV_REGV(BOOTSTAT)) | 0x1,
					DEV_REGV(BOOTSTAT));

		break;
	}

	/**
	*  4.  Generate an interrupt to the DM6467
	*/
	case DM646X_PCI_GEN_INTRPT:
	{
		dev_err(&dm646x_pci_dev->dev, "Generating interrupt on DM646x "
				"target is not supported.\n");
		ret_val = -1;
		break;
	}

	/**
	*  5. Send data to device using EDMA and shared PCI space
	*/
	case DM646X_PCI_EDMA_XFER:
	{
		dev_err(&dm646x_pci_dev->dev, "Initiating EDMA on PCI Target "
				"from Host is not supported.\n");
		ret_val = -1;
		break;
	}

	default:
		return -1;
	}
	return ret_val;
}

/* =============================================================================
 *  @func   dm646x_pci_mmap
 *
 *  @desc   The mmap routine will provide method to allow applications
 *          a direct access to the PCI mapped memory area
 *
 *  @modif  None
 *  ============================================================================
 */
int dm646x_pci_mmap(struct file *filp, struct vm_area_struct *vma)
{
	switch ((int)vma->vm_pgoff) {
	case E_PCI_BAR_TCM_RAM:
		vma->vm_pgoff = tcm_base;
		break;
	case E_PCI_BAR_EMIF_REGS:
		vma->vm_pgoff = emif_reg_base;
		break;
	case E_PCI_BAR_CHIP_MMR:
		vma->vm_pgoff = reg_base;
		break;
	case E_PCI_BAR_L2_RAM:
		vma->vm_pgoff = l2_ram_base;
		break;
	case E_PCI_BAR_DDR2_A:
		vma->vm_pgoff = ddr2a_base;
		break;
	case E_PCI_BAR_DDR2_B:
		vma->vm_pgoff = ddr2b_base;
		break;
	default:
		return -EAGAIN;
	}
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, (vma->vm_pgoff >> PAGE_SHIFT),
			vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN ;

	vma->vm_pgoff = vma->vm_pgoff >> PAGE_SHIFT;
	return 0 ;
}

/* =============================================================================
 *  @func   dm646x_pci_init
 *
 *  @desc   This function will be called when this module is inserted. It
 *	    will initialize the DM700 as a PCI device on the host machine
 *	    and will then read the BAR addresses to configure the DDR to
 *	    the PCI memory mapped area. Next, the host will initialize the
 *	    DDR of DM700 device to allow copying of the uBoot binary.
 *
 *  @modif  None.
 *  ============================================================================
 */
int __init dm646x_pci_init(void)
{
	int ret_val;

	/* ---------------------------------------------------------------------
	*  Find the DM646x device on the PCI bus
	* ---------------------------------------------------------------------
	*/
	if (dm646x_pci_find_device())
		return -1;

	dev_warn(&dm646x_pci_dev->dev, "This driver supports booting the "
			"first DM646x target found on the bus\n");

	/* ---------------------------------------------------------------------
	*  1.  Register a character driver to send data to this driver
	* ---------------------------------------------------------------------
	*/
	ret_val = alloc_chrdev_region(&dm646x_dev_id, 1, 0, DM646X_PCI_MODFILE);
	if (ret_val) {
		dev_err(&dm646x_pci_dev->dev,
				"could not allocate the character driver");
		goto err_post_pci;
	}
	dm646x_pci_major = MAJOR(dm646x_dev_id);

	/* ---------------------------------------------------------------------
	*  2.  Create the node for access through the application
	* ---------------------------------------------------------------------
	*/
	cdev_init(&dm646x_pci_cdev, &dm646x_pci_fops);
	dm646x_pci_cdev.owner = THIS_MODULE;
	dm646x_pci_cdev.ops = &dm646x_pci_fops;

	ret_val = cdev_add(&dm646x_pci_cdev, dm646x_dev_id, DM646X_DEV_COUNT);
	if (ret_val) {
		dev_err(&dm646x_pci_dev->dev,
				"Failed creation of node for PCI boot\n");
		unregister_chrdev_region(dm646x_dev_id, DM646X_DEV_COUNT);
		goto err_post_pci;
	}
	dev_info(&dm646x_pci_dev->dev, "Major number %d assigned\n",
			dm646x_pci_major);

	/* ---------------------------------------------------------------------
	*  3.  Creation of a class structure for pseudo file system access
	* ---------------------------------------------------------------------
	*/
	dm646x_pci_class = class_create(THIS_MODULE, DM646X_PCI_MODFILE);
	if (!dm646x_pci_class) {
		cdev_del(&dm646x_pci_cdev);
		unregister_chrdev_region(dm646x_dev_id, DM646X_DEV_COUNT);
		dev_err(&dm646x_pci_dev->dev,
				"Failed to add device to sys fs\n");
		ret_val = -1;
		goto err_post_pci;
	}
	device_create(dm646x_pci_class, NULL, dm646x_dev_id,
				NULL, DM646X_PCI_MODFILE);
	dev_info(&dm646x_pci_dev->dev, "Added device to the sys file system\n");


	/* ---------------------------------------------------------------------
	*  5.  Enable the PCI device
	* ---------------------------------------------------------------------
	*/
	ret_val = pci_enable_device(dm646x_pci_dev);
	if (ret_val) {
		dev_err(&dm646x_pci_dev->dev, "Failed to enable device.\n");
		goto err_post_cdev;
	}

	/* ---------------------------------------------------------------------
	*  6.  Set PCI as the bus master
	* ---------------------------------------------------------------------
	*/
	dm646x_pci_set_master() ;

	/* ---------------------------------------------------------------------
	*  7.  Access DM646x BARs and mapped regions over PCI  bus
	* ---------------------------------------------------------------------
	*/
	ret_val = dm646x_pci_read_BAR();
	if (ret_val == -1) {
		dev_err(&dm646x_pci_dev->dev, "Host device could not read the "
				"DM646x BAR registers\n");
		goto err_post_cdev;
	}

	/* ---------------------------------------------------------------------
	*  8.  Populate the PCI driver data
	* ---------------------------------------------------------------------
	*/
	pci_set_drvdata(dm646x_pci_dev, ddr2b_virt);

	return 0 ;

err_post_cdev:
	class_destroy(dm646x_pci_class);
	cdev_del(&dm646x_pci_cdev);
	unregister_chrdev_region(dm646x_dev_id, DM646X_DEV_COUNT);

err_post_pci:
	pci_dev_put(dm646x_pci_dev);

	return ret_val;
}
module_init(dm646x_pci_init);

/* =============================================================================
 *  @func   dm646x_pci_cleanup
 *
 *  @desc   This function will be called when this module is removed. It
 *	    will release all mapped memory regions and will release all
 *	    the PCI resources.
 *
 *  @modif  None.
 *  ============================================================================
 */
void __exit dm646x_pci_cleanup(void)
{
	/* ---------------------------------------------------------------------
	* Freeing the major number & other resources
	* ---------------------------------------------------------------------
	*/
	device_destroy(dm646x_pci_class, dm646x_dev_id);
	class_destroy(dm646x_pci_class);
	cdev_del(&dm646x_pci_cdev);
	unregister_chrdev_region(dm646x_dev_id, DM646X_DEV_COUNT);

	/* ---------------------------------------------------------------------
	* Unmap baseRegs region & release the reg region.
	* ---------------------------------------------------------------------
	*/
	iounmap(tcm_virt);
	release_mem_region(tcm_base, tcm_len);

	/* ---------------------------------------------------------------------
	* Unmap baseRegs region & release the reg region.
	* ---------------------------------------------------------------------
	*/
	iounmap(ddr2a_virt);
	iounmap(comm_virt);
	iounmap(linux_virt);
	release_mem_region(ddr2a_base, ddr2a_len);

	/* ---------------------------------------------------------------------
	* Unmap baseRegs region & release the reg region.
	* ---------------------------------------------------------------------
	*/
	iounmap(ddr2b_virt);
	release_mem_region(ddr2b_base, ddr2b_len);

	/* ---------------------------------------------------------------------
	* Unmap baseRegs region & release the reg region.
	* ---------------------------------------------------------------------
	*/
	iounmap(emif_virt);
	release_mem_region(emif_reg_base, emif_reg_len);

	/* ---------------------------------------------------------------------
	* Unmap baseRegs region & release the reg region.
	* ---------------------------------------------------------------------
	*/
	iounmap((void *)reg_virt);
	release_mem_region(reg_base, reg_len);

	pci_disable_device(dm646x_pci_dev);

	/* ---------------------------------------------------------------------
	*	 Release the PCI device
	* ---------------------------------------------------------------------
	*/
	pci_dev_put(dm646x_pci_dev);
}
module_exit(dm646x_pci_cleanup);
MODULE_LICENSE("GPL");

