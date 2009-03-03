/*
 * dm646x_pci_targetdrv.h
 *  Description:
 *  DM6467 PCI Target driver header file.
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


#ifndef __PCIDRV_H__
#define __PCIDRV_H__

#include <linux/types.h>

/** ============================================================================
 *  @const  DM646X_PCI_MODFILE
 *
 *  @desc   Name of the device node created
 *  ============================================================================
 */
#define DM646X_PCI_MODFILE	    ("dm646xpci")

#ifdef __KERNEL__

/** ============================================================================
 *  @const  PCI_BASE_ADDRESS
 *
 *  @desc   Base address of PCI memory space.
 *  ============================================================================
 */
#define CHIP_DM6467

#define PCI_MEMORY_SPACE_BASE_ADDR  (0x30000000)

/** ============================================================================
 *  @const  PCI_TI_VENDOR
 *
 *  @desc   TI Vendor ID.
 *  ============================================================================
 */
#define PCI_TI_VENDOR               (0x104c)
/** ============================================================================
 *  @const  PCI_TI_DEVICE
 *
 *  @desc   PCI Device ID.
 *  ============================================================================
 */
#define PCI_TI_DEVICE               (0xB002)

/** ============================================================================
 *  @const   DM6467PCI_MMRREG_BASE
 *
 *  @desc    Base address of all register bank
 *  ============================================================================
 */
#define DM6467PCI_MMRREG_BASE       (0x01C00000)

/** ============================================================================
 *  @const   DM6467PCI_PCIREG_BASE
 *
 *  @desc    Base address of PCI backend registers.
 *  ============================================================================
 */
#define DM6467PCI_PCIREG_BASE		(0x01C1A000)

/** ============================================================================
 *  @const   DM6467PCI_DEVREG_BASE
 *
 *  @desc    Base address of Device config registers.
 *  ============================================================================
 */
#define DM6467PCI_DEVREG_BASE		(0x01C40000)

/** ============================================================================
 *  @const   DM6467PCI_SOFTINT0_MASK
 *
 *  @desc    Mask for generating soft int0 (DSP->GPP)
 *  ============================================================================
 */
#define DM6467PCI_SOFTINT0_MASK		(0x01000000)

/** ============================================================================
 *  @const   DM6467PCI_SOFTINT1_MASK
 *
 *  @desc    Mask for generating soft int1 (GPP->DSP)
 *  ============================================================================
 */
#define DM6467PCI_SOFTINT1_MASK		(0x02000000)

/** ============================================================================
 *  @const   DM6467PCI_SOFTINT1_MASK
 *
 *  @desc    Mask for generating soft int1 (GPP->DSP)
 *  ============================================================================
 */
#define DM6467PCI_SOFTINT2_MASK		(0x04000000)

/** ============================================================================
 *  @const   DM6467PCI_SOFTINT1_MASK
 *
 *  @desc    Mask for generating soft int1 (GPP->DSP)
 *  ============================================================================
 */
#define DM6467PCI_SOFTINT3_MASK		(0x08000000)

/** ============================================================================
 *  @const   DM6467PCI_INTSTATUS_MASK
 *
 *  @desc    Bitmask for Interrupt status (DSP->GPP)
 *  ============================================================================
 */
#define DM6467PCI_INTSTATUS_MASK	(0x00080000)

/** ============================================================================
 *  @const   DM6467PCI_PCIADLEN
 *
 *  @desc    Length each segment of addressable PCI Space..
 *  ============================================================================
 */
#define DM6467PCI_PCIADLEN          (0x00800000)

/** ============================================================================
 *  @const   DM6467PCI_PCIADWRBITMASK
 *
 *  @desc    Mask indicating writeable bits in PCI Address Window registers.
 *  ============================================================================
 */
#define DM6467PCI_PCIADWRBITMASK    (0xFF800000)

/** ============================================================================
 *  @const   DM6467PCI_PAGEWRBITMASK
 *
 *  @desc    Mask indicating writeable bits in PCI Base Address Mask Register5.
 *  ============================================================================
 */
#define DM6467PCI_PAGEWRBITMASK     (0xFF800000)


/*
 *  Device Register Offsets
 */
#define BOOTSTAT			0x10

/*
 *  PCI Register Offsets
 */
#define PCISTATSET                      0x010
#define PCISTATCLR                      0x014
#define PCIHINTSET                      0x020
#define PCIHINTCLR                      0x024
#define PCIBINTSET                      0x030
#define PCIBINTCLR                      0x034
#define PCIVENDEVMIR                    0x100
#define PCICSRMIR                       0x104
#define PCICLREVMIR                     0x108
#define PCICLINEMIR                     0x10C
#define PCIBARMSK(n)                    (0x110 + (4*n))     /* 6 Registers */
#define PCISUBIDMIR                     0x12C
#define PCICPBPTRMIR                    0x134
#define PCILGINTMIR                     0x13C
#define PCISLVCNTL                      0x180
#define PCIBARTRL(n)                    (0x1C0 + (4*n))     /* 6 Registers */
#define PCIBARMIR(n)                    (0x1E0 + (4*n))     /* 6 Registers */
#define PCIMCFGDAT                      0x300
#define PCIMCFGADR                      0x304
#define PCIMCFGCMD                      0x308
#define PCIMSTCFG                       0x310
#define PCIADDSUB(n)                    (0x314 + (4*n))     /* 32 Registers */
#define PCIVENDEVPRG                    0x394
#define PCICLREVPRG                     0x39C
#define PCISUBIDPRG                     0x3A0
#define PCIMAXLGPRG                     0x3A4
#define PCICFGDONE                      0x3AC

#endif  /*  __KERNEL__  */

/* Arguments received by the remap BAR IOCTL */
struct DM646x_Remap_Bar_Args {
	u32 bar_number;
	u32 new_window;
};

/* IOCTLs defined for the application as well as driver */
#define DM646X_PCI_RUN_TCM_IMG  _IO('P', 1)
#define DM646X_PCI_GEN_INTRPT   _IO('P', 2)
#define DM646X_PCI_EDMA_XFER    _IOW('P', 3, unsigned int)
#define DM646X_PCI_REMAP_BAR    _IOW('P', 4, unsigned int)
#define DM646X_PCI_BOOT_IMAGE   _IOW('P', 5, unsigned int)

#endif	/* __PCIDRV_H__ */


