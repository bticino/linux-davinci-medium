/*
 * pci.h
 *  Description:
 *  Resource data for PCI host. Currently only has DM6467 specific information
 *  since other DaVinci family SoCs do not have PCI Host support.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __MACH_PCI_H
#define __MACH_PCI_H

/* Enumeration flag: Checked during PCI Enumeration of Bridges */
#define pcibios_assign_all_busses()     1

/*
 * PCI Resource allocation
 */

/* PCI IO window.
 * This could be set anywhere in the 4G space by adjusting PCIBIOS_MIN_IO and
 * PCIBIOS_MAX_IO, which, in turn are used by in/out macros to distinguish
 * between PCI IO and normal MMIO.
 */

/* Using 32M reserved window from DM6467 memory map as PCI IO region */
#define PCIBIOS_MIN_IO          	(0x4A000000)
#define PCIBIOS_MAX_IO          	(0x4BFFFFFF)

/* PCI Memory window base */
#define PCIBIOS_MIN_MEM         	(0x30000000)

/* PCI Control register base (backend) */
#define PCICTL_REG_BASE			(0x01C1A000)

#endif /* !__MACH_PCI_H */
