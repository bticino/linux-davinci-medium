/*
 * pci.h
 *  Description:
 *  PCI module specific types/configurations and definitions.
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
#ifndef __ARCH_PCI_H
#define __ARCH_PCI_H

#include <mach/pci.h>

/*
 * Configuration options
 */

/* Could be updated with optimum PCILGINTMIR values */
#define CFG_PCIM_MAX_LAT		0xF
#define CFG_PCIM_MIN_GRANT		0xF

#define CFG_PCI_INTSET_MASK		((0x3 << 1) | (0x3 << 5))
#define CFG_PCI_INTCLR_MASK		(CFG_PCI_INTSET_MASK)

#define PCI_MAX_SLOTS			(21)        /* Only checked for bus=0 as
							IDSEL can be provided
							only within AD[31:11] */

#define CFG_PCIM_WINDOW_SZ	(0x00800000)        /* Master window size */
#define CFG_PCIM_WINDOW_CNT	(32)                /* Number of windows */
#define CFG_PCIM_MEM_START	(PCIBIOS_MIN_MEM)   /* PCI master memory map
							base NOTE: We are using
							1:1 mapping, i.e.,
							0x30000000 from DMSoC
							side is translated as
							0x30000000 on PCI Bus */
#define CFG_PCIM_MEM_END	(CFG_PCIM_MEM_START                    \
					+ (CFG_PCIM_WINDOW_SZ          \
						* CFG_PCIM_WINDOW_CNT) \
					- 1)	/* PCI master memory map size */


/* The default value below can be substituted with proper timeout value which
 * taking into account various PCI system parameters (buses, devices/slots,
 * average latency to gain PCI bus access etc) to have sufficiant time till the
 * master completes previous transaction.
 */
#define CFG_PCI_READY_WAITCNT	0xffff

/*
 * Provide the virtual address of PCI Backend register at specified offset. Uses
 * SoC specific PCI register base for address calculation.
 */
#define PCI_REGV(reg)           (IO_ADDRESS(PCICTL_REG_BASE) + reg)

struct pcihost_bar_cfg {
    u32 bar_window_phys;
    u32 bar_window_size;
};

/* Forward declarations */
extern int __init ti_pci_setup(int nr, struct pci_sys_data *);
extern struct  pci_bus * __init ti_pci_scan(int nr, struct pci_sys_data *);
extern struct pcihost_bar_cfg bar_cfg[6];

#endif /* !__ARCH_PCI_H */
