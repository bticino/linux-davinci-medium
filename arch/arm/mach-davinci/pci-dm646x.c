/*
 * pci-dm646x.c
 *  Description:
 *  DM6467 (and in some parts, EVM) specific PCI initialization.
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

#undef DEBUG

#include <linux/kernel.h>       /* pr_ wrappers */
#include <linux/pci.h>		/* PCI data structures */
#include <asm/mach/pci.h>	/* hw_pci */
#include <linux/types.h>

#include "pci.h"		/* Platform spec */

#include <mach/cputype.h>
#include <mach/dm646x.h>	/* SoC reg  defs, __REG, IO info etc */
#include <linux/clk.h>		/* For clk_enable interface */

/* NOTE:
 * Most of the code in this file assumes that it is runnig as PCI Host and
 * configures the system accordingly. The only place where it is checked if we
 * are booted through PCI Boot, is the function dm646x_pci_init (below) before
 * it registers with the bios. Thus, if we were not hosting the PCI bus the
 * other functions wouldn;t get called anyway as the pci bios will not be
 * invoked.
 */

/* Forward declarations */
static void __init dm646x_pci_preinit(void);
static int __init dm646x_pci_map_irq(struct pci_dev *dev, u8 slot, u8 pin);

/* Specify the physical address and size information for Host windows to be
 * configured in BAR0-5 registers. Note that _ALL_ of the 6 entries must be
 * filled. For entries with no real window to be mapped, set bar_window_size=0.
 */
struct pcihost_bar_cfg bar_cfg[6] = {
	{ PHYS_OFFSET /* bar_window_phys */ , 0x8000000 /* bar_window_size */},
	{ 0                                 , 0                              },
	{ 0                                 , 0                              },
	{ 0                                 , 0                              },
	{ 0                                 , 0                              },
	{ 0                                 , 0                              }
};

/* Plug into Linux PCI BIOS Interface */
static struct hw_pci davinci_pci __initdata = {
	.nr_controllers     = 1,
	.setup              = ti_pci_setup,
	.scan               = ti_pci_scan,
	.preinit            = dm646x_pci_preinit,
	.postinit           = NULL,
	.swizzle            = pci_std_swizzle,       /* Using default Linux
							ARM-BIOS algorithm */
	.map_irq            = dm646x_pci_map_irq
};

/* dm646x_pci_preinit
 *  This function handles initializations to do before accessing PCI bus. This
 *  function is optional and most of the things done here could be handled in
 *  board/SoC level init.
 */
void dm646x_pci_preinit(void)
{
	/*
	 * Use clk_enable interface to do the PSC init as well as take care of
	 * pin muxing for PCI  module domain
	 */
	struct clk *pci_clk;

	pr_info("PCI: Enabling Clock...\n");
	pci_clk = clk_get(NULL, "pci");
	if (IS_ERR(pci_clk))
		return;

	clk_enable(pci_clk);

	/* At tis point, we assume here that the board level code has taken care
	 * of driving RST# over PCI Bus (or, if not, then already taken care in
	 * H/W - this is default behavior of the EVM)
	 */
}

int dm646x_pci_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	/*
	 * Standard EVM doesn't have any dedicated lines connected to PCI
	 * Interrupts.
	 */

	return -1;
}

/* dm646x_pci_init
 *  PCI system initialization. Fills in required information for PCI BIOS to
 *  perrform enumeratios and invokes pci_common_init.
 */
static int __init dm646x_pci_init(void)
{
	if (cpu_is_davinci_dm646x()) {
		/* Since DM646x can act either as PCI host or target, we skip
		 * setting up PCI BIOS for enumeration if we are 'target'.
		 * This is determined by checking BOOTCFG BOOTMODE[0:3] and
		 * PCIEN values.
		 * Note : BOOTCFG values are latched across soft resets and thus
		 * the check below cannot detect any change in actual boot mode.
		 */
		u32 bootcfg = __raw_readl(
				(IO_ADDRESS(DAVINCI_SYSTEM_MODULE_BASE)
				 + BOOTCFG));
		u32 bootmode = bootcfg & 0xf;

		pr_info("PCI: bootcfg = %#x, bootmode = %#x\n",
			bootcfg, bootmode);

		if (!((bootcfg & BIT(16))
			&& ((bootmode == 2) || (bootmode == 3)))) {
			pr_info("PCI: Invoking PCI BIOS...\n");
			pci_common_init(&davinci_pci);
		} else {
			pr_info("PCI: Skipping PCI Host setup...\n");
		}
	}

	return 0;
}

subsys_initcall(dm646x_pci_init);
