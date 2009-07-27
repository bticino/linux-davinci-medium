/*
 * pci-generic.c
 *  Description:
 *  Generic part of PCI Host Driver for TI PCI Module. Note that though code in
 *  this file is supposed to be 'generic' it might still require some tweaking
 *  when porting to other boards/platform using same TI PCI Module
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

#include <linux/kernel.h>               /* pr_ wrappers */
#include <linux/pci.h>                  /* PCI data structures */
#include <asm/mach/pci.h>               /* hw_pci */
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/irq.h>

#include "pci.h"               		/* Configuration data */

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
#define PCIBARMSK(n)                    (0x110 + (4 * n))     /* 6 Registers */
#define PCISUBIDMIR                     0x12C
#define PCICPBPTRMIR                    0x134
#define PCILGINTMIR                     0x13C
#define PCISLVCNTL                      0x180
#define PCIBARTRL(n)                    (0x1C0 + (4 * n))     /* 6 Registers */
#define PCIBARMIR(n)                    (0x1E0 + (4 * n))     /* 6 Registers */
#define PCIMCFGDAT                      0x300
#define PCIMCFGADR                      0x304
#define PCIMCFGCMD                      0x308
#define PCIMSTCFG                       0x310
#define PCIADDSUB(n)                    (0x314 + (4 * n))     /* 32 Registers */
#define PCIVENDEVPRG                    0x394
#define PCICLREVPRG                     0x39C
#define PCISUBIDPRG                     0x3A0
#define PCIMAXLGPRG                     0x3A4
#define PCICFGDONE                      0x3AC

#define PCI_WAIT_FOR_RDY()              pci_wait_for_rdy()

#define CFG_PCIM_CMD_IO                 BIT(2)
#define CFG_PCIM_CMD_CONFIG             (0)
#define CFG_PCIM_CMD_READ               BIT(0)
#define CFG_PCIM_CMD_WRITE              (0)

/* Command Register Values */
#define PCIM_INTA_DIS        BIT(10)    /* Don't pass interrupt on INTA */
#define PCIM_SERR_EN         BIT(8)     /* Propagate error on SERR# */
#define PCIM_PERR_EN         BIT(6)     /* Consider parity error */
#define PCIM_MEMWRINV_EN     BIT(4)     /* Invalidate memory writes */
#define PCIM_BUS_MASTER      BIT(2)     /* Set as PCI master */
#define PCIM_MEMACC_EN       BIT(1)     /* Host as memory slave */
#define PCIM_IOACC_EN        BIT(0)     /* Host as IO device */
#define PCIM_SPLCYC_EN       BIT(3)     /* Could be used for sideband
					   signalling (non-critical) */
/* Default value used for CSMIR register */
#define CFG_PCIM_CSR_VAL         (PCIM_SERR_EN | PCIM_MEMWRINV_EN	\
					| PCIM_BUS_MASTER             	\
					| PCIM_MEMACC_EN              	\
					| PCIM_INTA_DIS)

#define PCIM_MAX_LAT_SHIFT        (24)
#define PCIM_MAX_LAT_MASK         (0xFF << PCIM_MAX_LAT_SHIFT)
#define PCIM_MAX_LAT_VAL          (CFG_PCIM_MAX_LAT << PCIM_MAX_LAT_SHIFT)

#define PCIM_MIN_GRANT_SHIFT      (16)
#define PCIM_MIN_GRANT_MASK       (0xFF << PCIM_MIN_GRANT_SHIFT)
#define PCIM_MIN_GRANT_VAL        (CFG_PCIM_MIN_GRANT \
					<< PCIM_MIN_GRANT_SHIFT)

/* Error mask for errors to check on CFG/IO */
#define CFG_PCI_ERR_MASK	  ((0xf << 28) | (1 < 24))

/* Protect PCI accesses (config/io read/writes) */
static DEFINE_SPINLOCK(ti_pci_lock);

static inline void dump_pci_status(u32 status)
{
	pr_debug("PCI: Error Status:\nData Parity Error: %d, "
		"System Error: %d\nMaster Abort: %d, "
		"Target Abort: %d\nParity Report: %d\n",
		((status & BIT(31)) != 0), ((status & BIT(30)) != 0),
		((status & BIT(29)) != 0), ((status & BIT(28)) != 0),
		((status & BIT(24)) != 0));
}

/* ti_pci_setup
 *  This function does Host controller configuration before bios starts PCI bus
 *  enumeration (scan).
 */
int ti_pci_setup(int nr, struct pci_sys_data *sys)
{
	int i, bar_en = 0;
	struct resource *res;

	pr_info("PCI: Setting up Host Controller...\n");
	pr_debug("PCI: PCIADDSUB[0] @%#x\n", (u32)PCI_REGV(PCIADDSUB(0)));

	if (nr != 0)
		return 0;

	/* Make specified Host windows visible over PCI bus. This will enable
	 * PCI Masters to perform DMA to Host System RAM (1:1 mapping of
	 * course!).
	 */
	for (i = 0; i < 6; i++) {
		u32 size = bar_cfg[i].bar_window_size;
		u32 addr = bar_cfg[i].bar_window_phys;

		if (size) {
			/* Ensure size in 16 B to 2 GB range */
			if (size < 16)
				size = 16;
			else if (size > 0x80000000)
				size = 0x80000000;

			if (size & (size - 1))
				size = BIT(fls(size));

			bar_en |= BIT(i);

			/* 1:1 from PCI -> Host */
			__raw_writel(addr, PCI_REGV(PCIBARTRL(i)));

			/* Setup PCI BAR Mirror register to bus address (= phys)
			 */
			__raw_writel(addr, PCI_REGV(PCIBARMIR(i)));
		}

		/* Set writable bits - masking for size */
		__raw_writel(~(size-1), PCI_REGV(PCIBARMSK(i)));
	}

	/* Enable applicable  BAR windows in Slave control register and Set
	 * CFG_DONE to enable PCI access
	 */
	__raw_writel(((bar_en << 16) | 1), PCI_REGV(PCISLVCNTL));

	/* IMPORTANT *** Change our class code from
	 * PCI_BASE_CLASS_SIGNAL_PROCESSING(PCI_CLASS_SP_OTHER = 0x1180) to
	 * PCI_CLASS_BRIDGE_HOST (0x0600) to prevent resource (BAR) assignment
	 * to us. The PCI probe will be happy with whatever original BAR
	 * settings we have!
	 * If at all we want to restore the default class-subclass values, the
	 * best place would be to set mirror register after returning from
	 * pci_common_init () [in dm6467_pci_init()].
	 */
	__raw_writel(((__raw_readl(PCI_REGV(PCICLREVMIR)) & 0xffff)
			| (PCI_CLASS_BRIDGE_HOST << 16)),
			PCI_REGV(PCICLREVMIR));

	/*
	 * Set PHYADDR <-> BUSADDR mapping for accessing 256MB PCI space
	 */

	/* Using Direct 1:1 mapping of DMSoC <-> PCI memory space */
	for (i = 0; i < CFG_PCIM_WINDOW_CNT; i++)
		__raw_writel((CFG_PCIM_MEM_START + (CFG_PCIM_WINDOW_SZ*i)),
				PCI_REGV(PCIADDSUB(i)));


	/* Setup as PCI master */
	pr_debug("PCI: Default PCICSRMIR = %x\n",
			__raw_readl(PCI_REGV(PCICSRMIR)));

	__raw_writel((__raw_readl(PCI_REGV(PCICSRMIR)) | CFG_PCIM_CSR_VAL),
			PCI_REGV(PCICSRMIR));

	pr_debug("PCI: New PCICSRMIR = %x\n",
			__raw_readl(PCI_REGV(PCICSRMIR)));

	__raw_writel(((__raw_readl(PCI_REGV(PCILGINTMIR))
				& ~PCIM_MAX_LAT_MASK & ~PCIM_MIN_GRANT_MASK)
				| PCIM_MAX_LAT_VAL | PCIM_MIN_GRANT_VAL),
			PCI_REGV(PCILGINTMIR));

	/*
	 * Setup resources which will be used assign BARs to targets during
	 * scanning
	 */

	/* Overwrite the resources which were set up by default by
	 * pcibios_init_hw
	 */
	res = kzalloc(sizeof(*res) * 2, GFP_KERNEL);
	if (res == NULL) {
		panic("PCI: resource structure allocation failed.\n");
		/* Not reached */
		return 0;
	}

	res[0].start = PCIBIOS_MIN_IO;
	res[0].end = PCIBIOS_MAX_IO;
	res[0].name = "PCI I/O";
	res[0].flags = IORESOURCE_IO;

	res[1].start = CFG_PCIM_MEM_START;
	res[1].end = CFG_PCIM_MEM_END;
	res[1].name = "PCI Memory";
	res[1].flags = IORESOURCE_MEM;

	request_resource(&ioport_resource, &res[0]);
	request_resource(&iomem_resource, &res[1]);

	sys->resource[0] = &res[0];
	sys->resource[1] = &res[1];
	sys->resource[2] = NULL;

	return 1;
}

static inline u32 get_config_addr(u8 bus, u16 devfn, int where)
{
	u32 addr;

	/* Determine configuration cycle type depending upon the bus:
	 * - TYPE 0 for PCI bus directly connected to host
	 * - TYPE 1 for across bridge(s)
	 */
	if (bus == 0) {
		/* TYPE 0 */
		addr = BIT(11 + PCI_SLOT(devfn)) | (PCI_FUNC(devfn) << 8)
			| (where & ~3);
	} else {
		/* TYPE 1 */
		addr = (bus << 16) | (PCI_SLOT(devfn) << 11)
			| (PCI_FUNC(devfn) << 8) | (where & ~3) | 1;
	}

	return addr;
}

static inline int get_byte_enables(u32 addr, u32 size)
{
	/*
	 * Byte Enables (BE#[3:0]) will be generated taking into account AD[1:0]
	 * and size of transaction (1-4 Byte). Refer following table:
	 *  PCI_ADDR[1:0]       Starting Byte           BE#[3:0]
	 * -----------------------------------------------------------------
	 *  00                  Byte 0                  xxx0 (size = 1->4)
	 *  01                  Byte 1                  xx01 (size = 1->3)
	 *  10                  Byte 2                  x011 (size = 1->2)
	 *  11                  Byte 3                  0111 (size = 1)
	 *
	 *  (Here 'x' can either be '0' (byte enabled) or '1' as required by
	 *  'size')
	 *
	 *  As mentioned above, BE values are function of 'size' and AD[1:0] as
	 *  well as size so we start by doing sanity check on combination of
	 *  supplied parameters. We won't return any error as such, but any
	 *  wrong combination will result into either all BE de-asserted or
	 *  only/all possible BEs asserted. E.g.,
	 *  size=0 -> All BEs de-asserted
	 *  size>4 -> Only possible BEs de-asserted
	 */

	addr &= 3;

	pr_debug("PCI: AD[1:0]:size = %01x:%01x, Byte enables = %#lx\n",
			addr&3, size, (((BIT(size) - 1) << addr) & 0xf));

	/* BE values are inverted for BE# */
	return ((BIT(size) - 1) << addr) & 0xf;
}

/*
 * Returns:
 *  0   - When READY bit is not set within timeout, else non-zero.
 */
static inline int pci_wait_for_rdy(void)
{
	int wait_cnt = CFG_PCI_READY_WAITCNT;

	/* Note : This function doesn't check aborts since the READY bit it set
	 * when bus aborts happen. Need to verify this is true for all kinds of
	 * PCI access errors.
	 */
	while (wait_cnt && !(__raw_readl(PCI_REGV(PCIMCFGCMD)) & BIT(31)))
		wait_cnt--;

	return (wait_cnt != 0);
}

static inline int pci_err(void)
{
	u32 status = __raw_readl(PCI_REGV(PCICSRMIR));
	if (status & CFG_PCI_ERR_MASK) {
		dump_pci_status(status);
		return status >> 24;
	} else {
		return 0;
	}
}

static inline int pci_cfgio_read(u32 addr, int size, u32 *value,
					int be, int access_type)
{
	unsigned long flags;
	int ret_val = 0;

	spin_lock_irqsave(&ti_pci_lock, flags);

	/* Check of READY. Continue even if not READY as we will catch the error
	 * after operation. Same logic is applied for 'write' operation below...
	 */
	if (!PCI_WAIT_FOR_RDY())
		pr_warning("PCI: CFG/IO not ready."
			"Might need tuning CFG_PCI_READY_WAITCNT value.\n");

	/* Clear aborts */
	__raw_writel((__raw_readl(PCI_REGV(PCICSRMIR)) | (0xFF << 24)),
			PCI_REGV(PCICSRMIR));

	__raw_writel(addr, PCI_REGV(PCIMCFGADR));

	__raw_writel((BIT(31) | ((access_type == PCI_ACCESS_TYPE_CFG)
				? CFG_PCIM_CMD_CONFIG : CFG_PCIM_CMD_IO)
			| CFG_PCIM_CMD_READ
			| (be << 4)), PCI_REGV(PCIMCFGCMD));

	/* Check for READY. */
	ret_val = !PCI_WAIT_FOR_RDY();

	if (pci_err()) {
		pr_debug("PCI: CFG/IO Read @%#x Failed.\n", addr);
		*value = ~0;
		/* Clear aborts */
		__raw_writel((__raw_readl(PCI_REGV(PCICSRMIR)) | (0xFF << 24)),
				PCI_REGV(PCICSRMIR));
		ret_val = -1;
	} else {
		*value = __raw_readl(PCI_REGV(PCIMCFGDAT));
		pr_debug("PCI: Config/IO read done, value = %x\n", *value);
	}

	spin_unlock_irqrestore(&ti_pci_lock, flags);
	return ret_val;
}

/*
 * This is just a wrapper (exported) to provide BE value for I/O
 */
int ti_pci_io_read(u32 addr, int size, u32 *value)
{
	int ret_val = pci_cfgio_read(addr, size, value,
					get_byte_enables(addr, size),
					PCI_ACCESS_TYPE_IO);
	*value >>= ((addr & 3)*8);
	pr_debug("PCI: IO read @%#x = %#x\n", addr, *value);
	return ret_val;
}
EXPORT_SYMBOL(ti_pci_io_read);

static int ti_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *value)
{
	u8 bus_num = bus->number;

	/* Sanity checks */
	if ((bus_num == 0) && (PCI_SLOT(devfn) >= PCI_MAX_SLOTS))
		return PCIBIOS_DEVICE_NOT_FOUND;

	pr_debug("PCI: Reading config[%x] for device %04x:%02x:%02x ...",
			where, bus_num, PCI_SLOT(devfn), PCI_FUNC(devfn));

	if (pci_cfgio_read(get_config_addr(bus_num, devfn, where), size,
				value, get_byte_enables(where, size),
				PCI_ACCESS_TYPE_CFG)) {
		/* Ignoring other error codes (> 0) */
		pr_debug("failed.");
		return PCIBIOS_DEVICE_NOT_FOUND;
	} else {
		*value >>= ((where & 3)*8);
		pr_debug("done, value = %x\n", *value);
		return PCIBIOS_SUCCESSFUL;
	}
}

static int pci_cfgio_write(u32 addr, int size, u32 value,
				int be, int access_type)
{
	unsigned long flags;
	int ret_val = 0;

	spin_lock_irqsave(&ti_pci_lock, flags);

	/* Check of READY */
	if (!PCI_WAIT_FOR_RDY())
		pr_warning("PCI: CFG/IO not ready."
			"Might need tuning CFG_PCI_READY_WAITCNT value.\n");

	/* Clear aborts */
	__raw_writel((__raw_readl(PCI_REGV(PCICSRMIR)) | (0xFF << 24)),
			PCI_REGV(PCICSRMIR));

	__raw_writel((value << ((addr & 3)*8)), PCI_REGV(PCIMCFGDAT));
	__raw_writel(addr, PCI_REGV(PCIMCFGADR));

	__raw_writel((0x80000000 | ((access_type == PCI_ACCESS_TYPE_CFG)
					? CFG_PCIM_CMD_CONFIG : CFG_PCIM_CMD_IO)
			| CFG_PCIM_CMD_WRITE | (be << 4)),
			PCI_REGV(PCIMCFGCMD));

	/* Check for READY. */
	ret_val = !PCI_WAIT_FOR_RDY();

	if (pci_err()) {
		pr_debug("PCI: CFG/IO Write @%#x Failed\n", addr);
		/* Clear aborts */
		__raw_writel((__raw_readl(PCI_REGV(PCICSRMIR)) | (0xFF << 24)),
				PCI_REGV(PCICSRMIR));
		ret_val = -1;
	} else {
		pr_debug("PCI: Config/IO write done.\n");
	}

	spin_unlock_irqrestore(&ti_pci_lock, flags);
	return ret_val;
}

/*
 * This is just a wrapper (exported) to provide BE value for I/O
 */
int ti_pci_io_write(u32 addr, int size, u32 value)
{
	pr_debug("PCI: IO write @%#x = %#x\n", addr, value);
	return pci_cfgio_write(addr, size, value,
				get_byte_enables(addr, size),
				PCI_ACCESS_TYPE_IO);
}
EXPORT_SYMBOL(ti_pci_io_write);

static int ti_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 value)
{
	u8 bus_num = bus->number;

	/* Sanity checks */
	if ((bus_num == 0) && (PCI_SLOT(devfn) >= PCI_MAX_SLOTS))
		return PCIBIOS_DEVICE_NOT_FOUND;

	pr_debug("PCI: Writing config[%x] = %x "
			"for device %04x:%02x:%02x ...", where, value,
			bus_num, PCI_SLOT(devfn), PCI_FUNC(devfn));

	if (pci_cfgio_write(get_config_addr(bus_num, devfn, where),
				size, value, get_byte_enables(where, size),
				PCI_ACCESS_TYPE_CFG)) {
		/* Ignoring other error codes (> 0) */
		pr_debug("failed.\n");
		return PCIBIOS_DEVICE_NOT_FOUND;
	} else {
		pr_debug("done.\n");
		return PCIBIOS_SUCCESSFUL;
	}
}

static struct pci_ops ti_pci_ops = {
	.read	= ti_pci_read_config,
	.write	= ti_pci_write_config,
};

struct pci_bus *ti_pci_scan(int nr, struct pci_sys_data *sys)
{
	pr_info("PCI: Starting PCI scan...\n");
	if (nr == 0)
		return pci_scan_bus(0, &ti_pci_ops, sys);

	return NULL;
}
