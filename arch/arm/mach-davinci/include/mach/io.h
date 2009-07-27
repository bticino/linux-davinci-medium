/*
 * DaVinci IO address definitions
 *
 * Copied from include/asm/arm/arch-omap/io.h
 *
 * 2007 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT 0xffffffff

/*
 * We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */
#if !defined CONFIG_PCI || defined __ASSEMBLER__
#define __io(a)			__typesafe_io(a)
#else
#include <mach/pci.h>
/*
 * Here we provide DMSoC to PCI I/O space access as PCI I/O is performed
 * indirectly by accessing PCI IO Register.
 *
 * We will still use the default I/O (memory mapped for ARM)  for any access
 * which is outside the allotted PCI I/O window. Note: In this case we replicate
 * the default inx/outx behavior as found in include/asm-arm/io.h (except of
 * course, addition of sequence point, which is not needed in our case).
 */
#define CFG_PCIM_IO_START               (PCIBIOS_MIN_IO)
#define CFG_PCIM_IO_END                 0x4BFFFFFF

#define	outb(v, p)			_davinci_outb(v, p)
#define	outw(v, p)			_davinci_outw(v, p)
#define	outl(v, p)			_davinci_outl(v, p)

#define	outsb(p, d, l)			_davinci_outsb(p, d, l)
#define	outsw(p, d, l)			_davinci_outsw(p, d, l)
#define	outsl(p, d, l)			_davinci_outsl(p, d, l)

#define	inb(p)				_davinci_inb(p)
#define	inw(p)				_davinci_inw(p)
#define	inl(p)				_davinci_inl(p)

#define	insb(p, d, l)			_davinci_insb(p, d, l)
#define	insw(p, d, l)			_davinci_insw(p, d, l)
#define	insl(p, d, l)			_davinci_insl(p, d, l)

#define IS_PCI_IO(p)			(((p) >= PCIBIOS_MIN_IO) \
						&& ((p) <= PCIBIOS_MAX_IO))

/*
 * I/O Functions
 */
#define PCI_ACCESS_TYPE_CFG             0
#define PCI_ACCESS_TYPE_IO              1
extern int ti_pci_io_write(u32 addr, int size, u32 value);
extern int ti_pci_io_read(u32 addr, int size, u32 *value);

static inline void _davinci_outb(u8 value, u32 addr)
{
	if (IS_PCI_IO(addr))
		ti_pci_io_write(addr, 1, (u32)value);
	else
		__raw_writeb(value, __typesafe_io(addr));
}

static inline void _davinci_outsb(u32 addr, const void *data, u32 blen)
{
	while (blen--)
		_davinci_outb(*(const u8 *)data++, (u32)((u8 *)addr++));
}

static inline void _davinci_outw(u16 value, u32 addr)
{
	if (IS_PCI_IO(addr))
		ti_pci_io_write(addr, 2, (u32)value);
	else
		__raw_writew(cpu_to_le16(value), __typesafe_io(addr));
}

static inline void _davinci_outsw(u32 addr, const void *data, u32 wlen)
{
	while (wlen--)
		_davinci_outw(*(const u16 *)data++, (u32)((u16 *)addr++));
}

static inline void _davinci_outl(u32 value, u32 addr)
{
	if (IS_PCI_IO(addr))
		ti_pci_io_write(addr, 4, (u32)value);
	else
		__raw_writel(cpu_to_le32(value), __typesafe_io(addr));
}

static inline void _davinci_outsl(u32 addr, const void *data, u32 llen)
{
	while (llen--)
		_davinci_outl(*(const u32 *)data++, (u32)((u32 *)addr++));
}

static inline u8 _davinci_inb(u32 addr)
{
	if (IS_PCI_IO(addr)) {
		u32 value;
		ti_pci_io_read(addr, 1, &value);
		return (u8)value;
	} else {
		return __raw_readb(__typesafe_io(addr));
	}
}

static inline void _davinci_insb(u32 addr, void *data, u32 blen)
{
	while (blen--)
		*(u8 *)data++ = _davinci_inb((u32)((u8 *)addr++));
}

static inline u16 _davinci_inw(u32 addr)
{
	if (IS_PCI_IO(addr)) {
		u32 value;
		ti_pci_io_read(addr, 2, &value);
		return (u16)value;
	} else {
		return le16_to_cpu(__raw_readw(__typesafe_io(addr)));
	}
}

static inline void _davinci_insw(u32 addr, void *data, u32 wlen)
{
	while (wlen--)
		*(u16 *)data++ = _davinci_inw((u32)((u16 *)addr++));
}

static inline u32 _davinci_inl(u32 addr)
{
	if (IS_PCI_IO(addr)) {
		u32 value;
		ti_pci_io_read(addr, 4, &value);
		return value;
	} else {
		return le32_to_cpu(__raw_readl(__typesafe_io(addr)));
	}
}

static inline void _davinci_insl(u32 addr, void *data, u32 llen)
{
	while (llen--)
		*(u32 *)data++ = _davinci_inl((u32)((u32 *)addr++));
}

static inline unsigned int _davinci_ioread8(void __iomem *port)
{
	return (unsigned int)_davinci_inb((unsigned long __force)port);
}

static inline void _davinci_ioread8_rep(void __iomem *port, void *dst,
						unsigned long count)
{
	_davinci_insb((unsigned long __force)port, dst, count);
}

static inline unsigned int _davinci_ioread16(void __iomem *port)
{
	return (unsigned int)_davinci_inw((unsigned long __force)port);
}

static inline void _davinci_ioread16_rep(void __iomem *port, void *dst,
						unsigned long count)
{
	_davinci_insw((unsigned long __force)port, dst, count);
}

static inline unsigned int _davinci_ioread32(void __iomem *port)
{
	return (unsigned int)_davinci_inl((unsigned long __force)port);
}

static inline void _davinci_ioread32_rep(void __iomem *port, void *dst,
						unsigned long count)
{
	_davinci_insl((unsigned long __force)port, dst, count);
}

static inline void _davinci_iowrite8(u8 value, void __iomem *port)
{
	_davinci_outb(value, (unsigned long __force)port);
}

static inline void _davinci_iowrite8_rep(void __iomem *port, const void *src,
					unsigned long count)
{
	_davinci_outsb((unsigned long __force)port, src, count);
}

static inline void _davinci_iowrite16(u16 value, void __iomem *port)
{
	_davinci_outw(value, (unsigned long __force)port);
}

static inline void _davinci_iowrite16_rep(void __iomem *port, const void *src,
					unsigned long count)
{
	_davinci_outsw((unsigned long __force)port, src, count);
}

static inline void _davinci_iowrite32(u32 value, void __iomem *port)
{
	_davinci_outl(value, (unsigned long __force)port);
}

static inline void _davinci_iowrite32_rep(void __iomem *port, const void *src,
					unsigned long count)
{
	_davinci_outsl((unsigned long __force)port, src, count);
}

#define ioread8(p)			_davinci_ioread8(p)
#define ioread16(p)			_davinci_ioread16(p)
#define ioread32(p)			_davinci_ioread32(p)

#define iowrite8(v, p)			_davinci_iowrite8(v, p)
#define iowrite16(v, p)			_davinci_iowrite16(v, p)
#define iowrite32(v, p)			_davinci_iowrite32(v, p)

#define ioread8_rep(p, d, c)		_davinci_ioread8_rep(p, d, c)
#define ioread16_rep(p, d, c)		_davinci_ioread8_rep(p, d, c)
#define ioread32_rep(p, d, c) 		_davinci_ioread8_rep(p, d, c)

#define iowrite8_rep(p, s, c)		_davinci_iowrite8_rep(p, s, c)
#define iowrite16_rep(p, s, c)		_davinci_iowrite16_rep(p, s, c)
#define iowrite32_rep(p, s, c)		_davinci_iowrite32_rep(p, s, c)

#define	ioport_map(port, nr)		((void __iomem *)port)
#define	ioport_unmap(addr)

#endif /* CONFIG_PCI */

#define __mem_pci(a)		(a)
#define __mem_isa(a)		(a)

#ifndef __ASSEMBLER__
#define __arch_ioremap(p, s, t)	davinci_ioremap(p, s, t)
#define __arch_iounmap(v)	davinci_iounmap(v)

void __iomem *davinci_ioremap(unsigned long phys, size_t size,
			      unsigned int type);
void davinci_iounmap(volatile void __iomem *addr);
#endif
#endif /* __ASM_ARCH_IO_H */
