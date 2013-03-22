/*
 *
 * Copyright (C) 2010 Bticino S.p.a
 * Author: Davide Bonfanti <davide.bonfanti@bticino.it>
 *
 * Contributors:
 *     Raffaele Recalcati <raffaele.recalcati@bticino.it>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/gpio.h>

#include "davinci-pcm-copy.h"
#include "cq93vc_copy.h"

#define DAVINCI_VOICECODEC_MODULE_BASE 0x01D0C000
#define VC_PID          0x00
#define VC_CTRL         0x04
#define VC_INTEN        0x08
#define VC_INTSTATUS    0x0C
#define VC_INTCLR       0x10
#define VC_EMUL_CTRL    0x14
#define RFIFO           0x20
#define WFIFO           0x24
#define FIFOSTAT        0x28
#define VC_REG00        0x80
#define VC_REG01        0x84
#define VC_REG02        0x88
#define VC_REG03        0x8C
#define VC_REG04        0x90
#define VC_REG05        0x94
#define VC_REG06        0x98
#define VC_REG09        0xA4
#define VC_REG10        0xA8
#define VC_REG12        0xB0

/* bit definitions */
#define VC_CTRL_WFIFOMD BIT(14)
#define VC_CTRL_WFIFOCL BIT(13)
#define VC_CTRL_WFIFOEN BIT(12)
#define VC_CTRL_RFIFOMD BIT(10)
#define VC_CTRL_RFIFOCL BIT(9)
#define VC_CTRL_RFIFOEN BIT(8)
#define VC_CTRL_WDUNSIGNED      BIT(7)
#define VC_CTRL_WDSIZE  BIT(6)
#define VC_CTRL_RDUNSIGNED      BIT(5)
#define VC_CTRL_RDSIZE  BIT(4)
#define VC_CTRL_RSTDAC  BIT(1)
#define VC_CTRL_RSTADC  BIT(0)

#define VC_INTSTATUS_WDREQ	BIT(3)

#define FIFOSTAT_WDATACOUNT_MASK	0x1F00
#define FIFOSTAT_WDATACOUNT_OFFSET	8

#define HW_FIFO_SIZE 0x10

void __iomem *voice_codec_base;

void cq93vc_enable(void)
{
	__raw_writel(0, voice_codec_base + VC_CTRL);
	__raw_writel(0, voice_codec_base + VC_INTEN);
	__raw_writel(VC_CTRL_RFIFOMD | VC_CTRL_WFIFOEN |
		     VC_CTRL_WFIFOMD, voice_codec_base + VC_CTRL);
}

void cq93vc_write(u16 data)
{
	__raw_writew(data, voice_codec_base + WFIFO);
}

void cq93vc_wait_fifo_ready(void)
{
	u32 diff;
	do {
		diff = __raw_readl(voice_codec_base + VC_INTSTATUS);
	} while (!(diff & VC_INTSTATUS_WDREQ));
}

int cq93vc_get_fifo_size(void)
{
	return HW_FIFO_SIZE;
}

int cq93vc_get_fifo_status(void)
{
	int fifo;
	fifo =	__raw_readl(voice_codec_base + FIFOSTAT);
	fifo = (fifo & FIFOSTAT_WDATACOUNT_MASK) >> FIFOSTAT_WDATACOUNT_OFFSET;
	return fifo;
}

void cq93vc_init(void)
{
	voice_codec_base = ioremap(DAVINCI_VOICECODEC_MODULE_BASE, SZ_4K);

}

int cq93vc_hw_params(struct snd_pcm_substream *substream,
		     struct snd_pcm_hw_params *hw_params)
{
	int w;
	w =  __raw_readl(voice_codec_base + VC_CTRL);
	__raw_writew(w | VC_CTRL_WFIFOMD | VC_CTRL_RFIFOMD,
		     voice_codec_base + VC_CTRL);
	return 0;
}

struct davinci_pcm_copy_ops cq93vc_pcm_copy_ops = {
	.enable = cq93vc_enable,
	.write = cq93vc_write,
	.wait_fifo_ready = cq93vc_wait_fifo_ready,
	.get_fifo_size = cq93vc_get_fifo_size,
	.get_fifo_status = cq93vc_get_fifo_status,
	.init = cq93vc_init,
	.hw_params = cq93vc_hw_params,
};
EXPORT_SYMBOL_GPL(cq93vc_pcm_copy_ops);
