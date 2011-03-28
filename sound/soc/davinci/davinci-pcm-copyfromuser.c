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

#include "davinci-pcm.h"
#include "davinci-pcm-copy.h"

#define DEF_BUF_SIZE	2048
#define DEF_MIN_INT	500000
#define DEF_INT_MARGIN	500000

int pointer_sub;
int hw_fifo_size;
u16 *local_buffer;
static struct hrtimer hrtimer;
struct snd_pcm_substream *substream_loc;
int ns_for_interrupt = 1500000;
int min_interrupt_ps;
int interrupt_margin;

struct davinci_pcm_copy_ops *ops;

static struct snd_pcm_hardware pcm_hardware_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000),
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
	.period_bytes_min = 512,
	.period_bytes_max = 512,
	.fifo_size = 0,
};

/*
 * How this driver works...
 *
 * This driver implements a pcm interface without the use of a DMA but with
 * a copy_from_user.
 * There's a buffer of {platform_data->buffer_size} bytes in the driver
 * that is filled with davinci_pcm_copy.
 * When pcm is running, a TIMER interrupt is activated  in order to fill
 * HW FIFO.
 * It happens that the peripheral stop working so there's a trap...
 */

static snd_pcm_uframes_t
davinci_pcm_pointer(struct snd_pcm_substream *substream)
{
	return pointer_sub;
}

static int davinci_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hardware *ppcm;
	static ktime_t wakeups_per_second;
	int ret = 0;

	pointer_sub = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ppcm = &pcm_hardware_playback;
	else
		return -ENODEV;

	snd_soc_set_runtime_hwparams(substream, ppcm);
	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	hrtimer_start(&hrtimer, wakeups_per_second, HRTIMER_MODE_REL);
	ops->enable();
	return 0;
}

static int davinci_pcm_close(struct snd_pcm_substream *substream)
{
	hrtimer_cancel(&hrtimer);
	return 0;
}

static int davinci_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params)
{
	long rate;

	rate = hw_params->rate_num * hw_params->rate_den;
	rate = hw_params->msbits * (1000000000 / rate);

	/* let's take some margin of */
	rate -= interrupt_margin;

	/* assure the interrupt doesn't occupy too many resources */
	ns_for_interrupt = rate > min_interrupt_ps ? rate : min_interrupt_ps;

	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int davinci_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static int davinci_pcm_copy(struct snd_pcm_substream *substream, int channel,
	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (copy_from_user(local_buffer + hwoff, buf,
	    frames_to_bytes(runtime, frames))) {
		printk(KERN_ERR "ERROR COPY_FROM_USER\n");
		return -EFAULT;
	}
	return 0;
}

static struct snd_pcm_ops davinci_pcm_ops = {
	.open =		davinci_pcm_open,
	.close =	davinci_pcm_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	davinci_pcm_hw_params,
	.hw_free =	davinci_pcm_hw_free,
	.pointer =	davinci_pcm_pointer,
	.copy =		davinci_pcm_copy,
};

static u64 davinci_pcm_dmamask = 0xffffffff;

static enum hrtimer_restart dm_pcm_irq(struct hrtimer *handle)
{
	int fifo, diff, per_size, buf_size;
	static int last_ptr;

	if (substream_loc->runtime && substream_loc->runtime->status &&
	    snd_pcm_running(substream_loc)) {
		fifo = ops->get_fifo_status();
		if (fifo >= (hw_fifo_size - 1))
			ops->enable();

		buf_size = substream_loc->runtime->buffer_size;
		per_size = substream_loc->runtime->period_size;
		for (; fifo < hw_fifo_size; fifo++) {
			ops->write(local_buffer[pointer_sub++]);
			pointer_sub %= buf_size;
			if (ops->wait_fifo_ready)
				ops->wait_fifo_ready();
		}
		if (last_ptr >= pointer_sub)
			diff = buf_size + pointer_sub - last_ptr;
		else
			diff = pointer_sub - last_ptr;
		if (diff >= per_size) {
			snd_pcm_period_elapsed(substream_loc);
			last_ptr += per_size;
			if (last_ptr >= buf_size)
				last_ptr -= buf_size;
		}
	} else
		last_ptr = 0;
	hrtimer_add_expires_ns(&hrtimer, ns_for_interrupt);
	return HRTIMER_RESTART;
}

static int davinci_pcm_new(struct snd_card *card,
			   struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	struct snd_dma_buffer *buf;
	struct snd_pcm_substream *substream;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &davinci_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		substream = pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream;
		buf = &substream->dma_buffer;
		buf->dev.type = SNDRV_DMA_TYPE_DEV;
		buf->dev.dev = pcm->card->dev;
		buf->private_data = NULL;
		buf->bytes = pcm_hardware_playback.buffer_bytes_max;
		substream_loc = substream;
	}
	hrtimer_init(&hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hrtimer.function = dm_pcm_irq;

	return 0;
}

int davinci_pcm_probe(struct platform_device *pdev)
{
	struct snd_soc_device *dev = platform_get_drvdata(pdev);
	struct davinci_pcm_copy_platform_data *data;

	data = dev->codec_data;
	ops = data->ops;

	if ((!ops) || (!ops->enable) || (!ops->write) ||
	    (!ops->get_fifo_size) || (!ops->get_fifo_status))
		return -EINVAL;
	min_interrupt_ps = data->min_interrupt_interval_ps ?
			   data->min_interrupt_interval_ps : DEF_MIN_INT;
	interrupt_margin = data->interrupt_margin_ps ?
			   data->interrupt_margin_ps : DEF_INT_MARGIN;
	pcm_hardware_playback.buffer_bytes_max = data->buffer_size ?
			   data->buffer_size : DEF_BUF_SIZE;
	pcm_hardware_playback.periods_min = data->buffer_size / 512;
	pcm_hardware_playback.periods_max = data->buffer_size / 512;
	local_buffer = kmalloc(data->buffer_size, GFP_KERNEL);
	if (ops->init)
		ops->init();
	hw_fifo_size = ops->get_fifo_size();

	return 0;
}

struct snd_soc_platform davinci_soc_platform_copy = {
	.name =		"davinci-audio-copy",
	.pcm_ops =	&davinci_pcm_ops,
	.pcm_new =	davinci_pcm_new,
	.probe =	davinci_pcm_probe,
};
EXPORT_SYMBOL_GPL(davinci_soc_platform_copy);

static int __init davinci_soc_copy_platform_init(void)
{
	return snd_soc_register_platform(&davinci_soc_platform_copy);
}
module_init(davinci_soc_copy_platform_init);

static void __exit davinci_soc_copy_platform_exit(void)
{
	snd_soc_unregister_platform(&davinci_soc_platform_copy);
}
module_exit(davinci_soc_copy_platform_exit);

MODULE_AUTHOR("Davide Bonfanti - bticino s.p.a.");
MODULE_DESCRIPTION("TI DAVINCI PCM copy_from_user module");
MODULE_LICENSE("GPL");
