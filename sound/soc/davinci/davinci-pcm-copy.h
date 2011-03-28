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

#ifndef _DAVINCI_PCM_COPY_H
#define _DAVINCI_PCM_COPY_H

struct davinci_pcm_copy_ops {
	/* called to perform initial settings - optional */
	void (*init)(void);
	/* called to enable codec - mandatory */
	void (*enable)(void);
	/* called to push one data into hw_fifo - mandatory */
	void (*write)(u16);
	/* called to wait hw_fifo is ready for more data - optional */
	void (*wait_fifo_ready)(void);
	/* called to get hw_fifo size - mandatory */
	int (*get_fifo_size)(void);
	/* called to get number of samples already into the hw_fifo */
	int (*get_fifo_status)(void);
};

struct davinci_pcm_copy_platform_data {
	/* length required for samples buffer in bytes */
	int buffer_size;
	/* minimum time in ps between an interrupt and another */
	int min_interrupt_interval_ps;
	/*
	 * margin between next interrupt occurrency and hw_fifo end_of_play
	 * using loaded samples
	 */
	int interrupt_margin_ps;
	/* codec operations as explained above */
	struct davinci_pcm_copy_ops *ops;
};

#endif /* _DAVINCI_PCM_COPY_H */
