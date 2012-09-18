/*
 * linux/sound/davinci_amico_asoc.h -- Platform data for DAVINCI AMICO ASOC
 *
 * Copyright 2011 Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_DAVINCI_AMICO_ASOC
#define __LINUX_SND_DAVINCI_AMICO_ASOC

struct amico_asoc_platform_data {
	/* an analog device connected to audio path */
	void (*ext_codec_power) (int);

	/* a power switch for analog audio path */
	void (*ext_circuit_power) (int);

	/* delayed work for long delay */
	struct delayed_work delayed_work;
};

#endif
