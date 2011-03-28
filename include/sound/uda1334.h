/*
 * uda1334.h  --  UDA1334 ALSA SoC Codec driver
 *
 * Author: Davide Bonfanti <davide.bonfanti@bticino.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _UDA1334_H
#define _UDA1334_H

struct uda1334_platform_data {
	/* gpio connected to mute DAC pin. -1 if not connected */
	int mute_gpio;

	/* gpio connected to de-emphasys DAC pin. -1 if not connected */
	int deemp_gpio;

	/* gpio connected to PCS DAC pin. -1 if not connected */
	int reset_gpio;

	/* Function intended to turn on/off auxiliary circuitry */
	void (*power) (int);
};

#endif /* _UDA1334_H */
