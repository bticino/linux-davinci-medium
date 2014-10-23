/*
 * I2c_KB_Touch.h - Configuration for I2C_KB_TOUCH keypad driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __LINUX_I2C_KB_TOUCH_H
#define __LINUX_I2C_KB_TOUCH_H

#include <linux/types.h>

/*
 * Largest keycode that the chip can send, plus one,
 * so keys can be mapped directly at the index of the
 * LM8323 keycode instead of subtracting one.
 */

#define I2C_KB_TOUCH_KEYMAP_SIZE	(0x7f + 1)
#define I2C_KB_TOUCH_NUM_PWMS		3

struct i2c_kb_touch_platform_data {
	int debounce_time; /* Time to watch for key bouncing, in ms. */
	int active_time; /* Idle time until sleep, in ms. */
	int size_x;
	int size_y;
	bool repeat;
	const unsigned short *keymap;
	const char *name; /* Device name. */
};

#endif /* __LINUX_I2C_KB_TOUCH_H */
