/*
 * cir.h - Header file for the driver for Consumer Infrared (CIR)
	   (on Davinci-HD EVM)
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author: Suresh Rajashekara <suresh.r@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#ifndef __CIR_H__
#define __CIR_H__

#ifdef __KERNEL__
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/uaccess.h>

#include <mach/io.h>
#include <asm/atomic.h>

#define CIR_IOC_BASE     'C'
#define CIRIOC_GET_SYS_CLOCK_FREQ_PRESCALER  _IOR(CIR_IOC_BASE, 1, int)
#define CIRIOC_SET_SYS_CLOCK_FREQ_PRESCALER  _IOW(CIR_IOC_BASE, 2, int)
#define CIRIOC_GET_ENCODING                  _IOR(CIR_IOC_BASE, 3, int)

#define CIR_ENCODING_RC5   1

#define CIR_MAJOR_NUMBER 253	/* This macro is currently not used. Define it
				   to any value if you are using static minor
				   numbers and use the same in cir.c */
#define CIR_DEV_COUNT    1
#endif

#define CIR_IOC_MAGIC 'k'

/* Only 3 Ioctls now. Update if adding new ioctl */
#define CIR_IOC_MAXNR 2

/* Ioctl to flush the buffers driver uses to store the keys received */
#define CIR_FLUSH _IO(CIR_IOC_MAGIC, 0)

/* Change the duration, within which, if another key is received is neglected */
#define CIR_SET_REPEAT_DELAY _IOW(CIR_IOC_MAGIC, 1, int)

/* Read the duration, within which, if another key is received is neglected */
#define CIR_GET_REPEAT_DELAY _IOR(CIR_IOC_MAGIC, 2, int)

#endif /* _CIR_H__ */
