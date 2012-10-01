/*
 * linux/mcp453x.h -- Platform data for MXP453X
 *
 * Copyright 2012 Bticino S.p.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MCP453X_H
#define __LINUX_MCP453X_H

struct mcp453x_platform_data {
	u8 volatile_wiper0;
	u8 volatile_tcon;
};

#endif
