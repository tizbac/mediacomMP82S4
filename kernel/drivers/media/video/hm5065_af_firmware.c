/*
 * Driver for hm5065 CMOS Image Sensor from himax
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "hm5065.h"

static struct reginfo sensor_af_firmware[] =
{
		{0x070A, 0x03},      // Base on face mode , set to 0x03 . If set to contious mode , set to 0x01.
	{SEQUENCE_END,0x00},
};
static struct reginfo sensor_af_firmware1[] =
{
			{0x070B, 0x01},
	{SEQUENCE_END,0x00},
};
static struct reginfo sensor_af_firmware2[] =
{
			{0x070B, 0x02},
	{SEQUENCE_END,0x00},
};
