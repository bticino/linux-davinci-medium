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

#ifndef __DM365_ADC_H
#define __DM365_ADC_H

#define ADC_CH0_EANBLE	BIT(0)
#define ADC_CH1_EANBLE	BIT(1)
#define ADC_CH2_EANBLE	BIT(2)
#define ADC_CH3_EANBLE	BIT(3)
#define ADC_CH4_EANBLE	BIT(4)
#define ADC_CH5_EANBLE	BIT(5)

#define ADC_CH0_ONESHOOT BIT(8)
#define ADC_CH1_ONESHOOT BIT(9)
#define ADC_CH2_ONESHOOT BIT(10)
#define ADC_CH3_ONESHOOT BIT(11)
#define ADC_CH4_ONESHOOT BIT(12)
#define ADC_CH5_ONESHOOT BIT(13)

#define DM365_ADCIF_BASE	0x01C23C00
#define ADCTL			0x00
#define SET_DIV			0x10
#define CHSEL			0x14
#define AD_DAT(x)		0x18 + 4 * (x)

#define ADCTL_START		BIT(0)
#define ADCTL_SCAN_MODE		BIT(1)
#define ADCTL_SCNIEN		BIT(2)
#define ADCTL_SCNFLG		BIT(3)
#define ADCTL_BUSY		BIT(7)


struct davinci_adc_platform_data {
	u32 adc_configuration;
	struct completion done; /* used to wait for adc complete reading */
};

enum davinci_adc {
	DAVINCI_ADC0,
	DAVINCI_ADC1,
	DAVINCI_ADC2,
	DAVINCI_ADC3,
	DAVINCI_ADC4,
	DAVINCI_ADC5,
};

u32 davinci_adc_read_oneshot(struct davinci_adc_platform_data *adc_data, u32 index);

#endif /* __DM365_ADC_H */
