/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-16     aozima      the first version
 * 2012-08-23     aozima       add flash lock.
 */

#ifndef SPI_MS5611_H_
#define SPI_MS5611_H_

#include <rtthread.h>

#include "drv_spi.h"


#define MS5611_RESET 0x1E
#define MS5611_READ_ADC 0x00
#define MS5611_READ 0x00
//OSR 256 0.60 mSec conversion time (1666.67 Hz)
//OSR 512 1.17 mSec conversion time ( 854.70 Hz)
//OSR 1024 2.28 mSec conversion time ( 357.14 Hz)
//OSR 2048 4.54 mSec conversion time ( 220.26 Hz)
//OSR 4096 9.04 mSec conversion time ( 110.62 Hz)
#define D1_OSR_256 0x40
#define D1_OSR_512 0x42
#define D1_OSR_1024 0x44
#define D1_OSR_2048 0x46
#define D1_OSR_4096 0x48
#define D2_OSR_256 0x50
#define D2_OSR_512 0x52
#define D2_OSR_1024 0x54
#define D2_OSR_2048 0x56
#define D2_OSR_4096 0x58
#define MS5611_READ_PROM_RSV 0xA0 //0xA0 to 0xAE
#define MS5611_READ_PROM_C1 0xA2
#define MS5611_READ_PROM_C2 0xA4
#define MS5611_READ_PROM_C3 0xA6
#define MS5611_READ_PROM_C4 0xA8
#define MS5611_READ_PROM_C5 0xAA
#define MS5611_READ_PROM_C6 0xAC
#define MS5611_READ_PROM_CRC 0xAE


extern  rt_uint16_t MS5611_C1,MS5611_C2,MS5611_C3;


//u8 MS5611_SPIx_SendByte(u8);

//void MS5611_Init(void);
//void MS5611_GetTemperatureAndPressure(s32* TEMP, s32 *P);
//void MS5611_Cal(s32* T, s32 *P);

void MS5611_GetTemperatureAndPressure(struct rt_spi_device *device,rt_int64_t* T, rt_int64_t *P);
void MS5611_Cal(struct rt_spi_device *device,rt_int32_t* T, rt_int32_t *P);




#endif



