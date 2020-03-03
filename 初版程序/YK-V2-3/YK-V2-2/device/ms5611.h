/*
 *    $$\     $$\ $$\   $$\         $$\   $$\ $$$$$$$\
 *    \$$\   $$  |$$ | $$  |        $$ |  $$ |$$  __$$\
 *     \$$\ $$  / $$ |$$  /         $$ |  $$ |$$ |  $$ |
 *      \$$$$  /  $$$$$  /  $$$$$$\ $$$$$$$$ |$$ |  $$ |
 *       \$$  /   $$  $$<   \______|$$  __$$ |$$ |  $$ |
 *        $$ |    $$ |\$$\          $$ |  $$ |$$ |  $$ |
 *        $$ |    $$ | \$$\         $$ |  $$ |$$$$$$$  |
 *        \__|    \__|  \__|        \__|  \__|\_______/ 
 *                                                      
 * File      : ms5611.h
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#ifndef _MS5611_H
#define _MS5611_H

#include "common.h"

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


#define DELAY_OSR_256 	600       //us
#define DELAY_OSR_512 	1170
#define DELAY_OSR_1024 	2280
#define DELAY_OSR_2048 	4540
#define DELAY_OSR_4096 	9040


rt_bool_t sensor_ms5611_ready(void);
rt_err_t sensor_get_ms5611_data(float *sensor_temp,float *sensor_press);
#endif
