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
 * File      : mpu9250.cpp
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
#include <rtthread.h>
#include <rtdevice.h>

//#include "spi_mpu.h"
#include "spi_ms5611.h"
#include "drv_spi.h"
#include <drivers/spi.h>

//#include <stm32f4xx.h>
//#include <stm32f4xx_hal.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MS5611_DEBUG

#ifdef MS5611_DEBUG
#define MS5611_TRACE         rt_kprintf
#else
#define MPU_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define MS5611_CS_PIN    88

rt_uint16_t MS5611_C1 = 0, MS5611_C2 = 0, MS5611_C3 = 0, MS5611_C4 = 0, MS5611_C5 = 0, MS5611_C6 = 0;

static void Delay_Ms(rt_uint8_t time)
{
   rt_uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }	
}


rt_uint8_t SPIx_SendByte(struct rt_spi_device *device,rt_uint8_t byte)
{
	rt_uint8_t temp=0;
	temp = rt_spi_transfer(device, &byte, &temp, 1);
	return temp;
}

static rt_uint8_t MS5611_Reset(struct rt_spi_device *device)
{
	// Chip Select low 
//	Chip_Select(MS5611);
//	SPIx_SendByte(MS5611, MS5611_RESET);
//	Delay_Ms(3); //2.8ms reload from datasheet
//	// Chip Select high
//	Chip_DeSelect(MS5611);
	rt_int8_t  reset_cmd = MS5611_RESET ;
	rt_spi_send(device, &reset_cmd, 1);
	return 0;
}




static rt_uint16_t MS5611_SPIx_ReadWord(struct rt_spi_device *device, rt_int8_t addr)
{
	rt_uint8_t data[2] = {0};
	rt_uint16_t value = 0;

	// Chip Select low 
//	Chip_Select(MS5611);
	SPIx_SendByte(device, addr);
	data[0] = SPIx_SendByte(device, MS5611_READ);
	data[1] = SPIx_SendByte(device, MS5611_READ);
	// Chip Select high
//	Chip_DeSelect(MS5611);

	value = data[0] << 8 | data[1];
	return value;
}

static void MS5611_SPIx_ReadADC(struct rt_spi_device *device,rt_uint8_t osr, rt_uint32_t* value)
{
	rt_uint8_t data[3] = {0};

	// Chip Select low 
//	Chip_Select(MS5611);
	SPIx_SendByte(device, osr);
	// Chip Select high
//	Chip_DeSelect(MS5611);

	Delay_Ms(1);

	// Chip Select low 
//	Chip_Select(MS5611);
	SPIx_SendByte(device, MS5611_READ_ADC);
	data[0] = SPIx_SendByte(device, MS5611_READ_ADC);
	data[1] = SPIx_SendByte(device, MS5611_READ_ADC);
	data[2] = SPIx_SendByte(device, MS5611_READ_ADC);
	// Chip Select high
//	Chip_DeSelect(MS5611);

	*value = data[0] << 16 | data[1] << 8 | data[2];
}

static void MS5611_ReadPROM(struct rt_spi_device *device)
{
	// Read Calibration Data C1
	MS5611_C1 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C1);
	Delay_Ms(1);
	// Read Calibration Data C2
	MS5611_C2 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C2);
	Delay_Ms(1);
	// Read Calibration Data C3
	MS5611_C3 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C3);
	Delay_Ms(1);
	// Read Calibration Data C4
	MS5611_C4 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C4);
	Delay_Ms(1);
	// Read Calibration Data C5
	MS5611_C5 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C5);
	Delay_Ms(1);
	// Read Calibration Data C6
	MS5611_C6 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C6);
	Delay_Ms(1);
}
//typedef uint64_t u64;
//typedef int64_t rt_int64_t;

void MS5611_GetTemperatureAndPressure(struct rt_spi_device *device,rt_int64_t* T, rt_int64_t *P)
{
	rt_uint32_t D1, D2;
	rt_int32_t dT, TEMP, T2 = 0;
	rt_int64_t OFF, SENS, OFF2 = 0, SENS2 = 0;
	rt_int32_t lowTEMP, verylowTemp;
	
	MS5611_SPIx_ReadADC(device, D1_OSR_256, &D1);
	MS5611_SPIx_ReadADC(device, D2_OSR_256, &D2);
	//////////////////////////////////////////////////////////////////////////
	//
	dT = D2 - ((rt_uint32_t)MS5611_C5 << 8);
	TEMP = 2000 + (((rt_int64_t)dT * MS5611_C6) >> 23);
	OFF = ((rt_uint32_t)MS5611_C2 << 16) + ((MS5611_C4 * (rt_int64_t)dT) >> 7);
	SENS = ((rt_uint32_t)MS5611_C1 << 15) + ((MS5611_C3 * (rt_int64_t)dT) >> 8);
	//
	*T = TEMP;
	//////////////////////////////////////////////////////////////////////////
	//second order temperature compensation
	if(TEMP < 2000){
		T2 = (rt_int64_t)((rt_int64_t)dT * (rt_int64_t)dT) >> 31;
		lowTEMP = TEMP - 2000;
		lowTEMP *= lowTEMP;
		OFF2 = (5 * lowTEMP) >> 1;
		SENS2 = (5 * lowTEMP) >> 2;
		if(TEMP < -1500){
			verylowTemp = TEMP + 1500;
			verylowTemp *= verylowTemp;
			OFF2 = OFF2 + 7 * verylowTemp;
			SENS2 = SENS2 + ((11 * verylowTemp) >> 1);
		}
		//
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
		*T = TEMP - T2;
	}
	//////////////////////////////////////////////////////////////////////////
	*P = ((((rt_uint64_t)D1 * SENS) >> 21) - OFF) >> 15;
}

void MS5611_Cal(struct rt_spi_device *device,rt_int32_t* T, rt_int32_t *P)
{
	rt_uint32_t D1, D2;
	rt_int32_t dT, TEMP, T2 = 0;
	rt_int64_t OFF, SENS, OFF2 = 0, SENS2 = 0;
	rt_int32_t lowTEMP, verylowTemp;
	
	MS5611_SPIx_ReadADC(device, D1_OSR_256, &D1);
	MS5611_SPIx_ReadADC(device, D2_OSR_256, &D2);
	//////////////////////////////////////////////////////////////////////////
	//
	dT = D2 - ((rt_uint32_t)MS5611_C5 << 8);
	TEMP = 2000 + (((rt_int64_t)dT * MS5611_C6) >> 23);
	OFF = ((rt_uint32_t)MS5611_C2 << 16) + ((MS5611_C4 * (rt_int64_t)dT) >> 7);
	SENS = ((rt_uint32_t)MS5611_C1 << 15) + ((MS5611_C3 * (rt_int64_t)dT) >> 8);
	//
	*T = TEMP;
	//////////////////////////////////////////////////////////////////////////
	//second order temperature compensation
	if(TEMP < 2000)
	{
		T2 = (rt_int64_t)((rt_int64_t)dT * (rt_int64_t)dT) >> 31;
		lowTEMP = TEMP - 2000;
		lowTEMP *= lowTEMP;
		OFF2 = (5 * lowTEMP) >> 1;
		SENS2 = (5 * lowTEMP) >> 2;
		if(TEMP < -1500)
		{
			verylowTemp = TEMP + 1500;
			verylowTemp *= verylowTemp;
			OFF2 = OFF2 + 7 * verylowTemp;
			SENS2 = SENS2 + ((11 * verylowTemp) >> 1);
		}
		//
		OFF = OFF - OFF2;
		SENS = SENS - SENS2;
		*T = TEMP - T2;
	}
	//////////////////////////////////////////////////////////////////////////
	*P = ((((rt_uint64_t)D1 * SENS) >> 21) - OFF) >> 15;
}

rt_uint8_t MS5611_CRC4(rt_uint32_t n_prom[])
{
	int cnt; // simple counter
	rt_uint32_t n_rem; // crc reminder
	rt_uint32_t crc_read; // original value of the crc
	rt_uint8_t n_bit;
	n_rem = 0x00;
	crc_read = n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for(cnt = 0; cnt < 16; cnt++){ // operation is performed on bytes
		// choose LSB or MSB
		if((cnt & 0x01) == 1){
			n_rem ^= (rt_uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
		}
		else{
			n_rem ^= (rt_uint16_t)(n_prom[cnt >> 1] >> 8);
		}
		for(n_bit = 8; n_bit > 0; n_bit--){
			if(n_rem & (0x8000)){
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else{
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
	n_prom[7] =crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x00);
}



rt_err_t ms5611_init(const char * spi_device_name)
{
    rt_err_t result = RT_EOK;

    struct rt_spi_device*   rt_spi_device;

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        MS5611_TRACE("spi device %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 1 * 256 * 1000; /* 20 */
		rt_spi_configure(rt_spi_device, &cfg);

    }
	
	MS5611_Reset(rt_spi_device);
	Delay_Ms(1);
	MS5611_ReadPROM(rt_spi_device);
	Delay_Ms(1);	
	
	

//	MPU_TRACE("spi device ID:%ld !\r\n", id);

_error_exit:
    return result;
}

#define    VDD_3V3_SENSORS_EN_Pin    2


int rt_ms5611_init(void)
{
	
    stm32_spi_bus_attach_device(MS5611_CS_PIN, "spi1", "ms5611_spi");
	rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
    return ms5611_init("ms5611_spi");
}
INIT_DEVICE_EXPORT(rt_ms5611_init);





