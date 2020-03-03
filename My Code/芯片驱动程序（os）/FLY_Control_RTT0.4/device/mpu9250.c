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
#include "spi_imu_mpu9250.h"
#include "drv_spi.h"
#include <drivers/spi.h>

#include <stm32f4xx.h>
//#include <stm32f4xx_hal.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MPU_DEBUG

#ifdef MPU_DEBUG
#define MPU_TRACE         rt_kprintf
#else
#define MPU_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define MPU_CS_PIN    17

/* JEDEC Manufacturer’s ID */
#define MPU_ID           (0xEE)


static rt_int16_t MPU9250_AK8963_ASA[3] = {0, 0, 0};

static void Delay_Ms(rt_uint8_t time)
{
   rt_uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }	
}


static rt_int8_t MPU9250_SPIx_Write(struct rt_spi_device *device,rt_uint8_t reg_addr,rt_uint8_t data)
{	
	rt_spi_send(device, &reg_addr, 1);
	rt_spi_send(device, &data, 1);
	return 0;
}
static rt_int8_t MPU9250_SPIx_Writes(struct rt_spi_device *device,
	                                         rt_uint8_t reg_addr,
                                             rt_uint8_t length,
                                             rt_uint8_t* data)
{
	
	rt_spi_send(device, &reg_addr, 1);
	rt_spi_send(device, &data, length);
//	while(i < length)
//	{
//		rt_spi_send(device, data[i++], 1);
//	}	
	return 0;
}

static rt_int8_t MPU9250_SPIx_Read(struct rt_spi_device *device,
	                                         rt_uint8_t reg_addr)
{
	rt_uint8_t data = 0;
	rt_uint8_t cmd =  MPU9250_I2C_READ | reg_addr ;	
	rt_spi_send_then_recv(device, &cmd, 1, &data, 1);
	return data;
}

static rt_int8_t MPU9250_SPIx_Reads(struct rt_spi_device *device,
	                                           rt_uint8_t reg_addr,
                                               rt_uint8_t length,
                                               rt_uint8_t* data)
{

	rt_uint8_t cmd =  MPU9250_I2C_READ | reg_addr ;
	rt_spi_send_then_recv(device, &cmd, 1, data, length);	
	return 0;
}

static rt_int8_t MPU9250_AK8963_SPIx_Read(struct rt_spi_device *device,
	                                                rt_uint8_t akm_addr,
                                                    rt_uint8_t reg_addr,
                                                    rt_uint8_t* data) 
{
	rt_uint8_t status = 0;
	rt_uint32_t timeout = 0;

	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_REG, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_ADDR, 1, &reg_addr);
	Delay_Ms(1);
	reg_addr = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_CTRL, 1, &reg_addr);
	Delay_Ms(1);

	do
	{
		if (timeout++ > 50)
		{
			return -2;
		}
		MPU9250_SPIx_Reads(device, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} 
	while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	MPU9250_SPIx_Reads(device, MPU9250_I2C_SLV4_DI, 1, data);
	return 0;
}

static rt_int8_t MPU9250_AK8963_SPIx_Reads(struct rt_spi_device *device,
	                                                 rt_uint8_t akm_addr, 
                                                     rt_uint8_t reg_addr, 
                                                          rt_uint8_t len, 
                                                         rt_uint8_t* data)
{
	rt_uint8_t index = 0;
	rt_uint8_t status = 0;
	rt_uint32_t timeout = 0;
	rt_uint8_t tmp = 0;

	tmp = akm_addr | MPU9250_I2C_READ;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	while(index < len)
	{
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do 
		{
			if (timeout++ > 50)
			{
				return -2;
			}
			MPU9250_SPIx_Reads(device, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(2);
		} 
		while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		MPU9250_SPIx_Reads(device, MPU9250_I2C_SLV4_DI, 1, data + index);
		Delay_Ms(1);
		index++;
	}
	return 0;
}

static rt_int8_t MPU9250_AK8963_SPIx_Write(struct rt_spi_device *device,
	                                                 rt_uint8_t akm_addr,
                                                     rt_uint8_t reg_addr,
                                                         rt_uint8_t data)
{
	rt_uint32_t timeout = 0;
	rt_uint8_t status = 0;
	rt_uint8_t tmp = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);
	tmp = reg_addr;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_REG, 1, &tmp);
	Delay_Ms(1);
	tmp = data;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_DO, 1, &tmp);
	Delay_Ms(1);
	tmp = MPU9250_I2C_SLV4_EN;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
	Delay_Ms(1);

	do {
		if (timeout++ > 50)
			return -2;

		MPU9250_SPIx_Reads(device, MPU9250_I2C_MST_STATUS, 1, &status);
		Delay_Ms(1);
	} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
	if (status & MPU9250_I2C_SLV4_NACK)
		return -3;
	return 0;
}


static rt_int8_t MPU9250_AK8963_SPIx_Writes(struct rt_spi_device *device,
	                                                  rt_uint8_t akm_addr,
                                                      rt_uint8_t reg_addr, 
                                                           rt_uint8_t len,
                                                         rt_uint8_t* data)
{
	rt_uint32_t timeout = 0;
	rt_uint8_t status = 0;
	rt_uint8_t tmp = 0;
	rt_uint8_t index = 0;

	tmp = akm_addr;
	MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_ADDR, 1, &tmp);
	Delay_Ms(1);

	while(index < len){
		tmp = reg_addr + index;
		MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_REG, 1, &tmp);
		Delay_Ms(1);
		MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_DO, 1, data + index);
		Delay_Ms(1);
		tmp = MPU9250_I2C_SLV4_EN;
		MPU9250_SPIx_Writes(device, MPU9250_I2C_SLV4_CTRL, 1, &tmp);
		Delay_Ms(1);

		do {
			if (timeout++ > 50)
				return -2;
			MPU9250_SPIx_Reads(device, MPU9250_I2C_MST_STATUS, 1, &status);
			Delay_Ms(1);
		} while ((status & MPU9250_I2C_SLV4_DONE) == 0);
		if (status & MPU9250_I2C_SLV4_NACK)
			return -3;
		index++;
	}
	return 0;
}



void MPU9250_Get9AxisRawData(struct rt_spi_device *device,
	                                    rt_int16_t *accel,
                                        rt_int16_t * gyro,
                                        rt_int16_t * mag)
{
	rt_uint8_t data[22];
	MPU9250_SPIx_Reads(device, MPU9250_ACCEL_XOUT_H, 22, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
	
	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];

	if (!(data[14] & MPU9250_AK8963_DATA_READY) || (data[14] & MPU9250_AK8963_DATA_OVERRUN))
	{
		return;
	}
	if (data[21] & MPU9250_AK8963_OVERFLOW)
	{
		return;
	}
	mag[0] = (data[16] << 8) | data[15];
	mag[1] = (data[18] << 8) | data[17];
	mag[2] = (data[20] << 8) | data[19];

	//ned x,y,z
	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get6AxisRawData(struct rt_spi_device *device,rt_int16_t *accel, rt_int16_t * gyro)
{
	rt_uint8_t data[14];
	MPU9250_SPIx_Reads(device, MPU9250_ACCEL_XOUT_H, 14, data);
	
	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];

	gyro[0] = (data[8] << 8) | data[9];
	gyro[1] = (data[10] << 8) | data[11];
	gyro[2] = (data[12] << 8) | data[13];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisAccelRawData(struct rt_spi_device *device,rt_int16_t * accel)
{
	rt_uint8_t data[6];
	MPU9250_SPIx_Reads(device, MPU9250_ACCEL_XOUT_H, 6, data);

	accel[0] = (data[0] << 8) | data[1];
	accel[1] = (data[2] << 8) | data[3];
	accel[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisGyroRawData(struct rt_spi_device *device,rt_int16_t * gyro)
{
	rt_uint8_t data[6];
	MPU9250_SPIx_Reads(device, MPU9250_GYRO_XOUT_H, 6, data);

	gyro[0] = (data[0] << 8) | data[1];
	gyro[1] = (data[2] << 8) | data[3];
	gyro[2] = (data[4] << 8) | data[5];
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_Get3AxisMagnetRawData(struct rt_spi_device *device,rt_int16_t *mag)
{
	rt_uint8_t data[8];

	MPU9250_SPIx_Reads(device, MPU9250_EXT_SENS_DATA_00, 8, data);
	if (!(data[0] & MPU9250_AK8963_DATA_READY) || (data[0] & MPU9250_AK8963_DATA_OVERRUN))
	{
		return;
	}
	if (data[7] & MPU9250_AK8963_OVERFLOW)
	{
		return;
	}
	mag[0] = (data[2] << 8) | data[1];
	mag[1] = (data[4] << 8) | data[3];
	mag[2] = (data[6] << 8) | data[5];

	mag[0] = ((long)mag[0] * MPU9250_AK8963_ASA[0]) >> 8;
	mag[1] = ((long)mag[1] * MPU9250_AK8963_ASA[1]) >> 8;
	mag[2] = ((long)mag[2] * MPU9250_AK8963_ASA[2]) >> 8;
}
//////////////////////////////////////////////////////////////////////////
//
void MPU9250_GetTemperatureRawData(struct rt_spi_device *device,rt_int64_t *temperature)
{
	rt_uint8_t data[2];
	MPU9250_SPIx_Reads(device, MPU9250_TEMP_OUT_H, 2, data);
	temperature[0] = (((rt_int16_t)data[0]) << 8) | data[1];
}


static rt_uint8_t mpu9250_read_id(struct rt_spi_device *device)
{
    rt_uint8_t cmd;
    rt_uint8_t id_recv=0;

    cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
    rt_spi_send(device, &cmd, 1);

    /* read flash id */
    cmd = MPU9250_I2C_READ | MPU9250_WHO_AM_I;
    rt_spi_send_then_recv(device, &cmd, 1, &id_recv, 1);
    return id_recv;
//    return (rt_uint32_t)(id_recv[0] << 16) | (id_recv[1] << 8) | id_recv[2];
}


rt_err_t mpu9250_init(const char * spi_device_name)
{
    rt_err_t result = RT_EOK;
    rt_uint8_t id;
	rt_uint8_t data = 0, state = 0;
	rt_uint8_t response[3] = {0, 0, 0};	
    struct rt_spi_device*   rt_spi_device;

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        MPU_TRACE("spi device %s not found!\r\n", spi_device_name);
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
	
	
//	rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);

	//////////////////////////////////////////////////////////////////////////
	//MPU9250 Reset
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_PWR_MGMT_1, MPU9250_RESET);
	Delay_Ms(100);
	//MPU9250 Set Clock Source
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_PWR_MGMT_1,  MPU9250_CLOCK_PLLGYROZ);
	Delay_Ms(1);
	//MPU9250 Set Interrupt
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_INT_PIN_CFG,  MPU9250_INT_ANYRD_2CLEAR);
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_INT_ENABLE, ENABLE);
	Delay_Ms(1);
	//MPU9250 Set Sensors
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_PWR_MGMT_2, MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	Delay_Ms(1);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_SMPLRT_DIV, SMPLRT_DIV);
	Delay_Ms(1);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_GYRO_CONFIG, (MPU9250_FSR_2000DPS << 3));
	Delay_Ms(1);
	//MPU9250 Set Full Scale Accel Range PS:2G
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_ACCEL_CONFIG, (MPU9250_FSR_2G << 3));
	Delay_Ms(1);
	//MPU9250 Set Accel DLPF
	data = MPU9250_SPIx_Read(rt_spi_device, MPU9250_ACCEL_CONFIG2);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_ACCEL_CONFIG2, data);
	Delay_Ms(1);
	//MPU9250 Set Gyro DLPF
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_CONFIG, MPU9250_GYRO_DLPF_41HZ);
	Delay_Ms(1);
	//MPU9250 Set SPI Mode
	state = MPU9250_SPIx_Read(rt_spi_device, MPU9250_USER_CTRL);
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_USER_CTRL, state | MPU9250_I2C_IF_DIS);
	Delay_Ms(1);
	state = MPU9250_SPIx_Read(rt_spi_device, MPU9250_USER_CTRL);
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_USER_CTRL, state | MPU9250_I2C_MST_EN);
	Delay_Ms(1);
	//////////////////////////////////////////////////////////////////////////
	//AK8963 Setup
	//reset AK8963
	MPU9250_AK8963_SPIx_Write(rt_spi_device,MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	Delay_Ms(2);

	MPU9250_AK8963_SPIx_Write(rt_spi_device,MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	Delay_Ms(1);
	MPU9250_AK8963_SPIx_Write(rt_spi_device,MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	Delay_Ms(1);
	//
	//AK8963 get calibration data
	MPU9250_AK8963_SPIx_Reads(rt_spi_device,MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	MPU9250_AK8963_ASA[0] = (rt_int16_t)(response[0]) + 128;
	MPU9250_AK8963_ASA[1] = (rt_int16_t)(response[1]) + 128;
	MPU9250_AK8963_ASA[2] = (rt_int16_t)(response[2]) + 128;
	Delay_Ms(1);
	MPU9250_AK8963_SPIx_Write(rt_spi_device,MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	Delay_Ms(1);
	//
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_I2C_MST_CTRL, 0x5D);
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU9250_I2C_READ);
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	Delay_Ms(1);
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_I2C_SLV0_CTRL, 0x88);
	Delay_Ms(1);
	//
	MPU9250_AK8963_SPIx_Write(rt_spi_device,MPU9250_AK8963_I2C_ADDR, MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	Delay_Ms(1);

	//
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_I2C_SLV4_CTRL, 0x09);
	Delay_Ms(1);
	//
		
	MPU9250_SPIx_Write(rt_spi_device, MPU9250_I2C_MST_DELAY_CTRL, 0x81);
	Delay_Ms(100);	

	id = mpu9250_read_id(rt_spi_device);  //读ID
	
	if(id != 0x71)  MPU_TRACE("spi device ID  not right! %d\r\n",id);

//    id = mpu9250_read_id(rt_spi_device);
//	MPU_TRACE("spi device ID:%ld !\r\n", id);

_error_exit:
    return result;
}

#define    VDD_3V3_SENSORS_EN_Pin    2


int rt_mpu_init(void)
{
	
    stm32_spi_bus_attach_device(MPU_CS_PIN, "spi1", "mpu_spi");

    return mpu9250_init("mpu_spi");
}
INIT_DEVICE_EXPORT(rt_mpu_init);












