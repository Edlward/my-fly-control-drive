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
 * File      : mpu9250.c
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

#include "delay.h"
#include "mpu9250.h"
#include "drv_spi.h"
#include <drivers/spi.h>


//#include <uORB/topics/sensor_accel.h>
//#include <uORB/topics/sensor_gyro.h>
//#include <uORB/topics/sensor_mag.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define FRAM_DEBUG

#ifdef FRAM_DEBUG
#define FRAM_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define MPU9250_CS_PIN 17
#define VDD_3V3_SENSORS_EN_Pin 2


static struct rt_spi_device*   rt_spi_device;


static rt_uint8_t mpu9250_write_byte(struct rt_spi_device *device,
	                                           rt_uint8_t reg_addr,
                                               rt_uint8_t data)
{

	rt_uint8_t cmd = reg_addr ;
	rt_spi_send_then_send(device, &cmd, 1, &data, 1);	
	return RT_EOK;
}

static rt_uint8_t mpu9250_write(struct rt_spi_device *device,
	                                           rt_uint8_t reg_addr,
                                               rt_uint8_t length,
                                               rt_uint8_t* data)
{

	rt_uint8_t cmd = reg_addr ;
	rt_spi_send_then_send(device, &cmd, 1, data, length);	
	return length;
}

static rt_uint8_t mpu9250_read(struct rt_spi_device *device,
	                                           rt_uint8_t reg_addr,
                                               rt_uint8_t length,
                                               rt_uint8_t* data)
{

	rt_uint8_t cmd =  MPU9250_SPI_READ | reg_addr ;
	rt_spi_send_then_recv(device, &cmd, 1, data, length);	
	return length;
}


static void MPU9250_Get9AxisRawData(struct rt_spi_device *device,
	                                    rt_int16_t *accel,
                                        rt_int16_t * gyro,
                                        rt_int16_t * mag)
{
	rt_int16_t data[14];
	mpu9250_read(device, MPU9250_ACCEL_XOUT_H, 14, (rt_uint8_t*)data);
	
	accel[0] = (0x00ff&data[0] << 8) | (0x00ff&data[1]);
	accel[1] = (0x00ff&data[2] << 8) | (0x00ff&data[3]);
	accel[2] = (0x00ff&data[4] << 8) | (0x00ff&data[5]);
	
	gyro[0] = (0x00ff&data[8] << 8) | (0x00ff&data[9]);
	gyro[1] = (0x00ff&data[10] << 8) | (0x00ff&data[11]);
	gyro[2] = (0x00ff&data[12] << 8) | (0x00ff&data[13]);
	
	mag[0] = 0;
	mag[1] = 0;
	mag[2] = 0;

}

static rt_uint8_t mpu9250_read_id(struct rt_spi_device *device)
{
    rt_uint8_t cmd;
    rt_uint8_t id_recv;

    cmd = 0xFF; 
    rt_spi_send(device, &cmd, 1);

    /* read flash id */
    cmd = MPU9250_SPI_READ|MPU9250_WHO_AM_I;
    rt_spi_send_then_recv(device, &cmd, 1, &id_recv, 1);

    return id_recv;
}

rt_err_t mpu9250_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;
    rt_uint32_t id;
    rt_uint8_t  send_buffer[3];
    
    rt_spi_device = (struct rt_spi_device*) rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        FRAM_TRACE("spi device %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 20 * 1000 * 1000; /* 20 */
		rt_spi_configure(rt_spi_device, &cfg);

    }

    id = mpu9250_read_id(rt_spi_device);
	FRAM_TRACE("mpu9250 device ID:%o !\r\n", id);


_error_exit:
    return result;
}

int rt_mpu9250_init(void)
{
    stm32_spi_bus_attach_device(MPU9250_CS_PIN, "spi1", "mpuspi");
    rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
    return mpu9250_init("mpuspi");
}
INIT_DEVICE_EXPORT(rt_mpu9250_init);


static void mpu9250_measure()
{
	rt_int16_t acc[3],gro[3],mag[3];
	MPU9250_Get9AxisRawData(rt_spi_device,acc,gro,mag);
	FRAM_TRACE("accX:%d  accY:%d  accZ:%d !\r\n", acc[0],acc[1],acc[2]);

}



static void mpu9250_config()
{
	rt_uint8_t data = 0, state = 0;
	//MPU9250 Reset
	mpu9250_write_byte(rt_spi_device, MPU9250_PWR_MGMT_1,MPU9250_RESET);
	time_waitMs(100);
	//MPU9250 Set Clock Source
	mpu9250_write_byte(rt_spi_device, MPU9250_PWR_MGMT_1,MPU9250_CLOCK_PLLGYROZ);
	time_waitMs(5);
	//MPU9250 Set Interrupt
	mpu9250_write_byte(rt_spi_device, MPU9250_INT_PIN_CFG,MPU9250_INT_ANYRD_2CLEAR);
	time_waitMs(5);
	mpu9250_write(rt_spi_device, MPU9250_INT_ENABLE,1,0xFF);
	time_waitMs(5);
	//MPU9250 Set Sensors
	mpu9250_write_byte(rt_spi_device, MPU9250_PWR_MGMT_2,MPU9250_XYZ_GYRO & MPU9250_XYZ_ACCEL);
	time_waitMs(5);
	//MPU9250 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	mpu9250_write_byte(rt_spi_device, MPU9250_SMPLRT_DIV,SMPLRT_DIV);
	time_waitMs(5);
	//MPU9250 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	mpu9250_write_byte(rt_spi_device, MPU9250_GYRO_CONFIG,(MPU9250_FSR_2000DPS << 3));
	time_waitMs(5);
	//MPU9250 Set Full Scale Accel Range PS:2G
	mpu9250_write_byte(rt_spi_device, MPU9250_ACCEL_CONFIG,(MPU9250_FSR_2G << 3));
	time_waitMs(5);
	//MPU9250 Set Accel DLPF
	mpu9250_read(rt_spi_device, MPU9250_ACCEL_CONFIG2,1,&data);
	data |= MPU9250_ACCEL_DLPF_41HZ;
	time_waitMs(5);
	mpu9250_write_byte(rt_spi_device, MPU9250_ACCEL_CONFIG2,data);
	time_waitMs(5);
	//MPU9250 Set Gyro DLPF
	mpu9250_write_byte(rt_spi_device, MPU9250_CONFIG,MPU9250_GYRO_DLPF_41HZ);
	time_waitMs(5);
	//MPU9250 Set SPI Mode
	mpu9250_read(rt_spi_device,MPU9250_USER_CTRL,1,&state);
	time_waitMs(5);
	mpu9250_write_byte(rt_spi_device, MPU9250_USER_CTRL,state | MPU9250_I2C_IF_DIS);
	time_waitMs(5);
	mpu9250_read(rt_spi_device, MPU9250_USER_CTRL,1,&state);
	time_waitMs(5);
	mpu9250_write_byte(rt_spi_device, MPU9250_USER_CTRL,state | MPU9250_I2C_MST_EN);
	time_waitMs(5);
}


static void mpu9250_start(void)
{
	mpu9250_config();
	
	while(1)
	{
		mpu9250_measure();
		rt_thread_mdelay(500);
	}
	
}


rt_err_t mpu9250_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			mpu9250_start();
		}

		if (!strcmp(verb, "stop")) {
			//mpu9250_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//mpu9250_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//mpu9250_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//mpu9250_info(busid);
		}
	}
	

	//mpu9250_usage();
	return 0;
}

MSH_CMD_EXPORT(mpu9250_main,mpu9250)