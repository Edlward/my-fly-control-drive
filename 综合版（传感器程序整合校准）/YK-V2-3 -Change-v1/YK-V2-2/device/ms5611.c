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
 * File      : ms5611.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
#include "ms5611.h"
#include "drv_spi.h"
#include <drivers/spi.h>

#define MS_DEBUG

#ifdef MS_DEBUG
#define MS_TRACE         rt_kprintf
#else
#define MS_TRACE(...)
#endif /* #ifdef MS_DEBUG */

#define MS5611_CS_PIN 88
#define VDD_3V3_SENSORS_EN_Pin 2

static struct rt_spi_device*   rt_spi_device;

static rt_uint16_t MS5611_C1 = 0,MS5611_C2 = 0,MS5611_C3 = 0,MS5611_C4 = 0,MS5611_C5 = 0,MS5611_C6 = 0;
static rt_int32_t temperature;
static rt_int32_t pressure;
static hrt_abstime osr_send_time=0,last_osr_send_time=0;

static rt_uint8_t MS5611_Reset(struct rt_spi_device *device)
{
	rt_int8_t  reset_cmd = MS5611_RESET ;
	rt_spi_send(device, &reset_cmd, 1);
	return 0;
}

static rt_uint16_t MS5611_SPIx_ReadWord(struct rt_spi_device *device, rt_int8_t addr)
{
	rt_uint8_t data[2] = {0};
	rt_uint16_t value = 0;

	rt_spi_send_then_recv(device, &addr, 1, data, 2);
	//MS_TRACE("1:%o 2:%o !\r\n", data[0],data[1]);
	value = data[0] << 8 | data[1];
	return value;
}

static void MS5611_SPIx_ReadDATA(struct rt_spi_device *device,rt_uint8_t osr, rt_uint32_t* value)
{
	rt_uint8_t data[3] = {0};
	rt_spi_send(device, &osr,1);
	time_waitMs(1);
	
	rt_int8_t read_cmd = MS5611_READ_ADC;
	rt_spi_send_then_recv(device, &read_cmd,1,data,3);
	*value = data[0] << 16 | data[1] << 8 | data[2];
}

static void MS5611_SPIx_SendOSR(struct rt_spi_device *device,rt_uint8_t osr)
{
	osr_send_time = hrt_absolute_time_us();
	rt_spi_send(device, &osr,1);
	
}

static void MS5611_SPIx_ReadADC(struct rt_spi_device *device,rt_uint32_t* value)
{
	rt_uint8_t data[3] = {0};
	rt_int8_t read_cmd = MS5611_READ_ADC;
	rt_spi_send_then_recv(device, &read_cmd,1,data,3);
	*value = data[0] << 16 | data[1] << 8 | data[2];
}


static void MS5611_ReadPROM(struct rt_spi_device *device)
{
	// Read Calibration Data C1
	MS5611_C1 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C1);
	time_waitMs(1);
	// Read Calibration Data C2
	MS5611_C2 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C2);
	time_waitMs(1);
	// Read Calibration Data C3
	MS5611_C3 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C3);
	time_waitMs(1);
	// Read Calibration Data C4
	MS5611_C4 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C4);
	time_waitMs(1);
	// Read Calibration Data C5
	MS5611_C5 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C5);
	time_waitMs(1);
	// Read Calibration Data C6
	MS5611_C6 = MS5611_SPIx_ReadWord(device, MS5611_READ_PROM_C6);
	time_waitMs(1);
	
	//MS_TRACE("1:%d 2:%d 3:%d 4:%d 5:%d 6:%d!\r\n", MS5611_C1,MS5611_C2,MS5611_C3,MS5611_C4,MS5611_C5,MS5611_C6);
}

void MS5611_GetTemperatureAndPressure(struct rt_spi_device *device,rt_uint32_t D1, rt_uint32_t D2)
{	
	rt_int32_t dT, TEMP, T2 = 0;
	rt_int64_t OFF, SENS, OFF2 = 0, SENS2 = 0;
	rt_int32_t lowTEMP, verylowTemp;
	
//	MS5611_SPIx_ReadADC(device, D1_OSR_256, &D1);
//	MS5611_SPIx_ReadADC(device, D2_OSR_256, &D2);
	//MS_TRACE("D1:%d D2:%d!\r\n", D1,D2);
	//////////////////////////////////////////////////////////////////////////
	//
	dT = D2 - ((rt_uint32_t)MS5611_C5 << 8);
	TEMP = 2000 + (((rt_int64_t)dT * MS5611_C6) >> 23);
	OFF = ((rt_uint32_t)MS5611_C2 << 16) + ((MS5611_C4 * (rt_int64_t)dT) >> 7);
	SENS = ((rt_uint32_t)MS5611_C1 << 15) + ((MS5611_C3 * (rt_int64_t)dT) >> 8);
	//
	temperature = TEMP;
	//MS_TRACE("TEMP:%d !\r\n",temperature);
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
		temperature = TEMP - T2;
	}
	//////////////////////////////////////////////////////////////////////////
	pressure = ((((rt_uint64_t)D1 * SENS) >> 21) - OFF) >> 15;
	//MS_TRACE("PRESS:%d !\r\n",pressure);
}

rt_bool_t sensor_ms5611_ready(void)
{
	hrt_abstime now = hrt_absolute_time_us();
	if(now - osr_send_time>DELAY_OSR_256)
	{
		//LOG_RAW("1:%lld  2:%lld \n",now,osr_send_time);
		return RT_TRUE;
	}
	else
	{
		return RT_FALSE;
	}	
}
rt_uint32_t data[2]={0,0};
rt_err_t sensor_get_ms5611_data(float *sensor_temp,float *sensor_press)
{
	static rt_uint8_t d_flag = 3;	
	
	MS5611_SPIx_ReadADC(rt_spi_device,&data[d_flag]);
	if(3 != d_flag&&d_flag == 1)
	{		
		MS5611_GetTemperatureAndPressure(rt_spi_device,data[0],data[1]);
		*sensor_temp = temperature/100.0f;
		*sensor_press = pressure/100.0f;
		
	}
	if(0 != d_flag)
	{
		MS5611_SPIx_SendOSR(rt_spi_device,D1_OSR_256);
		d_flag = 0;
		return RT_EOK;
	}
	else 
	{
		MS5611_SPIx_SendOSR(rt_spi_device,D2_OSR_256);
		d_flag = 1;
		return RT_ERROR;
	}	

}

static void ms5611_start(void)
{

	//MS5611_ReadPROM(rt_spi_device);
	while(1)
	{
		//MS5611_GetTemperatureAndPressure(rt_spi_device);
		MS_TRACE("temperature:%d pressure:%d !\r\n", temperature,pressure);
		rt_thread_mdelay(500);
	}
	
}


rt_err_t ms5611_init(const char * spi_device_name)
{
    rt_err_t result = RT_EOK;

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        MS_TRACE("spi device %s not found!\r\n", spi_device_name);
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
	time_waitMs(3);
	MS5611_ReadPROM(rt_spi_device);
	time_waitMs(1);		

//	MPU_TRACE("spi device ID:%ld !\r\n", id);

_error_exit:
    return result;
}

int rt_ms5611_init(void)
{
	
    stm32_spi_bus_attach_device(MS5611_CS_PIN, "spi1", "ms_spi");
	rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
    return ms5611_init("ms_spi");
}
INIT_APP_EXPORT(rt_ms5611_init);



rt_err_t ms5611_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			ms5611_start();
		}

		if (!strcmp(verb, "stop")) {
			//ms5611_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//ms5611_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//ms5611_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//ms5611_info(busid);
		}
	}
	

	//ms5611_usage();
	return 0;
}

MSH_CMD_EXPORT(ms5611_main,ms5611)