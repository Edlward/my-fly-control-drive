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
 * File      : fm25vxx.cpp
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

//#include "spi_bmi088.h"
//#include "spi_bmi088_fm25vxx.h"

#include "bmi08x.h"
#include "bmi088.h"
#include "bmi088_stm32.h"


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define BMI_DEBUG

#ifdef  BMI_DEBUG
#define BMI_TRACE         rt_kprintf
#else
#define BMI_TRACE(...)
#endif /* #ifdef BMI_DEBUG */

#define BMI_ACCEL_CS_Pin          7
#define BMI_GYRO_CS_Pin           18

///* JEDEC Manufacturer’s ID */
//#define FM_ID           (0xEF)

struct bmi08x_dev bmi088_dev = {
        .accel_id = BMI_ACCEL_CS_Pin,
        .gyro_id = BMI_GYRO_CS_Pin,
        .intf = BMI08X_SPI_INTF,
        .read = &stm32_spi_read,//user_spi_read
        .write = &stm32_spi_write,//user_spi_write
        .delay_ms = &BMI_Delay_Ms//user_delay_milli_sec
		//.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ,
		//.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL,
		//.accel_cfg.range = BMI088_ACCEL_RANGE_3G
};


struct rt_spi_device*   rt_bmi088_gyro_spi_device;
struct rt_spi_device*   rt_bmi088_accel_spi_device;


struct bmi08x_int_cfg int_config;
struct bmi08x_data_sync_cfg sync_cfg;
//int8_t rslt;
//struct bmi08x_sensor_data user_accel_bmi088;
//struct bmi08x_sensor_data user_gyro_bmi088;


rt_err_t bmi088_final_init(void);

void BMI_Delay_Ms(uint32_t time)
{
   rt_uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }	
}

int8_t stm32_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	struct rt_spi_device*   rt_bmi088_spi_device;
	reg_addr &= 0x7f;

	if(cs_pin == BMI_GYRO_CS_Pin)   rt_bmi088_spi_device = rt_bmi088_gyro_spi_device;
	if(cs_pin == BMI_ACCEL_CS_Pin)   rt_bmi088_spi_device = rt_bmi088_accel_spi_device;

	rt_spi_send(rt_bmi088_spi_device, &reg_addr, 1);
	rt_spi_send(rt_bmi088_spi_device, data, len);
//	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
//	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) ;
//	HAL_SPI_Transmit(&hspi1, data, len, 50);
//	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) ;
	return 0;
}

int8_t stm32_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	struct rt_spi_device*   rt_bmi088_spi_device;
	reg_addr |= 0x80;
	if(cs_pin == BMI_GYRO_CS_Pin)   rt_bmi088_spi_device = rt_bmi088_gyro_spi_device;
	if(cs_pin == BMI_ACCEL_CS_Pin)   rt_bmi088_spi_device = rt_bmi088_accel_spi_device;

	rt_spi_send(rt_bmi088_spi_device, &reg_addr, 1);
	rt_spi_recv(rt_bmi088_spi_device, data, len);
//	HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 50);
//	HAL_SPI_Receive(&hspi1, data, len, 50);

	return 0;
}


rt_err_t bmi088_final_init(void)
{

	  
	  //等待标志位 等待下一次初始化
	  static rt_int8_t BMI_init_flag = 0;
	  BMI_init_flag = BMI_init_flag + 1;
	  if( 1 == BMI_init_flag )   return  RT_EOK;  //等待另一部分初始化完毕

    rt_err_t    result = RT_EOK;
    rt_int8_t rslt=0;
	  int init_flag=0;
	
		/* Initialize bmi085 sensors (accel & gyro)*/
		rslt = bmi088_init(&bmi088_dev);
	  
	  if (rslt != BMI08X_OK)  BMI_TRACE("bmi088_accel init fail \r\n");
		/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
		rslt = bmi08a_soft_reset(&bmi088_dev);
//		rslt = bmi08g_soft_reset(&bmi088_dev);

		/*! Max read/write length (maximum supported length is 32).
		 To be set by the user */
		bmi088_dev.read_write_len = 32;
		/*set accel power mode */
		bmi088_dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
		rslt = bmi08a_set_power_mode(&bmi088_dev);

		bmi088_dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
		bmi08g_set_power_mode(&bmi088_dev);

		/* API uploads the bmi08x config file onto the device and wait for 150ms
			 to enable the data synchronization - delay taken care inside the function */
		rslt = bmi088_apply_config_file(&bmi088_dev);

		/*assign accel range setting*/
		bmi088_dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
		/*assign gyro range setting*/
		bmi088_dev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
		/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
		sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;
		rslt = bmi088_configure_data_synchronization(sync_cfg, &bmi088_dev);


		/*set accel interrupt pin configuration*/
		/*configure host data ready interrupt */
		int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
		int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
		int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

		/*configure Accel syncronization input interrupt pin */
		int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
		int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
		int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

		/*set gyro interrupt pin configuration*/
		int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
		int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
		int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
		int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

		int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
		int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
		int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
		int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

		/* Enable synchronization interrupt pin */
		rslt = bmi088_set_data_sync_int_config(&int_config, &bmi088_dev);

		init_flag=1;
		
    return result;
}



rt_err_t bmi088_accel_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;

    rt_bmi088_accel_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_bmi088_accel_spi_device == RT_NULL)
    {
        BMI_TRACE("spi device bmi088_accel %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz =  1 * 1000 * 1000; /* 5k */
		    rt_spi_configure(rt_bmi088_accel_spi_device, &cfg);

    }

_error_exit:
    return result;
}



rt_err_t bmi088_gyro_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;
    
    rt_bmi088_gyro_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_bmi088_gyro_spi_device == RT_NULL)
    {
        BMI_TRACE("spi device bmi088_gyro %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz =  1 * 1000 * 1000; /* 5k */
		    rt_spi_configure(rt_bmi088_gyro_spi_device, &cfg);

    }

_error_exit:
    return result;
}



int rt_bmi088_accel_init(void)
{
	
    stm32_spi_bus_attach_device(BMI_ACCEL_CS_Pin, "spi1", "bmi088_accel_spi");

    bmi088_accel_init("bmi088_accel_spi");
	
	  BMI_TRACE("bmi088_accel spi bus init ok\r\n");
	  return bmi088_final_init();
}
INIT_DEVICE_EXPORT(rt_bmi088_accel_init);



int rt_bmi088_gyro_init(void)
{
	  stm32_spi_bus_attach_device(BMI_GYRO_CS_Pin, "spi1", "bmi088_gyro_spi");
	
    bmi088_gyro_init("bmi088_gyro_spi");
	  BMI_TRACE("bmi088_gyro spi bus init ok\r\n");
	  return bmi088_final_init();
}
INIT_DEVICE_EXPORT(rt_bmi088_gyro_init);




/** @}*/
