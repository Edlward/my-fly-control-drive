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

#include "mlx90393.h"
#include "drv_spi.h"
#include <drivers/spi.h>

#include <stm32f4xx.h>
//#include <stm32f4xx_hal.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MLX_DEBUG

#ifdef MLX_DEBUG
#define MPU_TRACE         rt_kprintf
#else
#define MLX_DEBUG(...)
#endif /* #ifdef FLASH_DEBUG */

#define MLX_CS_PIN    9

/* Extern	 variables ---------------------------------------------------------*/
//extern DriverType Driver;
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private	variables ---------------------------------------------------------*/

//static uint8_t posture[9];//Z,Y,X,T
//static uint8_t status  = 0;
//static uint32_t write[3] ;
//static uint32_t read[2] = {0};
static uint8_t write_buffer[10];

// int16_t dataTest1 = 0x0000; //
// uint16_t dataTest = 0x0000; //


/* Private	function pototype -------------------------------------------------*/


/**
 * @brief
 * @param  None
 * @retval
 */

static void Delay_Ms(rt_uint8_t time)
{
   rt_uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }	
}

void MLX90393_ReadPos(struct rt_spi_device *device , rt_uint8_t* pos )
{
	uint8_t statusByte;
//	static rt_int32_t pos = 0;	
	
	write_buffer[0] = (RM|X_AXIS|Y_AXIS|Z_AXIS|TEMPERATURE);
	rt_spi_send_then_recv(device, &write_buffer,1,&statusByte, 1);
	rt_spi_recv(device,pos,9);
//	for(int i = 0; i < 8; i++)
//	{
//        MPU_TRACE("pos[%d]:%d\r\n",i,posture[i]);
//	}	
//	MPU_TRACE("z_pos1:%d\t Y_pos2:%d\t x_pos3:%d\t temper:%d\r\n",
//						(uint16_t)(posture[0] | (posture[1]<<8)),
//						(uint16_t)(posture[2] | (posture[3]<<8)),
//						(uint16_t)(posture[4] | (posture[5]<<8)),
//						(uint16_t)(posture[6] | (posture[7]<<8)));	
}

//int a = 0;

rt_int32_t MLX90393_GetPosX(void)
{
//	return (posture[4] << 8 | posture[3]);
	  return 0;
}

rt_err_t mlx90393_init(const char * spi_device_name)
{
	uint8_t statusByte;
    rt_err_t result = RT_EOK;
    
    struct rt_spi_device*   rt_spi_device;

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        MPU_TRACE("mlx spi device %s not found!\r\n", spi_device_name);
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
	write_buffer[0] = (RT);
  rt_spi_transfer(rt_spi_device, &write_buffer,&statusByte, 1);

	Delay_Ms(1);
	
	write_buffer[0] = WR;
	write_buffer[1] = ((SPI_MODE_ONLY|TRIG_INT_SEL)&0xFF00)>>8;
	write_buffer[2] = ((SPI_MODE_ONLY|TRIG_INT_SEL)&0x00FF);
	write_buffer[3] = ADDRESS1  << 2;
//    rt_spi_send_then_recv(device, &write_buffer, 1, &statusByte, 1);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 4);

	Delay_Ms(1);
	
	write_buffer[0] = WR;
	write_buffer[1] = (((RES(0,0,0)) | (OSR(2)) | DIG_FILT(2))&0xFF00)>>8;
	write_buffer[2] = (((RES(0,0,0)) | (OSR(2)) | DIG_FILT(2))&0x00FF);
	write_buffer[3] = ADDRESS2  << 2;	
//    rt_spi_send_then_recv(device, &write_buffer, 1, &statusByte, 1);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 4);

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS7  << 2;	
//    rt_spi_send_then_recv(device, &write_buffer, 1, &statusByte, 1);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 4);

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS8  << 2;
//    rt_spi_send_then_recv(device, &write_buffer, 1, &statusByte, 1);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 4);

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS9  << 2;	
//    rt_spi_send_then_recv(device, &write_buffer, 1, &statusByte, 1);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 4);

	write_buffer[0] = (SB|X_AXIS|Y_AXIS|Z_AXIS|TEMPERATURE);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 1);


	write_buffer[0] = (SWOC|X_AXIS|Y_AXIS|Z_AXIS|TEMPERATURE);
  rt_spi_transfer(rt_spi_device, &write_buffer, &statusByte, 1);


//	MPU_TRACE("spi device ID:%ld !\r\n", id);

_error_exit:
    return result;
}

#define    VDD_3V3_SENSORS_EN_Pin    2

int rt_mlx90393_init(void)
{
	
    stm32_spi_bus_attach_device(MLX_CS_PIN, "spi1", "mlx90393_spi");
	rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
    return mlx90393_init("mlx90393_spi");
}
INIT_DEVICE_EXPORT(rt_mlx90393_init);


/***COPY RIGHT: ACTION 2019*******************************************************************************/

