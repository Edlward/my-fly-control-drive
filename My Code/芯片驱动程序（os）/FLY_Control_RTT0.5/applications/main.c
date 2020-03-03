/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>
#include <board.h>


#include "bmi08x.h"
#include "bmi088.h"
#include "bmi088_stm32.h"

#include "mlx90393.h"
#include "spi_ms5611.h"
#include "spi_imu_mpu9250.h"
#include "drv_spi.h"

#include "finsh.h"
#include <drivers/mtd_nand.h>

#define    VDD_3V3_SENSORS_EN_Pin    2
#define    LED1                      3
#define    LED2                      4

extern struct bmi08x_dev bmi088_dev;


static struct rt_thread led0_thread;//
static struct rt_thread led1_thread;//
static struct rt_thread recive_thread;//
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rt_led0_thread_stack[528];//
static rt_uint8_t rt_led1_thread_stack[528];//
static rt_uint8_t rt_recive_thread_stack[528];//


static void led0_thread_entry(void* parameter);
static void led1_thread_entry(void* parameter);
static void recive_thread_entry(void* parameter);






static void led0_thread_entry(void* parameter)
{
//	RT_Device_SPI1_BUS
	rt_device_t rt_device_spi = RT_NULL;
//	rt_err_t rt_err = RT_EOK;
	rt_pin_mode(LED1, PIN_MODE_OUTPUT);

	rt_device_spi = rt_device_find("spi1");
	if(RT_NULL == rt_device_spi)
		rt_kprintf("not register\r\n");
	else
		rt_kprintf("register successful:%d\r\n",rt_device_spi);
	
	while(1)
	{
		//LED2=0;                                 //??: F7 ???????, LED ?????????
		rt_pin_write(LED1, PIN_HIGH);
		rt_thread_delay(RT_TICK_PER_SECOND/5); //??
		//LED2=1;
		rt_pin_write(LED1, PIN_LOW);
		rt_thread_delay(RT_TICK_PER_SECOND/5); //??
	}
}

static void led1_thread_entry(void* parameter)
{

//	rt_err_t rt_err = RT_EOK;
	rt_int32_t temper=0;
	rt_int32_t pressure=0;
	rt_uint8_t posture[9]={0};
	rt_uint8_t i=0;
	
	rt_int16_t xMag,yMag,zMag;
	float tMag;
//	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* ? ? ? ? */
	struct rt_spi_device* rt_device_spi = RT_NULL;
	struct rt_spi_device* rt_device_spi2 = RT_NULL;
//	char str[] = "hello RT-Thread!\r\n";
	
	rt_device_spi = (struct rt_spi_device *)rt_device_find("ms5611_spi");
	if(!rt_device_spi)
		rt_kprintf("ms5611 not register\r\n");

	rt_device_spi2 = (struct rt_spi_device *)rt_device_find("mlx90393_spi");
	if(!rt_device_spi2)
		rt_kprintf("mlx90393_spi not register\r\n");
	while(1)
	{
		MS5611_Cal(rt_device_spi,&temper,&pressure);
//		rt_kprintf("temper:%d \t pressure:%d \r\n",temper,pressure);
		
		MLX90393_ReadPos(rt_device_spi2  , posture);

//		i = 1;  //ÆðÊ¼Î»
//		tMag = (uint16_t)(posture[i+1] | (posture[i+0]<<8));
//		xMag = (uint16_t)(posture[i+3] | (posture[i+2]<<8));
//		yMag = (uint16_t)(posture[i+5] | (posture[i+4]<<8));
//		zMag = (uint16_t)(posture[i+7] | (posture[i+6]<<8));
////	  tMag = (posture[2] + (posture[1]*256));
////		xMag = (posture[6] + (posture[5]*256));
////		yMag = (posture[4] + (posture[3]*256));
//		zMag = (posture[8] + (posture[7]*256)); 
//		if(xMag > 32767)
//		{
//			xMag -= 65536;
//		}
//		if(yMag > 32767)
//		{
//			yMag -= 65536;
//		}
//		if(zMag > 32767)
//		{
//			zMag -= 65536;
//		}
//	  tMag=(tMag - 46244)/45.2 + 25;
//		rt_kprintf("temper:%d \t x_pos3:%d\r\n",tMag,xMag);
//		rt_kprintf("x_pos3:%d\r\n",xMag);
//		rt_kprintf("Y_pos2:%d \t z_pos1:%d \r\n",yMag,zMag);
		for(i = 0; i < 9; i++)
		{
			rt_kprintf("pos[%d]:%d\r\n",i,posture[i]);
		}	
		
		rt_thread_delay(RT_TICK_PER_SECOND/2);
	}
}

static void recive_thread_entry(void* parameter)
{
//	RT_Device_SPI1_BUS
	
	int8_t err=0;
	
	rt_int16_t acc[3]={0};
	rt_int16_t gyr[3]={0};
	rt_int16_t mag[3]={0};
	rt_int64_t temper=0;
	int temper_rec=0.0;
	struct rt_spi_device* rt_device_spi = RT_NULL;

	struct bmi08x_sensor_data user_accel_bmi088;
	struct bmi08x_sensor_data user_gyro_bmi088;	
	
//    rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
//	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
	
	
	rt_device_spi = (struct rt_spi_device *)rt_device_find("mpu_spi");
	if(!rt_device_spi)
		rt_kprintf("not register\r\n");
	
	while(1)
	{

	   MPU9250_GetTemperatureRawData(rt_device_spi,&temper);
	   temper_rec=((temper-21)/333.87)+21;
	   rt_kprintf("temper: %d \r\n",temper_rec);	   

//	   MPU9250_Get3AxisAccelRawData(rt_device_spi,acc);
//	   rt_kprintf("Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",acc[0],acc[1],acc[2]);
//	   MPU9250_Get3AxisGyroRawData(rt_device_spi,gyr);
//	   rt_kprintf("Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",gyr[0],gyr[1],gyr[2]);
//	   MPU9250_Get3AxisMagnetRawData(rt_device_spi,mag);
//	   rt_kprintf("Magnet1:%d\t Magnet2:%d\t Magnet3:%d\r\n",mag[0],mag[1],mag[2]);		
	   
	   MPU9250_Get9AxisRawData(rt_device_spi,acc,gyr,mag);
		 rt_kprintf("Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",acc[0],acc[1],acc[2]);
		 rt_kprintf("Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",gyr[0],gyr[1],gyr[2]);
		 rt_kprintf("Magnet1:%d\t Magnet2:%d\t Magnet3:%d\r\n",mag[0],mag[1],mag[2]);
		
	  err=bmi088_get_synchronized_data(&user_accel_bmi088,&user_gyro_bmi088,&bmi088_dev);
	  if(err == BMI08X_OK)
	  {
		  rt_kprintf("BIM Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",
										 user_accel_bmi088.x ,
										 user_accel_bmi088.y ,
										 user_accel_bmi088.z );
		  rt_kprintf("BIM Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",
										 user_gyro_bmi088.x ,
										 user_gyro_bmi088.y ,
										 user_gyro_bmi088.z );
	  }		 		
		
		
		 rt_thread_delay(RT_TICK_PER_SECOND/2); //
		
	}
}

int main(void)
{
    /* user app entry */
	rt_thread_init(&led0_thread,                    //?????
					"led0",                         //????,? shell ??????
					led0_thread_entry,              //??????
					RT_NULL,                        //????????
					&rt_led0_thread_stack[0],       //???????
					sizeof(rt_led0_thread_stack),   //?????
					3,                              //??????
					20);                            //?????					
	rt_thread_startup(&led0_thread);                //???? led0_thread,????
	
	rt_thread_init(&led1_thread,                    //?????
					"led1",                         //????,? shell ??????
					led1_thread_entry,              //??????
					RT_NULL,                        //????????
					&rt_led1_thread_stack[0],       //???????
					sizeof(rt_led1_thread_stack),   //?????
					3,                              //??????
					20);
	rt_thread_startup(&led1_thread);                //???? led1_thread,????  
	
	rt_thread_init(&recive_thread,                   //?????
					"recive",                         //????,? shell ??????
					recive_thread_entry,              //??????
					RT_NULL,                         //????????
					&rt_recive_thread_stack[0],        //???????
					sizeof(rt_recive_thread_stack),   //?????
					4,                               //??????
					20);
	rt_thread_startup(&recive_thread);                //???? led1_thread,????  
  					
    return 0;
}
