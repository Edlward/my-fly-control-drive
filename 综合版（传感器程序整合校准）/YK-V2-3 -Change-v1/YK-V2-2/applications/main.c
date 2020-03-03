/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include "common.h"
#include <board.h>
#include "math.h"
//#include <uORB/uORB.h>
#include "drivers/pin.h"
#include "device/spi_fram_fm25vxx.h"
#include "device/mpu9250.h"
#include "device/bmi088.h"
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_combined.h>


#define LED1_PIN 3

static rt_device_t serial;
static rt_device_t dev ;
int main(void)
{
	int sensor_accel_sub,sensor_gyro_sub;
	struct sensor_accel_s _accel;
	struct sensor_gyro_s  _gyro;
	rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
	
	char buf[] = "hello rt-thread!\r\n";

    dev = rt_device_find("vcom");
    
    if (dev)
        rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    else
        return -RT_ERROR;
    

	rt_thread_mdelay(10000);
	
	sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	int sub_accel = orb_subscribe(ORB_ID(sensor_combined));
	while(1)
	{
		hrt_abstime now = hrt_absolute_time_us();
		bool updated = RT_FALSE;    
        orb_check(sensor_accel_sub,&updated);
        if(updated)
		{
			orb_copy(ORB_ID(sensor_accel),sensor_accel_sub,&_accel);
			rt_device_write(dev, 0, &_accel, sizeof(_accel));
			float roll = atan2(_accel.x, sqrt(_accel.z*_accel.z + _accel.y*_accel.y))*57.2f;
			float pitch = atan2(_accel.y,sqrt(_accel.z*_accel.z + _accel.x*_accel.x))*57.2f;
			//LOG_RAW("roll:%f  pitch:%f \n",roll,pitch);
			//LOG_RAW("_accel_x:%f  _accel_y:%f   _accel_z:%f  \n",_accel.x,_accel.y,_accel.z);
		}
		orb_check(sensor_gyro_sub,&updated);
        if(updated)
		{
			orb_copy(ORB_ID(sensor_gyro),sensor_gyro_sub,&_gyro);
			rt_device_write(dev, 0, &_gyro, sizeof(_gyro));
			//LOG_RAW("_gyrox:%f  _gyroy:%f   _gyroz:%f  \n",_gyro.x,_gyro.y,_gyro.z);
		}	

//    orb_check(sub_accel, &updated);
//        if(updated)
//		{

//			LOG_RAW("accel_sub update \n");
//		}			

		
		rt_pin_write(LED1_PIN,!rt_pin_read(LED1_PIN));
		rt_thread_mdelay(100);
	}

    return 0;
}
