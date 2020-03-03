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
//#include <uORB/uORB.h>
#include "drivers/pin.h"
#include "device/spi_fram_fm25vxx.h"
#include "device/mpu9250.h"
#include "device/bmi088.h"
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_gps_position.h>


#define LOG_TAG              "example"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>

#define LED1_PIN 3

static rt_device_t serial;
static rt_device_t dev ;
int main(void)
{
	int sensor_accel_sub,sensor_gyro_sub,vehicle_gps_sub;
	struct sensor_accel_s _accel;
	struct sensor_gyro_s  _gyro;
	struct vehicle_gps_position_s  _vehicle_gps;

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
	
	vehicle_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));	
	while(1)
	{
		bool updated = RT_FALSE;    
        orb_check(sensor_accel_sub,&updated);
        if(updated)
		{
			orb_copy(ORB_ID(sensor_accel),sensor_accel_sub,&_accel);
			rt_device_write(dev, 0, &_accel, sizeof(_accel));
			LOG_RAW("_accel_x:%f  _accel_y:%f   _accel_z:%f  \n",_accel.x,_accel.y,_accel.z);
		}
		orb_check(sensor_gyro_sub,&updated);
        if(updated)
		{
			orb_copy(ORB_ID(sensor_gyro),sensor_gyro_sub,&_gyro);
			rt_device_write(dev, 0, &_gyro, sizeof(_gyro));
			LOG_RAW("_gyrox:%f  _gyroy:%f   _gyroz:%f  \n",_gyro.x,_gyro.y,_gyro.z);
		}		

		orb_check(vehicle_gps_sub,&updated);
        if(updated)
		{
			orb_copy(ORB_ID(vehicle_gps_position),vehicle_gps_sub,&_vehicle_gps);
			rt_device_write(dev, 0, &_vehicle_gps, sizeof(_vehicle_gps));
//			LOG_RAW("_alt:%f  _lon:%f   _lat:%f  num:%d\r\n",_vehicle_gps.alt,_vehicle_gps.lon,_vehicle_gps.lat,_vehicle_gps.satellites_used);
			LOG_RAW("_alt:%d  _lon:%d   _lat:%d  num:%d\r\n",_vehicle_gps.alt,_vehicle_gps.lon,_vehicle_gps.lat,_vehicle_gps.satellites_used);

		}	
		
		rt_pin_write(LED1_PIN,!rt_pin_read(LED1_PIN));
		rt_thread_mdelay(1000);
	}

    return 0;
}
