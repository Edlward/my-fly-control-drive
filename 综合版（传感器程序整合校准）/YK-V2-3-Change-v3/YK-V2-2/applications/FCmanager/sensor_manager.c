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
 * File      : sensor_manager.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
 
#include "common.h"
#include "math.h"
#include <bmi088.h>
#include <ms5611.h>
#include <mlx90393.h>
#include <drivers/spi.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_attitude.h>


#define SENSOR_DEBUG

#ifdef SENSOR_DEBUG
#define SENSOR_TRACE         rt_kprintf
#else
#define SENSOR_TRACE(...)
#endif /* #ifdef SENSOR_DEBUG */

#define EVENT_SENSOR		(1<<0)

#define CONSTANTS_ABSOLUTE_NULL_CELSIUS		-273.15f				// ¡ãC
#define CONSTANTS_AIR_GAS_CONST 			 287.1f					// J/(kg * K)
#define CONSTANTS_ONE_G						9.80665f				// m/s^2

static struct sensor_combined_s		_sensors;
static struct sensor_baro_s			_baro;
static struct sensor_accel_s		_accel;
static struct sensor_gyro_s			_gyro;
static struct sensor_mag_s			_mag;

static orb_advert_t 			_sensors_pub;
static orb_advert_t 			_baro_pub;
static orb_advert_t 			_accel_pub;
static orb_advert_t 			_gyro_pub;
static orb_advert_t 			_mag_pub;
	
static struct rt_timer 			timer_sensor_updata;
static struct rt_event 			event_sensor_collect;
static char thread_sensor_stack[2048];
struct rt_thread thread_sensor_handle;



static void sensor_collect_update(void* parameter)
{
	rt_event_send(&event_sensor_collect, EVENT_SENSOR);
}

void sensor_collect(void)
{
	static hrt_abstime last_acc_gro_time;
	struct bmi08x_sensor_data accel;
	struct bmi08x_sensor_data gyro;
	struct mlx_txyz_s mlx90393_data;
	hrt_abstime now = hrt_absolute_time_us();
	if(sensor_bmi088_ready()){
		if(sensor_get_bmi088_data(&accel,&gyro) == RT_EOK){			
			//LOG_RAW("_accelx:%d  _accely:%d   _accelz:%d  \n",accel.x,accel.y,accel.z);
			hrt_abstime bmi088_time = hrt_absolute_time_us();
			_sensors.accelerometer_timestamp_relative = bmi088_time;
			_accel.device_id = BMI088_ACCEL_ID;
			_accel.timestamp = bmi088_time;
			_accel.x_raw = accel.x;
			_accel.y_raw = accel.y;
			_accel.z_raw = accel.z;			
			_sensors.accelerometer_m_s2[0] = (float)accel.x/32768.0f*3;
			_sensors.accelerometer_m_s2[1] = (float)accel.y/32768.0f*3;
			_sensors.accelerometer_m_s2[2] = (float)accel.z/32768.0f*3;
			_accel.x = _sensors.accelerometer_m_s2[0];
			_accel.y = _sensors.accelerometer_m_s2[1];
			_accel.z = _sensors.accelerometer_m_s2[2];
			_sensors.accelerometer_integral_dt = now - last_acc_gro_time;
			
			
			//LOG_RAW("_accelx:%f  _accely:%f   _accelz:%f  \n",sensors.accelerometer_m_s2[0],sensors.accelerometer_m_s2[1],sensors.accelerometer_m_s2[2]);
			_gyro.device_id = BMI088_GYRO_ID;
			_gyro.timestamp = bmi088_time;
			_gyro.x_raw = gyro.x;
			_gyro.y_raw = gyro.y;
			_gyro.z_raw = gyro.z;
			
			_sensors.gyro_rad[0] = (float)gyro.x*15.3f/1000.0f/57.2f;
			_sensors.gyro_rad[1] = (float)gyro.y*15.3f/1000.0f/57.2f;
			_sensors.gyro_rad[2] = (float)gyro.z*15.3f/1000.0f/57.2f;
			_gyro.x = _sensors.gyro_rad[0];
			_gyro.y = _sensors.gyro_rad[1];
			_gyro.z = _sensors.gyro_rad[2];
			_sensors.gyro_integral_dt = now - last_acc_gro_time;
			last_acc_gro_time = now;			
			//LOG_RAW("_gyrox:%d  _gyroy:%d   _gyroz:%d  \n",gyro.x,gyro.y,gyro.z);
			orb_publish(ORB_ID(sensor_accel),_accel_pub,&_accel);
			orb_publish(ORB_ID(sensor_gyro),_gyro_pub,&_gyro);			
			//LOG_RAW("_gyrox:%f  _gyroy:%f   _gyroz:%f  \n",sensors.gyro_rad[0],sensors.gyro_rad[1],sensors.gyro_rad[2]);
		}
	}

	if(sensor_mlx90393_ready()){
		if(sensor_get_mlx90393_data(&mlx90393_data) == RT_EOK){	
			hrt_abstime mlx90393_time = hrt_absolute_time_us();
			_mag.x = mlx90393_data.x;
			_mag.y = mlx90393_data.y;
			_mag.z = mlx90393_data.z;
			_mag.temperature = mlx90393_data.t;	
			_mag.timestamp = mlx90393_time;
			_mag.device_id = MLX90393_MAG_ID;
			_sensors.magnetometer_ga[0] = _mag.x;		
			_sensors.magnetometer_ga[1] = _mag.y;
			_sensors.magnetometer_ga[2] = _mag.z;
			_sensors.magnetometer_timestamp_relative = mlx90393_time;
			//LOG_RAW("time:%d \n",mlx90393_time);
			//LOG_RAW("time:%lld  _mag.t:%f  _mag.x:%f  _mag.y:%f   _mag.z:%f  \n",mlx90393_time,_mag.temperature,_mag.x,_mag.y,_mag.z);
			orb_publish(ORB_ID(sensor_mag),_mag_pub,&_mag);
		}
	}
//	
//	if(sensor_mpu9250_ready()){
//		if(sensor_get_mpu9250_data()){
//			
//			
//		}
//	}
//	
	if(sensor_ms5611_ready()){
		if(sensor_get_ms5611_data(&_baro.temperature,&_baro.pressure)==RT_EOK){
			hrt_abstime baro_time = hrt_absolute_time_us();
			// calculate altitude using the hypsometric equation
			static float T1 = 15.0f - CONSTANTS_ABSOLUTE_NULL_CELSIUS;	/* temperature at base height in Kelvin */
			static float a  = -6.5f / 1000.0f;	/* temperature gradient in degrees per metre */
			/* current pressure at MSL in kPa (QNH in hPa)*/
			float p1 = 1013.25f * 0.1f;

			/* measured pressure in kPa */
			float p = _baro.pressure *100.0f* 0.001f;

			/*
			 * Solve:
			 *
			 *     /        -(aR / g)     \
			 *    | (p / p1)          . T1 | - T1
			 *     \                      /
			 * h = -------------------------------  + h1
			 *                   a
			 */
			_sensors.baro_alt_meter = (((powf((p / p1), (-(a * CONSTANTS_AIR_GAS_CONST) / CONSTANTS_ONE_G))) * T1) - T1) / a;
			_sensors.baro_timestamp_relative = baro_time;
			_sensors.baro_temp_celcius = _baro.temperature;
			_baro.timestamp = baro_time;
			_baro.altitude = _sensors.baro_alt_meter;
			_baro.device_id = MS5611_BARO_ID;
			//LOG_RAW("time:%d temp:%f  pres:%f   alt:%f  \n",baro_time,_baro.temperature,_baro.pressure,_baro.altitude);
			orb_publish(ORB_ID(sensor_baro),_baro_pub,&_baro);
		}
	}
	_sensors.timestamp = now; 
	orb_publish(ORB_ID(sensor_combined),_sensors_pub,&_sensors);
}
void sensor_run(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_SENSOR;
	
	_sensors_pub=orb_advertise(ORB_ID(sensor_combined),&_sensors);
	_baro_pub=orb_advertise(ORB_ID(sensor_baro),&_baro);
	_accel_pub=orb_advertise(ORB_ID(sensor_accel),&_accel);
	_gyro_pub=orb_advertise(ORB_ID(sensor_gyro),&_gyro);
	_mag_pub=orb_advertise(ORB_ID(sensor_mag),&_mag);
	
	/* create event */
	res = rt_event_init(&event_sensor_collect, "sensorupdata", RT_IPC_FLAG_FIFO);

	/* register timer event */
	rt_timer_init(&timer_sensor_updata, "sensor_collect",
					sensor_collect_update,
					RT_NULL,
					1,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_sensor_updata);
	
	while(1)
	{
		res = rt_event_recv(&event_sensor_collect, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK){
			if(recv_set & EVENT_SENSOR){
				sensor_collect();
			}
		}
	}
}

void sensor_start(void)
{
	rt_err_t res;

		/* create thread */
	res = rt_thread_init(&thread_sensor_handle,
						   "sensor",
						   sensor_run,
						   RT_NULL,
						   &thread_sensor_stack[0],
						   sizeof(thread_sensor_stack),SENSOR_THREAD_PRIORITY,1);
	if (res == RT_EOK)
		rt_thread_startup(&thread_sensor_handle);
}


rt_err_t sensor_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			sensor_start();
		}

		if (!strcmp(verb, "stop")) {
			//sensor_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//sensor_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//sensor_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//sensor_info(busid);
		}
	}
	

	//sensor_usage();
	return 0;
}

MSH_CMD_EXPORT(sensor_main,sensor)