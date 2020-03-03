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
 * File      : ekf2.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include "common.h"
#include "ekf2.h"
#include <drivers/spi.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/vehicle_attitude.h>


#define EKF2_DEBUG

#ifdef EKF2_DEBUG
#define EKF2_TRACE         rt_kprintf
#else
#define EKF2_TRACE(...)
#endif /* #ifdef EKF2_DEBUG */



static struct sensor_combined_s _sensors;
static struct sensor_mag_s _mag;
static struct sensor_baro_s _baro;
static struct vehicle_attitude_s _att;

static int _sensors_sub;
static int _mag_sub;
static int _baro_sub;
static int ekf2_timestamps_pub;
static orb_advert_t _att_pub;
		
static char thread_ekf2_stack[4096];
struct rt_thread thread_ekf2_handle;

extern "C"  rt_err_t ekf2_main(int argc, char *argv[]);

static rt_uint8_t ekf2_read_id(struct rt_spi_device *device)
{
    rt_uint8_t id_recv;


    return id_recv;
}

rt_err_t ekf2_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;

    return result;
}

int rt_ekf2_init(void)
{
	return 0;
}
INIT_DEVICE_EXPORT(rt_ekf2_init);


static void ekf2_measure()
{

}

bool publish_attitude(const sensor_combined_s &data, const hrt_abstime &now)
{

	// generate vehicle attitude quaternion data
	_att.timestamp = now;
	const Quatf q{_ekf.calculate_quaternion()};
	q.copyTo(_att.q);
	
	float roll = matrix::Eulerf(q).phi()*57.2f;
	float pitch =  matrix::Eulerf(q).theta()*57.2f;
	float yaw = matrix::Eulerf(q).psi()*57.2f;
	//LOG_RAW("roll:%f  pitch:%f   yaw:%f  \r\n",roll,pitch,yaw);

	//_ekf.get_quat_reset(&_att.delta_q_reset[0], &_att.quat_reset_counter);
	// In-run bias estimates
	float gyro_bias[3];
	_ekf.get_gyro_bias(gyro_bias);
	_att.rollspeed = data.gyro_rad[0] - gyro_bias[0];
	_att.pitchspeed = data.gyro_rad[1] - gyro_bias[1];
	_att.yawspeed = data.gyro_rad[2] - gyro_bias[2];
	
	//_att_pub.publish(_att);
	orb_publish(ORB_ID(vehicle_attitude),_att_pub,&_att);
	return true;
}

static void ekf2_config()
{
	_sensors_sub = orb_subscribe(ORB_ID(sensor_combined));
	_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	_att_pub=orb_advertise(ORB_ID(vehicle_attitude),&_att);
}

void ekf2_run(void * parameter)
{
	ekf2_config();
	
	while(1)
	{
		bool updated = RT_FALSE;    
		orb_check(_sensors_sub,&updated);
		if(updated)
		{
			bool imu_bias_reset_request = false;
			orb_copy(ORB_ID(sensor_combined), _sensors_sub, &_sensors);

			// ekf2_timestamps (using 0.1 ms relative timestamps)
			struct ekf2_timestamps_s ekf2_timestamps;
			ekf2_timestamps.timestamp = _sensors.timestamp;

			ekf2_timestamps.airspeed_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			ekf2_timestamps.distance_sensor_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			ekf2_timestamps.gps_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;
			ekf2_timestamps.optical_flow_timestamp_rel = ekf2_timestamps_s::RELATIVE_TIMESTAMP_INVALID;

			// attempt reset until successful
			if (imu_bias_reset_request) 
			{
				imu_bias_reset_request = !_ekf.reset_imu_bias();
			}

			const hrt_abstime now = hrt_absolute_time_us();

			// push imu data into estimator
			struct imuSample imu_sample_new;
			imu_sample_new.time_us = now;
			imu_sample_new.delta_ang_dt = _sensors.gyro_integral_dt * 1.e-6f;
			imu_sample_new.delta_ang = Vector3f{_sensors.gyro_rad} * imu_sample_new.delta_ang_dt;
			imu_sample_new.delta_vel_dt = _sensors.accelerometer_integral_dt * 1.e-6f;
			imu_sample_new.delta_vel = Vector3f{_sensors.accelerometer_m_s2} * CONSTANTS_ONE_G * imu_sample_new.delta_vel_dt;
			//LOG_RAW("gyro_integral_dt:%f accelerometer_integral_dt:%f \r\n",_sensors.gyro_integral_dt,_sensors.accelerometer_integral_dt);
			//EKF2_TRACE("gyro_integral_dt:%d  accelerometer_integral_dt:%d \r\n",_sensors.gyro_integral_dt*1000,_sensors.accelerometer_integral_dt*1000);
			
			_ekf.setIMUData(imu_sample_new);

			// publish attitude immediately (uses quaternion from output predictor)
			publish_attitude(_sensors, now);
			
			orb_check(_mag_sub,&updated);
			if (updated) {
				orb_copy(ORB_ID(sensor_mag), _mag_sub, &_mag);
				// Reset learned bias parameters if there has been a persistant change in magnetometer ID
				// Do not reset parmameters when armed to prevent potential time slips casued by parameter set
				// and notification events
				// Check if there has been a persistant change in magnetometer ID
				if (_mag.device_id != 0) {
					if (_invalid_mag_id_count < 200) {
						_invalid_mag_id_count++;
					}

				} else {
					if (_invalid_mag_id_count > 0) {
						_invalid_mag_id_count--;
					}
				}
				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_mag_time_sum_ms += _mag.timestamp / 1000;
				_mag_sample_count++;
				_mag_data_sum[0] += _mag.x;
				_mag_data_sum[1] += _mag.y;
				_mag_data_sum[2] += _mag.z;
				int32_t mag_time_ms = _mag_time_sum_ms / _mag_sample_count;

				if ((mag_time_ms - _mag_time_ms_last_used) > 5) {
					const float mag_sample_count_inv = 1.0f / _mag_sample_count;
					// calculate mean of measurements and correct for learned bias offsets
					float mag_data_avg_ga[3] = {_mag_data_sum[0] *mag_sample_count_inv ,
									_mag_data_sum[1] *mag_sample_count_inv ,
									_mag_data_sum[2] *mag_sample_count_inv 
								   };

					_ekf.setMagData(1000 * (uint64_t)mag_time_ms, mag_data_avg_ga);

					_mag_time_ms_last_used = mag_time_ms;
					_mag_time_sum_ms = 0;
					_mag_sample_count = 0;
					_mag_data_sum[0] = 0.0f;
					_mag_data_sum[1] = 0.0f;
					_mag_data_sum[2] = 0.0f;
				}
			}

			// read baro data
			orb_check(_baro_sub,&updated);
			if (updated) {
				orb_copy(ORB_ID(sensor_baro), _baro_sub, &_baro);
				// If the time last used by the EKF is less than specified, then accumulate the
				// data and push the average when the specified interval is reached.
				_balt_time_sum_ms += _baro.timestamp / 1000;
				_balt_sample_count++;
				_balt_data_sum += _baro.altitude;
				uint32_t balt_time_ms = _balt_time_sum_ms / _balt_sample_count;

				if (balt_time_ms - _balt_time_ms_last_used > 5) {
					// take mean across sample period
					float balt_data_avg = _balt_data_sum / (float)_balt_sample_count;

					float pressure_to_density = 1.0f / (CONSTANTS_AIR_GAS_CONST * (20.0f -
					CONSTANTS_ABSOLUTE_NULL_CELSIUS));
					float rho = pressure_to_density * _baro.pressure;
					_ekf.set_air_density(rho);

//					// calculate static pressure error = Pmeas - Ptruth
//					// model position error sensitivity as a body fixed ellipse with a different scale in the positive and
//					// negative X and Y directions
//					const Vector3f vel_body_wind = get_vel_body_wind();

//					float K_pstatic_coef_x;

//					if (vel_body_wind(0) >= 0.0f) {
//						K_pstatic_coef_x = _param_ekf2_pcoef_xp.get();

//					} else {
//						K_pstatic_coef_x = _param_ekf2_pcoef_xn.get();
//					}

//					float K_pstatic_coef_y;

//					if (vel_body_wind(1) >= 0.0f) {
//						K_pstatic_coef_y = _param_ekf2_pcoef_yp.get();

//					} else {
//						K_pstatic_coef_y = _param_ekf2_pcoef_yn.get();
//					}

//					const float max_airspeed_sq = _param_ekf2_aspd_max.get() * _param_ekf2_aspd_max.get();
//					const float x_v2 = fminf(vel_body_wind(0) * vel_body_wind(0), max_airspeed_sq);
//					const float y_v2 = fminf(vel_body_wind(1) * vel_body_wind(1), max_airspeed_sq);
//					const float z_v2 = fminf(vel_body_wind(2) * vel_body_wind(2), max_airspeed_sq);

//					const float pstatic_err = 0.5f * rho * (
//									  K_pstatic_coef_x * x_v2 + K_pstatic_coef_y * y_v2 + _param_ekf2_pcoef_z.get() * z_v2);

//					// correct baro measurement using pressure error estimate and assuming sea level gravity
//					balt_data_avg += pstatic_err / (rho * CONSTANTS_ONE_G);

					// push to estimator
					_ekf.setBaroData(1000 * (uint64_t)balt_time_ms, balt_data_avg);

					_balt_time_ms_last_used = balt_time_ms;
					_balt_time_sum_ms = 0;
					_balt_sample_count = 0;
					_balt_data_sum = 0.0f;
				}				
			}

			_ekf.update();

			// integrate time to monitor time slippage
			if (_start_time_us == 0) 
			{
				_start_time_us = now;
				_last_time_slip_us = 0;

			} else if (_start_time_us > 0) 
			{
				_integrated_time_us += _sensors.gyro_integral_dt;
				_last_time_slip_us = (now - _start_time_us) - _integrated_time_us;
			}

		}
		rt_thread_mdelay(1);
	}	
	
}


void ekf2_start(void)
{
	
		rt_err_t res;

		/* create thread */
		res = rt_thread_init(&thread_ekf2_handle,
						   "ekf2",
						   ekf2_run,
						   RT_NULL,
						   &thread_ekf2_stack[0],
						   sizeof(thread_ekf2_stack),EKF2_THREAD_PRIORITY,1);
		if (res == RT_EOK)
			rt_thread_startup(&thread_ekf2_handle);	
}


rt_err_t ekf2_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			ekf2_start();
		}

		if (!strcmp(verb, "stop")) {
			//ekf2_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//ekf2_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//ekf2_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//ekf2_info(busid);
		}
	}
	

	//ekf2_usage();
	return 0;
}

MSH_CMD_EXPORT(ekf2_main,ekf2)