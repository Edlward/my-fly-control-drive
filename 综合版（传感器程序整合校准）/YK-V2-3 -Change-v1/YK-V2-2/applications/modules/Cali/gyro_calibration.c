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
 * File      : gyro_calibration.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include "gyro_calibration.h"
#include "math.h"

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>

static const unsigned max_gyros = 1;

/// Data passed to calibration worker routine
typedef struct  {
	orb_advert_t		*mavlink_log_pub;
	int32_t			device_id[max_gyros];
	int			gyro_sensor_sub[max_gyros];
	int			sensor_correction_sub;
	struct calibration_s	gyro_scale[max_gyros];
	struct sensor_gyro_s	gyro_report_0;
} gyro_worker_data_t;

static enum calibrate_return gyro_calibration_worker(void *data)
{
	gyro_worker_data_t	*worker_data = (gyro_worker_data_t *)(data);
	unsigned		calibration_counter[max_gyros] = { 0 }, slow_count = 0;
	const unsigned		calibration_count = 5000;
	struct sensor_gyro_s	gyro_report;

	struct sensor_correction_s sensor_correction; /**< sensor thermal corrections */

//	if (orb_copy(ORB_ID(sensor_correction), worker_data->sensor_correction_sub, &sensor_correction) != 0) {
		/* use default values */
		memset(&sensor_correction, 0, sizeof(sensor_correction));

		for (unsigned i = 0; i < 3; i++) {
			sensor_correction.gyro_scale_0[i] = 1.0f;
			sensor_correction.gyro_scale_1[i] = 1.0f;
			sensor_correction.gyro_scale_2[i] = 1.0f;
		}
//	}


	memset(&worker_data->gyro_report_0, 0, sizeof(worker_data->gyro_report_0));


	/* use slowest gyro to pace, but count correctly per-gyro for statistics */
	while (slow_count < calibration_count) {

		/* check if there are new thermal corrections */
		bool updated;
//		orb_check(worker_data->sensor_correction_sub, &updated);

//		if (updated) {
//			orb_copy(ORB_ID(sensor_correction), worker_data->sensor_correction_sub, &sensor_correction);
//		}

			unsigned update_count = calibration_count;

			for (unsigned s = 0; s < max_gyros; s++) {
				if (calibration_counter[s] >= calibration_count) {
					// Skip if instance has enough samples
					continue;
				}

				bool changed;
				orb_check(worker_data->gyro_sensor_sub[s], &changed);

				if (changed) {
					orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &gyro_report);

					if (s == 0) {
						// take a working copy
						worker_data->gyro_scale[s].x_offset += (gyro_report.x - sensor_correction.gyro_offset_0[0]) *
										       sensor_correction.gyro_scale_0[0];
						worker_data->gyro_scale[s].y_offset += (gyro_report.y - sensor_correction.gyro_offset_0[1]) *
										       sensor_correction.gyro_scale_0[1];
						worker_data->gyro_scale[s].z_offset += (gyro_report.z - sensor_correction.gyro_offset_0[2]) *
										       sensor_correction.gyro_scale_0[2];

						// take a reference copy of the primary sensor including correction for thermal drift
						orb_copy(ORB_ID(sensor_gyro), worker_data->gyro_sensor_sub[s], &worker_data->gyro_report_0);
						worker_data->gyro_report_0.x = (gyro_report.x - sensor_correction.gyro_offset_0[0]) * sensor_correction.gyro_scale_0[0];
						worker_data->gyro_report_0.y = (gyro_report.y - sensor_correction.gyro_offset_0[1]) * sensor_correction.gyro_scale_0[1];
						worker_data->gyro_report_0.z = (gyro_report.z - sensor_correction.gyro_offset_0[2]) * sensor_correction.gyro_scale_0[2];

					} else if (s == 1) {
						worker_data->gyro_scale[s].x_offset += (gyro_report.x - sensor_correction.gyro_offset_1[0]) *
										       sensor_correction.gyro_scale_1[0];
						worker_data->gyro_scale[s].y_offset += (gyro_report.y - sensor_correction.gyro_offset_1[1]) *
										       sensor_correction.gyro_scale_1[1];
						worker_data->gyro_scale[s].z_offset += (gyro_report.z - sensor_correction.gyro_offset_1[2]) *
										       sensor_correction.gyro_scale_1[2];

					} else if (s == 2) {
						worker_data->gyro_scale[s].x_offset += (gyro_report.x - sensor_correction.gyro_offset_2[0]) *
										       sensor_correction.gyro_scale_2[0];
						worker_data->gyro_scale[s].y_offset += (gyro_report.y - sensor_correction.gyro_offset_2[1]) *
										       sensor_correction.gyro_scale_2[1];
						worker_data->gyro_scale[s].z_offset += (gyro_report.z - sensor_correction.gyro_offset_2[2]) *
										       sensor_correction.gyro_scale_2[2];

					} else {
						worker_data->gyro_scale[s].x_offset += gyro_report.x;
						worker_data->gyro_scale[s].y_offset += gyro_report.y;
						worker_data->gyro_scale[s].z_offset += gyro_report.z;

					}

					calibration_counter[s]++;

				}

				// Maintain the sample count of the slowest sensor
				if (calibration_counter[s] && calibration_counter[s] < update_count) {
					update_count = calibration_counter[s];
				}

			}
			// Propagate out the slowest sensor's count
			if (slow_count < update_count) {
				slow_count = update_count;
			}

	}
	for (unsigned s = 0; s < max_gyros; s++) 
	{
		if (worker_data->device_id[s] != 0 && calibration_counter[s] < calibration_count / 2) 
		{
			return calibrate_return_error;
		}
		worker_data->gyro_scale[s].x_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].y_offset /= calibration_counter[s];
		worker_data->gyro_scale[s].z_offset /= calibration_counter[s];
	}

	return calibrate_return_ok;

}
int do_gyro_calibration(void)
{
	int			res = RT_EOK;
	gyro_worker_data_t	worker_data;

	struct calibration_s gyro_scale_zero;
	gyro_scale_zero.x_offset = 0.0f;
	gyro_scale_zero.x_scale = 1.0f;
	gyro_scale_zero.y_offset = 0.0f;
	gyro_scale_zero.y_scale = 1.0f;
	gyro_scale_zero.z_offset = 0.0f;
	gyro_scale_zero.z_scale = 1.0f;

	int device_prio_max = 0;
	int32_t device_id_primary = 0;

	worker_data.sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	for (unsigned s = 0; s < max_gyros; s++) {
		char str[30];

		// Reset gyro ids to unavailable.
		worker_data.device_id[s] = 0;
		// And set default subscriber values.
		worker_data.gyro_sensor_sub[s] = -1;

		// Reset all offsets to 0 and scales to 1
		(void)memcpy(&worker_data.gyro_scale[s], &gyro_scale_zero, sizeof(gyro_scale_zero));

	}


	for (unsigned cur_gyro = 0; cur_gyro < max_gyros; cur_gyro++) {

		// Lock in to correct ORB instance
		bool found_cur_gyro = false;

		worker_data.gyro_sensor_sub[cur_gyro] = orb_subscribe(ORB_ID(sensor_gyro));

		struct sensor_gyro_s report;
		orb_copy(ORB_ID(sensor_gyro), worker_data.gyro_sensor_sub[cur_gyro], &report);


		// For the DriverFramework drivers, we fill device ID (this is the first time) by copying one report.
		worker_data.device_id[cur_gyro] = report.device_id;
		found_cur_gyro = true;

	}

	unsigned try_count = 0;
	unsigned max_tries = 20;
	res = RT_ERROR;

	do {
		// Calibrate gyro and ensure user didn't move
		enum calibrate_return cal_return = gyro_calibration_worker(&worker_data);

		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already sent, we are done here
			LOG_RAW("Cancel ensure gyro didn't move\n");
			res = RT_ERROR;
			break;

		} else if (cal_return == calibrate_return_error) {
			LOG_RAW("ensure gyro didn't move fail\n");
			res = RT_ERROR;

		} else {
			/* check offsets */
			float xdiff = worker_data.gyro_report_0.x - worker_data.gyro_scale[0].x_offset;
			float ydiff = worker_data.gyro_report_0.y - worker_data.gyro_scale[0].y_offset;
			float zdiff = worker_data.gyro_report_0.z - worker_data.gyro_scale[0].z_offset;

			/* maximum allowable calibration error in radians */
			const float maxoff = 0.01f;

			if (fabsf(xdiff) > maxoff ||
			    fabsf(ydiff) > maxoff ||
			    fabsf(zdiff) > maxoff) {

				res = RT_ERROR;

			} else {
				LOG_RAW("x_offset:%f \t y_offset:%f \t z_offset:%f \t \n",worker_data.gyro_scale[0].x_offset
				                                                         ,worker_data.gyro_scale[0].y_offset
				                                                         ,worker_data.gyro_scale[0].z_offset);
				res = RT_EOK;
			}
		}

		try_count++;

	} while (res == RT_ERROR && try_count <= max_tries);

	if (try_count >= max_tries) {
    LOG_RAW("try count over\n");
		res = RT_ERROR;
	}



	/* if there is a any preflight-check system response, let the barrage of messages through */
	time_waitUs(200000);

	orb_unsubscribe(worker_data.sensor_correction_sub);

	/* give this message enough time to propagate */
	time_waitUs(600000);

	return res;
}

void gyro_cali_start(void)
{
	int res;
	res = do_gyro_calibration();
	if(res == RT_EOK)
	{
		rt_kprintf("gyro calibration successful !\n");
	}
	else 
	{
		rt_kprintf("gyro calibration fail!\n");
	}
}
rt_err_t gyro_cali(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			gyro_cali_start();
		}

	}
	return 0;
}

MSH_CMD_EXPORT(gyro_cali,gyro calibration)
