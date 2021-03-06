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
 * File      : mag_calibration.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include "mag_calibration.h"
#include "math.h"

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>


static const char *sensor_name = "mag";
static const unsigned int max_mags = 1;
static float mag_sphere_radius = 0.2f;
static unsigned int calibration_sides = 6;			///< The total number of sides
static unsigned int calibration_total_points = 240;		///< The total points per magnetometer
static unsigned int calibraton_duration_seconds = 42; 	///< The total duration the routine is allowed to take

static  float MAG_MAX_OFFSET_LEN =	1.3f;	///< The maximum measurement range is ~1.9 Ga, the earth field is ~0.6 Ga, so an offset larger than ~1.3 Ga means the mag will saturate in some directions.

int32_t	device_ids[max_mags];
bool internal[max_mags];
int device_prio_max = 0;
int32_t device_id_primary = 0;
static unsigned _last_mag_progress = 0;

enum calibrate_return mag_calibrate_all(void);

/// Data passed to calibration worker routine
typedef struct  {
	orb_advert_t	*mavlink_log_pub;
	unsigned	done_count;
	int		sub_mag[max_mags];
	unsigned int	calibration_points_perside;
	unsigned int	calibration_interval_perside_seconds;
	uint64_t	calibration_interval_perside_useconds;
	unsigned int	calibration_counter_total[max_mags];
	bool		side_data_collected[detect_orientation_side_count];
	float		*x[max_mags];
	float		*y[max_mags];
	float		*z[max_mags];
} mag_worker_data_t;


int do_mag_calibration(void)
{

	struct calibration_s mscale_null;
	mscale_null.x_offset = 0.0f;
	mscale_null.x_scale = 1.0f;
	mscale_null.y_offset = 0.0f;
	mscale_null.y_scale = 1.0f;
	mscale_null.z_offset = 0.0f;
	mscale_null.z_scale = 1.0f;

	int result = RT_EOK;

	// Determine which mags are available and reset each

	char str[30];


	for (size_t i = 0; i < max_mags; i++) {
		device_ids[i] = 0; // signals no mag
	}

	_last_mag_progress = 0;

	// Calibrate all mags at the same time
	if (result == RT_EOK) {
		switch (mag_calibrate_all()) {
		case calibrate_return_cancelled:
			// Cancel message already displayed, we're done here
			result = RT_ERROR;
			break;

		case calibrate_return_ok:
			/* if there is a any preflight-check system response, let the barrage of messages through */
			time_waitUs(200000);
			time_waitUs(20000);
			time_waitUs(20000);
			break;

		default:
			result = RT_ERROR;
			time_waitUs(20000);
			break;
		}
	}

	/* give this message enough time to propagate */
	time_waitUs(600000);

	return result;
}

static bool reject_sample(float sx, float sy, float sz, float x[], float y[], float z[], unsigned count,
			  unsigned max_count)
{
	float min_sample_dist = fabsf(5.4f * mag_sphere_radius / sqrtf(max_count)) / 3.0f;

	for (size_t i = 0; i < count; i++) {
		float dx = sx - x[i];
		float dy = sy - y[i];
		float dz = sz - z[i];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);

		if (dist < min_sample_dist) {
			return true;
		}
	}

	return false;
}

static unsigned progress_percentage(mag_worker_data_t *worker_data)
{
	return 100 * ((float)worker_data->done_count) / calibration_sides;
}

// Returns calibrate_return_error if any parameter is not finite
// Logs if parameters are out of range
static enum calibrate_return check_calibration_result(float offset_x, float offset_y, float offset_z,
		float sphere_radius,
		float diag_x, float diag_y, float diag_z,
		float offdiag_x, float offdiag_y, float offdiag_z,
			size_t cur_mag)
{
	float must_be_finite[] = {offset_x, offset_y, offset_z,
				  sphere_radius,
				  diag_x, diag_y, diag_z,
				  offdiag_x, offdiag_y, offdiag_z
				 };

	float should_be_not_huge[] = {offset_x, offset_y, offset_z};
	float should_be_positive[] = {sphere_radius, diag_x, diag_y, diag_z};

	// Make sure every parameter is finite
	const int num_finite = sizeof(must_be_finite) / sizeof(*must_be_finite);

	for (unsigned i = 0; i < num_finite; ++i) {
		if (!isfinite(must_be_finite[i])) {

			return calibrate_return_error;
		}
	}

	// Notify if offsets are too large
	const int num_not_huge = sizeof(should_be_not_huge) / sizeof(*should_be_not_huge);

	for (unsigned i = 0; i < num_not_huge; ++i) {
		if (fabsf(should_be_not_huge[i]) > MAG_MAX_OFFSET_LEN) {
			break;
		}
	}

	// Notify if a parameter which should be positive is non-positive
	const int num_positive = sizeof(should_be_positive) / sizeof(*should_be_positive);

	for (unsigned i = 0; i < num_positive; ++i) {
		if (should_be_positive[i] <= 0.0f) {

			break;
		}
	}

	return calibrate_return_ok;
}

static enum calibrate_return mag_calibration_worker(enum detect_orientation_return orientation, int cancel_sub, void *data)
{
	enum calibrate_return result = calibrate_return_ok;

	unsigned int calibration_counter_side;

	mag_worker_data_t *worker_data = (mag_worker_data_t *)(data);

	/*
	 * Detect if the system is rotating.
	 *
	 * We're detecting this as a general rotation on any axis, not necessary on the one we
	 * asked the user for. This is because we really just need two roughly orthogonal axes
	 * for a good result, so we're not constraining the user more than we have to.
	 */

	hrt_abstime detection_deadline = hrt_absolute_time_us() + worker_data->calibration_interval_perside_useconds * 5;
	hrt_abstime last_gyro = 0;
	float gyro_x_integral = 0.0f;
	float gyro_y_integral = 0.0f;
	float gyro_z_integral = 0.0f;

	const float gyro_int_thresh_rad = 0.5f;

	int sub_gyro = orb_subscribe(ORB_ID(sensor_gyro));

	while (fabsf(gyro_x_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_y_integral) < gyro_int_thresh_rad &&
	       fabsf(gyro_z_integral) < gyro_int_thresh_rad) {


		/* abort with timeout */
		if (hrt_absolute_time_us() > detection_deadline) {
			result = calibrate_return_error;
			break;
		}


		struct sensor_gyro_s gyro;
		bool updata;
		orb_check(sub_gyro,&updata);
		if(updata)
		{
			orb_copy(ORB_ID(sensor_gyro), sub_gyro, &gyro);

			/* ensure we have a valid first timestamp */
			if (last_gyro > 0) {

				/* integrate */
				float delta_t = (gyro.timestamp - last_gyro) / 1e6f;
				gyro_x_integral += gyro.x * delta_t;
				gyro_y_integral += gyro.y * delta_t;
				gyro_z_integral += gyro.z * delta_t;
			}

			last_gyro = gyro.timestamp;
		}
		
		
		
	}
	orb_unsubscribe(sub_gyro);

	uint64_t calibration_deadline = hrt_absolute_time_us() + worker_data->calibration_interval_perside_useconds;
	unsigned poll_errcount = 0;

	calibration_counter_side = 0;

	while (hrt_absolute_time_us() < calibration_deadline &&
	       calibration_counter_side < worker_data->calibration_points_perside) {

			int prev_count[max_mags];
			bool rejected = false;

			for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {

				prev_count[cur_mag] = worker_data->calibration_counter_total[cur_mag];

				if (worker_data->sub_mag[cur_mag] >= 0) {
					struct sensor_mag_s mag;

					orb_copy(ORB_ID(sensor_mag), worker_data->sub_mag[cur_mag], &mag);

					// Check if this measurement is good to go in
					rejected = rejected || reject_sample(mag.x, mag.y, mag.z,
									     worker_data->x[cur_mag], worker_data->y[cur_mag], worker_data->z[cur_mag],
									     worker_data->calibration_counter_total[cur_mag],
									     calibration_sides * worker_data->calibration_points_perside);

					worker_data->x[cur_mag][worker_data->calibration_counter_total[cur_mag]] = mag.x;
					worker_data->y[cur_mag][worker_data->calibration_counter_total[cur_mag]] = mag.y;
					worker_data->z[cur_mag][worker_data->calibration_counter_total[cur_mag]] = mag.z;
					worker_data->calibration_counter_total[cur_mag]++;
				}
			}

			// Keep calibration of all mags in lockstep
			if (rejected) {
				// Reset counts, since one of the mags rejected the measurement
				for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {
					worker_data->calibration_counter_total[cur_mag] = prev_count[cur_mag];
				}

			} else {
				calibration_counter_side++;

				unsigned new_progress = progress_percentage(worker_data) +
							(unsigned)((100 / calibration_sides) * ((float)calibration_counter_side / (float)
									worker_data->calibration_points_perside));

				if (new_progress - _last_mag_progress > 3) {
					// Progress indicator for side

					time_waitUs(20000);

					_last_mag_progress = new_progress;
				}
			}



		if (poll_errcount > worker_data->calibration_points_perside * 3) {
			result = calibrate_return_error;
			break;
		}
	}

	if (result == calibrate_return_ok) {

		worker_data->done_count++;
		time_waitUs(20000);
	}

	return result;
}





enum calibrate_return mag_calibrate_all(void)
{
	enum calibrate_return result = calibrate_return_ok;

	mag_worker_data_t worker_data;

	worker_data.done_count = 0;
	worker_data.calibration_points_perside = calibration_total_points / calibration_sides;
	worker_data.calibration_interval_perside_seconds = calibraton_duration_seconds / calibration_sides;
	worker_data.calibration_interval_perside_useconds = worker_data.calibration_interval_perside_seconds * 1000 * 1000;


	calibration_sides = 0;
	int32_t cal_mask = (1 << 6) - 1;
	for (unsigned int i = 0; i < (sizeof(worker_data.side_data_collected) / sizeof(worker_data.side_data_collected[0])); i++) {

		if ((cal_mask & (1 << i)) > 0) {
			// mark as missing
			worker_data.side_data_collected[i] = false;
			calibration_sides++;

		} else {
			// mark as completed from the beginning
			worker_data.side_data_collected[i] = true;

			time_waitUs(100000);
		}
	}

	for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {
		// Initialize to no subscription
		worker_data.sub_mag[cur_mag] = -1;

		// Initialize to no memory allocated
		worker_data.x[cur_mag] = 0;
		worker_data.y[cur_mag] = 0;
		worker_data.z[cur_mag] = 0;
		worker_data.calibration_counter_total[cur_mag] = 0;
	}

	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;

	char str[30];

	// Get actual mag count and alloate only as much memory as needed
	const unsigned orb_mag_count = 1;//orb_group_count(ORB_ID(sensor_mag));

	// Warn that we will not calibrate more than max_mags magnetometers
	if (orb_mag_count > max_mags) 
	{
	}

	for (size_t cur_mag = 0; cur_mag < orb_mag_count && cur_mag < max_mags; cur_mag++) {
		worker_data.x[cur_mag] = (malloc(sizeof(float) * calibration_points_maxcount));
		worker_data.y[cur_mag] = (malloc(sizeof(float) * calibration_points_maxcount));
		worker_data.z[cur_mag] = (malloc(sizeof(float) * calibration_points_maxcount));

		if (worker_data.x[cur_mag] == 0 || worker_data.y[cur_mag] == 0 || worker_data.z[cur_mag] == 0) 
		{
			result = calibrate_return_error;
		}
	}


	// Setup subscriptions to mag sensors
	if (result == calibrate_return_ok) {

		// We should not try to subscribe if the topic doesn't actually exist and can be counted.
		for (unsigned cur_mag = 0; cur_mag < orb_mag_count && cur_mag < max_mags; cur_mag++) 
		{

			// Lock in to correct ORB instance
			bool found_cur_mag = false;

			for (unsigned i = 0; i < orb_mag_count && !found_cur_mag; i++) 
			{
				worker_data.sub_mag[cur_mag] = orb_subscribe(ORB_ID(sensor_mag));

				struct sensor_mag_s report;
				orb_copy(ORB_ID(sensor_mag), worker_data.sub_mag[cur_mag], &report);


				// For the DriverFramework drivers, we fill device ID (this is the first time) by copying one report.
				device_ids[cur_mag] = report.device_id;
				found_cur_mag = true;
			}

			if (!found_cur_mag) {
				result = calibrate_return_error;
				break;
			}

			if (device_ids[cur_mag] != 0) {
				// Get priority
				int32_t prio;
				orb_priority(worker_data.sub_mag[cur_mag], &prio);

				if (prio > device_prio_max) {
					device_prio_max = prio;
					device_id_primary = device_ids[cur_mag];
				}

			} else {
				result = calibrate_return_error;
				break;
			}
		}
	}

	// Limit update rate to get equally spaced measurements over time (in ms)
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available
				unsigned int orb_interval_msecs = (worker_data.calibration_interval_perside_useconds / 1000) /
								  worker_data.calibration_points_perside;

				//calibration_log_info(mavlink_log_pub, "Orb interval %u msecs", orb_interval_msecs);
				orb_set_interval(worker_data.sub_mag[cur_mag], orb_interval_msecs);
			}
		}

	}

	if (result == calibrate_return_ok) {
		int cancel_sub  = calibrate_cancel_subscribe();

		result = calibrate_from_orientation(
						    cancel_sub,                         // Subscription to vehicle_command for cancel support
						    worker_data.side_data_collected,    // Sides to calibrate
						    mag_calibration_worker,             // Calibration worker
						    &worker_data,			// Opaque data for calibration worked
						    true);				// true: lenient still detection
		calibrate_cancel_unsubscribe(cancel_sub);
	}

	// Close subscriptions
	for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
		if (worker_data.sub_mag[cur_mag] >= 0) {
			//px4_close(worker_data.sub_mag[cur_mag]);
		}
	}

	// Calculate calibration values for each mag

	float sphere_x[max_mags];
	float sphere_y[max_mags];
	float sphere_z[max_mags];
	float sphere_radius[max_mags];
	float diag_x[max_mags];
	float diag_y[max_mags];
	float diag_z[max_mags];
	float offdiag_x[max_mags];
	float offdiag_y[max_mags];
	float offdiag_z[max_mags];

	for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
		sphere_x[cur_mag] = 0.0f;
		sphere_y[cur_mag] = 0.0f;
		sphere_z[cur_mag] = 0.0f;
		sphere_radius[cur_mag] = 0.2f;
		diag_x[cur_mag] = 1.0f;
		diag_y[cur_mag] = 1.0f;
		diag_z[cur_mag] = 1.0f;
		offdiag_x[cur_mag] = 0.0f;
		offdiag_y[cur_mag] = 0.0f;
		offdiag_z[cur_mag] = 0.0f;
	}

	// Sphere fit the data to get calibration values
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available and we should have values for it to calibrate

				ellipsoid_fit_least_squares(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
							    worker_data.calibration_counter_total[cur_mag],
							    100, 0.0f,
							    &sphere_x[cur_mag], &sphere_y[cur_mag], &sphere_z[cur_mag],
							    &sphere_radius[cur_mag],
							    &diag_x[cur_mag], &diag_y[cur_mag], &diag_z[cur_mag],
							    &offdiag_x[cur_mag], &offdiag_y[cur_mag], &offdiag_z[cur_mag]);

				result = check_calibration_result(sphere_x[cur_mag], sphere_y[cur_mag], sphere_z[cur_mag],
								  sphere_radius[cur_mag],
								  diag_x[cur_mag], diag_y[cur_mag], diag_z[cur_mag],
								  offdiag_x[cur_mag], offdiag_y[cur_mag], offdiag_z[cur_mag],
								  cur_mag);

				if (result == calibrate_return_error) {
					break;
				}
			}
		}
	}

	// Print uncalibrated data points
	if (result == calibrate_return_ok) {

		
	}

	// Data points are no longer needed
	for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {
		free(worker_data.x[cur_mag]);
		free(worker_data.y[cur_mag]);
		free(worker_data.z[cur_mag]);
	}

	if (result == calibrate_return_ok) {

		for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				struct calibration_s mscale;
				mscale.x_scale = 1.0;
				mscale.y_scale = 1.0;
				mscale.z_scale = 1.0;



				if (result == calibrate_return_ok) {
					mscale.x_offset = sphere_x[cur_mag];
					mscale.y_offset = sphere_y[cur_mag];
					mscale.z_offset = sphere_z[cur_mag];
					mscale.x_scale = diag_x[cur_mag];
					mscale.y_scale = diag_y[cur_mag];
					mscale.z_scale = diag_z[cur_mag];
          LOG_RAW("mscale.x_offset:%f \n",mscale.x_offset);
					LOG_RAW("mscale.y_offset:%f \n",mscale.y_offset);
					LOG_RAW("mscale.z_offset:%f \n",mscale.z_offset);
					LOG_RAW("mscale.x_scale:%f \n",mscale.x_scale);
					LOG_RAW("mscale.y_scale:%f \n",mscale.y_scale);
					LOG_RAW("mscale.z_scale:%f \n",mscale.z_scale);

					
				}

				if (result == calibrate_return_ok) {
					bool failed = false;
					if (failed) {
						result = calibrate_return_error;

					} else {

						time_waitUs(200000);
					}
				}
			}
		}

	}

	return result;
}

void mag_cali_start(void)
{
	int res;
	res = do_mag_calibration();
	if(res == RT_EOK)
	{
		rt_kprintf("mag calibration successful !");
	}
}
rt_err_t mag_cali(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			mag_cali_start();
		}

	}
	return 0;
}

MSH_CMD_EXPORT(mag_cali,mag calibration)
