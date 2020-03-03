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
 * File      : accel_calibration.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
/**
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up
 *                        | -g  0  0 |     // nose down
 *                        |  0  g  0 |     // left side down
 *                        |  0 -g  0 |     // right side down
 *                        |  0  0  g |     // on back
 *                        [  0  0 -g ]     // level
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
 * g = 9.80665
 *
 * ===== Rotation =====
 *
 * Calibrating using model:
 * accel_corr = accel_T_r * (rot * accel_raw - accel_offs_r)
 *
 * Actual correction:
 * accel_corr = rot * accel_T * (accel_raw - accel_offs)
 *
 * Known: accel_T_r, accel_offs_r, rot
 * Unknown: accel_T, accel_offs
 *
 * Solution:
 * accel_T_r * (rot * accel_raw - accel_offs_r) = rot * accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * (rot * accel_raw - accel_offs_r) = accel_T * (accel_raw - accel_offs)
 * rot^-1 * accel_T_r * rot * accel_raw - rot^-1 * accel_T_r * accel_offs_r = accel_T * accel_raw - accel_T * accel_offs)
 * => accel_T = rot^-1 * accel_T_r * rot
 * => accel_offs = rot^-1 * accel_offs_r
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

// FIXME: Can some of these headers move out with detect_ move?

#include "accel_calibration.h"
#include "math.h"

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>

#define FLT_EPSILON 1.1920929e-07F  /* 1E-5 */
#define CONSTANTS_ONE_G						9.80665f				// m/s^2

static const char *sensor_name = "accel";

static int device_prio_max = 0;
static int32_t device_id_primary = 0;


/// Data passed to calibration worker routine
typedef struct  {
	unsigned	done_count;
	int		subs;
	float		accel_ref[detect_orientation_side_count][3];
	int		sensor_correction_sub;
} accel_worker_data_t;

int mat_invert3(float src[3][3], float dst[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
		    src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
		    src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return RT_ERROR;        // Singular matrix
	}
	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return RT_EOK;
}

enum calibrate_return calculate_calibration_values(
		float accel_ref[][3], float accel_T[][3],
		float accel_offs[], float g)
{
	/* calculate offsets */
	for (unsigned i = 0; i < 3; i++) {
		accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	memset(mat_A, 0, sizeof(mat_A));

	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			float a = accel_ref[i * 2][j] - accel_offs[j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != RT_EOK) {
		return calibrate_return_error;
	}

	/* copy results to accel_T */
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[j][i] = mat_A_inv[j][i] * g;
		}
	}

	return calibrate_return_ok;
}

/*
 * Read specified number of accelerometer samples, calculate average and dispersion.
 */
enum calibrate_return read_accelerometer_avg(int sensor_correction_sub, int subs,
					float accel_avg[][3], unsigned orient, unsigned samples_num)
{
	/* get total sensor board rotation matrix */
	unsigned counts = 0;
	float accel_sum[3];
	memset(accel_sum, 0, sizeof(accel_sum));

	unsigned errcount = 0;
	struct sensor_correction_s sensor_correction; /**< sensor thermal corrections */

	/* try to get latest thermal corrections */
	if (orb_copy(ORB_ID(sensor_correction), sensor_correction_sub, &sensor_correction) != 0) {
		/* use default values */
		memset(&sensor_correction, 0, sizeof(sensor_correction));

		for (unsigned i = 0; i < 3; i++) {
			sensor_correction.accel_scale_0[i] = 1.0f;
			sensor_correction.accel_scale_1[i] = 1.0f;
			sensor_correction.accel_scale_2[i] = 1.0f;
		}
	}

	/* use the first sensor to pace the readout, but do per-sensor counts */
	while (counts < samples_num) 
	{
		bool changed;
		orb_check(subs, &changed);
		if (changed) 
		{
			struct sensor_accel_s arp;
			orb_copy(ORB_ID(sensor_accel), subs, &arp);
			accel_sum[0] += arp.x;
			accel_sum[1] += arp.y;
			accel_sum[2] += arp.z;
			counts++;
		}
		else 
		{
			errcount++;
			continue;
		}

		if (errcount > samples_num / 10) {
			return calibrate_return_error;
		}
	}

	for (unsigned i = 0; i < 3; i++) {
		accel_avg[orient][i] = accel_sum[i] / counts;
	}
	return calibrate_return_ok;
}

static enum calibrate_return accel_calibration_worker(enum detect_orientation_return orientation, int cancel_sub, void *data)
{
	const unsigned samples_num = 750;
	accel_worker_data_t *worker_data = (accel_worker_data_t *)(data);


	read_accelerometer_avg(worker_data->sensor_correction_sub, worker_data->subs, worker_data->accel_ref, orientation,
			       samples_num);


	worker_data->done_count++;
	return calibrate_return_ok;
}

enum detect_orientation_return detect_orientation(int cancel_sub, int accel_sub,bool lenient_still_position)
{
	float		accel_ema[3] = { 0.0f };		// exponential moving average of accel
	float		accel_disp[3] = { 0.0f, 0.0f, 0.0f };	// max-hold dispersion of accel
	static  float		ema_len = 0.5f;				// EMA time constant in seconds
	static  float	normal_still_thr = 0.25;		// normal still threshold
	float		still_thr2 = powf(lenient_still_position ? (normal_still_thr * 3) : normal_still_thr, 2);
	static  float		accel_err_thr = 5.0f;			// set accel error threshold to 5m/s^2
	const hrt_abstime	still_time = lenient_still_position ? 500000 : 1300000;	// still time required in us
	const hrt_abstime t_start = hrt_absolute_time_us();

	/* set timeout to 30s */
	static hrt_abstime timeout = 90000000;

	hrt_abstime t_timeout = t_start + timeout;
	hrt_abstime t = t_start;
	hrt_abstime t_prev = t_start;
	hrt_abstime t_still = 0;

	unsigned poll_errcount = 0;

	while (true) 
	{
		/* wait blocking for new data */
		struct sensor_combined_s sensor;
		bool	update;
		orb_check(accel_sub, &update);
		if(update)
		{
			orb_copy(ORB_ID(sensor_combined), accel_sub, &sensor);
			t = hrt_absolute_time_us();
			float dt = (t - t_prev) / 1000000.0f;
			t_prev = t;
			float w = dt / ema_len;

			for (unsigned i = 0; i < 3; i++) {

				float di = sensor.accelerometer_m_s2[i];

				float d = di - accel_ema[i];
				accel_ema[i] += d * w;
				d = d * d;
				accel_disp[i] = accel_disp[i] * (1.0f - w);

				if (d > still_thr2 * 8.0f) {
					d = still_thr2 * 8.0f;
				}

				if (d > accel_disp[i]) {
					accel_disp[i] = d;
				}
			}

			/* still detector with hysteresis */
			if (accel_disp[0] < still_thr2 &&
			    accel_disp[1] < still_thr2 &&
			    accel_disp[2] < still_thr2) 
			{
				/* is still now */
				if (t_still == 0) 
				{
					/* first time */
					t_still = t;
					t_timeout = t + timeout;

				} else
				{
					/* still since t_still */
					if (t > t_still + still_time) 
					{
						/* vehicle is still, exit from the loop to detection of its orientation */
						break;
					}
				}

			} else if (accel_disp[0] > still_thr2 * 4.0f ||
				   accel_disp[1] > still_thr2 * 4.0f ||
				   accel_disp[2] > still_thr2 * 4.0f) 
			{
				/* not still, reset still start time */
				if (t_still != 0) {
					time_waitUs(200000);
					t_still = 0;
				}
			}
		}

		if (t > t_timeout) 
		{
			poll_errcount++;
		}

		if (poll_errcount > 1000) 
		{
			return DETECT_ORIENTATION_ERROR;
		}
	}

	if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) 
	{
		return DETECT_ORIENTATION_TAIL_DOWN;        // [ g, 0, 0 ]
	}

	if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr)
	{
		return DETECT_ORIENTATION_NOSE_DOWN;        // [ -g, 0, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) 
	{
		return DETECT_ORIENTATION_LEFT;        // [ 0, g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
	    fabsf(accel_ema[2]) < accel_err_thr) 
	{
		return DETECT_ORIENTATION_RIGHT;        // [ 0, -g, 0 ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) 
	{
		return DETECT_ORIENTATION_UPSIDE_DOWN;        // [ 0, 0, g ]
	}

	if (fabsf(accel_ema[0]) < accel_err_thr &&
	    fabsf(accel_ema[1]) < accel_err_thr &&
	    fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) 
	{
		return DETECT_ORIENTATION_RIGHTSIDE_UP;        // [ 0, 0, -g ]
	}


	return DETECT_ORIENTATION_ERROR;	// Can't detect orientation
}


enum calibrate_return calibrate_from_orientation(
		int		cancel_sub,
		bool	side_data_collected[detect_orientation_side_count],
		calibration_from_orientation_worker_t calibration_worker,
		void	*worker_data,
		bool	lenient_still_position)
{
	enum calibrate_return result = calibrate_return_ok;

	// Setup subscriptions to onboard accel sensor

	int sub_accel = orb_subscribe(ORB_ID(sensor_combined));


	unsigned orientation_failures = 0;

	// Rotate through all requested orientation
	while (true) {

		unsigned int side_complete_count = 0;

		// Update the number of completed sides
		for (unsigned i = 0; i < detect_orientation_side_count; i++) {
			if (side_data_collected[i]) {
				side_complete_count++;
			}
		}

		if (side_complete_count == detect_orientation_side_count) {
			// We have completed all sides, move on
			break;
		}
		
		enum detect_orientation_return orient = detect_orientation(cancel_sub, sub_accel,
							lenient_still_position);

		// Call worker routine
		result = calibration_worker(orient, cancel_sub, worker_data);

		if (result != calibrate_return_ok) {
			break;
		}


		// Note that this side is complete
		side_data_collected[orient] = true;



		// temporary priority boost for the white blinking led to come trough

	}


	return result;
}

int calibrate_cancel_subscribe()
{
	int vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));

	if (vehicle_command_sub >= 0) {
		// make sure we won't read any old messages
		struct vehicle_command_s cmd;
		bool update;

		while (orb_check(vehicle_command_sub, &update) == 0 && update) {
			orb_copy(ORB_ID(vehicle_command), vehicle_command_sub, &cmd);
		}
	}

	return vehicle_command_sub;
}

void calibrate_cancel_unsubscribe(int cmd_sub)
{
	orb_unsubscribe(cmd_sub);
}

enum calibrate_return do_accel_calibration_measurements(float *accel_offs, float accel_T[3][3])
{
	enum calibrate_return result = calibrate_return_ok;

	accel_worker_data_t worker_data;
	worker_data.done_count = 0;

	bool data_collected[detect_orientation_side_count] = { false, false, false, false, false, false };

	// Initialise sub to sensor thermal compensation data
	worker_data.sensor_correction_sub = orb_subscribe(ORB_ID(sensor_correction));

	// Initialize subs to error condition so we know which ones are open and which are not
	worker_data.subs = -1;


	uint64_t timestamps = {0};
	worker_data.subs = orb_subscribe(ORB_ID(sensor_accel));
	struct sensor_accel_s report = {0};
	orb_copy(ORB_ID(sensor_accel), worker_data.subs, &report);

	if (result == calibrate_return_ok) {
		int cancel_sub = calibrate_cancel_subscribe();
		result = calibrate_from_orientation(cancel_sub, data_collected, accel_calibration_worker, &worker_data,
						    false /* normal still */);
		calibrate_cancel_unsubscribe(cancel_sub);
	}
	orb_unsubscribe(worker_data.sensor_correction_sub);

	result = calculate_calibration_values(worker_data.accel_ref, accel_T, accel_offs, CONSTANTS_ONE_G);



	return result;
}



int do_accel_calibration(void)
{
	int res;
	struct accel_calibration_s accel_scale;
	accel_scale.x_offset = 0.0f;
	accel_scale.x_scale = 1.0f;
	accel_scale.y_offset = 0.0f;
	accel_scale.y_scale = 1.0f;
	accel_scale.z_offset = 0.0f;
	accel_scale.z_scale = 1.0f;

	float accel_offs[3];
	float accel_T[3][3];

	/* measure and calculate offsets & scales */
	enum calibrate_return cal_return = do_accel_calibration_measurements(accel_offs, accel_T);

	if (cal_return == calibrate_return_cancelled) {
		// Cancel message already displayed, nothing left to do
		return RT_ERROR;

	} else if (cal_return == calibrate_return_ok) {
		res = RT_EOK;

	} else {
		res = RT_ERROR;
	}


	accel_scale.x_offset = accel_offs[0];
	accel_scale.x_scale = accel_T[0][0];
	accel_scale.y_offset = accel_offs[1];
	accel_scale.y_scale = accel_T[1][1];
	accel_scale.z_offset = accel_offs[2];
	accel_scale.z_scale = accel_T[2][2];

	/* give this message enough time to propagate */
	time_waitUs(600000);

	return res;
}
void accel_cali_start(void)
{
	int res;
	res = do_accel_calibration();
	if(res == RT_EOK)
	{
		rt_kprintf("accel calibration successful !");
	}
}
rt_err_t accel_cali(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			accel_cali_start();
		}

	}
	//bmi088_usage();
	return 0;
}

MSH_CMD_EXPORT(accel_cali,accel calibration)
