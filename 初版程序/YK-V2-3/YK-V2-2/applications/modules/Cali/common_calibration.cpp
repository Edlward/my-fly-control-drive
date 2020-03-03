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
 * File      : common_calibration.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

// FIXME: Can some of these headers move out with detect_ move?

#include "common_calibration.h"
#include <matrix/math.hpp>
#include "math/mathlib.h"
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>

#define CONSTANTS_ONE_G						9.80665f				// m/s^2

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
	static hrt_abstime timeout = 10000000;

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


int run_lm_sphere_fit(float *x, float *y, float *z, float _fitness, float _sphere_lambda,
		      unsigned int size, float *offset_x, float *offset_y, float *offset_z,
		      float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	//Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float _samples_collected = size;
	float fitness = _fitness;
	float fit1 = 0.0f, fit2 = 0.0f;

	matrix::SquareMatrix<float, 4> JTJ;
	matrix::SquareMatrix<float, 4> JTJ2;
	float JTFI[4] = {};
	float residual = 0.0f;

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < _samples_collected; k++) {

		float sphere_jacob[4];
		//Calculate Jacobian
		float A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(A * A + B * B + C * C);

		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.0f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
		sphere_jacob[2] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
		sphere_jacob[3] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
		residual = *sphere_radius - length;

		for (uint8_t i = 0; i < 4; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 4; j++) {
				JTJ(i, j) += sphere_jacob[i] * sphere_jacob[j];
				JTJ2(i, j) += sphere_jacob[i] * sphere_jacob[j]; //a backup JTJ for LM
			}

			JTFI[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	//refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[4] = {*sphere_radius, *offset_x, *offset_y, *offset_z};
	float fit2_params[4];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 4; i++) {
		JTJ(i, i) += _sphere_lambda;
		JTJ2(i, i) += _sphere_lambda / lma_damping;
	}

	if (!JTJ.I(JTJ)) {
		return -1;
	}

	if (!JTJ2.I(JTJ2)) {
		return -1;
	}

	for (uint8_t row = 0; row < 4; row++) {
		for (uint8_t col = 0; col < 4; col++) {
			fit1_params[row] -= JTFI[col] * JTJ(row, col);
			fit2_params[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	//Calculate mean squared residuals
	for (uint16_t k = 0; k < _samples_collected; k++) {
		float A = (*diag_x    * (x[k] - fit1_params[1])) + (*offdiag_x * (y[k] - fit1_params[2])) + (*offdiag_y *
				(z[k] + fit1_params[3]));
		float B = (*offdiag_x * (x[k] - fit1_params[1])) + (*diag_y    * (y[k] - fit1_params[2])) + (*offdiag_z *
				(z[k] + fit1_params[3]));
		float C = (*offdiag_y * (x[k] - fit1_params[1])) + (*offdiag_z * (y[k] - fit1_params[2])) + (*diag_z    *
				(z[k] - fit1_params[3]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = fit1_params[0] - length;
		fit1 += residual * residual;

		A = (*diag_x    * (x[k] - fit2_params[1])) + (*offdiag_x * (y[k] - fit2_params[2])) + (*offdiag_y *
				(z[k] - fit2_params[3]));
		B = (*offdiag_x * (x[k] - fit2_params[1])) + (*diag_y    * (y[k] - fit2_params[2])) + (*offdiag_z *
				(z[k] - fit2_params[3]));
		C = (*offdiag_y * (x[k] - fit2_params[1])) + (*offdiag_z * (y[k] - fit2_params[2])) + (*diag_z    *
				(z[k] - fit2_params[3]));
		length = sqrtf(A * A + B * B + C * C);
		residual = fit2_params[0] - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / _samples_collected;
	fit2 = sqrtf(fit2) / _samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

	if (isfinite(fitness) && fitness < _fitness) {
		_fitness = fitness;
		*sphere_radius = fit1_params[0];
		*offset_x = fit1_params[1];
		*offset_y = fit1_params[2];
		*offset_z = fit1_params[3];
		return 0;

	} else {
		return -1;
	}
}


int run_lm_ellipsoid_fit(float *x, float *y, float *z, float _fitness, float _sphere_lambda,
			 unsigned int size, float *offset_x, float *offset_y, float *offset_z,
			 float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	//Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float _samples_collected = size;
	float fitness = _fitness;
	float fit1 = 0.0f;
	float fit2 = 0.0f;

	float JTJ[81] = {};
	float JTJ2[81] = {};
	float JTFI[9] = {};
	float residual = 0.0f;
	float ellipsoid_jacob[9];

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < _samples_collected; k++) {

		//Calculate Jacobian
		float A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit1 += residual * residual;
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
		ellipsoid_jacob[1] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
		ellipsoid_jacob[2] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.0f * ((x[k] - *offset_x) * A) / length;
		ellipsoid_jacob[4] = -1.0f * ((y[k] - *offset_y) * B) / length;
		ellipsoid_jacob[5] = -1.0f * ((z[k] - *offset_z) * C) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.0f * (((y[k] - *offset_y) * A) + ((x[k] - *offset_x) * B)) / length;
		ellipsoid_jacob[7] = -1.0f * (((z[k] - *offset_z) * A) + ((x[k] - *offset_x) * C)) / length;
		ellipsoid_jacob[8] = -1.0f * (((z[k] - *offset_z) * B) + ((y[k] - *offset_y) * C)) / length;

		for (uint8_t i = 0; i < 9; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 9; j++) {
				JTJ[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
				JTJ2[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j]; //a backup JTJ for LM
			}

			JTFI[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	//refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[9] = {*offset_x, *offset_y, *offset_z, *diag_x, *diag_y, *diag_z, *offdiag_x, *offdiag_y, *offdiag_z};
	float fit2_params[9];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 9; i++) {
		JTJ[i * 9 + i] += _sphere_lambda;
		JTJ2[i * 9 + i] += _sphere_lambda / lma_damping;
	}


	if (!mat_inverse(JTJ, JTJ, 9)) {
		return -1;
	}

	if (!mat_inverse(JTJ2, JTJ2, 9)) {
		return -1;
	}

	for (uint8_t row = 0; row < 9; row++) {
		for (uint8_t col = 0; col < 9; col++) {
			fit1_params[row] -= JTFI[col] * JTJ[row * 9 + col];
			fit2_params[row] -= JTFI[col] * JTJ2[row * 9 + col];
		}
	}

	//Calculate mean squared residuals
	for (uint16_t k = 0; k < _samples_collected; k++) {
		float A = (fit1_params[3]    * (x[k] - fit1_params[0])) + (fit1_params[6] * (y[k] - fit1_params[1])) + (fit1_params[7] *
				(z[k] - fit1_params[2]));
		float B = (fit1_params[6] * (x[k] - fit1_params[0])) + (fit1_params[4]   * (y[k] - fit1_params[1])) + (fit1_params[8] *
				(z[k] - fit1_params[2]));
		float C = (fit1_params[7] * (x[k] - fit1_params[0])) + (fit1_params[8] * (y[k] - fit1_params[1])) + (fit1_params[5]    *
				(z[k] - fit1_params[2]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit1 += residual * residual;

		A = (fit2_params[3]    * (x[k] - fit2_params[0])) + (fit2_params[6] * (y[k] - fit2_params[1])) + (fit2_params[7] *
				(z[k] - fit2_params[2]));
		B = (fit2_params[6] * (x[k] - fit2_params[0])) + (fit2_params[4]   * (y[k] - fit2_params[1])) + (fit2_params[8] *
				(z[k] - fit2_params[2]));
		C = (fit2_params[7] * (x[k] - fit2_params[0])) + (fit2_params[8] * (y[k] - fit2_params[1])) + (fit2_params[5]    *
				(z[k] - fit2_params[2]));
		length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / _samples_collected;
	fit2 = sqrtf(fit2) / _samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (isfinite(fitness) && fitness < _fitness) {
		_fitness = fitness;
		*offset_x = fit1_params[0];
		*offset_y = fit1_params[1];
		*offset_z = fit1_params[2];
		*diag_x = fit1_params[3];
		*diag_y = fit1_params[4];
		*diag_z = fit1_params[5];
		*offdiag_x = fit1_params[6];
		*offdiag_y = fit1_params[7];
		*offdiag_z = fit1_params[8];
		return 0;

	} else {
		return -1;
	}
}


int ellipsoid_fit_least_squares(float *x, float *y, float *z,
				unsigned int size, int max_iterations, float delta, float *offset_x, float *offset_y, float *offset_z,
				float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	float _fitness = 1.0e30f, _sphere_lambda = 1.0f, _ellipsoid_lambda = 1.0f;

	for (int i = 0; i < max_iterations; i++) {
		run_lm_sphere_fit(x, y, z, _fitness, _sphere_lambda,
				  size, offset_x, offset_y, offset_z,
				  sphere_radius, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z);

	}

	_fitness = 1.0e30f;

	for (int i = 0; i < max_iterations; i++) {
		run_lm_ellipsoid_fit(x, y, z, _fitness, _ellipsoid_lambda,
				     size, offset_x, offset_y, offset_z,
				     sphere_radius, diag_x, diag_y, diag_z, offdiag_x, offdiag_y, offdiag_z);
	}

	return 0;
}