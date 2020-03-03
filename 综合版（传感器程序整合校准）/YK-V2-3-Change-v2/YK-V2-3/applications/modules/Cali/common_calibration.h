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
 * File      : common_calibration.h
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include "common.h"
#include <uORB/uORB.h>

#ifdef __cplusplus
extern "C" {
#endif

enum calibrate_return {
	calibrate_return_ok = 0,
	calibrate_return_error,
	calibrate_return_cancelled
};

// The order of these cannot change since the calibration calculations depend on them in this order
enum detect_orientation_return {
	DETECT_ORIENTATION_TAIL_DOWN,
	DETECT_ORIENTATION_NOSE_DOWN,
	DETECT_ORIENTATION_LEFT,
	DETECT_ORIENTATION_RIGHT,
	DETECT_ORIENTATION_UPSIDE_DOWN,
	DETECT_ORIENTATION_RIGHTSIDE_UP,
	DETECT_ORIENTATION_ERROR
};

static const unsigned detect_orientation_side_count = 6;

struct calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
};

typedef enum calibrate_return(*calibration_from_orientation_worker_t)(enum detect_orientation_return
		orientation,	///< Orientation which was detected
		int				cancel_sub,	///< Cancel subscription from calibration_cancel_subscribe
		void				*worker_data);	///< Opaque worker data


		
enum detect_orientation_return detect_orientation(int cancel_sub, int accel_sub,bool lenient_still_position);

enum calibrate_return calibrate_from_orientation(
		int		cancel_sub,
		bool	side_data_collected[detect_orientation_side_count],
		calibration_from_orientation_worker_t calibration_worker,
		void	*worker_data,
		bool	lenient_still_position);

int calibrate_cancel_subscribe();
		
void calibrate_cancel_unsubscribe(int cmd_sub);		

int run_lm_ellipsoid_fit(float *x, float *y, float *z, float _fitness, float _sphere_lambda,
			 unsigned int size, float *offset_x, float *offset_y, float *offset_z,
			 float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z);	
			 
int run_lm_sphere_fit(float *x, float *y, float *z, float _fitness, float _sphere_lambda,
		      unsigned int size, float *offset_x, float *offset_y, float *offset_z,
		      float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z);
		
			 
int ellipsoid_fit_least_squares(float *x, float *y, float *z,
				unsigned int size, int max_iterations, float delta, float *offset_x, float *offset_y, float *offset_z,
				float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z);
			 
#ifdef __cplusplus
}
#endif