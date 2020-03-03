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
 * File      : ekf2.h
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation	
 */

#ifndef _EKF2_H
#define _EKF2_H
#include "ekf.h"
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>


bool 	_replay_mode = false;			///< true when we use replay data from a log

// time slip monitoring
uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)


// Initialise time stamps used to send sensor data to the EKF and for logging
uint8_t _invalid_mag_id_count = 0;	///< number of times an invalid magnetomer device ID has been detected

// Used to down sample magnetometer data
float _mag_data_sum[3] = {};			///< summed magnetometer readings (Gauss)
uint64_t _mag_time_sum_ms = 0;		///< summed magnetoemter time stamps (mSec)
uint8_t _mag_sample_count = 0;		///< number of magnetometer measurements summed during downsampling
int32_t _mag_time_ms_last_used = 0;	///< time stamp of the last averaged magnetometer measurement sent to the EKF (mSec)

// Used to down sample barometer data
float _balt_data_sum = 0.0f;			///< summed pressure altitude readings (m)
uint64_t _balt_time_sum_ms = 0;		///< summed pressure altitude time stamps (mSec)
uint8_t _balt_sample_count = 0;		///< number of barometric altitude measurements summed
uint32_t _balt_time_ms_last_used =
	0;	///< time stamp of the last averaged barometric altitude measurement sent to the EKF (mSec)

float _last_valid_mag_cal[3] = {};	///< last valid XYZ magnetometer bias estimates (mGauss)
bool _valid_cal_available[3] = {};	///< true when an unsaved valid calibration for the XYZ magnetometer bias is available
float _last_valid_variance[3] = {};	///< variances for the last valid magnetometer XYZ bias estimates (mGauss**2)

// Used to control saving of mag declination to be used on next startup
bool _mag_decl_saved = false;	///< true when the magnetic declination has been saved

// Used to filter velocity innovations during pre-flight checks
bool _preflt_horiz_fail = false;	///< true if preflight horizontal innovation checks are failed
bool _preflt_vert_fail = false;		///< true if preflight vertical innovation checks are failed
bool _preflt_fail = false;		///< true if any preflight innovation checks are failed
matrix::Vector2f _vel_ne_innov_lpf = {};	///< Preflight low pass filtered NE axis velocity innovations (m/sec)
float _vel_d_innov_lpf = {};		///< Preflight low pass filtered D axis velocity innovations (m/sec)
float _hgt_innov_lpf = 0.0f;		///< Preflight low pass filtered height innovation (m)
float _yaw_innov_magnitude_lpf = 0.0f;	///< Preflight low pass filtered yaw innovation magntitude (rad)

static constexpr float _innov_lpf_tau_inv = 0.2f;	///< Preflight low pass filter time constant inverse (1/sec)
static constexpr float _vel_innov_test_lim =
	0.5f;	///< Maximum permissible velocity innovation to pass pre-flight checks (m/sec)
static constexpr float _hgt_innov_test_lim =
	1.5f;	///< Maximum permissible height innovation to pass pre-flight checks (m)
static constexpr float _nav_yaw_innov_test_lim =
	0.25f;	///< Maximum permissible yaw innovation to pass pre-flight checks when aiding inertial nav using NE frame observations (rad)
static constexpr float _yaw_innov_test_lim =
	0.52f;	///< Maximum permissible yaw innovation to pass pre-flight checks when not aiding inertial nav using NE frame observations (rad)
const float _vel_innov_spike_lim = 2.0f * _vel_innov_test_lim;	///< preflight velocity innovation spike limit (m/sec)
const float _hgt_innov_spike_lim = 2.0f * _hgt_innov_test_lim;	///< preflight position innovation spike limit (m)

// set pose/velocity as invalid if standard deviation is bigger than max_std_dev
// TODO: the user should be allowed to set these values by a parameter
static constexpr float ep_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated position
static constexpr float eo_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated orientation
//static constexpr float ev_max_std_dev = 100.0f;	///< Maximum permissible standard deviation for estimated velocity

static Ekf _ekf;


#endif