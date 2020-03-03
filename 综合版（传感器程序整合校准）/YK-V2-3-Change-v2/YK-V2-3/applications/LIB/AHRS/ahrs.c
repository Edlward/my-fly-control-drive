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
 * File      : ahrs.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include <math.h>
#include "ahrs.h"

// System constants
#define gyroMeasError 3.14159265358979 * (1.5f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.4f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)

static float beta = 0.8660254f * gyroMeasError; // compute beta
static float zeta = 0.8660254f * gyroMeasDrift; // compute zeta
// Global system variables
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

static float accU[3], magU[3];
static float delta[3];
static float FACTOR_P = 0.25f;
static float FACTOR_I = 0.1f;

static float acc_cross[3];
static float mag_cross[3];
static float acc_const[3] = {0.0f, 0.0f, -1.0f};
static float mag_const[3] = {1.0f, 0.0f, 0.0f};
static float gyr_bias[3];
static float errX_Int = 0.0f;
static float errY_Int = 0.0f;
static float errZ_Int = 0.0f;

float comp_gain = 0.05f;

void Runge_Kutta_1st(quaternion* attitude, quaternion q, float g[3], float dT)
{
	float halfT = 0.5f*dT;
	
//	OS_ENTER_CRITICAL;
	attitude->w += halfT * (-g[0] * q.x - g[1] * q.y - g[2] * q.z);
	attitude->x += halfT * ( g[0] * q.w - g[1] * q.z + g[2] * q.y);
	attitude->y += halfT * ( g[0] * q.z + g[1] * q.w - g[2] * q.x);
	attitude->z += halfT * (-g[0] * q.y + g[1] * q.x + g[2] * q.w);
//	OS_EXIT_CRITICAL;
}

/* use acc and mag data to calculate the current attitude */
void AHRS_reset(quaternion * q, const float acc[3],const float mag[3])
{
	quaternion q1,q2;
	float to[3];
	float from[3];
	
	/* pitch-row modification */
	/* calculate rotation from current acc to acc constant [0,0,-1] */
	to[0] = to[1] = 0.0f;
	to[2] = -1.0f;
	quaternion_fromTwoVectorRotation(&q1, acc, to);
	
	/* yaw modification */
	/* first rotate mag by q1 */
	quaternion_rotateVector(&q1, mag, from);
	from[2] = 0;
	to[0] = 1;
	to[1] = to[2] = 0;
	/* calculate rotation from mag to mag constant [1,0,0] */
	quaternion_fromTwoVectorRotation(&q2, from, to);
	
	/* combine two rotations to get current attitude */
	quaternion_mult(q, &q2, &q1);
	quaternion_normalize(q);
	
	gyr_bias[0] = 0.0f;
	gyr_bias[1] = 0.0f;
	gyr_bias[2] = 0.0f;
}

void AHRS_update(quaternion * q, const float gyr[3], const float acc[3], const float mag[3], float dT)
{	
	Vector3_Normalize(accU, acc);
	Vector3_Normalize(magU, mag);
	
	/* transfer acc and mag from body frame to nav frame */
	float acc_N[3], mag_N[3];
	quaternion_rotateVector(q, accU, acc_N);
	quaternion_rotateVector(q, magU, mag_N);
		
	/* we only need x and y value for mag to calculate error of yaw */
	mag_N[2] = 0.0f;
	Vector3_Normalize(mag_N, mag_N);
	
	/* cross product to calculate diffirence */
	Vector3_CrossProduct(acc_cross, acc_N, acc_const);
	Vector3_CrossProduct(mag_cross, mag_N, mag_const);
	
	/* calculate error in navigation frame */
	/* error of roll and pitch come from acc, error of yaw come from mag */
	float err_N[3], err_B[3];

	err_N[0] = acc_cross[0] + mag_cross[0];
	err_N[1] = acc_cross[1] + mag_cross[1];
	err_N[2] = acc_cross[2] + mag_cross[2];
	
	/* transfer error from navigation frame to body frame */
	quaternion_inv_rotateVector(q, err_N, err_B);
	
	/* integrate error to estimate gyr bias */
	gyr_bias[0] += err_B[0]*FACTOR_I*dT;  
	gyr_bias[1] += err_B[1]*FACTOR_I*dT;  
	gyr_bias[2] += err_B[2]*FACTOR_I*dT; 
 
	/* calculate delta value */
	delta[0] = gyr[0] + FACTOR_P*err_B[0] + gyr_bias[0];  
	delta[1] = gyr[1] + FACTOR_P*err_B[1] + gyr_bias[1];  
	delta[2] = gyr[2] + FACTOR_P*err_B[2] + gyr_bias[2];
	
	/* first order runge-kutta to create quaternion */
	Runge_Kutta_1st(q, *q, delta, dT);
	quaternion_normalize(q);
}


