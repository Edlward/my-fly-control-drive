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
 * File      : common.h
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
#ifndef _COMMON_H
#define _COMMON_H
#include <rtthread.h>
#include <rtdevice.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "delay.h"


#ifdef RT_USING_ULOG
#define LOG_TAG              "example"
#define LOG_LVL              LOG_LVL_ASSERT
#include <ulog.h>
#endif

#define BMI088_ACCEL_ID					1
#define BMI088_GYRO_ID					2
#define MPU9250_ACCEL_ID				3
#define MPU9250_GYRO_ID					4
#define MPU9250_MAG_ID					5
#define MLX90393_MAG_ID					6
#define MS5611_BARO_ID					7




#define SENSOR_THREAD_PRIORITY			3
#define EKF2_THREAD_PRIORITY			4
#define COPTER_THREAD_PRIORITY			4
#define STARRYIO_THREAD_PRIORITY		9
#define LOGGER_THREAD_PRIORITY			11
#define MAVLINK_RX_THREAD_PRIORITY		11
#define MAVLINK_THREAD_PRIORITY			12
#define LED_THREAD_PRIORITY				13
#define CALI_THREAD_PRIORITY			13


#endif