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
 * File      : pwmout.h
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
 
#ifndef _PWMOUT_H
#define	_PWMOUT_H


#include <rtthread.h>
#include <rtdevice.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define PWM_OUT_NUM		6				//PWM输出个数

#define PWM_DEFAULT_HZ  50.0f
#define PWM_DEFAULT_PERIOD  1.0f/PWM_DEFAULT_HZ*1000*1000      //周期单位ns

#define PWM_MAX_PLUS		2200		//单位us
#define PWM_MIX_PLUS		800			//单位us
#define PWM_MID_PLUS		1500		//单位us

#define PWM_DISABLE_PLUS	900			//单位us
#define PWM_ENABLE_PLUS		1080		//单位us

#define PWM_NS_TO_US		1000

#define PWM_DEV_GROUP_1        "pwm1"  /* PWM设备名称 */
#define PWM_DEV_GROUP_2        "pwm2"  /* PWM设备名称 */
#define PWM_DEV_GROUP_3        "pwm3"  /* PWM设备名称 */
#define PWM_DEV_GROUP_4        "pwm4"  /* PWM设备名称 */
#define PWM_DEV_CH1			1       /* PWM通道 */
#define PWM_DEV_CH2			2       /* PWM通道 */
#define PWM_DEV_CH3			3       /* PWM通道 */
#define PWM_DEV_CH4			4       /* PWM通道 */

enum pwm_type
{
	NONE = 0,
	MOTOR,
	SERVOR,
};

struct pwm_out_s
{
	struct rt_device_pwm parant;
	struct rt_pwm_configuration config;
	enum pwm_type type;
	rt_uint8_t		group;			
};



#endif