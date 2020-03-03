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

#define PWM_OUT_NUM		6				//PWM�������

#define PWM_DEFAULT_HZ  50.0f
#define PWM_DEFAULT_PERIOD  1.0f/PWM_DEFAULT_HZ*1000*1000      //���ڵ�λns

#define PWM_MAX_PLUS		2200		//��λus
#define PWM_MIX_PLUS		800			//��λus
#define PWM_MID_PLUS		1500		//��λus

#define PWM_DISABLE_PLUS	900			//��λus
#define PWM_ENABLE_PLUS		1080		//��λus

#define PWM_NS_TO_US		1000

#define PWM_DEV_GROUP_1        "pwm1"  /* PWM�豸���� */
#define PWM_DEV_GROUP_2        "pwm2"  /* PWM�豸���� */
#define PWM_DEV_GROUP_3        "pwm3"  /* PWM�豸���� */
#define PWM_DEV_GROUP_4        "pwm4"  /* PWM�豸���� */
#define PWM_DEV_CH1			1       /* PWMͨ�� */
#define PWM_DEV_CH2			2       /* PWMͨ�� */
#define PWM_DEV_CH3			3       /* PWMͨ�� */
#define PWM_DEV_CH4			4       /* PWMͨ�� */

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