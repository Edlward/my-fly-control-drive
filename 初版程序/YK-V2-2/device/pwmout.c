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
 * File      : pwmout.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */


#include "pwmout.h"
#include "delay.h"
#include <uORB/topics/input_rc.h>

#define LOG_TAG              "example"
#define LOG_LVL              LOG_LVL_ASSERT
#include <ulog.h>


#define PWM_DEBUG

#ifdef PWM_DEBUG
#define PWM_TRACE         rt_kprintf
#else
#define PWM_TRACE(...)
#endif /* #ifdef PWM_DEBUG */
#define PWM_DEV_NAME        "pwm1"  /* PWM设备名称 */
#define PWM_DEV_CHANNEL     4       /* PWM通道 */

struct rt_device_pwm *pwm_dev;      /* PWM设备句柄 */


int rt_pwmout_init(void)
{		
//	rt_uint32_t period, pulse, dir;
//    period = 5000000;    /* 周期为0.5ms，单位为纳秒ns */
//    dir = 1;            /* PWM脉冲宽度值的增减方向 */
//    pulse = 1000000;          /* PWM脉冲宽度值，单位为纳秒ns */

//    /* 查找设备 */
//    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM_DEV_NAME);
//    if (pwm_dev == RT_NULL)
//    {
//        rt_kprintf("pwm sample run failed! can't find %s device!\n", PWM_DEV_NAME);
//        //return RT_ERROR;
//    }

//    /* 设置PWM周期和脉冲宽度默认值 */
//    rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
//    /* 使能设备 */
//    rt_pwm_enable(pwm_dev, PWM_DEV_CHANNEL);

//	rt_bool_t result = RT_EOK;

    return 0;
}
INIT_APP_EXPORT(rt_pwmout_init);

static void pwmout_publish(orb_advert_t *_pwmout_adv,uint16_t *data)
{

}



static void pwmout_start(void)
{


	while(1)
	{
		rt_thread_mdelay(50);
//        if (dir)
//        {
//            pulse += 50000;      /* 从0值开始每次增加5000ns */
//        }
//        else
//        {
//            pulse -= 50000;      /* 从最大值开始每次减少5000ns */
//        }
//        if (pulse >= 2000000)
//        {
//            dir = 0;
//        }
//        if (1000000 >= pulse)
//        {
//            dir = 1;
//        }

//        /* 设置PWM周期和脉冲宽度 */
//        rt_pwm_set(pwm_dev, PWM_DEV_CHANNEL, period, pulse);
		//LOG_RAW("1:%d  2:%d  3:%d   4:%d  \n",r_raw_rc_values[0],r_raw_rc_values[1],r_raw_rc_values[2],r_raw_rc_values[3]);
	}
	
}


rt_err_t pwmout_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			pwmout_start();
		}

		if (!strcmp(verb, "stop")) {
			//pwmout_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//pwmout_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//pwmout_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//pwmout_info(busid);
		}
	}
	

	//pwmout_usage();
	return 0;
}

MSH_CMD_EXPORT(pwmout_main,pwmout)