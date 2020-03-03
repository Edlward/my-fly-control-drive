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


struct rt_device_pwm *pwm_dev;      /* PWMÉè±¸¾ä±ú */
struct pwm_out_s out_dev[PWM_OUT_NUM];


static void pwm_update_set(struct pwm_out_s *pwm_out,rt_uint32_t *value,rt_uint8_t count)
{
	
}

static void out_dev_map(struct pwm_out_s *pwm_out[])
{
	pwm_out[0] = (struct pwm_out_s *)rt_device_find(PWM_DEV_GROUP_1);
	pwm_out[0]->config.channel = PWM_DEV_CH4;
	pwm_out[0]->config.period = PWM_DEFAULT_PERIOD;
	pwm_out[0]->config.pulse = PWM_DISABLE_PLUS * PWM_NS_TO_US;
	pwm_out[0]->group = 1;
	
	pwm_out[1] = (struct pwm_out_s *)rt_device_find(PWM_DEV_GROUP_1);
	pwm_out[1]->config.channel = PWM_DEV_CH3;
	pwm_out[1]->config.period = PWM_DEFAULT_PERIOD;
	pwm_out[1]->config.pulse = PWM_DISABLE_PLUS * PWM_NS_TO_US;
	pwm_out[1]->group = 1;
	
	pwm_out[2] = (struct pwm_out_s *)rt_device_find(PWM_DEV_GROUP_1);
	pwm_out[2]->config.channel = PWM_DEV_CH2;
	pwm_out[2]->config.period = PWM_DEFAULT_PERIOD;
	pwm_out[2]->config.pulse = PWM_DISABLE_PLUS * PWM_NS_TO_US;
	pwm_out[2]->group = 1;
	
	pwm_out[3] = (struct pwm_out_s *)rt_device_find(PWM_DEV_GROUP_1);
	pwm_out[3]->config.channel = PWM_DEV_CH1;
	pwm_out[3]->config.period = PWM_DEFAULT_PERIOD;
	pwm_out[3]->config.pulse = PWM_DISABLE_PLUS * PWM_NS_TO_US;
	pwm_out[3]->group = 1;
	
	pwm_out[4] = (struct pwm_out_s *)rt_device_find(PWM_DEV_GROUP_2);
	pwm_out[4]->config.channel = PWM_DEV_CH4;
	pwm_out[4]->config.period = PWM_DEFAULT_PERIOD;
	pwm_out[4]->config.pulse = PWM_DISABLE_PLUS * PWM_NS_TO_US;
	pwm_out[4]->group = 2;
	
	pwm_out[5] = (struct pwm_out_s *)rt_device_find(PWM_DEV_GROUP_2);
	pwm_out[5]->config.channel = PWM_DEV_CH3;
	pwm_out[5]->config.period = PWM_DEFAULT_PERIOD;
	pwm_out[5]->config.pulse = PWM_DISABLE_PLUS * PWM_NS_TO_US;
	pwm_out[5]->group = 2;
	
}


int rt_pwmout_init(void)
{		
        
	out_dev_map((struct pwm_out_s **)out_dev);
	for(int i=0;i<PWM_OUT_NUM;i++)
	{
		rt_pwm_set((struct rt_device_pwm *)&out_dev[i],out_dev[i].config.channel,out_dev[i].config.period,out_dev[i].config.pulse);
		rt_pwm_enable((struct rt_device_pwm *)&out_dev[i],out_dev[i].config.channel);
	}
	
	rt_bool_t result = RT_EOK;
    return result;
}
//INIT_APP_EXPORT(rt_pwmout_init);

static void pwmout_publish(orb_advert_t *_pwmout_adv,uint16_t *data)
{

}



static void pwmout_start(void)
{


	while(1)
	{
		rt_thread_mdelay(50);
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