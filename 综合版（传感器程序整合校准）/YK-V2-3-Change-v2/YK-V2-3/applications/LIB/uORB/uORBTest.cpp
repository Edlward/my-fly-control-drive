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
 * File      : uORBTest.cpp
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-28      HD      first implementation
 */

#include <uORB/uORB.h>
#include <cstring>
#include "uORBTest.h"
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/uORBHelper.h>
#include "delay.h"

orb_advert_t       att_adv;
orb_advert_t       att_sp_adv;
vehicle_attitude_setpoint_s     att_sp;
vehicle_attitude_s              att;
void uORBPub_Task(void * args)
{

  
    att_sp.pitch_body=1.0f;
    
    orb_set_in_os();
  
    att_sp_adv=orb_advertise(ORB_ID(vehicle_attitude_setpoint),&att_sp);
    uint32_t counter=0;
    while(1)
    {
        rt_thread_mdelay(1000);
        
        att_sp.q_d[0]=(float)++counter;
        att_sp.pitch_body +=1.0f;
        att_sp.timestamp = hrt_absolute_time_ms();
        
        orb_publish(ORB_ID(vehicle_attitude_setpoint),att_sp_adv,&att_sp);

    }
}


vehicle_attitude_s              Att;
vehicle_attitude_setpoint_s     Att_sp;

void uORBSub_Task(void * args)
{
    int       att_sub,att_sp_sub;
    

	time_waitMs(5);
	att_sp_sub=orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

  
    while(1)
    {
        rt_thread_mdelay(1000);
        bool updated = RT_FALSE;    
        
        orb_check(att_sp_sub,&updated);
        if(updated)
		{
			orb_copy(ORB_ID(vehicle_attitude_setpoint),att_sp_sub,&Att_sp);
		}

    }
}


int uORB_TEST()
{
	int ret =RT_EOK;
    rt_thread_t thread = rt_thread_create("uORB_SUB", uORBSub_Task, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }
	
	thread = rt_thread_create("uORB_PUB", uORBPub_Task, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}


MSH_CMD_EXPORT(uORB_TEST, uORB test);