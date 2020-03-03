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
 * File      : fast_loop.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-28      HD      first implementation
 */

#include "fast_loop.h"
#include "sensor_manager.h"
#include "filter.h"
#include "hil_interface.h"
#include "control_main.h"

#define EVENT_FAST_LOOP		(1<<0)

static struct rt_timer timer_fastloop;
static struct rt_event event_fastloop;

static void timer_fastloop_update(void* parameter)
{
	rt_event_send(&event_fastloop, EVENT_FAST_LOOP);
}

void fast_loop(void)
{
	
#ifdef HIL_SIMULATION
	hil_sensor_collect();
#else
	//sensor_collect();
#endif
	
	//ctrl_att_adrc_update();
	
}

void fastloop_entry(void *parameter)
{
	rt_err_t res;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = EVENT_FAST_LOOP;
	
	/* create event */
	res = rt_event_init(&event_fastloop, "fastloop", RT_IPC_FLAG_FIFO);

	/* register timer event */
	rt_timer_init(&timer_fastloop, "timer_fast",
					timer_fastloop_update,
					RT_NULL,
					1,
					RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&timer_fastloop);
	
	while(1)
	{
		res = rt_event_recv(&event_fastloop, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
								RT_WAITING_FOREVER, &recv_set);
		
		if(res == RT_EOK){
			if(recv_set & EVENT_FAST_LOOP){
				fast_loop();
			}
		}
	}
}
