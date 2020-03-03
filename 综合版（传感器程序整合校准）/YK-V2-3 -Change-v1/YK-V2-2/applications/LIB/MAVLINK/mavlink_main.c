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
 * File      : mavlink_main.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
#include <rtthread.h>
#include <rtdevice.h>


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAV_DEBUG

#ifdef MAV_DEBUG
#define MAV_TRACE         rt_kprintf
#else
#define MAV_TRACE(...)
#endif /* #ifdef MAV_DEBUG */




int rt_mavlink_init(void)
{

	return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_mavlink_init);


static void mavlink_measure()
{

}



static void mavlink_config()
{

}


static void mavlink_start(void)
{
	mavlink_config();
	
	while(1)
	{
		mavlink_measure();
		rt_thread_mdelay(500);
	}	
}


rt_err_t mavlink_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			mavlink_start();
		}

		if (!strcmp(verb, "stop")) {
			//mavlink_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//mavlink_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//mavlink_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//mavlink_info(busid);
		}
	}
	

	//mavlink_usage();
	return 0;
}

MSH_CMD_EXPORT(mavlink_main,mavlink)