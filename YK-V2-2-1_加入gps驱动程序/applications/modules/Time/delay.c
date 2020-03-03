/**
* File      : delay.c
*
*
* Change Logs:
* Date           Author       	Notes
* 2016-6-6    		zoujiachi   	the first version
*/

#include <rtthread.h>
#include "delay.h"

DELAY_TIME_Def _delay_t;

// 获取当前时间，us。
uint64_t hrt_absolute_time_us(void)
{
    return _delay_t.msPeriod * (uint64_t)1000 + (SysTick->LOAD - SysTick->VAL) / _delay_t.ticksPerUs;
}

// 获取当前时间，ms。
uint32_t hrt_absolute_time_ms(void)
{
    return _delay_t.msPeriod + (SysTick->LOAD - SysTick->VAL) / _delay_t.ticksPerMs;
}

// 延时delay us，delay>=4时才准确。
void time_waitUs(uint32_t delay)
{
    uint64_t target = hrt_absolute_time_us() + delay;
    while(hrt_absolute_time_us() < target)
		;
}

// 延时delay ms。
void time_waitMs(uint32_t delay)
{
    time_waitUs(delay * 1000);
}

int device_delay_init(void)
{
	//RCC_ClkInitTypeDef  rcc_clocks;
	//uint32_t FLatency;

    // HAL_RCC_GetClockConfig(&rcc_clocks,&FLatency);
	
    _delay_t.msPeriod = 0;  							
    _delay_t.ticksPerUs = HAL_RCC_GetHCLKFreq()/ 1e6;    	
    _delay_t.ticksPerMs = HAL_RCC_GetHCLKFreq()/ 1e3;    	
    _delay_t.msPerPeriod = 1000/RT_TICK_PER_SECOND;  	
	
	return RT_EOK;
}

INIT_DEVICE_EXPORT(device_delay_init);
