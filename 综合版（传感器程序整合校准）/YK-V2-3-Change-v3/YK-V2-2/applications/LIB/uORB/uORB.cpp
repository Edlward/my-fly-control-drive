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
 * File      : uORB.cpp
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-28      HD      first implementation
 */


#include "uORB.h"
#include "topics/parameter_update.h"
#include "uORBHelper.h"
#include <cstdlib>
#include <cstring>
#include "delay.h"
#include "board.h"

//#include <poll.h>

#ifdef USE_UORB_CCMRAM

#include "uORBRam.h"

#pragma default_variable_attributes = @ ".ccmram"
uORBRam _ccmram;
#endif

ORBData               *orb_data[total_uorb_num];
pollfd_struct_t       poll_queue[UORB_MAX_POLL];

#ifdef USE_UORB_CCMRAM
#pragma default_variable_attributes =
#endif

#define UORBDEFSEM(name)   #name



//flag that indicates if we have entered the operating system
bool orb_in_os = false;

int orb_lock(void)
{
    if(orb_in_os)
    {
        return rt_hw_interrupt_disable();
    }
    
    return -1;
}

void orb_unlock(int state)
{
    if(orb_in_os && state != -1)
    {
        rt_hw_interrupt_enable(state);
    }
}


void orb_sem_lock(ORBData *_orb)
{
    if(_orb && orb_in_os)
    {
        int32_t res = rt_sem_take(_orb->sem, 0xFFFFFFFF);
        switch (res) {
            case RT_EOK:
                _orb->sem_taken = true;
                break;
            case RT_ETIMEOUT:
                _orb->sem_taken = false;
                break;
            case RT_ERROR:
                _orb->sem_taken = false;
                break;
            default:
                _orb->sem_taken = false;
                break;
       }
    }
}

void orb_sem_unlock(ORBData *_orb)
{
    if(_orb && orb_in_os)
    {
        int32_t res = rt_sem_release(_orb->sem);
        if (res == RT_EOK) {
            _orb->sem_taken = false;
        }
    }
}

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{   
	  orb_advert_t advert = nullptr;
    
    //get the serial number of the current orb
    int serial = get_orb_serial(meta->o_name);
    
    //we get an effective and multi-prio orb
    if(serial != -1)
    {   
		orb_sem_lock(orb_data[serial]);
		
		//reset the published flag to false to prevent subscribing and checking
		orb_data[serial]->published = false;
		
		int atomic_state = orb_lock();
		
		if(orb_data[serial]->data == nullptr)
		{
#ifndef USE_UORB_CCMRAM
			orb_data[serial]->data = new uint8_t[meta->o_size];
#else
			orb_data[serial][inst].data = (uint8_t*)_ccmram.uORBDataCalloc(meta->o_size, 1);
#endif
		}
		
		//copy the data
		std::memcpy(orb_data[serial]->data, data, meta->o_size);
		
		orb_unlock(atomic_state);
		
		//copy the serial number
		orb_data[serial]->serial = serial;
	
		//name the advert as the pointer of the orb's internal data
		advert = (void*)(orb_data[serial]);
		
		//get the current system time in us
		orb_data[serial]->last_updated_time = hrt_absolute_time_us();
		
		//update the published flag to true
		orb_data[serial]->published = true;
		
		int task_id = (int)rt_thread_self();
		for(int i = 0; i < UORB_MAX_SUB; ++i)
		{
			orb_data[serial]->authority_list[i] = -1;
		}
		
		orb_sem_unlock(orb_data[serial]);
		

    }
    return advert;
}



int orb_unadvertise(orb_advert_t handle)
{
    handle = nullptr;
    return 0;
}

int orb_publish_auto(const struct orb_metadata *meta, orb_advert_t *handle, const void *data, int *instance,
		     int priority)
{
	if (*handle == nullptr) {
                int serial = get_orb_serial(meta->o_name);
                if(is_orb_multi(serial))
				{
					// *handle = orb_advertise_multi(meta, data, instance, priority);				
				}                   
                else{
                    *handle = orb_advertise(meta, data);
                    if(instance)
                        *instance = 0;
                }

		if (*handle != nullptr) {
			return 0;
		}

	} else {
		return orb_publish(meta, *handle, data);
	}

	return -1;
}

void orb_poll_notify(int fd, pollevent_t events)
{
    int atomic_state = orb_lock();
    
    for(int i = 0; i < UORB_MAX_POLL; ++i)
    {
        if(poll_queue[i].fd == fd && poll_queue[i].sem)
        {
            rt_sem_release(poll_queue[i].sem);
            poll_queue[i].revents = events;
        }
    }
    
    orb_unlock(atomic_state);
}



int  orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
    
    int ret = -1;
    int serial = get_orb_serial(meta->o_name);
    
    if(serial != -1)
    {        
		orb_sem_lock(orb_data[serial]);
		
		orb_data[serial]->published = false;
		
		int atomic_state = orb_lock();
		
		std::memcpy(orb_data[serial]->data, data, meta->o_size);
		
		orb_unlock(atomic_state);
			
		orb_data[serial]->serial = serial;
			
		ret = 0;
			
		orb_data[serial]->last_updated_time = hrt_absolute_time_us();
		
		orb_data[serial]->published = true;
		
		for(int i = 0; i < UORB_MAX_SUB; ++i)
		{
			orb_data[serial]->authority_list[i] = -1;
		}
		
		for(int i = 0; i < UORB_MAX_POLL; ++i)
		{
			if(poll_queue[i].fd == (int)((serial << 4) | 0))
			{
				orb_poll_notify(poll_queue[i].fd, 1);
				break;
			}
		}
		
		orb_sem_unlock(orb_data[serial]);
        

    }
    
    return ret;
}

int  orb_subscribe(const struct orb_metadata *meta)
{
    if(meta == nullptr || meta->o_name == nullptr)
        return -1;
    
    int serial = get_orb_serial(meta->o_name);
    
    int ret = -1;
    
    if(serial >= 0 && serial < total_uorb_num)
    {
        int instance = 0;

        int task_id = (int)rt_thread_self();
        
        int atomic_state = orb_lock();
        
        for(int i = 0; i < UORB_MAX_SUB; ++i)
        {
			
            if(orb_data[serial]->registered_list[i] == task_id)
            {
				
                ret = (int)((serial << 4) | (instance));
                break;
            }
            
            if(orb_data[serial]->registered_list[i] == -1)
            {
                orb_data[serial]->registered_list[i] = task_id;
                ret = (int)((serial << 4) | (instance));
                break;
            }
        }
        
        orb_unlock(atomic_state);
    }

    return ret;
}

int  orb_unsubscribe(int handle)
{
    if(handle < 0)
    {
        return -1;
    }
    
    int serial = (handle >> 4);
    
    int instance = 0;
    
    if(serial >= 0 && serial < total_uorb_num)
    {   
        int task_id = (int)rt_thread_self();
        
        int atomic_state = orb_lock();
        for(int i = 0; i < UORB_MAX_SUB; ++i)
        {
            if(orb_data[serial]->registered_list[i] == task_id)
            {
                orb_data[serial]->registered_list[i] = -1;
                orb_unlock(atomic_state);
                return 0;
            }
        }
        orb_unlock(atomic_state);
    }
    else
    {
        return -1;
    }
    
    return 0;
}

int  orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{   
    int ret = -1;
    int serial = get_orb_serial(meta->o_name);
    if(serial != -1 && serial == (handle >> 4))
    {   
        //int instance = handle - ((handle >> 4) << 4);
        
        orb_sem_lock(orb_data[serial]);
        
        if(orb_data[serial]->data != nullptr)
        {
            bool authorised = false;
            int task_id = (int)rt_thread_self();
                
            for(int i = 0; i < UORB_MAX_SUB; ++i)
            {
                if(orb_data[serial]->authority_list[i] == -1)
                {
                    orb_data[serial]->authority_list[i] = task_id;
                    authorised = true;
                    break;
                }
            }
            
            if(authorised)
            {
                int atomic_state = orb_lock();
            
                std::memcpy(buffer, orb_data[serial]->data, meta->o_size);
            
                orb_unlock(atomic_state);
            
                ret = 0;
            }
            
            int registered_len = 0,authority_len = 0;
            for(int i = 0;i < UORB_MAX_SUB; ++i)
            {
                if(orb_data[serial]->registered_list[i] != -1)
                    ++registered_len;
                if(orb_data[serial]->authority_list[i] != -1)
                    ++authority_len;
            }
            
            if(authority_len >= registered_len)
                orb_data[serial]->published = false;
        }
        
        orb_sem_unlock(orb_data[serial]);
    }
    
    return ret;
}


int  orb_check(int handle, bool *updated)
{
    if(handle < 0)
    {
        *updated = false;
        return -1;
    }
    
    int serial = (handle >> 4);
    
     if(serial < 0 || serial >= total_uorb_num )
    {
        *updated = false;
        return -1;
    }
    
    orb_sem_lock(orb_data[serial]);
    
    bool authorised = false;
    int task_id = (int)rt_thread_self();
    
    for(int i = 0; i < UORB_MAX_SUB; ++i)
    {
        if(orb_data[serial]->authority_list[i] == task_id)
        {
            authorised = true;
            break;
        }
    }
    
    if(orb_data[serial]->published && !authorised)
    {
        *updated = true;
        
        orb_sem_unlock(orb_data[serial]);
        
        return 0;
    }
    else
        *updated = false;
    
    orb_sem_unlock(orb_data[serial]);
    
    return -1;
}

int  orb_stat(int handle, uint64_t *time)
{   
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >= 0 && serial < total_uorb_num &&
       instance>= 0 && instance <ORB_MULTI_MAX_INSTANCES)
    {
        *time = orb_data[serial][instance].last_updated_time;
        
        return 0;
    }
    
    return -1;
}

int  orb_exists(const struct orb_metadata *meta, int instance)
{
    int serial = get_orb_serial(meta->o_name);
    if(serial < 0 || serial >= total_uorb_num)
        return -1;
    if(!is_orb_multi(serial) && instance > 0)
        return -1;
    
    if((orb_data[serial][instance].data) != nullptr && 
       (orb_data[serial][instance].serial) != -1)
    {
        return 0;
    }
    
    return -1;
}

int  orb_group_count(const struct orb_metadata *meta)
{
	unsigned instance = 0;

        for(int i = 0; i < ORB_MULTI_MAX_INSTANCES; ++i)
        {
            if(orb_exists(meta, i) == 0)
                ++instance;
        }

	return instance;
}

int  orb_priority(int handle, int32_t *priority)
{
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >=0 && serial < total_uorb_num &&
       instance >=0 && instance < ORB_MULTI_MAX_INSTANCES)
    {
        *priority = orb_data[serial][instance].priority;
        return 0;
    }
    return -1;
}

int orb_set_interval(int handle, unsigned interval)
{
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >= 0 && serial < total_uorb_num &&
       instance >= 0 && instance < ORB_MULTI_MAX_INSTANCES)
    {
        orb_data[serial][instance].interval = interval;
        return 0;
    }
    return -1;
}

int orb_get_interval(int handle, unsigned *interval)
{
    int serial = (handle >> 4);
    int instance = handle - ((handle >> 4) << 4);
    
    if(serial >=0 && serial < total_uorb_num &&
       instance >=0 && instance < ORB_MULTI_MAX_INSTANCES)
    {
        *interval = orb_data[serial][instance].interval;
        return 0;
    }
    return -1;
}

rt_mempool rt_uorb_s_mem,rt_uorb_m_mem;
rt_mempool rt_reg_mem;
rt_mempool rt_auth_mem;
rt_err_t ccmram_init(void)
{
	rt_err_t ret;
	
	rt_size_t uorb_single_mem_size,uorb_mult_mem_size;
	rt_size_t auth_mem_size,reg_mem_size;
	
	
	uorb_single_mem_size = (sizeof(ORBData)+4)*(total_uorb_num);
	ret = rt_mp_init(&rt_uorb_s_mem,"uorb",CCMRAM_BEGIN,uorb_single_mem_size,sizeof(ORBData));
	if(ret != RT_EOK)
	{
		rt_kprintf("uorb alloc error:%d", ret);
		return ret;
	}
	
//	void * MULT_MEM_BEGIN = (void *)(STM32_CCMRAM_START+uorb_single_mem_size);
//	uorb_mult_mem_size = (sizeof(ORBData)*ORB_MULTI_MAX_INSTANCES+4)*MULTI_ORB_NUM;
//	rt_mp_init(&rt_uorb_m_mem,"uorb_mul",MULT_MEM_BEGIN,uorb_mult_mem_size,sizeof(ORBData));
//	if(ret != RT_EOK)
//	{
//		rt_kprintf("uorb_mul alloc error:%d", ret);
//		return ret;
//	}
	
	void * AUTH_MEM_BEGIN = (void *)(STM32_CCMRAM_START + uorb_single_mem_size);
	auth_mem_size = (sizeof(int32_t)*UORB_MAX_SUB+4)*(total_uorb_num);
	rt_mp_init(&rt_auth_mem,"auth",AUTH_MEM_BEGIN,auth_mem_size,sizeof(int32_t)*UORB_MAX_SUB);
	if(ret != RT_EOK)
	{
		rt_kprintf("auth_mem alloc error:%d", ret);
		return ret;
	}
	
	void * REG_MEM_BEGIN = (void *)(STM32_CCMRAM_START + uorb_single_mem_size + auth_mem_size);
	reg_mem_size = (sizeof(int32_t)*UORB_MAX_SUB+4)*(total_uorb_num);
	rt_mp_init(&rt_reg_mem,"reg",REG_MEM_BEGIN,reg_mem_size,sizeof(int32_t)*UORB_MAX_SUB);
	if(ret != RT_EOK)
	{
		rt_kprintf("reg_mem alloc error:%d", ret);
		return ret;
	}
	
	return ret;
}



int orb_init(void)
{	
	char semname[10]="sem";
	
	orb_in_os = false;
	rt_err_t ret;
	ret = ccmram_init();
	if(ret != RT_EOK)
	{
		rt_kprintf("ccmram init failed");
	}
	for(char i = 0; i < UORB_MAX_POLL; ++i)
    {
        poll_queue[i].fd = -1;
		char pollname[10]="poll";
		char str[10];
		sprintf(str,"%d",i);		
		std::strcat(pollname,str);
        poll_queue[i].sem = rt_sem_create(pollname, 0, RT_IPC_FLAG_FIFO);
        
        poll_queue[i].revents = 0;

    }
	int k=0;
	for(int i = 0; i < total_uorb_num; ++i)
    {
		orb_data[i] = (ORBData*)rt_mp_alloc(&rt_uorb_s_mem,1);
		orb_data[i]->data = get_orb_public_according_to_serial_and_instance(i,0);
		orb_data[i]->priority = ORB_PRIO_DEFAULT;
		orb_data[i]->serial = -1;
		orb_data[i]->interval = 0;
		orb_data[i]->queue = nullptr;
		orb_data[i]->published = false;

		char semname[10]="sem";
		char str[10];
		sprintf(str,"%d",i);		
		std::strcat(semname,str);
		orb_data[i]->sem = rt_sem_create(semname, 1, RT_IPC_FLAG_FIFO);
		orb_data[i]->sem_taken = false;

		orb_data[i]->last_updated_time = 0;

		orb_data[i]->registered_list = (int32_t*)rt_mp_alloc(&rt_reg_mem,1);
		orb_data[i]->authority_list = (int32_t*)rt_mp_alloc(&rt_auth_mem,1);
		std::memset(orb_data[i]->registered_list,-1,sizeof(int32_t)*UORB_MAX_SUB);
		std::memset(orb_data[i]->authority_list,-1,sizeof(int32_t)*UORB_MAX_SUB);
		//orb_data[i]->registered_list[0] = -1;
		//orb_data[i]->authority_list[0] = -1;
	
	}
	return 0;
}
INIT_APP_EXPORT(orb_init);