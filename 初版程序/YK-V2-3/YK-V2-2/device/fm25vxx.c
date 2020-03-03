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
 * File      : fm25vxx.cpp
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


//#include "spi_fram.h"
#include "spi_fram_fm25vxx.h"
#include "drv_spi.h"
#include <drivers/spi.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define FRAM_DEBUG

#ifdef FRAM_DEBUG
#define FRAM_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define FRAM_CS_PIN 57

/* JEDEC Manufacturer’s ID */
#define FM_ID           (0xEF)

#define FM25Vxx_CMD_WREN     (unsigned)0x06  // Set Write Enable Latch
#define FM25Vxx_CMD_WRDI     (unsigned)0x04  // Write Disable

#define FM25Vxx_CMD_RDSR     (unsigned)0x05  // Read  Status register
#define FM25Vxx_CMD_WRSR     (unsigned)0x01  // Write Status register

#define FM25Vxx_CMD_READ     (unsigned)0x03  // Read Memory data
#define FM25Vxx_CMD_FSTRD    (unsigned)0x0B  // Fast Read Memory data
#define FM25Vxx_CMD_WRITE    (unsigned)0x02  // Write memory data

#define FM25Vxx_CMD_SLEEP    (unsigned)0xB9  // Enter Sleep Mode

#define FM25Vxx_CMD_RDID     (unsigned)0x9F  // Read DeviceID
#define FM25Vxx_CMD_SNR      (unsigned)0xC3  // Read Serial Number

#define FM25Vxx_CMD_BULK     (unsigned)0xC7  // Bulk   Erase FULL
#define FM25Vxx_CMD_SEER     (unsigned)0xD8  // Sector Erase 64k
#define FM25Vxx_CMD_SER      (unsigned)0x20  // Sector Erase  4k

//Status Register
#define FM25Vxx_STA_WEL      (byte)0x02      // Write Enable Bit
#define FM25Vxx_STA_BP0      (byte)0x04      // Block Protect bit ??
#define FM25Vxx_STA_BP1      (byte)0x08      // Block Protect bit ??
#define FM25Vxx_STA_WPEN     (byte)0x80      // Write Protect Pin Enable bit


static	struct rt_spi_device*   rt_spi_device;
static  rt_uint8_t fram_init_complete_flag = 0;

static rt_uint8_t fm25vxx_read_status(void)
{
    return rt_spi_sendrecv8(rt_spi_device,FM25Vxx_CMD_RDSR);
}

static void fm25vxx_wait_busy(void)
{
    while( fm25vxx_read_status() & (0x01));
}

static rt_err_t fm25vxx_read_id(void)
{
    rt_uint8_t cmd;
    rt_uint8_t id_recv[9];

    cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
    rt_spi_send(rt_spi_device, &cmd, 1);

    cmd = FM25Vxx_CMD_WRDI;
    rt_spi_send(rt_spi_device, &cmd, 1);

    /* read flash id */
    cmd = FM25Vxx_CMD_RDID;
    rt_spi_send_then_recv(rt_spi_device, &cmd, 1, id_recv, 9);


    return (rt_uint32_t)(id_recv[6] << 16) | (id_recv[7] << 8) | id_recv[8];
}

rt_size_t fm25vxx_read(rt_off_t offset, rt_uint8_t *buffer, rt_size_t length)
{
    rt_uint8_t send_buffer[3];

    if((offset + length) > 256 * 1024)
        return 0;

    send_buffer[0] = FM25Vxx_CMD_WRDI;
    rt_spi_send(rt_spi_device, send_buffer, 1);

    send_buffer[0] = FM25Vxx_CMD_READ;
    send_buffer[1] = (rt_uint8_t)(offset>>8);
    send_buffer[2] = (rt_uint8_t)(offset);
    rt_spi_send_then_recv(rt_spi_device,
                          send_buffer, 3,
                          buffer, length);

    return length;
}

rt_size_t fm25vxx_write(rt_off_t offset, const rt_uint8_t *buffer, rt_size_t length)
{
    rt_uint8_t send_buffer[3];
    rt_uint8_t *write_ptr ;
    rt_size_t   write_size,write_total;

    if((offset + length) > 32 * 1024)
        return 0;

    send_buffer[0] = FM25Vxx_CMD_WREN;
    rt_spi_send(rt_spi_device, send_buffer, 1);
    fm25vxx_wait_busy(); // wait erase done.

    write_size  = length;
    write_total = 0;
    write_ptr   = (rt_uint8_t *)buffer;
    while(write_total < length)
    {
        send_buffer[0] = FM25Vxx_CMD_WREN;
        rt_spi_send(rt_spi_device, send_buffer, 1);

        //write first page...
        send_buffer[0] = FM25Vxx_CMD_WRITE;
        send_buffer[1] = (rt_uint8_t)(offset >> 8);
        send_buffer[2] = (rt_uint8_t)(offset);

        rt_spi_send_then_send(rt_spi_device,
                              send_buffer, 3,
                              write_ptr + write_total, write_size);
        fm25vxx_wait_busy();


        offset      += write_size;
        write_total += write_size;
    }

    send_buffer[0] = FM25Vxx_CMD_WRDI;
    rt_spi_send(rt_spi_device, send_buffer, 1);



    return length;
}

rt_err_t fram_init_complete(void)
{
	if(fram_init_complete_flag == 1)
	{
		return RT_EOK;
	}
	else
	{
		return RT_ERROR;
	}
}


rt_err_t fm25vxx_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;
    rt_uint32_t id;
    rt_uint8_t  send_buffer[3];
    rt_uint8_t status=0;
//    struct rt_spi_device*   rt_spi_device;

    rt_spi_device = (struct rt_spi_device*) rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        FRAM_TRACE("spi device %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 20 * 1000 * 1000; /* 20 */
		    rt_spi_configure(rt_spi_device, &cfg);

    }

    id = fm25vxx_read_id();
	  FRAM_TRACE("fm25vxx device ID:%o !\r\n", id);	
		
		status = fm25vxx_read_status();             //检测写保护
		FRAM_TRACE("read status value before write:%d\r\n",status);
		
//		    send_buffer[0] = FM25Vxx_CMD_WREN;
//        rt_spi_send(rt_spi_device, send_buffer, 1);		

//		    send_buffer[0] = FM25Vxx_CMD_WRSR;
//		    send_buffer[1] = 0x00;
//		    rt_spi_send(rt_spi_device, send_buffer, 2);	

////		    send_buffer[0] = FM25Vxx_CMD_WREN;
////        rt_spi_send(rt_spi_device, send_buffer, 1);		
////		
//		    send_buffer[0] = FM25Vxx_CMD_WRSR;
//		    send_buffer[1] = 0x02;
//		    rt_spi_send(rt_spi_device, send_buffer, 2);	
//		
//       send_buffer[0] = FM25Vxx_CMD_WRDI;
//       rt_spi_send(rt_spi_device, send_buffer, 1);		    
		
		
		
		status = fm25vxx_read_status();             //检测写保护
		if((status&0x8e)!=0) FRAM_TRACE("all write protect status value:%d\r\n",status);
		else FRAM_TRACE("write protect value:%d\r\n",status);
		
	  rt_uint8_t date_test[]="hello world!";
		if(fm25vxx_write(0x00,date_test,13))
		{
			fm25vxx_write(0x20,date_test,13);
			rt_uint8_t str_test[15];
			fm25vxx_read(0x00,(rt_uint8_t* )str_test,13);
			FRAM_TRACE("test date is: hello world! \r\n the read-write date is:%s  \r\n", str_test);	
		}
		else FRAM_TRACE("write date error\r\n");
		
		fram_init_complete_flag = 1;

#ifdef test_fm25v02
		
		rt_uint8_t reference_array[512]; 
		rt_uint8_t test_array[512]; 
		rt_uint8_t err_count=0;
		
		for (int index = 0; index < 256; index++) 
		{
			test_array[index] = index ;              //写入测试数据
		}
		
	  for (int index = 0; index < 4; index++) 
		{
			
			fm25vxx_write(index*255,test_array,100);

		}
		
	  for (int index = 0; index < 4; index++) 
		{			
			fm25vxx_read(index*255,reference_array,100);
			if (memcmp(reference_array, test_array, 100) != 0)
			{
				err_count++;
				FRAM_TRACE("large date write error:%d\r\n",index);
//				break;
			}
			for(int indey = 0; indey <100; indey++)
			{
				FRAM_TRACE(" %d \t",reference_array[indey]);
			}
    }
		
#endif
		
_error_exit:
    return result;
}

int rt_fram_init(void)
{
    stm32_spi_bus_attach_device(FRAM_CS_PIN, "spi2", "framspi");

    return fm25vxx_init("framspi");
}
INIT_DEVICE_EXPORT(rt_fram_init);
