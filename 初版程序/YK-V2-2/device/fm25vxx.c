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

/* JEDEC Manufacturer¡¯s ID */
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


static rt_uint8_t fm25vxx_read_status(struct rt_spi_device *device)
{
    return rt_spi_sendrecv8(device, FM25Vxx_CMD_RDSR);
}

static void fm25vxx_wait_busy(struct rt_spi_device *device)
{
    while( fm25vxx_read_status(device) & (0x01));
}

static rt_err_t fm25vxx_read_id(struct rt_spi_device *device)
{
    rt_uint8_t cmd;
    rt_uint8_t id_recv[9];

    cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
    rt_spi_send(device, &cmd, 1);

    cmd = FM25Vxx_CMD_WRDI;
    rt_spi_send(device, &cmd, 1);

    /* read flash id */
    cmd = FM25Vxx_CMD_RDID;
    rt_spi_send_then_recv(device, &cmd, 1, id_recv, 9);


    return (rt_uint32_t)(id_recv[6] << 16) | (id_recv[7] << 8) | id_recv[8];
}

static rt_size_t fm25vxx_read(struct rt_spi_device *device, rt_off_t offset, rt_uint8_t *buffer, rt_size_t length)
{
    rt_uint8_t send_buffer[4];

    if((offset + length) > 256 * 1024)
        return 0;

    send_buffer[0] = FM25Vxx_CMD_WRDI;
    rt_spi_send(device, send_buffer, 1);

    send_buffer[0] = FM25Vxx_CMD_READ;
    send_buffer[1] = (rt_uint8_t)(offset>>16);
    send_buffer[2] = (rt_uint8_t)(offset>>8);
    send_buffer[3] = (rt_uint8_t)(offset);
    rt_spi_send_then_recv(device,
                          send_buffer, 4,
                          buffer, length);

    return length;
}

static rt_size_t fm25vxx_write(struct rt_spi_device *device, rt_off_t offset, const rt_uint8_t *buffer, rt_size_t length)
{
    rt_uint8_t send_buffer[4];
    rt_uint8_t *write_ptr ;
    rt_size_t   write_size,write_total;

    if((offset + length) > 256 * 1024)
        return 0;

    send_buffer[0] = FM25Vxx_CMD_WREN;
    rt_spi_send(device, send_buffer, 1);
    fm25vxx_wait_busy(device); // wait erase done.

    write_size  = 0;
    write_total = 0;
    write_ptr   = (rt_uint8_t *)buffer;
    while(write_total < length)
    {
        send_buffer[0] = FM25Vxx_CMD_WREN;
        rt_spi_send(device, send_buffer, 1);

        //write first page...
        send_buffer[0] = FM25Vxx_CMD_WRITE;
        send_buffer[1] = (rt_uint8_t)(offset >> 16);
        send_buffer[2] = (rt_uint8_t)(offset >> 8);
        send_buffer[3] = (rt_uint8_t)(offset);

        rt_spi_send_then_send(device,
                              send_buffer, 4,
                              write_ptr + write_total, write_size);
        fm25vxx_wait_busy(device);


        offset      += write_size;
        write_total += write_size;
    }

    send_buffer[0] = FM25Vxx_CMD_WRDI;
    rt_spi_send(device, send_buffer, 1);



    return length;
}


rt_err_t fm25vxx_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;
    rt_uint32_t id;
    rt_uint8_t  send_buffer[3];

    struct rt_spi_device*   rt_spi_device;

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

    id = fm25vxx_read_id(rt_spi_device);
	FRAM_TRACE("fm25vxx device ID:%o !\r\n", id);

_error_exit:
    return result;
}

int rt_fram_init(void)
{
    stm32_spi_bus_attach_device(FRAM_CS_PIN, "spi2", "framspi");

    return fm25vxx_init("framspi");
}
INIT_DEVICE_EXPORT(rt_fram_init);
