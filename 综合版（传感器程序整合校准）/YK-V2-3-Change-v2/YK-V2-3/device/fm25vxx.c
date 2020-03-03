/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-16     aozima       the first version
 * 2012-05-06     aozima       can page write.
 * 2012-08-23     aozima       add flash lock.
 * 2012-08-24     aozima       fixed write status register BUG.
 */

#include <stdint.h>
#include <rtdevice.h>


#include "drv_spi.h"
#include <drivers/spi.h>

#include "spi_flash.h"
#include "spi_flash_fm25vxx.h"

#define FLASH_DEBUG

#ifdef FLASH_DEBUG
#define FLASH_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define FRAM_CS_PIN         57

#define PAGE_SIZE           256

/* JEDEC Manufacturer ID */
#define MF_ID           (0xC2)

/* JEDEC Device ID: Memory type and Capacity */
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
#define FM25Vxx_STA_BP0            0x04      // Block Protect bit ??
#define FM25Vxx_STA_BP1            0x08      // Block Protect bit ??
#define FM25Vxx_STA_WPEN     (byte)0x80      // Write Protect Pin Enable bit

#define DUMMY                       (0xFF)

//#define  BP1_MSK  0x08                       
//#define  BP2_MSK  0x04

#define MTC_FM25V02_BV  0x22
#define MTC_FM25V05_BV  0x23
#define MTC_FM25V10_BV  0x24


static struct spi_flash_device  spi_flash_device;

static void flash_lock(struct spi_flash_device * flash_device)
{
    rt_mutex_take(&flash_device->lock, RT_WAITING_FOREVER);
}

static void flash_unlock(struct spi_flash_device * flash_device)
{
    rt_mutex_release(&flash_device->lock);
}

static uint8_t fm25vxx_read_status(void)
{
    return rt_spi_sendrecv8(spi_flash_device.rt_spi_device, FM25Vxx_CMD_RDSR);
}

static void fm25vxx_wait_busy(void)
{
//    while( fm25vxx_read_status() & (0x01));
}

static rt_err_t fm25vxx_read_id()
{
    rt_uint8_t cmd;
    rt_uint8_t id_recv[9];

    cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
    rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

    cmd = FM25Vxx_CMD_WRDI;
    rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

    /* read flash id */
    cmd = FM25Vxx_CMD_RDID;
    rt_spi_send_then_recv(spi_flash_device.rt_spi_device, &cmd, 1, id_recv, 9);


    return (rt_uint32_t)(id_recv[6] << 16) | (id_recv[7] << 8) | id_recv[8];
}

/** \brief read [size] byte from [offset] to [buffer]
 *
 * \param offset uint32_t unit : byte
 * \param buffer uint8_t*
 * \param size uint32_t   unit : byte
 * \return uint32_t byte for read
 *
 */
static uint32_t fm25vxx_read(uint32_t offset, uint8_t * buffer, uint32_t length)
{
    uint8_t send_buffer[4];

    if((offset + length) > 31 * 1024)
        return 0;	
	
    send_buffer[0] = FM25Vxx_CMD_WRDI;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = FM25Vxx_CMD_READ;
    send_buffer[1] = (uint8_t)(offset>>8);
    send_buffer[2] = (uint8_t)(offset);

    rt_spi_send_then_recv(spi_flash_device.rt_spi_device,
                          send_buffer, 3,
                          buffer, length);

    return length;
}

/** \brief write N page on [page]
 *
 * \param page_addr uint32_t unit : byte (4096 * N,1 page = 4096byte)
 * \param buffer const uint8_t*
 * \return uint32_t
 *
 */
uint32_t fm25vxx_page_write(uint32_t page_addr, const uint8_t* buffer)
{
    uint32_t index;
    uint8_t send_buffer[4];

    RT_ASSERT((page_addr&0xFF) == 0); /* page addr must align to 256byte. */

    send_buffer[0] = FM25Vxx_CMD_WREN;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = FM25Vxx_CMD_WRITE;
    send_buffer[1] = (page_addr >> 8);
    send_buffer[2] = (page_addr);
    rt_spi_send_then_send(spi_flash_device.rt_spi_device, send_buffer, 3,
		                                buffer, PAGE_SIZE);

//    fm25vxx_wait_busy(); // wait erase done.

//    for(index=0; index < (PAGE_SIZE / 256); index++)
//    {
//        send_buffer[0] = CMD_WREN;
//        rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

//        send_buffer[0] = CMD_PP;
//        send_buffer[1] = (uint8_t)(page_addr >> 16);
//        send_buffer[2] = (uint8_t)(page_addr >> 8);
//        send_buffer[3] = (uint8_t)(page_addr);

//        rt_spi_send_then_send(spi_flash_device.rt_spi_device,
//                              send_buffer,
//                              4,
//                              buffer,
//                              256);

//        buffer += 256;
//        page_addr += 256;
//        fm25vxx_wait_busy();
//    }

    send_buffer[0] = FM25Vxx_CMD_WRDI;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    return PAGE_SIZE;
}

/* RT-Thread device interface */
static rt_err_t fm25vxx_flash_init(rt_device_t dev)
{
    return RT_EOK;
}



static rt_err_t fm25vxx_flash_open(rt_device_t dev, rt_uint16_t oflag)
{
    uint8_t send_buffer[3];
	
	  uint8_t fm25vxx_status=fm25vxx_read_status();
	  
    flash_lock((struct spi_flash_device *)dev);

    send_buffer[0] = FM25Vxx_CMD_WREN;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = FM25Vxx_CMD_WRSR;
	  send_buffer[1] = (fm25vxx_status&~(FM25Vxx_STA_BP1|FM25Vxx_STA_BP0));      //ËøÄÚ´æ

    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 2);

    fm25vxx_wait_busy();

    flash_unlock((struct spi_flash_device *)dev);

    return RT_EOK;
}

static rt_err_t fm25vxx_flash_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t fm25vxx_flash_control(rt_device_t dev, int cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = spi_flash_device.geometry.bytes_per_sector;
        geometry->sector_count = spi_flash_device.geometry.sector_count;
        geometry->block_size = spi_flash_device.geometry.block_size;
    }

    return RT_EOK;
}

static rt_size_t fm25vxx_flash_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
    flash_lock((struct spi_flash_device *)dev);

    fm25vxx_read(pos*spi_flash_device.geometry.bytes_per_sector,
                buffer,
                size*spi_flash_device.geometry.bytes_per_sector);

    flash_unlock((struct spi_flash_device *)dev);

    return size;
}

static rt_size_t fm25vxx_flash_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
    rt_size_t i = 0;
    rt_size_t block = size;
    const uint8_t * ptr = buffer;

    flash_lock((struct spi_flash_device *)dev);

    while(block--)
    {
        fm25vxx_page_write((pos + i)*spi_flash_device.geometry.bytes_per_sector,
                          ptr);
        ptr += PAGE_SIZE;
        i++;
    }
     
	  
    flash_unlock((struct spi_flash_device *)dev);

    return size;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops fm25vxx_device_ops =
{
    fm25vxx_flash_init,
    fm25vxx_flash_open,
    fm25vxx_flash_close,
    fm25vxx_flash_read,
    fm25vxx_flash_write,
    fm25vxx_flash_control
};
#endif

rt_err_t fm25vxx_init(const char * flash_device_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;

    /* initialize mutex */
    if (rt_mutex_init(&spi_flash_device.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init sd lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        FLASH_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_flash_device.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 20 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_flash_device.rt_spi_device, &cfg);
    }

    /* init flash */
    {
        rt_uint8_t cmd;
        rt_uint8_t id_recv[3];
			  rt_uint32_t  ID_recv;
        uint16_t memory_type_capacity;

        flash_lock(&spi_flash_device);

//        cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
//        rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

//        cmd = CMD_WRDI;
//        rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

//        /* read flash id */
//        cmd = CMD_JEDEC_ID;
//        rt_spi_send_then_recv(spi_flash_device.rt_spi_device, &cmd, 1, id_recv, 3);
        
			  ID_recv=fm25vxx_read_id();
			
        flash_unlock(&spi_flash_device);
        
				
				id_recv[2] = ID_recv&0xFF;
				id_recv[1] = (ID_recv&0xFF00)>>8;
				id_recv[0] = (ID_recv&0xFF0000)>>16;
        if(id_recv[0] != MF_ID)
        {
            FLASH_TRACE("Manufacturers ID error!\r\n");
            FLASH_TRACE("JEDEC Read-ID Data : %02X %02X %02X\r\n", id_recv[0], id_recv[1], id_recv[2]);
            return -RT_ENOSYS;
        }

        spi_flash_device.geometry.bytes_per_sector = 256;
        spi_flash_device.geometry.block_size = 256; /* block erase: 4k */

        /* get memory type and capacity */
        memory_type_capacity = id_recv[1];
//        memory_type_capacity = (memory_type_capacity << 8) | id_recv[2];

        if(memory_type_capacity == MTC_FM25V02_BV)
        {
            FLASH_TRACE("FM25V02BV detection\r\n");
            spi_flash_device.geometry.sector_count = 128;
        }
        else if(memory_type_capacity == MTC_FM25V05_BV)
        {
            FLASH_TRACE("FM25V05 detection\r\n");
            spi_flash_device.geometry.sector_count = 256;
        }
        else if(memory_type_capacity == MTC_FM25V10_BV)
        {
            FLASH_TRACE("FM25V10 detection\r\n");
            spi_flash_device.geometry.sector_count = 512;
        }
//        else if(memory_type_capacity == MTC_W25Q32_BV)
//        {
//            FLASH_TRACE("W25Q32BV detection\r\n");
//            spi_flash_device.geometry.sector_count = 1024;
//        }
//        else if(memory_type_capacity == MTC_W25Q32_DW)
//        {
//            FLASH_TRACE("W25Q32DW detection\r\n");
//            spi_flash_device.geometry.sector_count = 1024;
//        }
//        else if(memory_type_capacity == MTC_W25Q16_BV_CL_CV)
//        {
//            FLASH_TRACE("W25Q16BV or W25Q16CL or W25Q16CV detection\r\n");
//            spi_flash_device.geometry.sector_count = 512;
//        }
//        else if(memory_type_capacity == MTC_W25Q16_DW)
//        {
//            FLASH_TRACE("W25Q16DW detection\r\n");
//            spi_flash_device.geometry.sector_count = 512;
//        }
//        else if(memory_type_capacity == MTC_W25Q80_BV)
//        {
//            FLASH_TRACE("W25Q80BV detection\r\n");
//            spi_flash_device.geometry.sector_count = 256;
//        }
        else
        {
            FLASH_TRACE("Memory Capacity error!\r\n");
            return -RT_ENOSYS;
        }
    }

    /* register device */
    spi_flash_device.flash_device.type    = RT_Device_Class_Block;
#ifdef RT_USING_DEVICE_OPS
    spi_flash_device.flash_device.ops     = &fm25vxx_device_ops;
#else
    spi_flash_device.flash_device.init    = fm25vxx_flash_init;
    spi_flash_device.flash_device.open    = fm25vxx_flash_open;
    spi_flash_device.flash_device.close   = fm25vxx_flash_close;
    spi_flash_device.flash_device.read    = fm25vxx_flash_read;
    spi_flash_device.flash_device.write   = fm25vxx_flash_write;
    spi_flash_device.flash_device.control = fm25vxx_flash_control;
#endif
    /* no private */
    spi_flash_device.flash_device.user_data = RT_NULL;

    rt_device_register(&spi_flash_device.flash_device, flash_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}

int rt_fm25vxx_init(void)
{
	
    stm32_spi_bus_attach_device(FRAM_CS_PIN, "spi2", "spi20");
//	rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
//	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
    return fm25vxx_init("fm25v02","spi20");
}
//INIT_APP_EXPORT(rt_fm25vxx_init);





