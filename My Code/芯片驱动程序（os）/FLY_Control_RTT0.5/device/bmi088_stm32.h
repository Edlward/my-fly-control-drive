/*! \file bmi088_stm32.h
 \STM32 specific SPI functions */

/*********************************************************************/
/* header files */
#include "bmi08x_defs.h"

#include "drv_spi.h"
#include <drivers/spi.h>

extern struct rt_spi_device*   rt_bmi088_gyro_spi_device;
extern struct rt_spi_device*   rt_bmi088_accel_spi_device;



void BMI_Delay_Ms(uint32_t time);

int8_t stm32_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

int8_t stm32_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);

