/* 
 * File:   SPI.h
 * Author: kpa
 *
 * Created on June 30, 2016, 2:02 PM
 */

#ifndef __STM32_SPI_H
#define __STM32_SPI_H


#include "stm32f4xx_hal.h"
#include "sys.h"

//#include <Cdef21489.h>
//#include <sru21489.h>
//#include "struct.h"
//#include "mydef.h"

#define Delay_Ms(n)   HAL_Delay(n);


#ifdef __cplusplus
extern "C"
{
#endif

enum SPI1_CS_SIGAL {
    pMPU9250 = 0,
    MS5611,
    MLX90393,
    BMI1088_ACCEL,
	BMI1088_GYRO,
//	BARO,
};	
	
void Chip_Select(u8 Chip);
void Chip_DeSelect(u8 Chip);


//void select_spi_slave(uint adr, uint freq); //  выбор адреса spib

///* deselect all SPIB slaves */
//void spib_cs_set_high(void);

///* Wait for the SPI TX buffer is empty. */
//void spib_wait_till_tx_buffer_is_empty(void);

///* Wait for the SPI External Transaction Complete*/
//void spib_wait_till_external_transaction_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* SPI_H */

