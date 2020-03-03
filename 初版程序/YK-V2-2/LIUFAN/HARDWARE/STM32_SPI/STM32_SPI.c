#include "STM32_SPI.h"
//#pragma default_section(CODE,"seg_swmy3")

//¼ÙÉè¶¼ÊÇµÍµçÆ½Ñ¡ÖÐ
void Chip_Select(u8 Chip)
{
	switch(Chip)
	{
		case pMPU9250:
			HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_RESET);
		    //¸´Î»ÆäËûÆ¬Ñ¡
		    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,BMI_GYRO_CS_Pin,GPIO_PIN_SET);
		    HAL_GPIO_WritePin(BMI_ACCEL_CS_GPIO_Port,BMI_ACCEL_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(BARO_CS_GPIO_Port,BARO_CS_Pin,GPIO_PIN_SET);
            
		    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port,ACCEL_MAG_CS_Pin,GPIO_PIN_SET);
			break;	
		case MS5611:
			HAL_GPIO_WritePin(BARO_CS_GPIO_Port,BARO_CS_Pin,GPIO_PIN_RESET);
		
		    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,BMI_GYRO_CS_Pin,GPIO_PIN_SET);
		    HAL_GPIO_WritePin(BMI_ACCEL_CS_GPIO_Port,BMI_ACCEL_CS_Pin,GPIO_PIN_SET);
            
            HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
		    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port,ACCEL_MAG_CS_Pin,GPIO_PIN_SET);
			break;		
		case MLX90393:
			HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port,ACCEL_MAG_CS_Pin,GPIO_PIN_RESET);
		
		    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,BMI_GYRO_CS_Pin,GPIO_PIN_SET);
		    HAL_GPIO_WritePin(BMI_ACCEL_CS_GPIO_Port,BMI_ACCEL_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(BARO_CS_GPIO_Port,BARO_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
		    
			break;		
		case BMI1088_ACCEL:
			HAL_GPIO_WritePin(BMI_ACCEL_CS_GPIO_Port,BMI_ACCEL_CS_Pin,GPIO_PIN_RESET);
		
		    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,BMI_GYRO_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(BARO_CS_GPIO_Port,BARO_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
		    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port,ACCEL_MAG_CS_Pin,GPIO_PIN_SET);
			break;

		case BMI1088_GYRO:
			HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,BMI_GYRO_CS_Pin,GPIO_PIN_RESET);
		    
		    HAL_GPIO_WritePin(BMI_ACCEL_CS_GPIO_Port,BMI_ACCEL_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(BARO_CS_GPIO_Port,BARO_CS_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
		    HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port,ACCEL_MAG_CS_Pin,GPIO_PIN_SET);
			break;		
		default :
//			HAL_GPIO_WritePin(GPIOC,MPU_CS_Pin,GPIO_PIN_SET);
			break;	
		
	}
}

void Chip_DeSelect(u8 Chip)
{
	switch(Chip)
	{
		case pMPU9250:
			HAL_GPIO_WritePin(MPU_CS_GPIO_Port,MPU_CS_Pin,GPIO_PIN_SET);
			break;		
		case MS5611:
			HAL_GPIO_WritePin(BARO_CS_GPIO_Port,BARO_CS_Pin,GPIO_PIN_SET);
			break;		
		case MLX90393:
			HAL_GPIO_WritePin(ACCEL_MAG_CS_GPIO_Port,ACCEL_MAG_CS_Pin,GPIO_PIN_SET);
			break;		
		case BMI1088_ACCEL:
			HAL_GPIO_WritePin(BMI_ACCEL_CS_GPIO_Port,BMI_ACCEL_CS_Pin,GPIO_PIN_SET);
			break;	
		case BMI1088_GYRO:
			HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,BMI_GYRO_CS_Pin,GPIO_PIN_SET);
			break;			
		default :
//			HAL_GPIO_WritePin(GPIOC,MPU_CS_Pin,GPIO_PIN_SET);
			break;		
	}
}

///* âûáîð àäðåñà spib */
//void select_spi_slave(uint adr, uint freq)
//{
//  uint b, a;

//  *pSPIBAUDB = fz.fcore / freq;

//  a = adr;

//  ua.fl_sru9313 = 1;

//  /* address demultiplexor bit 0 */
//  b = a % 2;
//  if (b == 0)
//  {
//    SRU(LOW, DAI_PB05_I);
//  }
//  else
//  {
//    SRU(HIGH, DAI_PB05_I);
//  }
//  
//  /* address demultiplexer bit 1 */
//  a = a / 2;
//  b = a % 2;
//  if (b == 0)
//  {
//    SRU(LOW, DAI_PB06_I);
//  }
//  else
//  {
//    SRU(HIGH, DAI_PB06_I);
//  }
//  
//  /* address demultiplexer bit 2 */
//  a = a / 2;
//  b = a % 2;
//  if (b == 0)
//  {
//    SRU(LOW, DAI_PB07_I);
//  }
//  else
//  {
//    SRU(HIGH, DAI_PB07_I);
//  }

//  /* select demultiplexer IC */
//  if (fz.ver1 > 1) {
//    a = a / 2;
//    b = a % 2;
//    if (b == 0)
//    {
//      SRU(LOW, DAI_PB14_I);
//    }
//    else
//    {
//      SRU(HIGH, DAI_PB14_I);
//    }
//  }
//  ua.fl_sru9313 = 0;
//}

///* deselect all SPIB slaves */
//void spib_cs_set_high(void)
//{
//  *pSPIFLGB = (
//	  SPIFLG0 |
//	  SPIFLG1 |
//	  SPIFLG2 |
//	  SPIFLG3 |
//	  DS0EN); // 0x0F01; //ñíÿòèå CS
//}

///* Wait for the SPI TX buffer is empty. */
//void spib_wait_till_tx_buffer_is_empty(void)
//{
//  while ((*pSPISTATB & TXS))
//    asm("nop;");
//}

///* Wait for the SPI External Transaction Complete*/
//void spib_wait_till_external_transaction_complete(void)
//{
//  while (!(*pSPISTATB & SPIFE))
//    asm("nop;");
//}
