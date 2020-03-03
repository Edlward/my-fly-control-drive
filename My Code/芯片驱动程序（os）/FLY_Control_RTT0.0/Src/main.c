/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <rtthread.h>


#include "stm32f4_mpu9250.h"
#include "stm32f4_ms5611.h"
#include "mlx90393.h"
#include "STM32_SPI.h"
#include "FM25V10.H"

#include "bmi08x.h"
#include "bmi088.h"
#include "bmi088_stm32.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static struct rt_thread led0_thread;//
static struct rt_thread led1_thread;//
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t rt_led0_thread_stack[128];//
static rt_uint8_t rt_led1_thread_stack[128];//

rt_device RT_Device_SPI1_BUS;

struct bmi08x_dev dev = {
//        .accel_id = BMI_ACCEL_CS_Pin,
//        .gyro_id = BMI_GYRO_CS_Pin,
        .accel_id = 0x01,
        .gyro_id = 0x02,
        .intf = BMI08X_SPI_INTF,
        .read = &stm32_spi_read,//user_spi_read
        .write = &stm32_spi_write,//user_spi_write
        .delay_ms = &HAL_Delay//user_delay_milli_sec
		//.accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ,
		//.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL,
		//.accel_cfg.range = BMI088_ACCEL_RANGE_3G
};

struct bmi08x_int_cfg int_config;
struct bmi08x_data_sync_cfg sync_cfg;
int8_t rslt;
struct bmi08x_sensor_data user_accel_bmi088;
struct bmi08x_sensor_data user_gyro_bmi088;
uint8_t data = 0;
int count2=0;
int init_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void led0_thread_entry(void* parameter);
static void led1_thread_entry(void* parameter);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
////ÖØÓ³Éä´®¿Ú1µ½rt_kprintf
void rt_hw_console_output(const char *str)
{
    /* empty console output */
	
	rt_enter_critical();

	while(*str!='\0')
	{
		if(*str=='\n')
		{
			USART2->DR = (u8)  '\r'; 
			while((USART2->SR&0X40)==0);
		}
		USART2->DR =*str++;
		while((USART2->SR&0X40)==0);	
	}

	rt_exit_critical();
}
static void led0_thread_entry(void* parameter)
{
	RT_Device_SPI1_BUS
	while(1)
	{
		LED2=0;                                 //??: F7 ???????, LED ?????????
		rt_thread_delay(RT_TICK_PER_SECOND/5); //??
		LED2=1;
		rt_thread_delay(RT_TICK_PER_SECOND/5); //??
	}
}

static void led1_thread_entry(void* parameter)
{
	while(1)
	{
		LED1=0;
		rt_thread_delay(RT_TICK_PER_SECOND/2); //
		LED1=1;
		rt_thread_delay(RT_TICK_PER_SECOND/2); //
	}
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//  long temper=0;
//  float temper_rec=0.0;
//  short sig[3]={0};
//  u8 temp=0;
//  s32 tep1;
//  s32 tep2;
//  int8_t err=0;
//  u8 FM25V02_ID[9]= {0};
//  uint FM25V02_SN_ID[8]= {0};
//  u8 i=0;
//  uint32_t status=0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */


  printf("mx_uart4_init");
  /* USER CODE BEGIN 2 */
	rt_thread_init(&led0_thread,                    //?????
					"led0",                         //????,? shell ??????
					led0_thread_entry,              //??????
					RT_NULL,                        //????????
					&rt_led0_thread_stack[0],       //???????
					sizeof(rt_led0_thread_stack),   //?????
					3,                              //??????
					20);                            //?????					
	rt_thread_startup(&led0_thread);                //???? led0_thread,????
	
					rt_thread_init(&led1_thread,    //?????
					"led1",                         //????,? shell ??????
					led1_thread_entry,              //??????
					RT_NULL,                        //????????
					&rt_led1_thread_stack[0],       //???????
					sizeof(rt_led1_thread_stack),   //?????
					3,                              //??????
					20);
	rt_thread_startup(&led1_thread);                //???? led1_thread,????  
  
//  MPU9250_Init();

//  MS5611_Init();
//  MLX90393_Init();
  
//  rslt = bmi088_init(&dev);
//  /* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
//  rslt = bmi08a_soft_reset(&dev);


//  /*! Max read/write length (maximum supported length is 32).
//   To be set by the user */
//  dev.read_write_len = 32;
//  /*set accel power mode */
//  dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
//  rslt = bmi08a_set_power_mode(&dev);

//  dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
//  bmi08g_set_power_mode(&dev);

//  /* API uploads the bmi08x config file onto the device and wait for 150ms
//     to enable the data synchronization - delay taken care inside the function */
//  rslt = bmi088_apply_config_file(&dev);

//  /*assign accel range setting*/
//  dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
//  /*assign gyro range setting*/
//  dev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
//  /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
//  sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;
//  rslt = bmi088_configure_data_synchronization(sync_cfg, &dev);  
//  
//  rslt = bmi088_set_data_sync_int_config(&int_config, &dev);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//  /* USER CODE END WHILE */

//  /* USER CODE BEGIN 3 */
//      
////	  MPU9250_GetTemperatureRawData(&temper);
////	  temper_rec=((temper-21)/333.87)+21;
////	  printf("temper:%f\r\n",temper_rec);
////	  MPU9250_Get3AxisAccelRawData(sig);
////	  printf("Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",sig[0],sig[1],sig[2]);
////	  MPU9250_Get3AxisGyroRawData(sig);
////	  printf("Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",sig[0],sig[1],sig[2]);
////	  MPU9250_Get3AxisMagnetRawData(sig);
////	  printf("Magnet1:%d\t Magnet2:%d\t Magnet3:%d\r\n",sig[0],sig[1],sig[2]);
//     
//	  fm25v10_get_id(FM25V02_ID);
//	  for(i=0;i<9;i++)
//	  {
//		  printf("FM25V02_ID[%d]:%d\r\n",i,FM25V02_ID[i]);
//	  }
//      status=fm25v10_test();
//      printf("status:%d\r\n",status);
////	 Chip_Select(BMI1088_ACCEL); 
////     temp=SPI1_ReadWriteByte(BMI08X_ACCEL_CHIP_ID_REG | BMI08X_SPI_RD_MASK); 
////	 Chip_DeSelect(BMI1088_ACCEL);
////	 printf("ACCEL_id:%d\r\n",temp);
////     HAL_Delay(500);
////	 Chip_Select(BMI1088_GYRO); 
////     temp=SPI1_ReadWriteByte(BMI08X_GYRO_CHIP_ID_REG | BMI08X_SPI_RD_MASK); 
////	 Chip_DeSelect(BMI1088_GYRO);
////	 printf("GYRO_id:%d\r\n",temp);	  
//	//	  temp=MPU9250_SPIx_Read(0x00,0x80|MPU9250_WHO_AM_I);
//	//	  printf("temp:%d\r\n",temp);
//	  
////	  MS5611_Cal(&tep2,&tep1);
////	  printf("tep1:%d \t tep2:%d\r\n",tep1,tep2);
////	  MLX90393_ReadPos();
////	  
////	  err=bmi088_get_synchronized_data(&user_accel_bmi088,&user_gyro_bmi088,&dev);
////	  if(err == BMI08X_OK)
////	  {
////		  printf("BIM Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",
////										 user_accel_bmi088.x ,
////										 user_accel_bmi088.y ,
////										 user_accel_bmi088.z );
////		  printf("BIM Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",
////										 user_gyro_bmi088.x ,
////										 user_gyro_bmi088.y ,
////										 user_gyro_bmi088.z );
////	  }		  
////	  else
////	  {
////		  printf("err:%d\r\n",err);
////	  }
//	  LED1 = ~LED1; 
//	  HAL_Delay(500);
//	  
//  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
