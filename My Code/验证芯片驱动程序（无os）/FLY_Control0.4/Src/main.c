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
struct bmi08x_sensor_data user_accel_bmi088[100];
struct bmi08x_sensor_data user_gyro_bmi088[100];
uint8_t data = 0;
int count2=0;
int count=0;
unsigned int acc_int_count=0;
unsigned int gyr_int_count=0;

unsigned int mpu_int_count=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define mpu9250_test
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  long temper=0;
  float temper_rec=0.0;
  short sig1[100][3]={0};
  short sig2[100][3]={0};
  short sig3[100][3]={0};
  u8 temp=0;
  s32 tep1;
  s32 tep2;
  int8_t err=0;
  u8 FM25V02_ID[9]= {0};
  uint FM25V02_SN_ID[8]= {0};
  u8 i=0;
  uint32_t status=0;
	int16_t xMag,yMag,zMag;
	float tMag;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_UART4_Init();

  /* USER CODE BEGIN 2 */
  while(fm25v10_test());
//  MLX90393_Getdata_main();

  MPU9250_Init();

  MS5611_Init();
  MLX90393_Init();
  
  /* Initialize bmi085 sensors (accel & gyro)*/
  rslt = bmi088_init(&dev);
  /* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */
  rslt = bmi08a_soft_reset(&dev);
//  rslt = bmi08g_soft_reset(&dev);

  /*! Max read/write length (maximum supported length is 32).
   To be set by the user */
  dev.read_write_len = 32;
  /*set accel power mode */
  dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
  rslt = bmi08a_set_power_mode(&dev);

  dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
  bmi08g_set_power_mode(&dev);

  /* API uploads the bmi08x config file onto the device and wait for 150ms
     to enable the data synchronization - delay taken care inside the function */
  rslt = bmi088_apply_config_file(&dev);

  /*assign accel range setting*/
  dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
  /*assign gyro range setting*/
  dev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;
  /*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
  sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;
  rslt = bmi088_configure_data_synchronization(sync_cfg, &dev);


  /*set accel interrupt pin configuration*/
  /*configure host data ready interrupt */
  int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
  int_config.accel_int_config_1.int_type = BMI08X_ACCEL_DATA_RDY_INT;
  int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
  int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
  int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

  /*configure Accel syncronization input interrupt pin */
  int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
  int_config.accel_int_config_2.int_type = BMI08X_ACCEL_DATA_RDY_INT;
  int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
  int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
  int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

  /*set gyro interrupt pin configuration*/
  int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
  int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
  int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
  int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
  int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

  int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
  int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
  int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
  int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
  int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

  /* Enable synchronization interrupt pin */
  rslt = bmi088_set_data_sync_int_config(&int_config, &dev);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    for(count = 0; count<100;count++)
		{
#ifdef bmi088_test		
			err = bmi08g_get_data(&user_gyro_bmi088[count],&dev);
			err = bmi08a_get_data(&user_accel_bmi088[count],&dev);	
#endif
			
#ifdef mpu9250_test
			MPU9250_Get9AxisRawData(&sig1[count][0],&sig2[count][1],&sig3[count][2]);
			
#endif
      HAL_Delay(1);			
		}
		for(count = 0; count<100;count++)
		{
#ifdef bmi088_test			
		  printf("BIM Accel1:\t%d\t Accel2:\t%d\t Accel3:\t%d\r\n",
										 user_accel_bmi088[count].x ,
										 user_accel_bmi088[count].y ,
										 user_accel_bmi088[count].z );
#endif				
#ifdef mpu9250_test
		  printf("Accel1: \t %d \t Accel2: \t%d \t Accel3: \t %d \r\n",
										 sig1[count][0],
										 sig1[count][1] ,
										 sig1[count][2] );			

#endif
		}
		
		for(count = 0; count<100;count++)
		{
#ifdef bmi088_test			
		  printf("BIM Gyro1: \t%d\t Gyro2: \t%d\t Gyro3: \t %d\r\n",
										 user_gyro_bmi088[count].x ,
										 user_gyro_bmi088[count].y ,
										 user_gyro_bmi088[count].z );
#endif
#ifdef mpu9250_test
		  printf("gyro1: \t %d \t gyro2: \t %d \t gyro3: \t %d \r\n",
										 sig2[count][0],
										 sig2[count][1] ,
										 sig2[count][2] );			
#endif			

    }
		
		
#ifdef bmi088_test
    printf("acc :%d \t gyr:%d\r\n",acc_int_count,gyr_int_count);
		acc_int_count =0;
		gyr_int_count =0;
#endif
		
#ifdef mpu9250_test		
    printf("mpu :\t%d \t \r\n",mpu_int_count);
		mpu_int_count =0;
#endif	
		
		
//	  if(err == BMI08X_OK)
//	  {
//		  printf("BIM Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",
//										 user_accel_bmi088.x ,
//										 user_accel_bmi088.y ,
//										 user_accel_bmi088.z );
//		  printf("BIM Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",
//										 user_gyro_bmi088.x ,
//										 user_gyro_bmi088.y ,
//										 user_gyro_bmi088.z );
										 
//	  }		  
//	  else
//	  {
//		  printf("err:%d\r\n",err);
//	  }
	  LED1 = ~LED1; 
//	  HAL_Delay(500);
	  
  }
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
//#define ZERO_RANGE   0.1
//void EXTI0_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI0_IRQn 0 */
//  if(count>1000)
//  {
////   HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//   LED2=~LED2;
//   count=0;
//  }
//  count++;
//  if(init_flag==1)
//  {
//	   bmi088_get_synchronized_data(&user_accel_bmi088,&user_gyro_bmi088, &dev);
//	  
//	   printf("BIM Accel1:%d\t Accel2:%d\t Accel3:%d\r\n",
//									 user_accel_bmi088.x ,
//									 user_accel_bmi088.y ,
//									 user_accel_bmi088.z );
//	   printf("BIM Gyro1:%d\t Gyro2:%d\t Gyro3:%d\r\n",
//									 user_gyro_bmi088.x ,
//									 user_gyro_bmi088.y ,
//									 user_gyro_bmi088.z );
//	  
//	   if((user_accel_bmi088.x>-ZERO_RANGE)&&(user_accel_bmi088.x<ZERO_RANGE))
//	   {
////			(LD1_GPIO_Port,LD1_Pin,GPIO_PIN_SET);
//	   }
//	   else
//	   {
////			(LD1_GPIO_Port,LD1_Pin,GPIO_PIN_RESET);
//	   }
//	   if((user_accel_bmi088.y>-ZERO_RANGE)&&(user_accel_bmi088.y<ZERO_RANGE))
//	   {
////			(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
//	   }
//	   else
//	   {
////			(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
//	   }
//  }

//  /* USER CODE END EXTI0_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
//  /* USER CODE BEGIN EXTI0_IRQn 1 */

//  /* USER CODE END EXTI0_IRQn 1 */
//}
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
