/**
 ******************************************************************************
 * @file	  mlx90393
 * @author	Oliver
 * @version V1.0
 * @date	  2018.12.5
 * @brief	 mlx90393的读取
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
/* Includes -------------------------------------------------------------------*/
#include "mlx90393.h"
#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"
//#include "tim.h"
#include "spi.h"
#include "usart.h"
//#include "ctrl.h"
//#include "util.h"
//#include "motorconf.h"
//#include "can.h"
#include "stm32f4xx_hal_spi.h"
#include "STM32_SPI.h"
/* Extern	 variables ---------------------------------------------------------*/
//extern DriverType Driver;
/* Private typedef -----------------------------------------------------------*/
#include "math.h"

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/**
 * 由于使用SPI单线模式，即数据线只有一条，
 * 进行发送和接受信息时，需要配置发送引脚的模式，使不冲突
 */
//#define //SPI_TX_OFF()            \
//	do                            \
//	{                             \
//		GPIOA->MODER &= 0xFFFF3FFF; \
//		GPIOA->MODER |= 0x00000000; \
//	} while (0) // PA7--MOSI输入模式

//#define //SPI_TX_ON()             \
//	do                            \
//	{                             \
//		GPIOA->MODER &= 0xFFFF3FFF; \
//		GPIOA->MODER |= 0x00008000; \
//	} while (0) // PA7--MOSI复用

//#define MLX90393_CS_ENABLE() HAL_GPIO_WritePin(NSS1_GPIO_Port, NSS1_Pin, GPIO_PIN_RESET)
//#define MLX90393_CS_DISABLE() HAL_GPIO_WritePin(NSS1_GPIO_Port, NSS1_Pin, GPIO_PIN_SET)
/* Private	variables ---------------------------------------------------------*/


/* Private	function pototype -------------------------------------------------*/



/**
 * @brief
 * @param  None
 * @retval
 */
uint8_t posture[10];//Z,Y,X,T
// int16_t dataTest1 = 0x0000; //
// uint16_t dataTest = 0x0000; //
static uint8_t status  = 0;
//static uint32_t write[3] ;
//static uint32_t read[2] = {0};
static uint8_t write_buffer[10];


void MLX90393_CS_ENABLE(void)
{
	Chip_Select(MLX90393);
}

void MLX90393_CS_DISABLE(void)
{
	Chip_DeSelect(MLX90393);
}

void MLX90393_ReadStatus(void)
{
//	MLX90393_CS_ENABLE();
//	status = 0;
////	write_buffer[0] = SM|X_AXIS;
//	write_buffer[0] = WR;
//	write_buffer[1] = ((0x0C|GAIN_SEL(0))&0xFF00)>>8;
//	write_buffer[2] = (0x0C|GAIN_SEL(0))&0x00FF;
//	write_buffer[3] = ADDRESS0  << 2;
//	//SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
//	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,3);
//	MLX90393_CS_DISABLE();
//	
//	////printf("%d\t",status);
//	MLX90393_CS_ENABLE();
//	write_buffer[0] = WR;
//	write_buffer[1] = (((RES(0,0,0)) | (OSR(3)) | DIG_FILT(7))&0xFF00)>>8;
//	write_buffer[2] = (((RES(0,0,0)) | (OSR(3)) | DIG_FILT(7))&0x00FF);
//	write_buffer[3] = ADDRESS2  << 2;	
//	//SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);
//	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,3);
//	MLX90393_CS_DISABLE();
//	
//	////printf("%d\t",status);	
//	MLX90393_CS_ENABLE();
//	write_buffer[0] = WR;
//	write_buffer[1] = (SPI_MODE_ONLY&0xFF00)>>8;
//	write_buffer[2] = (SPI_MODE_ONLY&0x00FF);
//	write_buffer[3] = ADDRESS1  << 2;
//	//SPI_TX_ON();
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[1],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[2],1,4);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[3],1,4);	
//	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
//	MLX90393_CS_DISABLE();
//	////printf("%d\r\n",status);
}

void MLX90393_Init(void)
{
	MLX90393_CS_ENABLE();
//	//SPI_TX_ON();
	//芯片复位 reset
	write_buffer[0] = (RT);
	status=SPI1_ReadWriteByte(write_buffer[0]);
	printf("%d\t",(uint8_t)status);
	//MLX90393_CS_DISABLE();
    
	  //设置增益 0-5  halconf=0x0c
//	MLX90393_CS_ENABLE();
	write_buffer[0] = WR;
	write_buffer[1] = ((0x0C|GAIN_SEL(0))&0xFF00)>>8;
	write_buffer[2] = (0x0C|GAIN_SEL(0))&0x00FF;
	write_buffer[3] = ADDRESS0  << 2;

	
	status=SPI1_ReadWriteByte(write_buffer[0]);
	status=SPI1_ReadWriteByte(write_buffer[1]);
	status=SPI1_ReadWriteByte(write_buffer[2]);
	status=SPI1_ReadWriteByte(write_buffer[3]);	

	HAL_Delay(1);
	
	
	////printf("%d\t",status);
	write_buffer[0] = WR;
	write_buffer[1] = ((SPI_MODE_ONLY|TRIG_INT_SEL)&0xFF00)>>8;
	write_buffer[2] = ((SPI_MODE_ONLY|TRIG_INT_SEL)&0x00FF);
	write_buffer[3] = ADDRESS1  << 2;

	status=SPI1_ReadWriteByte(write_buffer[0]);
	status=SPI1_ReadWriteByte(write_buffer[1]);
	status=SPI1_ReadWriteByte(write_buffer[2]);
	status=SPI1_ReadWriteByte(write_buffer[3]);
	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
	//MLX90393_CS_DISABLE();
	HAL_Delay(1);
	printf("%d\t",status);
//	
	write_buffer[0] = WR;
	write_buffer[1] = (((RES(2,2,2)) | (OSR(2)) | DIG_FILT(2))&0xFF00)>>8;
	write_buffer[2] = (((RES(2,2,2)) | (OSR(2)) | DIG_FILT(2))&0x00FF);
	write_buffer[3] = ADDRESS2  << 2;	
	
	
	//MLX90393_CS_ENABLE();
	//SPI_TX_ON();
	status=SPI1_ReadWriteByte(write_buffer[0]);
	status=SPI1_ReadWriteByte(write_buffer[1]);
	status=SPI1_ReadWriteByte(write_buffer[2]);
	status=SPI1_ReadWriteByte(write_buffer[3]);
	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
	printf("%d\t",status);
	//SendBuf();
	//MLX90393_CS_DISABLE();
	

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS7  << 2;	
	
	//MLX90393_CS_ENABLE();
	//SPI_TX_ON();
	status=SPI1_ReadWriteByte(write_buffer[0]);
	status=SPI1_ReadWriteByte(write_buffer[1]);
	status=SPI1_ReadWriteByte(write_buffer[2]);
	status=SPI1_ReadWriteByte(write_buffer[3]);
	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
	printf("%d\t",status);
	//SendBuf();
	//MLX90393_CS_DISABLE();

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS8  << 2;	
	
	//MLX90393_CS_ENABLE();
	//SPI_TX_ON();
	status=SPI1_ReadWriteByte(write_buffer[0]);
	status=SPI1_ReadWriteByte(write_buffer[1]);
	status=SPI1_ReadWriteByte(write_buffer[2]);
	status=SPI1_ReadWriteByte(write_buffer[3]);
	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
	printf("%d\t",status);
	//SendBuf();
	//MLX90393_CS_DISABLE();	

	write_buffer[0] = WR;
	write_buffer[1] = ((3000)&0xFF00)>>8;
	write_buffer[2] = ((3000)&0x00FF);
	write_buffer[3] = ADDRESS9  << 2;	

	//SPI_TX_ON();
	status=SPI1_ReadWriteByte(write_buffer[0]);
	status=SPI1_ReadWriteByte(write_buffer[1]);
	status=SPI1_ReadWriteByte(write_buffer[2]);
	status=SPI1_ReadWriteByte(write_buffer[3]);
	
	
	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,&status,1,4);
	printf("%d\t",status);
	//SendBuf();

  //启动定时转换模式
  write_buffer[0] = (SB|X_AXIS|Y_AXIS|Z_AXIS|TEMPERATURE);
	status=SPI1_ReadWriteByte(write_buffer[0]);

	//启动阈值唤醒功能
//	write_buffer[0] = (SWOC|X_AXIS|Y_AXIS|Z_AXIS|TEMPERATURE);
//	status=SPI1_ReadWriteByte(write_buffer[0]);

	printf("%d\r\n",(uint8_t)status);
	MLX90393_CS_DISABLE();
	//SendBuf();
}
int a = 0;
void MLX90393_ReadPos(void)
{
 
	uint8_t statusByte;
	static int pos = 0;
//	MLX90393_CS_ENABLE();
//	//SPI_TX_ON();
	//单次转换
//	write_buffer[0] = (SM|0x06);

	MLX90393_CS_ENABLE();
	//读取测量值
	write_buffer[0] = (RM|X_AXIS|Y_AXIS|Z_AXIS|TEMPERATURE);
//	HAL_SPI_Transmit(&hspi1,(uint8_t*)&write_buffer[0],1,4);
	//SPI_TX_OFF();
//	HAL_SPI_Receive(&hspi1,(uint8_t*)&statusByte,1,4);
	status=SPI1_ReadWriteByte(write_buffer[0]);
//	printf("status:%d\r\n",(uint8_t)status);
	for(int i = 0; i < 9; i++)
	{
//		HAL_SPI_Receive(&hspi1,(uint8_t*)&posture[i],1,4);
		posture[i] = SPI1_ReadWriteByte(0);
	}
	MLX90393_CS_DISABLE();
	
//	for(int i = 0; i < 9; i++)
//	{
//        printf("pos[%d]:%d\r\n",i,posture[i]);
//	}
	
//	HAL_SPI_Receive(&hspi1,(uint8_t*)&posture,8,2); 	
//	printf("temper:%d \t x_pos3:%d \t Y_pos2:%d \t z_pos1:%d \r\n",
//						(uint16_t)(posture[2] | (posture[1]<<8)),
//						(uint16_t)(posture[4] | (posture[3]<<8)),
//						(uint16_t)(posture[6] | (posture[5]<<8)),
//						(uint16_t)(posture[8] | (posture[7]<<8)));
//	HAL_Delay(1);

//	//SendBuf();
}

int32_t MLX90393_GetPosX(void)
{
	return (posture[4] << 8 | posture[3]);
}


enum { STATUS_OK = 0, STATUS_ERROR = 0xff } return_status_t;
enum { Z_FLAG = 0x8, Y_FLAG = 0x4, X_FLAG = 0x2, T_FLAG = 0x1 } axis_flag_t;
enum { I2C_BASE_ADDR = 0x0c };
enum { GAIN_SEL_REG = 0x0, GAIN_SEL_MASK = 0x0070, GAIN_SEL_SHIFT = 4 };
enum { HALLCONF_REG = 0x0, HALLCONF_MASK = 0x000f, HALLCONF_SHIFT = 0 };
enum { BURST_SEL_REG = 0x1, BURST_SEL_MASK = 0x03c0, BURST_SEL_SHIFT = 6};
enum { TRIG_INT_SEL_REG = 0x1, TRIG_INT_SEL_MASK = 0x8000, TRIG_INT_SEL_SHIFT = 15 };
enum { EXT_TRIG_REG = 0x1, EXT_TRIG_MASK = 0x0800, EXT_TRIG_SHIFT = 11 };
enum { OSR_REG = 0x2, OSR_MASK = 0x0003, OSR_SHIFT = 0 };
enum { OSR2_REG = 0x2, OSR2_MASK = 0x1800, OSR2_SHIFT = 11 };
enum { DIG_FLT_REG = 0x2, DIG_FLT_MASK = 0x001c, DIG_FLT_SHIFT = 2 };
enum { RES_XYZ_REG = 0x2, RES_XYZ_MASK = 0x07e0, RES_XYZ_SHIFT = 5 };
enum { TCMP_EN_REG = 0x1, TCMP_EN_MASK = 0x0400, TCMP_EN_SHIFT = 10 };
enum { X_OFFSET_REG = 4, Y_OFFSET_REG = 5, Z_OFFSET_REG = 6 };
enum { WOXY_THRESHOLD_REG = 7, WOZ_THRESHOLD_REG = 8, WOT_THRESHOLD_REG = 9 };
enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,
			 POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,
			 RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };


float gain_multipliers[8];
float base_xy_sens_hc0;
float base_z_sens_hc0;
float base_xy_sens_hc0xc;
float base_z_sens_hc0xc;

cache_t  cache;

void MLX90393_init(void)
{


  // gain steps derived from datasheet section 15.1.4 tables
  gain_multipliers[0] = 5.f;
  gain_multipliers[1] = 4.f;
  gain_multipliers[2] = 3.f;
  gain_multipliers[3] = 2.5f;
  gain_multipliers[4] = 2.f;
  gain_multipliers[5] = 1.66666667f;
  gain_multipliers[6] = 1.33333333f;
  gain_multipliers[7] = 1.f;

  // from datasheet
  // for hallconf = 0
  base_xy_sens_hc0 = 0.196f;
  base_z_sens_hc0 = 0.316f;
  // for hallconf = 0xc
  base_xy_sens_hc0xc = 0.150f;
  base_z_sens_hc0xc = 0.242f;
}

 
txyz*  convertRaw(txyzRaw raw)
{
//  const uint8_t gain_sel = (cache.reg[GAIN_SEL_REG] & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
//  const uint8_t hallconf = (cache.reg[HALLCONF_REG] & HALLCONF_MASK) >> HALLCONF_SHIFT;
//  const uint8_t res_xyz = (cache.reg[RES_XYZ_REG] & RES_XYZ_MASK) >> RES_XYZ_SHIFT;

  const uint8_t gain_sel = 0;
  const uint8_t hallconf = 0x0c;
  const uint8_t res_xyz = 0x2a;


  const uint8_t res_x = (res_xyz >> 0) & 0x3;
  const uint8_t res_y = (res_xyz >> 2) & 0x3;
  const uint8_t res_z = (res_xyz >> 4) & 0x3;
//  uint8_t tcmp_en = (cache.reg[TCMP_EN_REG] & TCMP_EN_MASK) >> TCMP_EN_SHIFT;

	uint8_t tcmp_en = 0;
  
  txyz data;
  float xy_sens;
  float z_sens;

  switch(hallconf)
 {
		default:
		case 0:
			xy_sens = base_xy_sens_hc0;
			z_sens = base_z_sens_hc0;
			break;
		case 0xc:
			xy_sens = base_xy_sens_hc0xc;
			z_sens = base_z_sens_hc0xc;
			break;
  }

  float gain_factor = gain_multipliers[gain_sel & 0x7];

  if (tcmp_en)
	{
    data.x = ( (raw.x - 32768.f) * xy_sens *
               gain_factor * (1 << res_x) );
  }
	else 
 {
    switch(res_x)
	 {
			case 0:
			case 1:
				data.x = (int16_t)(raw.x) * xy_sens * gain_factor * (1 << res_x);
				break;
			case 2:
				data.x = ( (raw.x - 32768.f) * xy_sens *
									 gain_factor * (1 << res_x) );
				break;
			case 3:
				data.x = ( (raw.x - 16384.f) * xy_sens *
									 gain_factor * (1 << res_x) );
				break;
    }
  }

  if (tcmp_en)
	{
    data.y = ( (raw.y - 32768.f) * xy_sens *
               gain_factor * (1 << res_y) );
  }
	else 
	{
    switch(res_y)
		{
			case 0:
			case 1:
				data.y = (int16_t)(raw.y) * xy_sens * gain_factor * (1 << res_y);
				break;
			case 2:
				data.y = ( (raw.y - 32768.f) * xy_sens *
									 gain_factor * (1 << res_y) );
				break;
			case 3:
				data.y = ( (raw.y - 16384.f) * xy_sens *
									 gain_factor * (1 << res_y) );
				break;
    }
  }

  if (tcmp_en)
	{
    data.z = ( (raw.z - 32768.f) * z_sens *
               gain_factor * (1 << res_z) );
  } 
	else 
	{
    switch(res_z)
		{
			case 0:
			case 1:
				data.z = (int16_t)(raw.z) * z_sens * gain_factor * (1 << res_z);
				break;
			case 2:
				data.z = ( (raw.z - 32768.f) * z_sens *
									 gain_factor * (1 << res_z) );
				break;
			case 3:
				data.z = ( (raw.z - 16384.f) * z_sens *
									 gain_factor * (1 << res_z) );
				break;
    }
  }

  data.t = 25 + (raw.t - 46244.f)/45.2f;
  return &data;
}


int32_t MLX90393_Getdata_main(void)
{
	txyz *rec_data;
	txyzRaw send_data;
	float Xmax=-2000,Xmin=2000,Ymax=-2000,Ymin=2000,Zmax,Zmin;
	float Xoffset,Yoffset,Zoffset;
	float XH,YH;
	float sec;
	int i;
	MLX90393_Init();
	MLX90393_init();
//  for( i = 0 ; i < 100 ; i++)
//	{
//		MLX90393_ReadPos();
//		send_data.t=		    (posture[2] + (posture[1]*256));
//		send_data.x=				(posture[4] + (posture[3]*256));
//		send_data.y=				(posture[6] + (posture[5]*256));
//		send_data.z=				(posture[8] + (posture[7]*256));	

//		rec_data = convertRaw(send_data );

//		if(Xmin > rec_data->x) Xmin = rec_data->x;
//		if(Xmax < rec_data->x) Xmax = rec_data->x;
//		
//		if(Ymin > rec_data->y) Ymin = rec_data->y;
//		if(Ymax < rec_data->y) Ymax = rec_data->y;
//		HAL_Delay (100);
//  }
//  Xoffset=(Xmax+Xmin)/2;
//	Yoffset=(Ymax+Ymin)/2;	

	for(;;)
	{
		MLX90393_ReadPos();
		send_data.t=	(posture[2] + (posture[1]*256));
		send_data.x=	(posture[4] + (posture[3]*256));
		send_data.y=	(posture[6] + (posture[5]*256));
		send_data.z=	(posture[8] + (posture[7]*256));	
    
		printf("original:-> temper:%f \t x_pos3:%f \t Y_pos2:%f \t z_pos1:%f \r\n",
		                       send_data.t,send_data.x,send_data.y,send_data.z);
			
		
		
		rec_data = convertRaw(send_data );
		printf("convert:->temper:%f \t x_pos3:%f \t Y_pos2:%f \t z_pos1:%f \r\n",
		                       rec_data->t,rec_data ->x,rec_data ->y,rec_data ->z);
	
		XH=rec_data->x - Xoffset;
		YH=rec_data->y - Yoffset;
    
    sec = atan(YH/XH);		
		printf("fanwei:%f\r\n",sec);
//		printf("Xmin:%f \t Xmax:%f\r\n",Xmin,Xmax);
//		printf("Ymin:%f \t Ymax:%f\r\n",Ymin,Ymax);
//		Zoffset=（Zmax+Zmin）/2;	
		
		HAL_Delay (500);
	}
	return 0;
	
	
//	return (posture[4] << 8 | posture[3]);
}


/***COPY RIGHT: ACTION 2019*******************************************************************************/




