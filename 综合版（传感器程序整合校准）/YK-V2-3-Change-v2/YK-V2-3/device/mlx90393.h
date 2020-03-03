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
 * File      : mlx90393.h
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#ifndef _MLX90393_H
#define _MLX90393_H

#include "common.h"

// Key commands.

#define MLX90393_CMD_NOP  	(0x00)  /* No OPeration */
#define MLX90393_CMD_SB  	(0x10)  /* Start Burst mode */
#define MLX90393_CMD_SWOC  	(0x20)  /* Start Wake-up On Change */
#define MLX90393_CMD_SM  	(0x30)  /* Start Measurement (polling mode) */
#define MLX90393_CMD_RM  	(0x40)  /* Read Measurement */
#define MLX90393_CMD_RR  	(0x50)  /* Read from a Register */
#define MLX90393_CMD_WR  	(0x60)  /* Write from a Register */
#define MLX90393_CMD_EX  	(0x80)  /* EXit */
#define MLX90393_CMD_HR  	(0xd0)  /* Memory Recall */
#define MLX90393_CMD_HS  	(0xe0)  /* Memory Store */
#define MLX90393_CMD_RT  	(0xf0)  /* Reset */



// Flags to use with "zyxt" variables.
#define MLX90393_T  (0x01)  /* Temperature */
#define MLX90393_X  (0x02)  /* X-axis */
#define MLX90393_Y  (0x04)  /* Y-axis */
#define MLX90393_Z  (0x08)  /* Z-axis */



// Memory areas.
#define MLX90393_CUSTOMER_AREA_BEGIN  (0x00)
#define MLX90393_CUSTOMER_AREA_END  (0x1f)
#define MLX90393_MELEXIS_AREA_BEGIN  (0x20)
#define MLX90393_MELEXIS_AREA_END  (0x3f)

//MEMORY
/*********************************

**********************************/
#define WRITE_REG_MLX(address,val)  ((WR)|((val&0xff00) << 8) | (val & 0xff) << 16| (address << 26))//((WR)|((val&0xff00) << 8) | (val & 0xff) << 16| (address << 26))
#define READ_REG_MLX(address)        ((RR)| (address << 10))  
//val
#define SPI_MODE_ONLY 0x0400

//address
#define ADDRESS0 0x00
#define BIST(bit_1) 		((bit_1 << 8) & 0x0100)
#define Z_SERIES(bit_1) 	((bit_1 << 7) & 0x0080)
#define GAIN_SEL(bit_3) 	((bit_3 << 4) & 0x0070)
#define HALLCONF(bit_4) 	((bit_4 << 0) & 0x000F)

#define ADDRESS1 0x01
#define TRIG_INT(bit_1) 		((bit_1 << 15) & 0x8000)
#define COMM_MODE(bit_2) 		((bit_2 << 13) & 0x6000)
#define WOC_FIDD(bit_1) 		((bit_1 << 12) & 0x1000)
#define EXT_TRIG(bit_1) 		((bit_1 << 11) & 0x0800)
#define TCMP_EN(bit_1) 			((bit_1 << 10) & 0x0400)
#define BURST_SEL(bit_4) 		((bit_4 << 6) & 0x03C0)
#define BURST_DATA_RATE(bit_6) 	((bit_6 << 0) & 0x003F)

#define ADDRESS2 0x02
#define OSR2(bit_2) 		((bit_2 << 11) & 0x1800)
#define RES_XYZ(bit_6) 		((bit_6 << 5) & 0x07E0)
#define DIG_FILT(bit_3) 		((bit_3 << 2) & 0x001C)
#define OSR(bit_2) 		((bit_2 << 0) & 0x0003)


#define ADDRESS3 0x03
#define SENS_TC_HT(bit_8) 		((bit_8 << 8) & 0xFF00)
#define SENS_TC_LT(bit_8) 		((bit_8 << 0) & 0x00FF)


#define ADDRESS4 0x04
#define ADDRESS5 0x05
#define ADDRESS6 0x06
#define ADDRESS7 0x07
#define ADDRESS8 0x08
#define ADDRESS9 0x09




struct mlx_txyz_s
{
	float t;
	float x;
	float y;
	float z;
};

typedef struct mlx_txyz_s*  mlx_txyz;

union rev_txyz
{
	unsigned char rev_byte[8];
	struct to_word
	{
		rt_uint16_t T;
		rt_int16_t	X;
		rt_int16_t	Y;
		rt_int16_t	Z;
	}word;
};

rt_bool_t sensor_mlx90393_ready(void);								
rt_err_t sensor_get_mlx90393_data(mlx_txyz get_txyz_data);
#endif
