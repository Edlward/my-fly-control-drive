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
 * File      : param.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-08-2      HD      first implementation
 */

#include "param.h"
#include "spi_fram_fm25vxx.h"

typedef enum{
	PARAM_PARSE_START = 0,
	PARAM_PARSE_LIST,
	PARAM_PARSE_GROUP_INFO,
	PARAM_PARSE_GROUP_NAME,
	PARAM_PARSE_GROUP,
	PARAM_PARSE_PARAM,
	PARAM_PARSE_PARAM_NAME,
	PARAM_PARSE_PARAM_VAL,
	PARAM_PARSE_PARAM_VAL_CONTENT,
}PARAM_PARSE_STATE;

/* step 3: Define Parameter */
PARAM_GROUP(CALIBRATION) PARAM_DECLARE_GROUP(CALIBRATION) = \
{ \
	PARAM_DEFINE_FLOAT(GYR_X_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(GYR_Y_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(GYR_Z_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(GYR_X_GAIN, 1.0),
	PARAM_DEFINE_FLOAT(GYR_Y_GAIN, 1.0),
	PARAM_DEFINE_FLOAT(GYR_Z_GAIN, 1.0),
	PARAM_DEFINE_UINT32(GYR_CALIB, 0),
	PARAM_DEFINE_FLOAT(ACC_X_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(ACC_Y_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(ACC_Z_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT00, 1.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT01, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT02, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT10, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT11, 1.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT12, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT20, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT21, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT22, 1.0),
	PARAM_DEFINE_UINT32(ACC_CALIB, 0),
	PARAM_DEFINE_FLOAT(MAG_X_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(MAG_Y_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(MAG_Z_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT00, 1.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT01, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT02, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT10, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT11, 1.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT12, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT20, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT21, 0.0),
	PARAM_DEFINE_FLOAT(MAG_TRANS_MAT22, 1.0),
	PARAM_DEFINE_UINT32(MAG_CALIB, 0),
};

PARAM_GROUP(ATT_CONTROLLER) PARAM_DECLARE_GROUP(ATT_CONTROLLER) = \
{ \
	PARAM_DEFINE_FLOAT(ATT_ROLL_P, 0.1), 
	PARAM_DEFINE_FLOAT(ATT_ROLL_RATE_P, 0.05), 
	PARAM_DEFINE_FLOAT(ATT_ROLL_RATE_I, 0.05), 
	PARAM_DEFINE_FLOAT(ATT_ROLL_RATE_D, 0.0005), 
	PARAM_DEFINE_FLOAT(ATT_PITCH_P, 0.1), 
	PARAM_DEFINE_FLOAT(ATT_PITCH_RATE_P, 0.05), 
	PARAM_DEFINE_FLOAT(ATT_PITCH_RATE_I, 0.05), 
	PARAM_DEFINE_FLOAT(ATT_PITCH_RATE_D, 0.0005), 
	PARAM_DEFINE_FLOAT(ATT_YAW_P, 0.1), 
	PARAM_DEFINE_FLOAT(ATT_YAW_RATE_P, 0.2), 
	PARAM_DEFINE_FLOAT(ATT_YAW_RATE_I, 0.05), 
	PARAM_DEFINE_FLOAT(ATT_YAW_RATE_D, 0.0005), 
	PARAM_DEFINE_FLOAT(ATT_ROLLOUT_LIM, 0.5), 
	PARAM_DEFINE_FLOAT(ATT_PITCHOUT_LIM, 0.5), 
	PARAM_DEFINE_FLOAT(ATT_YAWOUT_LIM, 0.2), 
	PARAM_DEFINE_FLOAT(ATT_ROLLR_I_LIM, 0.1), 
	PARAM_DEFINE_FLOAT(ATT_PITCHR_I_LIM, 0.1), 
	PARAM_DEFINE_FLOAT(ATT_YAWR_I_LIM, 0.1), 
};

PARAM_GROUP(ALT_CONTROLLER) PARAM_DECLARE_GROUP(ALT_CONTROLLER) = \
{ \
	PARAM_DEFINE_FLOAT(ALT_P, 1.0),
	PARAM_DEFINE_FLOAT(ALT_RATE_P, 4.0),
	PARAM_DEFINE_FLOAT(ALT_ACC_P, 0.35),
	PARAM_DEFINE_FLOAT(ALT_ACC_I, 0.85),
	PARAM_DEFINE_FLOAT(ALT_ACC_D, 0.002),
	PARAM_DEFINE_FLOAT(ALT_ERR_MIN, -200.0),
	PARAM_DEFINE_FLOAT(ALT_ERR_MAX, 200.0),
	PARAM_DEFINE_FLOAT(VEL_ERR_MIN, -400),
	PARAM_DEFINE_FLOAT(VEL_ERR_MAX, 400),
	PARAM_DEFINE_FLOAT(ACC_ERR_MIN, -600),
	PARAM_DEFINE_FLOAT(ACC_ERR_MAX, 600),
	PARAM_DEFINE_FLOAT(ALT_OUTPUT_MIN, -250.0),
	PARAM_DEFINE_FLOAT(ALT_OUTPUT_MAX, 250.0),
	PARAM_DEFINE_FLOAT(VEL_OUTPUT_MIN, -600),
	PARAM_DEFINE_FLOAT(VEL_OUTPUT_MAX, 600),
	PARAM_DEFINE_FLOAT(ACC_OUTPUT_MIN, 100),
	PARAM_DEFINE_FLOAT(ACC_OUTPUT_MAX, 900),
	PARAM_DEFINE_FLOAT(ACC_I_MIN, -300),
	PARAM_DEFINE_FLOAT(ACC_I_MAX, 300),
	PARAM_DEFINE_INT32(FEEDFORWARD_EN, 1),
	PARAM_DEFINE_INT32(ACC_ERR_LPF_EN, 1),
	PARAM_DEFINE_FLOAT(ACC_ERR_LPF_FREQ, 5), /* LPF CUTOFF FREQ */
};

PARAM_GROUP(ADRC_ATT) PARAM_DECLARE_GROUP(ADRC_ATT) = \
{ \
	PARAM_DEFINE_INT32(ADRC_ENABLE, 1),
	PARAM_DEFINE_INT32(ADRC_MODE, 1),		// MODE1: FULL ADRC		MODE2: PID + LESO
	PARAM_DEFINE_FLOAT(TD_CONTROL_R2, 25.0f),
	PARAM_DEFINE_FLOAT(TD_CONTROL_H2F, 20.0f),
	PARAM_DEFINE_FLOAT(TD_R0, 1000.0f),
	PARAM_DEFINE_FLOAT(NLSEF_R1, 100.0f),
	PARAM_DEFINE_FLOAT(NLSEF_H1F, 50.0f),
	PARAM_DEFINE_FLOAT(NLSEF_C, 0.01f),
	PARAM_DEFINE_FLOAT(NLSEF_KI, 0.05f),
	PARAM_DEFINE_FLOAT(LESO_W, 120.0f),
	PARAM_DEFINE_FLOAT(T_UP, 0.0125f),
	PARAM_DEFINE_FLOAT(T_DOWN, 0.025f),
	PARAM_DEFINE_FLOAT(GAMMA, 0.50f),
	PARAM_DEFINE_FLOAT(B0, 400.0f),
};

/* step 4: Define param list */
param_list_t param_list = { \
	PARAM_DEFINE_GROUP(CALIBRATION),
	PARAM_DEFINE_GROUP(ATT_CONTROLLER),
	PARAM_DEFINE_GROUP(ALT_CONTROLLER),
	PARAM_DEFINE_GROUP(ADRC_ATT),
};

param_list_t read_param_list;
/* Define Parameter End */

static char* TAG = "PARAM";

void param_store(void);


void param_store(void)
{
	rt_uint8_t data[256*3];
	for(int i=0;i<256*3;i++)
	{
		data[i] = i;
	}
	for(int index = 0;index < 3;index++)
	{
		if(fram_init_complete() == RT_EOK)
		{
			fm25vxx_write(0,data,256*3);	
			rt_kprintf("write done");		
		}
		fm25vxx_write(100,data,100);
		fm25vxx_write(100,data,100);
		fm25vxx_write(100,data,100);
	}
	
}

void param_load(void)
{
	rt_uint8_t data[256*3];

		fm25vxx_read(0,data,256*3);
		for(int i=0;i<256*3;i++)
		{
			rt_kprintf(" %d ",data[i]);
		}
		
		rt_kprintf("\n\r");


	//LOG_RAW("read param:%f",read_param_list._param_CALIBRATION.content->GYR_X_OFFSET.val.f);
}

uint8_t param_init(void)
{
	param_load();	
	return 0;
}

rt_err_t param(int argc, char *argv[])
{
	uint8_t group_flag = 0;
	int flag_cnt = 0;
	int param_num;
	if(argc>1)
	{
		const char *verb = argv[1];
		for(int i = 0 ; i < argc-1 ; i++)
		{
			if(strcmp(argv[i+1], "-g") == 0 || strcmp(argv[i+1], "--group") == 0){
				group_flag =1;
			}
			if(argv[i+1][0] == '-')
				flag_cnt++;
		}

		param_num = argc - flag_cnt;
		
		/*
		 * Start/load the driver.
		 */
		if(strcmp(argv[1], "load") == 0){
			param_load();
		}
		if(strcmp(argv[1], "store") == 0){
			param_store();
		}
		if(strcmp(argv[1], "get") == 0 && param_num == 2){

		}
		if(strcmp(argv[1], "get") == 0 && param_num == 3){

		}
		if(strcmp(argv[1], "get") == 0 && param_num == 4){

		}
		if(strcmp(argv[1], "set") == 0 && argc == 5){

		}
	}
	return 0;
}

MSH_CMD_EXPORT(param,param process)
