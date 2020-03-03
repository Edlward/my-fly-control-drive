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
 * File      : mlx90393.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */
#include "mlx90393.h"
#include "drv_spi.h"
#include <drivers/spi.h>


#define MLX_DEBUG

#ifdef MLX_DEBUG
#define MLX_TRACE         rt_kprintf
#else
#define MLX_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define MLX90393_CS_PIN 9
#define VDD_3V3_SENSORS_EN_Pin 2

static struct rt_spi_device*   	rt_spi_device;
static struct mlx_txyz_s 		decode_data;
static hrt_abstime 				command_send_time = 0;

const float gain_sel_xy[8] = {0.805,
							0.644,
							0.483,
							0.403,
							0.322,
							0.268,
							0.215,
							0.161
							};

							
const float gain_sel_z[8] = {1.468,
							1.174,
							0.881,
							0.734,
							0.587,
							0.489,
							0.391,
							0.294
							};

const rt_uint32_t T_conv_us[8][4]={	{1270,  1840,  3000,	5300},
									{1460,  2230,  3760,	6840},
									{1840,  3000,  5300,  	9910},
									{2610,  4530,  8370,  	16050},
									{4150,  7600,  14520,  	28340},
									{7220,  13750, 26800,  	52920},
									{13360, 26040, 51380,  	102070},
									{25650, 50610, 100530, 	200370}
								};



static void mlx90393_command(uint8_t cmd, 
							uint8_t zyxt, 
							uint8_t address, 
							uint16_t data,
							uint8_t* mlx90393_rdata_buffer)
{
	rt_uint8_t rx_size = 1;
	rt_uint8_t tx_size = 1;
	rt_uint8_t mlx90393_wdata_buffer[4];
	cmd = cmd&0xf0;
	zyxt = zyxt&0x0f;
	address = address&0x3f;
	switch (cmd)
	{
		case MLX90393_CMD_SB:
		case MLX90393_CMD_SWOC:
		case MLX90393_CMD_SM:
			cmd |= zyxt;
			break;
		case MLX90393_CMD_RM:
			cmd |= zyxt;
			if ((zyxt&MLX90393_T)!=0) 
				rx_size += 2;
			if ((zyxt&MLX90393_X)!=0) 
				rx_size += 2;
			if ((zyxt&MLX90393_Y)!=0) 
				rx_size += 2;
			if ((zyxt&MLX90393_Z)!=0) 
				rx_size += 2;
			break;
		case MLX90393_CMD_RR:
			mlx90393_wdata_buffer[1] = address << 2;
			rx_size += 2;
			tx_size = 2;
			break;
		case MLX90393_CMD_WR:
			mlx90393_wdata_buffer[1] = (data&0xff00) >> 8;
			mlx90393_wdata_buffer[2] = data & 0x00ff;
			mlx90393_wdata_buffer[3] = address << 2;
			tx_size = 4;
			break;
		case MLX90393_CMD_NOP:
		case MLX90393_CMD_EX:
		case MLX90393_CMD_RT:
		case MLX90393_CMD_HR:
		case MLX90393_CMD_HS:
			break;
		default:
			break;
	}
	mlx90393_wdata_buffer[0] = cmd;
	rt_spi_send_then_recv(rt_spi_device,mlx90393_wdata_buffer,tx_size,mlx90393_rdata_buffer,rx_size);
}

static void mlx90393_read_memory(rt_uint8_t address,rt_uint8_t size,rt_uint8_t *dst_data)
{
	for(int i = 0;i < size/2; i++)
	{
		mlx90393_command(MLX90393_CMD_RR,0,address,0,dst_data);
		address++;
		dst_data+=3;
	}
}

static rt_uint8_t mlx90393_read_status(struct rt_spi_device *device)
{
    rt_uint8_t cmd;
    rt_uint8_t status_recv;
	
    /* read flash id */
    cmd = MLX90393_CMD_RT;
    rt_spi_send_then_recv(device, &cmd, 1, &status_recv, 1);

    return status_recv;
}

static void mlx90393_decode_raw(union rev_txyz *raw,mlx_txyz decode_data_raw)
{

	decode_data_raw->t = (raw->word.T-46244.0f)/45.2f+25.0f;

	decode_data_raw->x = raw->word.X*gain_sel_xy[7];

	decode_data_raw->y = raw->word.Y*gain_sel_xy[7];
	
	decode_data_raw->z = raw->word.Z*gain_sel_z[7];
	
	LOG_RAW("1:%f  2:%f  3:%f   4:%f  \n",decode_data.t,decode_data.x,decode_data.y,decode_data.z);

}


static void mlx90393_measure()
{
	rt_uint8_t read_data[9];
	rt_uint8_t read_zyxt = MLX90393_T|MLX90393_X|MLX90393_Y|MLX90393_Z;
	mlx90393_command(MLX90393_CMD_SM,read_zyxt,0,0,0);
	time_waitMs(6);
	mlx90393_command(MLX90393_CMD_RM,read_zyxt,0,0,read_data);
	MLX_TRACE("0:%o  1:%o  2:%o  3:%o  4:%o  5:%o  6:%o  7:%o  8:%o\r\n", read_data[0], read_data[1], read_data[2], read_data[3], read_data[4], read_data[5], read_data[6], read_data[7], read_data[8]);
	union rev_txyz to_txyz;
	to_txyz.rev_byte[0] = read_data[2];
	to_txyz.rev_byte[1] = read_data[1];
	to_txyz.rev_byte[2] = read_data[4];
	to_txyz.rev_byte[3] = read_data[3];
	to_txyz.rev_byte[4] = read_data[6];
	to_txyz.rev_byte[5] = read_data[5];
	to_txyz.rev_byte[6] = read_data[8];
	to_txyz.rev_byte[7] = read_data[7];
	LOG_RAW("1:%d  2:%d  3:%d   4:%d  \n",to_txyz.word.T,to_txyz.word.X,to_txyz.word.Y,to_txyz.word.Z);
	mlx90393_decode_raw(&to_txyz,&decode_data);	
	
}



static void mlx90393_config(void)
{
	rt_uint8_t set_zyxt = MLX90393_T|MLX90393_X|MLX90393_Y|MLX90393_Z;
	rt_uint16_t data = 0;
	
	//set regist address 0x00h
	data = BIST(0)|Z_SERIES(0)|GAIN_SEL(7)|HALLCONF(0x0c);
	mlx90393_command(MLX90393_CMD_WR,0,ADDRESS0,data,RT_NULL);
	
	//set regist address 0x01h
	data = TRIG_INT(0)|COMM_MODE(2)|WOC_FIDD(0)|EXT_TRIG(0x0)|TCMP_EN(0)|BURST_SEL(0)|BURST_DATA_RATE(0);
	mlx90393_command(MLX90393_CMD_WR,0,ADDRESS1,data,RT_NULL);
	
	//set regist address 0x02h
	data = OSR2(0)|RES_XYZ(0)|DIG_FILT(2)|OSR(2);
	mlx90393_command(MLX90393_CMD_WR,0,ADDRESS2,data,RT_NULL);
	
	//set regist address 0x03h
	data = SENS_TC_HT(0)|SENS_TC_LT(0);
	mlx90393_command(MLX90393_CMD_WR,0,ADDRESS3,data,RT_NULL);
	
	//set sm model zyxt
	mlx90393_command(MLX90393_CMD_SM,set_zyxt,0,0,RT_NULL);
	
	//set sm model zyxt
	//mlx90393_command(MLX90393_CMD_WR,0,0,0,RT_NULL);
	
}

rt_bool_t sensor_mlx90393_ready(void)
{
	hrt_abstime now = hrt_absolute_time_us();
	if(now - command_send_time>T_conv_us[2][2])
	{
		//LOG_RAW("1:%lld  2:%lld \n",now,command_send_time);
		return RT_TRUE;
	}
	else
	{
		return RT_FALSE;
	}	
}

rt_err_t sensor_get_mlx90393_data(mlx_txyz get_txyz_data)
{
	static rt_uint8_t d_flag = 0;	
	rt_uint8_t read_zyxt = MLX90393_T|MLX90393_X|MLX90393_Y|MLX90393_Z;
	if(0 == d_flag)
	{		
		mlx90393_command(MLX90393_CMD_SM,read_zyxt,0,0,0);
		command_send_time = hrt_absolute_time_us();
		d_flag = 1;
		return RT_ERROR;
	}
	else 
	{
		rt_uint8_t read_data[9];
		mlx90393_command(MLX90393_CMD_RM,read_zyxt,0,0,read_data);
		union rev_txyz to_txyz;
		to_txyz.rev_byte[0] = read_data[2];
		to_txyz.rev_byte[1] = read_data[1];
		to_txyz.rev_byte[2] = read_data[4];
		to_txyz.rev_byte[3] = read_data[3];
		to_txyz.rev_byte[4] = read_data[6];
		to_txyz.rev_byte[5] = read_data[5];
		to_txyz.rev_byte[6] = read_data[8];
		to_txyz.rev_byte[7] = read_data[7];
		mlx90393_decode_raw(&to_txyz,get_txyz_data);
		mlx90393_command(MLX90393_CMD_SM,read_zyxt,0,0,0);
		command_send_time = hrt_absolute_time_us();
		return RT_EOK;
	}	

}


rt_uint8_t memory_data[12];
static void mlx90393_start(void)
{
	
	mlx90393_config();
	while(1)
	{
		mlx90393_read_memory(ADDRESS0,8,memory_data);		
		mlx90393_measure();
		rt_thread_mdelay(500);
	}
	
}



rt_err_t mlx90393_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;
    rt_uint32_t status;    
    rt_spi_device = (struct rt_spi_device*) rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        MLX_TRACE("spi device %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 20 * 1000 * 1000; /* 20 */
		rt_spi_configure(rt_spi_device, &cfg);

    }

    status = mlx90393_read_status(rt_spi_device);
	MLX_TRACE("mlx90393 device status:%o !\r\n", status);
	mlx90393_config();
_error_exit:
    return result;
}

int rt_mlx90393_init(void)
{
    stm32_spi_bus_attach_device(MLX90393_CS_PIN, "spi1", "mlxspi");
    rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
    return mlx90393_init("mlxspi");
}
INIT_DEVICE_EXPORT(rt_mlx90393_init);



rt_err_t mlx90393_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			mlx90393_start();
		}

		if (!strcmp(verb, "stop")) {
			//mlx90393_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//mlx90393_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//mlx90393_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//mlx90393_info(busid);
		}
	}
	

	//mlx90393_usage();
	return 0;
}

MSH_CMD_EXPORT(mlx90393_main,mlx90393)