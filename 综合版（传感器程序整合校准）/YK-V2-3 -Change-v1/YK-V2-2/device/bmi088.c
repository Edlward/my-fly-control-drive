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
 * File      : bmi088.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include "bmi088.h"
#include "bmi08x.h"
#include "drv_spi.h"
#include <drivers/spi.h>


#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>

#define BMI_DEBUG

#ifdef BMI_DEBUG
#define BMI_TRACE         rt_kprintf
#else
#define BMI_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define BMI088_GRO_CS_PIN 18
#define BMI088_ACC_CS_PIN 7

#define VDD_3V3_SENSORS_EN_Pin 2


static struct sensor_accel_s _accel;
static struct sensor_gyro_s _gyro;

static struct rt_spi_device*   rt_acc_spi_device;
static struct rt_spi_device*   rt_gro_spi_device;

typedef struct bmi08x_sensor_data bmi_data;
struct bmi08x_dev dev;
static hrt_abstime 				getdata_send_time = 0;
/****************************************************************************/
/*! Static Function Declarations
 ****************************************************************************/
/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi08x_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev);
/****************************************************************************/
/**\name        Extern Declarations
 ****************************************************************************/
extern const uint8_t bmi08x_config_file[];
/**\name        Globals
 ****************************************************************************/


rt_uint8_t bmi088_write(struct rt_spi_device *device,
	                                           rt_uint8_t reg_addr,
                                               rt_uint8_t* data,
                                               rt_uint16_t length)
{

	reg_addr &= 0x7f;
	rt_uint8_t cmd = reg_addr ;
	rt_spi_send_then_send(device, &cmd, 1, data, length);	
	return 0;
}

rt_uint8_t bmi088_read(struct rt_spi_device *device,
	                                           rt_uint8_t reg_addr,
                                               rt_uint8_t* data,
                                               rt_uint16_t length)
{
	reg_addr |= 0x80;
	rt_uint8_t cmd =  reg_addr ;
	rt_spi_send_then_recv(device, &cmd, 1, data, length);
	return 0;
}

/****************************************************************************/
/**\name        Function definitions
 ****************************************************************************/
/*!
 *  @brief This API is the entry point for bmi088 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 */
int8_t bmi088dev_init(struct bmi08x_dev *dev)
{
	int8_t rslt;
	/*initialize bmi088 accel sensor*/
	rslt = bmi08a_init(dev);

	if (rslt == BMI08X_OK) {
		/*initialize bmi088 gyro sensor*/
		rslt = bmi08g_init(dev);
	}

	return rslt;
}

/*!
 *  @brief This API uploads the bmi088 config file onto the device.
 */
int8_t bmi088_apply_config_file(struct bmi08x_dev *dev)
{
	int8_t rslt;
	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		/* Assign stream file */
		dev->config_file_ptr = bmi08x_config_file;
		/* Upload binary */
		rslt = bmi08a_write_config_file(dev);
	}

	return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the data synchronization
 *  feature.
 */
int8_t bmi088_configure_data_synchronization(struct bmi08x_data_sync_cfg sync_cfg, struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint16_t data[BMI08X_ACCEL_DATA_SYNC_LEN];
	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		/* change sensor meas config */
		switch (sync_cfg.mode) {
		case BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ:
			dev->accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
			dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
			dev->gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
			dev->gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
			break;
		case BMI08X_ACCEL_DATA_SYNC_MODE_1000HZ:
			dev->accel_cfg.odr = BMI08X_ACCEL_ODR_800_HZ;
			dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
			dev->gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
			dev->gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;
			break;
		case BMI08X_ACCEL_DATA_SYNC_MODE_400HZ:
			dev->accel_cfg.odr = BMI08X_ACCEL_ODR_400_HZ;
			dev->accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
			dev->gyro_cfg.odr = BMI08X_GYRO_BW_47_ODR_400_HZ;
			dev->gyro_cfg.bw = BMI08X_GYRO_BW_47_ODR_400_HZ;
			break;
		default:
			break;
		}
		rslt = bmi08a_set_meas_conf(dev);
		if (rslt != BMI08X_OK)
			return rslt;

		rslt = bmi08g_set_meas_conf(dev);
		if (rslt != BMI08X_OK)
			return rslt;

		/* Enable data synchronization */
		data[0] = (sync_cfg.mode & BMI08X_ACCEL_DATA_SYNC_MODE_MASK);
		rslt = bmi08a_write_feature_config(BMI08X_ACCEL_DATA_SYNC_ADR, &data[0],
				BMI08X_ACCEL_DATA_SYNC_LEN, dev);
	}

	return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the anymotion
 *  feature.
 */
int8_t bmi088_configure_anymotion(struct bmi08x_anymotion_cfg anymotion_cfg, const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint16_t data[BMI08X_ACCEL_ANYMOTION_LEN];
	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if (rslt == BMI08X_OK) {
		/* Enable data synchronization */
		data[0] = (anymotion_cfg.threshold & BMI08X_ACCEL_ANYMOTION_THRESHOLD_MASK);
		data[0] |= ((anymotion_cfg.nomotion_sel << BMI08X_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT) &
					BMI08X_ACCEL_ANYMOTION_NOMOTION_SEL_MASK);
		data[1] = (anymotion_cfg.duration & BMI08X_ACCEL_ANYMOTION_DURATION_MASK);
		data[1] |= ((anymotion_cfg.x_en << BMI08X_ACCEL_ANYMOTION_X_EN_SHIFT) &
					BMI08X_ACCEL_ANYMOTION_X_EN_MASK);
		data[1] |= ((anymotion_cfg.y_en << BMI08X_ACCEL_ANYMOTION_Y_EN_SHIFT) &
					BMI08X_ACCEL_ANYMOTION_Y_EN_MASK);
		data[1] |= ((anymotion_cfg.z_en << BMI08X_ACCEL_ANYMOTION_Z_EN_SHIFT) &
					BMI08X_ACCEL_ANYMOTION_Z_EN_MASK);
		rslt = bmi08a_write_feature_config(BMI08X_ACCEL_ANYMOTION_ADR, &data[0],
					BMI08X_ACCEL_ANYMOTION_LEN, dev);
	}

	return rslt;
}
/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the bmi08x_sensor_data structure instance
 *  passed by the user.
 */
int8_t bmi088_get_synchronized_data(struct bmi08x_sensor_data *accel, struct bmi08x_sensor_data *gyro,
		const struct bmi08x_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr, data[6];
	uint8_t lsb, msb;
	uint16_t msblsb;
	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	/* Proceed if null check is fine */
	if ((rslt == BMI08X_OK) && (accel != NULL) && (gyro != NULL)) {
		/* Read accel x,y sensor data */
		reg_addr = BMI08X_ACCEL_GP_0_REG;
		rslt = bmi08a_get_regs(reg_addr, &data[0], 4, dev);

		if (rslt == BMI08X_OK) {
			/* Read accel sensor data */
			reg_addr = BMI08X_ACCEL_GP_4_REG;
			rslt = bmi08a_get_regs(reg_addr, &data[4], 2, dev);

			if (rslt == BMI08X_OK) {
				lsb = data[0];
				msb = data[1];
				msblsb = (msb << 8) | lsb;
				accel->x = ((int16_t) msblsb); /* Data in X axis */

				lsb = data[2];
				msb = data[3];
				msblsb = (msb << 8) | lsb;
				accel->y = ((int16_t) msblsb); /* Data in Y axis */

				lsb = data[4];
				msb = data[5];
				msblsb = (msb << 8) | lsb;
				accel->z = ((int16_t) msblsb); /* Data in Z axis */

				/* Read gyro sensor data */
				rslt = bmi08g_get_data(gyro, dev);
			}
		}

	} else {
		rslt = BMI08X_E_NULL_PTR;
	}

	return rslt;
}
/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the bmi08x_int_cfg
 *  structure instance.
 */
int8_t bmi088_set_data_sync_int_config(const struct bmi08x_int_cfg *int_config,
	const struct bmi08x_dev *dev)
{
	int8_t rslt;
	/*configure accel sync data ready interrupt configuration*/
	rslt = bmi08a_set_int_config(&int_config->accel_int_config_1, dev);
	if (rslt != BMI08X_OK)
		return rslt;

	rslt = bmi08a_set_int_config(&int_config->accel_int_config_2, dev);
	if (rslt != BMI08X_OK)
		return rslt;

	/*configure gyro data ready interrupt configuration*/
	rslt = bmi08g_set_int_config(&int_config->gyro_int_config_1, dev);
	if (rslt != BMI08X_OK)
		return rslt;

	rslt = bmi08g_set_int_config(&int_config->gyro_int_config_2, dev);

	return rslt;
}

/*****************************************************************************/
/* Static function definition */
/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi08x_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = BMI08X_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = BMI08X_OK;
	}

	return rslt;
}

static rt_uint8_t bmi088_read_id(struct rt_spi_device *device,uint8_t reg_addr)
{
    rt_uint8_t cmd;
    rt_uint8_t id_recv;

    /* read flash id */
    cmd = reg_addr | BMI08X_SPI_RD_MASK;
    rt_spi_send_then_recv(device, &cmd, 1, &id_recv, 1);

    return id_recv;
}

static void bmidev_register(void)
{
	dev.accel_id = rt_acc_spi_device;
	dev.gyro_id = rt_gro_spi_device;
	dev.read = &bmi088_read;
	dev.write = &bmi088_write;
	dev.delay_ms = &time_waitMs;
	dev.intf = BMI08X_SPI_INTF;
	
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_1600_HZ;
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_230_ODR_2000_HZ;
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_500_DPS;
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_230_ODR_2000_HZ;
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	
}

bool sensor_bmi088_ready(void)
{
	uint8_t status;
	hrt_abstime now = hrt_absolute_time_us();
	if(now - getdata_send_time>900)
	{
		return true;
	}else
	{
		return false;
	}
}

rt_err_t sensor_get_bmi088_data(struct bmi08x_sensor_data *accel_bmi088,struct bmi08x_sensor_data *gyro_bmi088)
{
	rt_int8_t rslt;
	
	rslt = bmi08a_get_data(accel_bmi088, &dev);
	
	if(rslt != BMI08X_OK)
	{
		BMI_TRACE("bmi08acc get data Failed !\r\n");
		return rslt;
	}
	rslt = bmi08g_get_data(gyro_bmi088, &dev);
	
	if(rslt != BMI08X_OK)
	{
		BMI_TRACE("bmi08gyro get data Failed !\r\n");
		return rslt;
	}
	getdata_send_time = hrt_absolute_time_us();
	return RT_EOK;
}

static void bmi088_measure(struct bmi08x_sensor_data *accel_bmi088,struct bmi08x_sensor_data *gyro_bmi088)
{	
	rt_int8_t rslt;	
	rslt = bmi08a_get_data(accel_bmi088, &dev);
	if(rslt == BMI08X_OK)
	{

	}
	else
	{
		BMI_TRACE("bmi08acc get data Failed !\r\n");
	}
	rslt = bmi08g_get_data(gyro_bmi088, &dev);
	if(rslt == BMI08X_OK)
	{

	}
	else
	{
		BMI_TRACE("bmi08gro get data Failed !\r\n");
	}		
}



static void bmi088_config()
{
	rt_int8_t rslt;
	bmi08a_soft_reset(&dev);
	bmi08g_soft_reset(&dev);
	dev.delay_ms(5);

	rslt = bmi08a_set_power_mode(&dev);
	dev.delay_ms(10);
	/* Wait for 10ms to switch between the power modes - delay taken care inside the function*/
	rslt = bmi08a_set_meas_conf(&dev);
	dev.delay_ms(50);
	
	rslt = bmi08g_set_power_mode(&dev);
	dev.delay_ms(10);
	/* Wait for 30ms to switch between the power modes - delay taken care inside the function*/
	rslt = bmi08g_set_meas_conf(&dev);
	dev.delay_ms(50);

}
static void bmi088_publish(orb_advert_t *_accel_adv,orb_advert_t *_gyro_adv,bmi_data *accel_data,bmi_data *gyro_data)
{
	_accel.timestamp = hrt_absolute_time_us();
	_accel.x = (float)accel_data->x/32768.0f*3;
	_accel.y = (float)accel_data->y/32768.0f*3;
	_accel.z = (float)accel_data->z/32768.0f*3;
	orb_publish(ORB_ID(sensor_accel),_accel_adv,&_accel);
	
	_gyro.timestamp = hrt_absolute_time_us();
	_gyro.x = (float)gyro_data->x*15.3f/1000.0f;
	_gyro.y = (float)gyro_data->y*15.3f/1000.0f;
	_gyro.z = (float)gyro_data->z*15.3f/1000.0f;
	orb_publish(ORB_ID(sensor_gyro),_gyro_adv,&_gyro);
}

struct rt_spi_device *bmi088_init(const char * spi_device_name)
{
    rt_err_t    result = RT_EOK;
    rt_uint32_t id;
    rt_uint8_t  send_buffer[3];
    struct rt_spi_device* rt_spi_device;
    rt_spi_device = (struct rt_spi_device*) rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        BMI_TRACE("spi device %s not found!\r\n", spi_device_name);
        result = -RT_ENOSYS;

        goto _error_exit;
    }
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 1 * 1000 * 1000; /* 20 */
		rt_spi_configure(rt_spi_device, &cfg);
    }
	if (!strcmp(spi_device_name, "bmiacc")) {
		id = bmi088_read_id(rt_spi_device,BMI08X_ACCEL_CHIP_ID_REG);
		id = bmi088_read_id(rt_spi_device,BMI08X_ACCEL_CHIP_ID_REG);
		BMI_TRACE("bmi088 acc device ID:%o !\r\n", id);
	}
	if (!strcmp(spi_device_name, "bmigro")) {
		id = bmi088_read_id(rt_spi_device,BMI08X_GYRO_CHIP_ID_REG);
		BMI_TRACE("bmi088 gro device ID:%o !\r\n", id);
	}
	
_error_exit:
    return rt_spi_device;
}

int rt_bmi088_init(void)
{
    stm32_spi_bus_attach_device(BMI088_ACC_CS_PIN, "spi1", "bmiacc");
	stm32_spi_bus_attach_device(BMI088_GRO_CS_PIN, "spi1", "bmigro");
    rt_pin_mode(VDD_3V3_SENSORS_EN_Pin, PIN_MODE_OUTPUT);
	rt_pin_write(VDD_3V3_SENSORS_EN_Pin, PIN_HIGH);
	rt_gro_spi_device = bmi088_init("bmigro");
	rt_acc_spi_device = bmi088_init("bmiacc");	
	bmidev_register();
	bmi088dev_init(&dev);
	bmi088_config();
	return RT_EOK;
}
INIT_APP_EXPORT(rt_bmi088_init);



static void bmi088_start(void)
{
	orb_advert_t       sensor_accel_adv;
	orb_advert_t       sensor_gyro_adv;
	bmi_data _accel_bmi;
	bmi_data _gyro_bmi;
	bmi088_config();
	
  sensor_accel_adv=orb_advertise(ORB_ID(sensor_accel),&_accel);
	sensor_gyro_adv=orb_advertise(ORB_ID(sensor_gyro),&_gyro);
	
	while(1)
	{
		bmi088_measure(&_accel_bmi,&_gyro_bmi);
		bmi088_publish(&sensor_accel_adv,&sensor_gyro_adv,&_accel_bmi,&_gyro_bmi);
		rt_thread_mdelay(500);
	}
	
}

static void bmi088_test(void)
{
	
}


rt_err_t bmi088_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			bmi088_start();
			BMI_TRACE("bmi088 start !\r\n");
		}

		if (!strcmp(verb, "stop")) {
			//bmi088_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			bmi088_test();
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//bmi088_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//bmi088_info(busid);
		}
	}
	

	//bmi088_usage();
	return 0;
}

MSH_CMD_EXPORT(bmi088_main,bmi088)