#include <rtthread.h>
#include <rtdevice.h>

#include "gps.h"

#include "delay.h"

#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>

#include <uORB/topics/vehicle_gps_position.h>

#define SERIAL_BAUD   9600
#define GPS_BAUD      9600
#define N_FLOATS      4
#define SAMPLE_UART_NAME   "uart4" 


#define GPS_DEBUG

#ifdef GPS_DEBUG
#define GPS_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* #ifdef GPS_DEBUG */

static rt_device_t serial; 
static struct vehicle_gps_position_s  _vehicle_gps;


Ublox M8_Gps;
// Altitude - Latitude - Longitude - N Satellites
float gpsArray[N_FLOATS] = {0, 0, 0, 0};

static struct rt_semaphore rx_sem; ///* 用于接收消息的信号量 */


void RingBuf_Write(char data);
char RingBuf_Read(char* pData);



/**
* @brief 写一个字节到环形缓冲区
* @param data：待写入的数据
* @return none
*/
void RingBuf_Write(char data)
{
  buffer.ringBuf[buffer.tailPositon]=data;     //从尾部追加
  if(++buffer.tailPositon>=BUFFER_MAX)         //尾节点偏移
    buffer.tailPositon=0;                      //大于数组最大长度 归零 形成环形队列
  if(buffer.tailPositon == buffer.headPosition)//如果尾部节点追到头部节点，则修改头节点偏移位置丢弃早期数据
    if(++buffer.headPosition>=BUFFER_MAX)
      buffer.headPosition=0;
}

/**
* @brief 读取环形缓冲区的一个字节的数据
* @param *pData:指针，用于保存读取到的数据
* @return 1表示缓冲区是空的，0表示读取数据成功
*/
char RingBuf_Read(char* pData)
{
  if(buffer.headPosition == buffer.tailPositon)    //如果头尾接触表示缓冲区为空
	{
					return 1;                              //返回1，环形缓冲区是空的
	}
  else
  {
    *pData=buffer.ringBuf[buffer.headPosition];    //如果缓冲区非空则取头节点值并偏移头节点
    if(++buffer.headPosition>=BUFFER_MAX)
      buffer.headPosition=0;
    return 0;                                      //返回0，表示读取数据成功
  }
}



static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
//	rt_sem_release(&rx_sem);
	char c;
  rt_off_t pos;  
  rt_device_read(serial , pos , &c ,1);
	RingBuf_Write(c);	                                //向环形队列写数据
	return RT_EOK;
}



void setup() 
{
//   Serial.begin(SERIAL_BAUD);
//   Serial1.begin(GPS_BAUD);
 
}

void loop(char c) 
{
//   if(RingBuf_Read())
//		return;
	char date=0;
  while(!RingBuf_Read(&date))
	{
//		M8_Gps.encode(date);
		 GPS_TRACE("%c",date);
		 if (M8_Gps.encode(date)) 
		 {
				gpsArray[0] = M8_Gps.altitude;     //高度  GNGGA
				gpsArray[1] = M8_Gps.latitude;     //纬度  GNGGA
				gpsArray[2] = M8_Gps.longitude;    //经度  GNGGA
				gpsArray[3] = M8_Gps.sats_in_use;  //搜到的星数  GNGGA
			
			
				for(rt_uint8_t i = 0; i < N_FLOATS; i++) 
				{
			//    Serial.print(gpsArray[i], 6);Serial.print(" ");
					GPS_TRACE("gpsArray[%d]:%d\r\n",i,(int)gpsArray[i]);
				}	
		 }
	}
	
}



Ublox::Tokeniser::Tokeniser(char* _str, char _token)
{
    str = _str;
    token = _token;
}


bool Ublox::Tokeniser::next(char* out, int len)
{
    uint8_t count = 0;

    if(str[0] == 0)
        return false;

    while(true)
    {
        if(str[count] == '\0')
        {
            out[count] = '\0';
            str = &str[count];
            return true;
        }

        if(str[count] == token)
        {
            out[count] = '\0';
            count++;
            str = &str[count];
            return true;
        }

        if(count < len)
            out[count] = str[count];

        count++;
    }
    return false;
}


bool Ublox::encode(char c)
{
    buf[pos] = c;
    pos++;

    if(c == '\n') //linefeed
    {
        bool ret = process_buf();
        memset(buf, '\0', 120);
        pos = 0;
			  
        return ret;
    }

    if(pos >= 120) //avoid a buffer overrun
    {
        memset(buf, '\0', 120);
        pos = 0;
    }
    return false;
}


bool Ublox::process_buf()
{
    if(!check_checksum()) //if checksum is bad
    {
        return false; //return
    }

    //otherwise, what sort of message is it
    if(strncmp(buf, "$GNGGA", 6) == 0)
    
    {
        read_gga();
    }
    if(strncmp(buf, "$GNGSA", 6) == 0)
    {
        read_gsa();
    }

    if(strncmp(buf, "$GPGSV", 6) == 0)
    {
        read_gsv();
    }

    if(strncmp(buf, "$GNRMC", 6) == 0)
    
    {
        read_rmc();
    }
    if(strncmp(buf, "$GNVTG", 6) == 0)
    {
        read_vtg();
    }
    return true;
}

// GNGGA 
void Ublox::read_gga()
{
    int counter = 0;
    char token[20];
    Tokeniser tok(buf, ',');

    while(tok.next(token, 20))
    {
        switch(counter)
        {
        case 1: //time
        {
            float time = atof(token);
            int hms = int(time);

            datetime.millis = time - hms;
            datetime.seconds = fmod(hms, 100);
            hms /= 100;
            datetime.minutes = fmod(hms, 100);
            hms /= 100;
            datetime.hours = hms;

            time_age = hrt_absolute_time_us();
        }
        break;
        case 2: //latitude
        {
            float llat = atof(token);
            int ilat = llat/100;
            double mins = fmod(llat, 100);
            latitude = ilat + (mins/60);
        }
        break;
        case 3: //north/south
        {
            if(token[0] == 'S')
                latitude = -latitude;
        }
        break;
        case 4: //longitude
        {
            float llong = atof(token);
            int ilat = llong/100;
            double mins = fmod(llong, 100);
            longitude = ilat + (mins/60);
        }
        break;
        case 5: //east/west
        {
            if(token[0] == 'W')
                longitude = -longitude;
            latlng_age = hrt_absolute_time_us();
        }
        break;
        case 6:
        {
            fixtype = _fixtype(atoi(token));
        }
        break;
        case 7:
        {
            sats_in_use = atoi(token);
        }
        break;
        case 8:
        {
            hdop = atoi(token);
        }
        break;
        case 9:
        {
            float new_alt = atof(token);
            vert_speed = (new_alt - altitude)/((hrt_absolute_time_us()-alt_age)/1000.0);
            altitude = atof(token);
            alt_age = hrt_absolute_time_us();
        }
        break;
        }
        counter++;
    }
		//GPS_TRACE("hour:%d minute:%d second:%d\r\n",datetime.hours,datetime.minutes,datetime.seconds);
		//GPS_TRACE("speet:%d\r\n",(int)vert_speed);
}


void Ublox::read_gsa()
{
    int counter = 0;
    char token[20];
    Tokeniser tok(buf, ',');

    while(tok.next(token, 20))
    {
        switch(counter)
        {
        case 1: //operating mode
        {
            if(token[0] == 'A')
                op_mode = MODE_AUTOMATIC;
            if(token[0] == 'M')
                op_mode = MODE_MANUAL;
        }
        break;
        case 2:
        {
            fix = _fix(atoi(token));
            fix_age = hrt_absolute_time_us();
        }
        break;
        case 14:
        {
            pdop = atof(token);
        }
        break;
        case 15:
        {
            hdop = atof(token);
        }
        break;
        case 16:
        {
            vdop = atof(token);
            dop_age = hrt_absolute_time_us();
        }
        break;
        }
        counter++;
    }
		//GPS_TRACE("fix_age:%d\r\n",(int)fix_age);
}


void Ublox::read_gsv()
{
    char token[20];
    Tokeniser tok(buf, ',');

    tok.next(token, 20);
    tok.next(token, 20);

    tok.next(token, 20);
    int mn = atoi(token); //msg number

    tok.next(token, 20);
    sats_in_view = atoi(token); //number of sats

    int8_t j = (mn-1) * 4;
    int8_t i;

    for(i = 0; i <= 3; i++)
    {
        tok.next(token, 20);
        sats[j+i].prn = atoi(token);

        tok.next(token, 20);
        sats[j+i].elevation = atoi(token);

        tok.next(token, 20);
        sats[j+i].azimuth = atoi(token);

        tok.next(token, 20);
        sats[j+i].snr = atoi(token);
    }
    sats_age = hrt_absolute_time_us();
}


void Ublox::read_rmc()
{
    int counter = 0;
    char token[20];
    Tokeniser tok(buf, ',');

    while(tok.next(token, 20))
    {
        switch(counter)
        {
        case 1: //time
        {
            float time = atof(token);
            int hms = int(time);

            datetime.millis = time - hms;
            datetime.seconds = fmod(hms, 100);
            hms /= 100;
            datetime.minutes = fmod(hms, 100);
            hms /= 100;
            datetime.hours = hms;

            time_age = hrt_absolute_time_us();
        }
        break;
        case 2:
        {
            if(token[0] == 'A')
                datetime.valid = true;
            if(token[0] == 'V')
                datetime.valid = false;
        }
        break;
        /*
        case 3:
        {
            float llat = atof(token);
            int ilat = llat/100;
            double latmins = fmod(llat, 100);
            latitude = ilat + (latmins/60);
        }
        break;
        case 4:
        {
            if(token[0] == 'S')
                latitude = -latitude;
        }
        break;
        case 5:
        {
            float llong = atof(token);
            float ilat = llong/100;
            double lonmins = fmod(llong, 100);
            longitude = ilat + (lonmins/60);
        }
        break;
        case 6:
        {
             if(token[0] == 'W')
                longitude = -longitude;
            latlng_age = millis();
        }
        break;
        */
        case 8:
        {
            course = atof(token);
            course_age = hrt_absolute_time_us();
        }
        break;
        case 9:
        {
            uint32_t date = atoi(token);
            datetime.year = fmod(date, 100);
            date /= 100;
            datetime.month = fmod(date, 100);
            datetime.day = date / 100;
            date_age = hrt_absolute_time_us();
        }
        break;
        }
        counter++;
    }
}

void Ublox::read_vtg()
{
    int counter = 0;
    char token[20];
    Tokeniser tok(buf, ',');

    while(tok.next(token, 20))
    {
        switch(counter)
        {
        case 1:
        {
            course = (atof(token)*100);
            course_age = hrt_absolute_time_us();
        }
        break;
        case 5:
        {
            knots = (atof(token)*100);
            knots_age = hrt_absolute_time_us();
        }
        break;
        case 7:
        {
            speed = (atof(token)*100);
            speed_age = hrt_absolute_time_us();
        }
        break;
        }
        counter++;
    }
}


bool Ublox::check_checksum()
{
    if (buf[strlen(buf)-5] == '*')
    {
        uint16_t sum = parse_hex(buf[strlen(buf)-4]) * 16;
        sum += parse_hex(buf[strlen(buf)-3]);

        for (uint8_t i=1; i < (strlen(buf)-5); i++)
            sum ^= buf[i];
        if (sum != 0)
            return false;

        return true;
    }
    return false;
}


uint8_t Ublox::parse_hex(char c)
{
    if (c < '0')
        return 0;
    if (c <= '9')
        return c - '0';
    if (c < 'A')
        return 0;
    if (c <= 'F')
        return (c - 'A')+10;
    return 0;
}


static void gps_publish(orb_advert_t *_gps_adv)
{
	_vehicle_gps.timestamp = hrt_absolute_time_us();

	_vehicle_gps.lat = (int32_t)(M8_Gps.latitude*10000000);
	_vehicle_gps.lon = (int32_t)(M8_Gps.longitude*10000000);
	_vehicle_gps.alt = (int32_t)(M8_Gps.altitude*10000000);
			
	_vehicle_gps.satellites_used = (uint8_t)M8_Gps.sats_in_use;
//	_vehicle_gps.timestamp_time_relative =
//	_vehicle_gps.fix_type = 
	orb_publish (ORB_ID(vehicle_gps_position),_gps_adv,&_vehicle_gps);
}


rt_err_t gps_init()
{
    rt_err_t    result = RT_EOK;
//    rt_int32_t id;
 
	  serial = rt_device_find(SAMPLE_UART_NAME);
    
	  if(serial == RT_NULL)
    {
        GPS_TRACE("serial device %s not found!\r\n",SAMPLE_UART_NAME);
        result = -RT_ENOSYS;			
//        goto _error_exit;
			  return result;
    }
    /* config serial */
    {
        struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 配 置 参 数 */

   			config.baud_rate = BAUD_RATE_9600;
				config.data_bits = DATA_BITS_8;
				config.stop_bits = STOP_BITS_1;
				config.parity = PARITY_NONE;
				rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    }
		rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
		rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
		rt_device_set_rx_indicate(serial, uart_input); 
    GPS_TRACE("serial device init success!\r\n");		
    return RT_EOK;
}

int rt_gps_init(void)
{
    return gps_init();
}
INIT_DEVICE_EXPORT(rt_gps_init);


static void gps_measure()
{
	char c;
	loop(c); 	
}

static void gps_start(void)
{
	orb_advert_t    vehicle_gps_adv;
//	gps_config();
  vehicle_gps_adv=orb_advertise(ORB_ID(vehicle_gps_position),&_vehicle_gps);

	while(1)
	{
		gps_measure();
		gps_publish(&vehicle_gps_adv);
		rt_thread_mdelay(2500);
	}

}


rt_err_t gps_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {

			gps_start();
		}

		if (!strcmp(verb, "stop")) {
			//gps_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//gps_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//gps_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//gps_info(busid);
		}
	}
	

	//gps_usage();
	return 0;
}

MSH_CMD_EXPORT(gps_main,gps)






