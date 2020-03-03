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
 * File      : sbus.c
 * This file is part of YK-HD
 * COPYRIGHT (C) 2019, YK-HD Develop Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-07-3      HD      first implementation
 */

#include <rtthread.h>
#include <rtdevice.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "sbus.h"
#include "drv_usart.h"
#include "delay.h"
#include <uORB/topics/input_rc.h>

#define LOG_TAG              "example"
#define LOG_LVL              LOG_LVL_ASSERT
#include <ulog.h>


#define SBUS_DEBUG

#ifdef SBUS_DEBUG
#define SBUS_TRACE         rt_kprintf
#else
#define SBUS_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */
#define RC_INPUT_CHANNELS		18
#define SBUS_DEBUG_LEVEL 	0 /* Set debug output level */
#define SBUS_START_SYMBOL	0x0f
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

// testing with a SBUS->PWM adapter shows that
// above 300Hz SBUS becomes unreliable. 333 would
// be the theoretical achievable, but at 333Hz some
// frames are lost
#define SBUS1_MAX_RATE_HZ	300
#define SBUS1_MIN_RATE_HZ	50

// this is the rate of the old code
#define SBUS1_DEFAULT_RATE_HZ	72

#define SBUS_SINGLE_CHAR_LEN_US		(1/((100000/10)) * 1000 * 1000)

#define SBUS_FRAME_INTERVAL_US	2500
#define SBUS_MIN_CALL_INTERVAL_US	(SBUS_FRAME_GAP_US / 3)
#define SBUS_EPSILON_US	2500

/*
  Measured values with Futaba FX-30/R6108SB:
    -+100% on TX:  PCM 1.100/1.520/1.950ms -> SBus raw values: 350/1024/1700  (100% ATV)
    -+140% on TX:  PCM 0.930/1.520/2.112ms -> SBus raw values:  78/1024/1964  (140% ATV)
    -+152% on TX:  PCM 0.884/1.520/2.160ms -> SBus raw values:   1/1024/2047  (140% ATV plus dirty tricks)
*/

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))
	

#define SBUS_UART_NAME       "uart3"      /* 串口设备名称 */
#define BAUD_RATE_100000	 100000

static struct input_rc_s _input_rc;

struct serial_configure sbus_config = {
										BAUD_RATE_100000, /* 115200 bits/s */  \
										DATA_BITS_8,      /* 8 databits */     \
										STOP_BITS_2,      /* 1 stopbit */      \
										PARITY_EVEN,      /* No parity  */     \
										BIT_ORDER_LSB,    /* LSB first sent */ \
										NRZ_NORMAL,       /* Normal mode */    \
										RT_SERIAL_RB_BUFSZ, /* Buffer size */  \
										0  
									   };
struct rt_serial_device *sbus_serial;

static uint64_t last_rx_time;
static uint64_t last_frame_time;
static uint64_t last_txframe_time = 0;

#define SBUS2_FRAME_SIZE_RX_VOLTAGE	3
#define SBUS2_FRAME_SIZE_GPS_DIGIT	3

static enum SBUS2_DECODE_STATE {
	SBUS2_DECODE_STATE_DESYNC = 0xFFF,
	SBUS2_DECODE_STATE_SBUS_START = 0x2FF,
	SBUS2_DECODE_STATE_SBUS1_SYNC = 0x00,
	SBUS2_DECODE_STATE_SBUS2_SYNC = 0x1FF,
	SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE = 0x04,
	SBUS2_DECODE_STATE_SBUS2_GPS = 0x14,
	SBUS2_DECODE_STATE_SBUS2_DATA1 = 0x24,
	SBUS2_DECODE_STATE_SBUS2_DATA2 = 0x34
} sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;

static uint8_t sbus_frame[SBUS_FRAME_SIZE + (SBUS_FRAME_SIZE / 2)];

static unsigned partial_frame_count;
static unsigned sbus1_frame_delay = (1000U * 1000U) / SBUS1_DEFAULT_RATE_HZ;

static unsigned sbus_frame_drops;

unsigned sbus_dropped_frames()
{
	return sbus_frame_drops;
}

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static bool sbus_decode(uint64_t frame_time, uint8_t *frame, uint16_t *values, uint16_t *num_values,
	    bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t max_values)
{

	/* check frame boundary markers to avoid out-of-sync cases */
	if ((frame[0] != SBUS_START_SYMBOL)) {
		sbus_frame_drops++;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
		printf("DECODE FAIL: ");

		for (unsigned i = 0; i < SBUS_FRAME_SIZE; i++) {
			printf("%0x ", frame[i]);
		}

		printf("\n");
#endif
		sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
		return false;
	}

	switch (frame[24]) {
	case 0x00:
		/* this is S.BUS 1 */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS1_SYNC;
		break;

	case 0x04:
		/* receiver voltage */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE;
		break;

	case 0x14:
		/* GPS / baro */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_GPS;
		break;

	case 0x24:
		/* Unknown SBUS2 data */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
		break;

	case 0x34:
		/* Unknown SBUS2 data */
		sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
		break;

	default:
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
		printf("DECODE FAIL: END MARKER\n");
#endif
		sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
		return false;
	}

	/* we have received something we think is a frame */
	last_frame_time = frame_time;

	unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
			     SBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	for (unsigned channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}


		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		values[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	}

	/* decode switch channels if data fields are wide enough */
	if (max_values > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = (((frame[SBUS_FLAGS_BYTE] & (1 << 0)) > 0) ? 1 : 0) * 1000 + 998;
		/* channel 18 (index 17) */
		values[17] = (((frame[SBUS_FLAGS_BYTE] & (1 << 1)) > 0) ? 1 : 0) * 1000 + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	/* decode and handle failsafe and frame-lost flags */
	if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
		/* report that we failed to read anything valid off the receiver */
		*sbus_failsafe = true;
		*sbus_frame_drop = true;

	} else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
		/* set a special warning flag
		 *
		 * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
		 * condition as fail-safe greatly reduces the reliability and range of the radio link,
		 * e.g. by prematurely issuing return-to-launch!!! */

		*sbus_failsafe = false;
		*sbus_frame_drop = true;

	} else {
		*sbus_failsafe = false;
		*sbus_frame_drop = false;
	}

	return true;
}


static bool sbus_parse(uint64_t now, uint8_t *frame, unsigned len, uint16_t *values,
	   uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, unsigned *frame_drops, uint16_t max_channels)
{

	last_rx_time = now;

	/* this is set by the decoding state machine and will default to false
	 * once everything that was decodable has been decoded.
	 */
	bool decode_ret = false;

	/* keep decoding until we have consumed the buffer */
	for (unsigned d = 0; d < len; d++) {

		/* overflow check */
		if (partial_frame_count == sizeof(sbus_frame) / sizeof(sbus_frame[0])) {
			partial_frame_count = 0;
			sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("SBUS2: RESET (BUF LIM)\n");
#endif
		}

		if (partial_frame_count == SBUS_FRAME_SIZE) {
			partial_frame_count = 0;
			sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("SBUS2: RESET (PACKET LIM)\n");
#endif
		}

#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 1
		printf("sbus state: %s%s%s%s%s%s, count: %d, val: %02x\n",
		       (sbus_decode_state == SBUS2_DECODE_STATE_DESYNC) ? "SBUS2_DECODE_STATE_DESYNC" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS_START) ? "SBUS2_DECODE_STATE_SBUS_START" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS1_SYNC) ? "SBUS2_DECODE_STATE_SBUS1_SYNC" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS2_SYNC) ? "SBUS2_DECODE_STATE_SBUS2_SYNC" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE) ? "SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE" : "",
		       (sbus_decode_state == SBUS2_DECODE_STATE_SBUS2_GPS) ? "SBUS2_DECODE_STATE_SBUS2_GPS" : "",
		       partial_frame_count,
		       (unsigned)frame[d]);
#endif

		switch (sbus_decode_state) {
		case SBUS2_DECODE_STATE_DESYNC:

			/* we are de-synced and only interested in the frame marker */
			if (frame[d] == SBUS_START_SYMBOL) {
				sbus_decode_state = SBUS2_DECODE_STATE_SBUS_START;
				partial_frame_count = 0;
				sbus_frame[partial_frame_count++] = frame[d];
			}

			break;

		/* fall through */
		case SBUS2_DECODE_STATE_SBUS_START:
		case SBUS2_DECODE_STATE_SBUS1_SYNC:

		/* fall through */
		case SBUS2_DECODE_STATE_SBUS2_SYNC: {
				sbus_frame[partial_frame_count++] = frame[d];

				/* decode whatever we got and expect */
				if (partial_frame_count < SBUS_FRAME_SIZE) {
					break;
				}

				/*
				 * Great, it looks like we might have a frame.  Go ahead and
				 * decode it.
				 */
				decode_ret = sbus_decode(now, sbus_frame, values, num_values, sbus_failsafe, sbus_frame_drop, max_channels);

				/*
				 * Offset recovery: If decoding failed, check if there is a second
				 * start marker in the packet.
				 */
				unsigned start_index = 0;

				if (!decode_ret && sbus_decode_state == SBUS2_DECODE_STATE_DESYNC) {

					for (unsigned i = 1; i < partial_frame_count; i++) {
						if (sbus_frame[i] == SBUS_START_SYMBOL) {
							start_index = i;
							break;
						}
					}

					/* we found a second start marker */
					if (start_index != 0) {
						/* shift everything in the buffer and reset the state machine */
						for (unsigned i = 0; i < partial_frame_count - start_index; i++) {
							sbus_frame[i] = sbus_frame[i + start_index];
						}

						partial_frame_count -= start_index;
						sbus_decode_state = SBUS2_DECODE_STATE_SBUS_START;

#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
						printf("DECODE RECOVERY: %d\n", start_index);
#endif
					}
				}

				/* if there has been no successful attempt at saving a failed
				 * decoding run, reset the frame count for successful and
				 * unsuccessful decode runs.
				 */
				if (start_index == 0) {
					partial_frame_count = 0;
				}

			}
			break;

		case SBUS2_DECODE_STATE_SBUS2_RX_VOLTAGE: {
				sbus_frame[partial_frame_count++] = frame[d];

				if (partial_frame_count == 1 && sbus_frame[0] == SBUS_START_SYMBOL) {
					/* this slot is unused and in fact S.BUS2 sync */
					sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
				}

				if (partial_frame_count < SBUS2_FRAME_SIZE_RX_VOLTAGE) {
					break;
				}

				/* find out which payload we're dealing with in this slot */
				switch (sbus_frame[0]) {
				case 0x03: {
						// Observed values:
						// (frame[0] == 0x3 && frame[1] == 0x84 && frame[2] == 0x0)
						// (frame[0] == 0x3 && frame[1] == 0xc4 && frame[2] == 0x0)
						// (frame[0] == 0x3 && frame[1] == 0x80 && frame[2] == 0x2f)
						// (frame[0] == 0x3 && frame[1] == 0xc0 && frame[2] == 0x2f)
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 2
						uint16_t rx_voltage = (sbus_frame[1] << 8) | sbus_frame[2];
						printf("rx_voltage %d\n", (int)rx_voltage);
#endif
					}

					partial_frame_count = 0;
					break;

				default:
					/* this is not what we expect it to be, go back to sync */
					sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
					sbus_frame_drops++;
				}

			}
			break;

		case SBUS2_DECODE_STATE_SBUS2_GPS: {
				sbus_frame[partial_frame_count++] = frame[d];

				if (partial_frame_count == 1 && sbus_frame[0] == SBUS_START_SYMBOL) {
					/* this slot is unused and in fact S.BUS2 sync */
					sbus_decode_state = SBUS2_DECODE_STATE_SBUS2_SYNC;
				}

				if (partial_frame_count < 24) {
					break;
				}

				/* find out which payload we're dealing with in this slot */
				switch (sbus_frame[0]) {
				case 0x13: {
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
						uint16_t gps_something = (frame[1] << 8) | frame[2];
						printf("gps_something %d\n", (int)gps_something);
#endif
					}

					partial_frame_count = 0;
					break;

				default:
					/* this is not what we expect it to be, go back to sync */
					sbus_decode_state = SBUS2_DECODE_STATE_DESYNC;
					sbus_frame_drops++;
					/* throw unknown bytes away */
				}
			}
			break;

		default:
#if defined(SBUS_DEBUG_LEVEL) && SBUS_DEBUG_LEVEL > 0
			printf("UNKNOWN PROTO STATE");
#endif
			decode_ret = false;
		}


	}

	if (frame_drops) {
		*frame_drops = sbus_frame_drops;
	}

	/* return false as default */
	return decode_ret;
}


bool sbus_input(rt_device_t sbus_dev,uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop,
	   uint16_t max_channels)
{
	int		ret = 1;
	uint64_t	now;

	/*
	 * The S.BUS protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time_us();

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * a complete frame.
	 */
	uint8_t buf[SBUS_FRAME_SIZE * 2];
	bool sbus_decoded = false;

	ret = rt_device_read(sbus_dev,0,&buf[0],SBUS_FRAME_SIZE);

	/* if the read failed for any reason, just give up here */
	if (ret < 1) {
		return false;
	}

	/*
	 * Try to decode something with what we got
	 */
	if (sbus_parse(now, &buf[0], ret, values, num_values, sbus_failsafe,
		       sbus_frame_drop, &sbus_frame_drops, max_channels)) {

		sbus_decoded = true;
	}

	return sbus_decoded;
}



/*
  set output rate of SBUS in Hz
 */
void sbus1_set_output_rate_hz(uint16_t rate_hz)
{
	if (rate_hz > SBUS1_MAX_RATE_HZ) {
		rate_hz = SBUS1_MAX_RATE_HZ;
	}

	if (rate_hz < SBUS1_MIN_RATE_HZ) {
		rate_hz = SBUS1_MIN_RATE_HZ;
	}

	sbus1_frame_delay = (1000U * 1000U) / rate_hz;
}


int rt_sbus_init(void)
{	

	rt_bool_t result = RT_EOK;
	//sbus_serial->config = sbus_config;
//	
	sbus_serial = (struct rt_serial_device *)rt_device_find(SBUS_UART_NAME);
	sbus_serial->config = sbus_config;
	if (!sbus_serial)
    {
        rt_kprintf("find %s failed!\n", SBUS_UART_NAME);
        return RT_ERROR;
    }
	rt_device_open((rt_device_t)sbus_serial, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
//	sbus_serial->parent = *sbus;
//	/* apply configuration */
    if (sbus_serial->ops->configure)
        result = sbus_serial->ops->configure(sbus_serial, &sbus_serial->config);
//	
	partial_frame_count = 0;
	last_rx_time = hrt_absolute_time_us();
	last_frame_time = last_rx_time;
	sbus_frame_drops = 0;
	
    return result;
}
INIT_APP_EXPORT(rt_sbus_init);

static void sbus_publish(orb_advert_t *_sbus_adv,uint16_t *data)
{
	_input_rc.timestamp = hrt_absolute_time_us();
	for(int i = 0;i < SBUS_INPUT_CHANNELS; i++)
	{
		_input_rc.values[i] = data[i];
	}	
	orb_publish(ORB_ID(input_rc),_sbus_adv,&_input_rc);
}

static void sbus_start(void)
{
	orb_advert_t input_rc_pub;
	uint16_t r_raw_rc_count = SBUS_INPUT_CHANNELS;
	uint16_t r_raw_rc_values[r_raw_rc_count];
	input_rc_pub = orb_advertise(ORB_ID(input_rc),&_input_rc);
	while(1)
	{
		bool sbus_failsafe, sbus_frame_drop;
		bool sbus_updated = sbus_input((rt_device_t)sbus_serial,r_raw_rc_values, &r_raw_rc_count, &sbus_failsafe, &sbus_frame_drop,
				       RC_INPUT_CHANNELS);
		sbus_publish(input_rc_pub,r_raw_rc_values);
		//LOG_RAW("1:%d  2:%d  3:%d   4:%d  \n",r_raw_rc_values[0],r_raw_rc_values[1],r_raw_rc_values[2],r_raw_rc_values[3]);
		rt_thread_mdelay(20);
	}
	
}


rt_err_t sbus_main(int argc, char *argv[])
{
	if(argc>1)
	{
		const char *verb = argv[1];
		/*
		 * Start/load the driver.
		 */
		if (!strcmp(verb, "start")) {
			sbus_start();
		}

		if (!strcmp(verb, "stop")) {
			//sbus_stop(busid);
		}

		/*
		 * Test the driver/device.
		 */
		if (!strcmp(verb, "test")) {
			//sbus_test(busid);
		}

		/*
		 * Reset the driver.
		 */
		if (!strcmp(verb, "reset")) {
			//sbus_reset(busid);
		}

		/*
		 * Print driver information.
		 */
		if (!strcmp(verb, "info")) {
			//sbus_info(busid);
		}
	}
	

	//sbus_usage();
	return 0;
}

MSH_CMD_EXPORT(sbus_main,sbus)