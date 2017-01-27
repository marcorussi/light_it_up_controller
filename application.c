/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2016] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/




/* ------------------- Inclusions ------------------- */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util_platform.h"

#include "config.h"
#ifdef UART_DEBUG
#include "uart.h"
#endif
#ifdef ENABLE_ACCELEROMETER
#include "mpu6050.h"
#endif
#include "ble_manager.h"
#include "application.h"




/* ------------------- Local defines ------------------- */

#ifdef ENABLE_ACCELEROMETER
/* Number of acceleration axis */
#define MPU6050_NUM_OF_ACC_AXIS					3
/* Number of gyroscope axis */
#define MPU6050_NUM_OF_GYRO_AXIS					3

/* Burst read APP timer defines */
#define BURST_READ_TIMER_TICK_PERIOD_MS		CFG_MPU6050_BURST_READ_UPDATE_MS
#define BURST_READ_TIMER_TICK_COUNT				((uint32_t)(((uint64_t)BURST_READ_TIMER_TICK_PERIOD_MS * 1000000)/30517))

/* BLE advertisement update APP timer defines */
#define BLE_UPDATE_TIMER_TICK_PERIOD_MS		CFG_BLE_ADV_UPDATE_MS
#define BLE_UPDATE_TIMER_TICK_COUNT				((uint32_t)(((uint64_t)BLE_UPDATE_TIMER_TICK_PERIOD_MS * 1000000)/30517))

/* 1G threshold value according to accelerometer LSB sensitivity */
#define ACC_SENS_RANGE_2G							16384
#define ACC_SENS_RANGE_4G							8192
#define ACC_SENS_RANGE_8G							4096
#define ACC_SENS_RANGE_16G							2048

/* 90°/s threshold value according to gyroscope LSB sensitivity */
#define GYRO_SENS_RANGE_250DPS					11796
#define GYRO_SENS_RANGE_500DPS					5898
#define GYRO_SENS_RANGE_1000DPS					2949
#define GYRO_SENS_RANGE_2000DPS					1474

/* Accelerometer LSB tolerance (should depend on the selected threshold value) */
#define ACC_LSB_THRESHOLD_TOL						2000

/* Gyroscope LSB tolerance (should depend on the selected threshold value) */
#define GYRO_LSB_THRESHOLD_TOL					1500

/* 1G/-1G thresholds according to range set at 4G in MPU6500 init */
#define ACC_LSB_THRESHOLD_1G						(ACC_SENS_RANGE_4G - ACC_LSB_THRESHOLD_TOL)		
#define ACC_LSB_THRESHOLD_N1G						(-ACC_SENS_RANGE_4G + ACC_LSB_THRESHOLD_TOL)

/* 90°/-90° thresholds according to range set at 250°/s in MPU6500 init */
#define GYRO_LSB_THRESHOLD_90DPS					(GYRO_SENS_RANGE_250DPS - GYRO_LSB_THRESHOLD_TOL)		
#define GYRO_LSB_THRESHOLD_N90DPS				(-GYRO_SENS_RANGE_250DPS + GYRO_LSB_THRESHOLD_TOL)

/* XYZ motion flags axis offset - NOT USED */
#define MOTION_FLAGS_X_OFFSET						0
#define MOTION_FLAGS_Y_OFFSET						1
#define MOTION_FLAGS_Z_OFFSET						2

/* XYZ motion flags positions */
#define MOTION_FLAGS_1G_POS						1
#define MOTION_FLAGS_N1G_POS						2
#define MOTION_FLAGS_90DPS_POS					4
#define MOTION_FLAGS_N90DPS_POS					8

/* Codified XYZ motion info from flags */
#define MOTION_FLAGS_X_FACE_UP					0x0001	/* X 1G */
#define MOTION_FLAGS_X_FACE_DOWN					0x0002 	/* X N1G */
#define MOTION_FLAGS_X_ROTATE_RIGHT_1			0x0005	/* X 90DPS and 1G */
#define MOTION_FLAGS_X_ROTATE_RIGHT_2			0x0006	/* X 90DPS and N1G */
#define MOTION_FLAGS_X_ROTATE_LEFT_1			0x0009	/* X N90DPS and 1G */
#define MOTION_FLAGS_X_ROTATE_LEFT_2			0x000A	/* X N90DPS and N1G */

#define MOTION_FLAGS_Y_FACE_UP					0x0010	/* Y 1G */
#define MOTION_FLAGS_Y_FACE_DOWN					0x0020 	/* Y N1G */
#define MOTION_FLAGS_Y_ROTATE_RIGHT_1			0x0050	/* Y 90DPS and 1G */
#define MOTION_FLAGS_Y_ROTATE_RIGHT_2			0x0060	/* Y 90DPS and N1G */
#define MOTION_FLAGS_Y_ROTATE_LEFT_1			0x0090	/* Y N90DPS and 1G */
#define MOTION_FLAGS_Y_ROTATE_LEFT_2			0x00A0	/* Y N90DPS and N1G */

#define MOTION_FLAGS_Z_FACE_UP					0x0100	/* Z 1G */
#define MOTION_FLAGS_Z_FACE_DOWN					0x0200 	/* Z N1G */
#define MOTION_FLAGS_Z_ROTATE_RIGHT_1			0x0500	/* Z 90DPS and 1G */
#define MOTION_FLAGS_Z_ROTATE_RIGHT_2			0x0600	/* Z 90DPS and N1G */
#define MOTION_FLAGS_Z_ROTATE_LEFT_1			0x0900	/* Z N90DPS and 1G */
#define MOTION_FLAGS_Z_ROTATE_LEFT_2			0x0A00	/* Z 90DPS and N1G */

#endif




/* ------------------- Local typedefs ------------------- */

typedef enum
{
	MOTION_STATE_X_FACE_UP,
 	MOTION_STATE_X_FACE_DOWN,
 	MOTION_STATE_X_ROTATE_RIGHT,
 	MOTION_STATE_X_ROTATE_LEFT,
	MOTION_STATE_Y_FACE_UP,
 	MOTION_STATE_Y_FACE_DOWN,
 	MOTION_STATE_Y_ROTATE_RIGHT,
 	MOTION_STATE_Y_ROTATE_LEFT,
	MOTION_STATE_Z_FACE_UP,
 	MOTION_STATE_Z_FACE_DOWN,
 	MOTION_STATE_Z_ROTATE_RIGHT,
 	MOTION_STATE_Z_ROTATE_LEFT,
	NUM_OF_MOTION_STATES,		/* Number of motion detected states */
	MOTION_STATE_INVALID = 0xFF		
} motion_state_ke;




/* ------------------- Local macros ------------------- */

/* "motion_codified" register configuration */
/* meaning:  n/a n/a n/a n/a - Zn90dps Z90dps Zn1g Z1g - Yn90dps Y90dps Yn1g Y1g - Xn90dps X90dps Xn1g X1g
   bit pos:   15  14  13  12 -   11      10     9    8      7       6     5    4      3       2     1    0 */

/* Macros to set, clear and check motion flags */
#define SET_MOTION_FLAG(a, f)						(motion_codified |= (f<<(4*a)))
#define CLEAR_MOTION_FLAG(a, f)					(motion_codified &= (~(f<<(4*a))))

#ifdef ENABLE_ACCELEROMETER
/* Define timer for MPU6050 burst read trigger */
APP_TIMER_DEF(read_trigger);

/* Define timer for updating BLE advertisement packet */
APP_TIMER_DEF(ble_update_trigger);
#endif




/* ------------------- Local const variables ------------------- */
#ifdef ENABLE_ACCELEROMETER
/* Default characteristic values for each motion state */
static const uint8_t adv_values[NUM_OF_MOTION_STATES] = 
{
	0x10,					
	0x11,
	0x12,	
	0x13,
	0x14,
	0x15,
	0x16,
	0x17,
	0x18,
	0x19,
	0x1A,
	0x1B						
};

/* Default strings for each motion state */
static const char *motion_states_strings[NUM_OF_MOTION_STATES] = 
{
	"X UP",					
	"X DOWN",	
	"X ROT R",	
	"X ROT L",	
	"Y UP",					
	"Y DOWN",	
	"Y ROT R",	
	"Y ROT L",	
	"Z UP",					
	"Z DOWN",	
	"Z ROT R",	
	"Z ROT L"					
};
#endif




/* ------------------- Local variables ------------------- */
#ifdef ENABLE_ACCELEROMETER
/* Register to store motion flags */
static uint16_t motion_codified = 0;

/* store last valid motion state index. Init to an invalid greater than 6 value */
static uint8_t last_motion_state_index = MOTION_STATE_INVALID;
#endif

/* store boolean flag for indicating BLE update timer running */
static bool update_timer_running = false;




/* ------------------- Local functions prototypes ------------------- */
#ifdef ENABLE_ACCELEROMETER
static void mpu6050_burst_read_callback	(int16_t *, uint8_t);
static void read_timeout_handler				(void *);
static void update_timeout_handler			(void *);
static void timer_config						(void);
#endif




/* ------------------- Local functions ------------------- */

#ifdef ENABLE_ACCELEROMETER
/* Burst read callback function */
void mpu6050_burst_read_callback( int16_t *p_data, uint8_t data_length)
{
	uint8_t i;
	uint8_t motion_state_index = 0;

	/* detect +/-1G for x,y,z acc */
	for(i=0; i<MPU6050_NUM_OF_ACC_AXIS; i++)
	{
		/* over 1G threshold */
		if(p_data[i] > ACC_LSB_THRESHOLD_1G)
		{
			SET_MOTION_FLAG(i, MOTION_FLAGS_1G_POS);
		}
		/* around 0G */
		else if((p_data[i] < ACC_LSB_THRESHOLD_1G)
		     && (p_data[i] > ACC_LSB_THRESHOLD_N1G))
		{
			CLEAR_MOTION_FLAG(i, MOTION_FLAGS_1G_POS);
			CLEAR_MOTION_FLAG(i, MOTION_FLAGS_N1G_POS);
		}
		/* over -1G threshold */
		else if(p_data[i] < ACC_LSB_THRESHOLD_N1G)
		{
			SET_MOTION_FLAG(i, MOTION_FLAGS_N1G_POS);
		}
		else
		{
			/* keep current value. NEVER HERE! */
		}
	}

	/* store temperature. TODO */
	//p_data[3];

	/* detect +/-90DPS for x,y,z gyro */
	for(i=4; i<(4+MPU6050_NUM_OF_GYRO_AXIS); i++)
	{
		/* over 90DPS threshold */
		if(p_data[i] > GYRO_LSB_THRESHOLD_90DPS)
		{
			SET_MOTION_FLAG((i-4), MOTION_FLAGS_90DPS_POS);
		}
		/* around 90DPS */
		else if((p_data[i] < GYRO_LSB_THRESHOLD_90DPS)
		     && (p_data[i] > GYRO_LSB_THRESHOLD_N90DPS))
		{
			CLEAR_MOTION_FLAG((i-4), MOTION_FLAGS_90DPS_POS);
			CLEAR_MOTION_FLAG((i-4), MOTION_FLAGS_N90DPS_POS);
		}
		/* over N90DPS threshold */
		else if(p_data[i] < GYRO_LSB_THRESHOLD_N90DPS)

		{
			SET_MOTION_FLAG((i-4), MOTION_FLAGS_N90DPS_POS);
		}
		else
		{
			/* keep current value. NEVER HERE! */
		}
	}

//#ifdef UART_DEBUG
#if 0
	uint8_t uart_string[20];
	sprintf((char *)uart_string, "ACC_X: %d", p_data[0]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "ACC_Y: %d", p_data[1]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "ACC_Z: %d", p_data[2]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "GYRO_X: %d", p_data[4]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "GYRO_Y: %d", p_data[5]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "GYRO_Z: %d", p_data[6]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif

	/* get motion state index to broadcast */
	switch(motion_codified)
	{
		case MOTION_FLAGS_X_FACE_UP:
			motion_state_index = MOTION_STATE_X_FACE_UP;
			break;
		case MOTION_FLAGS_X_FACE_DOWN:
			motion_state_index = MOTION_STATE_X_FACE_DOWN;
			break;
		case MOTION_FLAGS_X_ROTATE_RIGHT_1:
		case MOTION_FLAGS_X_ROTATE_RIGHT_2:
			motion_state_index = MOTION_STATE_X_ROTATE_RIGHT;
			break;
		case MOTION_FLAGS_X_ROTATE_LEFT_1:
		case MOTION_FLAGS_X_ROTATE_LEFT_2:
			motion_state_index = MOTION_STATE_X_ROTATE_LEFT;
			break;
		case MOTION_FLAGS_Y_FACE_UP:
			motion_state_index = MOTION_STATE_Y_FACE_UP;
			break;
		case MOTION_FLAGS_Y_FACE_DOWN:
			motion_state_index = MOTION_STATE_Y_FACE_DOWN;
			break;
		case MOTION_FLAGS_Y_ROTATE_RIGHT_1:
		case MOTION_FLAGS_Y_ROTATE_RIGHT_2:
			motion_state_index = MOTION_STATE_Y_ROTATE_RIGHT;
			break;
		case MOTION_FLAGS_Y_ROTATE_LEFT_1:
		case MOTION_FLAGS_Y_ROTATE_LEFT_2:
			motion_state_index = MOTION_STATE_Y_ROTATE_LEFT;
			break;
		case MOTION_FLAGS_Z_FACE_UP:
			motion_state_index = MOTION_STATE_Z_FACE_UP;
			break;
		case MOTION_FLAGS_Z_FACE_DOWN:
			motion_state_index = MOTION_STATE_Z_FACE_DOWN;
			break;
		case MOTION_FLAGS_Z_ROTATE_RIGHT_1:
		case MOTION_FLAGS_Z_ROTATE_RIGHT_2:
			motion_state_index = MOTION_STATE_Z_ROTATE_RIGHT;
			break;
		case MOTION_FLAGS_Z_ROTATE_LEFT_1:
		case MOTION_FLAGS_Z_ROTATE_LEFT_2:
			motion_state_index = MOTION_STATE_Z_ROTATE_LEFT;
			break;
		default:
			/* invalid motion state index */
			motion_state_index = MOTION_STATE_INVALID;
			break;
	}	

	/* if motion state has changed */
	if((motion_state_index != last_motion_state_index)
	&& (motion_state_index < NUM_OF_MOTION_STATES))
	{
		/* store motion state index */
		last_motion_state_index = motion_state_index;
#ifdef UART_DEBUG
		uint8_t uart_string[40];
		sprintf((char *)uart_string, "_MOTION STATE: %x - %s", motion_codified, 
																				 (const char *)motion_states_strings[motion_state_index]);
		uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif

		/* ATTENTION: using this advertising mechanism, only motions of faces up and down will be advertised! */
		/* start BLE adv update trigger timer */
		uint32_t err_code = app_timer_start(ble_update_trigger, BLE_UPDATE_TIMER_TICK_COUNT, NULL);
		APP_ERROR_CHECK(err_code);
		/* update timer is running */
		update_timer_running = true;
	}
	else
	{
		/* do nothing */
	}
}


/* Timer timeout handler for triggering a new conversion */
static void read_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	/* start burst read of MPU6050 */
	mpu6050_start_burst_read();
}


/* Timer timeout handler for triggering a new conversion */
static void update_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
#ifdef LED_DEBUG
	nrf_gpio_pin_toggle(7);
#endif
	/* if motion state index is valid */
	if(last_motion_state_index < NUM_OF_MOTION_STATES)
	{
		/* update BLE adv packet with new data values */
		ble_man_adv_update((uint8_t *)&adv_values[last_motion_state_index], 1);
	}
	else
	{
		/* do nothing */
	}

	/* update timer is running */
	update_timer_running = false;
}


/* Function to init the read timer */
static void timer_config(void)
{
	uint32_t err_code;

	/* init burst read trigger timer */
	err_code = app_timer_create(&read_trigger, APP_TIMER_MODE_REPEATED, read_timeout_handler);
	APP_ERROR_CHECK(err_code);

	/* init BLE adv update trigger timer */
	err_code = app_timer_create(&ble_update_trigger, APP_TIMER_MODE_SINGLE_SHOT, update_timeout_handler);
	APP_ERROR_CHECK(err_code);

	/* start burst read trigger timer */
	err_code = app_timer_start(read_trigger, BURST_READ_TIMER_TICK_COUNT, NULL);
	APP_ERROR_CHECK(err_code);

	/* update trigger timer is started on a new motion detection */
}
#endif




/* --------------- Exported functions ----------------- */

/* Function to init the application */
void app_init( void )
{
#ifdef ENABLE_ACCELEROMETER
	mpu6050_init_st init_data;
#endif

	/* init BLE manager */
	ble_man_init();

#ifdef ENABLE_ACCELEROMETER
	/* set burst read callback function */
	init_data.burst_read_handler = (burst_read_handler_st)mpu6050_burst_read_callback;

	/* if mpu6050 initialised successfully */
	if(true == mpu6050_init(&init_data))
	{
		/* start burst read timer */
		timer_config();
	}
	else
	{
		/* mpu6050 init failed. Do nothing */
	}
#endif

#ifdef FACE_INDEX_TEST
#ifdef UART_DEBUG
		uint8_t uart_string[] = "FAKE TEST";
		uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
#endif
	/* start advertising */
	ble_man_adv_start();
}


/* Main loop function of the application */
void app_run( void )
{
	/* do nothing */
}




/* End of file */




