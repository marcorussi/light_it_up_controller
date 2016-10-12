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
/* Number of motion axis */
#define MPU6050_NUM_OF_MOTION_AXIS			3

/* Burst read APP timer defines */
#define BURST_READ_TIMER_TICK_PERIOD_MS		CFG_MPU6050_BURST_READ_UPDATE_MS
#define BURST_READ_TIMER_TICK_COUNT			((uint32_t)(((uint64_t)BURST_READ_TIMER_TICK_PERIOD_MS * 1000000)/30517))

/* BLE advertisement update APP timer defines */
#define BLE_UPDATE_TIMER_TICK_PERIOD_MS		CFG_BLE_ADV_UPDATE_MS
#define BLE_UPDATE_TIMER_TICK_COUNT			((uint32_t)(((uint64_t)BLE_UPDATE_TIMER_TICK_PERIOD_MS * 1000000)/30517))

/* LSB sensitivity */
#define LSB_SENS_RANGE_2G					16384
#define LSB_SENS_RANGE_4G					8192
#define LSB_SENS_RANGE_8G					4096
#define LSB_SENS_RANGE_16G					2048

/* 1G threshold */
#define ACC_LSB_THRESHOLD_1G				(LSB_SENS_RANGE_4G)
/* -1G threshold */
#define ACC_LSB_THRESHOLD_N1G				(-LSB_SENS_RANGE_4G)

/* LSB tolerance */
#define ACC_LSB_THRESHOLD_TOL				2000

/* G thresholds */
#ifndef LIMIT_XYZ_VALUES_TO_2G
#define ACC_LSB_THRESHOLD_4G_HIGH			((ACC_LSB_THRESHOLD_1G*4) + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_4G_LOW			((ACC_LSB_THRESHOLD_1G*4) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_3G_HIGH			((ACC_LSB_THRESHOLD_1G*3) + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_3G_LOW			((ACC_LSB_THRESHOLD_1G*3) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_2G_HIGH			((ACC_LSB_THRESHOLD_1G*2) + ACC_LSB_THRESHOLD_TOL)
#endif
#define ACC_LSB_THRESHOLD_2G_LOW			((ACC_LSB_THRESHOLD_1G*2) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_1G_HIGH			(ACC_LSB_THRESHOLD_1G + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_1G_LOW			(ACC_LSB_THRESHOLD_1G - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N1G_HIGH			(ACC_LSB_THRESHOLD_N1G + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N1G_LOW			(ACC_LSB_THRESHOLD_N1G - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N2G_HIGH			((ACC_LSB_THRESHOLD_N1G*2) + ACC_LSB_THRESHOLD_TOL)
#ifndef LIMIT_XYZ_VALUES_TO_2G
#define ACC_LSB_THRESHOLD_N2G_LOW			((ACC_LSB_THRESHOLD_N1G*2) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N3G_HIGH			((ACC_LSB_THRESHOLD_N1G*3) + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N3G_LOW			((ACC_LSB_THRESHOLD_N1G*3) - ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N4G_HIGH			((ACC_LSB_THRESHOLD_N1G*4) + ACC_LSB_THRESHOLD_TOL)
#define ACC_LSB_THRESHOLD_N4G_LOW			((ACC_LSB_THRESHOLD_N1G*4) - ACC_LSB_THRESHOLD_TOL)
#endif

/* Codified ZYX values for face index */
/* ATTENTION: only on axis is considered for each face position */
/* Codified axis positions */
#define X_PG_POS							0x01
#define Y_PG_POS							0x02
#define Z_PG_POS							0x04
#define X_NG_POS							0x08
#define Y_NG_POS							0x10
#define Z_NG_POS							0x20
/* Codified xyz for each face */
#define XYZ_G_FACE_INDEX_1					0b00000001
#define XYZ_G_FACE_INDEX_2					0b00000010
#define XYZ_G_FACE_INDEX_3					0b00000100
#define XYZ_G_FACE_INDEX_4					0b00001000
#define XYZ_G_FACE_INDEX_5					0b00010000
#define XYZ_G_FACE_INDEX_6					0b00100000
/* Faces index */
#define FACE_INDEX_1						0
#define FACE_INDEX_2						1
#define FACE_INDEX_3						2
#define FACE_INDEX_4						3
#define FACE_INDEX_5						4
#define FACE_INDEX_6						5
#endif




/* ------------------- Local typedefs ------------------- */

#ifdef ENABLE_ACCELEROMETER
/* Structure containing x,y,z and temperature values */
typedef struct
{
	int16_t acc_xyz[MPU6050_NUM_OF_MOTION_AXIS];
	int16_t temp;
} motion_temp_st;

/* Structure to store motion and temperature values */
static motion_temp_st motion_temp_values;
#endif




/* ------------------- Local macros ------------------- */

#ifdef ENABLE_ACCELEROMETER
/* Define timer for MPU6050 burst read trigger */
APP_TIMER_DEF(read_trigger);

/* Define timer for updating BLE advertisement packet */
APP_TIMER_DEF(ble_update_trigger);
#endif




/* ------------------- Local const variables ------------------- */

/* Default characteristic values */
const uint8_t adv_values[6] = 
{
	0x20,					
	0x21,		
	0x22,		
	0x23,		
	0x24,	
	0x25							
};




/* ------------------- Local functions prototypes ------------------- */

#ifdef ENABLE_ACCELEROMETER
static void mpu6050_burst_read_callback	(int16_t *, uint8_t);
static void read_timeout_handler		(void *);
static void update_timeout_handler		(void *);
static void timer_config				(void);
#endif




/* ------------------- Local functions ------------------- */

#ifdef ENABLE_ACCELEROMETER
/* Burst read callback function */
void mpu6050_burst_read_callback( int16_t *p_data, uint8_t data_length)
{
	uint8_t i;

	/* convert x,y,z in G */
	for(i=0; i<MPU6050_NUM_OF_MOTION_AXIS; i++)
	{
		/* calculate value in G */
#ifndef LIMIT_XYZ_VALUES_TO_2G
		/* if higher than ACC_LSB_THRESHOLD_4G_HIGH */
		if(p_data[i] > ACC_LSB_THRESHOLD_4G_HIGH)
		{
			/* saturate at 4G */
			motion_temp_values.acc_xyz[i] = 4;
		}
		/* around 4G */
		if((p_data[i] < ACC_LSB_THRESHOLD_4G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_4G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 4;
		}
		/* around 3G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_3G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_3G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 3;
		}
		/* around 2G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_2G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_2G_LOW))
#else
		/* any value bigger than 2G is limited to 2G */
		if(p_data[i] > ACC_LSB_THRESHOLD_2G_LOW)
#endif
		{
			motion_temp_values.acc_xyz[i] = 2;
		}
		/* around 1G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_1G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_1G_LOW))
		{
			motion_temp_values.acc_xyz[i] = 1;
		}
		/* around 0G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_1G_LOW)
		&& (p_data[i] > ACC_LSB_THRESHOLD_N1G_HIGH))
		{
			motion_temp_values.acc_xyz[i] = 0;
		}
		/* around -1G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_N1G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_N1G_LOW))
		{
			motion_temp_values.acc_xyz[i] = -1;
		}
#ifdef LIMIT_XYZ_VALUES_TO_2G
		else if(p_data[i] < ACC_LSB_THRESHOLD_N2G_HIGH)
		{
			motion_temp_values.acc_xyz[i] = -2;
		}
#else
		/* around -2G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_N2G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_N2G_LOW))
		{
			motion_temp_values.acc_xyz[i] = -2;
		}
		/* around -3G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_N3G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_N3G_LOW))
		{
			motion_temp_values.acc_xyz[i] = -3;
		}
		/* around -4G */
		else 
		if((p_data[i] < ACC_LSB_THRESHOLD_N4G_HIGH)
		&& (p_data[i] > ACC_LSB_THRESHOLD_N4G_LOW))
		{
			motion_temp_values.acc_xyz[i] = -4;
		}
		/* else if higher than ACC_LSB_THRESHOLD_N4G_LOW */
		else if(p_data[i] < ACC_LSB_THRESHOLD_N4G_LOW)
		{
			/* saturate at -4G */
			motion_temp_values.acc_xyz[i] = -4;
		}
#endif
		else
		{
			/* keep current value */
		}
	}
#ifdef LED_DEBUG
	//nrf_gpio_pin_toggle(21);
#endif
	/* store temperature */
	motion_temp_values.temp = p_data[3];

#ifdef UART_DEBUG
	uint8_t uart_string[20];
	sprintf((char *)uart_string, "_X: %d - %d", p_data[0], motion_temp_values.acc_xyz[0]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "_Y: %d - %d", p_data[1], motion_temp_values.acc_xyz[1]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
	sprintf((char *)uart_string, "_Z: %d - %d", p_data[2], motion_temp_values.acc_xyz[2]);
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
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
	uint8_t motion_codified_g = 0;
	uint8_t face_index = 0xFF;	/* Init to an invalid greater than 6 value */

	UNUSED_PARAMETER(p_context);
#ifdef LED_DEBUG
	//nrf_gpio_pin_toggle(21);
#endif

	/* calculate face index according to last acquired motion values */
	/* ATTENTION: only one axis should be considered for each face: see defines above */
	/* X */
	if(motion_temp_values.acc_xyz[0] == 1)
	{
		motion_codified_g |= X_PG_POS;
	}
	else if(motion_temp_values.acc_xyz[0] == -1)
	{
		motion_codified_g |= X_NG_POS;
	}
	else
	{
		/* do nothing */
	}
	/* Y */
	if(motion_temp_values.acc_xyz[1] == 1)
	{
		motion_codified_g |= Y_PG_POS;
	}
	else if(motion_temp_values.acc_xyz[1] == -1)
	{
		motion_codified_g |= Y_NG_POS;
	}
	else
	{
		/* do nothing */
	}
	/* Z */
	if(motion_temp_values.acc_xyz[2] == 1)
	{
		motion_codified_g |= Z_PG_POS;
	}
	else if(motion_temp_values.acc_xyz[2] == -1)
	{
		motion_codified_g |= Z_NG_POS;
	}
	else
	{
		/* do nothing */
	}

	/* get final face index to broadcast */
	switch(motion_codified_g)
	{
		case XYZ_G_FACE_INDEX_1:
			face_index = FACE_INDEX_1;
			break;
		case XYZ_G_FACE_INDEX_2:
			face_index = FACE_INDEX_2;
			break;
		case XYZ_G_FACE_INDEX_3:
			face_index = FACE_INDEX_3;
			break;
		case XYZ_G_FACE_INDEX_4:
			face_index = FACE_INDEX_4;
			break;
		case XYZ_G_FACE_INDEX_5:
			face_index = FACE_INDEX_5;
			break;
		case XYZ_G_FACE_INDEX_6:
			face_index = FACE_INDEX_6;
			break;
		default:
			/* invalid face index */
			break;
	}	
	
	/* if valid calculated face index */
	if(face_index <= FACE_INDEX_6)
	{
#ifdef UART_DEBUG
		uint8_t uart_string[20];
		sprintf((char *)uart_string, "_FACE: %x - %d", motion_codified_g, face_index);
		uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
		/* update BLE adv packet with new data values */
		ble_man_adv_update((uint8_t *)&adv_values[face_index], 1);
	}
	else
	{
		/* invalid face index: do nothing */
	}
}


/* Function to init the read timer */
static void timer_config(void)
{
    uint32_t err_code;

	/* init burst read trigger timer */
	err_code = app_timer_create(&read_trigger, APP_TIMER_MODE_REPEATED, read_timeout_handler);
    APP_ERROR_CHECK(err_code);

	/* init BLE adv update trigger timer */
	err_code = app_timer_create(&ble_update_trigger, APP_TIMER_MODE_REPEATED, update_timeout_handler);
    APP_ERROR_CHECK(err_code);

	/* start burst read trigger timer */
	err_code = app_timer_start(read_trigger, BURST_READ_TIMER_TICK_COUNT, NULL);
    APP_ERROR_CHECK(err_code);

	/* start BLE adv update trigger timer */
	err_code = app_timer_start(ble_update_trigger, BLE_UPDATE_TIMER_TICK_COUNT, NULL);
    APP_ERROR_CHECK(err_code);
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
	/* start advertising */
	ble_man_adv_start();
}


/* Main loop function of the application */
void app_run( void )
{
	/* do nothing */
#ifdef ENABLE_ACCELEROMETER
#ifdef LED_DEBUG
	if(motion_temp_values.acc_xyz[0] > 0)
	{
		nrf_gpio_pin_write(22, 0);
	}
	else if(motion_temp_values.acc_xyz[0] < 0)
	{
		nrf_gpio_pin_write(22, 1);
	}

	if(motion_temp_values.acc_xyz[1] > 0)
	{
		nrf_gpio_pin_write(23, 0);
	}
	else if(motion_temp_values.acc_xyz[1] < 0)
	{
		nrf_gpio_pin_write(23, 1);
	}

	if(motion_temp_values.acc_xyz[2] > 0)
	{
		nrf_gpio_pin_write(24, 0);
	}
	else if(motion_temp_values.acc_xyz[2] < 0)
	{
		nrf_gpio_pin_write(24, 1);
	}
#endif
#endif
}




/* End of file */




