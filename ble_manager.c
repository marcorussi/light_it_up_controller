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

#include "config.h"
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util_platform.h"
#ifdef UART_DEBUG
#include "uart.h"
#endif
#include "ble_manager.h"




/* ------------------- Local defines ------------------- */

#ifdef FACE_INDEX_TEST
/* Timer period in milli seconds */
#define TRIGGER_TIMER_PERIOD_MS				4000		/* 4 s */
/* Timer counter value */
#define TRIGGER_TIMER_COUNT					((uint32_t)(((uint64_t)TRIGGER_TIMER_PERIOD_MS * 1000000)/30517))
#endif

/* Low frequency clock source to be used by the SoftDevice */
#define NRF_CLOCK_LFCLKSRC      				{.source        = NRF_CLOCK_LF_SRC_RC,			\
                                 		 	 .rc_ctiv       = 15,                        \
                                 		 	 .rc_temp_ctiv  = 15,								\
                                 		 	 .xtal_accuracy = 0}

/* Number of central links used by the application. When changing this number remember to adjust the RAM settings */
#define CENTRAL_LINK_COUNT              	0     

/* Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings */
#define PERIPHERAL_LINK_COUNT            	1  

/* Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device */
#define IS_SRVC_CHANGED_CHARACT_PRESENT 	0                                 

/* The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms) */
#define APP_ADV_INTERVAL                 	MSEC_TO_UNITS(100, UNIT_0_625_MS) 

/* The advertising timeout (in units of seconds). 
   ATTENTION: do not exceed the maximum value in case of LIMITED DISCOVERABILITY MODE */
#define APP_ADV_TIMEOUT_IN_SECONDS      	60  

/* Value of the RTC1 PRESCALER register */
#define APP_TIMER_PRESCALER              	0    

/* Number of data byte in advertising packet */
#define NUM_OF_DATA_BYTES						8

/* Initial face index value */
#define INITIAL_STATE							1

/* Max adv length */
#define MAX_ADV_LENGTH							31

#define ADV_FLAGS_TYPE							BLE_GAP_AD_TYPE_FLAGS
#define BR_EDR_NOT_SUPPORTED					BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
#define MANUF_DATA_TYPE							BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA
#define MANUFACTURER_ID							TEMP_COMPANY_ID
#define MANUF_DATA_LENGTH						11




/* ---------- Local typedefs ---------- */

/* Device serial number integer type */
typedef uint32_t serial_num_int;

/* Device serial number string type */
typedef char serial_num_string[12];

/* Manufacturer data packet format to scan */
typedef enum
{
	FIRST_LENGTH_POS,					/* first length */
	ADV_TYPE_FLAGS_POS,				/* adv flags type */
	BR_EDR_NOT_SUPPORTED_POS,		/* BR/EDR not supported */
	SECOND_LENGTH_POS,				/* second length */
	MANUF_DATA_TYPE_POS,				/* manufacturer data type */
	MANUF_ID_BYTE_0_POS,				/* manufacturer ID lower byte */
	MANUF_ID_BYTE_1_POS,				/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH_POS,			/* data length */
	SERVICE_ID_BYTE_0_POS,			/* service ID lower byte */
	SERVICE_ID_BYTE_1_POS,			/* service ID higher byte */
	DATA_BYTE_0_POS,					/* data byte 0 */
	DATA_BYTE_1_POS,					/* data byte 1 */
	DATA_BYTE_2_POS,					/* data byte 2 */
	DATA_BYTE_3_POS,					/* data byte 3 */
	DATA_BYTE_4_POS,					/* data byte 4 */
	DATA_BYTE_5_POS,					/* data byte 5 */
	DATA_BYTE_6_POS,					/* data byte 6 */
	DATA_BYTE_7_POS,					/* data byte 7 */
	CALIB_RSSI_POS,					/* calibrated RSSI */
	ADV_DATA_PACKET_LENGTH			/* Manufacturer data adv packet length. This is not included. It is for fw purpose only */
} manuf_data_packet_e;




/* ------------------ Local macros ----------------------- */

#ifdef FACE_INDEX_TEST
/* Advertisement timer for test update */
APP_TIMER_DEF(adv_timer);
#endif

/* Define a pointer type to the device serial number stored in the UICR */
#define UICR_DEVICE_SERIAL_NUM			(*((serial_num_int *)(NRF_UICR_BASE + UICR_CUSTOMER_RESERVED_OFFSET)))




/* ---------- Local const variables ---------- */

/* Default manufaturer data packet. This string represent the  fixed part of the adv packet */
static const uint8_t adv_data_packet[ADV_DATA_PACKET_LENGTH] = 
{
	0x02,											/* first length */
	ADV_FLAGS_TYPE,							/* adv flags type */
	BR_EDR_NOT_SUPPORTED,					/* BR/EDR not supported */
	(uint8_t)(MANUF_DATA_LENGTH + 4),	/* second length */
	MANUF_DATA_TYPE,							/* manufacturer data type */
	(uint8_t)MANUFACTURER_ID,				/* manufacturer ID lower byte */
	(uint8_t)(MANUFACTURER_ID >> 8),		/* manufacturer ID higher byte */
	MANUF_DATA_LENGTH,						/* manufacturer data length */
	(uint8_t)MANUF_SERVICE_ID,				/* service ID lower byte */
	(uint8_t)(MANUF_SERVICE_ID >> 8),	/* service ID higher byte */
	INITIAL_STATE,								/* Data 0 */
	0x00,											/* Data 1 */
	0x00,											/* Data 2 */
	0x00,											/* Data 3 */
	0x00,											/* Data 4 */
	0x00,											/* Data 5 */
	0x00,											/* Data 6 */
	0x00,											/* Data 7 */
	TX_POWER_MEASURED_RSSI					/* RSSI TX power */
};




/* ------------------- Local variables ------------------- */ 

/* Parameters to be passed to the stack when starting advertising */
static ble_gap_adv_params_t m_adv_params;                                                                             

/* Array containing advertisement packet data */
uint8_t adv_data[ADV_DATA_PACKET_LENGTH];

/* Array containing scan response packet data */
uint8_t scan_resp_data[MAX_ADV_LENGTH];

/* Parameters to be passed to the stack when starting advertising */
static uint8_t device_name[] = DEVICE_NAME; 




/* ------------------- Local functions prototypes ------------------- */

static void on_ble_evt					(ble_evt_t *);
static void ble_evt_dispatch			(ble_evt_t *);
static void gap_params_init			(void);
static void advertising_init			(void);        
static void ble_stack_init				(void);
#ifdef FACE_INDEX_TEST
static void timer_handler				(void *);                
#endif




/* ------------------- Local functions ------------------- */

/* Function for handling the Application's BLE Stack events.
   Parameters:
   - p_ble_evt: Bluetooth stack event. */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
	const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;	

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_TIMEOUT:
		{
			/* if advertising timeout */
         if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISING)
         {
				/* ATTENTION: TODO */
				//uint32_t err_code = sd_ble_gap_adv_start(&m_adv_params);
				//APP_ERROR_CHECK(err_code);
			}
			else
			{
				/* do nothing */
			}
		}
		case BLE_GAP_EVT_CONNECTED:
		case BLE_GAP_EVT_DISCONNECTED:
		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
		case BLE_GATTC_EVT_TIMEOUT:
		case BLE_GATTS_EVT_TIMEOUT:
		{
      	/* Never here! */
      	break;
		}
		default:
		{
   		/* No implementation needed. */
   		break;
		}
	}
}


/* Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
   This function is called from the BLE Stack event interrupt handler after a BLE stack
   event has been received.
   Parameter:
   - p_ble_evt:  Bluetooth stack event. */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	on_ble_evt(p_ble_evt);
}


/* Function for the GAP initialization.
   This function sets up all the necessary GAP (Generic Access Profile) parameters of the
   device including the device name, appearance. Preferred connection parameters are not initialised. */
static void gap_params_init(void)
{
	uint32_t                err_code;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
		                                   (const uint8_t *)DEVICE_NAME,
		                                   strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	/* TODO: Use an appearance value matching the application's use case. */
	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
	APP_ERROR_CHECK(err_code);
}


/* Function for initializing the Advertising functionality.
   Encodes the required advertising data and passes it to the stack.
   Also builds a structure to be passed to the stack when starting advertising.
*/
static void advertising_init(void)
{
	uint32_t err_code;

	/* set adv data */
	memcpy((void *)&adv_data[0], (const void *)adv_data_packet, ADV_DATA_PACKET_LENGTH);

	/* set scan response data with shortened device name */
	scan_resp_data[0] = 1 + strlen((const char*)device_name);
	scan_resp_data[1] = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
	memcpy(&scan_resp_data[2], device_name, (scan_resp_data[0] - 1));

	/* set advertising data */
	err_code = sd_ble_gap_adv_data_set((uint8_t const *)adv_data, 
												  ADV_DATA_PACKET_LENGTH, 
												  (uint8_t const *)scan_resp_data, 
												  (scan_resp_data[0] + 1));
	APP_ERROR_CHECK(err_code);

	/* Initialize advertising parameters (used when starting advertising) */
	memset(&m_adv_params, 0, sizeof(m_adv_params));

	m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_SCAN_IND;	/* Not connectable */ //BLE_GAP_ADV_TYPE_ADV_IND;
	m_adv_params.p_peer_addr = NULL;	/* Undirected advertisement */
	m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
	m_adv_params.p_whitelist = NULL;
	m_adv_params.interval    = APP_ADV_INTERVAL;
	m_adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

	/* advertising is started in a separated function */
}


/* Function for initializing the BLE stack.
   Initializes the SoftDevice and the BLE event interrupt */
static void ble_stack_init(void)
{
	uint32_t err_code;

	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

	/* Initialize the SoftDevice handler module. */
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
		                                             PERIPHERAL_LINK_COUNT,
		                                             &ble_enable_params);
	APP_ERROR_CHECK(err_code);

	/* Check the ram settings against the used number of links */
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

	/* Enable BLE stack. */
	err_code = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(err_code);

	/* Register with the SoftDevice handler module for BLE events. */
	err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}


#ifdef FACE_INDEX_TEST
static uint8_t fake_face_index = 0;
const uint8_t fake_adv_values[2] = 
{
	0x00,
	0x01
};
/* Timer timeout function */
static void timer_handler(void * p_context)
{
	static uint8_t bytes_to_change[3];
#ifdef LED_DEBUG
	nrf_gpio_pin_toggle(7);
#endif
#ifdef UART_DEBUG
	uint8_t uart_string[20];
	sprintf((char *)uart_string, "_ADV TOGGLE");
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif
	/* increment fake face index */
	fake_face_index++;
	if(fake_face_index > 1)
	{
		fake_face_index = 0;
	}

	bytes_to_change[0] = 0; /* ATTENTION: not used at the moment */
	bytes_to_change[1] = fake_face_index;
	bytes_to_change[2] = fake_adv_values[fake_face_index];

	/* update BLE adv packet with new data values */
	ble_man_adv_update((uint8_t *)&bytes_to_change[0], 3);
}
#endif




/* ------------------- Exported functions ------------------- */

/* Function for updating the adv packet */
void ble_man_adv_update(uint8_t *data_values, uint8_t length)
{
	uint32_t err_code;

	/* check data pointer and length */
	if((length > 0) && (length <= NUM_OF_DATA_BYTES)
	&& (data_values != NULL))
	{
		/* stop advertising */
		err_code = sd_ble_gap_adv_stop();
		APP_ERROR_CHECK(err_code);

		/* set new adv data */
		memcpy(&adv_data[DATA_BYTE_0_POS], &data_values[0], length);
		
		/* set new adv data packet */
		err_code = sd_ble_gap_adv_data_set((uint8_t const *)adv_data, 
													  ADV_DATA_PACKET_LENGTH, 
												     (uint8_t const *)scan_resp_data, 
													  (scan_resp_data[0] + 1));
		APP_ERROR_CHECK(err_code);

		/* start advertising again */
		err_code = sd_ble_gap_adv_start(&m_adv_params);
		APP_ERROR_CHECK(err_code);
	}
	else
	{
		/* do nothing */
	}
}


/* Function for starting advertising */
void ble_man_adv_start(void)
{
	uint32_t err_code;

	/* init advertising */
	advertising_init();

#ifdef UART_DEBUG
	uint8_t uart_string[20];
	sprintf((char *)uart_string, "_ADV INITIALISED");
	uart_send_string((uint8_t *)uart_string, strlen((const char *)uart_string));
#endif

	/* start advertising */
	err_code = sd_ble_gap_adv_start(&m_adv_params);
	APP_ERROR_CHECK(err_code);
}


/* Function for application main entry */
void ble_man_init(void)
{
	/* stack BLE stack */
	ble_stack_init();
	gap_params_init();

#ifdef FACE_INDEX_TEST
	uint32_t err_code;

	/* init button timer */
	err_code = app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, timer_handler);
	APP_ERROR_CHECK(err_code);

	/* start timer */
	err_code = app_timer_start(adv_timer, TRIGGER_TIMER_COUNT, NULL);
	APP_ERROR_CHECK(err_code);
#endif
}


/*
	uint32_t err_code;

	err_code = sd_ble_gap_tx_power_set(TX_POWER_LEVEL);
	APP_ERROR_CHECK(err_code);
*/




/* End of file */




