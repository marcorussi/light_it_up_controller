/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2015] [Marco Russi]
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


/* ------------- Inclusions --------------- */

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>




/* ------------- Exported definitions --------------- */

/* The UUID of the CUBE_CFG service */
#define BLE_UUID_CUBE_CFG_SERVICE 					0x0001		

/* UUID type for the CUBE_CFG Service (vendor specific) */ 
#define CUBE_CFG_SERVICE_UUID_TYPE					BLE_UUID_TYPE_VENDOR_BEGIN      

/* BYTE 1 Characteristic value length in bytes */
#define BLE_CUBE_CFG_BYTE1_CHAR_LENGTH				1

/* BYTE 2 Characteristic value length in bytes */
#define BLE_CUBE_CFG_BYTE2_CHAR_LENGTH				1

/* SHORT 1 Characteristic value length in bytes */
#define BLE_CUBE_CFG_SHORT1_CHAR_LENGTH			2

/* SHORT 2 Characteristic value length in bytes */
#define BLE_CUBE_CFG_SHORT2_CHAR_LENGTH			2

/* SPECIAL OP Characteristic value length in bytes */
#define BLE_CUBE_CFG_SPECIAL_OP_CHAR_LENGTH		1

/* Total characteristics length in bytes */
#define BLE_CUBE_CFG_SERVICE_CHARS_LENGTH			(BLE_CUBE_CFG_BYTE1_CHAR_LENGTH + 		\
																 BLE_CUBE_CFG_BYTE2_CHAR_LENGTH + 		\
																 BLE_CUBE_CFG_SHORT1_CHAR_LENGTH +		\
																 BLE_CUBE_CFG_SHORT2_CHAR_LENGTH +		\
																 BLE_CUBE_CFG_SPECIAL_OP_CHAR_LENGTH	\

/* Characteristics positions for memory storage */
/* BYTE1 Characteristic value position in bytes */
#define BLE_CUBE_CFG_BYTE1_CHAR_POS					0

/* BYTE2 Characteristic value position in bytes */
#define BLE_CUBE_CFG_BYTE2_CHAR_POS					(BLE_CUBE_CFG_BYTE1_CHAR_POS + BLE_CUBE_CFG_BYTE1_CHAR_LENGTH)

/* SHORT 1 Characteristic value position in bytes */
#define BLE_CUBE_CFG_SHORT1_CHAR_POS				(BLE_CUBE_CFG_BYTE2_CHAR_POS + BLE_CUBE_CFG_BYTE2_CHAR_LENGTH)

/* SHORT 2 Characteristic value position in bytes */
#define BLE_CUBE_CFG_SHORT2_CHAR_POS				(BLE_CUBE_CFG_SHORT1_CHAR_POS + BLE_CUBE_CFG_SHORT1_CHAR_LENGTH)

/* Special Op characteristic doesn't need a position because is not saved in memory */
/* Needed memory for stored characteristics of CFG service */
#define BLE_CUBE_CFG_STORED_CHARS_LENGTH			(BLE_CUBE_CFG_SHORT2_CHAR_POS + BLE_CUBE_CFG_SHORT2_CHAR_LENGTH)
/* ATTENTION: this value must be equal of or smaller than MEM_BUFFER_DATA_LENGTH */



/* ------------- Exported structures --------------- */

/* Forward declaration of the ble_cube_cfg_st type */
typedef struct ble_cube_cfg_s ble_cube_cfg_st;

/* CUBE_CFG Service event handler type */
typedef void (*ble_cube_cfg_data_handler_st) (uint16_t char_uuid, uint8_t *data_ptr, uint16_t data_len);//(ble_cube_cfg_st * p_cube_cfg);

/* CUBE_CFG Service initialization structure.
   This structure contains the initialization information for the service. The application
   must fill this structure and pass it to the service using the ble_cube_cfg_init() function */
typedef struct
{
    ble_cube_cfg_data_handler_st data_handler;	/* Event handler to be called for handling received data. */
} ble_cube_cfg_init_st;


/* CUBE_CFG Service structure.
   This structure contains status information related to the service */
struct ble_cube_cfg_s
{
	uint8_t                  		uuid_type;          				/* UUID type for CUBE_CFG Service Base UUID. */
	uint16_t                 		service_handle;					/* Handle of CUBE_CFG Service. */
	ble_gatts_char_handles_t 		byte1_charac_handles;			/* Handle for BYTE1 characteristic. */
	ble_gatts_char_handles_t		byte2_charac_handles;			/* Handle for BYTE2 characteristic. */
	ble_gatts_char_handles_t 		short1_charac_handles;			/* Handle for SHORT1 characteristic. */
	ble_gatts_char_handles_t		short2_charac_handles;			/* Handle for SHORT2 characteristic. */
	ble_gatts_char_handles_t		special_op_charac_handles;		/* Handle for rebooting into DFU Upgrade */
	uint16_t                 		conn_handle;						/* Handle of the current connection. BLE_CONN_HANDLE_INVALID if not in a connection. */
	ble_cube_cfg_data_handler_st	data_handler;						/* Event handler to be called for handling received data. */
};




/* ------------- Exported functions prototypes --------------- */

/* Function for initializing the CUBE_CFG Service */
extern uint32_t ble_cube_cfg_init(ble_cube_cfg_st *, const ble_cube_cfg_init_st *);


/* Function for handling the CUBE_CFG Service's BLE events.
 * The CUBE_CFG Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the CUBE_CFG Service event handler of the
 * application if necessary */
extern void ble_cube_cfg_on_ble_evt(ble_cube_cfg_st *, ble_evt_t *);




/* End of file */


