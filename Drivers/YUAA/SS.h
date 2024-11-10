/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file SS.h
* @brief Sun Sensor Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elena W., Nich S., Elijah B.
* @version           1.0.0
* @date              2023.06.13
*
* @details           Declares drivers for the NA-DSS1-G0-R0 sun sensor
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "MCU_Init.h"
#include "User_types.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* NOT IMPLEMENTED on sun sensor
typedef enum {
	SS_SUN_NO = 0,	//Sun not detected
	SS_SUN_YES = 1	//Sun detected
} SS_Sun_e;
*/

typedef enum {
	SS_SETTING_UNLOCKED = 0,	//Settings unlocked
	SS_SETTING_LOCKED = 1		//Settings locked
} SS_SettingLock_e;

/* NOT IMPLEMENTED on sun sensor
typedef enum {
	SS_FLIGHT = 0b00,		//Sun sensor operating in flight mode
	SS_ERROR = 0b01,	//Sun sensor in an error state
	SS_DEBUG = 0b11		//Sun sensor operating in debug mode
} SS_Mode_e;
*/
/* NOT IMPLEMENTED on sun sensor
typedef enum {
	SS_STOP	= 0,	//Sun sensor is idle
	SS_RUN = 1	//Sun sensor actively taking measurements
} SS_State_e;
*/
/* NOT IMPLEMENTED on sun sensor
typedef struct {
	SS_Sun_e sun;							//Indicates whether sun is detected
	uint8_t error;						//Indicates whather error log is present
	uint8_t errorFull;					//Indicates whether error log is full
	SS_SettingLock_e settingLock;			//Indicates whether settings are locked for writing
	SS_Mode_e mode;						//Indicates current sun sensor mode
} SS_Status_t;
*/

/*Type for storing whether sun sensor is in the sun, together with
a sun sensor measurement as a vector in the sensor reference frame 
with 32-bit floating-point components. */
/* NOT IMPLEMENTED on sun sensor
typedef struct {
	SS_Sun_e sun;
	Vec3D_t vec;
} SS_Result_t;
*/

//Sun sensor status byte enum
typedef enum {
	SS_PACKET_ERR = 0xFF,
	SS_PACKET_TO = 0xFE,
	SS_CRC_ERR = 0xFD,
	SS_INV_CMD = 0xFC,
	SS_SETTING_LOCK = 0xFB,
	SS_BUSY = 0xBB,
	SS_NO_START = 0x02,
	SS_OK = 0x01
} SS_Ret_e;

//Return type for capturing both HAL and SS status after command execution
typedef struct {
	HAL_StatusTypeDef HAL_ret;
	SS_Ret_e SS_ret;
} CmdRet_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute GET_STATUS command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]      dataPtr - pointer to SS_Status_t to populate with sun sensor reply
* @return            CmdRet_t containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* NOT IMPLEMENTED on sun sensor
CmdRet_t SS_GetStatus(SS_Status_t* dataPtr);*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute GET_TEMP command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     dataPtr - pointer to int16_t to populate with sun sensor reply 
*					 in units of degC.
* @return            HAL_StatusTypeDef containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef SS_GetTemp(int16_t* dataPtr);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute GET_VECTOR command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     data - pointer to SS_Result_t to populate with sun sensor reply
* @return            CmdRet_t containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* NOT IMPLEMENTED on sun sensor
CmdRet_t SS_GetVector(SS_Result_t* data);*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute GET_ADC_CODE command and compute sun vector from ADC values
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     data - pointer to SS_Result_t to populate with sun sensor reply.
*					 The entire Vec3D_t will be set to 0 if sun sensor currently does not
*					 see the sun.
* @return            HAL_StatusTypeDef containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef SS_GetVector_FromADC(Vec3D_t* data);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute SETTING_BLOCK or SETTING_UNBLOCK command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      lock - boolean value specifying whether settings should be locked or 
*					 unlocked
* @return            HAL_StatusTypeDef containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef SS_SetSettingLock(SS_SettingLock_e lock);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute GO_TO_STOP_MODE or GO_TO_RUN_MODE command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      state - boolean value specifying whether run or stop mode should be 
*					 entered
* @return            CmdRet_t containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/* NOT IMPLEMENTED on sun sensor
CmdRet_t SS_SetState(SS_State_e state);*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute RESET command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef SS_Reset();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute SET_MEAS_INTERVAL command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 interval - interval between measurements in ms
* @return            HAL_StatusTypeDef containing result of command execution
* @detail			 If SS returns that settings are locked, this function will attempt to 
*					 unlock them before attempting to set the measurement interval.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef SS_SetMeasInterval(uint16_t interval);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute arbitrary command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 cmd - Command number, param - pointer to block of bytes to transmit as
*					 parameters (LSB first), paramLen - number of bytes to transmit as 
					 parameters (should be either 0 or 2), answerLen - number of
					 bytes to receive (LSB first), cmd_CRC - block of 4 bytes containing
					 32-bit CRC of the request frame, LSB first; if null pointer passed in, 
					 CRC will be calculated by this method.
* @param[output]	 answer - pointer to block of bytes to populate with answer					 
* @return            CmdRet_t containing result of SPI comms and sun sensor status byte
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
CmdRet_t ExecCmd(uint8_t cmd, uint8_t* param, uint8_t* answer, 
				 uint8_t paramLen, uint8_t answerLen, 
				 const uint8_t* cmd_CRC);