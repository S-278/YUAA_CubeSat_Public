/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TCV.h
* @brief UHF transceiver drivers Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.2.0
* @date              2023.06.13
*
* @details           Declares drivers for the EnduroSat UHF Transceiver Type II
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#define TCV_MAX_BEACON_LEN (77) // Maximum length of beacon contents. 

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef struct {
	uint8_t HFXT;
	uint8_t FRAM;
	uint8_t RFTS;
	uint8_t resetCount;
} TCV_Err_t; // Struct to hold all TCV error statuses. 1 means error, 0 means OK.

typedef enum {
	TCV_1200bps = 0b000,
	TCV_2400bps = 0b001,
	TCV_4800bps = 0b010,
	TCV_9600bps_narrow = 0b011,
	TCV_9600bps_wide = 0b100,
	TCV_19200bps_narrow = 0b101,
	TCV_19200bps_wide = 0b110,
	TCV_19200bps_Xwide = 0b111
} TCV_RFMode_e; // Enum to specify RF mode
/* Default TCV link speed */
#define TCV_BASE_RF_MODE (TCV_1200bps)


typedef struct {
	uint32_t numRxPackets;
	uint32_t numRxPacketsDropped;
} TCV_RxPacket_t; // Struct to hold received packet statistics

typedef enum {
	TCV_AntReleaseDisabled = 0b00000000,		// Both First and EN flags cleared
	TCV_AntReleaseEnabled = 0b00000001,		// EN set but First cleared
	TCV_AntReleaseRobust = 0b00010001		// EN and First set
} TCV_AntRelease_e; 

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Read TCV error statuses (command 0x00)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	 result - pointer to struct to populate with answer
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_ReadErrorStat(TCV_Err_t* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Signal the transceiver to reset (command 0x00)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_Reset(void);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set transceiver RF mode (command 0x00)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 mode - which RF mode to set
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_SetRFMode(TCV_RFMode_e mode);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Turn beacon on/off (command 0x00)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 state - 0 for off, anything else for on
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_SetBeacon(char state);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Turn Pipe (transparent) mode on/off (command 0x00)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 state - 0 for off, anything else for on
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_SetPipe(char state);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Read number of received and dropped packets (commands 0x04 and 0x05)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	 result - pointer to TCV_RxPacket_t to populate with reply
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_GetRxPackets(TCV_RxPacket_t* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get transceiver temperature (command 0x0a)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	 result - pointer to float to populate with reply
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_GetTemp(float* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get antenna release configuration (command 0xf2)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	 result - pointer to TCV_AntRelease_e enum to populate with reply
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_GetAntRelease(TCV_AntRelease_e* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set antenna release configuration (command 0xf2)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 mode - release mode to set, minutes - deployment timer duration in 
*					 minutes (minimum 10)
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_SetAntRelease(TCV_AntRelease_e mode, uint8_t minutes);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set beacon content (command 0xfb)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 message - pointer to null-terminated string containing beacon content
* @return            HAL_StatusTypeDef containing result of I2C comms.
* @note				 Max TCV_MAX_BEACON_LEN characters will be copied from message
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_SetBeaconContent(const char* message);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set beacon period (command 0x07)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 seconds - number of seconds between beacon transmissions. Must not be 0.
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_SetBeaconPeriod(uint16_t seconds);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Turn off pipe mode and beacon, and change transceiver I2C address so it stops 
*        receiveng commands from the OBC (commands 0x00 and 0xfc)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_Kill(void);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute arbitrary ESTTC read command and store answer in answerBuf
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      cmd - ESTTC command number, answerMaxLen - maximum length of reply
*					 excluding the [B][C..C]<CR> suffix (minimum 3)
* @param[output]	 answerBuf - pointer to block into which to copy the TCV answer, size
*					 of answerBuf is assumed to be answerMaxLen+1
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_ESTTC_R(uint8_t cmd, uint8_t answerMaxLen, char* answerBuf);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Execute arbitrary ESTTC write command and store answer in answerBuf
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      cmd - ESTTC command number, writeData - pointer to null-terminated  
*					 string to append after pefix, answerMaxLen - maximum length of reply
*					 excluding the [B][C..C]<CR> suffix (minimum 3)
* @param[output]	 answerBuf - pointer to block into which to copy the TCV answer, size
*					 of answerBuf is assumed to be answerMaxLen+1. The reply is discarded 
*					 if answerBuf is passed as NULL.
* @return            HAL_StatusTypeDef containing result of I2C comms.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_ESTTC_W(uint8_t cmd, const char* writeData, uint8_t answerMaxLen, char* answerBuf);
