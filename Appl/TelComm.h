/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TelComm.h
* @brief Telecommand handlers Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2023.07.01
*
* @details           Declares handlers for YUAA telecommands
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "EEPROM_emul.h"
#include "Memory_Logs.h"
#include "TCV.h"
#include "ff.h"
#include "stm32f4xx_hal.h"
#include <time.h>
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Size of Data Field 2 of a radio packet in bytes
#define RADIO_PACKET_DF2_SZ (128)

#define CMD_NUM_FMT "%.2X"
#define print_OK_W(buf, cmd_num) sprintf(buf, "OK+W" CMD_NUM_FMT "\r", cmd_num)

#define ACKNOWLEDGE_FMT "%c%c%c%c%c%c%c"
#define ACKNOWLEDGE(buf) sprintf(buf, ACKNOWLEDGE_FMT"\r", 0xA0, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Telecommand result enum
typedef enum {
	TELCOMM_SUCCESS,		// successfully executed
	TELCOMM_CALL_NEXT,		// partially executed; the "next packet" function for this command must still be called to execute completely
	TELCOMM_ERR_FORMAT,		// misformatted
	TELCOMM_ERR_INV_ARG,	// argument out of range
	TELCOMM_ERR_CRC,		// ESTTC CRC wrong 
	TELCOMM_ERR_FAULT,		// device fault occurred during execution
	TELCOMM_ERR_RES_LOCKED,	// execution impossible due to power level, existing device fault, or resource usage
	TELCOMM_ERR_RUNTIME,	// expected error during command execution
	TELCOMM_ERR_FORBIDDEN,	// this command is illegal in the current context 
	TELCOMM_ERR_UNSUPPORTED,// unrecognized command
} TelComm_Result_e;

// Type for the Data Field 2 of a radio packet
typedef struct {
	uint8_t data[RADIO_PACKET_DF2_SZ];		// Data Field 2 of a radio packet
	uint8_t sz;								// Number of bytes of `data' which are actually used
} RadioPacket_t;

typedef struct {
	uint8_t done;			// Flag is set by `TelComm_GetLogsInRange_Next' upon generating the last packet
	// TODO
} TelComm_GetLogsInRange_State_t;

typedef struct {
	uint8_t done;			// Flag is set by `TelComm_SetPassTimes_Next' upon receiving the last packet
	// TODO
} TelComm_SetPassTimes_State_t;

typedef struct {
	uint8_t done;			// Flag is set by `TelComm_SetGndCoordinates_Next' upon receiving the last packet
	// TODO
} TelComm_SetGndCoordinates_State_t;

typedef struct {
	uint8_t done;			// Flag is set by `TelComm_SetTLE_Next' upon writing the last TLE packet
	// TODO
} TelComm_SetTLE_State_t;

typedef struct {
	uint8_t done;			// Flag is set by `TelComm_GetAttMat_Next' upon writing the matrix data packet
	// TODO
} TelComm_GetAttMat_State_t;

// Type to hold internal state of the command handler library.
// Every task which uses stateful functions from this header
// should declare a variable of this type on its stack,
// and pass a pointer to it to stateful command handlers when necessary.
// DO NOT directly use any members of this struct!
typedef struct {
	TelComm_GetLogsInRange_State_t GetLogsInRange_State;
	TelComm_SetPassTimes_State_t SetPassTimes_State;
	TelComm_SetGndCoordinates_State_t SetGndCoordinates_State;
	TelComm_SetTLE_State_t SetTLE_State;
	TelComm_GetAttMat_State_t GetAttMat_State;
} TelComm_State_t;

typedef enum {
    TELCOMM_CMD_RECITE_BEACON      = 0xF0U,
    TELCOMM_CMD_SET_PWR_CEIL_FLOOR = 0xF6U,
    TELCOMM_CMD_SET_PWR_TH         = 0xF7U,
    TELCOMM_CMD_SET_TEMP_TH        = 0xF8U,

    TELCOMM_CMD_ACKNOWLEDGE        = 0xA0U,
    TELCOMM_CMD_DOWNLINK_REQ       = 0xA2U,
    TELCOMM_CMD_LINK_SPEED         = 0xA8U,

    TELCOMM_CMD_SET_OBC_SPEED      = 0x10U,
    TELCOMM_CMD_OBC_TIME           = 0x11U,
    TELCOMM_CMD_OBC_TIME           = 0x11U,
    TELCOMM_CMD_COUNTERS           = 0x12U,
    TELCOMM_CMD_GET_LOG_STATUS     = 0x15U,
    TELCOMM_CMD_LOGS_RANGE         = 0x17U,
    TELCOMM_CMD_SET_PASS_TIMES     = 0x18U,
    TELCOMM_CMD_SET_RADIO_TIMEOUTS = 0x19U,
    TELCOMM_CMD_SET_GND_COORDS     = 0x1AU, 

    TELCOMM_CMD_ENABLE_GBB         = 0x30U,
    TELCOMM_CMD_GBB_EXT            = 0x31U,
    TELCOMM_CMD_SET_GBB_EXT_TARG   = 0x32U,
    TELCOMM_CMD_SET_GBB_SPEED      = 0x33U,
    TELCOMM_CMD_SET_GBB_SUN_REQ    = 0x34U,

    TELCOMM_CMD_SET_DETUMBLING     = 0x40U,
    TELCOMM_CMD_SET_MTQ_POL        = 0x41U,
    TELCOMM_CMD_SET_ADCS_LIM       = 0x42U,
    TELCOMM_CMD_SET_SUN_POS        = 0x43U,
    TELCOMM_CMD_SET_TLE            = 0x44U,
    TELCOMM_CMD_GET_ATT_MATRIX     = 0x45U,
    TELCOMM_CMD_GET_OMEGA          = 0x46U,
    TELCOMM_CMD_GET_SAT_EQ         = 0x47U,
    TELCOMM_CMD_GET_PHASE          = 0x48U,
    TELCOMM_CMD_GET_BDOT           = 0x49U,
    TELCOMM_CMD_GET_BODY_VEC       = 0x4BU,
    TELCOMM_CMD_SET_MTQ_OUT        = 0x4EU,
    TELCOMM_CMD_ADCS_OVERRIDE      = 0x4FU,

    TELCOMM_CMD_SET_PULSE_BINS     = 0x51U
} TelComm_Cmd_e;


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0xF_ - System-wide ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Recite beacon (RF0)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     output - pointer to output buffer
* @return			 TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_ReciteBeacon(char* output);

/* NOTE: Clear device faults (WF4) has no handler, 
as its execution is noreturn. */

/* NOTE: Set power ceiling/floor (WF6) has no handler,
as its execution is noreturn. */

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set power thresholds (WF7)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		newTh - pointer to PwrDistrib_Th_t containing new thresholds
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetPwrTh(char* output, const PwrDistrib_Th_t* newTh);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set temperature thresholds (WF8)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]     newTh - pointer to Temperature_Thresholds_t containing the new thresholds
* @return			TelComm_Result_e giving the result of command execution
* @detail			Updates the corresponding EEPROM variable
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetTempTh(char* output, const Temperature_Thresholds_t* newTh);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0xA_ - Radio link ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Print acknowledge (0xA0)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			none
* @detail			This function only prints an Acknowledge to the given buffer. It does 
*					not implement the state changes or flow control related to reception or 
*					transmission of an Acknowledge packet!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_AcknowledgePrint(char* output);

/* NOTE: Downlink request (0xA2) has no handler,
as it serves only to change the state of the TelComm Parser. */

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set link speed (WA8)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]     newMode - new TCV RF mode
* @return			HAL_StatusTypeDef containing result of TCV comms
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TelComm_X_SetLinkSpeed(char* output, TCV_RFMode_e newMode);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x1_ - OBC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set OBC CPU speed (W10)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]     newMHz - new OBC CPU speed in MHz
* @return			TelComm_Result_e giving the result of command execution
* @note				Details of this method are TODO and its signature may change
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TelComm_X_SetCPUSpeed(char* output, uint8_t newMHz);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get OBC time (R11)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TelComm_X_GetOBCTime(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set OBC time (W11)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		newT - new time to set as the on-board time
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TelComm_X_SetOBCTime(char* output, time_t newT);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get reset counters (R12)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_GetResetCounters(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get log status (R15)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TelComm_X_GetLogStatus(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get logs in range (R16), initialize command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @param[input]		logType - type of log to search
* @param[input]		startT - reporting period start time
* @param[input]		endT - reporting period end time
* @return			TelComm_Result_e giving the result of command execution
* @detail			This function initializes the command and generates the initial
*					command response. Use `TelComm_GetLogsInRange_Next' to get the next 
*					packet after initialization.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TelComm_X_GetLogsInRange_Init(char* output, TelComm_State_t* const state, \
	LOG_TYPE logType, time_t startT, time_t endT);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get logs in range (R16), get next packet
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @detail			This function generates the next log packet. Use only after initializing
*					the command state using `TelComm_GetLogsInRange_Init'.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TelComm_X_GetLogsInRange_Next(char* output, TelComm_State_t* const state);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Clear logs in range (W17)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		logType - type of log to clear
* @param[input]		startT - clear period start time
* @param[input]		endT - celar period end time
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TelComm_X_ClearLogsInRange(char* output, LOG_TYPE logType, time_t startT, time_t endT);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set pass times (W18), set next packet
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @param[input]		passTimes - pointer to block of time_t to write to SD
* @param[input]		numTimes - number of elements of `passTimes'
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TelComm_X_SetPassTimes_Next(char* output, \
	time_t* passTimes, uint8_t numTimes);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set pass timeout (W19)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		newTimeouts - pointer to Radio_Timeouts_t containing new timeouts
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetPassTimeout(char* output, const Radio_Timeouts_t* newTimeouts);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x3_ - GGB ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Enable/disable GGB (W30)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		enable - flag indicating whether to enable or disable GGB
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_EnableDisableGGB(char* output, uint8_t enable);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get GGB extension (R31)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_GetGGBExtension(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set GGB extension (W31)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		overrideExt - override GGB extension level in meters
* @return			TelComm_Result_e giving the result of command execution
* @note				This command has many side effects! Carefully read its specification!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_SetGGBExtension(char* output, float overrideExt);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set GGB extension target (W32)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		target - new GGB extension target in meters
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetGGBExtensionTarget(char* output, float target);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set GGB speed (W33)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		newSp - new GGB extension speed
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetGGBSpeed(char* output, uint8_t newSpeed);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Enable/disable GGB sun requirement (W34)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		enableReq - flag indicating whether to enable GGB sun requirement
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetGGBSunReq(char* output, uint8_t enableReq);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x4_ - ADCS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Enable/disable detumbling (W40)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		enable - flag indicating whether to enable detumbling
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_EnableDisableDetumbling(char* output, uint8_t enable);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set ADCS thresholds (W42)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		newTh - pointer to ADCS_Thresholds_t containing new thresholds
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetADCSTh(char* output, const ADCS_Thresholds_t* newTh);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set TLE (W44), initialize command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @return			TelComm_Result_e giving the result of command execution
* @detail			This function initializes the command and generates the initial
*					command response. Use `TelComm_SetTLE_Next' to set the next 
*					packet after initialization.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_SetTLE_Init(char* output, TelComm_State_t* const state);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set TLE (W44), set next packet
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @return			TelComm_Result_e giving the result of command execution
* @detail			This function writes the next TLE packet. Use only after initializing
*					the command state using `TelComm_SetTLE_Next'.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TelComm_X_SetTLE_Next(char* output, TelComm_State_t* const state);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get attitude matrix (R45), initialize command
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @return			TelComm_Result_e giving the result of command execution
* @detail			This function initializes the command and generates the initial
*					command response. Use `TelComm_GetAttMat_Next' to get the next 
*					packet after initialization.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_GetAttMat_Init(char* output, TelComm_State_t* const state);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get attitude matrix (R45), get next packet
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		state - pointer to library state variable
* @return			TelComm_Result_e giving the result of command execution
* @detail			This function generates the matrix data packet. Use only after initializing
*					the command state using `TelComm_GetAttMat_Init'.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_GetAttMat_Next(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get omega (R46)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_GetOmega(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get sat equatorial (R47)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_GetSatEquatorial(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get phase (R48)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TelComm_X_GetPhase(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get Bdot vector (R49)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_GetBdotVec(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get body vectors (R4B)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_GetSunVec(char* output);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set MTQ timed output (W4E)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		numSeconds - number of seconds for which to maintain MTQ output
* @param[input]		MTQ_vec - pointer MTQ output vector, where components are given as 
*					signed integer numbers of percent
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
status_t TelComm_X_GetBVec(char* output);

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x5_ - CRD ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Set pulse bins (W51)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	output - output buffer
* @param[input]		params - pointer to DAC_InitParams_t containing new DAC outputs
* @return			TelComm_Result_e giving the result of command execution
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TelComm_X_SetPulseBins(char* output, const DAC_InitParams_t* params);
