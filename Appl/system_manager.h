/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file system_manager.h
* @brief Header of system_manager.c
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Rome T., Elijah B.
*
* @details           Declares global system functions
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
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Maximum string length of system error messages
#define SYS_ERR_MAX_LEN     (7)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef enum {
	L0 = 0,		// CRD, SS, ADCS, radio unpowered, MCU put into sleep state until next sun phase
	L1,			// CRD, SS, ADCS, radio unpowered (LUP 3V3)
	L2,			// CRD, SS, ADCS unpowered (stop all ADCS tasks, 5V unpowered)
	L3,			// CRD unpowered (LUP 5V)
	L4,			// All systems on
	NumPwrLvls
} Sys_PowerLevel_e;

typedef enum {
    DEV_MTM_LO,
    DEV_MTM_HI,
    DEV_RTC,
    DEV_SD,
    DEV_OBC_MCU_RTD,

    DEV_EPS,
    DEV_EPS_BATT_1,     // Does not support raising errors
    DEV_EPS_BATT_2,     // Does not support raising errors

    DEV_TCV,

    DEV_MUX,
    DEV_CTR_0,
    DEV_CTR_1,
    DEV_CTR_2,
    DEV_CTR_3,
    DEV_CTR_4,
    DEV_CTR_5,
    DEV_CTR_6,
    DEV_CTR_7,
    DEV_DAC_1,
    DEV_DAC_2,
    DEV_RTD_1,
    DEV_RTD_2,

    DEV_GGB_CTRL,

    DEV_SS,

    DEV_EEPROM,

    NUM_DEV_NAMES,
} DevName_e;


typedef enum {
    DEV_STAT_NORMAL = 0b1,          // Device is powered and operational. No errors raised.
    DEV_STAT_WARNING = 0b10,        // Device is powered and operational. Errors were reported for this device.
    DEV_STAT_FAULT = 0b100,         // The device is not operational because of an error.
    DEV_STAT_UNPOWERED = 0b1000,    // The device is not operational because the current power level is too low.
} DevStat_e;

typedef enum {
    TASK_TASK_MONITOR,
    TASK_SYSMAN,
    TASK_RADIO_CTL,
    TASK_ADCS_CTL,
    TASK_ESTTC_UART,
    TASK_LOG_WRITER,
    TASK_ADCS_DETUMB,
    TASK_ADCS_GGB,
    TASK_ADCS_MEAS,
    TASK_PWRDIST,
    TASK_RADIO_AUTODL,
    TASK_SERVICES,
    TASK_CRD_DATA,
    TASK_SCRUBBER,
    TASK_INIT,
    NUM_RTOS_TASKS
} RTOS_TASKS_e;

typedef enum {
    LERR_FATAL,
    LERR_WARN,
} LogicError_Behavior_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Check-in for application tasks
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            none
* @detail            Must be called by every RTOS task regularly at a safe point
*                    in its loop. It is possible that this call does not return because
*                    SysMan decides to delete the calling task!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Sys_IAmAlive();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Report failure of a hardware device
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - specifies the device which experienced an error; error_status -
*					 value returned by low-level driver
* @note              Reporting SS errors: set the upper two bytes of `errorStatus' to 0, 
*                    the lowest byte to the HAL_StatusTypeDef, and the next-lowest byte
*                    to the SS_Ret_e.
* @return            none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Sys_RaiseDevError(DevName_e dev, unsigned int errorStatus);
#define SYS_SS_ERRORSTAT(HAL_ret, SS_ret) ( (unsigned int)0U | (uint8_t)HAL_ret | ( ((uint16_t)SS_ret) << 8 ) )

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Report logic error
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      behavior - error behavior
* @param[input]      format - format of error message. Formated error message will be 
*                    silently truncated to `SYS_ERR_MAX_LEN'.
* @return            none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void _Sys_RaiseLogicError(const char* file, int line, LogicError_Behavior_e behavior, const char* format, ...);
#define Sys_RaiseLogicError(behavior) _Sys_RaiseLogicError(__FILE__, __LINE__, behavior, "")
#define Sys_RaiseLogicErrorMsg(behavior, format, ...) _Sys_RaiseLogicError(__FILE__, __LINE__, behavior, format, __VA_ARGS__)

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get device status
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device the status of which is queried
* @return            DevStat_e reporting the status of the device
* @note              A device can be offline if a fault has ocurred or if the current system
*                    power level does not support it.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
DevStat_e Sys_GetDevStatus(DevName_e dev);
/* Use this macro to check if a device is available for work. */
#define Sys_IsDevAvail(dev) (Sys_GetDevStatus(dev) & (DEV_STAT_NORMAL | DEV_STAT_WARNING))


#ifdef DEBUG_ENABLED

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Start RTOS task
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      task - task to start
* @return            none
* @note              For debug only - does not perform any checking!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Sys_DebugTaskStart(RTOS_TASKS_e task);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Stop RTOS task
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      task - task to stop
* @return            none
* @note              For debug only - does not perform any checking!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Sys_DebugTaskStop(RTOS_TASKS_e task);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Expect a device error
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device from which to expect an error.
* @param[input]      errorStatus - pointer to unsigned int containing expected error code
*                    value. Will be read at time of call.
*                    Pass in NULL to expect any error code from this device. 
* @param[input]      callback - pointer to uint8_t. This uint8_t will be set to 1 when
*                    an expected device error is raised, and should therefore be initiaziled
*                    to 0 beforehand. Pass NULL if not required.
* @return            none
* @note              For debug only! Next time a device error is raised, if it matches
*                    the expectation, a warning message will be printed, but the system
*                    will not be stopped.
*                    The expected error always resets after the next raised device error.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Sys_ExpectDevError(DevName_e dev, const unsigned int* errorStatus, uint8_t* callback);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Expect a logic error
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      file - file from which to expect a logic error, provide using `__FILE__'
* @param[input]      line - line from which to expect a logic error. If 0 is passed,
*                    an error from any line in the specified `file' will be expected.
* @param[input]      callback - pointer to uint8_t. This uint8_t will be set to 1 when 
*                    an expected logic error is raised, and should therefore be initiaziled
*                    to 0 beforehand. Pass NULL if not required.
* @return            none
* @note              For debug only! Next time a logic error is raised, if it matches
*                    the expectation, a warning message will be printed, but the system 
*                    will not be stopped. 
*                    The expected error always resets after the next raised logic error.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Sys_ExpectLogicError(const char* file, int line, uint8_t* callback);

#endif
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Use this macro to try executing `operation' which is susceptible to `dev' failing 
until the operation succeeds or `dev' is determined to be in fault. 
After calling this macro, if `retstatVar' equals `successVal',
the operation succeeded. Else the device is not available,
and any appropriate errors have already been raised to SysMan.

Example usage:

HAL_StatusTypeDef myHALretstat; time_t currT;
Sys_TryDev(myHALretstat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
if (myHALretstat == HAL_OK) (operation succeded);
else (operation failed because RTC is in fault);

*/
#define Sys_TryDev(retstatVar, operation, successVal, dev)  \
{                                                           \
    retstatVar = ~successVal;                               \
	while (Sys_IsDevAvail(dev)) {                           \
		retstatVar = operation;                             \
		if (retstatVar == successVal) break;                \
		else Sys_RaiseDevError(dev, retstatVar);            \
	}                                                       \
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Initialize all sat components and start RTOS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            Noreturn because RTOS kernel is started
* @note              Exclusively for main
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
_Noreturn void Sys_X_Init();
