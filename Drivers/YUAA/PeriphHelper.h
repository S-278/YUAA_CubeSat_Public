/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file PeriphHelper.h
* @brief Peripherals helper header file
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @details           Declares helpers for communicating with peripherals
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "system_manager.h"
#include "stm32f4xx_hal.h"
#include "User_types.h"
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPE DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef enum {
	PERIPH_Rx,
	PERIPH_Tx,
} PeriphComms_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#define HAL_DEFAULT_TO	(50) // Default timeout for HAL calls in polling mode is 50ms

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
	This helper supports the following devices:
	- DEV_RTC
	- DEV_EPS
	- DEV_GGB_CTRL
	- DEV_CTR_0 and DEV_CTR_1 (use MUX to reach other counters)
	- DEV_DAC_1 and DEV_DAC_2
	- DEV_MUX
	- DEV_RTD_1 and DEV_RTD_2
	- DEV_SS
	- DEV_TCV
	- DEV_EEPROM (`Periph_BeginTransact' and `Periph_EndTransact' only)
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */


/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Begin a transaction with a peripheral device
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device with which transaciton is being started
* @param[input]		 minSz - minimum number of bytes transferred as part of the transaction.
*					 This need not be exact as it is used as an optimization hint.
* @param[input]		 timeout - maximum time to wait for device availability. 
*					 Pass `HAL_MAX_DELAY' to wait indefinitely.
* @return            `E_PENDING' if device was not available within the timeout, 
*					 `E_OK' otherwise.
* @detail			 Nested transacitons with the same device are supported,
*					 but nested transactions with different devices are not!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType Periph_BeginTransact(DevName_e dev, uint16_t minSz, uint32_t timeout);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief End a transaction with a peripheral device
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device with which transaciton is being ended
* @return            none
* @detail			 To finish a nested transaction, one call of this function is needed
*					 for every call to `Periph_BeginTransact'
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Periph_EndTransact(DevName_e dev);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Transmit to or receive from a peripheral device
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      type - PeriphComms_e indicating whether to transmit or to receive
* @param[input]		 dev - device to communicate with;
* @param[input]		 data - pointer to data buffer;
* @param[input]		 size - number of bytes to transfer;
* @param[input]		 timeout - maximum time allowed for data transfer in ms
* @return            HAL_StatusTypeDef containing result of interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef Periph_Comm(PeriphComms_e type, DevName_e dev, uint8_t* data, uint16_t size, uint32_t timeout);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Transmit data to a peripheral device
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device to transmit to;
*					 dataToTransmit - pointer to buffer containing data to transmit;
*					 size - number of bytes to transmit;
*					 timeout - maximum time allowed for data transfer
* @return            HAL_StatusTypeDef containing result of interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define Periph_Tx(dev, dataToTransmit, size, timeout) Periph_Comm(PERIPH_Tx, dev, dataToTransmit, size, timeout)

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Receive data from a peripheral device
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device to receive from;
*					 buffer - pointer to buffer to fill with received data;
*					 size - number of bytes to receive;
*					 timeout - maximum time allowed for data transfer
* @return            HAL_StatusTypeDef containing result of interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define Periph_Rx(dev, buffer, size, timeout) Periph_Comm(PERIPH_Rx, dev, buffer, size, timeout)

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Begin safe access to shared memory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return               none
* @detail				Accesses to memory shared by tasks must be guarded by a call to 
*						`SharedMem_BeginAccess' and a corresponding call to
*						`SharedMem_EndAccess'.
* @note					Should not be called before the scheduler is started.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void SharedMem_BeginAccess();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief End safe access to shared memory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return               none
* @detail				Accesses to memory shared by tasks must be guarded by a call to
*						`SharedMem_BeginAccess' and a corresponding call to
*						`SharedMem_EndAccess'.
* @note					Should not be called before the scheduler is started.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void SharedMem_EndAccess();

#ifdef DEBUG_ENABLED
/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Signal expected transmission
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      dev - device to which transmission is expected
* @param[input]		 expectedMsg - pointer to block holding expected transmission
* @param[input]		 expectedSz - size of expectedMsg
* @note				 Use this method to test that a certain message is transmitted. The next
*					 time Periph_Tx is called on `dev', it will check that the
*				     contents of `dataToTransmit' match `expectedMsg', otherwise Periph_Tx will return
*				     HAL_ERROR
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Periph_ExpectTx(DevName_e dev, const uint8_t* expectedMsg, uint16_t expectedSz);
#endif
