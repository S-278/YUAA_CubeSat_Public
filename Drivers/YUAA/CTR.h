/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file CTR.h
* @brief Counter Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.2.0
* @date              2023.06.13
*
* @details           Declares drivers for the PCF8583 counters
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "PeriphHelper.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// There are 4 even-numbered and 4 odd-numbered counters
typedef enum {
	CTR_EVEN = 0,
	CTR_ODD = 1,
} CTR_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#define CTR_I2C_TO (HAL_DEFAULT_TO)

//Value to set the control register of each counter in CTR_InitForScint().
/*
MSb 7 - 1, counting off
b 6 - 0, do not hold last count
b 5-4 - 10, event counter mode
b 3 - 0, masking off
b 2 - 1, alarm enable (for overflow control)
b 1 - 0, no flag
LSb 0 - 0, no flag
*/
#define CTR_CTRL_REG_VAL 0b10100100

//Value to set the alarm control register of each counter in CTR_InitForScint().
/*
MSb 7 - 1, enable alarm interrupt
b 6 - 0, disable timer alarm
b 5-4 - 01, event alarm
b 3 - 0, disable timer interrupt
b 2-0 LSb - 000, disable timer
*/
#define CTR_ALARM_REG_VAL 0b10010000

//Value at which the alarm will trigger.
//Set to something close to 1 million to warn for overflow
#define CTR_ALARM_VAL ((uint32_t)(900000))

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Initialize a counter for counting scintillator pulses
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 ctr - counter to initialize
* @return            HAL_StatusTypeDef containing result of I2C comms
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef CTR_InitForScint(CTR_e ctr);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Initialize a counter for counting GGB motor encoder pulses
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 addr - address of counter to initialize, one of CTR_ADDR_EVEN or
*					 CTR_ADDR_ODD
* @return            HAL_StatusTypeDef containing result of I2C comms
* @note				 TODO
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
//HAL_StatusTypeDef CTR_InitForGGB(uint8_t addr);
// TODO - is this needed?

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Read and reset counts
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      ctr - counter to read, 
*					 data - pointer to populate with count data
*					 (pass in a NULL pointer to reset the counts without reading them),
*					 overflow - pointer to byte that will be set to 1 if an overflow
*					 happened after the last reading, and to 0 otherwise
* @return            HAL_StatusTypeDef containing result of I2C comms
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef CTR_ReadReset(CTR_e ctr, uint32_t* data, uint8_t* overflow);