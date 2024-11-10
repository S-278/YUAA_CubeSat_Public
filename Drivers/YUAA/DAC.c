/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file DAC.c
* @brief DAC C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.1
* @date              2022.06.16
*
* @details           Defines drivers for the Maxim MAX520 Digital-to-Analog Converter
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "DAC.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) VARIABLES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
//static uint8_t shutdownFlag;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Note about thread-safety:
The only thing the OBC does with the DACs is write to them,
and there are no "transactions" consisting of multiple transfers in sequence.
So the Peripheral helper transaction API is not used here. */

HAL_StatusTypeDef DAC_SetOutput(DevName_e DAC_dev, DAC_SetupType output)
{
	assert(DAC_dev == DEV_DAC_1 || DAC_dev == DEV_DAC_2);
	//if (shutdownFlag) return 0x04;

	uint8_t msg[8]; //a command byte and an output byte for each output pin
	for (uint8_t i = 0; i < 4; i++)
	{
		msg[i * 2] = i; //set command byte equal to number of output pin
		msg[i * 2 + 1] = output.outputVals[i]; //set output byte equal to value supplied in output struct
	}

	return Periph_Tx(DAC_dev, msg, 8, HAL_DEFAULT_TO);
}

HAL_StatusTypeDef DAC_ResetOutput(DevName_e DAC_dev)
{
	assert(DAC_dev == DEV_DAC_1 || DAC_dev == DEV_DAC_2);
	//uint8_t msg = 0b00010000 | (shutdownFlag << 3); //command byte to reset outputs, preserving current shutdown state
	uint8_t msg = 0b00010000 | (0 << 3); //command byte to reset outputs
	return Periph_Tx(DAC_dev, &msg, 1, HAL_DEFAULT_TO);
}

/*
HAL_StatusTypeDef DAC_Shutdown(uint8_t DAC_chip_addr, uint8_t shutdown)
{
	if (shutdown) shutdown = 1; //force value of 1 for logical true
	
	//If the chip is already in the desired mode, do nothing
	if ( !(shutdown ^ shutdownFlag) ) return HAL_OK;

	uint8_t msg = shutdown << 3;	
	HAL_StatusTypeDef retstat = Periph_Tx(DAC_dev, &msg, 1, HAL_DEFAULT_TO);
	
	if (retstat == HAL_OK) shutdownFlag = shutdown;
	return retstat;
}
*/