/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file MUX.c
* @brief MUX C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2022.06.23
*
* @details           Defines drivers for the PCA9546A I2C Multiplexer
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "MUX.h"
#include "MCU_Init.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include "stm32f4xx_hal.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Note about thread-safety:
The only thing the OBC does with the MUX is either single writes or single reads,
and there are no "transactions" consisting of multiple transfers in sequence.
So the Peripheral helper transaction API is not used here. */

HAL_StatusTypeDef MUX_GetConfig(uint8_t* outputVal)
{
	return Periph_Rx(DEV_MUX, outputVal, 1, HAL_DEFAULT_TO);
}

HAL_StatusTypeDef MUX_SelectCtrChip(uint8_t ctrChipNo)
{
	assert(ctrChipNo <= 7);
	uint8_t outputNo = ctrChipNo / 2;
	uint8_t msg = 1 << outputNo;
	return Periph_Tx(DEV_MUX, &msg, 1, HAL_DEFAULT_TO);
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* PARKED CODE
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
void MUX_Reset() // TODO: confirm we have reserved a pin for MUX reset
{
	HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, GPIO_PIN_RESET);
	osDelay(1);
	HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, GPIO_PIN_SET);
}
*/