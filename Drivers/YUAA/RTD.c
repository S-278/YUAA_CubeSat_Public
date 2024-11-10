/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file RTD.c
* @brief RTD Driver C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2022.06.16
*
* @details           Defines driver for temeprature sensors in CRD
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "RTD.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include <stdint.h>
#include <math.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define RTD_CHK_HAL(x, lbl) if (x != HAL_OK) goto lbl
#define RTD_I2C_TO (HAL_DEFAULT_TO)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef enum {
	REG_TEMP = 0, //temperature value register 0x00
	REG_CONFIG, //config register 0x01
	REG_HYST, //temp hysteresis register 0x02
	REG_TOS, //temp threshold register 0x03
	REG_ONESHOT //oneshot trigger register 0x04
} RTD_RegTypeDef;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Use only within a transaction!
HAL_StatusTypeDef RTD_SetAddrPtrReg(DevName_e RTD_dev, RTD_RegTypeDef reg)
{
	uint8_t msg = reg;
	return Periph_Tx(RTD_dev, &msg, 1, RTD_I2C_TO);
}

// Use only within a transaction!
HAL_StatusTypeDef RTD_GetConfigReg(DevName_e RTD_dev, uint8_t* configVal)
{
	HAL_StatusTypeDef retstat = RTD_SetAddrPtrReg(RTD_dev, REG_CONFIG);
	RTD_CHK_HAL(retstat, end);
	retstat = Periph_Rx(RTD_dev, configVal, 1, RTD_I2C_TO);
end:
	return retstat;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef RTD_GetTemp(DevName_e RTD_dev, float* output)
{
	assert(output);
	HAL_StatusTypeDef retstat;
	uint8_t msg, configVal;

	Periph_BeginTransact(RTD_dev, 6, HAL_MAX_DELAY);

	retstat = RTD_GetConfigReg(RTD_dev, &configVal); RTD_CHK_HAL(retstat, end);
	//if chip is in oneshot mode, trigger a temperature conversion before reading the temperature
	if (configVal & 0b00100000)
	{
		msg = 0x04;
		retstat = Periph_Tx(RTD_dev, &msg, 1, RTD_I2C_TO);
		RTD_CHK_HAL(retstat, end);
	}

	//set address pointer register
	retstat = RTD_SetAddrPtrReg(RTD_dev, REG_TEMP);
	RTD_CHK_HAL(retstat, end);

	uint16_t rawTemp = 0; uint8_t tempreg[2];
	retstat = Periph_Rx(RTD_dev, tempreg, 2, RTD_I2C_TO);
	RTD_CHK_HAL(retstat, end);

	rawTemp = (tempreg[0] << 8) + tempreg[1];
	float temp;
	//Perform temperature value conversion as specified in RTD datasheet
	if (rawTemp >= 0) temp = (rawTemp >> 4) / 16.0;
	else temp = ((rawTemp >> 4) - 4096) / 16.0;

	*output = temp;
end:
	Periph_EndTransact(RTD_dev);
	return retstat;
}

HAL_StatusTypeDef RTD_SetOneshotMode(DevName_e RTD_dev, uint8_t oneshot)
{
	if (oneshot) oneshot = 1; //force value of 1 for logical true

	HAL_StatusTypeDef retstat;
	uint8_t msg, configVal;

	Periph_BeginTransact(RTD_dev, 2, HAL_MAX_DELAY);

	retstat = RTD_GetConfigReg(RTD_dev, &configVal); RTD_CHK_HAL(retstat, end);

	//set address pointer register to config register
	retstat = RTD_SetAddrPtrReg(RTD_dev, REG_CONFIG);
	RTD_CHK_HAL(retstat, end);

	//prepare byte to be written to the config register such that previous settings are retained
	if (oneshot) msg = configVal | (oneshot << 5);
	else msg = configVal & (oneshot << 5);

	//write to the config register to update the settings
	retstat = Periph_Tx(RTD_dev, &msg, 1, RTD_I2C_TO);
	RTD_CHK_HAL(retstat, end);

end:
	Periph_EndTransact(RTD_dev);
	return retstat;
}

/*
HAL_StatusTypeDef RTD_SetShutdownMode(uint8_t RTD_dev, uint8_t shutdown)
{
	if (shutdown) shutdown = 1; //force value of 1 for logical true

	//if chip is already in the desired mode, do nothing
	if (!(shutdown ^ (configVal & 0b00000001))) return HAL_OK;
	else
	{
		HAL_StatusTypeDef retstat;
		uint8_t msg;

		//set address pointer register to config register
		retstat = RTD_SetAddrPtrReg(RTD_dev, REG_CONFIG);
		RTD_CHK_HAL(retstat, end);

		//prepare byte to be written to the config register such that previous settings are retained
		if (shutdown) msg = configVal | shutdown;
		else msg = configVal & shutdown;

		//write to the config register to update the settings
		retstat = Periph_Tx(RTD_dev, &msg, 1, RTD_I2C_TO);
		RTD_CHK_HAL(retstat, end);

		configVal = msg; //update internal copy of config reg
		return retstat;
	}
}
*/