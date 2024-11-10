/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file CTR.c
* @brief Counter C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2022.08.22
*
* @details           Defines drivers for the PCF8583 counters
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "CTR.h"
#include "system_manager.h"
#include <math.h>
#include <string.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define CTR_errorcheck(x, lbl) if ((HAL_StatusTypeDef)(x) != HAL_OK) goto lbl

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
//Alarm digit registers, populated by CTR_ALARM_VAL in BCD format (4 bits per decimal digit)
//Calculated once at compile-time
static const uint8_t G_alarmDigits[3] = {
	(((CTR_ALARM_VAL / 10) % 10) << 4)		| ((CTR_ALARM_VAL / 1) % 10), 
	(((CTR_ALARM_VAL / 1000) % 10) << 4)	| ((CTR_ALARM_VAL / 100) % 10), 
	(((CTR_ALARM_VAL / 100000) % 10) << 4)  | ((CTR_ALARM_VAL / 10000) % 10) 
};

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef CTR_InitForScint(CTR_e ctr)
{
	assert(ctr == CTR_EVEN || ctr == CTR_ODD);
	DevName_e CTR_dev;
	if (ctr == CTR_EVEN) CTR_dev = DEV_CTR_0;
	else CTR_dev = DEV_CTR_1;

	Periph_BeginTransact(CTR_dev, 8, HAL_MAX_DELAY);

	HAL_StatusTypeDef retstat;
	uint8_t ctrlMsg[2];
	ctrlMsg[0] = 0x00; ctrlMsg[1] = CTR_CTRL_REG_VAL; //Control register at memory address 0x00
	retstat = Periph_Tx(CTR_dev, ctrlMsg, 2, CTR_I2C_TO);
	CTR_errorcheck(retstat, end);

	//Writing the alarm control register and alarm digits in one operation
	//since counter chip auto increments its address reg
	uint8_t alarmMsg[5];  
	alarmMsg[0] = 0x08; alarmMsg[1] = CTR_ALARM_REG_VAL; //Alarm control register at memory address 0x08
	memcpy(alarmMsg + 2, G_alarmDigits, 3);
	retstat = Periph_Tx(CTR_dev, alarmMsg, 5, CTR_I2C_TO);
	CTR_errorcheck(retstat, end);

	//Enable counting once alarm regs have been set
	ctrlMsg[1] &= 0b01111111;
	retstat = Periph_Tx(CTR_dev, ctrlMsg, 2, CTR_I2C_TO);
	CTR_errorcheck(retstat, end);
end:
	Periph_EndTransact(CTR_dev);
	return retstat;
}

HAL_StatusTypeDef CTR_ReadReset(CTR_e ctr, uint32_t* data, uint8_t* overflow)
{
	assert(ctr == CTR_EVEN || ctr == CTR_ODD);
	DevName_e CTR_dev;
	if (ctr == CTR_EVEN) CTR_dev = DEV_CTR_0;
	else CTR_dev = DEV_CTR_1;

	Periph_BeginTransact(CTR_dev, 16, HAL_MAX_DELAY);

	HAL_StatusTypeDef retstat; uint8_t msg[4]; 

	//Get current control register value
	msg[0] = 0x00; 
	retstat = Periph_Tx(CTR_dev, msg, 1, CTR_I2C_TO); CTR_errorcheck(retstat, end);
	uint8_t ctrlRegVal;
	retstat = Periph_Rx(CTR_dev, &ctrlRegVal, 1, CTR_I2C_TO); CTR_errorcheck(retstat, end);

	//Check overflow and set "hold last count" and "stop counting" flags in control register
	if (overflow != NULL) *overflow = (ctrlRegVal >> 1) & 1;
	msg[0] = 0x00; msg[1] = ctrlRegVal | 0b11000000;
	retstat = Periph_Tx(CTR_dev, msg, 2, CTR_I2C_TO); CTR_errorcheck(retstat, end);

	//Read off the counter digits
	msg[0] = 0x01; for (unsigned char i = 1; i < 4; i++) msg[i] = 0;
	uint8_t ctVal[3];
	retstat = Periph_Tx(CTR_dev, msg, 1, CTR_I2C_TO); CTR_errorcheck(retstat, end);
	retstat = Periph_Rx(CTR_dev, ctVal, 3, CTR_I2C_TO); CTR_errorcheck(retstat, end);

	//Write 0 to the counter digits
	retstat = Periph_Tx(CTR_dev, msg, 4, CTR_I2C_TO); CTR_errorcheck(retstat, end);

	//Clear "hold", "stop", and "alarm" flags in control register
	msg[0] = 0x00; msg[1] = ctrlRegVal & 0b00111101;
	retstat = Periph_Tx(CTR_dev, msg, 2, CTR_I2C_TO); CTR_errorcheck(retstat, end);

end:
	Periph_EndTransact(CTR_dev);
	if (retstat == HAL_OK && data != NULL)
	{
		//Convert ctVal into full value
		uint32_t result = 0;
		for (char i = 0; i < 6; i++)
		{
			char digit;
			if (i % 2 == 0) digit = ctVal[i / 2] % 16; //Digits 0,2,4 are the 4 LSb of their byte
			else digit = ctVal[i / 2] >> 4; //Digits 1,3,5 are the 4 MSb of their byte
			result += digit * (int)pow(10, i);
		}
		*data = result;
	}
	return retstat;
}
