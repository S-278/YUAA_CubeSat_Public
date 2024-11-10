/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file SS.c
* @brief Sun Sensor Driver C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elena W., Nich S., Elijah B.
* @version           0.1.0
* @date              2022.08.14
*
* @details           Defines driver for Sun Sensor
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "SS.h"
#include "system_manager.h"
#include "PeriphHelper.h"
#include "FreeRTOS.h"
#include "task.h"
#include "User_types.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef DEBUG_ENABLED
#include <stdio.h>
#endif
#include "integer.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define SS_HAL_TIMEOUT 500 // Time in ms after which a blocking call to a HAL SPI function must return
#define SS_EXEC_TIMEOUT 1000 // Time in ms given to the SS to return a reply to a command
//Number of times a command will be retried following a CRC error or Sun Sensor Busy message
//before giving up
#define SS_MAX_CMD_ATTEMPTS 8 
#define START_BYTE 0x55
#define BUFSZ 64 // This MUST be larger than the largest expected SS reply size
#define CRC_INIT 0xFFFFFFFF
#define errorcheck(x, lbl) if (x.HAL_ret != HAL_OK) goto lbl

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static uint8_t TxBuf[BUFSZ];
static uint8_t RxBuf[BUFSZ];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// CRC-32/MPEG-2 CRC calculator for the sun sensor
// Source: https://community.st.com/s/question/0D50X00009XkbNMSAZ/crc-computation
static DWORD Crc32(DWORD Crc, DWORD Data)
{
    int i;

    Crc = Crc ^ Data;
    for (i = 0; i < 32; i++)
        if (Crc & 0x80000000)
            Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
        else
            Crc = (Crc << 1);

    return (Crc);
}
static DWORD Crc32Block(DWORD Crc, DWORD Size, DWORD* Buffer) // 32-bit units
{
    while (Size--)
        Crc = Crc32(Crc, *Buffer++);

    return(Crc);
}

static HAL_StatusTypeDef evaluateCmdRet(uint8_t cmd, CmdRet_t retstat)
{
	if (retstat.HAL_ret != HAL_OK) return retstat.HAL_ret;
	// Debug ensure we don't insert any wrong command numbers in code
	assert(retstat.SS_ret != SS_INV_CMD);
	// Flight: raise a logic error and propagate error so that caller
	// raises a device error. This can be both due to OBC code corruption
	// and due to comms channel malfunction or SS malfunction/bug.
	if (retstat.SS_ret == SS_INV_CMD)
	{
		Sys_RaiseLogicErrorMsg(LERR_FATAL, "%hhu", cmd);
		return HAL_ERROR;
	}
	// Debug: the only command that changes settings checks for this separately itself,
	// in all other cases we didn't program something correctly.
	assert(retstat.SS_ret != SS_SETTING_LOCK);
	if (retstat.SS_ret == SS_OK) return HAL_OK;
	// All other SS errors are to be interpreted as
	// either comms channel malfunction or SS malfunction/bug.
	// Propagate error so that caller raises device error.
	else return HAL_ERROR;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

CmdRet_t ExecCmd(uint8_t cmd, uint8_t* param, uint8_t* answer, uint8_t paramLen, uint8_t answerLen, const uint8_t* cmd_CRC)
{
	CmdRet_t ret;
	assert(paramLen == 0 || paramLen == 2);

	uint8_t requestPayloadSize = 1 + paramLen; //Command byte + any parameters
	uint8_t TxSize = 1 + 1 + requestPayloadSize + 4; //Start byte + payload size + payload + 4-byte CRC

	uint8_t answerPayloadSize = 1 + answerLen; //Status byte + any answer parameters
	uint8_t RxSize = 1 + 1 + answerPayloadSize + 4; ////Start byte + payload size + payload + 4-byte CRC
	assert(RxSize < BUFSZ);
	if (!(RxSize < BUFSZ)) RxSize = BUFSZ - 1;

	Periph_BeginTransact(DEV_SS, TxSize + RxSize, HAL_MAX_DELAY);

#ifdef DEBUG_ENABLED
	memset(TxBuf, 0, sizeof(TxBuf));
#endif // DEBUG_ENABLED

	TxBuf[0] = START_BYTE;
	TxBuf[1] = requestPayloadSize;
	TxBuf[2] = cmd;
	memcpy(&(TxBuf[3]), param, paramLen);

	// If precalculated CRC value provided, append it
	if (cmd_CRC != NULL) memcpy(TxBuf + 3 + paramLen, cmd_CRC, 4);
	// Else calculate CRC
	else
	{
		//If transmit frame length is not a multiple of 4 bytes, it must be right-padded
		uint8_t TxPadding = (4 - ((1 + 1 + requestPayloadSize) % 4)) % 4;
		memset(TxBuf+3 + paramLen, 0, TxPadding);

		uint32_t crc = Crc32Block(CRC_INIT, (1 + 1 + requestPayloadSize + TxPadding) / 4, (DWORD*)TxBuf);

		//Save 4-byte CRC into TxBuf, least significant byte first, erasing the padding
		for (uint8_t i = 0; i < 4; i++) TxBuf[3 + paramLen + i] = (uint8_t)((crc >> (i * 8)) % 0x100);
	}

	/* Begin exchange */
	uint8_t actualAnswerPayloadSize; uint8_t* RxBegin;

	for (uint8_t attempt = 1; attempt <= SS_MAX_CMD_ATTEMPTS; attempt++)
	{
		TickType_t start_t = xTaskGetTickCount();
#ifdef DEBUG_ENABLED
		memset(RxBuf, 0, BUFSZ);
#endif
		//Transmit command
		ret.HAL_ret = Periph_Tx(DEV_SS, TxBuf, TxSize, SS_HAL_TIMEOUT);
		errorcheck(ret, end);
		//Receive reply
		/* Rx strategy: 
		- Receive one buffer's worth of data
		- Check if a start byte was received
			- If yes:
				- If RxSize fits in buffer entirely: break 
				- Else copy the message start to start of buffer, then receive enough trailing bytes 
				to fill the buffer afterwards and break
			- Else repeat until timeout 
		*/
		do
		{
			ret.HAL_ret = Periph_Rx(DEV_SS, RxBuf, BUFSZ, SS_HAL_TIMEOUT);
			errorcheck(ret, end);
#if defined(DEBUG_ENABLED) && defined(DEBUG_L3) // Debug print data received from SS
			uint8_t ptrStart = 0;
			for (; RxBuf[ptrStart] == 0 && ptrStart < BUFSZ; ptrStart++); // Ignore leading zeroes
			uint8_t ptrEnd = BUFSZ - 1;
			for (; RxBuf[ptrEnd] == 0 && ptrEnd > ptrStart; ptrEnd--); // Ignore trailing zeroes
			// Print just the stuff in between
			if (ptrEnd > ptrStart)
			{
				fprintf(DEBUG_COMM, "SS >> ");
				for (uint8_t i = ptrStart; i <= ptrEnd; i++) { unsigned int tmp = RxBuf[i]; fprintf(DEBUG_COMM, "%.2X ", tmp); }
				fprintf(DEBUG_COMM, "\n");
			}
#endif

			RxBegin = memchr(RxBuf, START_BYTE, BUFSZ);
			if (RxBegin != NULL)
			{
				uint8_t receivedSize = RxBuf + BUFSZ - RxBegin;
				if (receivedSize >= RxSize) break;
				else
				{
					memmove(RxBuf, RxBegin, receivedSize); RxBegin = RxBuf;
					ret.HAL_ret = Periph_Rx(DEV_SS, RxBuf + receivedSize, BUFSZ - receivedSize, SS_HAL_TIMEOUT);
					errorcheck(ret, end);
					break;
				}
			}
#ifdef DEBUG_ENABLED
			else memset(RxBuf, 0, BUFSZ);
#endif
		} while (xTaskGetTickCount() - start_t < pdMS_TO_TICKS(SS_EXEC_TIMEOUT));

		//Check for successful reception
		if (RxBegin != NULL)
		{
			//First find size of answer frame (could be smaller than expected due to an error)
			actualAnswerPayloadSize = RxBegin[1];
			//Get sun sensor's CRC
			uint32_t SS_CRC = 0;
			for (uint8_t i = 0; i < 4; i++) SS_CRC += RxBegin[1 + 1 + actualAnswerPayloadSize + i] << (i * 8);

			//Calculate CRC on received answer (excluding the sun sensor's CRC at the end)
			//If answer frame size is not multiple of 4 bytes, must pad it with 0 on the right
			uint8_t RxPadding = (4 - ((1 + 1 + actualAnswerPayloadSize) % 4)) % 4;
			memset(RxBegin + 1 + 1 + actualAnswerPayloadSize, 0, RxPadding);
			uint32_t myCRC = Crc32Block(CRC_INIT, (1 + 1 + actualAnswerPayloadSize + RxPadding) / 4, (DWORD*)RxBegin);

			//Check CRC of answer
			if (myCRC != SS_CRC) ret.SS_ret = SS_CRC_ERR;
			//If CRC matches, check status byte
			else ret.SS_ret = RxBegin[2];
		}
		else ret.SS_ret = SS_NO_START;

		switch (ret.SS_ret)
		{
		case SS_OK: //Exit loop
			break;
		case SS_CRC_ERR: //CRC of command was wrong
		case SS_PACKET_ERR: //Something else messed up on the comms channel probably. Has never come up in testing so far.
		case SS_PACKET_TO: //Something else messed up on the comms channel probably. Has never come up in testing so far.
			continue; //Repeat loop because error was probably due to random comms channel noise
		case SS_NO_START:
			continue; // Repeat loop because sometimes the start byte gets cut off somehow
		case SS_BUSY:
			HAL_Delay(500); // Busy-wait without relinquishing CPU control
			continue; //Repeat loop because sun sensor was busy
		default:
			break; //Give up on any other error
		}
		break;
	} 

end:

	//Iff command was successful, copy answer to given pointer
	if (ret.SS_ret == SS_OK && answer != NULL)
	{
		//Copy number of bytes equal to min of the real length of answer and given length of answer
		//in case given length of answer was wrong
		uint8_t minSize;
		if (answerLen < actualAnswerPayloadSize - 1) minSize = answerLen;
		else minSize = actualAnswerPayloadSize - 1;
		memcpy(answer, RxBegin+3, minSize);
	}
	Periph_EndTransact(DEV_SS);
	return ret;
}
/* NOT IMPLEMENTED on sun sensor
CmdRet_t SS_GetStatus(SS_Status_t* dataPtr)
{
	// 0x7A4FDCCB
	const static uint8_t crc[4] = {0xcb, 0xdc, 0x4f, 0x7a};

	uint8_t answer = 0;
	CmdRet_t retstat = ExecCmd(0x00, NULL, &answer, 0, 1, crc);
	if (retstat.HAL_ret != HAL_OK || retstat.SS_ret != SS_OK) return retstat;
	dataPtr->sun = answer & 0b00000001;
	dataPtr->error = (answer & 0b00000010) >> 1;
	dataPtr->errorFull = (answer & 0b00000100) >> 2;
	dataPtr->settingLock = (answer & 0b00001000) >> 3;
	uint8_t mode = (answer & 0b00110000) >> 4;
	if (mode == 0b10) mode = SS_DEBUG;
	dataPtr->mode = mode;
	return retstat;
}
*/

HAL_StatusTypeDef SS_GetTemp(int16_t* dataPtr)
{
	// 0x2AFBA59B
	static const uint8_t crc[4] = { 0x9b, 0xa5, 0xfb, 0x2a };

	uint8_t tempArr[2];
	CmdRet_t retstat = ExecCmd(0x70, NULL, tempArr, 0, 2, crc);
	HAL_StatusTypeDef HAL_ret = evaluateCmdRet(0x70, retstat);
	if (HAL_ret == HAL_OK)
	{
		int16_t temp = tempArr[0];
		temp += (int16_t)(tempArr[1] << 8);
		*dataPtr = temp;
	}
	return HAL_ret;
}

HAL_StatusTypeDef SS_GetVector_FromADC(Vec3D_t * data)
{
	// 0x2D991787
	static const uint8_t crc[4] = { 0x87, 0x17, 0x99, 0x2d };

	/* TODO clarify how background subtraction is supposed to work
	static float background;
	static int background_called = 0;
	if (background_called==0)
	{
		background = get_background();
		background_called = 1;
	}
	float get_background()
	{
		uint8_t answer[16];
		CmdRet_t retstat = ExecCmd(0x74, NULL, answer, 0, 16, NULL);
		int16_t ADC_1 = (answer[5] << 8) | answer[4];
		int16_t ADC_2 = (answer[7] << 8) | answer[6];
		int16_t ADC_3 = (answer[9] << 8) | answer[8];
		int16_t ADC_4 = (answer[11] << 8) | answer[10];
		float background_level = (ADC_1 + ADC_2 + ADC_3 + ADC_4) / 4;
		return background_level;
	}
	*/

	uint8_t answer[16];
	CmdRet_t retstat = ExecCmd(0x74, NULL, answer, 0, 16, crc);
	HAL_StatusTypeDef HAL_ret = evaluateCmdRet(0x74, retstat);
	if (HAL_ret == HAL_OK)
	{
		int16_t ADC_1 = (answer[5] << 8) | answer[4];
		int16_t ADC_2 = (answer[7] << 8) | answer[6];
		int16_t ADC_3 = (answer[9] << 8) | answer[8];
		int16_t ADC_4 = (answer[11] << 8) | answer[10];
		/*
		ADC_1 -= background;
		ADC_2 -= background;
		ADC_3 -= background;
		ADC_4 -= background;
		*/

		// TODO determine if sun is visible or not

		// Equations according to SS manual section 1.2
		float x_coord = (5.7 * ((ADC_1 + ADC_3) - (ADC_2 + ADC_4))) / (2 * (ADC_1 + ADC_2 + ADC_3 + ADC_4));
		float y_coord = (5.7 * ((ADC_2 + ADC_3) - (ADC_1 + ADC_4))) / (2 * (ADC_1 + ADC_2 + ADC_3 + ADC_4));

		// x_coord and y_coord contain the planar coordinates
		// of the sun spot on the flat sensor surface.
		// We invert x and y and set z to H (height of pinhole above sensor surface)
		// to get the 3D coordinates of the sun vector 
		// in the sun sensor body reference frame
		
		// See CubeSat wiki for documentation of coordinate system
		// - it is NOT the same as that stated in the SS manual!

		data->X = -1 * x_coord; data->Y = -1 * y_coord; data->Z = 3.3;
	}
	return HAL_ret;
}
/* NOT IMPLEMENTED on sun sensor
CmdRet_t SS_GetVector(SS_Result_t* data)
{
	// 0x28925012
	static const uint8_t crc[4] = { 0x12, 0x50, 0x92, 0x28 };

	uint8_t answer[13];
	// CmdRet_t retstat = ExecCmd(0x73, NULL, answer, 0, 13, crc);
	CmdRet_t retstat = ExecCmd(0x73, NULL, answer, 0, 13, crc);

	if (retstat.HAL_ret != HAL_OK || retstat.SS_ret != SS_OK) return retstat;
	data->sun = answer[0];
	if (answer[0] == SS_SUN_NO) return retstat;

	// For each of the vec components, add each of the 4 bytes,
	// LSB first, bitshifting to the left to preserve value 
	int32_t vecInts[3];
	for (unsigned char i = 0; i < 3; i++)
	{
		for (unsigned char j = 0; j < 4; j++) vecInts[i] += (int32_t)(answer[1 + i * 4 + j] << j * 8);
	}

	for (uint8_t i = 0; i < 3; i++) data->vec.Vec[i] = vecInts[i] * (float32_t)(1e-9);
	return retstat;
}
*/

HAL_StatusTypeDef SS_SetSettingLock(SS_SettingLock_e lock)
{
	// 0x96199F4B
	static const uint8_t crc_80[4] = { 0x4b, 0x9f, 0x19, 0x96 };
	// 0x97C133CC
	static const uint8_t crc_81[4] = { 0xcc, 0x33, 0xc1, 0x97 };

	CmdRet_t retstat;
	uint8_t cmd = lock ? 0x81 : 0x80;
	const uint8_t* crc = lock ? crc_81 : crc_80;
	retstat = ExecCmd(cmd, NULL, NULL, 0, 0, crc);
	return evaluateCmdRet(cmd, retstat);
}

/* NOT IMPLEMENTED on sun sensor
CmdRet_t SS_SetState(SS_State_e state)
{
	// 0x94706AC2
	static const uint8_t crc_83[4] = { 0xc2, 0x6a, 0x70, 0x94 };
	// 0x917B2D57
	static const uint8_t crc_84[4] = { 0x57, 0x2d, 0x7b, 0x91 };
	if (state) return ExecCmd(0x84, NULL, NULL, 0, 0, crc_84);
	else return ExecCmd(0x83, NULL, NULL, 0, 0, crc_83);
}
*/

HAL_StatusTypeDef SS_Reset()
{
	// 0x90A381D0
	static const uint8_t crc[4] = { 0xd0, 0x81, 0xa3, 0x90 };

	CmdRet_t ret = ExecCmd(0x85, NULL, NULL, 0, 0, crc);
	// Sun sensor usually does not respond to a reset command
	// even if it goes through successfully. 
	return ret.HAL_ret;
}

HAL_StatusTypeDef SS_SetMeasInterval(uint16_t interval)
{
	uint8_t param[2];
	param[0] = (uint8_t)(interval % 256);
	param[1] = (uint8_t)(interval >> 8);
	CmdRet_t retstat = ExecCmd(0xA2, param, NULL, 2, 0, NULL);
	if (retstat.HAL_ret != HAL_OK) return retstat.HAL_ret;
	assert(retstat.SS_ret == SS_OK);
	if (retstat.SS_ret == SS_SETTING_LOCK)
	{
		HAL_StatusTypeDef subcmd_retstat = SS_SetSettingLock(SS_SETTING_UNLOCKED);
		if (subcmd_retstat != HAL_OK) return subcmd_retstat;
		retstat = ExecCmd(0xA2, param, NULL, 2, 0, NULL);
	}
	return evaluateCmdRet(0xA2, retstat);
}

/* ~~~~~~~ OLD SS TESTING LIBRARY ~~~~~~~~~~~~~~~ */
// I sincerely hope you'll never need this again ;)

// SS tests begin
//#define SS_TEST 	// Define this to test the SS
//#ifdef SS_TEST
//CmdRet_t SS_ret;
//uint8_t ans[32]; uint32_t print[32];
//SS_ret = ExecCmd(0x00, NULL, ans, 0, 1, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS get status - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//
//SS_ret = ExecCmd(0x01, NULL, ans, 0, 1, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS get lock - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//
//SS_ret = ExecCmd(0x80, NULL, ans, 0, 0, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS unblock - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//
//SS_ret = ExecCmd(0x00, NULL, ans, 0, 1, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS get status - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//
//SS_ret = ExecCmd(0x84, NULL, ans, 0, 0, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS go to run - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//
//SS_ret = ExecCmd(0x00, NULL, ans, 0, 1, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS get status - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//
//SS_ret = ExecCmd(0x71, NULL, ans, 0, 1 + 2 * 2, NULL);
//uint8_to_uint32(ans, print, 32);
//fprintf(COMM, "SS get raw pos - HAL: %d SS - %d, ans: ", SS_ret.HAL_ret, SS_ret.SS_ret);
//for (int i = 0; i < 8; i++) fprintf(COMM, "%X ", print[i]); fprintf(COMM, "\n");
//if (SS_ret.HAL_ret != HAL_OK || (SS_ret.SS_ret != OK && SS_ret.SS_ret != INV_CMD)) exit(1);
//#endif
