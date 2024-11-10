/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TelCommParser.c
* @brief Telecommand Parser C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author			 Wesley A.
* @version           0.1.0
* @date              2023.06.17
*
* @details           Defines the parser for YUAA telecommands
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "ESTTC.h"
#include "DAT_Outputs.h"
#include "TelComm.h"
#include "radio_controller.h"
#include "es_crc32.h"
#include "main.h"
#include <stdint.h>
#include "TCV.h"
#include "TCV.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* DEFINES AND CONSTANTS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static const char ERR_FORMAT_MSG[] = "ERR_FORMAT";
static const char ERR_CRC_MSG[] = "ERR_CRC";
static const char ERR_RES_LOCKED_MSG[] = "ERR_RES_LOCKED";
static const char ERR_FORBIDDEN_MSG[] = "ERR_FORBIDDEN";
static const char ERR_UNSUPPORTED_MSG[] = "ERR_UNSUPPORTED";
static const uint8_t ACKNOWLEDGE_MSG[] = { 0xA0, 0x12, 0x12, 0x12, 0x12, 0x12, 0x12, '\r' };
#define ACKNOWLEDGE(buf) memcpy(buf, ACKNOWLEDGE_MSG, sizeof(ACKNOWLEDGE_MSG))

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef enum {
	PARSE_NO_LINK,			// Not in Controled Downlink. No saved state.
	PARSE_LINK,				// In Controlled Downlink. Expect start of a new command. No saved state.
	PARSE_LINK_CMD,			// In Controlled Downlink, in the middle of processing a command. Stateful.
} ParseState_e;

// Substates of `PARSE_LINK_CMD'
typedef enum {
	CMD_EXPECT_ACK,			// Expecting an Acknowledge from gnd
	CMD_EXPECT_UPLINK,		// Expecting a data packet from gnd
} LinkCmd_State_e;

// Data structure to hold entire state of TelComm Parser
typedef struct {
	ParseState_e parserState;
	LinkCmd_State_e linkCmdState;
	TelComm_Cmd_e currentCmd;
	uint8_t numAckRetries;
	TelComm_Result_e lastCmdRes;
} ParseState_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static ParseState_t State;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
int TelComm_X_Parse(char* rxline, char* txline, TelComm_State_t* const TelCommState) 
{
	// Explicitly null-terminate Rx buffer to prevent buffer OF
	// TODO after ESTTC.c::GetPhrase implementation is finished,
	// make sure this doesn't mess up the Rx data when it's raw binary data instead of a string.
	rxline[LINE_BUFFER_SIZE - 1] = 0; 
	uint8_t ProcessedPacket = 0;
	switch (State.parserState)
	{
	case PARSE_NO_LINK:
	{
		/* Outside of Controlled Downlink, 
		the only supported commands are the link control commands: 
		Downlink request, Kill, Resume. */

		// Check if the data in rxline is an echo from Kill cmd
		if (...) {
			// Will be receiving answer in rxline, so need to check if it matches the one specified on p.36 of TCV manual
			// Check for callbacks in system_manager.h
		}
		// Check if the data in rxline is an echo from Resume cmd
		else if (...) {
			// Watch out for the address in the response: 22 for kill and 23 for resume
		}
		// Check if the data in rxline is a Downlink request beginning with 0xA0
		else if (...) {
			// TODO: Review this code, certainly wrong
			// char* begin = strstr(rxline, "ES+");
			// if (begin != NULL) {
			// 	uint32_t len = strlen(begin);
			// 	if (len <= LINE_BUFFER_SIZE) {
			// 		strcpy(txline, begin);
			// 		if (txline[4] == UHF_I2C_ADDRESS) {

			// 		}
			// 	}
			// }
			State.parserState = PARSE_LINK;
		}
		break;
	}
	case PARSE_LINK:
	{
		// Check for valid header
		const char* YU_header = strstr(rxline, "YU+");
		if (YU_header == NULL) {
			ProcessedPacket = 0;
			break;
		}
		TelComm_X_ParserReset();

		// Signal radio controller
		RadioCtl_X_ESTTCReceivedCallback();

		// TODO: Check if we need to wait for an acknowledge after error
		State.parserState = PARSE_LINK_CMD;

		// Check CRC
		uint32_t len = strlen(YU_header);
		uint32_t CRC_value_calc = crc32(0, (BYTE *)YU_header, len-ESTTC_CYMBOLS_IN_CRC);
		// TODO: Review implementation to not use HexToBin, since it has no error handler
    	uint32_t CRC_value_rx = ESTTC_ExtractCRC(&YU_header[len-ESTTC_CYMBOLS_IN_CRC+1]);
		if (CRC_value_calc != CRC_value_rx)
		{
			strcpy(txline, ERR_CRC_MSG);
			ProcessedPacket = 1;
			break;
		}

		// Find command number
		uint8_t cmd_type = rxline[3];
		// TODO: Check HexToBin
		TelComm_Cmd_e cmd = HexToBin(rxline[4], rxline[5]);
		State.lastCmdRes = TELCOMM_SUCCESS;
		const char* data = &rxline[6];
		// Parse and call handler
		switch (cmd)
		{
		case TELCOMM_CMD_RECITE_BEACON:
			State.lastCmdRes = TelComm_X_ReciteBeacon(txline);
			break;
		case TELCOMM_CMD_SET_PWR_CEIL_FLOOR:
			// No library handler since noreturn
			// TODO parsing arguments
			// First print the response, then handle command
			print_OK_W(txline, cmd);
			// TODO command handling here
			break;
		case TELCOMM_CMD_SET_PWR_TH:
			PwrDistrib_Th_t nePwrTh; // TODO: Populate this
			State.lastCmdRes = TelComm_X_SetPwrTh(txline, &nePwrTh);
			break;
		case TELCOMM_CMD_SET_TEMP_TH:
			ACKNOWLEDGE(txline);
			//TODO
			break;
		case TELCOMM_CMD_LINK_SPEED:
			// TODO: Error handling
			TCV_RFMode_e mode = HexToBin(rxline[6],rxline[7]);
			TCV_SetRFMode(mode);
			ACKNOWLEDGE(txline);
		case TELCOMM_CMD_SET_OBC_SPEED:
			uint8_t newMHz = HexToBin(rxline[6],rxline[7]);
			State.lastCmdRes = TelComm_X_SetCPUSpeed(txline, newMHz);
			break;
		case TELCOMM_CMD_OBC_TIME:
			if (cmd_type == 'W'){
				time_t time;
				memcpy(&time, &(rxline[6]), sizeof(time_t));
			 	State.lastCmdRes = TelComm_X_SetOBCTime(txline, time);
				// Reset last gnd contact time?
			} else {
				State.lastCmdRes = TelComm_X_GetOBCTime(txline);
			}
			break;
		case TELCOMM_CMD_COUNTERS:
			break;
		case TELCOMM_CMD_GET_LOG_STATUS:
			State.lastCmdRes = TelComm_X_GetLogStatus(txline);
			break;
		case TELCOMM_CMD_LOGS_RANGE:

			break;
		case TELCOMM_CMD_SET_PASS_TIMES:
			uint8_t passes_number;
			sscanf(data, "%2hhx ", &passes_number);
			State.lastCmdRes = TelComm_X_SetPassTimes_Init(txline, TelCommState, passes_number);
			break;
		case TELCOMM_CMD_SET_RADIO_TIMEOUTS:
			uint8_t a, b, c, d;
			sscanf(data, "%2hhx+%2hhx+%2hhx+%2hhx ", &a, &b, &c, &d);
			//Check errors here
			//Check return value for scanf
			//Advance pointer and check where end up in comparison to CRC
			//May use sizeof?
			Radio_Timeouts_t timeouts = {a, b, c, d};
			State.lastCmdRes = TelComm_X_SetRadioTimeouts(txline, &timeouts);
			break;
		case TELCOMM_CMD_SET_GND_COORDS:
			uint8_t coord_numbers;
			sscanf(data, "%2hhx ", &coord_numbers);
			State.lastCmdRes = TelComm_X_SetGndCoordinates_Init(txline, TelCommState, coord_numbers);
			break;
		case TELCOMM_CMD_ENABLE_GBB:
			uint8_t enable_gbb;
			sscanf(data, "%hhx %*s", &enable_gbb);
			State.lastCmdRes = TelComm_X_EnableDisableGGB(txline, enable_gbb);
			break;
		case TELCOMM_CMD_GBB_EXT:
			break;
		case TELCOMM_CMD_SET_GBB_EXT_TARG:
			float target;
			sscanf(data, "%f %*s", &target);
			State.lastCmdRes = TelComm_X_SetGGBExtensionTarget(txline, target);
			break;
		case TELCOMM_CMD_SET_GBB_SPEED:
			uint8_t newSpeed;
			sscanf(data, "%hhx ", &newSpeed);
			State.lastCmdRes = TelComm_X_SetGGBSpeed(txline, newSpeed);
			break;
		case TELCOMM_CMD_SET_GBB_SUN_REQ:
			uint8_t enableReq;
			sscanf(data, "%hhu %s", &enableReq);
			State.lastCmdRes = TelComm_X_SetGGBSunReq(txline, enableReq);
			break;
		case TELCOMM_CMD_SET_DETUMBLING:
			uint8_t enable;
			sscanf(data, "%hhu %*s", enable);
			State.lastCmdRes = TelComm_X_EnableDisableDetumbling(txline, &enable);
			break;
		case TELCOMM_CMD_SET_MTQ_POL:
			uint8_t invert;
			sscanf(data, "%hhu %*s", invert);
			State.lastCmdRes = TelComm_X_SetMTQPolarity(txline, &invert);
			break;
		case TELCOMM_CMD_SET_ADCS_LIM:
			ADCS_Thresholds_t newTh;
			float d, e, f, g, m, n;
			sscanf(data, "+%f+%f+%f+%f+%f+%f %*s", &d, &e, &f, &g, &m, &n);
			newTh.detumbStop.B = d;
			newTh.detumbStop.O = e;
			newTh.detumbResume.B = f;
			newTh.detumbResume.O = g;
			newTh.crit.B = m;
			newTh.crit.O = n;
			State.lastCmdRes = TelComm_X_SetADCSTh(txline, &newTh);
			break;
		case TELCOMM_CMD_SET_SUN_POS:
			// TODO?
			break;
		case TELCOMM_CMD_SET_TLE:
			State.lastCmdRes = TelComm_X_SetTLE_Init(txline, &state);
			break;
		case TELCOMM_CMD_GET_ATT_MATRIX:
			State.lastCmdRes = TelComm_X_GetAttMat_Init(txline, &state);
			break;
		case TELCOMM_CMD_GET_OMEGA:
			State.lastCmdRes = TelComm_X_GetOmega(txline);
			break;
		case TELCOMM_CMD_GET_SAT_EQ:
			State.lastCmdRes = TelComm_X_GetSatEquatorial(txline);
			break;
		case TELCOMM_CMD_GET_PHASE:
			State.lastCmdRes = TelComm_X_GetPhase(txline);
			break;
		case TELCOMM_CMD_GET_BDOT:
			State.lastCmdRes = TelComm_X_GetBdotVec(txline);
			break;
		case TELCOMM_CMD_GET_BODY_VEC:
			State.lastCmdRes = TelComm_X_GetBodyVecs(txline);
			break;
		case TELCOMM_CMD_SET_MTQ_OUT:
			uint16_t numSeconds;
			uint16_t temp_vec[3];
			sscanf(data, "+%4hx;%hx;%hx;%hx %*s", &numSeconds, &temp_vec[0], &temp_vec[1], &temp_vec[2]);
			int8_t MTQ_vec[3] = {(uint8_t) temp_vec[0], (uint8_t) temp_vec[1], (uint8_t) temp_vec[2]};
			State.lastCmdRes = TelComm_X_SetMTQTimedOutput(txline, numSeconds, MTQ_vec);
			break;
		case TELCOMM_CMD_ADCS_OVERRIDE:
			State.lastCmdRes = TelComm_X_ADCSOverride(txline);
			break;
		case TELCOMM_CMD_SET_PULSE_BINS:
			DAC_SetupType dac_1 = {
				(uint8_t) data[0], 
				(uint8_t) data[1], 
				(uint8_t) data[2], 
				(uint8_t) data[3]
			};
			DAC_SetupType dac_2 = {
				(uint8_t) data[4], 
				(uint8_t) data[5], 
				(uint8_t) data[6], 
				(uint8_t) data[7]
			};
			DAC_InitParams_t params;
			params.outputs[0] = dac_1;
			params.outputs[1] = dac_2;
			State.lastCmdRes = TelComm_X_SetPulseBins(txline, &params);
			break;
		default:
			State.lastCmdRes = TELCOMM_ERR_UNSUPPORTED;
			break;
		}
		// May put before parser?
		State.currentCmd = cmd;

		switch (State.lastCmdRes)
		{
		case TELCOMM_ERR_FORMAT:
			strcpy(txline, ERR_FORMAT_MSG);
			break;
		case TELCOMM_ERR_CRC:
			strcpy(txline, ERR_CRC_MSG);
			break;
		case TELCOMM_ERR_RES_LOCKED:
			strcpy(txline, ERR_RES_LOCKED_MSG);
			break;
		case TELCOMM_ERR_FORBIDDEN:
			strcpy(txline, ERR_FORBIDDEN_MSG);
			break;
		case TELCOMM_ERR_UNSUPPORTED:
			strcpy(txline, ERR_UNSUPPORTED_MSG);
		default:
			// This case we either have success or an error dealt by TelComm.c
			break;
		}
		ProcessedPacket = 1;
		break;
	}
	case PARSE_LINK_CMD:
	{
		// If expecting an Ack, check for Ack
		// set state during PARSE_LINK

		// Else, TODO

		// Invoke cmd handler
		switch (State.currentCmd)
		{
		case TELCOMM_CMD_SET_TEMP_TH:

		}
		break;
	}
	}
	//TODO: Need to figure out a way of checking time that the parser was last called
	return ProcessedPacket;
}

void TelComm_X_ParserReset()
{
	State.parserState = PARSE_NO_LINK;
	State.linkCmdState = CMD_EXPECT_ACK;
	State.currentCmd = 0;
	State.numAckRetries = 0;
	State.lastCmdRes = TELCOMM_SUCCESS;
}