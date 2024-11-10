/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TelComm.c
* @brief Telecommand handlers C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author			 Kenan E., Aviv M., John W., Elijah B.
* @version           0.1.0
* @date              2023.07.01
*
* @details           Defines handlers for YUAA telecommands
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "TelComm.h"
#include "compChecks.h"
#include "DAC.h"
#include "Detumbler.h"
#include "GGB.h"
#include "OrbitProp.h"
#include "PeriphHelper.h"
#include "SD_Routines.h"
#include "system_manager.h"
#include "TimeUtils.h"
#include "TRIAD.h"
#include "main.h"
#include "MCU_init.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL CONSTANTS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static const char ERR_INV_ARG_MSG[] = "ERR_INV_ARG";
static const char ERR_FAULT_DEV_PREFIX[] = "ERR_FAULT_";
static const char ERR_RUNTIME_MSG[] = "ERR_RUNTIME";

static const char* const DEV_NAMES[NUM_DEV_NAMES] = {
	[DEV_OBC_MCU] = "OBC_MCU",
	[DEV_MTM_HI] = "MTM_HI",
	[DEV_MTM_LO] = "MTM_LO",
	[DEV_RTC] = "RTC",
	[DEV_SD] = "SD",
	[DEV_HAL_GENERIC] = "HAL",
	[DEV_OBC_GENERIC] = "OBC",

	[DEV_EPS] = "EPS",

	[DEV_UHF_TCV] = "TCV",
	[DEV_UHF_ANT] = "ANT",

	[DEV_MUX] = "MUX",
	[DEV_CTR] = "CTR",
	[DEV_DAC_1] = "DAC_1",
	[DEV_DAC_2] = "DAC_2",
	[DEV_RTD_1] = "RTD_1",
	[DEV_RTD_2] = "RTD_2",

	[DEV_GGB_CTRL] = "GGB_CTRL",

	[DEV_SS] = "SS",

	[DEV_MTQ] = "MTQ",
};

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define errorcheck_goto(val, success_val, fail_lbl) if (val != success_val) goto fail_lbl
#define assert_bool(bool_val) if (bool_val != 0 && bool_val != 1) Sys_RaiseLogicError(__FILE__, __LINE__, "inval bool")

#define EEPROM_begin Periph_BeginTransact(DEV_EEPROM, EEPROM_FRAME_SIZE, HAL_MAX_DELAY)
#define EEPROM_end Periph_EndTransact()

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Prints `+[XXXX]+[YYYY]+[ZZZZ]'.
Returns the number of bytes written into `buf'. */
static int printVec(char* buf, const Vec3D_t * vec)
{
	int offset = 0;
	for (int i = 0; i < 3; i++)
	{
		buf += sprintf(buf + offset, "+");
		memcpy(buf + offset, &(vec.Vec[i]), sizeof(float32_t)); offset += sizeof(float32_t);
	}
	return offset;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0xF_ - System-wide ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

TelComm_Result_e TelComm_X_ReciteBeacon(char* output)
{
	sprintf(output, "OK+%s\r", CompCheck_X_Beacon_Generate());
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_SetPwrTh(char* output, const PwrDistrib_Th_t* newTh)
{
	// TODO
}

#define TEMP_THRESHOLDS_MIN	(500)
#define TEMP_THRESHOLDS_MAX	(500)
TelComm_Result_e TelComm_X_SetTempTh(char* output, const Temperature_Thresholds_t* newTh)
{
	TelComm_Result_e ret = TELCOMM_SUCCESS;
	// Check thresholds make sense
	for (int i = 0; i < TEMP_THRESHOLDS_NUM; i++)
	{
		const T_Threshold_t* th = &newTh->thresholds[i];
		if (!(TEMP_THRESHOLDS_MIN <= th->min_outer && th->min_outer <= TEMP_THRESHOLDS_MAX) ||
			!(TEMP_THRESHOLDS_MIN <= th->min_inner && th->min_inner <= TEMP_THRESHOLDS_MAX) ||
			!(TEMP_THRESHOLDS_MIN <= th->max_inner && th->max_inner <= TEMP_THRESHOLDS_MAX) ||
			!(TEMP_THRESHOLDS_MIN <= th->max_outer && th->max_outer <= TEMP_THRESHOLDS_MAX))
		{
			ret = TELCOMM_ERR_INV_ARG;
			break;
		}
		if (!(th->min_outer <= th->min_inner &&
			th->min_inner < th->max_inner &&
			th->max_inner <= th->max_outer))
		{
			ret = TELCOMM_ERR_INV_ARG;
			break;
		}
	}
	if (ret == TELCOMM_SUCCESS)
	{
		SharedMem_BeginAccess();
		memcpy(&EEPROM_temp.TempThresholds, newTh, sizeof(Temperature_Thresholds_t));
		SharedMem_EndAccess();
		EEPROM_begin;
		EEPROM_Emul_SyncInfo();
		EEPROM_end;
		print_OK_W(output, TELCOMM_CMD_SET_TEMP_TH);
	}
	else if (ret == TELCOMM_ERR_INV_ARG)
	{
		sprintf(output, "%s\r", ERR_INV_ARG_MSG);
	}
	return ret;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x1_ - OBC ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

TelComm_Result_e TelComm_X_SetCPUSpeed(char* output, uint8_t newMHz)
{
	// TODO
}

TelComm_Result_e TelComm_X_GetOBCTime(char* output)
{
	time_t currT;
	Sys_TryDev(*retstat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
	if (*retstat == HAL_OK)
	{
		int offset = sprintf(output, "OK+");
		memcpy((uint8_t*)output + offset, &currT, sizeof(time_t));
		sprintf(output + offset + sizeof(time_t), "\r");
		return TELCOMM_SUCCESS;
	}
	else
	{
		sprintf(output, "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[DEV_RTC]);
		return TELCOMM_ERR_FAULT;
	}
}

TelComm_Result_e TelComm_X_SetOBCTime(char* output, time_t newT)
{
	// Sanity check on new time
	if (newT >= 0)
	{
		Sys_TryDev(*retstat, RTC_X_SetTime(newT), HAL_OK, DEV_RTC);
		if (*retstat == HAL_OK)
		{
			print_OK_W(output, TELCOMM_CMD_OBC_TIME);
			return TELCOMM_SUCCESS;
		}
		else
		{
			sprintf(output "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[DEV_RTC]);
			return TELCOMM_ERR_FAULT;
		}
	}
	else
	{
		sprintf(output, "%s\r", ERR_INV_ARG_MSG);
		return TELCOMM_ERR_INV_ARG;
	}
}

TelComm_Result_e TelComm_X_GetResetCounters(char* output)
{
	int offset = sprintf(output, "OK+");
	memcpy((uint8_t*)output + offset, &EEPROM_temp.ResetFaults.RST_WWD, sizeof(uint32_t) * NumResetReasons);
	sprintf(output + offset + sizeof(uint32_t) * NumResetReasons, "\r");
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_GetLogStatus(char* output)
{
	uint32_t free_KiB;
	Sys_TryDev(*retstat, SD_getFreeSpace(&free_KiB), FR_OK, DEV_SD); errorcheck_goto(*retstat, FR_OK, SD_fail);
	Log_Counts_t logCts;
	Sys_TryDev(*retstat, Log_GetLogCounts(&logCts), FR_OK, DEV_SD); errorcheck_goto(*retstat, FR_OK, SD_fail);
	uint32_t numPassTimes;
	Sys_TryDev(*retstat, Log_X_GetNumPassTimes(&numPassTimes), FR_OK, DEV_SD); errorcheck_goto(*retstat, FR_OK, SD_fail);

	int offset = sprintf(output, "OK+%.8lX", free_KiB);
	for (Log_Type_e logType = LOG_INFO; logType != LOG_TYPES_NUM; logType++)
	{
		offset += sprintf(output + offset, "+%.8lX", logCts[logType]);
	}
	offset += sprintf(output + offset, "+%.8lX\r", numPassTimes);
	return TELCOMM_SUCCESS;

SD_fail:
	sprintf(output, "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[DEV_SD]);
	return TELCOMM_ERR_FAULT;
}

TelComm_Result_e TelComm_X_GetLogsInRange_Init(char* output, TelComm_State_t* const state, \
	LOG_TYPE logType, time_t startT, time_t endT)
{
	// TODO
}

TelComm_Result_e TelComm_X_GetLogsInRange_Next(char* output, TelComm_State_t* const state)
{
	// TODO
}

TelComm_Result_e TelComm_X_ClearLogsInRange(char* output, LOG_TYPE logType, time_t startT, \
	time_t endT)
{
	// TODO
}

TelComm_Result_e TelComm_X_SetPassTimes_Init(char* output, TelComm_State_t* const state, \
	uint8_t numTimes)
{
	// TODO
}

TelComm_Result_e TelComm_X_SetPassTimes_Next(char* output, \
	time_t* passTimes, uint8_t numTimes)
{
	// TODO
}

TelComm_Result_e TelComm_X_SetRadioTimeouts(char* output, const Radio_Timeouts_t* newTimeouts)
{
	SharedMem_BeginAccess();
	memcpy(&EEPROM_temp.radioTOs, newTimeouts, sizeof(Radio_Timeouts_t));
	SharedMem_EndAccess();
	EEPROM_begin;
	EEPROM_Emul_SyncInfo();
	EEPROM_end;
	RadioCtl_X_RecoveryTOUpdateCallback();
	print_OK_W(output, TELCOMM_CMD_SET_RADIO_TIMEOUTS);
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_SetGndCoordinates_Init(char* output, TelComm_State_t* const state, \
	uint8_t numStations)
{
	// TODO
}

TelComm_Result_e TelComm_X_SetGndCoordinates_Next(char* output, TelComm_State_t* const state, \
	double* coordinates, uint8_t numLines)
{
	//TODO
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x3_ - GGB ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

TelComm_Result_e TelComm_X_EnableDisableGGB(char* output, uint8_t enable)
{
	assert_bool(enable);
	ADCS_X_GGB_EnableDisableCallback(enable);
	print_OK_W(output, TELCOMM_CMD_ENABLE_GBB);
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_GetGGBExtension(char* output)
{
	SharedMem_BeginAccess();
	double currExt = EEPROM_temp.GGB_ex_params.extCurrent;
	uint32_t en = EEPROM_temp.GGB_state.enabled;
	SharedMem_EndAccess();

	sprintf(output, "OK+%lu+%05.2f\r", en, currExt);
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_SetGGBExtension(char* output, float overrideExt)
{
	if (overrideExt < 0 || overrideExt > GGB_MAX_EXTENSION)
	{
		sprintf(output, "%s\r", ERR_INV_ARG_MSG);
		*retstat = SEN_ERROR;
		return TELCOMM_ERR_INV_ARG;
	}
	DevStat_e overrideSuccess = Sys_X_Start_GGB_Override(overrideExt);
	if (overrideSuccess == DEV_STAT_NORMAL)
	{
		print_OK_W(output, TELCOMM_CMD_GBB_EXT);
		*retstat = SEN_SUCCESS;
		return TELCOMM_SUCCESS;
	}
	else
	{
		sprintf(output, "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[DEV_GGB_CTRL]);
		*retstat = SEN_ERROR;
		return TELCOMM_ERR_FAULT;
	}
}

TelComm_Result_e TelComm_X_SetGGBExtensionTarget(char* output, float target)
{
	if (target < 0 || target > GGB_MAX_EXTENSION)
	{
		sprintf(output, "%s\r", ERR_INV_ARG_MSG);
		*retstat = SEN_ERROR;
		return TELCOMM_ERR_INV_ARG;
	}
	ADCS_X_GGB_SetExtTarget(target);
	print_OK_W(output, TELCOMM_CMD_SET_GBB_EXT_TARG);
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_SetGGBSpeed(char* output, uint8_t newSpeed)
{
	ADCS_X_GGB_SetExtSpeed(newSpeed);
	print_OK_W(output, TELCOMM_CMD_SET_GBB_SPEED);
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_SetGGBSunReq(char* output, uint8_t enableReq)
{
	ADCS_X_GGB_SetSunReq(enableReq);
	print_OK_W(output, TELCOMM_CMD_SET_GBB_SUN_REQ);
	return TELCOMM_SUCCESS;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x4_ - ADCS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

TelComm_Result_e TelComm_X_EnableDisableDetumbling(char* output, uint8_t enable)
{
	assert_bool(enable);
	ADCS_X_Detumb_EnableDisableCallback(enable);
	print_OK_W(output, TELCOMM_CMD_SET_DETUMBLING);
	return TELCOMM_SUCCESS;
}

TelComm_Result_e TelComm_X_SetMTQPolarity(char* output, uint8_t invert)
{
	assert_bool(invert);
	ADCS_X_MTQPolarityCallback(invert ? MTQ_POLARITY_INVERT : MTQ_POLARITY_NORMAL);
	print_OK_W(output, TELCOMM_CMD_SET_MTQ_POL);
	return TELCOMM_SUCCESS;
}


TelComm_Result_e TelComm_X_SetADCSTh(char* output, const ADCS_Thresholds_t* newTh)
{
	TelComm_Result_e retstat = TELCOMM_SUCCESS;
	for (const ADCS_Threshold_t* th = &(newTh->crit); th <= &(newTh->detumbResume); th++)
	{
		if (th->B < 0 || th->O < 0) { retstat = TELCOMM_ERR_INV_ARG; break; }
	}
	if (retstat == TELCOMM_SUCCESS)
	{
		ADCS_X_SetThresholdsCallback(newTh);
		print_OK_W(output, TELCOMM_CMD_SET_ADCS_LIM);
	}
	else if (retstat == TELCOMM_ERR_INV_ARG)
	{
		sprintf(output, "%s\r", ERR_INV_ARG_MSG);
	}
	return retstat;
}

TelComm_Result_e TelComm_X_SetTLE_Init(char* output, TelComm_State_t* const state)
{
	// TODO
}

TelComm_Result_e TelComm_X_SetTLE_Next(char* output, TelComm_State_t* const state)
{
	// TODO
}

TelComm_Result_e TelComm_X_GetAttMat_Init(char* output, TelComm_State_t* const state)
{
	// TODO
}

TelComm_Result_e TelComm_X_GetAttMat_Next(char* output, TelComm_State_t* const state)
{
	// TODO
}

TelComm_Result_e TelComm_X_GetOmega(char* output, FRESULT * retsxtat)
{
	time_t currTime;
	HAL_StatusTypeDef RTC_stat;
	Sys_TryDev(RTC_stat, RTC_GetTime(&currTime), HAL_OK, DEV_RTC);
	if (RTC_stat != HAL_OK)
	{
		sprintf(output, "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[DEV_RTC]);
		return TELCOMM_ERR_FAULT;
	}

	TRIAD_VecPair_t firstBodyVecs;
	TRIAD_VecPair_t secondBodyVecs;
	uint16_t intv_ms;
	time_t measTime;
	time_t measAge;
	ES_ReturnType meas_retstat = ADCS_Meas_GetLastMeas(
					&(firstBodyVecs.S), &(secondBodyVecs.S),
					&(firstBodyVecs.B), &(secondBodyVecs.B), 
					&intv_ms, &measAge);
	
	if (meas_retstat == E_OK)
	{
		measTime = currTime - measAge;
		Vec3D_t Omega;
		Sys_TryDev(*retstat, TRIAD_GetOmegaVec(&firstBodyVecs, &secondBodyVecs, &measTime, intv_ms, &Omega), FR_OK, DEV_SD);

		if (*retstat == FR_OK)
		{
			int offset = sprintf(output, "OK");
			offset += printVec(output + offset, &Omega);
			offset += sprintf(output, "+");
			memcpy(output + offset, &measTime, sizeof(time_t)); offset += sizeof(time_t);
			sprintf(output + offset, "\r");
			return TELCOMM_SUCCESS;
		}
		else
		{
			sprintf(output, "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[DEV_SD]);
			return TELCOMM_ERR_FAULT;
		}
	}
	else
	{
		sprintf(output, "%s\r", ERR_RUNTIME_MSG);
		return TELCOMM_ERR_RUNTIME;
	}
}

TelComm_Result_e TelComm_X_GetSatEquatorial(char* output)
{
	Vec3D_t result;
	ES_ReturnType exec_status = Orbit_GetSatEquatorial(&result);
	if (exec_status == E_OK)
	{
		int offset = sprintf(output, "OK");
		offset += printVec(buf + offset, &result);
		sprintf(output + offset, "\r");
		*retstat = SEN_SUCCESS;
		return TELCOMM_SUCCESS;
	}
	else
	{
		sprintf(output, "%s\r", ERR_RUNTIME_MSG);
		*retstat = SEN_ERROR;
		return TELCOMM_ERR_RUNTIME;
	}
}

TelComm_Result_e TelComm_X_GetPhase(char* output)
{
	// TODO
	// Call State_getPhase();
}

TelComm_Result_e TelComm_X_GetBdotVec(char* output)
{
	Vec3D_t Bdot;
	ES_ReturnType retstat = ADCS_Meas_GetBdot(&Bdot);
	if (retstat == E_OK)
	{
		int offset = sprintf(output, "OK");
		offset += printVec(output + offset, &Bdot);
		sprintf(output + offset, "\r");
		*retstat = SEN_SUCCESS;
		return TELCOMM_SUCCESS;
	}
	else
	{
		sprintf(output, "%s\r", ERR_RUNTIME_MSG);
		*retstat = SEN_ERROR;
		return TELCOMM_ERR_RUNTIME;
	}
}

TelComm_Result_e TelComm_X_GetBodyVecs(char* output)
{
	Vec3D_t B;
	ES_ReturnType retstat = ADCS_Meas_GetB(&B, NULL, NULL);
	if (retstat == E_OK)
	{
		Vec3D_t S;
		ES_ReturnType retstat_S = ADCS_Meas_GetS(&S, NULL, NULL);

		int offset = sprintf(output, "OK");
		offset += printVec(output + offset, &B);

		if (retstat_S != E_OK) for (int i = 0; i < 3; i++) S.Vec[i] = 0;
		offset += printVec(output + offset, &S);

		sprintf(output + offset, "\r");
		return TELCOMM_SUCCESS;
	}
	else
	{
		sprintf(output, "%s\r", ERR_RUNTIME_MSG);
		return TELCOMM_ERR_RUNTIME;
	}
}

TelComm_Result_e TelComm_X_SetMTQTimedOutput(char* output, uint16_t numSeconds, int8_t MTQ_vec[3])
{
	TelComm_Result_e retstat = TELCOMM_SUCCESS;
	if (numSeconds > 0)
	{
		for (int i = 0; i < 3; i++)
		{
			if (MTQ_vec[i] < -100 || MTQ_vec[i] > 100) { retstat = TELCOMM_ERR_INV_ARG; break; }
		}
	}
	else retstat = TELCOMM_ERR_INV_ARG;

	if (retstat == TELCOMM_SUCCESS)
	{
		ADCS_X_ManualMTQCallback(numSeconds, MTQ_vec);
		print_OK_W(output, TELCOMM_CMD_SET_MTQ_OUT);
	}
	else if (retstat == TELCOMM_ERR_INV_ARG)
	{
		sprintf(output, "%s\r", ERR_INV_ARG_MSG);
	}
	return retstat;
}

TelComm_Result_e TelComm_X_ADCSOverride(char* output)
{
	ADCS_X_CritOverrideCallback();
	print_OK_W(output, TELCOMM_CMD_ADCS_OVERRIDE);
	return TELCOMM_SUCCESS;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0x5_ - CRD ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

TelComm_Result_e TelComm_X_SetPulseBins(char* output, const DAC_InitParams_t* params)
{
	DevName_e currentDAC = DEV_DAC_1;
	for (; currentDAC <= DEV_DAC_2; currentDAC++)
	{
		Sys_TryDev(*retstat, DAC_SetOutput(currentDAC, params->outputs[currentDAC - DEV_DAC_1]), HAL_OK, currentDAC);
		if (*retstat != HAL_OK) break;
	}
	
	if (*retstat == HAL_OK)
	{
		print_OK_W(output, TELCOMM_CMD_SET_PULSE_BINS);
		return TELCOMM_SUCCESS;
	}
	else
	{
		sprintf(output, "%s%s\r", ERR_FAULT_DEV_PREFIX, DEV_NAMES[currentDAC]);
		return TELCOMM_ERR_FAULT;
	}
}