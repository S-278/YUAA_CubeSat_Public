/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ADCS_Meas.c
* @brief ADCS Measurement Task C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2023.06.22
*
* @details           Defines ADCS measurement task
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "ADCS_Meas.h"
#include "ADCS.h"
#include "Detumbler.h"
#include "PeriphHelper.h"
#include "power_dist.h"
#include "SS.h"
#include "system_manager.h"
#include "TimeUtils.h"
#include "VecUtils.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "event_groups.h"
#include "task.h"
#include "panels.h"
#include "User_types.h"
#include "arm_math.h"
#include "LIS3MDL_MAG_driver.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>
#include <time.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Shortest time interval between measurements in ms.
*  Defined to 1/(0.625 Hz) assuming MTM data rate is the bottleneck on measurements.
*  Measurements of B and S must be synchronized for use in attitude calcs. */
#define MEAS_INTV_UNIT (1.0/0.625 * 1000) 
/* Maximum time interval between measurements in ms. 
*  This upper bound is determined roughly as order 10sec, 
*  as above that a measurement pair cannot be reliably used 
*  to compute numeric derivatives for TRIAD. */
#define MEAS_INTV_MAX (MEAS_INTV_UNIT * 10)
_Static_assert(MEAS_INTV_MAX < UINT16_MAX, "Intv_t too small for MEAS_INTV_MAX");
/* Radius of Gaussian bubble created by noise around the true measurement vector.
*  Equal to the standard deviation of the distance from a measured vector
*  to the mean vector averaged over many measurements in a static position. */
#define B_MEAS_BUBBLE_RAD () // TODO
#define S_MEAS_BUBBLE_RAD () // TODO
/* Minimum distance between successive measurements
*  to qualify as true motion and not just noise. */
#define B_MEAS_MIN_DIST (B_MEAS_BUBBLE_RAD * 3) 
#define S_MEAS_MIN_DIST (S_MEAS_BUBBLE_RAD * 3) 
/* Constant sleep duration between measurement pairs 
*  if this task is in passive mode, in ms. */
#define MEAS_PASSIVE_PERIOD (30*60*1000)
/* Time needed for MTQ solenoid coils to relax, in ms */
#define MEAS_MTQ_RELAX_TIME (30)

#define errorcheck_ret(retstat, successVal) if (retstat != successVal) return retstat

#define MTM_addrToDev(MTM_addr) (MTM_addr == LIS3MDL_MAG_I2C_ADDRESS_HIGH ? DEV_MTM_HI : DEV_MTM_LO)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Convention: A MeasPair_t does not actually store meaningful data if the interval is 0.
#define measPairIsValid(measPairPtr) ( ( (MeasPair_t*)measPairPtr ) -> intv > 0 )

// Type used for all data related to a type of measurement (B or S).
typedef struct {
	// A B measurement pair is cached when an attempt to update 
	// the current S measurement fails (and vice versa).
	// This is done to ensure there is always a most recent 
	// synchronized set of measurements available.
	MeasPair_t cachedMeasPair;
	MeasPair_t currMeasPair;
} MeasData_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Address of the MTM to be using. 
// Out of the two redundant MTMs this task chooses one to use.
// `MTM_addr' == 0 means no MTM is available.
static uint8_t MTM_addr = 0;

static MeasData_t S_Meas_B = {
	.currMeasPair = {.intv = 0, .M0_tick = 0},
	.cachedMeasPair = {.intv = 0, .M0_tick = 0},
};
static MeasData_t S_Meas_S = {
	.currMeasPair = {.intv = 0, .M0_tick = 0},
	.cachedMeasPair = {.intv = 0, .M0_tick = 0},
};

/* Every listener has one bit assigned to it in the event group.
Whenever the measurement task publishes a new measurement, it sets the bits of all listeners.
Whenever a listener calls `ADCS_Meas_MarkAsRead', it clears its bit. */
static StaticEventGroup_t EventGroupBuf;
static EventGroupHandle_t EventGrp;
#define listenerBit(listener) (1 << listener)
#define EVENTGRP_SUN_BIT (1 << 24) // Use 24th bit of event group to indicate whether sun sensor currently sees sun
#define IS_SUN (xEventGroupGetBits(EventGrp) & EVENTGRP_SUN_BIT)
#define SET_SUN xEventGroupSetBits(EventGrp, EVENTGRP_SUN_BIT)
#define CLEAR_SUN xEventGroupClearBits(EventGrp, EVENTGRP_SUN_BIT)

static ORBIT_PHASE currPhase = PHASE_UNKNOWN;
// This static flag records whether a successful measurement of the sun vector 
// has happened the last sun phase. 
// This task sets this flag to 1 whenever it gets a successful sun vector measurement,
// and clears it to 0 whenever state trackers says a new sun phase has begun.
static uint8_t S_SunLastLight = 0;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static void staticDataInit()
{
	MTM_addr = 0;

	// `S_Meas_B' and `S_Meas_S' are initialized to invalid at boot.
	// At task start, if there is a valid current measurement, 
	// it should be pushed to the cache.
	// In any case the current measurement is invalidated.
	if (measPairIsValid(&S_Meas_B.currMeasPair))
		memcpy(&S_Meas_B.cachedMeasPair, &S_Meas_B.currMeasPair, sizeof(MeasPair_t));
	S_Meas_B.currMeasPair.intv = 0;
	if (measPairIsValid(&S_Meas_S.currMeasPair))
		memcpy(&S_Meas_S.cachedMeasPair, &S_Meas_S.currMeasPair, sizeof(MeasPair_t));
	S_Meas_S.currMeasPair.intv = 0;

	EventGrp = xEventGroupCreateStatic(&EventGroupBuf);
	xEventGroupClearBits(EventGrp, ~0); // Ensure all bits are cleared

	currPhase = PHASE_UNKNOWN;
	S_SunLastLight = 0;
}

// Copy M1 of target pair into M0, 
// then copy newVec into M1 of target pair.
static void slidePair(MeasPair_t* targetPairPtr, const Vec3D_t* newVecPtr) 
{
	memcpy(&targetPairPtr->M0, &targetPairPtr->M1, sizeof(Vec3D_t));
	memcpy(&targetPairPtr->M1, newVecPtr, sizeof(Vec3D_t));
}

static MeasPair_t* getLatestValid(const MeasData_t* measData)
{
	if (measPairIsValid(&measData->currMeasPair)) return &measData->currMeasPair;
	else if (measPairIsValid(&measData->cachedMeasPair)) return &measData->cachedMeasPair;
	else return NULL;
}

static float getVecRot(const MeasPair_t* meas)
{
	float32_t dotProd, magM0, magM1;
	arm_dot_prod_f32((float32_t *)&meas->M0, (float32_t *)&meas->M1, 3, &dotProd);
	arm_dot_prod_f32((float32_t *)&meas->M0, (float32_t *)&meas->M0, 3, &magM0);
	arm_sqrt_f32(magM0, &magM0);
	arm_dot_prod_f32((float32_t *)&meas->M1, (float32_t *)&meas->M1, 3, &magM0);
	arm_sqrt_f32(magM1, &magM1);
	float cosTheta = dotProd / (magM0 * magM1);
	float theta = acosf(cosTheta);
	return theta / ((float)meas->intv / 1000);
}

// This helper updates the published static variables according to the given measurement data.
// Pass NULL for either pair if no data was recorded - this will invalidate the current measurement.
static void publishMeasurement(
	const MeasPair_t* pairB, 
	const MeasPair_t* pairS)
{
	ORBIT_PHASE newPhase = PowerDist_getPhase(); // Will be used later to update SunLastLight, should be before SharedMem access

	// This prevents another task from trying to read the published measurements
	// while this task is in the middle of updating them.
	SharedMem_BeginAccess();

	uint8_t cacheIsSynced = measPairIsValid(&S_Meas_B.cachedMeasPair) && measPairIsValid(&S_Meas_S.cachedMeasPair);
	uint8_t currIsSynced = measPairIsValid(&S_Meas_B.currMeasPair) && measPairIsValid(&S_Meas_S.currMeasPair);
	uint8_t newIsSynced = pairB != NULL && pairS != NULL;

	if (newIsSynced)
	// If the new measurements are synced, we just take them and overwrite the current measurements.
	{
		memcpy(&S_Meas_B.currMeasPair, pairB, sizeof(MeasPair_t));
		memcpy(&S_Meas_S.currMeasPair, pairS, sizeof(MeasPair_t));
	}
	else if (currIsSynced)
	// If the new measurements are not synced but the current ones are,
	// we make sure to save the current measurements in the cache,
	// and then we overwrite the current with the new.
	{
		memcpy(&S_Meas_B.cachedMeasPair, &S_Meas_B.currMeasPair, sizeof(MeasPair_t));
		memcpy(&S_Meas_S.cachedMeasPair, &S_Meas_S.currMeasPair, sizeof(MeasPair_t));

		if (pairB != NULL) memcpy(&S_Meas_B.currMeasPair, pairB, sizeof(MeasPair_t));
		else S_Meas_B.currMeasPair.intv = 0;

		if (pairS != NULL) memcpy(&S_Meas_S.currMeasPair, pairS, sizeof(MeasPair_t));
		else S_Meas_S.currMeasPair.intv = 0;
	}
	else
	// If neither the new nor the current measurements are synced, 
	// but the cache IS synced, we do not want to touch the cache! Just overwrite the current with the new.
	// If the cache is also not synced, and the new measurement is missing,
	// but the current measurement is valid, 
	// then we can save the current into the cache. 
	{
		if (pairB != NULL) memcpy(&S_Meas_B.currMeasPair, pairB, sizeof(MeasPair_t));
		else
		{
			if (!cacheIsSynced && measPairIsValid(&S_Meas_B.currMeasPair)) 
				memcpy(&S_Meas_B.cachedMeasPair, &S_Meas_B.currMeasPair, sizeof(MeasPair_t));

			S_Meas_B.currMeasPair.intv = 0;
		}
		if (pairS != NULL) memcpy(&S_Meas_S.currMeasPair, pairS, sizeof(MeasPair_t));
		else
		{
			if (!cacheIsSynced && measPairIsValid(&S_Meas_S.currMeasPair)) 
				memcpy(&S_Meas_S.cachedMeasPair, &S_Meas_S.currMeasPair, sizeof(MeasPair_t));

			S_Meas_S.currMeasPair.intv = 0;
		}
	}

	uint8_t gotSun = pairS != NULL;
	// At the start of a new sun phase, SunLastLight gets reset (unless we got a sun measurement immediately).
	if (newPhase == PHASE_SUN && currPhase != PHASE_SUN) S_SunLastLight = gotSun;
	// At any other time, SunLastLight gets set if we got sun right now.
	else if (gotSun) S_SunLastLight = 1;
	currPhase = newPhase;

	SharedMem_EndAccess();

	// Update sun bit
	// Do this before broadcasting a new measurement,
	// so that tasks reading the new measurements also already have the correct sun bit value
	if (gotSun) SET_SUN;
	else CLEAR_SUN;

	// Set all bits in event group except sun bit
	// to unblock tasks waiting for their bits to be set
	xEventGroupSetBits(EventGrp, ~0 - EVENTGRP_SUN_BIT);
}

/* Reads the magnitometer measurement by directly reading the registers containing
the measurement, and normalizing the measurement by the gain given in the datasheet
to produce a value in gauss. */
static status_t MTM_GetB(uint8_t addr, Vec3D_t* data)
{
	int16_t rawRegVals[6];
#ifdef DEBUG_ENABLED
	for (uint8_t i = 0; i < 6; i++) rawRegVals[i] = 0;
#endif // DEBUG_ENABLED
	int16_t fullVals[3];

	//Read the 8-bit MTM registers into 16-bit ints, and bitshift
	//the odd-numbered registers to the left since they contain the
	//MSB of the 16-bit measurement value
	status_t retstat;
	for (char i = 0; i < 6; i++)
	{
		retstat = LIS3MDL_MAG_ReadReg(addr, 0x28 + i, &(rawRegVals[i]));
		if (retstat != SEN_SUCCESS) return retstat;
		if (i % 2 != 0) rawRegVals[i] = rawRegVals[i] << 8;
	}
	//Build the full 16-bit values by adding the LSB and the MSB
	for (char i = 0; i < 3; i++) fullVals[i] = rawRegVals[i * 2] | rawRegVals[i * 2 + 1];

	//Normalize values into units of gauss by dividing by the gain
	for (int i = 0; i < 3; i++) data->Vec[i] = fullVals[i] / (float)GN;
	return retstat;
}

inline void convertBToBodyFrame(Vec3D_t* B)
{
	// TODO: do we need to rotate B at all to get it into the body frame?
	// In what orientation are the MTMs mounted on the OBC?
}

inline void convertSToBodyFrame(Vec3D_t* S)
{
	// TODO
}

static status_t completeBMeasurement(Vec3D_t* result)
{
	assert(MTM_addr);
	if (MTM_addr == 0) return SEN_ERROR;
	Stop_Magnetorquers();
	vTaskDelay(pdMS_TO_TICKS(MEAS_MTQ_RELAX_TIME));
	status_t MTM_retstat;
	Sys_TryDev(MTM_retstat, MTM_GetB(MTM_addr, result), SEN_SUCCESS, MTM_addrToDev(MTM_addr));
	Detumbling_X_RestoreOutput();
	errorcheck_ret(MTM_retstat, SEN_SUCCESS);
	convertBToBodyFrame(result); 
	return MTM_retstat;
}

static status_t completeSMeasurement(Vec3D_t* result)
{
	HAL_StatusTypeDef SS_retstat;
	Sys_TryDev(SS_retstat, SS_GetVector_FromADC(result), HAL_OK, DEV_SS);
	if (SS_retstat != HAL_OK) return SEN_ERROR;
	if (result->X == 0 && result->Y == 0 && result->Z == 0) return SEN_ERROR; // Sun not visible
	convertSToBodyFrame(result);
	return SEN_SUCCESS;
}

static int minDistanceReached(const Vec3D_t* M0, const Vec3D_t* M1, float minDistance)
{
	Vec3D_t M_delta; float32_t M_delta_norm;
	arm_sub_f32(M1, M0, &M_delta, 3);
	arm_dot_prod_f32(&M_delta, &M_delta, 3, &M_delta_norm);
	arm_sqrt_f32(M_delta_norm, &M_delta_norm);
	return M_delta_norm > minDistance;
}

static uint8_t pickMTM()
{
	if (Sys_IsDevAvail(DEV_MTM_HI)) MTM_addr = LIS3MDL_MAG_I2C_ADDRESS_HIGH;
	else if (Sys_IsDevAvail(DEV_MTM_LO)) MTM_addr = LIS3MDL_MAG_I2C_ADDRESS_LOW;
	else MTM_addr = 0;
	return MTM_addr;
}

// Analogous to Sys_TryDev for reading out the MTM:
// tries to complete a B measurement while there is at least one available MTM.
// The final status_t is stored in retstat_var
// and the measurement result, if any, in Vec3D_t_ptr.
#define TryMTM(retstat_var, Vec3D_t_ptr) {				\
while (pickMTM() != 0)									\
{														\
	retstat_var = completeBMeasurement(Vec3D_t_ptr);	\
	if (retstat_var == SEN_SUCCESS) break;				\
}														\
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_GetB(MeasPair_t* meas)
{
	MeasPair_t* source = getLatestValid(&S_Meas_B);
	if (source == NULL) return E_PENDING;

	memcpy(meas, source, sizeof(MeasPair_t)); 
	return E_OK;
}

ES_ReturnType ADCS_Meas_GetBdot(Vec3D_t* Bdot, TickType_t* meas_tick)
{
	MeasPair_t* source = getLatestValid(&S_Meas_B);
	if (source == NULL) return E_PENDING;
	if (source->intv == INTV_INVAL) return E_FILTERING;
	arm_sub_f32(&source->M1, &source->M0, Bdot, 3);
	arm_scale_f32(Bdot, 1000.0 / source->intv, Bdot, 3);
	if (meas_tick != NULL) *meas_tick = source->M0_tick;
	return E_OK;
}

ES_ReturnType ADCS_Meas_GetMagBdot(float* result, TickType_t* meas_tick)
{
	Vec3D_t Bdot;
	ES_ReturnType Bdot_retstat = ADCS_Meas_GetBdot(&Bdot, meas_tick); if (Bdot_retstat != E_OK) return Bdot_retstat;
	float32_t Bdot_mag;
	arm_dot_prod_f32(&Bdot, &Bdot, 3, &Bdot_mag);
	arm_sqrt_f32(Bdot_mag, &Bdot_mag);
	*result = Bdot_mag; return E_OK;
}

ES_ReturnType ADCS_Meas_GetRotB(float* result, TickType_t* meas_tick)
{
	MeasPair_t* source = getLatestValid(&S_Meas_B);
	if (source == NULL) return E_PENDING;
	if (source->intv == INTV_INVAL) return E_FILTERING;

	*result = getVecRot(source);
	if (meas_tick != NULL) *meas_tick = source->M0_tick;
	return E_OK;
}

ES_ReturnType ADCS_Meas_GetS(MeasPair_t* meas)
{
	MeasPair_t* source = getLatestValid(&S_Meas_S);
	if (source == NULL) return E_PENDING;

	memcpy(meas, source, sizeof(MeasPair_t));
	return E_OK;
}

ES_ReturnType ADCS_Meas_GetRotS(float *result, TickType_t *meas_tick)
{
	MeasPair_t *source = getLatestValid(&S_Meas_S);
	if (source == NULL) return E_PENDING;
	if (source->intv == INTV_INVAL) return E_FILTERING;

	*result = getVecRot(source);
	if (meas_tick != NULL) *meas_tick = source->M0_tick;
	return E_OK;
}

ES_ReturnType ADCS_Meas_GetLastMeas(MeasPair_t *meas_B, MeasPair_t *meas_S)
{
	MeasPair_t *B_source, *S_source;
	if (measPairIsValid(&S_Meas_B.currMeasPair) && measPairIsValid(&S_Meas_S.currMeasPair))
	{
		B_source = &S_Meas_B.currMeasPair; S_source = &S_Meas_S.currMeasPair;
	}
	else if (measPairIsValid(&S_Meas_B.cachedMeasPair) && measPairIsValid(&S_Meas_S.cachedMeasPair))
	{
		B_source = &S_Meas_B.cachedMeasPair; S_source = &S_Meas_S.cachedMeasPair;
	}
	else return E_PENDING;
	
	memcpy(meas_B, B_source, sizeof(MeasPair_t));
	memcpy(meas_S, S_source, sizeof(MeasPair_t));
	return E_OK;
}

uint8_t ADCS_Meas_GetSunLastLight()
{
	return S_SunLastLight;
}

ES_ReturnType ADCS_Meas_WaitForUpdate(ADCS_Meas_Listener_e listener, uint32_t timeout_ms)
{
	ADCS_Meas_MarkAsRead(listener);
	EventBits_t eventMask = listenerBit(listener);
	EventBits_t retval = xEventGroupWaitBits(EventGrp, eventMask, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
	if (retval & (eventMask)) return E_OK;
	else return E_PENDING;
}

ES_ReturnType ADCS_Meas_WaitForSun(uint32_t timeout_ms)
{
	if (xEventGroupWaitBits(EventGrp, EVENTGRP_SUN_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms)) \
		& EVENTGRP_SUN_BIT) return E_OK;
	else return E_PENDING;
}

void ADCS_Meas_MarkAsRead(ADCS_Meas_Listener_e listener)
{
	xEventGroupClearBits(EventGrp, listenerBit(listener));
}

int ADCS_Meas_IsUnread(ADCS_Meas_Listener_e listener)
{
	return xEventGroupGetBits(EventGrp) & listenerBit(listener);
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ADCS_Meas_X_Task(void* arg)
{
	ADCS_MeasMode_e MODE = (ADCS_MeasMode_e)arg;

	staticDataInit();

	pickMTM();

	MeasPair_t pairB = { .intv = 0 }, pairS = { .intv = 0 };
	status_t S_status = SEN_ERROR, B_status = SEN_ERROR; 

	TickType_t refTick = xTaskGetTickCount();
	TickType_t currTick;

	// Intransitive: we publish measurements exactly once per iteration of this loop
	// Active mode general idea:
	// 1. Make sure M1 is ready because that's the one we're going to measure minimum distance from.
	// 2. Keep sampling both S and B until both have cleared the minimum distance relative to M1. 
	// 3. Slide the measurement pairs (M1 -> M0, last sample -> M1), record interval, record tick, publish.
	// Passive mode general idea:
	// 1. Take M0. 
	// 2. Keep sampling both S and B until both have cleared the minimum distance relative to M0.
	// 3. Last sample -> M1, record interval, record tick, publish. 
	while (1)
	{
		// If in active mode, and we have measurement errors (or the measurement pairs are just not initialized),
		// then we need to take one measurement now to prepare M1.
		if (MODE == ADCS_MEAS_ACTIVE)
		{
			if (B_status != SEN_SUCCESS)
			{
				TryMTM(B_status, &pairB.M1);
			}
			if (S_status != SEN_SUCCESS)
			{
				S_status = completeSMeasurement(&(pairS.M1));
			}
		}

		// Else we are in passive mode, so do the first measurument of the pair now
		else
		{
			TryMTM(B_status, &pairB.M0);
			S_status = completeSMeasurement(&(pairS.M0));
		}


		// Adaptive measurement interval loop:
		// Sample both sensors at intervals of `MEAS_INTV_UNIT',
		// and break when for both B and S the minimum distance between last measurement and M1 is reached.
		// (on SEN_ERROR the measurement pair is invalidated)
		int B_minDistanceReached = 0, S_minDistanceReached = 0;
		Vec3D_t B_sample, S_sample;
		for (refTick = xTaskGetTickCount();
			currTick - refTick < pdMS_TO_TICKS(MEAS_INTV_MAX) && (!B_minDistanceReached || !S_minDistanceReached);
			currTick = xTaskGetTickCount()
		)
		{
			vTaskDelay(pdMS_TO_TICKS(MEAS_INTV_UNIT));
			// If right now B_status or S_status are not SEN_SUCCESS,
			// there's an error carried over from the start of the loop 
			// which we're going to deal with at the start of the next iteration,
			// and for this iteration we give up on the measurement pair.
			if (B_status == SEN_SUCCESS)
			{
				TryMTM(B_status, &B_sample);
				if (B_status == SEN_SUCCESS) B_minDistanceReached = minDistanceReached(&pairB.M1, B_sample, B_MEAS_MIN_DIST);
				else {pairB.intv = 0; B_minDistanceReached = 1;} // Measurement error - give up on the measurement pair for this iteration
			}
			if (S_status == SEN_SUCCESS)
			{
				S_status = completeSMeasurement(&S_sample);
				if (S_status == SEN_SUCCESS) S_minDistanceReached = minDistanceReached(&pairS.M1, S_sample, S_MEAS_MIN_DIST);
				else { pairS.intv = 0; S_minDistanceReached = 1; }
			}
			
		} 

		// Record measurement interval once we are out of the adaptive loop
		MeasIntv_t measIntv = pdTICKS_TO_MS(currTick - refTick);
		
		// Prepare pairB and pairS for publishing.
		// If the min distance was not reached before the loop exited,
		// the interval is set to "invalid" to indicate that no derivatives/deltas should be performed.
		if (B_status == SEN_SUCCESS)
		{
			if (MODE == ADCS_MEAS_ACTIVE) slidePair(&pairB, &B_sample);
			else pairB.M1 = B_sample;
			pairB.intv = B_minDistanceReached ? measIntv : INTV_INVAL;
			pairB.M0_tick = refTick;
		}
		if (S_status == SEN_SUCCESS)
		{
			if (MODE == ADCS_MEAS_ACTIVE) slidePair(&pairS, &S_sample);
			else pairS.M1 = S_sample;
			pairS.intv = S_minDistanceReached ? measIntv : INTV_INVAL;
			pairS.M0_tick = refTick;
		}

		// Publish
		publishMeasurement(B_status == SEN_SUCCESS ? &pairB : NULL,
						   S_status == SEN_SUCCESS ? &pairS : NULL);

		if (MODE == ADCS_MEAS_PASSIVE) vTaskDelay(pdMS_TO_TICKS(MEAS_PASSIVE_PERIOD));
	}
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TESTS DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DEBUG_ENABLED
TestStatus_e ADCS_Meas_TestFixture_Reset()
{
	staticDataInit();
	return TEST_PASS;
}

void ADCS_Meas_DebugPublish(const MeasPair_t *pairB, const MeasPair_t *pairS)
{
	publishMeasurement(pairB, pairS);
}

#endif