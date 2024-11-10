/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ADCS_Meas.h
* @brief ADCS Measurement Task Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2023.06.22
*
* @details           Declares ADCS measurement synchronization task
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "User_types.h"
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Integer type to hold the interval between measurements in ms.
typedef uint16_t MeasIntv_t;
#define INTV_INVAL (UINT16_MAX)

typedef struct {
	// A MeasPair_t records two measurements separated by a short time interval (~sec).
	Vec3D_t M0;
	Vec3D_t M1;
	// Interval between the two measurements, in ms.
	// If this value equals `INTV_INVAL', 
	// the time interval between M1 and M0 is undefined,
	// so the measurement pair shall not be used for derivatives/deltas.
	MeasIntv_t intv;
	// RTOS tick at time of `M0'
	// Note on tick count overflows - FreeRTOS's tick count is a 32-bit variable,
	// so at 1 tick every ms it will overflow in ~50 days. 
	TickType_t M0_tick;
} MeasPair_t;


// Enum for all tasks that listen for measurement updates
typedef enum {
	ADCS_MEAS_LISTENER_ADCSCTRL = 0,
	ADCS_MEAS_LISTENER_DETUMB,
	ADCS_MEAS_LISTENER_GGB,
} ADCS_Meas_Listener_e;

// Measurement modes for the measurement task
typedef enum {
	// 5 min delay between measurement updates. 
	// To be used when neither active detumb nor GGB are running.
	ADCS_MEAS_PASSIVE,
	// Measurements are continuous. 
	// To be used when either active detumb or GGB are running.
	ADCS_MEAS_ACTIVE,
} ADCS_MeasMode_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Wait until a new ADCS measurement
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]				listener - indicates the task calling this function
* @param[input]				timeout_ms - maximum wait duration in ms
* @return					E_PENDING if a new measurement was not completed before the
*							timeout, E_OK otherwise
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_WaitForUpdate(ADCS_Meas_Listener_e listener, uint32_t timeout_ms);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Wait until sun sensor catches a valid sun measurement
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]				timeout_ms - maximum wait duration in ms
* @return					E_PENDING if sun sensor did not detect the sun before the
*							timeout, E_OK otherwise
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_WaitForSun(uint32_t timeout_ms);


/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Mark the current measurement as read
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]				listener - indicates the task calling this function
* @return					none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ADCS_Meas_MarkAsRead(ADCS_Meas_Listener_e listener);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Check whether the current measurement has been read already
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]				listener - indicates the task calling this function
* @return					1 if the most recent measurement has not been marked as read
*							by this listener yet,
*							0 if it has.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
int ADCS_Meas_IsUnread(ADCS_Meas_Listener_e listener);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get current B measurements
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]			meas - pointer to MeasPair_t to populate with result
* @note						Vectors are in the sat body frame and in units of gauss
* @return					E_OK if measurement was read successfully, 
*							E_PENDING if there is no valid measurement available
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_GetB(MeasPair_t* meas);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get Bdot using current B measurements
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]			Bdot - pointer to Vec3D_t to populate with result
* @param[output]			meas_tick - pointer to TickType_t to populate with system tick
*							at time of first measurement. Pass in NULL if not required.
* @note						Bdot is in units of gauss/sec
* @return					E_OK if measurement was read successfully,
*							E_PENDING if there is no valid measurement available,
*							E_FILTERING if there is no measurement with a valid time 
*							interval available
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_GetBdot(Vec3D_t* Bdot, TickType_t* meas_tick);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get magnitude of Bdot using current B measurements
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]			result - pointer to float to populate with result
* @param[output]			meas_tick - pointer to TickType_t to populate with system tick
*							at time of first measurement. Pass in NULL if not required.
* @note						Result is in units of gauss/sec
* @return					E_OK if measurement was read successfully,
*							E_PENDING if there is no valid measurement available,
*							E_FILTERING if there is no measurement with a valid time 
*							interval available
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_GetMagBdot(float* result, TickType_t* meas_tick);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get rotation of B vector using current B measurements
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]			result - pointer to float to populate with result
* @param[output]			meas_tick - pointer to TickType_t to populate with system tick
*							at time of first measurement. Pass in NULL if not required.
* @note						Result is in units of rad/sec
* @return					E_OK if measurement was read successfully,
*							E_PENDING if there is no valid measurement available,
*							E_FILTERING if there is no measurement with a valid time 
*							interval available
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_GetRotB(float* result, TickType_t* meas_tick);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get current sun vector measurements
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]			meas - pointer to MeasPair_t to populate with result
* @note						Vectors are in the sat body frame and in arbitrary units
* @return					E_OK if measurement was read successfully,
*							E_ERROR if there is no valid measurement available
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType ADCS_Meas_GetS(MeasPair_t* meas);

/*!
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @brief Get rotation of S vector using current S measurements
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @param[output]			result - pointer to float to populate with result
 * @param[output]			meas_tick - pointer to TickType_t to populate with system tick
 *							at time of first measurement. Pass in NULL if not required.
 * @note					Result is in units of rad/sec
 * @return					E_OK if measurement was read successfully,
 *							E_PENDING if there is no valid measurement available,
 *							E_FILTERING if there is no measurement with a valid time
 *							interval available
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
ES_ReturnType ADCS_Meas_GetRotS(float *result, TickType_t *meas_tick);

/*!
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @brief Get most recent synchronized measurement of B and S
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @param[output]			meas_B - pointer to MeasPair_t to populate with B result
 * @param[output]			meas_S - pointer to MeasPair_t to populate with S result
 * @note						Sun vectors are in the sat body frame and in arbitrary units,
 *							B vectors are in the sat body frame and in units of gauss.
 * @return					E_OK if measurement was read successfully,
 *							E_PENDING if there is no valid measurement available
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
ES_ReturnType ADCS_Meas_GetLastMeas(MeasPair_t* meas_B, MeasPair_t* meas_S);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get whether sun was observed during latest sun phase
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return					1 if a pair of sun sensor measurements were successfully made
*							during the latest sun phase, 0 otherwise
* @note						If the current phase is a sun phase, it is considered the latest
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
uint8_t ADCS_Meas_GetSunLastLight();

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICED ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief ADCS measurement task function
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]				arg - will be cast to ADCS_MeasMode_e, specifies task mode
* @note						Exclusively for SysMan
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void ADCS_Meas_X_Task(void* arg);

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TESTS DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DEBUG_ENABLED
#include "AppTasks.h"

TestStatus_e ADCS_Meas_TestFixture_Reset();

/* Public debug interface for manually publishing measurements. 
See ADCS_Meas.c::publishMeasurement*/
void ADCS_Meas_DebugPublish(const MeasPair_t* pairB, const MeasPair_t* pairS);

#endif