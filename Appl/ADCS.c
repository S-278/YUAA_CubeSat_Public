/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file ADCS.c
* @brief ADCS C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Karim A., Elijah B., Rome T.
* @date              2023.04.05
* @details           Defines ADCS code
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "ADCS.h"
#include "ADCS_Meas.h"
#include "Detumbler.h"
#include "GGB.h"
#include "GGB_extend.h"
#include "Memory_Logs.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include "TimeUtils.h"
#include "TRIAD.h"
#include "panels.h"
#include "User_types.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "LIS3MDL_MAG_driver.h"
#include "stm32f4xx_hal_def.h"
#include <math.h>
#include <time.h>
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Macro aliases for data in EEPROM
// Note: these are safe to use as constants while reading, 
// because no other tasks besides the ADCS controller ever write to them
#define GGB_EXTEND_POWER_LEVEL EEPROM_temp.GGB_ex_params.extSpeed // GGB motor level when on [1 - 255]

/* Maximum age at which an ADCS measurement is still considered current.
   Rationale for 10sec: when the GGB is extending we don't want to look any further in the past. */
#define MEAS_MAX_AGE (10) // In s

/* Time for which the ADCS Controller task would wait after initializing,
   to give the ADCS Measurement task time to make a first attempt at measurements. */
#define MEAS_INIT_WAIT (30) // In s

/* Hysteresis on the critical rotation thresholds.
   ADCS override is cleared only if each of the two rotation metrics
   falls below the crit threshold minus hysteresis. */
#define CRIT_TH_HYSTERESIS (0.9)

/* Tolerance on the GGB extension target. 
   If the current GGB extension is within this tolerance from the target,
   consider the target met. */
#define GGB_TARGET_TOLERANCE (0.05) // In m

#define GGB_targetIsValid(trg) (0 < (float)trg && (float)trg < 10.0f)
#define GGB_speedIsValid(sp) (0 < (uint8_t)sp && (uint8_t)sp <= 255)
#define flagIsValid(flag) (flag == 0 || flag == 1)

/* How long the ADCS controller waits 
   between checking the newest measurements against ADCS thresholds */
#define MEAS_WAIT_ACTIVE (1*1000) // In ms. Used in Phases 1 and 2.
#define MEAS_WAIT_PASSIVE (30*60*1000) // In ms. Used in Phases 0 and 3.

/* Memory log constants */
#define WARN_STAT_SUBTYPE_DETUMB (0x1)
#define WARN_STAT_SUBTYPE_GGB (0x2)

#define errorcheck(x, success_val, lbl) if (x != success_val) goto lbl
#define EEPROM_begin Periph_BeginTransact(DEV_EEPROM, sizeof(EEPROM_Payload_t), HAL_MAX_DELAY)
#define EEPROM_end Periph_EndTransact(DEV_EEPROM)

// Parameters for receiving callbacks from other tasks
#define CB_MAILBOX_SZ (sizeof(ADCS_Thresholds_t)) // Currently largest piece of mail is a set of ADCS thresholds

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Restricted subset of the private task enum used by SysMan,
// to be used for SysMan_X_ADCS_SetTask
typedef enum {
    ADCS_ControllerTask_No = 3,
    ADCS_DetumblingTask_No,
    ADCS_GGBTask_No,
    ADCS_MeasTask_No,
} ADCS_Tasks_e;

// Enum for callback tickets. 
// Enumerates all actions which the ADCS controller takes on behalf of other tasks
// when tasks invoke the ADCS Controller's callbacks
typedef enum {
    Q_FAULT,
    Q_GGB_EN,
    Q_GGB_OVERRIDE,
    Q_GGB_SETTGT,
    Q_GGB_SETSPD,
    Q_GGB_SETSUN,
    Q_GGB_CPLT,
    Q_DETUMB_EN,
    Q_MTQ_POLARITY,
    Q_MTQ_MANUAL,
    Q_CRITOVERRIDE,
    Q_SET_TH,
    Q_TIMEOUT,      // No callback was called during the time the ADCS Controller waited for a callback
} ADCS_Q_Ticket_e;

typedef enum {
    ADCS_PHASE_0,           // Do nothing except monitoring; wait for override telecommand
    ADCS_PHASE_1,           // Active detumbling is running but GGB movement is stopped
    ADCS_PHASE_2,           // Active detumbling is stopped but GGB is moving
    ADCS_PHASE_3,           // Do nothing except monitoring
    ADCS_PHASE_NO_MEAS,     // No measurements available
    ADCS_PHASE_INV,         // Phase not determined
} ADCS_Phase_e;

typedef enum {
    GGBext_MODE_B,			// Use magnitude of Bdot as rotation threshold
    GGBext_MODE_S,
    GGBext_MODE_OVERRIDE,
} GGBext_Mode_e;

typedef struct 
{
    uint8_t GGB_ready; // GGB
    uint8_t GGB_autonomous; // GGB & MTM (& maybe SS)
    uint8_t active_detumb; // MTM
}  ADCS_Avail_t;


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static TaskHandle_t ADCSCtl_handle;

// ADCS controller mailbox.
// Callbacks place parameters/additional info into the mailbox.
// Mailbox contents are always left-justified (lower-numbered bytes populated first).
// Notice: unused mailbox bytes are not guaranteed to be zeroed!
// For now the mailbox is implemented as a FreeRTOS queue of length 1
static uint8_t CB_Mailbox[CB_MAILBOX_SZ * 1];
static StaticQueue_t CB_Mailbox_obj;
static QueueHandle_t CB_Mailbox_handle;

/* Whether the GGB is currently being moved in override mode. */
static uint8_t GGB_overrideFlag = 0;

static uint8_t GGB_taskIsRunning = 0;
/* Whether the MTQs are currently driven in override mode. */
static uint8_t MTQ_overrideFlag = 0;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static inline void initStaticData()
{
    ADCSCtl_handle = xTaskGetCurrentTaskHandle();
    // Set up the mailbox queue
    CB_Mailbox_handle = xQueueCreateStatic(1, CB_MAILBOX_SZ, CB_Mailbox, &CB_Mailbox_obj);
    
    GGB_overrideFlag = 0;
    GGB_taskIsRunning = 0;
    MTQ_overrideFlag = 0;
}

/* Updates GGB and/or detumb status in EEPROM 
and writes any appropriate memory logs. 
Will not do anything if new status passed in 
already matches current status in EEPROM.
Pass in -1 to keep the status unchanged. */
static void updateStatus(GGB_Status_e GGB_status, Detumb_Status_e detumb_status)
{
    uint8_t updateGGB = GGB_status != -1 && GGB_status != EEPROM_temp.GGB_state.status;
    uint8_t updateDetumb = detumb_status != -1 && detumb_status != EEPROM_temp.detumbState.status;
    if (!updateGGB && !updateDetumb) return;
    SharedMem_BeginAccess();
    if (updateGGB) EEPROM_temp.GGB_state.status = GGB_status;
    if (updateDetumb) EEPROM_temp.detumbState.status = detumb_status;
    SharedMem_EndAccess();
    EEPROM_begin;
    EEPROM_Emul_SyncInfo();
    EEPROM_end;
    if (updateGGB)
    {
        uint8_t logPayload[2] = { WARN_STAT_SUBTYPE_GGB, GGB_status };
        Log_WriteWarn(WARN_ENTRY_ADCS_STAT, logPayload, 2);
    }
    if (updateDetumb)
    {
        unint8_t logPayload[2] = { WARN_STAT_SUBTYPE_DETUMB, detumb_status };
        Log_WriteWarn(WARN_ENTRY_ADCS_STAT, logPayload, 2);
    }
}

/* Returns a struct showing which ADCS activities are available 
*  based on querying SysMan device status. */
static ADCS_Avail_t checkActivityDevs()
{
    ADCS_Avail_t avail;

    uint8_t checkGGB = Sys_IsDevAvail(DEV_GGB_CTRL);
    uint8_t checkMTM = Sys_IsDevAvail(DEV_MTM_HI) || Sys_IsDevAvail(DEV_MTM_LO);
    uint8_t checkSS = Sys_IsDevAvail(DEV_SS);

    avail.active_detumb = checkMTM;
    avail.GGB_ready = checkGGB;
    avail.GGB_autonomous = checkGGB && checkMTM && (checkSS || !EEPROM_temp.GGB_ex_params.requireSun);

    return avail;
}

static void stopDetumb()
{
    // Stop detumbling task
    Sys_X_ADCS_SetTask(ADCS_DetumblingTask_No, 0, NULL);
    // Stop MTQs
    Stop_Magnetorquers();
}

static void stopGGB()
{
    // Stop GGB movement task
    Sys_X_ADCS_SetTask(ADCS_GGBTask_No, 0, NULL); GGB_taskIsRunning = 0; 
    
    // Stop GGB motor driver.
    if (Sys_IsDevAvail(DEV_GGB_CTRL))    // If the motor driver is already in fault then there's nothing for us to do.
    { 
        HAL_StatusTypeDef retDisableDrivers;
        Sys_TryDev(retDisableDrivers, GGB_DISABLE_DRIVERS(), HAL_OK, DEV_GGB_CTRL);
        if (retDisableDrivers != HAL_OK)
        {
            // TODO: if we tried to stop the GGB but got a bus error,
            // we should force-reset the GGB controller because it might still be moving the GGB
            // TBR how to reset the GGB controller
        }
    }
}

#define startDetumb() Sys_X_ADCS_SetTask(ADCS_DetumblingTask_No, 1, NULL)

#define startGGB() { Sys_X_ADCS_SetTask(ADCS_GGBTask_No, 1, NULL); GGB_taskIsRunning = 1; }

#define restartMeasPassive() Sys_X_ADCS_SetTask(ADCS_MeasTask_No, 1, (void*)ADCS_MEAS_PASSIVE)
#define restartMeasActive() Sys_X_ADCS_SetTask(ADCS_MeasTask_No, 1, (void*)ADCS_MEAS_ACTIVE)

#define stopMeas() Sys_X_ADCS_SetTask(ADCS_MeasTask_No, 0, NULL)

/* Get the best available rotation rate measurement
*  from the ADCS Measurement task.
*  Measurement preference: 1) omega 2) B 3) S. */
static void getBestMeasurement(ADCS_Threshold_t* th, float* meas)
{
    // First try getting omega
    Vec3D_t omega = {.X = 0, .Y = 0, .Z = 0};
    float32_t omegaProject = 0;
    MeasPair_t pairB, pairS;
    ES_ReturnType retLastMeas = ADCS_Meas_GetLastMeas(&pairB, &pairS);
    if (retLastMeas == E_OK)
    {
        TickType_t measAge_Ticks = xTaskGetTickCount() - pairS.M0_tick;
        if (measAge_Ticks <= pdMS_TO_TICKS(MEAS_MAX_AGE * 1000) && pairB.intv != INTV_INVAL && pairS.intv != INTV_INVAL)
        {
            HAL_StatusTypeDef RTC_stat;
            time_t currT;
            Sys_TryDev(RTC_stat, GetTime(&currT), HAL_OK, DEV_RTC);
            // If we managed to get the current UTC time, then try to get omega.
            // Else we can't get omega.
            if (RTC_stat == HAL_OK && currT > TIME_SYNC_BOUNDARY)
            {
                time_t M0_T = currT - pdTICKS_TO_MS(measAge_Ticks) / 1000;
                FRESULT retOmegaVec;
                Sys_TryDev(retOmegaVec, TRIAD_GetOmegaVec(&pairB, &pairS, &M0_T, &omega), FR_OK, DEV_SD);
                if (retOmegaVec == FR_OK)
                {
                    omegaProject = omega.X * omega.X + omega.Y * omega.Y;
                    arm_sqrt_f32(omegaProject, &omegaProject);
                }
            }
        }
    }

    // If omega fails, get single-vector rotation rate
    float vecRot;
    uint8_t slowRot = 0;
    if (omegaProject == 0)
    {
        TickType_t measTick, currTick = xTaskGetTickCount();
        ES_ReturnType retRotS = E_PENDING, retRotB = E_PENDING;
        retRotB = ADCS_Meas_GetRotB(&vecRot, &measTick);
        if (currTick - measTick > pdMS_TO_TICKS(MEAS_MAX_AGE))
            retRotB = E_PENDING;
        if (retRotB != E_OK)
        {
            retRotS = ADCS_Meas_GetRotS(&vecRot, &measTick);
            if (currTick - measTick > pdMS_TO_TICKS(MEAS_MAX_AGE))
                retRotS = E_PENDING;
        }

        // If at least one of the return vals is OK, then vecRot contains the correct measurement.
        if (retRotB != E_OK && retRotS != E_OK)
        {
            // Else if neither return values is OK, we do not have a valid measurement, ...
            vecRot = 0;
            if (retRotB == E_FILTERING || retRotS == E_FILTERING)
                // ... but if we got an E_FILTERING,
                // we at least know rotation is very slow.
                slowRot = 1;
        }
    }

    if (omegaProject != 0)
    {
        *th = EEPROM_temp.ADCS_th.omega;
        *meas = omegaProject;
    }
    else if (vecRot != 0)
    {
        *th = EEPROM_temp.ADCS_th.singleVec;
        *meas = vecRot;
    }
    else *meas = 0;
}

/* Determines which ADCS phase we should currently be in
*  based on current measurements. */
static ADCS_Phase_e determinePhase()
{
    ADCS_Threshold_t th; float meas = 0;
    getBestMeasurement(&th, &meas);
    if (meas == 0) return ADCS_PHASE_NO_MEAS;

    // If critical override is set...
    if (EEPROM_temp.ADCS_critOverride)
    {
        // ... and rotation rates have fallen low enough, reset the override
        // and continue checking measurements.
        if (meas < th.crit * CRIT_TH_HYSTERESIS)
        {
            EEPROM_begin;
            EEPROM_temp.ADCS_critOverride = 0;
            EEPROM_Emul_SyncInfo();
            EEPROM_end;
        }
        // If override is set but rates have not fallen enough yet,
        // we should be actively detumbling.
        else return ADCS_PHASE_1;
    }
    // Else if critical override is not set and the critical threshold is exceeded, return Phase 0
    else if (meas > th.crit) return ADCS_PHASE_0;

    // If detumb status recorded in EEPROM is "running", check detumbling stop threshold,
    // and return Phase 1 if we're above that threshold.
    if (EEPROM_temp.detumbState.status == DETUMB_RUNNING)
    {
        if (meas > th.detumbStop) return ADCS_PHASE_1;
    }
    // Else if detumb status is not "running", check detumbling resume threshold,
    // and return Phase 1 if we're above that threshold.
    else
    {
        if (meas > th.detumbResume) return ADCS_PHASE_1;
    }

    // if the GGB extension target is met, return Phase 3, if not return Phase 2
    float deltaGGB = fabs(EEPROM_temp.GGB_ext_params.extCurrent - EEPROM_temp.GGB_ext_params.extTarget);
    if (deltaGGB < GGB_TARGET_TOLERANCE) return ADCS_PHASE_3;
    else return ADCS_PHASE_2;
}

/* Attempts to transition into the target phase. 
*  If a transition into the target phase is not possible 
*  because of device faults or enable/disable flags, 
*  transitions into Phase 3 instead. 
*  In any case, updates EEPROM and writes memory logs as appropriate.
*  Returns the phase entered into. */
static ADCS_Phase_e phaseTransition(ADCS_Phase_e newPhase)
{
    ADCS_Avail_t avail = checkActivityDevs();

    switch (newPhase)
    {
        case ADCS_PHASE_0:
        {
            stopDetumb();
            stopGGB();

            // GGB status gets set to Canceled unless GGB is already done, in which case it stays done
            GGB_Status_e newGGBstat = EEPROM_temp.GGB_state.status != GGB_DONE ? GGB_CANCELED : -1;
            // Detumbling status gets set to Canceled unconditionally
            Detumb_Status_e newDetumbstat = DETUMB_CANCELED;
            updateStatus(newGGBstat, newDetumbstat);
            restartMeasPassive();

            return ADCS_PHASE_0;
        }
        case ADCS_PHASE_1:
        {
            // First check device availability.
            // If detumbling is unavailable or disabled, transition to Phase 3 instead
            if (!MTQ_overrideFlag)
            {
                if (!avail.active_detumb)
                {
                    updateStatus(-1, DETUMB_FAULT);
                    return phaseTransition(ADCS_PHASE_3);
                }
                if (!EEPROM_temp.detumbState.enabled)
                    return phaseTransition(ADCS_PHASE_3);
            }

            stopGGB();
            // If MTQs are being overriden, neither the detumbling nor the measurement tasks
            // should be running
            if (!MTQ_overrideFlag)
            {
                restartMeasActive();
                startDetumb();
            }
            else
            {
                stopMeas();
                stopDetumb();
            }

            // GGB status gets set to Canceled unless GGB is already done, in which case it stays done
            updateStatus(EEPROM_temp.GGB_state.status != GGB_DONE ? GGB_CANCELED : -1,
                         DETUMB_RUNNING);
            
            return ADCS_PHASE_1;
        }
        case ADCS_PHASE_2:
        {
            // First check device availability.
            // If GGB movement is unavailable or disabled, transition to Phase 3 instead
            // Skip this check if the GGB is in override
            if (!GGB_overrideFlag)
            {
                if (!avail.GGB_autonomous)
                {
                    updateStatus(GGB_FAULT, -1);
                    return phaseTransition(ADCS_PHASE_3);
                }
                if (!EEPROM_temp.GGB_state.enabled)
                    return phaseTransition(ADCS_PHASE_3);
            }

            // Restart measurement task in active mode
            // Set GGB status to moving
            // Start GGB task as appropriate
            stopDetumb();

            restartMeasActive();

            // If GGB is in override or there is no sun requirement, start the GGB task now
            // Else don't touch GGB task, it will be controlled by periodic refresh from Controller task
            if (GGB_overrideFlag || !EEPROM_temp.GGB_ex_params.requireSun)
                startGGB();

            updateStatus(GGB_MOVING, DETUMB_DONE);

            return ADCS_PHASE_2;
        }
        case ADCS_PHASE_3:
        {
            stopGGB();
            stopDetumb();

            restartMeasPassive();

            // If detumbling is currently running (we got here from Phase 1)
            // or Canceled (we got here from Phase 0),
            // set it to Done. 
            // Else don't change its status (we got here because of a detumbling Fault).
            // If GGB is currently moving (we got here from Phase 2),
            // set it to Done. 
            // Else don't change its status (we got here because of a detumbling or GGB Fault).
            updateStatus(
                EEPROM_temp.detumbState.status == DETUMB_RUNNING
                || EEPROM_temp.detumbState.status == DETUMB_CANCELED ?
                DETUMB_DONE : -1,
                EEPROM_temp.GGB_state.status == GGB_MOVING ?
                GGB_DONE : -1
            );

            return ADCS_PHASE_3;
        }
        case ADCS_PHASE_NO_MEAS:
        {
            stopGGB();
            stopDetumb();

            updateStatus(DETUMB_FAULT, GGB_FAULT);

            return ADCS_PHASE_NO_MEAS;
        }
        #ifdef DEBUG_ENABLED
        default:
            Sys_RaiseLogicError(__FILE__, __LINE__, "inv enum val");
        #endif
    }
}

// Using `determinePhase', checks if we need to enter a new phase,
// and if so, uses `phaseTransition' to do that.
// Returns the value of its argument if no phase transition happened,
// or the new phase if a phase transition did happen.
static ADCS_Phase_e ADCS_Refresh(ADCS_Phase_e currPhase)
{
    ADCS_Phase_e ret = currPhase;

    if (!GGB_overrideFlag && !MTQ_overrideFlag)
    // Else either the GGB is being moved in override mode
    // or there is manual output to the MTQs,
    // and in that case phase determination and updating is skipped.
    {
        ADCS_Phase_e targetPhase = determinePhase();
        if (targetPhase != currPhase)
            ret = phaseTransition(targetPhase);
    }

    // If we need to be extending the GGB with sun requirement, check the sun requirement here.
    if (ret == ADCS_PHASE_2 && !GGB_overrideFlag && EEPROM_temp.GGB_ex_params.requireSun)
    {
        uint8_t haveSun = ADCS_Meas_GetS(NULL, NULL, NULL) == E_OK;
        if (haveSun && !GGB_taskIsRunning) startGGB();
        else if (!haveSun && GGB_taskIsRunning) stopGGB();
    }

    return ret;
}

// Generic handler for all callbacks.
// If `q_mail' != NULL, will copy `mailSz' bytes from `q_mail' into the mailbox.
// In any case, sends the signal `q_ticket' to the ADCS Controller.
static void genericCBHandler(ADCS_Q_Ticket_e q_ticket, const void* q_mail, uint32_t mailSz)
{
    if (q_mail != NULL)
    {
        #ifdef DEBUG_ENABLED
        if (mailSz > Q_MAILBOX_SZ) 
        {
            Sys_RaiseLogicError(__FILE__, __LINE__, "mail too big");
        }
        #endif

        if (xQueueSendToBack(CB_Mailbox_handle, q_mail, 0) != pdPASS)
        { 
            Sys_RaiseLogicError(__FILE__, __LINE__, "q full");
        }
    }
    if (xTaskNotify(ADCSCtl_handle, q_ticket, eSetValueWithoutOverwrite) != pdPASS) 
    {
        Sys_RaiseLogicError(__FILE__, __LINE__, "notif pending");
    }
}

// Generic function to wait for callbacks
// to abstract away which FreeRTOS object is used.
// mail must point to a buffer big enough to receive callback mail.
static ADCS_Q_Ticket_e waitForCallback(uint8_t mail[CB_MAILBOX_SZ], TickType_t timeout)
{
    ADCS_Q_Ticket_e retstat;
    BaseType_t gotNotification = xTaskNotifyWait(0, 0, &retstat, timeout); // Wait for a task notification with given timeout
    if (gotNotification == pdFALSE) 
    {
        retstat = Q_TIMEOUT; // If FreeRTOS returned `pdFALSE' then no task notification was received
    }
    else 
    {
        xQueueReceive(CB_Mailbox_handle, mail, 0); // If a callback included mail, receive it.
    }
    return retstat;
}

// Returns the system tick count
// at which the ADCS Controller task should next 
// check measurements against thresholds.
static TickType_t getNextMeasTime(ADCS_Phase_e currPhase)
{
    TickType_t ticksToWait;
    switch (currPhase)
    {
        case ADCS_PHASE_1:
        case ADCS_PHASE_2:
        {
            uint16_t measIntv;
            // If possible, sync up with the ADCS measurement task
            if (ADCS_Meas_GetB(NULL, NULL, &measIntv) == E_OK) 
            {
                ticksToWait = measIntv;
            }
            else 
            {
                ticksToWait = pdMS_TO_TICKS(MEAS_WAIT_ACTIVE);
            }
            break;
        }
        case ADCS_PHASE_3:
        case ADCS_PHASE_0:
            ticksToWait = pdMS_TO_TICKS(MEAS_WAIT_PASSIVE);
            break;
        case ADCS_PHASE_NO_MEAS:
            ticksToWait = portMAX_DELAY;
            break;
        default:
            ticksToWait = pdMS_TO_TICKS(MEAS_INIT_WAIT * 1000);
            break;
    }

    return xTaskGetTickCount() + ticksToWait;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

void ADCS_X_ControllerTask(void* arg)
{
    initStaticData();

    ADCS_Phase_e currPhase = ADCS_PHASE_INV; 

    // Variable to hold the number of system tick counts
    // which is the latest time at which the ADCS Controller task should wake.
    TickType_t nextWakeup_Ticks = getNextMeasTime(currPhase);

    // Main task loop
    ADCS_Q_Ticket_e cb_ticket; 
    uint8_t cb_mail[CB_MAILBOX_SZ];
    while (1)
    {
        TickType_t currTick = xTaskGetTickCount();
        cb_ticket = waitForCallback(cb_mail, 
                                    nextWakeup_Ticks > currTick ? 
                                    currTick - nextWakeup_Ticks
                                    : 0);

        // No callbacks were invoked during the timeout - 
        // if new measurements are available, check them against ADCS thresholds
        if (cb_ticket == Q_TIMEOUT)
        {
            // If we timed out while the MTQs are in override,
            // that means MTQ override is finished and we should return to normal operation.
            if (MTQ_overrideFlag)
            {
                MTQ_overrideFlag = 0;
                stopDetumb();
                // Pass in invalid phase to force a clean phase transition
                currPhase = ADCS_Refresh(ADCS_PHASE_INV);
            }
            // Normally, the ADCS controller wakes up to check if there's a new measurement available
            else if (ADCS_Meas_IsUnread(ADCS_ControllerTask_No))
            {
                // Refresh using new measurement
                currPhase = ADCS_Refresh(currPhase);
                ADCS_Meas_MarkAsRead(ADCS_ControllerTask_No);
            }

            // Update time of next measurement check
            nextWakeup_Ticks = getNextMeasTime(currPhase);
        }
        // A callback was invoked before the timeout elapsed
        else
        {
            switch (cb_ticket)
            {
                case Q_FAULT:
                {    
                    // SysMan recieved a device error from a subordinate task, 
                    // determined the device to be faulted, and terminated the task if it was reliant on the device
                    DevName_e faultedDev; memcpy(&faultedDev, cb_mail, sizeof(DevName_e)); 

                    switch (faultedDev)
                    {
                    case DEV_GGB_CTRL:
                    {
                        GGB_taskIsRunning = 0; GGB_overrideFlag = 0;
                        break;
                    }
                    default:
                        break;
                    }
                    
                    // Just do a refresh, this handles status updates and memory logs
                    currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                case Q_GGB_EN:
                {
                    // from telecommand parser, update EEPROM GGB enable/disable flag and rerun ADCS_Refresh
                    if (!flagIsValid(cb_mail[0])) Sys_RaiseLogicError(__FILE__, __LINE__, "inv EN val");
                    SharedMem_BeginAccess();
                    EEPROM_temp.GGB_state.enabled = cb_mail[0];
                    SharedMem_EndAccess();
                    if (cb_mail[0] == 0) updateStatus(GGB_DONE, -1);
                    currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                case Q_GGB_OVERRIDE:
                {
                    ADCS_Avail_t avail = checkActivityDevs();
                    if (avail.GGB_ready) 
                    {
                        GGB_overrideFlag = 1;
                        EEPROM_begin;
                        memcpy(&EEPROM_temp.GGB_ex_params.extTarget, cb_mail, sizeof(float)); 
                        EEPROM_temp.detumbState.enabled = 0;
                        EEPROM_Emul_SyncInfo();
                        EEPROM_end;
                        // Forcefully transition into Phase 2 to turn off detumb and start GGB task
                        if (phaseTransition((currPhase = ADCS_PHASE_2)) != ADCS_PHASE_2)
                            Sys_RaiseLogicError(__FILE__, __LINE__, "failed GGB override");
                    } 
                    break;
                }
                // GGB set target, speed, and sun requirement 
                // make appropriate updates in EEPROM then rerun ADCS_Refresh 
                // (except don't for the speed callback as the speed doesn't change what phase we should be in). 
                case Q_GGB_SETTGT:
                {   
                    float newTgt; memcpy(&newTgt, cb_mail, sizeof(float));
                    if (!GGB_targetIsValid(newTgt)) Sys_RaiseLogicError(__FILE__, __LINE__, "inv GGB tgt");
                    EEPROM_begin;
                    EEPROM_temp.GGB_ex_params.extTarget = newTgt;
                    EEPROM_Emul_SyncInfo();
                    EEPROM_end;
                    currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                case Q_GGB_SETSPD:
                {
                    uint8_t newSp = cb_mail[0];
                    if (!GGB_speedIsValid(newSp)) Sys_RaiseLogicError(__FILE__, __LINE__, "inv GGB sp");
                    EEPROM_begin;
                    EEPROM_temp.GGB_ex_params.extSpeed = newSp;
                    EEPROM_Emul_SyncInfo();
                    EEPROM_end;
                    break;
                }
                case Q_GGB_SETSUN:
                {
                    if (!flagIsValid(cb_mail[0])) Sys_RaiseLogicError(__FILE__, __LINE__, "inv req val");
                    EEPROM_begin;
                    EEPROM_temp.GGB_ex_params.requireSun = cb_mail[0];  
                    EEPROM_Emul_SyncInfo();
                    EEPROM_end;
                    currPhase = ADCS_Refresh(currPhase); 
                    break;
                }
                case Q_GGB_CPLT:
                {
                    stopGGB();
                    // If called after override movement, also call the appropriate SysMan CB.
                    if (GGB_overrideFlag)
                    {
                        GGB_overrideFlag = 0;
                        Sys_X_ADCS_End_GGB_Override();
                        // Pass in invalid phase to ensure clean phase transition
                        currPhase = ADCS_Refresh(ADCS_PHASE_INV);
                    }
                    else currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                case Q_DETUMB_EN:
                { 
                    if (!flagIsValid(cb_mail[0])) Sys_RaiseLogicError(__FILE__, __LINE__, "inv EN val");
                    // update EEPROM detumbling enable/disable flag,
                    // if detumb is being disabled then force detumb status to done,
                    // then rerun ADCS_Refresh
                    SharedMem_BeginAccess();
                    EEPROM_temp.detumbState.enabled = cb_mail[0]; 
                    SharedMem_EndAccess();
                    if (cb_mail[0] == 0) updateStatus(-1, DETUMB_DONE);
                    currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                case Q_MTQ_POLARITY:
                {
                    MTQ_Polarity_e newPolarity; memcpy(&newPolarity, cb_mail, sizeof(MTQ_Polarity_e));
                    if (newPolarity != MTQ_POLARITY_NORMAL && newPolarity != MTQ_POLARITY_INVERT)
                        Sys_RaiseLogicError(__FILE__, __LINE__, "inv polarity val");
                    EEPROM_begin;
                    EEPROM_temp.MTQ_polarity = newPolarity;
                    EEPROM_Emul_SyncInfo();
                    EEPROM_end;
                    // If we are currently in Phase 1, re-enter phase 1, otherwise do nothing else.
                    if (currPhase == ADCS_PHASE_1)
                    {
                        phaseTransition(ADCS_PHASE_1);
                    }
                    break;
                }
                case Q_MTQ_MANUAL:
                {  
                    uint16_t duration; memcpy(&duration, cb_mail, sizeof(uint16_t));
                    if (duration == 0) Sys_RaiseLogicError(__FILE__, __LINE__, "inv duration");
                    MTQ_out_comp_t output[3]; memcpy(output, cb_mail + sizeof(uint16_t), sizeof(output));
                    // TODO add check output validity
                    
                    MTQ_overrideFlag = 1;
                    // Forcefully transition into Phase 1 to turn off all subordinate ADCS tasks
                    if (phaseTransition((currPhase = ADCS_PHASE_1)) != ADCS_PHASE_1)
                        Sys_RaiseLogicError(__FILE__, __LINE__, "failed MTQ override");

                    status_t setMTQ = Detumbling_X_SetOutput(output);
                    // TODO MTQ error handling

                    nextWakeup_Ticks = xTaskGetTickCount() + pdMS_TO_TICKS(1000 * duration);
                    // Note: we should be in no danger of overflow here,
                    // since a tick count overflows every ~50 days,
                    // but UINT16_MAX seconds is a lot less than a day.

                    break;
                }
                case Q_CRITOVERRIDE:
                {
                    // set appropriate flag in EEPROM and rerun ADCS_Refresh .
                    EEPROM_begin;
                    EEPROM_temp.ADCS_critOverride = 1;
                    EEPROM_Emul_SyncInfo();
                    EEPROM_end;
                    currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                case Q_SET_TH:
                {
                    EEPROM_begin;
                    memcpy(&EEPROM_temp.ADCS_th, cb_mail, sizeof(ADCS_Thresholds_t));
                    EEPROM_Emul_SyncInfo();
                    EEPROM_end;
                    currPhase = ADCS_Refresh(currPhase);
                    break;
                }
                default:
                    Sys_RaiseLogicError(__FILE__, __LINE__, "inv enum val");
            }
        }
    }
}

// Callback for use exclusively by the system manager.
// SysMan will call this callback if one of the ADCS subordinate tasks
// raises a device error, and that device is determined to be faulted.
// If the subordinate task was reliant on the device, SysMan already terminated it.
// This callback needs to respond to the fault, 
// including updating the ADCS status.
void ADCS_X_FaultCallback(DevName_e device)
{
    genericCBHandler(Q_FAULT, &device, sizeof(DevName_e));
}

// Request change of GGB state to enable or disable it,
// for use exclusively by TelComm Parser.
void ADCS_X_GGB_EnableDisableCallback(uint8_t enable)
{
    genericCBHandler(Q_GGB_EN, &enable, sizeof(uint8_t));
}

// Request override of GGB extension level 
// for use exclusively by SysMan.
void ADCS_X_GGB_OverrideCallback(float overrideLevel)
{
    genericCBHandler(Q_GGB_OVERRIDE, &overrideLevel, sizeof(float));
}

// Request change in GGB extension target
// for use exclusively by TelComm Parser.
void ADCS_X_GGB_SetExtTarget(float newTarget)
{
    genericCBHandler(Q_GGB_SETTGT, &newTarget, sizeof(float));
}

// Request change in GGB extension speed
// for use exclusively by TelComm Parser.
void ADCS_X_GGB_SetExtSpeed(uint8_t newSpeed)
{
    genericCBHandler(Q_GGB_SETSPD, &newSpeed, sizeof(uint8_t));
}

// Request change in GGB sun requirement
// for use exclusively by TelComm Parser.
void ADCS_X_GGB_SetSunReq(uint8_t requireSun)
{
    genericCBHandler(Q_GGB_SETSUN, &requireSun, sizeof(uint8_t));
}

// Request change of detumbling state to enable or disable it,
// for use exclusively by TelComm Parser.
void ADCS_X_Detumb_EnableDisableCallback(uint8_t enable)
{
    genericCBHandler(Q_DETUMB_EN, &enable, sizeof(uint8_t));
}

void ADCS_X_GGB_Cplt_Callback()
{
    genericCBHandler(Q_GGB_CPLT, NULL, 0);
}

void ADCS_X_MTQPolarityCallback(MTQ_Polarity_e newPolarity)
{
    genericCBHandler(Q_MTQ_POLARITY, &newPolarity, sizeof(MTQ_Polarity_e));
}

void ADCS_X_CritOverrideCallback()
{
    genericCBHandler(Q_CRITOVERRIDE, NULL, 0);
}

void ADCS_X_SetThresholdsCallback(const ADCS_Thresholds_t* newTh)
{
    genericCBHandler(Q_SET_TH, newTh, sizeof(ADCS_Thresholds_t)); 
}

void ADCS_X_ManualMTQCallback(uint16_t duration, int8_t outputs[3])
{
    uint8_t mailBuf[sizeof(uint16_t) + 3 * sizeof(MTQ_out_comp_t)];

    memcpy(mailBuf, &duration, sizeof(uint16_t));

    MTQ_out_comp_t* out_comp = (MTQ_out_comp_t*)(mailBuf + sizeof(uint16_t));
    for (int i = 0; i < 3; i++, out_comp++)
    {
        if (outputs[i] > 0) { out_comp->dir = 1; out_comp->perc = outputs[i]; }
        else { out_comp->dir = 0; out_comp->perc = outputs[i] * -1; }
    }

    genericCBHandler(Q_MTQ_MANUAL, mailBuf, sizeof(mailBuf));
}

