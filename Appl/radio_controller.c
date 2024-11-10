/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file radio_controller.c
* @brief Implementation of radio control system
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Jack M., Elijah B.
* @version           1.0.0
* @date              2023.01.21
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "radio_controller.h"
#include "AutoDL_Package.h"
#include "compChecks.h"
#include "Memory_Logs.h"
#include "OrbitProp.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include "TimeUtils.h"
#include "TCV.h"
#include "EEPROM_emul.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "timers.h"
#include "User_types.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define EEPROM_begin Periph_BeginTransact(DEV_EEPROM, 1024, HAL_MAX_DELAY)
#define EEPROM_end Periph_EndTransact(DEV_EEPROM)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef enum {
    IDLE_MODE, 
    WAIT_MODE, 
    LOSS_OF_TIME_WAIT_MODE, 
    RECOVERY_MODE, 
    CONTROLLED_DOWNLINK, 
    AUTONOMOUS_DOWNLINK, 
    END_GND_PASS, 
} RadioState_e;

typedef RadioState_e(*mode_handler_t)(void);

typedef enum {
    SIGNAL_RECOVERY,
    SIGNAL_TO_UPDATE,
    SIGNAL_DOWNLINK_REQ,
    SIGNAL_ESTTC_RCV,
} RadioSignal_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static RadioState_e idle_mode_update();
static RadioState_e wait_mode_update();
static RadioState_e loss_of_time_update();
static RadioState_e recovery_mode_update();
static RadioState_e controlled_downlink_update();
static RadioState_e autonomous_downlink_update();
static RadioState_e end_gnd_pass_update();

/* Attempts to schedule the next gnd pass, 
first using pass times on SD, then trying orbit propagation.
Both methods will accept pass times which are in the past,
but by no longer than the duration of the passive pass window. 
If pass was successfully found, populates passTick with ticks until pass.
If passTick is zero when this function returns 1, 
controller should immediately switch to Wait mode.
Returns whether a pass was successfully scheduled. */
static int schedulePass(const time_t* currT, TickType_t* passTick);

static inline void setBeacon(uint8_t enable)
{
    HAL_StatusTypeDef TCV_stat;
    Sys_TryDev(TCV_stat, TCV_SetBeacon(1), HAL_OK, DEV_TCV); // If TCV fails SysMan will terminate this task
    CompCheck_X_Beacon_Enable_Callback(enable);
}

static inline HAL_StatusTypeDef setRecModeAlarm()
{
    HAL_StatusTypeDef RTC_stat;
    Sys_TryDev(RTC_stat, RTC_X_SetAlarm(EEPROM_emul_DataTemp.recModeAlarm), HAL_OK, DEV_RTC); 
    return RTC_stat;
}

static HAL_StatusTypeDef resetRecModeAlarm()
{
    HAL_StatusTypeDef RTC_stat; time_t currT;
    Sys_TryDev(RTC_stat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
    if (RTC_stat != HAL_OK) return RTC_stat;
    
    time_t recModeAlarmT = currT + (REC_MODE_TO * 60 * 60 * 24);
    SharedMem_BeginAccess();
    EEPROM_emul_DataTemp.recModeAlarm = recModeAlarmT;
    SharedMem_EndAccess();
    EEPROM_begin;
    EEPROM_Emul_SyncInfo();
    EEPROM_end;

    return setRecModeAlarm();
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static TaskHandle_t RadioCtl_TaskHandle;

static const mode_handler_t mode_handlers[] = {
    idle_mode_update,
    wait_mode_update,
    loss_of_time_update,
    recovery_mode_update,
    controlled_downlink_update,
    autonomous_downlink_update,
    end_gnd_pass_update,
};

static TickType_t pass_start_time = 0, ctrl_DL_start_time = 0;

static inline void init_static_data()
{
    RadioCtl_TaskHandle = xTaskGetCurrentTaskHandle();
    
    pass_start_time = 0; ctrl_DL_start_time = 0;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Macros for EEPROM values. Even though the ESTTC UART task has write access,
these are all fundamental data types so reading them is atomic. */
#define COMMAND_TO EEPROM_emul_DataTemp.radioTOs.cmdTO
#define ACTIVE_PASS_WIN_TO EEPROM_emul_DataTemp.radioTOs.acivePassWindow
#define PASSIVE_PASS_WIN_TO EEPROM_emul_DataTemp.radioTOs.passPassWindow
#define REC_MODE_TO EEPROM_emul_DataTemp.radioTOs.recModeTO

/* Number of ms to wait before retrying pass prediction 
if orbit model fails to find a pass within the maximum lookahead window */
#define PASS_PRED_RETRY_PERIOD (30*60*1000)

#define raiseUnexpectedSignal(signalVal, currMode) \
    {\
        char tempMsg[30]; \
        snprintf(tempMsg, 30, "unexpected signal %lu in mode %lu", signalVal, currMode); \
        Sys_RaiseLogicError(__FILE__, __LINE__, tempMsg); \
    }


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TASK FUNCTION DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

void RadioCtl_X_Task(void* arg)
{
    time_t currT; HAL_StatusTypeDef RTC_stat;
    RadioState_e curr_state = LOSS_OF_TIME_WAIT_MODE;

	/* One-time task initialization */
    init_static_data();

    // Check what's up with Recovery Mode.
    // If the RTC is offline, we never enter Recovery Mode.
    Sys_TryDev(RTC_stat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
    if (RTC_stat == HAL_OK)
    {
        // If the RTC is online, check if we should already be in Rec Mode,
        // otherwise set the rec mode RTC alarm.
        // Init routine has already propagated/updated recovery mode alarm
        // to be consistent with current time
        if (EEPROM_emul_DataTemp.recModeAlarm < currT) curr_state = RECOVERY_MODE;
        else
        {
            RTC_stat = setRecModeAlarm();
            // If we are successfully not in recovery mode, 
            // check time sync status and set entry mode accordingly
            if (RTC_stat == HAL_OK)
            {
                TimeSync_e timeStatus;
                Sys_TryDev(RTC_stat, RTC_GetTimeSync(&timeStatus), HAL_OK, DEV_RTC);
                if (RTC_stat == HAL_OK && timeStatus == TIME_SYNCED) curr_state = IDLE_MODE;
                else curr_state = LOSS_OF_TIME_WAIT_MODE;
            }
        }
    }

	while (1)
	{
		
		// Call the appropriate mode handler
		// based on the mode we are entering right now.
		// This call only returns when it is time to transition 
		// to a new mode.
		RadioState_e new_state = (*mode_handlers[curr_state])();
        // If we got a Recovery Mode signal,
        // check for false alarm. 
        // This is the only case where a signal may or may not 
        // trigger a change into a different mode.
        if (new_state == RECOVERY_MODE)
        {
            /* Since the RTC can only set an alarm for a day of the month and not a specific date,
            if the recovery mode TO is more than 30 days (or if it's February),
            the RTC alarm will actually fire early within the current month. */
            HAL_StatusTypeDef RTC_stat; time_t currT;
            Sys_TryDev(RTC_stat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
            if (RTC_stat == HAL_OK && currT < EEPROM_emul_DataTemp.recModeAlarm)
            {
                // If this is a false alarm,
                // just reset the RTC alarm and re-enter the same mode you were in before
                // (curr_state is not updated).
                RTC_stat = setRecModeAlarm();
                // Note - we do not read RTC_stat, 
                // because even if setting the alarm failed,
                // the helper already raised the error,
                // so we don't have to do anything about it here.
            }
            // Else actually do go to recovery mode
            else curr_state = RECOVERY_MODE;
        }
        else curr_state = new_state;
	}
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* MODE HANDLERS DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static int schedulePass(const time_t* currT, TickType_t* passTick)
{
    FRESULT logStat; time_t nextPassT = 0;
    Sys_TryDev(logStat, Log_X_CleanPassTime(currT), FR_OK, DEV_SD);
    if (logStat == FR_OK) Sys_TryDev(logStat, Log_X_GetPassTime(&nextPassT), FR_OK, DEV_SD);
    if (logStat == FR_OK && nextPassT != 0) 
    {
        // If at this point nextPassT is in the past,
        // we should immediately switch to Wait mode
        if (nextPassT < *currT) { *passTick = 0; return 1; }
        // Else just go to the end of the function
    }
    else // If reading time from the SD failed, try orbit propagation
    {
        time_t propStartT = *currT - EEPROM_emul_DataTemp.radioTOs.passivePassWindow * 60;
        do
        {
            ES_ReturnType propStat = Orbit_GetNextPassPred(&propStartT, &nextPassT, &logStat);
            if (propStat != E_OK) break; // No valid TLEs, give up
            else if (logStat != FR_OK) Sys_RaiseDevError(DEV_SD, logStat); // SD error, try again
            else break; // Propagation ran without error
        } while (SYS_DEV_IS_AVAIL(Sys_GetDevStatus(DEV_SD)));
    }

    if (nextPassT != 0) { *passTick = (nextPassT - *currT) * configTICK_RATE_HZ; return 1; }
    else return 0;
}


RadioState_e idle_mode_update() {

    HAL_StatusTypeDef RTC_stat; time_t currT;
    Sys_TryDev(RTC_stat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
    if (RTC_stat != HAL_OK || currT < TIME_SYNC_BOUNDARY) return LOSS_OF_TIME_WAIT_MODE;

    setBeacon(1);

    RadioSignal_e receivedSignal; TickType_t ticksUntilPass;
    while (1)
    {
        if (schedulePass(&currT, &ticksUntilPass))
        {
            // If we successfully schedule a pass,
            // wait for a signal. If ticksUntilPass elapses with no signals, 
            // switch to Wait mode.
            if (xTaskNotifyWait(0, 0, &receivedSignal, ticksUntilPass) == pdTRUE) break; // Go to end of function to examine signal
            else return WAIT_MODE; 
        }
        else
        {
            // If pass scheduling fails for any reason, 
            // wait for signals for 30 min, then retry scheduling
            if (xTaskNotifyWait(0, 0, &receivedSignal, pdMS_TO_TICKS(PASS_PRED_RETRY_PERIOD)) == pdTRUE) break; // Go to end of function to examine signal
        }
    }

    switch (receivedSignal)
    {
    case SIGNAL_DOWNLINK_REQ: return CONTROLLED_DOWNLINK;
    case SIGNAL_RECOVERY: return RECOVERY_MODE;
    default: raiseUnexpectedSignal(receivedSignal, IDLE_MODE);
    }
}

RadioState_e wait_mode_update() {
    pass_start_time = xTaskGetTickCount();

    Sys_X_Radio_SetAutoDLTask(1);

    RadioSignal_e signalReceived;
    if (xTaskNotifyWait(0, 0, &signalReceived, pdMS_TO_TICKS(COMMAND_TO * 60 * 1000)) \
        == pdTRUE)
    {
        switch (signalReceived)
        {
        case SIGNAL_DOWNLINK_REQ: return CONTROLLED_DOWNLINK;
        case SIGNAL_RECOVERY: return RECOVERY_MODE;
        default: raiseUnexpectedSignal(signalReceived, WAIT_MODE);
        }
    }
    else return AUTONOMOUS_DOWNLINK;
}

RadioState_e loss_of_time_update() {

    setBeacon(1);
    
    RadioSignal_e receivedSignal;
    xTaskNotifyWait(0, 0, &receivedSignal, portMAX_DELAY);

    switch (receivedSignal)
    {
    case SIGNAL_DOWNLINK_REQ: return CONTROLLED_DOWNLINK;
    case SIGNAL_RECOVERY: return RECOVERY_MODE;
    default: raiseUnexpectedSignal(receivedSignal, IDLE_MODE);
    }
}

RadioState_e recovery_mode_update(){
    
    setBeacon(0);

    // wait for downlink request
    RadioSignal_e signalReceived;
    xTaskNotifyWait(0, 0, &signalReceived, portMAX_DELAY);

    if (signalReceived != SIGNAL_DOWNLINK_REQ) raiseUnexpectedSignal(signalReceived, RECOVERY_MODE);
    else return CONTROLLED_DOWNLINK;
}

RadioState_e controlled_downlink_update() {
    
    ctrl_DL_start_time = xTaskGetTickCount();
    
    setBeacon(0);

    HAL_StatusTypeDef TCV_retstat;
    Sys_TryDev(TCV_retstat, TCV_SetPipe(1), HAL_OK, DEV_TCV); //enter transparent mode
    
    resetRecModeAlarm();
    
    uint32_t receivedSignal;
    while (xTaskNotifyWait(0, 0, &receivedSignal, pdMS_TO_TICKS(COMMAND_TO * 60 * 1000))\
        == pdTRUE) 
    {
        switch (receivedSignal)
        {
        case SIGNAL_ESTTC_RCV: continue;
        case SIGNAL_TO_UPDATE:
        {
            // Note - we don't read the alarm set return status,
            // because if it failed there's nothing to do
            resetRecModeAlarm();
        }
        default: raiseUnexpectedSignal(receivedSignal, CONTROLLED_DOWNLINK);
        }
    }

    // At this point we stopped getting ESTTC received signals
    if (ctrl_DL_start_time == 0) Sys_RaiseLogicError(__FILE__, __LINE__, "ctl DL start time not set");
    if ((xTaskGetTickCount() - ctrl_DL_start_time) / (configTICK_RATE_HZ * 60) >= ACTIVE_PASS_WIN_TO) \
        return END_GND_PASS;
    else return AUTONOMOUS_DOWNLINK;
}

RadioState_e autonomous_downlink_update() {
    
    setBeacon(0);

    HAL_StatusTypeDef TCV_retstat;
    Sys_TryDev(TCV_retstat, TCV_SetPipe(1), HAL_OK, DEV_TCV); //enter transparent mode
    
    // TODO transmitting AutoDL package


}

RadioState_e end_gnd_pass_update() {

    Sys_X_Radio_SetAutoDLTask(0);
    
    HAL_StatusTypeDef TCV_retstat;
    Sys_TryDev(TCV_retstat, TCV_SetPipe(0), HAL_OK, DEV_TCV); //exit transparent mode
    Sys_TryDev(TCV_retstat, TCV_SetRFMode(TCV_BASE_RF_MODE), HAL_OK, DEV_TCV); //restore base link speed

    CompCheck_X_Beacon_ESLPClear_Callback();

    HAL_StatusTypeDef RTC_stat; time_t currT;
    Sys_TryDev(RTC_stat, RTC_GetTime(&currT), HAL_OK, DEV_TCV);
    if (RTC_stat == HAL_OK)
    {
        SharedMem_BeginAccess();
        EEPROM_emul_DataTemp.lastGndPass = currT; 
        SharedMem_EndAccess();
        EEPROM_begin;
        EEPROM_Emul_SyncInfo();
        EEPROM_end;
    }

    ctrl_DL_start_time = (pass_start_time = 0);
    
    if (RTC_stat != HAL_OK) return LOSS_OF_TIME_WAIT_MODE;
    else
    {
        if (currT < TIME_SYNC_BOUNDARY) return LOSS_OF_TIME_WAIT_MODE;
        else return IDLE_MODE;
    }
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* CALLBACKS DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Generic way to handle callbacks
static inline void genericCBHandler(RadioSignal_e signal)
{
    /* Callbacks to the Radio Controller are implemented as FreeRTOS Task Notifications,
    because signals to the Radio Controller never carry parameters,
    and there is no need to queue them because the Radio Controller immediately preempts 
    any task which could have notified it. */
    xTaskNotify(RadioCtl_TaskHandle, signal, eSetValueWithOverwrite);
}
static inline void genericCBHandlerFromISR(RadioSignal_e signal)
{
    BaseType_t requestContextSwitch = pdFALSE;
    xTaskNotifyFromISR(RadioCtl_TaskHandle, signal, eSetValueWithOverwrite, &requestContextSwitch);
    portYIELD_FROM_ISR(requestContextSwitch);
}

void RadioCtl_X_TimeSyncCallback(time_t delta)
{
    // Not implemented using `genericCBHandler' because this isn't part of the state machine.
    // TODO
}

void RadioCtl_X_RecoveryCallback()
{
    genericCBHandlerFromISR(SIGNAL_RECOVERY);
}

void RadioCtl_X_RecoveryTOUpdateCallback()
{
    genericCBHandler(SIGNAL_TO_UPDATE);
}

void RadioCtl_X_DownlinkReqCallback()
{
    genericCBHandler(SIGNAL_DOWNLINK_REQ);
}

void RadioCtl_X_ESTTCReceivedCallback()
{
    genericCBHandler(SIGNAL_ESTTC_RCV);
}
