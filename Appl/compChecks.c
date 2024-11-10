/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file compChecks.c
* @brief Component checks C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Bill F., Elijah B.
* @version           0.1.1
* @date              2022.10.23
*
* @details           Defines component check-up routines
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "compChecks.h"
#include "ADCS_Meas.h"
#include "power_dist.h"
#include "system_manager.h"
#include "EPS.h"
#include "GGB.h"
#include "GGB_config.h"
#include "Memory_Logs.h"
#include "RTD.h"
#include "SS.h"
#include "TCV.h"
#include "TimeUtils.h"
#include "TRIAD.h"
#include "EEPROM_emul.h"
#include "LIS3MDL_MAG_driver.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <float.h>
#include <stddef.h>
/*
*~~~~~~
* PRIVATE DEFINES
*~~~~~~
*/
#define TEMPERATURE_CHECK_PERIOD (1*60*1000) // Temperature Checker runs every minute
#define GGB_CHECK_PERIOD (1*60*60*1000) // GGB checker runs every hour
#define TCV_CHECK_PERIOD (1*60*1000) // Transceiver checker runs every minute
#define SS_CHECK_PERIOD (1*60*60*1000) // Sun sensor checker runs every hour
#define ANT_CHECK_NORMAL_PERIOD (15*60*1000) // ANT checker runs every 15 min
#define TCV_SHORT_REL_PERIOD (10) // ANT checker sets TCV to 10min antenna release timer after the first failure
#define INFO_DUMP_PERIOD (10*60*1000) // 10 min 
#define RADIO_BEACON_PERIOD (60*1000) // 1 min

/* Maximum number of attempts the TCV should make to deploy the UHF antenna
before giving up. */
#define ANT_DEPLOY_MAX_ATTEMPTS (10)

// For some reason FreeRTOS requires timers to have a plain text name.
// For now just giving all comp checks timers this name.
// If needed, can add individual names later.
#define DEFAULT_TIMER_NAME ("CompChecks Timer")

// We have one callback function per timer, so not using the timer ID for anything for now
#define DEFAULT_TIMER_ID (0)

// Each threshold is a 32-bit float, 
// so reading it is atomic
#define EEPROM_TempTh EEPROM_temp.TempThresholds

// Temporary symbol defined for debugging purposes while the static timer feature of FreeRTOS
// hasn't been imported into the codebase yet
#define TEMP_DYNAMIC_TIMERS

// Wraps safe procedure for setting overtemperature flag in EEPROM
#define EEPROM_SetOT(OT_var)                                            \
{                                                                       \
    SharedMem_BeginAccess();                                            \
    EEPROM_temp.beaconData.OT_var = 1;                                  \
    SharedMem_EndAccess();                                              \
    Periph_BeginTransact(DEV_EEPROM, EEPROM_FRAME_SIZE, HAL_MAX_DELAY); \
    EEPROM_Emul_SyncInfo();                                             \
    Periph_EndTransact(DEV_EEPROM);                                     \
}
/*
*~~~~~~
* INTERNAL TYPES
*~~~~~~
*/
typedef enum {
    ERROR_TYPE_WARN,
    ERROR_TYPE_ERROR,
    ERROR_TYPE_CRITICAL
} CompCheckErrorType;

typedef enum {
    S_NOMINAL =  0x0,
    S_OUTER_LO = 0x1,
    S_INNER_LO = 0x2,
    S_INNER_HI = 0x4,
    S_OUTER_HI = 0x8,
} ThresholdStatus_e;

typedef enum {
    CHECK_TEMP = 0,
    CHECK_GGB,
    CHECK_TCV,
    CHECK_SS,
    CHECK_ANT,
    CHECK_INFO_DUMP,
    CHECK_BEACON,
    NUM_CHECK_ROUTINES,
} CompChecks_e;

typedef struct {
#ifndef TEMP_DEBUG_DYNAMIC_TIMERS
    StaticTimer_t buf;
#endif
    TimerHandle_t handle;
} CheckerTimer_t;

typedef union {
    struct {
        float OBC_MCU;
        float EPS_BATT_1;
        float EPS_BATT_2;
        float SS;
        float TCV;
        float CRD_RTD_1;
        float CRD_RTD_2;
    };
    float temps[7];
} InfoDump_TempCache_t;

typedef enum {
    ANTDEPL_SUCCESS = 0x1,
    ANTDEPL_FAIL = 0x2,
    ANTDEPL_CANCEL = 0x3,
} AntDepl_Status_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* CHECKER ROUTINE DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void TemperatureChecker(TimerHandle_t timer);
void GGBChecker(TimerHandle_t timer);
void TCVChecker(TimerHandle_t timer);
void SSChecker(TimerHandle_t timer);
void ANTChecker(TimerHandle_t timer);
void INFO_dump(TimerHandle_t timer);
void RadioBeaconHelper(TimerHandle_t timer);
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Internal Variables
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static CheckerTimer_t checkerTimers[NUM_CHECK_ROUTINES];

static const TimerCallbackFunction_t checkerFuncts[NUM_CHECK_ROUTINES] = {
    [CHECK_TEMP] = TemperatureChecker,
    [CHECK_GGB] = GGBChecker,
    [CHECK_TCV] = TCVChecker,
    [CHECK_SS] = SSChecker,
    [CHECK_ANT] = ANTChecker,
    [CHECK_INFO_DUMP] = INFO_dump,
    [CHECK_BEACON] = RadioBeaconHelper,
};

static const TickType_t checkerPeriods[NUM_CHECK_ROUTINES] = {
    [CHECK_TEMP] = pdMS_TO_TICKS(TEMPERATURE_CHECK_PERIOD),
    [CHECK_GGB] = pdMS_TO_TICKS(GGB_CHECK_PERIOD), 
    [CHECK_TCV] = pdMS_TO_TICKS(TCV_CHECK_PERIOD), 
    [CHECK_SS] = pdMS_TO_TICKS(SS_CHECK_PERIOD), 
    [CHECK_ANT] = pdMS_TO_TICKS(ANT_CHECK_NORMAL_PERIOD), 
    [CHECK_INFO_DUMP] = pdMS_TO_TICKS(INFO_DUMP_PERIOD), 
    [CHECK_BEACON] = pdMS_TO_TICKS(RADIO_BEACON_PERIOD),
};

static InfoDump_TempCache_t InfoDump_TempCache;

static char S_beaconMsg[TCV_MAX_BEACON_LEN] = { 0 };

// Whether or not the beacon update should be running 
// depends partially on radio controller, 
// but this is impossible for CompChecks_X_Refresh to determine,
// so store enable flag here.
// This flag is updated by `CompCheck_X_Beacon_Enable_Callback'.
static uint8_t S_beaconEN = 1;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL HELPER ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static void enableChecker(CompChecks_e checker)
{
    if (checkerTimers[checker].handle == NULL) checkerTimers[checker].handle =
#ifndef TEMP_DYNAMIC_TIMERS
        xTimerCreateStatic(
#else
        xTimerCreate(
#endif
            DEFAULT_TIMER_NAME, checkerPeriods[checker],
            pdTRUE, DEFAULT_TIMER_ID,
            checkerFuncts[checker]
#ifndef TEMP_DYNAMIC_TIMERS
            , &(checkerTimers[checker].buf)
#endif
        );
}

static void disableChecker(CompChecks_e checker)
{
    if (checkerTimers[checker].handle != NULL)
    {
        // TODO: for now setting the timer command delay to max delay.
        // But probably shouldn't keep it that way because this stuff is executed by SysMan,
        // and it's not nice to have it stall to wait for the timer command queue?
        xTimerDelete(checkerTimers[checker].handle, portMAX_DELAY);
        checkerTimers[checker].handle = NULL;
    }
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INITIALIZATION ROUTINE
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Initialize timer intervals for component checks
void CompCheck_X_Init(void)
{
    memset(&(InfoDump_TempCache.temps), 0, sizeof(InfoDump_TempCache));
    CompCheck_X_Refresh(); 
}
//

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* CALLBACKS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// For each checker routine, decide whether it should be running right now
// based on device status from SysMan.
// Then enable/disable the routine as appropriate.
void CompCheck_X_Refresh()
{
    // Temperature checker should always be enabled
    enableChecker(CHECK_TEMP);

    if (Sys_IsDevAvail(DEV_GGB_CTRL)) enableChecker(CHECK_GGB);
    else disableChecker(CHECK_GGB);

    if (Sys_IsDevAvail(DEV_TCV)) enableChecker(CHECK_TCV);
    else disableChecker(CHECK_TCV);

    if (Sys_IsDevAvail(DEV_SS)) enableChecker(CHECK_SS);
    else disableChecker(CHECK_SS);

    // ANT checker communicates with TCV not ANT since ANT isn't connected to the OBC
    if (Sys_IsDevAvail(DEV_TCV) 
        && EEPROM_temp.AntUHFDeploy.attempts < ANT_DEPLOY_MAX_ATTEMPTS
        && !EEPROM_temp.AntUHFDeploy.success)
        enableChecker(CHECK_ANT);
    else disableChecker(CHECK_ANT);

    // INFO dump should always be enabled
    enableChecker(CHECK_INFO_DUMP);

    if (Sys_IsDevAvail(DEV_TCV) && S_beaconEN) enableChecker(CHECK_BEACON);
    else disableChecker(CHECK_BEACON);
}

void CompCheck_X_Beacon_Enable_Callback(uint8_t beaconEnable)
{
    S_beaconEN = beaconEnable;
    CompCheck_X_Refresh();
}

void CompCheck_X_Beacon_ESLPClear_Callback()
{
    SharedMem_BeginAccess();
    memset(&(EEPROM_temp.beaconData), 0, sizeof(EEPROM_temp.beaconData));
    SharedMem_EndAccess();
    EEPROM_Emul_SyncInfo();
}

const char* CompCheck_X_Beacon_Get()
{
    return S_beaconMsg;
}
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* CHECKER ROUTINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/


// Evaluates val against th
static ThresholdStatus_e checkThreshold(const Threshold_t* th, float val)
{
    if (val < th->min_outer) return S_OUTER_LO;
    else if (th->max_outer < val) return S_OUTER_HI;
    else if (val < th->min_inner) return S_INNER_LO;
    else if(th->max_inner < val) return S_INNER_HI;
    else return S_NOMINAL;
}

// Write a warning or critical log (specified by `log') for device `dev' with threshold status `stat'
static void temperatureLog(DevName_e dev, ThresholdStatus_e stat, Log_Type_e log)
{
    uint8_t payload[2];
    payload[0] = dev; payload[1] = stat;
    // TODO update log writing error checking
    if (log == LOG_WARNING) Log_WriteWarn(WARN_ENTRY_TEMPERATURE, payload, sizeof(payload));
    else if (log == LOG_CRITICAL) Log_WriteCrit(CRIT_ENTRY_TEMPERATURE, payload, sizeof(payload));
}


void TemperatureChecker(TimerHandle_t timer)
{
    // CPU TEMP CHECK
    do
    {
        float temp;
        status_t RTD_stat;
        Sys_TryDev(RTD_stat, GetCpuTemperature(&temp), SEN_SUCCESS, DEV_OBC_MCU_RTD);
        if (RTD_stat == SEN_SUCCESS) 
        {
            InfoDump_TempCache.OBC_MCU = temp;
            ThresholdStatus_e th_stat = checkThreshold(&EEPROM_TempTh.CPU, temp);
            switch (th_stat)
            {
            case S_INNER_LO:
            case S_INNER_HI:
                temperatureLog(DEV_OBC_MCU_RTD, th_stat, LOG_WARNING);
                break;
            case S_OUTER_HI:
                EEPROM_SetOT(OBC_OT);
                temperatureLog(DEV_OBC_MCU_RTD, th_stat, LOG_CRITICAL);
                Sys_X_CritTemperatureCB();
                continue;
            case S_OUTER_LO:
                temperatureLog(DEV_OBC_MCU_RTD, th_stat, LOG_CRITICAL);
                break;
            case S_NOMINAL:
                break;
            }
        }
    } while (0);

    // MTM CHECK
    for (DevName_e mtm = DEV_MTM_LO; mtm == DEV_MTM_LO || mtm == DEV_MTM_HI; mtm = DEV_MTM_HI)
    {
        // Fetch Mtm temperature
        // Assuming MTM method puts the low byte first then the high byte,
        // but this is not documented and must be tested! TODO
        uint8_t temp_buf[2]; int16_t temp;
        status_t result;
        uint8_t MTM_addr = mtm == DEV_MTM_LO ? LIS3MDL_MAG_I2C_ADDRESS_LOW : LIS3MDL_MAG_I2C_ADDRESS_HIGH;
        Sys_TryDev(result, LIS3MDL_MAG_Get_Temperature(MTM_addr, temp_buf), SEN_SUCCESS, mtm);
        if (result == SEN_SUCCESS) {
            temp |= temp_buf[1] << 8; // High byte
            temp |= temp_buf[0]; // Low byte
            ThresholdStatus_e th_stat = checkThreshold(&EEPROM_TempTh.MTM, (float)temp);
            if (th_stat == S_OUTER_HI || th_stat == S_OUTER_LO)
            {
                temperatureLog(mtm, th_stat, LOG_CRITICAL);
                Sys_X_SetPowerLevelLim(1, L2, 10);
            }
            else if (th_stat == S_INNER_HI || th_stat == S_INNER_LO)
                temperatureLog(mtm, th_stat, LOG_WARNING);
        }
    }

    // EPS CHECK
    do {
        EPS_TemperatureType EPS_Temps;
        HAL_StatusTypeDef retstat;
        Sys_TryDev(retstat, EPS_R_Temps(&EPS_Temps), HAL_OK, DEV_EPS);
        if (retstat == HAL_OK)
        {
            InfoDump_TempCache.EPS_BATT_1 = EPS_Temps.Batt[0]; InfoDump_TempCache.EPS_BATT_2 = EPS_Temps.Batt[1];
            ThresholdStatus_e mcu = checkThreshold(&(EEPROM_TempTh.EPS_MCU), EPS_Temps.MCU);
            switch (mcu)
            {
            case S_OUTER_HI:
                EEPROM_SetOT(EPS_OT1);
                temperatureLog(DEV_EPS, mcu, LOG_CRITICAL);
                Sys_X_CritTemperatureCB();
                continue;
            case S_OUTER_LO:
                temperatureLog(DEV_EPS, mcu, LOG_CRITICAL);
                break;
            case S_INNER_LO:
            case S_INNER_HI:
                temperatureLog(DEV_EPS, mcu, LOG_WARNING);
                break;
            case S_NOMINAL:
                break;
            }
            ThresholdStatus_e batt1 = checkThreshold(&(EEPROM_TempTh.EPS_Batt), EPS_Temps.Batt[0]);
            ThresholdStatus_e batt2 = checkThreshold(&(EEPROM_TempTh.EPS_Batt), EPS_Temps.Batt[1]);
            // Use the highest severity status for both batteries
            if (batt1 == S_OUTER_HI || batt2 == S_OUTER_HI)
            {
                EEPROM_SetOT(EPS_OT1);
                if (batt1 == S_OUTER_HI) temperatureLog(DEV_EPS_BATT_1, S_OUTER_HI, LOG_CRITICAL);
                if (batt2 == S_OUTER_HI) temperatureLog(DEV_EPS_BATT_2, S_OUTER_HI, LOG_CRITICAL);
                Sys_X_CritTemperatureCB();
                continue;
            }
            else if (batt0 == S_OUTER_LO || batt1 == S_OUTER_LO)
            {
                if (batt1 == S_OUTER_LO) temperatureLog(DEV_EPS_BATT_1, S_OUTER_LO, LOG_CRITICAL);
                if (batt2 == S_OUTER_LO) temperatureLog(DEV_EPS_BATT_2, S_OUTER_LO, LOG_CRITICAL);
            }
            else if (batt0 != S_NOMINAL || batt1 != S_NOMINAL)
            {
                if (batt1 != S_NOMINAL) temperatureLog(DEV_EPS_BATT_1, batt1, LOG_WARNING);
                if (batt2 != S_NOMINAL) temperatureLog(DEV_EPS_BATT_2, batt2, LOG_WARNING);
            }
        }
    } while (0);

    // TCV TEMP CHECK
    float TCV_temp;
    HAL_StatusTypeDef TCV_retstat;
    Sys_TryDev(TCV_retstat, TCV_GetTemp(&TCV_temp), HAL_OK, DEV_TCV);
    if (retstat == HAL_OK) 
    {
        InfoDump_TempCache.TCV = TCV_temp;
        ThresholdStatus_e th_stat = checkThreshold(&(EEPROM_TempTh.TCV), TCV_temp);
        switch (th_stat)
        {
        case S_OUTER_HI:
            EEPROM_SetOT(TCV_OT);
        case S_OUTER_LO:
            temperatureLog(DEV_TCV, th_stat, LOG_CRITICAL);
            Sys_X_SetPowerLevelLim(1, L1, 10);
            break;
        case S_INNER_HI:
        case S_INNER_LO:
            temperatureLog(DEV_TCV, th_stat, LOG_WARNING);
            break;
        case S_NOMINAL:
            break;
        }
    }

    // SS TEMP CHECK
    int16_t SS_temp;
    // TODO update SS error type
    CmdRet_t status;
    Sys_TryDev(status, SS_GetTemp(&SS_temp), HAL_OK, DEV_SS);
    if (status == HAL_OK) 
    {
        InfoDump_TempCache.SS = SS_temp;
        ThresholdStatus_e th_stat = checkThreshold(&(EEPROM_TempTh.SS), SS_temp);
        if (th_stat == S_OUTER_HI || th_stat == S_OUTER_LO)
        {
            temperatureLog(DEV_SS, th_stat, LOG_CRITICAL);
            Sys_X_SetPowerLevelLim(1, L2, 10);
        }
        else if (th_stat == S_INNER_HI || th_stat == S_INNER_LO)
            temperatureLog(DEV_SS, th_stat, LOG_WARNING);
    }
   
    // CRD TEMP CHECK
    float temp1, temp2;
    HAL_StatusTypeDef status1, status2;
    Sys_TryDev(status1, RTD_GetTemp(RTD_1, &temp1), HAL_OK, DEV_RTD_1);
    if (status1 == HAL_OK)
    {
        InfoDump_TempCache.CRD_RTD_1 = temp1;
        ThresholdStatus_e th_stat = checkThreshold(&EEPROM_TempTh.CRD1, temp1);
        if (th_stat == S_OUTER_HI || th_stat == S_OUTER_LO)
            temperatureLog(DEV_RTD_1, th_stat, LOG_CRITICAL);
        else if (th_stat == S_INNER_HI || th_stat == S_INNER_LO)
            temperatureLog(DEV_RTD_1, th_stat, LOG_WARNING);
    }
    Sys_TryDev(status2, RTD_GetTemp(RTD_2, &temp2), HAL_OK, DEV_RTD_2);
    if (status2 == HAL_OK)
    {
        InfoDump_TempCache.CRD_RTD_2 = temp2;
        ThresholdStatus_e th_stat = checkThreshold(&EEPROM_TempTh.CRD2, temp2);
        switch (th_stat)
        {
        case S_OUTER_HI:
            Sys_X_SetPowerLevelLim(1, L3, 30);
        case S_OUTER_LO:
            temperatureLog(DEV_RTD_2, th_stat, LOG_CRITICAL);
            break;
        case S_INNER_LO:
        case S_INNER_HI:
            temperatureLog(DEV_RTD_2, th_stat, LOG_WARNING);
            break;
        case S_NOMINAL:
            break;
        }
    }
}

void GGBChecker(TimerHandle_t timer)
{
    uint8_t val;
    HAL_StatusTypeDef ret; 
    Sys_TryDev(ret, GGB_read_register(GGB_ID, &val), HAL_OK, DEV_GGB_CTRL);
    // Don't check exactly what the GGB returns - just make sure it responds and there's no interface error
    }

void TCVChecker(TimerHandle_t timer)
{
    TCV_Err_t err; HAL_StatusTypeDef ret;
    do
    {
        Sys_TryDev(ret, TCV_ReadErrorStat(&err), HAL_OK, DEV_TCV);
        if (ret != HAL_OK) return;
            
        // TCV is a special snowflake -
        // if it shows any of its errors,
        // we pack them into the SysMan error raise in the high portions of the int,
        // where they won't collide with the HAL_StatusTypeDef error codes
        uint32_t errorStat = err.HFXT << 16 | err.FRAM << 17 | err.RFTS << 18;
        if (errorStat != 0) Sys_RaiseDevError(DEV_TCV, errorStat);
        // After SysMan has dealt with the error, try reading the TCV again to see if it was solved.
        // Else if there was no error, return.
        else return;
    } while Sys_IsDevAvail(DEV_TCV);
    }

void SSChecker(TimerHandle_t timer)
{
    SS_Status_t status;
    // TODO update SS error returning
    CmdRet_t ret = SS_GetStatus(&status);

    if(ret.HAL_ret != HAL_OK || ret.SS_ret != SS_OK){
        // Command delivery or execution error
        unsigned int errorNum = ret.HAL_ret || ret.SS_ret << 8;
        Sys_RaiseDevError(DEV_SS, errorNum);
    }
    if (status.error)
    {
        // Command went through fine but error log present on SS
    }
}

void ANTChecker(TimerHandle_t timer)
{
    if (EEPROM_temp.AntUHFDeploy.attempts >= ANT_DEPLOY_MAX_ATTEMPTS)
    {
        disableChecker(CHECK_ANT); return;
    }

    TCV_AntRelease_e relStat; HAL_StatusTypeDef TCV_stat;
    Sys_TryDev(relStat, TCV_GetAntRelease(&relStat), HAL_OK, DEV_TCV);
    if (TCV_stat == HAL_OK)
    {
        if (relStat == TCV_AntReleaseDisabled)
        {
            EEPROM_temp.AntUHFDeploy.success = 1;
            EEPROM_temp.AntUHFDeploy.attempts = ANT_DEPLOY_MAX_ATTEMPTS;
            EEPROM_Emul_SyncInfo();
            uint8_t payload = ANTDEPL_SUCCESS;
            Log_WriteCrit(CRIT_ENTRY_ANTDEPL, &payload, sizeof(payload));
            disableChecker(CHECK_ANT);
        }
        else
        {
            EEPROM_temp.AntUHFDeploy.attempts++;
            EEPROM_Emul_SyncInfo();
            if (EEPROM_temp.AntUHFDeploy.attempts >= ANT_DEPLOY_MAX_ATTEMPTS)
            {
                uint8_t payload = ANTDEPL_CANCEL;
                Log_WriteCrit(CRIT_ENTRY_ANTDEPL, &payload, sizeof(payload));
                disableChecker(CHECK_ANT);
            }
            else
            {
                Sys_TryDev(TCV_stat, TCV_SetAntRelease(relStat, TCV_SHORT_REL_PERIOD), HAL_OK, DEV_TCV);
                uint8_t payload = ANTDEPL_FAIL;
                Log_WriteCrit(CRIT_ENTRY_ANTDEPL, &payload, sizeof(payload));
                Sys_X_AntFailCallback();
            }
        }
    }
}

void INFO_dump(TimerHandle_t timer)
{
    uint8_t payload[49]; uint8_t payloadPtr = 0;
    
    memcpy(payload, &(InfoDump_TempCache.temps), sizeof(InfoDump_TempCache.temps)); payloadPtr += sizeof(InfoDump_TempCache.temps);
    
    EPS_VoltageType EPS_V; float SoC; HAL_StatusTypeDef EPS_stat;
    Sys_TryDev(EPS_stat, EPS_R_Voltages(&EPS_V), HAL_OK, DEV_EPS);
    if (EPS_stat == HAL_OK)
    {
        memcpy(payload + payloadPtr, &(EPS_V.V_Batt), sizeof(EPS_V.V_Batt)); payloadPtr += sizeof(EPS_V.V_Batt);
        SoC = PowerDist_GetSoC(EPS_V.V_Batt);
        memcpy(payload + payloadPtr, &SoC, sizeof(SoC)); payloadPtr += sizeof(SoC);
    }
    else
    {
        memset(payload + payloadPtr, 0, sizeof(EPS_V.V_Batt) + sizeof(SoC));
        payloadPtr += sizeof(EPS_V.V_Batt) + sizeof(SoC);
    }

    EPS_CurrentType EPS_I; 
    Sys_TryDev(EPS_stat, EPS_R_Currents(&EPS_I), HAL_OK, DEV_EPS);
    if (EPS_stat == HAL_OK) memcpy(payload + payloadPtr, &(EPS_I.I_Batt), sizeof(EPS_I.I_Batt));
    else memset(payload + payloadPtr, 0, sizeof(EPS_I.I_Batt));
    payloadPtr += sizeof(EPS_I.I_Batt);

    uint8_t heaterStat;
    Sys_TryDev(EPS_stat, EPS_R_Heaters(&heaterStat), HAL_OK, DEV_EPS);
    if (EPS_stat == HAL_OK) memcpy(payload + payloadPtr, &heaterStat, sizeof(heaterStat));
    else memset(payload + payloadPtr, 0, sizeof(heaterStat));
    payloadPtr += sizeof(heaterStat);

    float magBdot; Vec3D_t omega;
    if (ADCS_Meas_GetMagBdot(&magBdot) == E_OK)
    {
        memcpy(payload + payloadPtr, &magBdot, sizeof(magBdot)); payloadPtr += sizeof(magBdot);
        FRESULT TRIAD_stat = FR_DISK_ERR;
        do
        {
            HAL_StatusTypeDef RTC_stat; time_t t;
            TRIAD_VecPair_t M0, M1; uint16_t intv_ms;
            Sys_TryDev(RTC_stat, RTC_GetTime(&t), HAL_OK, DEV_RTC);
            if (RTC_stat == HAL_OK
                && ADCS_Meas_GetS(&(M0.S), &(M1.S), &intv_ms) == E_OK
                && ADCS_Meas_GetB(&(M0.B), &(M1.B), NULL) == E_OK)
            {
                Sys_TryDev(TRIAD_stat, TRIAD_GetOmegaVec(&M0, &M1, &t, intv_ms, &omega), FR_OK, DEV_SD);
            }
        } while (0);
        if (TRIAD_stat == FR_OK)
        {
            memcpy(payload + payloadPtr, &omega, sizeof(omega));
            payloadPtr += sizeof(omega);
        }
        else
        {
            memset(payload + payloadPtr, 0, sizeof(omega));
            payloadPtr += sizeof(omega);
        }
    }
    else
    {
        memset(payload + payloadPtr, 0, sizeof(magBdot) + sizeof(omega));
        payloadPtr += sizeof(magBdot) + sizeof(omega);
    }

    Log_WriteInfo(INFO_ENTRY_COMPCHECKS, payload, payloadPtr);
}

void RadioBeaconHelper(TimerHandle_t timer)
{
    uint16_t bootCt = 0;
    uint32_t bootCt_l = EEPROM_GetBootCount();
    bootCt = bootCt_l <= UINT16_MAX ? bootCt_l : UINT16_MAX;

    uint16_t faults = 0;
    // Constant order of faults in the beacon message template,
    // from low to high bit.
    static const DevName_e devOrder[] = { DEV_MTM_LO, DEV_MTM_HI, DEV_RTC, DEV_SD, 
                                          DEV_EPS, DEV_MUX, DEV_CTR_0, DEV_DAC_1, 
                                          DEV_RTD_1, DEV_GGB_CTRL, DEV_SS };
    for (int i = 0; i < sizeof(devOrder) / sizeof(devOrder[0]); i++)
    {
        uint8_t inFault = 0;
        switch (devOrder[i])
        {
        case DEV_CTR_0:
        {
            // We aggregate all counter errors into one bit
            for (DevName_e ctr = DEV_CTR_0; ctr <= DEV_CTR_7; ctr++)
            {
                if (Sys_GetDevStatus(ctr) == DEV_STAT_FAULT) {inFault = 1; break;}
            }
            break;
        }
        case DEV_DAC_1:
        {
            // We aggregate all DAC errors into one bit
            for (DevName_e dac = DEV_DAC_1; dac <= DEV_DAC_2; dac++)
            {
                if (Sys_GetDevStatus(dac) == DEV_STAT_FAULT) { inFault = 1; break; }
            }
            break;
        }
        case DEV_RTD_1:
        {
            // We aggregate all RTD errors into one bit
            for (DevName_e rtd = DEV_RTD_1; rtd <= DEV_RTD_2; rtd++)
            {
                if (Sys_GetDevStatus(rtd) == DEV_STAT_FAULT) { inFault = 1; break; }
            }
            break;
        }
        default:
        {
            inFault = Sys_GetDevStatus(devOrder[i]) == DEV_STAT_FAULT;
            break;
        }
        }
        faults |= inFault << i;
    }
    // ANT fault flag is not based on SysMan information
    SharedMem_BeginAccess();
    uint8_t ANT_fault = (!EEPROM_temp.AntUHFDeploy.success && EEPROM_temp.AntUHFDeploy.attempts > 0);
    SharedMem_EndAccess();
    faults |= ANT_fault << 11;

    uint8_t OBC_stat = 0;
    SharedMem_BeginAccess();
    OBC_stat |= EEPROM_temp.beaconData.OBC_SoftRst  << 0;
    OBC_stat |= EEPROM_temp.beaconData.OBC_PowRst   << 1;
    OBC_stat |= EEPROM_temp.beaconData.OBC_UseRst   << 2;
    OBC_stat |= EEPROM_temp.beaconData.OBC_HardRst  << 3;
    OBC_stat |= EEPROM_temp.beaconData.OBC_OT       << 4;
    SharedMem_EndAccess();
    TimeSync_e t_sync; HAL_StatusTypeDef RTC_stat;
    Sys_TryDev(RTC_stat, RTC_GetTimeSync(&t_sync), HAL_OK, DEV_RTC);
    if (RTC_stat == HAL_OK && t_sync == TIME_SYNCED) OBC_stat |= 1 << 7;

    uint8_t EPS_stat = 0;
    SharedMem_BeginAccess();
    EPS_stat |= EEPROM_temp.beaconData.EPS_UV   << 0;
    EPS_stat |= EEPROM_temp.beaconData.EPS_SC   << 1;
    EPS_stat |= EEPROM_temp.beaconData.EPS_OT2  << 2;
    EPS_stat |= EEPROM_temp.beaconData.EPS_OT1  << 3;
    SharedMem_EndAccess();

    uint8_t UHF_stat = 0;
    SharedMem_BeginAccess();
    UHF_stat |= EEPROM_temp.beaconData.TCV_HFXT     << 0;
    UHF_stat |= EEPROM_temp.beaconData.TCV_FRAM     << 1;
    UHF_stat |= EEPROM_temp.beaconData.TCV_RFTS     << 2;
    UHF_stat |= EEPROM_temp.beaconData.TCV_OT       << 3;
    uint8_t ant_stat = 0;
    if (EEPROM_temp.AntUHFDeploy.success) ant_stat = 0b11;
    else
    {
        if (EEPROM_temp.AntUHFDeploy.attempts < ANT_DEPLOY_MAX_ATTEMPTS) ant_stat = 0b00;
        else ant_stat = 0b01;
    }
    UHF_stat |= ant_stat << 4;
    SharedMem_EndAccess();

    uint8_t ADCS_stat;
    SharedMem_BeginAccess();
    ADCS_stat |= EEPROM_temp.detumbState.status             << 0;
    ADCS_stat |= (EEPROM_temp.detumbState.enabled == 1)     << 2;
    ADCS_stat |= EEPROM_temp.GGB_state.status               << 3;
    ADCS_stat |= (EEPROM_temp.GGB_state.enabled == 1)       << 5;
    SharedMem_EndAccess();
    ADCS_stat |= (PowerDist_getPhase() == PHASE_SUN)        << 6;
    ADCS_stat |= (ADCS_Meas_GetSunLastLight() == 1)         << 7;

    uint8_t SoC = 0;
    EPS_VoltageType EPS_V; HAL_StatusTypeDef EPS_stat; 
    Sys_TryDev(EPS_stat, EPS_R_Voltages(&EPS_V), HAL_OK, DEV_EPS);
    if (EPS_stat == HAL_OK) SoC = roundf(PowerDist_GetSoC(EPS_V.V_Batt));

    Sys_PowerLevel_e pwrLvl = Sys_GetPowerLevel();

    uint8_t EPS_avg_temp = roundf((InfoDump_TempCache.EPS_BATT_1 + InfoDump_TempCache.EPS_BATT_2) / 2);

    uint8_t OBC_temp = roundf(InfoDump_TempCache.OBC_MCU);

    uint8_t UHF_temp = roundf(InfoDump_TempCache.TCV);

    uint8_t CRD_RTD_avg_temp = roundf((InfoDump_TempCache.CRD_RTD_1 + InfoDump_TempCache.CRD_RTD_2) / 2);

    Log_Counts_t logCts;
    uint16_t InfoCt = 0; uint8_t WarnCt = 0, CritCt = 0;
    FRESULT fr;
    Sys_TryDev(fr, Log_GetLogCounts(&logCts), FR_OK, DEV_SD);
    if (fr == FR_OK)
    {
        InfoCt = logCts[LOG_INFO] < UINT16_MAX ? logCts[LOG_INFO] : UINT16_MAX;
        WarnCt = logCts[LOG_WARNING] < UINT8_MAX ? logCts[LOG_WARNING] : UINT8_MAX;
        CritCt = logCts[LOG_CRITICAL] < UINT8_MAX ? logCts[LOG_CRITICAL] : UINT8_MAX;
    }
    
    uint8_t GGB_ext = roundf(EEPROM_temp.GGB_ex_params.extCurrent * 10);

    SharedMem_BeginAccess();
    int msgPtr = 0;
    msgPtr += sprintf(S_beaconMsg + msgPtr, "BLAST%.4hX", bootCt);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.4hX", faults);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhX", OBC_stat);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhX", EPS_stat);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhX", UHF_stat);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhX", ADCS_stat);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhu", SoC);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.1d", pwrLvl);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%+.3hhu", EPS_avg_temp);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%+.3hhu", OBC_temp);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%+.3hhu", UHF_temp);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%+.3hhu", CRD_RTD_avg_temp);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.4hu", InfoCt);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhu", WarnCt);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhu", CritCt);
    msgPtr += sprintf(S_beaconMsg + msgPtr, "%.2hhu", GGB_ext);
    SharedMem_EndAccess();
    debug_assert(msgPtr <= TCV_MAX_BEACON_LEN);

    HAL_StatusTypeDef TCV_stat;
    Sys_TryDev(TCV_stat, TCV_SetBeaconContent(S_beaconMsg), HAL_OK, DEV_TCV);
}
