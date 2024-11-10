/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TimeUtils.c
* @brief Time Utilities C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            John W. 
* @version           0.2.0
* @date              2023.06.13
*
* @details           Defines helpers for on-board timekeeping
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "TimeUtils.h"
//#include "Memory_Logs.h"
#include "PeriphHelper.h"
//#include "radio_controller.h"
#include "system_manager.h"
#include "main.h"
#include "MCU_init.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define RTC_TimeInit(rtc_time) {rtc_time.Hours = 0; \
                                rtc_time.Minutes = 0; \
                                rtc_time.Seconds = 0; \
                                rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE; \
                                rtc_time.StoreOperation = RTC_STOREOPERATION_RESET; }
#define RTC_DateInit(rtc_date) {rtc_date.Year = 0; \
                                rtc_date.Month = 0; \
                                rtc_date.Date = 0; \
                                rtc_date.WeekDay = 0; }
#define errorcheck(HAL_ret) if ((HAL_StatusTypeDef)HAL_ret != HAL_OK) return HAL_ret

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Enumerates all time sync callbacks.
// Time sync callbacks are called when RTC_SetTime is called
// to update all timestamps saved in memory.
typedef enum {
    /*TIMESYNC_CB_RADIOCTL,
    TIMESYNC_CB_LOG,
    TIMESYNC_CB_SYSMAN*/
    TIMESYNC_CB_NUM,
} TimeSyncCallbacks_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Array holding all time sync callbacks
static TimeSync_Callback_t timeSyncCBs[TIMESYNC_CB_NUM] = { \
        /*ADCS_X_TimeSyncCallback,
        RadioCtl_X_TimeSyncCallback,
        State_X_TimeSyncCallback,
        //Log_X_TimeSyncCallback*/
};

#if defined(DEBUG_ENABLED) && defined(TIMEUTILS_TEST_RTC_IRQ)
// Very not thread-safe mailbox for testing of RTC IRQ
volatile static int RTC_IRQ_mail = 0;
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Implementation by Aviv M. taken from Orbit Propagator
static time_t convertRTCtoTime(const RTC_TimeTypeDef* rtc_time, const RTC_DateTypeDef* rtc_date)
{
    struct tm time; // time struct built into time.h

    time.tm_year = (rtc_date->Year + 70); // Convert years since 1970 to years since 1900
    time.tm_mon = (rtc_date->Month - 1); // HAL uses month indexes 1-12, while c uses 0-11
    time.tm_mday = rtc_date->Date;
    time.tm_hour = rtc_time->Hours;
    time.tm_min = rtc_time->Minutes;
    time.tm_sec = rtc_time->Seconds;

    return mktime(&time);
}
static void convertTimeToRTCDate(const time_t* time_in, RTC_DateTypeDef* result) {
    struct tm *gm_ret;
    struct tm time_struct;

    // gmtime uses an internal buffer, which is not thread-safe
    SharedMem_BeginAccess();
    gm_ret = gmtime(time_in);
    memcpy(&time_struct, gm_ret, sizeof(struct tm));
    SharedMem_EndAccess();

    result->Year = (time_struct.tm_year - 70);
    result->Month = (time_struct.tm_mon + 1);
    result->Date = time_struct.tm_mday;
}
static void convertTimeToRTCTime(const time_t* time_in, RTC_TimeTypeDef* result) {
    struct tm* gm_ret;
    struct tm time_struct;

    // gmtime uses an internal buffer, which is not thread-safe
    SharedMem_BeginAccess();
    gm_ret = gmtime(time_in);
    memcpy(&time_struct, gm_ret, sizeof(struct tm));
    SharedMem_EndAccess();

    result->Hours = time_struct.tm_hour;
    result->Minutes = time_struct.tm_min;
    result->Seconds = time_struct.tm_sec;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

HAL_StatusTypeDef RTC_GetTimeSync(TimeSync_e* result)
{
    assert(result);
    time_t currTime;
    HAL_StatusTypeDef retstat = RTC_GetTime(&currTime); errorcheck(retstat);
    if (currTime >= TIME_SYNC_BOUNDARY) *result = TIME_SYNCED;
    else *result = TIME_NOT_SYNCED;
    return retstat;
}

HAL_StatusTypeDef RTC_GetTime(time_t* result)
{
    assert(result);

    RTC_TimeTypeDef rtc_time; RTC_TimeInit(rtc_time);
    RTC_DateTypeDef rtc_date; RTC_DateInit(rtc_date);
    HAL_StatusTypeDef retstat;

    Periph_BeginTransact(DEV_RTC, sizeof(rtc_time) + sizeof(rtc_date), HAL_MAX_DELAY);
    HAL_RTC_WaitForSynchro(&hrtc);
    retstat = HAL_RTC_GetTime(&hrtc, &rtc_time, calendar_format); errorcheck(retstat);
    retstat = HAL_RTC_GetDate(&hrtc, &rtc_date, calendar_format); errorcheck(retstat);
    Periph_EndTransact(DEV_RTC);

    *result = convertRTCtoTime(&rtc_time, &rtc_date);

    return retstat;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

HAL_StatusTypeDef RTC_X_SetTime(time_t newtime)
{
// First get time to record the delta
    time_t oldTime;
    HAL_StatusTypeDef retstat = RTC_GetTime(&oldTime); errorcheck(retstat);
    time_t delta = newtime - oldTime;
    
    // Convert time_t to RTC
    RTC_TimeTypeDef newTime; RTC_TimeInit(newTime);
    RTC_DateTypeDef newDate; RTC_DateInit(newDate);
    convertTimeToRTCTime(&newtime, &newTime);
    convertTimeToRTCDate(&newtime, &newDate);

    // Set time
    Periph_BeginTransact(DEV_RTC, sizeof(newTime) + sizeof(newDate), HAL_MAX_DELAY);
    retstat = HAL_RTC_SetDate(&hrtc, &newDate, calendar_format); errorcheck(retstat);
    retstat = HAL_RTC_SetTime(&hrtc, &newTime, calendar_format); errorcheck(retstat);
    Periph_EndTransact(DEV_RTC);

    // Call to callbacks for timestamp update
    for (int i = 0; i < TIMESYNC_CB_NUM; i++) (timeSyncCBs[i])(delta);
    
    return retstat;
}

HAL_StatusTypeDef RTC_X_SetAlarm(time_t alarmSetTime) {
    // Convert alarm time to RTC
    RTC_TimeTypeDef alarmTime; RTC_TimeInit(alarmTime);
    RTC_DateTypeDef alarmDate; RTC_DateInit(alarmDate);
    convertTimeToRTCTime(&alarmSetTime, &alarmTime);
    convertTimeToRTCDate(&alarmSetTime, &alarmDate);

    // Configure alarm
    RTC_AlarmTypeDef alarmConfig;
    RTC_TimeInit(alarmConfig.AlarmTime);
    alarmConfig.AlarmTime.Hours = alarmTime.Hours;
    alarmConfig.AlarmTime.Minutes = alarmTime.Minutes;
    alarmConfig.AlarmTime.Seconds = alarmTime.Seconds; 
    alarmConfig.AlarmMask = RTC_ALARMMASK_NONE;
    alarmConfig.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    alarmConfig.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    alarmConfig.AlarmDateWeekDay = alarmDate.Date;
    alarmConfig.Alarm = TIME_UTILS_ALARM;

    Periph_BeginTransact(DEV_RTC, sizeof(alarmConfig), HAL_MAX_DELAY);
    HAL_StatusTypeDef retstat = HAL_RTC_SetAlarm_IT(&hrtc, &alarmConfig, calendar_format);
    Periph_EndTransact(DEV_RTC);

    return retstat;
}

void RTC_Alarm_IRQHandler()
{
    // TODO insert call to radio controller callback
    HAL_RTC_AlarmIRQHandler(&hrtc);
#if defined(DEBUG_ENABLED) && defined(TIMEUTILS_TEST_RTC_IRQ)
    RTC_IRQ_mail = 1;
#endif
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TESTS DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DEBUG_ENABLED
static const Test_t TimeUtils_BenchmarkSetArray[] = {
        {.test = TimeUtils_Test_Conversions, .testName = "Time conversions"},
        {.test = TimeUtils_Test_GetTime, .testName = "Get time"},
        {.test = TimeUtils_Test_SetTime, .testName = "Set time"},
        {.test = TimeUtils_Test_TimeSyncThreshold, .testName = "Time sync threshold"},
#ifdef TIMEUTILS_TEST_RTC_IRQ
        {.test = TimeUtils_Test_RTCAlarm, .testName = "RTC alarm"},
#endif
};

DefineTestSet(TimeUtils_BenchmarkSet, TimeUtils, TimeUtils_BenchmarkSetArray);

// Currently the clocks on the dev boards are not set up correctly,
// which results in the HAL and FreeRTOS timebases being 2x slower than real time.
// This factor is used in this testing library to account for this as follows:
// elapsed time on RTC = elapsed real time = CORE_CLK_ADJ_FACTOR * elapsed time as tracked by either HAL or FreeRTOS
#define CORE_CLK_ADJ_FACTOR (2)

#include <stdlib.h>
#define assert_elapsed(measuredDuration, correctDuration) test_assert(llabs((measuredDuration) - (correctDuration)) <= 1)

TestStatus_e TimeUtils_Test_Conversions()
{
    RTC_TimeTypeDef myRTCtime = {
        .Hours = 13,
        .Minutes = 0,
        .Seconds = 13,
    };
    RTC_DateTypeDef myRTCdate = {
        .Month = RTC_MONTH_JANUARY,
        .Date = 14,
        .Year = 2024 - 1970,
    };
    time_t t = convertRTCtoTime(&myRTCtime, &myRTCdate);
    test_assert(t == 1705237213); // Reference value taken from https://unixtime.org/ for Jan 14th 2024, GMT 13:00:13 

    RTC_TimeTypeDef yourTime; RTC_DateTypeDef yourDate;
    t = 1705237213;
    convertTimeToRTCDate(&t, &yourDate);
    test_assert(yourDate.Month == RTC_MONTH_JANUARY &&  \
                yourDate.Date == 14 &&                  \
                yourDate.Year == 2024 - 1970);
    convertTimeToRTCTime(&t, &yourTime);
    test_assert(yourTime.Hours == 13 &&         \
                yourTime.Minutes == 0 &&        \
                yourTime.Seconds == 13);

    return TEST_PASS;
}

TestStatus_e TimeUtils_Test_GetTime()
{
    time_t myTime;
    HAL_StatusTypeDef retstat;
    retstat = RTC_GetTime(&myTime);
    test_assert(retstat == HAL_OK);

    HAL_Delay(10000);

    time_t mySecondTime;
    retstat = RTC_GetTime(&mySecondTime);
    test_assert(retstat == HAL_OK);
    test_assert(mySecondTime > myTime);
    assert_elapsed(mySecondTime - myTime, CORE_CLK_ADJ_FACTOR * 10);

    return TEST_PASS;
}

TestStatus_e TimeUtils_TestFixture_ZeroRTC()
{
    HAL_StatusTypeDef retstat;
    retstat = RTC_X_SetTime(0);
    test_assert(retstat == HAL_OK);
    time_t myTime;
    retstat = RTC_GetTime(&myTime);
    test_assert(retstat == HAL_OK);
    test_assert(0<= myTime && myTime <= 1);
    return TEST_PASS;
}

TestStatus_e TimeUtils_Test_SetTime()
{
    HAL_StatusTypeDef retstat;
    time_t myTime;
    
    use_test_fixture(TimeUtils_TestFixture_ZeroRTC);

    retstat = RTC_X_SetTime(TIME_SYNC_BOUNDARY);
    test_assert(retstat == HAL_OK);
    retstat = RTC_GetTime(&myTime);
    test_assert(retstat == HAL_OK);
    test_assert(myTime == TIME_SYNC_BOUNDARY || myTime == TIME_SYNC_BOUNDARY + 1);

    HAL_Delay(10000);
    retstat = RTC_GetTime(&myTime);
    test_assert(retstat == HAL_OK);
    assert_elapsed(myTime - TIME_SYNC_BOUNDARY, CORE_CLK_ADJ_FACTOR * 10);

    return TEST_PASS;
}

TestStatus_e TimeUtils_Test_TimeSyncThreshold()
{
    use_test_fixture(TimeUtils_TestFixture_ZeroRTC);

    HAL_StatusTypeDef retstat;
    TimeSync_e syncStat = 42;
    retstat = RTC_GetTimeSync(&syncStat);
    test_assert(retstat == HAL_OK);
    test_assert(syncStat == TIME_NOT_SYNCED);

    retstat = RTC_X_SetTime(TIME_SYNC_BOUNDARY);
    test_assert(retstat == HAL_OK);
    retstat = RTC_GetTimeSync(&syncStat);
    test_assert(retstat == HAL_OK);
    test_assert(syncStat == TIME_SYNCED);

    retstat = RTC_X_SetTime(TIME_SYNC_BOUNDARY + (10*365*24*60*60));
    test_assert(retstat == HAL_OK);
    retstat = RTC_GetTimeSync(&syncStat);
    test_assert(retstat == HAL_OK);
    test_assert(syncStat == TIME_SYNCED);

    return TEST_PASS;
}

#ifdef TIMEUTILS_TEST_RTC_IRQ
TestStatus_e TimeUtils_Test_RTCAlarm()
{
    // First test software interrupt activation
    RTC_IRQ_mail = 0;
    HAL_NVIC_SetPendingIRQ(RTC_Alarm_IRQn);
    test_assert(RTC_IRQ_mail == 1);
    test_assert(HAL_NVIC_GetPendingIRQ(RTC_Alarm_IRQn) == 0);
    RTC_IRQ_mail = 0;

    // Real RTC alarm
    use_test_fixture(TimeUtils_TestFixture_ZeroRTC);
    HAL_StatusTypeDef retstat;
    retstat = RTC_X_SetAlarm(10);
    test_assert(retstat == HAL_OK);
    HAL_Delay(9000 / CORE_CLK_ADJ_FACTOR);
    // Check that alarm doesn't fire too early
    test_assert(RTC_IRQ_mail == 0);
    HAL_Delay(2000 / CORE_CLK_ADJ_FACTOR);
    // By this time alarm should have fired
    test_assert(RTC_IRQ_mail == 1);

    return TEST_PASS;
}
#endif

#endif
