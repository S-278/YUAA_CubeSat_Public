/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file Memory_Logs.h
* @brief Memory Logs Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah L., Jeb C. 
* @version           1.0.0
* @date              2022.11.22
*
* @details           Declares memory log routines for use by higher-level code
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "SD_Routines.h"
#include "Svc_RTC.h"
#include "ff.h"
#include <stdint.h>
#include <time.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Maximum number of bytes in one queue ticket submitted to Log Writer
// Currently the longest possible entry is a component checks dump
#define LOG_ENTRY_MAX_SZ        (17+49+2)

#define LOG_NUM_FILENAME 	 	("lognums")
#define LOG_NUM_PATH            SD_ROOT

#define INFO_LOG_FILENAME    	("info_log")
#define INFO_LOG_PATH           SD_ROOT

#define WARNING_LOG_FILENAME	("warn_log")
#define WARNING_LOG_PATH        SD_ROOT

#define CRITICAL_LOG_FILENAME	("crit_log")
#define CRITICAL_LOG_PATH       SD_ROOT

#define BOOT_LOG_FILENAME       ("boot_log")
#define BOOT_LOG_PATH           SD_ROOT

#define CRD_LOG_FILENAME		("crd_log")
#define CRD_LOG_PATH            SD_ROOT
#define CRD_BIN_NUM (8) // Number of event bins on the CRD

#define PASSTIME_LOG_FILENAME   ("pass_times")
#define PASSTIME_LOG_PATH       SD_ROOT

#if defined(DEBUG_ENABLED) && !defined(DEBUG_SD)
/* This inclusion intercepts usage of items from ff.h 
*  with mock items from ff_Mock.h */
#include "ff_Mock.h"
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

typedef enum {
    LOG_INFO = 0,
    LOG_WARN,
    LOG_CRIT,
    LOG_BOOT,
    LOG_CRD,
    LOG_TYPES_NUM
} Log_Type_e;

typedef uint32_t Log_Counts_t[LOG_TYPES_NUM]; // Holds count of each type of log in the same order as in Log_Type_e

typedef enum {
    INFO_ENTRY_COMPCHECKS = 0x01,
    INFO_ENTRY_PWRLVL = 0x10,
    INFO_ENTRY_GNDPASS = 0x20,
} InfoEntryTypes_e;

typedef enum {
    WARN_ENTRY_TEMPERATURE = 0x02,
    WARN_ENTRY_PWRLVL = 0x10,
    WARN_ENTRY_DEVWARN = 0x11,
    WARN_ENTRY_RADIOKILL = 0x25,
    WARN_ENTRY_RECMODE_TIMESET = 0x26,
    WARN_ENTRY_ADCS_STAT = 0x40,
    WARN_ENTRY_MEMINTEG = 0x50,
} WarnEntryTypes_e;

typedef enum {
    CRIT_ENTRY_TEMPERATURE = 0x02,
    CRIT_ENTRY_PWRLVL = 0x10,
    CRIT_ENTRY_DEVFAULT = 0x12,
    CRIT_ENTRY_SWRESET = 0x13,
    CRIT_ENTRY_RECMODE = 0x27,
    CRIT_ENTRY_ANTDEPL = 0x28,
    CRIT_ENTRY_EPSRST = 0x30,
} CritEntryTypes_e;

// Type used for CRD log entries on SD
typedef uint32_t CRD_Count_t[CRD_BIN_NUM];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write an INFO log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      entryType - type of INFO entry
* @param[input]      payload - pointer to block of binary data to use as entry payload
* @param[input]      payloadSize - size of payload in bytes
* @return            none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_WriteInfo(InfoEntryTypes_e entryType, const void* payload, uint16_t payloadSize);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write a WARNING log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      entryType - type of WARNING entry
* @param[input]      payload - pointer to block of binary data to use as entry payload
* @param[input]      payloadSize - size of payload in bytes
* @return            none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_WriteWarn(WarnEntryTypes_e entryType, const void* payload, uint16_t payloadSize);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write a CRITICAL log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      entryType - type of CRITICAL entry
* @param[input]      payload - pointer to block of binary data to use as entry payload
* @param[input]      payloadSize - size of payload in bytes
* @return            none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_WriteCrit(CritEntryTypes_e entryType, const void* payload, uint16_t payloadSize);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write a CRD_DATA log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      countData - pointer to bin count data
* @return            none
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_WriteCRD(const CRD_Count_t countData);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write a BOOT log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      resetReasonMask - mask indicating which reset reasons were recorded
* @return            HAL_StatusTypeDef indicating result of RTC interaction
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef Log_WriteBoot(uint16_t resetReasonMask);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get entry counts for all log types
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     result - pointer to Log_Counts_t to populate with counts
* @return            FRESULT containing result of SD comms
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_GetLogCounts(Log_Counts_t* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Find entries in a log within a time range
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      type - type of log to search
* @param[input]      startT - period start time
* @param[input]      endT - period end time
* @param[output]     offsetStart - pointer to FSIZE_t to populate with file offset of the 
*                    start of the first entry within the given period. Pass NULL if not 
*                    needed.
* @param[output]     offsetend - pointer to FSIZE_t to populate with file offset of the 
*                    start of the first entry not within the given period. Pass NULL if not 
*                    needed.
* @param[output]     totalBytes - pointer to uint32_t to populate with total size of all 
*                    entries found within the given period.
* @param[output]     fd - pointer to an uninitialized FIL. This function will open the file
*                    into this descriptor with read & write privileges. 
* @return            FRESULT containing result of SD comms.
*                    IMPORTANT - if this function returns FR_OK or FR_CORRUPT, 
*                    user must close file by calling f_close with `fd'.
*                    On any other return, the user must not attempt to close `fd'. 
*                    If this function returns FR_CORRUPT, 
*                    the values of `offsetStart' and `offsetEnd' will still point to 
*                    intact log entries.
* @note              See Radio Comms Protocol document for semantics of Get logs in range
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_FindEntryRange(Log_Type_e type, const time_t* startT, const time_t* endT, \
                           FSIZE_t* offsetStart, FSIZE_t* offsetEnd, uint32_t* totalBytes,\
                           FIL* fd);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Clear entries within a time range from a given log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      type - type of log to clear
* @param[input]      startT - period start time
* @param[input]      endT - period end time
* @param[output]     clearedBytes - pointer to populate with number of bytes freed
*                    as a result of the operation
* @return            FRESULT containing result of SD comms
* @detail            CAUTION! A large stack-allocated buffer is used to hold data
*                    while it is being moved around inside the file!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_ClearEntryRange(Log_Type_e type, const time_t* startT, const time_t* endT, 
    uint32_t* clearedBytes);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Delete oldest log entries to free space on SD
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            FRESULT containing result of SD comms
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_MakeSpace();

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Log Writer task function
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      arg - unused
* @return            none
* @note              Exclusively for SysMan
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_X_LogWriterTask(void* arg);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Reconstruct log times on on-board time sync
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 delta - difference between the unsynced and synced timebases
* @return            None
* @detail			 TODO: this is a long operation and should be delegated to the RTOS daemon
* @note              Exclusively for TimeUtils
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_X_TimeSyncCallback(time_t delta);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Delete outdated entries from pass times log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      currT - pointer to time_t containing current UTC time
* @return            FRESULT containing status of SD interface
* @note              Exclusively for Radio Controller
* @detail            Will delete all times in pass times log which are earlier than `currT' 
*                    by more than the passive pass window
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_X_CleanPassTime(const time_t* currT);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get pass time from pass times log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	 passT - populated with the last (earliest) pass time in the log file. 
*                    If the log file is empty, this value is set to 0.
* @return            FRESULT containing status of SD interface
* @note              Exclusively for Radio Controller
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_X_GetPassTime(time_t* passT);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get number of pass times recorded in log
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]	 num - number of pass times in log
*                    If the log file is empty, this value is set to 0.
* @return            FRESULT containing status of SD interface
* @note              Exclusively for Radio Controller and Telecommands Library
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Log_X_GetNumPassTimes(uint32_t* num);

// Testing
#ifdef DEBUG_ENABLED // Include AppTasks.h for code tests
#include "AppTasks.h"
#endif

#ifdef DEBUG_ENABLED
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TEST SET DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
DeclareTestSet(Memory_Logs_Benchmark_Set);

TestStatus_e Log_TestFixture_ClearAllLogs();

TestStatus_e Log_Test_BasicFormatting();

TestStatus_e Log_Test_BasicFatFS();

TestStatus_e Memory_Logs_Info_Test();

TestStatus_e Memory_Logs_Warn_Test();

TestStatus_e Memory_Logs_Crit_Test();

TestStatus_e Memory_Logs_Boot_Test();

TestStatus_e Memory_Logs_CRD_Test();

TestStatus_e Log_Test_LogCounts();

TestStatus_e Log_Test_Continuity();

TestStatus_e Memory_Logs_Log_FindEntryRange_All_UTC_Test();

TestStatus_e Memory_Logs_Log_FindEntryRange_All_OnBoardTime_Test();

TestStatus_e Memory_Logs_Log_FindEntryRange_MixedTime_Test();

TestStatus_e Memory_Logs_Log_ClearEntryRange_Test();

TestStatus_e Memory_Logs_Log_TimeSyncCallback_Test();

// TestStatus_e Memory_Logs_Log_FindEntryRange_Test();

// TestStatus_e Memory_Logs_Log_X_TimeSyncCallback_Test();

// TestStatus_e Memory_Logs_Log_X_ClearEntryRange_Test();

#endif
