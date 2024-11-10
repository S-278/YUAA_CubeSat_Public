/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file Memory_Logs.c
* @brief Memory Logs C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah L., Jeb C. 
* @version           1.0.0
* @date              2022.09.10
*
* @details           Provides memory log routines for higher-level code
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "Memory_Logs.h"
#include "SD_Routines.h"
#include "system_manager.h"
#include "TimeUtils.h"
#include "EEPROM_emul.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Svc_RTC.h"
#include <string.h>
#ifdef DEBUG_ENABLED
#include <stdio.h>
#include <stdlib.h>
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EEPROM_GetBootCount intercept for mock in debug mode and return EEPROM_BOOT_COUNT_MOCK value
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DEBUG_ENABLED
#define EEPROM_GetBootCount EEPROM_GetBootCount_Mock

/* use this variable in tests to alter result of EEPROM_GetBootCount */
uint32_t EEPROM_BOOT_COUNT_MOCK = 0;

uint32_t EEPROM_GetBootCount_Mock()
{
    return EEPROM_BOOT_COUNT_MOCK;
}
#endif


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

_Static_assert(sizeof(LOG_NUM_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");
_Static_assert(sizeof(INFO_LOG_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");
_Static_assert(sizeof(WARNING_LOG_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");
_Static_assert(sizeof(CRITICAL_LOG_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");
_Static_assert(sizeof(BOOT_LOG_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");
_Static_assert(sizeof(CRD_LOG_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");
_Static_assert(sizeof(PASSTIME_LOG_FILENAME) / sizeof(char) <= SD_MAX_FILENAME, "log name exceeds max filename");

#define LOG_WRITER_QUEUE_LENGTH (6) // TODO: refine through testing
/* Size of buffer on the stack used by `Log_X_ClearEntryRange' to move data within a file. 
   TODO: to optimize SD access speed this should probably be sector aligned or something like that... */
#define FILE_MOVE_BUFSZ         (512) 

#define errorcheck_fr_goto(fres, lbl) if ((FRESULT)fres != FR_OK) goto lbl
#define errorcheck_fr_ret(fres) if ((FRESULT)fres != FR_OK) return fres
#define errorcheck_HAL_ret(retstat) if ((HAL_StatusTypeDef)retstat != HAL_OK) return retstat

#define MARKER_SIZE             sizeof(char[4])
#define INFO_LOG_MARKER         "INFO"
#define WARNING_LOG_MARKER      "WARN"
#define CRITICAL_LOG_MARKER     "CRIT"
#define BOOT_LOG_MARKER         "BOOT"
#define CRD_LOG_MARKER          "CRDL"

#define HEADER_SIZE             (MARKER_SIZE +sizeof(time_t)+sizeof(uint32_t))

#define BOOT_LOG_ENTRY_SIZE     (HEADER_SIZE+sizeof(uint16_t))
#define CRD_LOG_ENTRY_SIZE      (HEADER_SIZE+sizeof(CRD_Count_t))

#define PASSTIME_LOG_LINE_SZ    sizeof(time_t)

#define LOG_ENTRY_MIN_SZ        (BOOT_LOG_ENTRY_SIZE) // Boot log currently has the smallest entries

_Static_assert(LOG_ENTRY_MAX_SZ ==
    HEADER_SIZE + sizeof(uint8_t)
    + 49
    + sizeof(uint16_t), "Entry max sz def inconsistent");

#if defined(DEBUG_ENABLED)
/* Define this symbol to bypass the Log Writer task and directly write to the SD */
#define BYPASS_LOG_WRITER

/* If defined, uses this time for the entry timestamp instead of querying RTC */
//#define OVERRIDE_ENTRY_TIME ((time_t)42)

#define RAND_FLOAT_IN_RANGE(minval, maxval)                 (((float)rand())/RAND_MAX * (maxval-minval) + minval)
#define RAND_TEMP                                           RAND_FLOAT_IN_RANGE(-200.0, 200.0)
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Type used in log entries to hold the entry size
typedef uint16_t EntrySz_t;

// Type given to Log Writer to add a log entry
typedef struct {
    uint8_t data[LOG_ENTRY_MAX_SZ]; // Left-justified buffer containing entry data 
    EntrySz_t entrySz; // Number of bytes of data acutally used
    Log_Type_e entryType;
} LogEntry_t;

typedef struct {
    const char* const filename;
    const char* const startMarker;
} LogStrings_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static StaticQueue_t LogWriterQueue;
static uint8_t LogWriterQueueStorage[LOG_WRITER_QUEUE_LENGTH * sizeof(LogEntry_t)];
static QueueHandle_t LogWriterQueueHandle;

static const LogStrings_t LOG_STRINGS[LOG_TYPES_NUM] = {
    [LOG_INFO] = {.filename = INFO_LOG_FILENAME, .startMarker = INFO_LOG_MARKER},
    [LOG_WARN] = {.filename = WARNING_LOG_FILENAME, .startMarker = WARNING_LOG_MARKER},
    [LOG_CRIT] = {.filename = CRITICAL_LOG_FILENAME, .startMarker = CRITICAL_LOG_MARKER},
    [LOG_BOOT] = {.filename = BOOT_LOG_FILENAME, .startMarker = BOOT_LOG_MARKER},
    [LOG_CRD] = {.filename = CRD_LOG_FILENAME, .startMarker = CRD_LOG_MARKER},
};

#ifdef DEBUG_ENABLED // Debug: storing debugEntry
static LogEntry_t debugEntry;

static uint8_t using_RTC = 1; // Variable to toggle using RTC time or not
#define Sys_IsDevAvail(dev) (using_RTC)
#endif 

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static void createEntryHeader(LogEntry_t* entry)
{
    time_t currT = 0; HAL_StatusTypeDef RTC_stat;
#ifdef OVERRIDE_ENTRY_TIME
    currT = OVERRIDE_ENTRY_TIME; RTC_stat = HAL_OK;
#else
    Sys_TryDev(RTC_stat, RTC_GetTime(&currT), HAL_OK, DEV_RTC);
    if (RTC_stat != HAL_OK)
    // use RTOS tick count to get time since boot
    {
        currT = xTaskGetTickCount() / configTICK_RATE_HZ;
    }
#endif
    
    uint8_t entryPtr = 0;

    // Start marker
    memcpy(&(entry->data) + entryPtr, LOG_STRINGS[entry->entryType].startMarker, MARKER_SIZE); entryPtr += MARKER_SIZE;

    // RTC time
    memcpy(entry->data + entryPtr, &currT, sizeof(time_t)); entryPtr += sizeof(time_t);

    // Boot count
    uint32_t bootCt = EEPROM_GetBootCount();
    memcpy(entry->data + entryPtr, &bootCt, sizeof(uint32_t)); entryPtr += sizeof(uint32_t);
}

static FRESULT increment_log_counters(Log_Type_e log_type)
{
    FIL fd; FRESULT fres;
    fres = f_open(&fd, LOG_NUM_FILENAME, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
    errorcheck_fr_ret(fres);

    Log_Counts_t counts; UINT rwOut;
    fres = f_read(&fd, &counts, sizeof(Log_Counts_t), &rwOut); errorcheck_fr_goto(fres, end);

    counts[log_type]++;
    fres = f_rewind(&fd); errorcheck_fr_goto(fres, end);
    fres = f_write(&fd, &counts, sizeof(Log_Counts_t), &rwOut); errorcheck_fr_goto(fres, end);
end:;
    FRESULT closeRes = f_close(&fd);
    if (fres != FR_OK) return fres;
    else return closeRes;
}

/* Writes the data in `newEntry' to the SD */
static FRESULT LogWriter_Write(const LogEntry_t* newEntry)
{
#ifdef DEBUG_ENABLED   
    assert(LOG_ENTRY_MIN_SZ <= newEntry->entrySz && newEntry->entrySz <= LOG_ENTRY_MAX_SZ);
    assert(newEntry->entryType < LOG_TYPES_NUM);
#endif

#ifdef DEBUG_ENABLED // Debug: store newEntry in static variable rather than to SD card
    debugEntry = *newEntry;
    // return FR_OK;
#endif

    // Append incoming data
    FRESULT fres = SD_appendData(
        LOG_STRINGS[newEntry->entryType].filename,
        (const char*)newEntry->data, newEntry->entrySz);
    errorcheck_fr_ret(fres);

    // Update log counts
    fres = increment_log_counters(newEntry->entryType);

    debug_L2("MemLog wrote log %s", LOG_STRINGS[newEntry->entryType].startMarker);
    return fres;
}

/* Submits `newEntry' to the Log Writer task queue
so that the Log Writer writes it to the SD whenever the file becomes available. */
static void LogWriter_Submit(const LogEntry_t* newEntry)
{
#ifndef BYPASS_LOG_WRITER
    if (
        xQueueSendToBack(LogWriterQueueHandle, newEntry, 0)
        != pdPASS)
        Sys_RaiseLogicError(__FILE__, __LINE__, "fail send to queue");
#else
    FRESULT fres = LogWriter_Write(newEntry);
    if (fres != FR_OK) Sys_RaiseLogicError(__FILE__, __LINE__, "log write fail");
#endif
}

static void InfoWarnCrit_writeEntry(Log_Type_e logType, int entryType, const void* payload, uint16_t payloadSize)
{
    // Buffer in RAM in which the new entry is constructed.
    // When all the formatting is done, 
    // this buffer is submitted to the Log Writer queue.
    LogEntry_t tempEntry;
    tempEntry.entryType = logType;
    
    createEntryHeader(&tempEntry); 

    uint8_t entryPtr = HEADER_SIZE;

    // Entry type
    tempEntry.data[entryPtr] = entryType; entryPtr += sizeof(uint8_t);

    // Entry payload
    memcpy(tempEntry.data + entryPtr, payload, payloadSize); entryPtr += payloadSize;

    // Entry size
    tempEntry.entrySz = entryPtr + sizeof(EntrySz_t);
    memcpy(tempEntry.data + entryPtr, &(tempEntry.entrySz), sizeof(tempEntry.entrySz));

    // Submit to Log writer queue
    LogWriter_Submit(&tempEntry);
}

// Updates the read/write pointer of `fd' to point to the start of the previous entry,
// assuming that `fd' currently points to the start of an entry.
// Then reads and passes out the entry timestamp and bootct. 
static FRESULT prevEntry(FIL* fd, Log_Type_e logType, time_t* entryTime, uint32_t* entryBootCt)
{
    FSIZE_t currPos = f_tell(fd);
    if (currPos == 0) { *entryTime = 0; *entryBootCt = 0; return FR_OK; }
    else if (currPos < LOG_ENTRY_MIN_SZ) return FR_CORRUPT;

    FRESULT fres; UINT bytesRead;
    EntrySz_t entry_size;

    switch (logType)
    {
    case LOG_INFO:
    case LOG_WARN:
    case LOG_CRIT:
    {
        //position 2 bytes back
        fres = f_lseek(fd, currPos - sizeof(EntrySz_t)); errorcheck_fr_ret(fres);

        //read record_size
        fres = f_read(fd, &entry_size, sizeof(EntrySz_t), &bytesRead); errorcheck_fr_ret(fres);
        if (bytesRead != sizeof(EntrySz_t)
            || entry_size < LOG_ENTRY_MIN_SZ
            || entry_size > LOG_ENTRY_MAX_SZ) {
            // this should only happen if the file is corrupted somehow
            return FR_CORRUPT;
        }
        break;
    }
    case LOG_BOOT:
        entry_size = BOOT_LOG_ENTRY_SIZE;
        break;
    case LOG_CRD:
        entry_size = CRD_LOG_ENTRY_SIZE;
        break;
    default:
        Sys_InvEnumError(logType);
    }

    //there is something wrong if we don't have enough bytes to read the record
    if (currPos < entry_size) {
        // Return an error code instead of raising a logic error, 
        // because this might be due to file system corruption
        return FR_CORRUPT;
    }

    FSIZE_t prevEntryStart = currPos - entry_size;

    //position at the timestamp field of the entry
    fres = f_lseek(fd, prevEntryStart + MARKER_SIZE); errorcheck_fr_ret(fres);

    //read time and boot count in one go since they're adjacent
    uint8_t buf[sizeof(time_t) + sizeof(uint32_t)];
    fres = f_read(fd, buf, sizeof(buf), &bytesRead); errorcheck_fr_ret(fres);
    if (bytesRead != sizeof(buf)) return FR_CORRUPT;
    memcpy(entryTime, buf, sizeof(time_t)); memcpy(entryBootCt, buf + sizeof(time_t), sizeof(uint32_t));

    // Rewind the r/w pointer to the start of the entry
    fres = f_lseek(fd, prevEntryStart); 
    return fres;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL (NON-STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Log_WriteInfo(InfoEntryTypes_e entryType, const void* payload, uint16_t payloadSize)
{
#ifdef DEBUG_ENABLED // Debug check entry type & payload size
    switch (entryType)
    {
    case INFO_ENTRY_COMPCHECKS:
        assert(payloadSize == 37 || payloadSize == 49);
        break;
    case INFO_ENTRY_PWRLVL:
        assert(payloadSize == 1);
        break;
    case INFO_ENTRY_GNDPASS:
        assert(payloadSize == 1 || payloadSize == 9);
        break;
    }
#endif 

    InfoWarnCrit_writeEntry(LOG_INFO, (int)entryType, payload, payloadSize);
}

void Log_WriteWarn(WarnEntryTypes_e entryType, const void* payload, uint16_t payloadSize)
{
#ifdef DEBUG_ENABLED // Debug check entry type & payload size
    switch (entryType)
    {
    case WARN_ENTRY_TEMPERATURE:
        assert(payloadSize == 2);
        break;
    // Other entry types TODO
	case WARN_ENTRY_PWRLVL:
		assert(payloadSize == 1);
		break;
	case WARN_ENTRY_DEVWARN:
		assert(payloadSize == 3);
		break;
	case WARN_ENTRY_RADIOKILL:
		assert(payloadSize == 0);
		break;
	case WARN_ENTRY_RECMODE_TIMESET:
		assert(payloadSize == 8);
		break;
	case WARN_ENTRY_ADCS_STAT:
		assert(payloadSize == 2);
		break;
	case WARN_ENTRY_MEMINTEG:
		// TODO ! The document does not list any byte size requirements for this as the memory intergrity checker has not been implemented yet.
		// for now:
		Sys_RaiseLogicError(__FILE__, __LINE__, "inv entry type");
        break;
    }
#endif 

    InfoWarnCrit_writeEntry(LOG_WARN, (int)entryType, payload, payloadSize);
}

void Log_WriteCrit(CritEntryTypes_e entryType, const void* payload, uint16_t payloadSize)
{
#ifdef DEBUG_ENABLED // Debug check entry type & payload size
    switch (entryType)
    {
    case CRIT_ENTRY_TEMPERATURE:
        assert(payloadSize == 2);
        break;
    case CRIT_ENTRY_PWRLVL:
        assert(payloadSize == 1);
        break;
    case CRIT_ENTRY_DEVFAULT:
        assert(payloadSize == 2);
        break;
    case CRIT_ENTRY_SWRESET:
        assert(payloadSize == 1);
        break;
    case CRIT_ENTRY_RECMODE:
        assert(payloadSize == 0);
        break;
    case CRIT_ENTRY_ANTDEPL:
        assert(payloadSize == 0);
        break;
    case CRIT_ENTRY_EPSRST:
        assert(payloadSize == 1);
        break;
    }
#endif 

    InfoWarnCrit_writeEntry(LOG_CRIT, (int)entryType, payload, payloadSize);
}

void Log_WriteCRD(const CRD_Count_t countData)
{
    LogEntry_t tempEntry;
    tempEntry.entryType = LOG_CRD;

    createEntryHeader(&tempEntry);

    uint8_t entryPtr = HEADER_SIZE;

    // Copy counts data
    memcpy(tempEntry.data + entryPtr, countData, sizeof(CRD_Count_t)); entryPtr += sizeof(CRD_Count_t);

    tempEntry.entrySz = CRD_LOG_ENTRY_SIZE; 
#ifdef DEBUG_ENABLED
    assert(entryPtr == CRD_LOG_ENTRY_SIZE);
#endif

    LogWriter_Submit(&tempEntry);
}

void Log_WriteBoot(uint16_t resetReasonMask)
{
    LogEntry_t tempEntry;
    tempEntry.entryType = LOG_BOOT;

    createEntryHeader(&tempEntry);

    uint8_t entryPtr = HEADER_SIZE;

    memcpy(tempEntry.data + entryPtr, &resetReasonMask, sizeof(resetReasonMask)); entryPtr += sizeof(resetReasonMask);

    tempEntry.entrySz = BOOT_LOG_ENTRY_SIZE;
#ifdef DEBUG_ENABLED
    assert(entryPtr == BOOT_LOG_ENTRY_SIZE);
#endif

    LogWriter_Submit(&tempEntry);
}

FRESULT Log_GetLogCounts(Log_Counts_t* result)
{
    return SD_getFileContents(LOG_NUM_FILENAME, result, sizeof(Log_Counts_t));
}

FRESULT Log_FindEntryRange(Log_Type_e type, const time_t* startT, const time_t* endT, 
                           FSIZE_t* offsetStart, FSIZE_t* offsetEnd, uint32_t* totalBytes,
                           FIL* fd)
{
    FRESULT fres;

    //check for wrong calling parameters andy of those should be NULL or invalid timerange (when not clipping)
    if(startT==NULL || endT==NULL || totalBytes==NULL || fd == NULL ||
    		(
				(*startT != UINT64_MAX && *endT != UINT64_MAX) &&
				(*startT > *endT || ((*startT > TIME_SYNC_BOUNDARY) != (*endT > TIME_SYNC_BOUNDARY)))
			)
		)
        assert(0);


    //opening the existing file for writing 
    fres = f_open(fd, LOG_STRINGS[type].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);

    *totalBytes = 0; if (offsetStart != NULL) *offsetStart = f_size(fd); if (offsetEnd != NULL) *offsetEnd = f_size(fd);

    uint8_t clipToStart = *startT == UINT64_MAX, clipToEnd = *endT == UINT64_MAX; // UINT64_MAX is interpretted as -1
    #define checkRange(entry_time) ((clipToStart || *startT <= entry_time) && (entry_time < *endT || clipToEnd))
    uint8_t usingUTC = (*startT > TIME_SYNC_BOUNDARY || *endT > TIME_SYNC_BOUNDARY); // startT or endT could be UINT64_MAX

    // Short-circuit
    if (clipToEnd && clipToStart)
    {
        *offsetStart = 0;
        *offsetEnd = f_size(fd);
        *totalBytes = f_size(fd);
        return FR_OK;
    }

    time_t entry_time; uint32_t entryBootCt, currBootCt = EEPROM_GetBootCount();
    uint8_t first_entry = 1;
    FSIZE_t lastCheckedEntry = f_size(fd);
    // For the program and calculating totalBytes in boot time sandwiched between UTC times
    FSIZE_t private_offsetEnd = f_size(fd);

    // Seek to the end of the file
    fres = f_lseek(fd, f_size(fd)); errorcheck_fr_ret(fres);

    while (f_tell(fd) > 0) {

        // Go one entry back and gets its timestamp and boot count
        fres = prevEntry(fd, type, &entry_time, &entryBootCt);
        if (fres == FR_CORRUPT) return FR_CORRUPT;
        else errorcheck_fr_goto(fres, cleanup);

        // If the given start & end are not in UTC,
        // we first need to check the boot count of the entry
        if (!usingUTC)
        {
            // As soon as we hit an entry where the boot count is not the same as the current,
            // we're done because all further entries will also be from prior boots.
            if (entryBootCt != currBootCt) return FR_OK;
        }

        //Now decide if we are taking this entry
        if (
            // If we are clipping to the end, automatically take the last entry of the log if it's within range
            (first_entry && clipToEnd)
            // If arguments are in UTC and so is this timestamp, check it against the range as normal
         || (usingUTC && entry_time > TIME_SYNC_BOUNDARY && checkRange(entry_time)) 
            // If arguments are in UTC and entry timestamp isn't and not first entry, take it
         || (usingUTC && entry_time <= TIME_SYNC_BOUNDARY && !first_entry) 
            // If arguments are not in UTC and so is the entry timestamp, check it against the range as normal
         || (!usingUTC && entry_time <= TIME_SYNC_BOUNDARY && checkRange(entry_time)) 
            ) 
        {
            //first entry in range
            if(first_entry){
                // For clipping to end, if the first entry (at the end of file) doesn't fall within range,
                // then no other entry should fall within the range (given the increasing time of logs)
            	// given that the entry's time and range's time are both of the same type
                if (clipToEnd) {
                    if ((usingUTC && entry_time > TIME_SYNC_BOUNDARY || !usingUTC && entry_time <= TIME_SYNC_BOUNDARY)) {
                        if (! checkRange(entry_time))
                        	return FR_OK;
                    } else {
                    	// Continue to find a valid entry
                    	continue;
                    }
                }
                //we found the first entry in range
                if(offsetStart!=NULL)
                    *offsetStart = f_tell(fd);
                // offsetEnd doesn't need to be updated if clipToEnd
                if(offsetEnd!=NULL && !clipToEnd)
                    *offsetEnd = lastCheckedEntry;
                private_offsetEnd = lastCheckedEntry;
                if (!clipToEnd) {
                	// Normal totalBytes update
                	*totalBytes = lastCheckedEntry - f_tell(fd);
                } else {
                	// Handling the case in which the first valid entry is not at the end
                	*totalBytes = f_size(fd) - f_tell(fd);
                }
                first_entry = 0;

                // If we are clipping to the start and have already found the first entry in the time range, take all prior entries
                // If clipping to start just short-circuit out of FindEntryRange
                if (clipToStart) {
                    if(offsetStart!=NULL)
                        *offsetStart = 0;
                    *totalBytes = lastCheckedEntry;
                    return FR_OK;
                }
            } else {
                // For arguments of UTC time, we only update offsetStart/totalBytes if the entry has UTC time
                if ((usingUTC && entry_time > TIME_SYNC_BOUNDARY)) {
                	//adjust the offsetStart and totalBytes
                	if(offsetStart!=NULL) {
                        *offsetStart = f_tell(fd);
                    }
                	*totalBytes = private_offsetEnd - f_tell(fd);
				// If not using UTC time, update as normally
                } else if (!usingUTC) {
                	if(offsetStart!=NULL) {
						*offsetStart = f_tell(fd);
					}
                	*totalBytes += lastCheckedEntry - f_tell(fd);
                }
            }
        } else if (!first_entry) {
            // range must be contiguous, 
            // so as soon as we find the first entry that doesn't fit the criteria,
            // we're done
            return FR_OK;
        }
        lastCheckedEntry = f_tell(fd);
    }

cleanup:
    // If we ended up here because of a FatFS error,
    // close the file.
    if (fres != FR_OK)
    {
        f_close(fd); // Don't check the return status because we want to return the first error recorded
    }
    // Else we ended up here because there were no matching entries in the log,
    // in which case do nothing and just return
    return fres;
    #undef checkRange
}

FRESULT Log_ClearEntryRange(Log_Type_e type, const time_t* startT, const time_t* endT, 
    uint32_t* clearedBytes) {
    if (startT == NULL || endT == NULL || clearedBytes == NULL)
        Sys_RaiseLogicError(__FILE__, __LINE__, "null ptr passed");

    //variables Log_FindEntryRange                            
    FSIZE_t startMarkerOffset, endMarkerOffset;
    FIL fd;
    uint32_t bytesToClear;
    FRESULT fres = Log_FindEntryRange(type, startT, endT, &startMarkerOffset, &endMarkerOffset, &bytesToClear, &fd);
    if (fres != FR_OK && fres != FR_CORRUPT) return fres;

    //define a buffer on the stack to hold data while it is being moved
    uint8_t buffer[FILE_MOVE_BUFSZ];

    //compute how many bytes after the end marker needs to be move to the start marker
    FSIZE_t bytesToMove = f_size(&fd) - endMarkerOffset;

    *clearedBytes = 0;
    UINT actualRW; FSIZE_t plannedRW;

    //there are still bytes to be moved
    while (bytesToMove > 0) {
        fres = f_lseek(&fd, endMarkerOffset); errorcheck_fr_goto(fres, cleanup);
        //check how many bytes to read (not exceed buffer size or bytesToMove whatever is smaller)
        plannedRW = (bytesToMove > sizeof(buffer)) ? sizeof(buffer) : bytesToMove;
        fres = f_read(&fd, buffer, plannedRW, &actualRW); errorcheck_fr_goto(fres, cleanup);
        if (actualRW != plannedRW) { fres = FR_CORRUPT; goto cleanup; }
        fres = f_lseek(&fd, startMarkerOffset); errorcheck_fr_goto(fres, cleanup);
        fres = f_write(&fd, buffer, plannedRW, &actualRW); errorcheck_fr_goto(fres, cleanup);
        if (actualRW != plannedRW) { fres = FR_CORRUPT; goto cleanup; }
        //move markers according to what was read and write (moved) 
        startMarkerOffset += actualRW;
        endMarkerOffset += actualRW;
        bytesToMove -= actualRW;
        *clearedBytes += actualRW;
    }

    fres = f_lseek(&fd, startMarkerOffset); errorcheck_fr_goto(fres, cleanup);
#ifdef DEBUG_ENABLED
    assert((f_size(&fd) - endMarkerOffset == 0) && (bytesToClear == endMarkerOffset - startMarkerOffset));
#endif
    fres = f_truncate(&fd);
cleanup:;
    FRESULT closeRes = f_close(&fd);
    // Return the first file system error encountered
    if (fres != FR_OK && fres != FR_CORRUPT) return fres;
    else if (closeRes != FR_OK) return closeRes;
    else return fres;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifndef BYPASS_LOG_WRITER
void Log_X_LogWriterTask(void* arg)
{
    // Create static queue
    LogWriterQueueHandle = 
        xQueueCreateStatic(LOG_WRITER_QUEUE_LENGTH, sizeof(LogEntry_t), LogWriterQueueStorage, &LogWriterQueue);

    FRESULT fres;
    LogEntry_t incomingEntry;
    while (1)
    {
        // Wait for something to come in to queue
        xQueueReceive(LogWriterQueueHandle, &incomingEntry, portMAX_DELAY);

        // When a log entry gets submitted,
        Sys_TryDev(fres, LogWriter_Write(&incomingEntry), FR_OK, DEV_SD);
    }
}
#endif

void Log_X_TimeSyncCallback(time_t delta) 
{
    FIL fd;
    FRESULT fres; UINT bytesRead;
    time_t entryTime;
    uint32_t currBootCt = EEPROM_GetBootCount(), entryBootCt;

    // Open each of the 5 log files on the SD card
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++)
    {
        // Declare a label at the start of every iteration,
        // so that if at any point during handling of a file we are interrupted by an SD error
        // and SysMan resets the SD card,
        // we try again from the same file, until SysMan decides the SD is in fault.
    retry:
#define report_retry(fres) if (fres != FR_OK) { Sys_RaiseDevError(DEV_SD, fres); goto retry; }

        fres = f_open(&fd, LOG_STRINGS[i].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); report_retry(fres);

        // Seek to the end of the file
        fres = f_lseek(&fd, f_size(&fd)); report_retry(fres);

        // Iterate through the log entries in reverse order and adjust timestamps
        while (f_tell(&fd) > 0) 
        {
            fres = prevEntry(&fd, i, &entryTime, &entryBootCt);
            // If we run into issues navigating the entries, move on to the next file
            if (fres == FR_CORRUPT) break;

            if (entryBootCt == currBootCt)
            {
                // Adjust the timestamp by adding the time delta
                entryTime += delta;

                // Write the adjusted timestamp back to the log entry
                fres = f_lseek(&fd, f_tell(&fd) + MARKER_SIZE); report_retry(fres);
                fres = f_write(&fd, &entryTime, sizeof(time_t), &bytesRead); report_retry(fres);
                if (bytesRead != sizeof(time_t)) break; // File is probably corrupted
                // Move to the start of this log entry
                fres = f_lseek(&fd, f_tell(&fd) - sizeof(time_t) - MARKER_SIZE); report_retry(fres);
            }
            // As soon as we find an entry with a different boot count, 
            // all prior entries will also be from prior boots so we're done with this file.
            else break;
#undef report_retry
        }
        // Close the log file
        fres = f_close(&fd);
        // At this point we're done with this file,
        // so if we get an error while closing it,
        // report it, move on to the next file if the SD is still available,
        // and stop if it isn't.
        if (fres != FR_OK)
        {
            Sys_RaiseDevError(DEV_SD, fres);
            if (Sys_IsDevAvail(DEV_SD)) continue;
            else return;
        }
    }
}

FRESULT Log_X_CleanPassTime(const time_t* currT)
{
    Sys_CheckPointer(currT);
    
    if (*currT < TIME_SYNC_BOUNDARY) return FR_OK;

    FIL fd; FRESULT fres, fres_temp;

    fres = f_open(&fd, PASSTIME_LOG_FILENAME, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);

    if (f_size(&fd) < PASSTIME_LOG_LINE_SZ) goto end;
    
    time_t passT; UINT bytesRead;
    while (1)
    {
        fres = f_lseek(&fd, f_size(&fd) - PASSTIME_LOG_LINE_SZ); errorcheck_fr_goto(fres, end);
        fres = f_read(&fd, &passT, PASSTIME_LOG_LINE_SZ, &bytesRead); errorcheck_fr_goto(fres, end);
        if (passT < (*currT - EEPROM_emul_DataTemp.radioTOs.passivePassWindow * 60)) break;
        fres = f_lseek(&fd, f_size(&fd) - PASSTIME_LOG_LINE_SZ); errorcheck_fr_goto(fres, end);
        fres = f_truncate(&fd); errorcheck_fr_goto(fres, end);
    }

end:
    fres_temp = f_close(&fd);
    if (fres != FR_OK) return fres;
    else return fres_temp;
}

FRESULT Log_X_GetPassTime(time_t* passT)
{
    Sys_CheckPointer(passT);

    FIL fd; FRESULT fres, fres_temp; 

    fres = f_open(&fd, PASSTIME_LOG_FILENAME, FA_READ | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);

    if (f_size(&fd) < PASSTIME_LOG_LINE_SZ) { *passT = 0; goto end; }
     
    // Seek to the last line in the file
    fres = f_lseek(&fd, f_size(&fd) - PASSTIME_LOG_LINE_SZ); errorcheck_fr_goto(fres, end);

    // Read the last line into a time_t
    UINT bytesRead;
    fres = f_read(&fd, passT, PASSTIME_LOG_LINE_SZ, &bytesRead); 
end:
    fres_temp = f_close(&fd);
    if (fres != FR_OK) return fres;
    else return fres_temp;
}

FRESULT Log_X_GetNumPassTimes(uint32_t* num)
{
    FILINFO finfo;
    FRESULT retstat = f_stat(PASSTIME_LOG_FILENAME, &finfo); errorcheck_fr_ret(retstat);
    *num = finfo.fsize / PASSTIME_LOG_LINE_SZ;
    return retstat;
}

// Testing
#ifdef DEBUG_ENABLED 
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TEST SET DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static const Test_t Memory_Logs_Tests[] = {
        {.test = Log_Test_BasicFormatting, .testName = "Memory Logs Basic formatting"},
        {.test = Log_Test_BasicFatFS, .testName = "Memory Logs Basic FatFS"},
        {.test = Memory_Logs_Info_Test, .testName = "Memory Logs Info Test"},
		{.test = Memory_Logs_Warn_Test, .testName = "Memory Logs Warn Test"},
        {.test = Memory_Logs_Crit_Test, .testName = "Memory Logs Critical Test"},
        {.test = Memory_Logs_Boot_Test, .testName = "Memory Logs Boot Test"},
        {.test = Memory_Logs_CRD_Test, .testName = "Memory Logs CRD Test"},
        {.test = Log_Test_LogCounts, .testName = "Memory Logs Log Counts Test"},
        {.test = Log_Test_Continuity, .testName = "Memory Logs Continuity Test"},
        {.test = Memory_Logs_Log_FindEntryRange_All_UTC_Test, .testName = "Memory Logs FindEntryRange All UTC time Test"},
        {.test = Memory_Logs_Log_FindEntryRange_All_OnBoardTime_Test, .testName = "Memory Logs FindEntryRange All On Board time Test"},
        {.test = Memory_Logs_Log_FindEntryRange_MixedTime_Test, .testName = "Memory Logs FindEntryRange UTC time and On Board time Test"},
        {.test = Memory_Logs_Log_ClearEntryRange_Test, .testName = "Memory Logs ClearEntryRange Test"},
        {.test = Memory_Logs_Log_TimeSyncCallback_Test, .testName = "Memory Logs TimeSyncCallback Test"}
        //{.test = Memory_Logs_Log_FindEntryRange_Test, .testName = "Memory Logs Log_FindEntryRange Test"},
        //{.test = Memory_Logs_Log_X_TimeSyncCallback_Test, .testName = "Memory Logs Log_X_TimeSyncCallback Test"},
        //{.test = Memory_Logs_Log_X_ClearEntryRange_Test, .testName = "Memory Logs Log_X_ClearEntryRange Test"}
};

DefineTestSet(Memory_Logs_Benchmark_Set, "Memory Logs", Memory_Logs_Tests);

// Check entry header (start marker + entry time + boot count) against expected values.
// If NULL is passed for `expT', assume correct time is `OVERRIDE_ENTRY_TIME' if defined or current RTC time otherwise.
// If NULL is passed for `expBootCt', assume correct boot count is returned by `EEPROM_GetBootCount'.
TestStatus_e Log_TestFixture_CheckEntryHeader(void* entryStart, const char* expMarker, time_t* expT, uint32_t* expBootCt)
{
    test_assert(memcmp(entryStart, expMarker, MARKER_SIZE) == 0);
    time_t correctT;
    if (expT == NULL)
    {
#ifdef OVERRIDE_ENTRY_TIME
        correctT = OVERRIDE_ENTRY_TIME;
#else
        HAL_StatusTypeDef HAL_stat = RTC_GetTime(&correctT);
        test_assert(HAL_stat == HAL_OK);
#endif
    }
    else correctT = *expT;
    test_assert(memcmp(entryStart + MARKER_SIZE, &correctT, sizeof(time_t)) == 0);
    uint32_t correctBootCt;
    if (expBootCt == NULL) correctBootCt = EEPROM_GetBootCount();
    else correctBootCt = *expBootCt;
    test_assert(memcmp(entryStart + MARKER_SIZE + sizeof(time_t), &correctBootCt, sizeof(uint32_t)) == 0);
    return TEST_PASS;
}

// Checks the static `debugEntry' against the given expected data for an INFO/WARN/CRIT entry
TestStatus_e Log_TestFixture_CheckDebugEntry(const char* entry_header, int entry_type, void *data, size_t data_size) {
    // Check raw entry data
    test_assert(Log_TestFixture_CheckEntryHeader(&debugEntry.data, entry_header, NULL, NULL) == TEST_PASS);
    test_assert(memcmp(debugEntry.data + HEADER_SIZE, &entry_type, 1) == 0);
    test_assert(memcmp(debugEntry.data + HEADER_SIZE + 1, data, data_size) == 0);
    uint16_t expected_entry_size = HEADER_SIZE + 1 + data_size + sizeof(uint16_t);
    test_assert(memcmp(debugEntry.data + HEADER_SIZE + 1 + data_size, &expected_entry_size, sizeof(uint16_t)) == 0);
    
    // Check entry metadata
    test_assert(debugEntry.entrySz == expected_entry_size);
    test_assert(strcmp(LOG_STRINGS[debugEntry.entryType].startMarker, entry_header) == 0);

    return TEST_PASS;
}

TestStatus_e Log_TestFixture_ClearAllLogs()
{
    FRESULT retstat;
    for (Log_Type_e l = 0; l < LOG_TYPES_NUM; l++)
    {
        retstat = SD_overwriteData(LOG_STRINGS[l].filename, NULL, 0);
        test_assert(retstat == FR_OK);
    }

    Log_Counts_t zeroCounts = {};
    // TODO make sure this empty initializer actually initializes to zero
    retstat = SD_overwriteData(LOG_NUM_FILENAME, &zeroCounts, sizeof(Log_Counts_t));
    test_assert(retstat == FR_OK);

    return TEST_PASS;
}

// Set up CRD logs with time_num - 1 log files
TestStatus_e Log_TestFixture_SetupLogs(uint8_t is_UTC, time_t* start_times, size_t time_num, Log_Type_e type) {  
    // Set using RTC or not
    using_RTC = is_UTC;

    // Set up array of time_t to record times of log writing
    HAL_StatusTypeDef RTC_stat;

    // Set time if using RTC
    if (using_RTC) {
    	RTC_stat = RTC_X_SetTime(TIME_SYNC_BOUNDARY + 1000); errorcheck_HAL_ret(RTC_stat);
    }

    // Data for each type of log
    CRD_Count_t rand_CRD_count;
    // Make life easier: just payload sizes of 1
    uint8_t payload[1];
    payload[0] = 1;
    uint16_t payloadSize = 1;
    if (type == LOG_CRD) {
        for (size_t i = 0; i < 8; i++) {
            rand_CRD_count[i] = (uint32_t) RAND_FLOAT_IN_RANGE(0, UINT32_MAX);
        }
    }
    
    // Create five logs
    for (size_t i = 0; i < time_num - 1; i++) {
        // Get start time
        if (is_UTC) {
            RTC_stat = RTC_GetTime(&start_times[i]); errorcheck_HAL_ret(RTC_stat);
        } else {
            start_times[i] = xTaskGetTickCount() / configTICK_RATE_HZ;
        }
        HAL_StatusTypeDef RTC_stat;
        
        switch(type) {
            case LOG_INFO:
                RTC_stat = Log_WriteInfo(INFO_ENTRY_PWRLVL, (void*) payload, payloadSize);
                break;
            case LOG_WARN:
                RTC_stat = Log_WriteWarn(WARN_ENTRY_PWRLVL, (void*) payload, payloadSize);
                break;
            case LOG_CRIT:
                RTC_stat = Log_WriteCrit(CRIT_ENTRY_SWRESET, (void*) payload, payloadSize);
                break;
            case LOG_CRD:
                RTC_stat = Log_WriteCRD(rand_CRD_count);
                break;
            case LOG_BOOT:
                RTC_stat = Log_WriteBoot(0);
                break;
        }
        
        test_assert(RTC_stat == HAL_OK);
        HAL_Delay(1000); // Busy-wait to make sure RTC time increases a bit // Does this affect tick count?
    }
    // Record end time
    if (is_UTC) {
        RTC_stat = RTC_GetTime(&start_times[time_num - 1]); errorcheck_HAL_ret(RTC_stat);
    } else {
        start_times[time_num - 1] = xTaskGetTickCount() / configTICK_RATE_HZ;
    }

    return TEST_PASS;
}

// Checks that the timestamps in the file matches the times in the array
TestStatus_e Log_TestFixture_CheckTimes(FIL* fd, Log_Type_e type, time_t* times, size_t num_times) {
    // Look through the file
    time_t entry_time;
    uint32_t entryBootCt = EEPROM_GetBootCount();
    int entry_idx = num_times - 1;
    // Seek to the end of the file
    FRESULT fres = f_lseek(fd, f_size(fd)); errorcheck_fr_ret(fres);

    while (f_tell(fd) > 0 && entry_idx >= 0) {
        // Go one entry back and gets its timestamp and boot count
        fres = prevEntry(fd, type, &entry_time, &entryBootCt);

        if (times[entry_idx] != entry_time) {
            return TEST_FAIL;
        }

        entry_idx--;
    }

    // Didn't look through the whole file
    if (f_tell(fd) > 0) {
        return TEST_FAIL;
    }

    // If there are more times expected than in the file
    if (entry_idx >= 0) {
        return TEST_FAIL;
    }

    return TEST_PASS;
}

#define TEST_DEFAULT_DATA_SZ (6)

TestStatus_e Log_Test_BasicFormatting()
{
    // Just test the correct construction of an info/warn/crit entry
    char mystr[TEST_DEFAULT_DATA_SZ] = "hello";
    HAL_StatusTypeDef RTC_stat = InfoWarnCrit_writeEntry(0, 42, mystr, sizeof(mystr));
    test_assert(RTC_stat == HAL_OK);

    test_assert(Log_TestFixture_CheckDebugEntry(LOG_STRINGS[0].startMarker, 42, mystr, TEST_DEFAULT_DATA_SZ) == TEST_PASS);

    return TEST_PASS;
}

TestStatus_e Log_Test_BasicFatFS()
{
    // Test that an entry can be retrieved correctly through FatFS
    use_test_fixture(Log_TestFixture_ClearAllLogs);

    char mystr[TEST_DEFAULT_DATA_SZ] = "hello";
    HAL_StatusTypeDef RTC_stat = InfoWarnCrit_writeEntry(0, 42, mystr, sizeof(mystr));
    test_assert(RTC_stat == HAL_OK);

    FIL fd;
    FRESULT retstat = f_open(&fd, LOG_STRINGS[LOG_INFO].filename, FA_READ | FA_OPEN_EXISTING);
    test_assert(retstat == FR_OK);
    char buf[HEADER_SIZE + 1 + TEST_DEFAULT_DATA_SZ + sizeof(uint16_t)];
    UINT bytesRead = 0;
    retstat = f_read(&fd, buf, sizeof(buf), &bytesRead);
    test_assert(retstat == FR_OK);
    retstat = f_close(&fd);
    test_assert(retstat == FR_OK);

    test_assert(bytesRead == sizeof(buf));
    memcpy(&debugEntry.data, buf, sizeof(buf));
    debugEntry.entrySz = sizeof(buf);
    debugEntry.entryType = 0;
    test_assert(Log_TestFixture_CheckDebugEntry(LOG_STRINGS[0].startMarker, 42, mystr, TEST_DEFAULT_DATA_SZ) == TEST_PASS);

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Info_Test()
{
    // Test Component checks dump
    uint8_t comp_checks_payload[49];

    float rand_float;
    for (size_t i = 0; i < 32; i += 4) {
        float rand_float = RAND_FLOAT_IN_RANGE(0, 100);
        memcpy(comp_checks_payload + i, &rand_float, 4);
    }
    uint8_t batt_heaters = rand() % 256;
    memcpy(comp_checks_payload + 32, &batt_heaters, 1);
    rand_float = RAND_FLOAT_IN_RANGE(0, 100);
    memcpy(comp_checks_payload + 33, &rand_float, 4);
    float omega[3] = {RAND_FLOAT_IN_RANGE(-1000, 1000), RAND_FLOAT_IN_RANGE(-1000, 1000), RAND_FLOAT_IN_RANGE(-1000, 1000)};
    memcpy(comp_checks_payload + 37, &omega, 12);
    
    HAL_StatusTypeDef RTC_stat = Log_WriteInfo(INFO_ENTRY_COMPCHECKS, comp_checks_payload, 49);
    test_assert(RTC_stat == HAL_OK);

    // Test just the entry formatting - Log Writer will copy the entry into debugEntry
    test_assert(Log_TestFixture_CheckDebugEntry(INFO_LOG_MARKER, INFO_ENTRY_COMPCHECKS, &comp_checks_payload, 49) == TEST_PASS);

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Warn_Test()
{
	HAL_StatusTypeDef RTC_stat;

    // Test temperature warning
	uint8_t temp_warn_payload[2];
	float rand_float_temp = RAND_FLOAT_IN_RANGE(-200, 200);
	memcpy(temp_warn_payload, &rand_float_temp, 4);
	rand_float_temp = RAND_FLOAT_IN_RANGE(-200, 200);
	memcpy(temp_warn_payload + 4, &rand_float_temp, 4);
	RTC_stat = Log_WriteWarn(WARN_ENTRY_TEMPERATURE, temp_warn_payload, 2);
    test_assert(RTC_stat == HAL_OK);

	test_assert(Log_TestFixture_CheckDebugEntry(WARNING_LOG_MARKER, WARN_ENTRY_TEMPERATURE, &temp_warn_payload, 2) == TEST_PASS);

	// Test power level warning
	uint8_t pwr_lvl_warn_payload[1];
	uint8_t rand_byte = rand() % 256;
	memcpy(pwr_lvl_warn_payload, &rand_byte, 1);
	RTC_stat = Log_WriteWarn(WARN_ENTRY_PWRLVL, pwr_lvl_warn_payload, 1);
    test_assert(RTC_stat == HAL_OK);

	test_assert(Log_TestFixture_CheckDebugEntry(WARNING_LOG_MARKER, WARN_ENTRY_PWRLVL, &pwr_lvl_warn_payload, 1) == TEST_PASS);

	// Test device warning
	uint8_t dev_warn_payload[3];
	float rand_float_dev = RAND_FLOAT_IN_RANGE(-200, 200);
	memcpy(dev_warn_payload, &rand_float_dev, 4);
	rand_float_dev = RAND_FLOAT_IN_RANGE(-200, 200);
	memcpy(dev_warn_payload + 4, &rand_float_dev, 4);
	rand_float_dev = RAND_FLOAT_IN_RANGE(-200, 200);
	memcpy(dev_warn_payload + 8, &rand_float_dev, 4);
	RTC_stat = Log_WriteWarn(WARN_ENTRY_DEVWARN, dev_warn_payload, 3);
    test_assert(RTC_stat == HAL_OK);

	test_assert(Log_TestFixture_CheckDebugEntry(WARNING_LOG_MARKER, WARN_ENTRY_DEVWARN, &dev_warn_payload, 3) == TEST_PASS);

	// Test radio kill warning
	uint8_t radio_kill_warn_payload[0];
	RTC_stat = Log_WriteWarn(WARN_ENTRY_RADIOKILL, radio_kill_warn_payload, 0);
    test_assert(RTC_stat == HAL_OK);

	test_assert(Log_TestFixture_CheckDebugEntry(WARNING_LOG_MARKER, WARN_ENTRY_RADIOKILL, &radio_kill_warn_payload, 0) == TEST_PASS);

	// Test record mode time set warning
	uint8_t rec_mode_time_set_warn_payload[8];
	float rand_float_timeset;
	// loop might need a fix depending on how the data is stored (maybe make i a pointer)
    for (uint8_t* i = rec_mode_time_set_warn_payload; i <rec_mode_time_set_warn_payload + 25; i += 4) {
        rand_float_timeset = RAND_FLOAT_IN_RANGE(0, 100);
        memcpy(i, &rand_float_timeset, 4);
    }
	RTC_stat = Log_WriteWarn(WARN_ENTRY_RECMODE_TIMESET, rec_mode_time_set_warn_payload, 8);
    test_assert(RTC_stat == HAL_OK);
	
	test_assert(Log_TestFixture_CheckDebugEntry(WARNING_LOG_MARKER, WARN_ENTRY_RECMODE_TIMESET, &rec_mode_time_set_warn_payload, 8) == TEST_PASS);

	// Test ADCS status warning
	uint8_t adcs_status_warn_payload[2];
	uint8_t rand_byte_adcs = rand() % 256;
	memcpy(adcs_status_warn_payload, &rand_byte_adcs, 1);
	rand_byte_adcs = rand() % 256;
	memcpy(adcs_status_warn_payload + 1, &rand_byte_adcs, 1);
	RTC_stat = Log_WriteWarn(WARN_ENTRY_ADCS_STAT, adcs_status_warn_payload, 2);
    test_assert(RTC_stat == HAL_OK);

	test_assert(Log_TestFixture_CheckDebugEntry(WARNING_LOG_MARKER, WARN_ENTRY_ADCS_STAT, &adcs_status_warn_payload, 2) == TEST_PASS);

	// Test memory integrity warning
	// TODO ! The document does not list any byte size requirements for this as the memory intergrity checker has not been implemented yet.

	return TEST_PASS;
}

TestStatus_e Memory_Logs_Crit_Test() {
	HAL_StatusTypeDef RTC_stat;

    // Test critical log of out-of-bounds temperature 
    uint8_t temp_payload[2];
    temp_payload[0] = DEV_TCV; // Transceiver measured unsafe temperature
    temp_payload[1] = 0x3;
    RTC_stat = Log_WriteCrit(CRIT_ENTRY_TEMPERATURE, temp_payload, 2);
    test_assert(RTC_stat == HAL_OK);
    test_assert(Log_TestFixture_CheckDebugEntry(CRITICAL_LOG_MARKER, CRIT_ENTRY_TEMPERATURE, &temp_payload, 2) == TEST_PASS);

    // Test critical log of out-of-bounds power level change 
    uint8_t power_payload[1];
    power_payload[0] = L2;
    RTC_stat = Log_WriteCrit(CRIT_ENTRY_PWRLVL, power_payload, 1);
    test_assert(RTC_stat == HAL_OK);
    test_assert(Log_TestFixture_CheckDebugEntry(CRITICAL_LOG_MARKER, CRIT_ENTRY_PWRLVL, &power_payload, 1) == TEST_PASS);

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Boot_Test()
{
	HAL_StatusTypeDef RTC_stat;

    uint16_t resetReasons = 42;
    RTC_stat = Log_WriteBoot(resetReasons);
    test_assert(RTC_stat == HAL_OK);

    // Check contents are correct
    test_assert(Log_TestFixture_CheckEntryHeader(&debugEntry.data, BOOT_LOG_MARKER, NULL, NULL) == TEST_PASS);
    test_assert(memcmp(&resetReasons, debugEntry.data + HEADER_SIZE, sizeof(uint16_t)) == 0);

    // Check size is correct
    test_assert(debugEntry.entrySz == BOOT_LOG_ENTRY_SIZE);

    // Check entry type is correct
    test_assert(debugEntry.entryType == LOG_BOOT);

    return TEST_PASS;
}

TestStatus_e Memory_Logs_CRD_Test() {
    CRD_Count_t rand_CRD_count;
    for (size_t i = 0; i < 8; i++) {
        rand_CRD_count[i] = (uint32_t)RAND_FLOAT_IN_RANGE(0, UINT32_MAX);
    }

    HAL_StatusTypeDef RTC_stat = Log_WriteCRD(rand_CRD_count);
    test_assert(RTC_stat == HAL_OK);

    // Check contents are correct
    test_assert(Log_TestFixture_CheckEntryHeader(&debugEntry.data, CRD_LOG_MARKER, NULL, NULL) == TEST_PASS);
    test_assert(memcmp(&rand_CRD_count, debugEntry.data + 16, 32) == 0);

    // Check size is correct
    test_assert(debugEntry.entrySz == CRD_LOG_ENTRY_SIZE);

    // Check entry type is correct
    test_assert(debugEntry.entryType == LOG_CRD);

    return TEST_PASS;
}

TestStatus_e Log_Test_LogCounts()
{
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    Log_Counts_t ct;
    FRESULT retstat;
    HAL_StatusTypeDef RTC_stat;

    retstat = Log_GetLogCounts(&ct);
    test_assert(retstat == FR_OK);
    for (Log_Type_e l = 0; l < LOG_TYPES_NUM; l++) test_assert(ct[l] == 0);

    char mystr[TEST_DEFAULT_DATA_SZ] = "hello";
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    retstat = Log_GetLogCounts(&ct); test_assert(retstat == FR_OK);
    test_assert(ct[LOG_INFO] == 1);
    for (Log_Type_e l = LOG_INFO + 1; l < LOG_TYPES_NUM; l++) test_assert(ct[l] == 0);

    RTC_stat = Log_WriteWarn(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    retstat = Log_GetLogCounts(&ct); test_assert(retstat == FR_OK);
    for (Log_Type_e l = LOG_INFO; l <= LOG_WARN; l++) test_assert(ct[l] == 1);
    for (Log_Type_e l = LOG_WARN + 1; l < LOG_TYPES_NUM; l++) test_assert(ct[l] == 0);

    RTC_stat = Log_WriteCrit(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    retstat = Log_GetLogCounts(&ct); test_assert(retstat == FR_OK);
    for (Log_Type_e l = LOG_INFO; l <= LOG_CRIT; l++) test_assert(ct[l] == 1);
    for (Log_Type_e l = LOG_CRIT + 1; l < LOG_TYPES_NUM; l++) test_assert(ct[l] == 0);

    RTC_stat = Log_WriteBoot(42);
    test_assert(RTC_stat == HAL_OK);
    retstat = Log_GetLogCounts(&ct); test_assert(retstat == FR_OK);
    for (Log_Type_e l = LOG_INFO; l <= LOG_BOOT; l++) test_assert(ct[l] == 1);
    for (Log_Type_e l = LOG_BOOT + 1; l < LOG_TYPES_NUM; l++) test_assert(ct[l] == 0);

    CRD_Count_t rand_CRD_count;
    for (size_t i = 0; i < 8; i++) {
        rand_CRD_count[i] = (uint32_t)RAND_FLOAT_IN_RANGE(0, UINT32_MAX);
    }
    RTC_stat = Log_WriteCRD(rand_CRD_count);
    test_assert(RTC_stat == HAL_OK);
    retstat = Log_GetLogCounts(&ct); test_assert(retstat == FR_OK);
    for (Log_Type_e l = LOG_INFO; l <= LOG_CRD; l++) test_assert(ct[l] == 1);
    for (Log_Type_e l = LOG_CRD + 1; l < LOG_TYPES_NUM; l++) test_assert(ct[l] == 0);

    for (int i = 0; i < 9; i++) Log_WriteCRD(rand_CRD_count);
    retstat = Log_GetLogCounts(&ct); test_assert(retstat == FR_OK);
    for (Log_Type_e l = LOG_INFO; l < LOG_TYPES_NUM; l++)
    {
        if (l != LOG_CRD) test_assert(ct[l] == 1);
    }
    test_assert(ct[LOG_CRD] == 1 + 9);

    return TEST_PASS;
}

// Tests that the timestamps and boot counts of successive entries make sense
TestStatus_e Log_Test_Continuity()
{
    // TODO
	HAL_StatusTypeDef RTC_stat;

    char mystr[TEST_DEFAULT_DATA_SZ] = "hello";
    RTC_stat = Log_WriteWarn(42, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    // Store this entry's time and boot count
    time_t firstT;
    uint32_t firstBootCt;
    memcpy(&firstT, debugEntry.data + MARKER_SIZE, sizeof(time_t));
    memcpy(&firstBootCt, debugEntry.data + MARKER_SIZE + sizeof(time_t), sizeof(uint32_t));

    HAL_Delay(5000); // Busy-wait to make sure RTC time increases a bit
    RTC_stat = Log_WriteWarn(42, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);

    time_t secondT;
    uint32_t secondBootCt;
    memcpy(&secondT, debugEntry.data + MARKER_SIZE, sizeof(time_t));
    memcpy(&secondBootCt, debugEntry.data + MARKER_SIZE + sizeof(time_t), sizeof(uint32_t));

#ifdef OVERRIDE_ENTRY_TIME
    // Check that both entries have the same time
    test_assert(firstT == secondT);
#else
    // Check the next entry comes after in time
    test_assert(firstT < secondT);
#endif
    // Check that the boot count is the same
    test_assert(firstBootCt == secondBootCt);

    return TEST_PASS;
}

TestStatus_e Log_TestFixture_CheckSameTimeType(uint8_t is_UTC) {
    // Set up 5 CRD logs
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    time_t start_times[6];
    test_assert(Log_TestFixture_SetupLogs(is_UTC, start_times, 6, LOG_CRD) == TEST_PASS);

    // For recording FindEntryRange outputs
    FSIZE_t offsetStart;
    FSIZE_t offsetEnd;
    uint32_t totalBytes;
    FIL fd;
    FRESULT fres;
    time_t startT;
    time_t endT;

    // all entries are older
    startT = start_times[0] - 10;
    endT = start_times[0];
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == offsetEnd);
    test_assert(totalBytes == 0);

    // Clipping to start, should not take any entries
    startT = UINT64_MAX;
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == offsetEnd);
    test_assert(totalBytes == 0);

    // Clipping to end, should take all entries
    startT = start_times[0] - 10;
    endT = UINT64_MAX;
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 0);
    test_assert(offsetEnd = 5 * CRD_LOG_ENTRY_SIZE);
    test_assert(totalBytes == 5 * CRD_LOG_ENTRY_SIZE);

    // all entries are younger
    startT = start_times[5];
    endT = start_times[5] + 10;
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == offsetEnd);
    test_assert(totalBytes == 0);

    // Clipping to start, should take all entries
    startT = UINT64_MAX;
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 0);
    test_assert(offsetEnd == 5 * CRD_LOG_ENTRY_SIZE);
    test_assert(totalBytes == 5 * CRD_LOG_ENTRY_SIZE);

    // Clipping to end, should take no entries
    startT = start_times[5];
    endT = UINT64_MAX;
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == offsetEnd);
    test_assert(totalBytes == 0);

    // valid entry range is at start of file
    startT = start_times[0];
    endT = start_times[1];
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 0);
    test_assert(offsetEnd == CRD_LOG_ENTRY_SIZE);
    test_assert(totalBytes == CRD_LOG_ENTRY_SIZE);

    // valid entry range is at end of file
    startT = start_times[4];
    endT = start_times[5];
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == CRD_LOG_ENTRY_SIZE * 4);
    test_assert(offsetEnd == CRD_LOG_ENTRY_SIZE * 5);
    test_assert(totalBytes == CRD_LOG_ENTRY_SIZE);

    // valid entry range is in the middle of file
    // Range includes middle three entries
    startT = start_times[1];
    endT = start_times[4];
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == CRD_LOG_ENTRY_SIZE * 1);
    test_assert(offsetEnd == CRD_LOG_ENTRY_SIZE * 4);
    test_assert(totalBytes == CRD_LOG_ENTRY_SIZE * 3);

    // exactly one valid entry in the middle of file
    // Range only includes 3rd entry in middle of file
    startT = start_times[2];
    endT = start_times[3];
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == CRD_LOG_ENTRY_SIZE * 2);
    test_assert(offsetEnd == CRD_LOG_ENTRY_SIZE * 3);
    test_assert(totalBytes == CRD_LOG_ENTRY_SIZE);

    // clip to start, valid range stops in the middle of the file
    // Range includes first three entries
    startT = UINT64_MAX;
    endT = start_times[3];
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == CRD_LOG_ENTRY_SIZE * 0);
    test_assert(offsetEnd == CRD_LOG_ENTRY_SIZE * 3);
    test_assert(totalBytes == CRD_LOG_ENTRY_SIZE * 3);

    // clip to end, valid range starts in the middle of the file
    // Range includes last three entries
    startT = start_times[2];
    endT = UINT64_MAX;
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == CRD_LOG_ENTRY_SIZE * 2);
    test_assert(offsetEnd == CRD_LOG_ENTRY_SIZE * 5);
    test_assert(totalBytes == CRD_LOG_ENTRY_SIZE * 3);

    // whole file has entries in UTC (or on board time) but arguments are in on-board time (or UTC time): should not take any entries
    startT = 1;
    endT = 5;
    if (!is_UTC) {
        startT += TIME_SYNC_BOUNDARY;
        endT += TIME_SYNC_BOUNDARY;
    }
    
    fres = Log_FindEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == offsetEnd);
    test_assert(totalBytes == 0);

    using_RTC = 1; // Reset back to default: using RTC

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_FindEntryRange_All_UTC_Test() {    
    test_assert(Log_TestFixture_CheckSameTimeType(1) == TEST_PASS);
    
    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_FindEntryRange_All_OnBoardTime_Test() {
    test_assert(Log_TestFixture_CheckSameTimeType(0) == TEST_PASS);
    
    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_FindEntryRange_MixedTime_Test() {
    // Just INFO logs for now
    use_test_fixture(Log_TestFixture_ClearAllLogs);

    // set up file with entries as follows:
    // boot 0, on-board time
    // boot 1, UTC
    // boot 2, UTC
    // boot 3, on-board
    // boot 4, UTC,
    // boot 5, on-board
    // boot 6, on-board

    // Set RTC time
    HAL_StatusTypeDef RTC_stat;
    RTC_stat = RTC_X_SetTime(TIME_SYNC_BOUNDARY + 1000); errorcheck_HAL_ret(RTC_stat);

    // Set up array of time_t to record times of log writing

    time_t start_times[7];
    time_t end_times[7];

    char mystr[TEST_DEFAULT_DATA_SZ] = "hello";

    // boot 0
    EEPROM_BOOT_COUNT_MOCK = 0; // Update boot count
    using_RTC = 0; // on board time
    start_times[0] = xTaskGetTickCount() / configTICK_RATE_HZ;
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    end_times[0] = xTaskGetTickCount() / configTICK_RATE_HZ;

    // boot 1
    EEPROM_BOOT_COUNT_MOCK = 1; // Update boot count
    using_RTC = 1; // UTC time
    RTC_stat = RTC_GetTime(&start_times[1]); errorcheck_HAL_ret(RTC_stat);
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    RTC_stat = RTC_GetTime(&end_times[1]); errorcheck_HAL_ret(RTC_stat);

    // boot 2
    EEPROM_BOOT_COUNT_MOCK = 2; // Update boot count
    using_RTC = 1; // UTC time
    RTC_stat = RTC_GetTime(&start_times[2]); errorcheck_HAL_ret(RTC_stat);
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    RTC_stat = RTC_GetTime(&end_times[2]); errorcheck_HAL_ret(RTC_stat); 

    // boot 3
    EEPROM_BOOT_COUNT_MOCK = 3; // Update boot count
    using_RTC = 0; // on board time
    start_times[3] = xTaskGetTickCount() / configTICK_RATE_HZ;
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    end_times[3] = xTaskGetTickCount() / configTICK_RATE_HZ;

    // boot 4
    EEPROM_BOOT_COUNT_MOCK = 4; // Update boot count
    using_RTC = 1; // UTC time
    RTC_stat = RTC_GetTime(&start_times[4]); errorcheck_HAL_ret(RTC_stat);
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    RTC_stat = RTC_GetTime(&end_times[4]); errorcheck_HAL_ret(RTC_stat); 

    // boot 5
    EEPROM_BOOT_COUNT_MOCK = 5; // Update boot count
    using_RTC = 0; // on board time
    start_times[5] = xTaskGetTickCount() / configTICK_RATE_HZ;
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    end_times[5] = xTaskGetTickCount() / configTICK_RATE_HZ;

    // boot 6
    EEPROM_BOOT_COUNT_MOCK = 6; // Update boot count
    using_RTC = 0; // on board time
    start_times[6] = xTaskGetTickCount() / configTICK_RATE_HZ;
    RTC_stat = Log_WriteInfo(0, mystr, TEST_DEFAULT_DATA_SZ);
    test_assert(RTC_stat == HAL_OK);
    HAL_Delay(1000); // Busy-wait to make sure time increases
    end_times[6] = xTaskGetTickCount() / configTICK_RATE_HZ;

    // Size of one entry
    uint32_t entry_size = HEADER_SIZE + sizeof(uint8_t) + TEST_DEFAULT_DATA_SZ + sizeof(uint16_t);

    // For recording FindEntryRange outputs
    FSIZE_t offsetStart;
    FSIZE_t offsetEnd;
    uint32_t totalBytes;
    FIL fd;
    FRESULT fres;
    time_t startT;
    time_t endT;

    // arguments in UTC: valid range starts in boot 4, 
    // should not take boot 5 / 6
    startT = start_times[4];
    endT = start_times[4] + 1000;
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 4 * entry_size);
    test_assert(offsetEnd == 5 * entry_size);
    test_assert(totalBytes == entry_size);

    // Above case, but clipping to end, should take boot 4 - 6 entries
    endT = UINT64_MAX;
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 4 * entry_size);
    test_assert(offsetEnd == 7 * entry_size);
    test_assert(totalBytes == 3 * entry_size);

    // arguments in UTC: entries in boot 4 already too old, should not take anything
    startT = start_times[4] + 1000;
    endT = start_times[4] + 2000;
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(totalBytes == 0 * entry_size);

    // arguments in UTC: valid range starts in boot 1 and ends in boot 4, 
    // should take contiguous span of entries from boot 1 to boot 4 incl. boot 3
    startT = start_times[1];
    endT = end_times[4];
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == entry_size);
    test_assert(offsetEnd == 5 * entry_size);
    test_assert(totalBytes == 4 * entry_size);

    // arguments in UTC: valid range ends in boot 4 but last entry of boot 2 is already too old, 
    // should take appropriate entries in boot 4, but not boot 3
    startT = end_times[2];
    endT = end_times[4];
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 4 * entry_size);
    test_assert(offsetEnd == 5 * entry_size);
    test_assert(totalBytes == entry_size);

    // arguments in on-board time: current boot = 6 and range surrounds boot 6
    // Should take entry in boot 6
    startT = start_times[6];
    endT = end_times[6];
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 6 * entry_size);
    test_assert(offsetEnd == 7 * entry_size);
    test_assert(totalBytes == entry_size);

    // arguments in on-board time: current boot = 6 and range surrounds boot 5/6
    // Should only take entry in boot 6
    startT = start_times[5];
    endT = end_times[6];
    fres = Log_FindEntryRange(LOG_INFO, (const time_t*) &startT, (const time_t*) &endT, &offsetStart, &offsetEnd, &totalBytes, &fd);
    test_assert(fres == FR_OK);
    f_close(&fd);
    test_assert(offsetStart == 6 * entry_size);
    test_assert(offsetEnd == 7 * entry_size);
    test_assert(totalBytes == entry_size);

    EEPROM_BOOT_COUNT_MOCK = 0; // Reset boot count

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_ClearEntryRange_Test() {
    // This test is assuming FindEntryRange works
    // Create CRD logs
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    time_t start_times[6];
    test_assert(Log_TestFixture_SetupLogs(1, start_times, 6, LOG_CRD) == TEST_PASS);

    // For recording ClearEntryRange outputs
    uint32_t clearedBytes;
    FRESULT fres;
    time_t startT;
    time_t endT;

    // clear entry range at end of file
    startT = start_times[4];
    endT = start_times[5];
    
    fres = Log_X_ClearEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &clearedBytes);
    test_assert(fres == FR_OK);

    // Check timestamps in remaining entries
    FIL fd;
    f_open(&fd, LOG_STRINGS[LOG_CRD].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
    test_assert(f_size(&fd) == 4 * CRD_LOG_ENTRY_SIZE);
    test_assert(Log_TestFixture_CheckTimes(&fd, LOG_CRD, start_times, 4) == TEST_PASS);
    f_close(&fd);

    // clear two entries at end of file
    // Reset log file
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    test_assert(Log_TestFixture_SetupLogs(1, start_times, 6, LOG_CRD) == TEST_PASS);
    startT = start_times[3];
    endT = start_times[5];
    fres = Log_X_ClearEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &clearedBytes);
    test_assert(fres == FR_OK);

    // Check timestamps in remaining entries
    f_open(&fd, LOG_STRINGS[LOG_CRD].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
    test_assert(f_size(&fd) == 3 * CRD_LOG_ENTRY_SIZE);
    test_assert(Log_TestFixture_CheckTimes(&fd, LOG_CRD, start_times, 3) == TEST_PASS);
    f_close(&fd);
    // temp storage for list of times to check timestamps in file
    time_t times_4[4];
    time_t times_3[3];

    // clear one entries at front of file
    // Reset log file
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    test_assert(Log_TestFixture_SetupLogs(1, start_times, 6, LOG_CRD) == TEST_PASS);
    startT = start_times[0];
    endT = start_times[1];
    fres = Log_X_ClearEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &clearedBytes);
    test_assert(fres == FR_OK);

    // Check timestamps in remaining entries
    f_open(&fd, LOG_STRINGS[LOG_CRD].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
    test_assert(f_size(&fd) == 4 * CRD_LOG_ENTRY_SIZE);
    memcpy(times_4, &start_times[1], 4 * sizeof(time_t));
    test_assert(Log_TestFixture_CheckTimes(&fd, LOG_CRD, times_4, 4) == TEST_PASS);
    f_close(&fd);

    // clear two entries at front of file
    // Reset log file
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    test_assert(Log_TestFixture_SetupLogs(1, start_times, 6, LOG_CRD) == TEST_PASS);
    startT = start_times[0];
    endT = start_times[2];
    fres = Log_X_ClearEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &clearedBytes);
    test_assert(fres == FR_OK);

    // Check timestamps in remaining entries
    f_open(&fd, LOG_STRINGS[LOG_CRD].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
    test_assert(f_size(&fd) == 3 * CRD_LOG_ENTRY_SIZE);
    memcpy(times_3, &start_times[2], 3 * sizeof(time_t));
    test_assert(Log_TestFixture_CheckTimes(&fd, LOG_CRD, times_3, 3) == TEST_PASS);
    f_close(&fd);

    // clear one entry in the middle of file
    // Reset log file
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    test_assert(Log_TestFixture_SetupLogs(1, start_times, 6, LOG_CRD) == TEST_PASS);
    startT = start_times[1];
    endT = start_times[2];
    fres = Log_X_ClearEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &clearedBytes);
    test_assert(fres == FR_OK);

    // Check timestamps in remaining entries
    f_open(&fd, LOG_STRINGS[LOG_CRD].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
    test_assert(f_size(&fd) == 4 * CRD_LOG_ENTRY_SIZE);
    memcpy(times_4, start_times, sizeof(time_t));
    memcpy(&times_4[1], &start_times[2], 3 * sizeof(time_t));
    test_assert(Log_TestFixture_CheckTimes(&fd, LOG_CRD, times_4, 4) == TEST_PASS);
    f_close(&fd);

    // clear two entries in the middle of file
    // Reset log file
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    test_assert(Log_TestFixture_SetupLogs(1, start_times, 6, LOG_CRD) == TEST_PASS);
    startT = start_times[1];
    endT = start_times[3];
    fres = Log_X_ClearEntryRange(LOG_CRD, (const time_t*) &startT, (const time_t*) &endT, &clearedBytes);
    test_assert(fres == FR_OK);

    // Check timestamps in remaining entries
    f_open(&fd, LOG_STRINGS[LOG_CRD].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
    test_assert(f_size(&fd) == 3 * CRD_LOG_ENTRY_SIZE);
    memcpy(times_3, start_times, sizeof(time_t));
    memcpy(&times_3[1], &start_times[3], 2 * sizeof(time_t));
    test_assert(Log_TestFixture_CheckTimes(&fd, LOG_CRD, times_3, 3) == TEST_PASS);
    f_close(&fd);

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_TimeSyncCallback_Test() {
    time_t delta = 5;

    // All files in same boot
    // Create the Log files
    use_test_fixture(Log_TestFixture_ClearAllLogs);
    // Only five entries per log file (num times is start times + the ending time)
    size_t num_times = 6;
    time_t start_times[LOG_TYPES_NUM][num_times];
    // Write logs for each of 5 log files on the SD card
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        test_assert(Log_TestFixture_SetupLogs(1, start_times[i], num_times, i) == TEST_PASS);
    }
    // Adjust all timestamps by delta
    Log_X_TimeSyncCallback(delta); 
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        for (size_t j = 0; j < num_times; j++) {
            start_times[i][j] += delta;
        }
    }
    // Check all timestamps
    FIL fd;
    FRESULT fres;
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        fres = f_open(&fd, LOG_STRINGS[i].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
        // num_times - 1, since only considering start times and the last time is the ending time
        test_assert(Log_TestFixture_CheckTimes(&fd, i, start_times[i], num_times - 1) == TEST_PASS);
        f_close(&fd);
    }

    // No files in the current boot
    EEPROM_BOOT_COUNT_MOCK = 1;
    // Adjust all timestamps by delta
    Log_X_TimeSyncCallback(delta);
    // No timestamp should have changed
    // Check all timestamps
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        fres = f_open(&fd, LOG_STRINGS[i].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
        // num_times - 1, since only considering start times and the last time is the ending time
        test_assert(Log_TestFixture_CheckTimes(&fd, i, start_times[i], num_times - 1) == TEST_PASS);
        f_close(&fd);
    }

    // Some files in the current boot
    size_t num_times_addit = 4;
    time_t start_times_addit[LOG_TYPES_NUM][num_times_addit];
    // Write logs for each of 5 log files on the SD card
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        test_assert(Log_TestFixture_SetupLogs(1, start_times_addit[i], num_times_addit, i) == TEST_PASS);
    }
    // Adjust new timestamps by delta
    Log_X_TimeSyncCallback(delta); 
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        for (size_t j = 0; j < num_times_addit; j++) {
            start_times_addit[i][j] += delta;
        }
    }
    // Combine the two start time arrays
    time_t entries = num_times - 1 + num_times_addit - 1;
    time_t start_times_total[LOG_TYPES_NUM][entries];
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        memcpy(start_times_total[i], start_times[i], (num_times - 1) * sizeof(time_t));
        memcpy(start_times_total[i] + (num_times - 1), start_times_addit[i], (num_times_addit - 1) * sizeof(time_t));
    }
    // Check all timestamps
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        fres = f_open(&fd, LOG_STRINGS[i].filename, FA_READ | FA_WRITE | FA_OPEN_EXISTING); errorcheck_fr_ret(fres);
        test_assert(Log_TestFixture_CheckTimes(&fd, i, start_times_total[i], entries) == TEST_PASS);
        f_close(&fd);
    }

    return TEST_PASS;
}

/*
TestStatus_e Memory_Logs_Log_FindEntryRange_Test() {
    // Test CRD log
	// Clear CRD log
	FIL crd_fd; FRESULT fres;
	fres = f_open(&crd_fd, CRD_LOG_FILENAME, FA_WRITE);
	fres = f_truncate(&crd_fd);
	fres = f_close(&crd_fd); // for mocking, need to save the updated size

    // Get time before "writing" crd files
    HAL_StatusTypeDef myHALretstat; time_t crd_startT;
    Sys_TryDev(myHALretstat, RTC_GetTime(&crd_startT), HAL_OK, DEV_RTC);

    // Write three times to Log
    for (size_t n = 0; n < 3; n++) {
        CRD_Count_t rand_CRD_count;
        Log_WriteCRD(rand_CRD_count);
    }

    // Get time after "writing" crd files
    time_t crd_endT;
    Sys_TryDev(myHALretstat, RTC_GetTime(&crd_endT), HAL_OK, DEV_RTC);
    crd_endT += 1;

    // Check that the range has all the entries
    fres = f_open(&crd_fd, CRD_LOG_FILENAME, FA_WRITE);
    FSIZE_t crd_offsetStart, crd_offsetEnd;
    uint32_t crd_totalBytes;
    fres = Log_FindEntryRange(LOG_CRD, &crd_startT, &crd_endT, &crd_offsetStart, &crd_offsetEnd, &crd_totalBytes, &crd_fd);

    test_assert(fres == FR_OK);
    test_assert(crd_offsetStart == 0);
    test_assert(crd_offsetEnd == 3 * (CRD_LOG_ENTRY_SIZE));
    test_assert(crd_totalBytes == 3 * (CRD_LOG_ENTRY_SIZE));

    // Test Info Log
	// Clear Info log
	FIL info_fd;
	fres = f_open(&info_fd, INFO_LOG_FILENAME, FA_WRITE);
	fres = f_truncate(&info_fd);
	fres = f_close(&info_fd); // for mocking, need to save the updated size

    // Get time before "writing" info files
    time_t info_startT;
    Sys_TryDev(myHALretstat, RTC_GetTime(&info_startT), HAL_OK, DEV_RTC);

    uint8_t comp_checks_payload[49];
    Log_WriteInfo(INFO_ENTRY_COMPCHECKS, comp_checks_payload, 49);
    uint8_t power_level_payload[1];
    Log_WriteInfo(INFO_ENTRY_PWRLVL, power_level_payload, 1);
    osDelay(1000);

    // Get time in the midst of "writing" info files
    time_t info_midT;
    Sys_TryDev(myHALretstat, RTC_GetTime(&info_midT), HAL_OK, DEV_RTC);
    osDelay(1000);

    uint8_t entry_gndpass_payload[9];
    Log_WriteInfo(INFO_ENTRY_GNDPASS, entry_gndpass_payload, 9);
    osDelay(1000);

    // Get time after "writing" info files
    time_t info_endT;
    Sys_TryDev(myHALretstat, RTC_GetTime(&info_endT), HAL_OK, DEV_RTC);

    fres = f_open(&info_fd, INFO_LOG_FILENAME, FA_WRITE);
    FSIZE_t info_offsetStart, info_offsetEnd;
    uint32_t info_totalBytes;
    fres = Log_FindEntryRange(LOG_INFO, &info_startT, &info_endT, &info_offsetStart, &info_offsetEnd, &info_totalBytes, &info_fd);

    test_assert(fres == FR_OK);
    test_assert(info_offsetStart == 0);
    test_assert(info_offsetEnd == 3 * (17 + 2) + 49 + 1 + 9);
    test_assert(info_totalBytes == 3 * (17 + 2) + 49 + 1 + 9);

    FSIZE_t firstHalf_offsetStart, firstHalf_offsetEnd;
    uint32_t firstHalf_totalBytes;
    f_lseek(&info_fd, 0);
    fres = Log_FindEntryRange(LOG_INFO, &info_startT, &info_midT, &firstHalf_offsetStart, &firstHalf_offsetEnd, &firstHalf_totalBytes, &info_fd);

    FSIZE_t secondHalf_offsetStart, secondHalf_offsetEnd;
    uint32_t secondHalf_totalBytes;
    f_lseek(&info_fd, 0);
    fres = Log_FindEntryRange(LOG_INFO, &info_midT, &info_endT, &secondHalf_offsetStart, &secondHalf_offsetEnd, &secondHalf_totalBytes, &info_fd);

    test_assert(firstHalf_totalBytes + secondHalf_totalBytes == info_totalBytes);
    test_assert(firstHalf_offsetStart == 0);
    test_assert(secondHalf_offsetEnd == info_offsetEnd);

    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_X_TimeSyncCallback_Test() {
    // Clear all log files
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        FIL fno;
        f_open(&fno, LOG_STRINGS[i].filename, FA_WRITE | FA_OPEN_EXISTING);
        f_lseek(&fno, 0); // Move offset to beginning of file
        f_truncate(&fno); // Truncate all file contents
    }
    HAL_StatusTypeDef myHALretstat; 
    time_t current_time;
    Sys_TryDev(myHALretstat, RTC_GetTime(&current_time), HAL_OK, DEV_RTC);
    time_t boot_times[] = {current_time - 1000000, current_time - 500000, current_time};
    time_t entry_times[10];
    // Add first boot time to boot log file
    uint16_t resetReasonMask = 0;
    Log_WriteBoot(&boot_times[0], resetReasonMask);
    // Add entries to info log file
    for (int n = 0; n < 10; n++) {
        if (n == 0) {
            *debug_time = boot_times[0];
        }
        if (n == 4) {
            *debug_time = boot_times[1];
        }
        if (n == 7) {
            *debug_time = boot_times[2]; // Last three entries part of current boot
        }
        entry_times[n] = *debug_time;
        if (n % 3 == 0) {
            uint8_t comp_checks_payload[49]; // Empty payloads
            Log_WriteInfo(INFO_ENTRY_COMPCHECKS, comp_checks_payload, 49);
        } else if (n % 2 == 1) {
            uint8_t power_level_change_payload[1];
            Log_WriteInfo(INFO_ENTRY_PWRLVL, power_level_change_payload, 1);
        } else {
            uint8_t ground_pass_update_payload[9];
            Log_WriteInfo(INFO_ENTRY_GNDPASS, ground_pass_update_payload, 9);
        }
        *debug_time += 10000;
    }
    Log_WriteBoot(&boot_times[1], resetReasonMask);
    // Add entries to warn log file
    for (int n = 0; n < 10; n++) {
        *debug_time = entry_times[n];
        if (n % 3 == 0) {
            uint8_t temperature_payload[2];
            Log_WriteWarn(WARN_ENTRY_TEMPERATURE, temperature_payload, 2);
        } else if (n % 2 == 1) {
            uint8_t *radio_kill_payload = NULL;
            Log_WriteWarn(WARN_ENTRY_RADIOKILL, radio_kill_payload, 0);
        } else {
            uint8_t device_warning_payload[3];
            Log_WriteWarn(WARN_ENTRY_DEVWARN, device_warning_payload, 3);
        }
    }
    Log_WriteBoot(&boot_times[2], resetReasonMask);
    // Add entries to critical log file
    for (int n = 0; n < 10; n++) {
        *debug_time = entry_times[n];
        if (n % 3 == 0) {
            uint8_t temperature_payload[2];
            Log_WriteCrit(CRIT_ENTRY_TEMPERATURE, temperature_payload, 2);
        } else if (n % 2 == 1) {
            uint8_t *antenna_failure_payload = NULL;
            Log_WriteCrit(CRIT_ENTRY_ANTDEPL_FAIL, antenna_failure_payload, 0);
        } else {
            uint8_t eps_reset_payload[1];
            Log_WriteCrit(CRIT_ENTRY_EPSRST, eps_reset_payload, 1);
        }
    }
    // Add entries to CRD log file
    for (int i = 0; i < 3; i++) {
       *debug_time = boot_times[i];
        uint32_t counts[8];
        Log_WriteCRD(counts); // Only last count is in current boot
    }
    // Call Log_X_TimeSyncCallback and verify that timestamps are updated correctly
    time_t delta = 123456; 
    Log_X_TimeSyncCallback(delta);
    time_t entryTime;
    uint32_t currBootCt = EEPROM_GetBootCount(), entryBootCt;
    for (Log_Type_e i = 0; i < LOG_TYPES_NUM; i++) {
        int n = 9; 
        time_t *times = entry_times;
        if (i == LOG_BOOT || i == LOG_CRD) {
            n = 2; 
            times = boot_times;
        }
        FIL fd;
        f_open(&fd, LOG_STRINGS[i].filename, FA_OPEN_EXISTING | FA_READ);
        f_lseek(&fd, f_size(&fd));
        while (f_tell(&fd) > 0) {
            prevEntry(&fd, i, &entryTime, &entryBootCt);
            // fprintf(DEBUG_COMM, "%ld %ld %ld %ld\n", (long long) entryTime, (long long) times[n], (long long) entryBootCt, (long long) currBootCt);
            if (entryBootCt == currBootCt) {
                if (entryTime != times[n] + delta){
                	return TEST_FAIL;
                }
            } else {
                if (entryTime != times[n]) return TEST_FAIL;
            }
            n--;
        }
    }
    return TEST_PASS;
}

TestStatus_e Memory_Logs_Log_X_ClearEntryRange_Test() {
    // Clear CRD log
	FIL crd_fd; FRESULT fres;
	fres = f_open(&crd_fd, CRD_LOG_FILENAME, FA_WRITE);
	fres = f_truncate(&crd_fd);
	fres = f_close(&crd_fd); // for mocking, need to save the updated size

    //Testing clear CDR log
    // Write 3 records to CRD log
    // Find the range of the middle records
    // Check that the range is now the first and last record

    // Get time before "writing" crd records
    HAL_StatusTypeDef myHALretstat; 
    time_t start_CRD_write_time;
    Sys_TryDev(myHALretstat, RTC_GetTime(&start_CRD_write_time), HAL_OK, DEV_RTC);
    osDelay(1000);

    CRD_Count_t rand_CRD_count;
    Log_WriteCRD(rand_CRD_count);
    osDelay(1000);
    time_t after_CRD1_write_Time;
    Sys_TryDev(myHALretstat, RTC_GetTime(&after_CRD1_write_Time), HAL_OK, DEV_RTC);

    Log_WriteCRD(rand_CRD_count);
    osDelay(1000);
    time_t after_CRD2_write_Time;
    Sys_TryDev(myHALretstat, RTC_GetTime(&after_CRD2_write_Time), HAL_OK, DEV_RTC);

    Log_WriteCRD(rand_CRD_count);
    osDelay(1000);
    time_t end_CRD_Write_Time;
    Sys_TryDev(myHALretstat, RTC_GetTime(&end_CRD_Write_Time), HAL_OK, DEV_RTC);

    // Check that the range has all the entries
    fres = f_open(&crd_fd, CRD_LOG_FILENAME, FA_WRITE);
    FSIZE_t crd_offsetStart, crd_offsetEnd;
    uint32_t crd_totalBytes;
    fres = Log_FindEntryRange(LOG_CRD, &after_CRD1_write_Time, &after_CRD2_write_Time, &crd_offsetStart, &crd_offsetEnd, &crd_totalBytes, &crd_fd);

    
    if (fres != FR_OK) { // any error means test fail
        return TEST_FAIL;
    }
    if (crd_offsetStart != (CRD_LOG_ENTRY_SIZE)) { // should be after first record
        return TEST_FAIL;
    }
    if (crd_offsetEnd != 2 * (CRD_LOG_ENTRY_SIZE)) { // should be after the second record
        return TEST_FAIL;
    }
    if (crd_totalBytes != (CRD_LOG_ENTRY_SIZE)) { // should be the size of one record
        return TEST_FAIL;
    }

    // Clear the range
    uint32_t clearedBytes;
    fres = Log_X_ClearEntryRange(LOG_CRD, &after_CRD1_write_Time, &after_CRD2_write_Time, &clearedBytes);        
    if(fres!=FR_OK){
        return TEST_FAIL;
    }
    
    // Check that the range has all the remaining 2 entries
    fres = Log_FindEntryRange(LOG_CRD, &start_CRD_write_time, &end_CRD_Write_Time, &crd_offsetStart, &crd_offsetEnd, &crd_totalBytes, &crd_fd);
    if (fres != FR_OK) { // any error means test fail
        return TEST_FAIL;
    }
    if (crd_offsetStart != 0) { // should be after first record
        return TEST_FAIL;
    }
    if (crd_offsetEnd != 2 * (CRD_LOG_ENTRY_SIZE)) { // should be after the second record
        return TEST_FAIL;
    }
    if (crd_totalBytes != 2 * (CRD_LOG_ENTRY_SIZE)) { // should be the size of two records
        return TEST_FAIL;
    }

    //TODO: Test other log types (info, warn, crit, debug)
    return TEST_PASS;
}
*/
#endif



//TestStatus_e Memory_Logs_Warn_Test()
//{
    /* Definition of test function */
	// want to test by different types for now

	// Subtest 1: warn sequence
	// u_int32_t byteSequence = 0x5741524E;
	// u_int32_t byte_seq_payload[4];
	// memcpy(byte_seq_payload, &byteSequence, 4);
	// Log_WriteWarn(INFO_ENTRY_COMPCHECKS, &byteSequence, 4);
	// if (Log_TestFixture_CheckDebugEntry(&byteSequence, 4) == TEST_FAIL) {
	// 	return TEST_FAIL;
	// }

	// // Subtest 2: RTC time, return Current on-board time as an 8-byte Unix timestamp.
	// time_t currT;
	// // ...

	// //Subtest 3: Entry type (uint8_t)
	// uint8_t entryType = 0x01;
	// uint8_t entry_type_payload[1];
	// memcpy(entry_type_payload, &entryType, 1);
	// Log_WriteWarn(INFO_ENTRY_COMPCHECKS, &entryType, 1);
	// if (Log_TestFixture_CheckDebugEntry(&entryType, 1) == TEST_FAIL) {
	// 	return TEST_FAIL;
	// }

	// // Subtest 4: entry payload (variable length. see .h file and google doc for details on what the entry type specs are.)
	// // for now i will just write the power level one.
	// u_int8_t power_level = 0x01;
	// u_int8_t power_level_payload[1];
	// memcpy(power_level_payload, &power_level, 1);
	// Log_WriteWarn(INFO_ENTRY_COMPCHECKS, &power_level, 1);
	// if (Log_TestFixture_CheckDebugEntry(&power_level, 1) == TEST_FAIL) {
	// 	return TEST_FAIL;
	// }

	// // Subtest 5: entry size (uint16_t)
	// u_int16_t entrySize = 0x0001;
	// u_int16_t entry_size_payload[2];
	// memcpy(entry_size_payload, &entrySize, 2);
	// Log_WriteWarn(INFO_ENTRY_COMPCHECKS, &entrySize, 2);
	// if (Log_TestFixture_CheckDebugEntry(&entrySize, 2) == TEST_FAIL) {
	// 	return TEST_FAIL;
	// }
// }
