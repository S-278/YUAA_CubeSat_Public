/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include "system_manager.h"
#include "AppTasks.h"
#include "EEPROM_emul.h"
#include "ESTTC.h"
#include "main.h"
#include "MCU_Init.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "cmsis_gcc.h"
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define printToAll(format, ...) { fprintf(COMM, format, __VA_ARGS__); \
                                  fprintf(SYSCON, format, __VA_ARGS__);\
                                  fprintf(PAYLOAD, format, __VA_ARGS__); }

#define errorcheck_ret(retstat, success_val) if (retstat != success_val) return retstat

/* When SysMan callbacks are invoked,
if extra parameters are needed, they are put in the mailbox. */
#define SYSMAN_MAILBOX_SZ (sizeof(uint32_t)*4)
_Static_assert(sizeof(char*) + sizeof(int) + SYS_ERR_MAX_LEN <= SYSMAN_MAILBOX_SZ, "Inconsistent SysMan mailbox sz");

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

typedef struct {
    const char* const name;
    TaskHandle_t handle;
    const uint32_t stackDepth; // number of words (not bytes!) in the stack
    const StackType_t* const stackPtr;
    StaticTask_t data;
    const UBaseType_t prio;
    const TaskFunction_t funct;
    // bit i of devsReliant is 1 iff this task relies on
    // the hardware device (DevNames_e)(i)
    const uint32_t devsReliant;
    const Sys_PowerLevel_e minPwrLvl;
} RTOS_TaskInfo_t;

#define isTaskReliant(task, dev) ((RTOS_TaskInfo_t)task.devsReliant & (1 << (DevName_e)dev))
#define relyDev(dev) (1 << (DevName_e)dev)

// note - this macro uses the stringizing and token-pasting preprocessor operators
#define TaskInfo_Initializer(task, _prio, _funct, _devsReliant, _minPwrLvl) \
{                                                                       \
    .name = #task,                                                      \
    .handle = NULL,                                                     \
    .stackDepth = sizeof(task##_StackBuf) / sizeof(StackType_t),        \
    .stackPtr = task##_StackBuf,                                        \
    .prio = _prio,                                                       \
    .funct = _funct,                                                     \
    .devsReliant = _devsReliant,                                         \
    .minPwrLvl = _minPwrLvl,                                             \
}

typedef enum {
    REQ_DEV_ERR,
    REQ_LOGIC_ERR,
    REQ_IAMALIVE,
    // TODO etc. for each callback in the header
} SysMan_Request_e;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DEFINITION/DECLARATION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static StackType_t TASK_MONITOR_StackBuf[configMINIMAL_STACK_SIZE];
static StackType_t SYSMAN_StackBuf[1024];
static StackType_t RADIO_CTL_StackBuf[1024];
static StackType_t ADCS_CTL_StackBuf[1024+512];
static StackType_t ESTTC_UART_StackBuf[2048];
static StackType_t LOG_WRITER_StackBuf[512];
static StackType_t ADCS_DETUMB_StackBuf[512];
static StackType_t ADCS_GGB_StackBuf[512];
static StackType_t ADCS_MEAS_StackBuf[1024];
static StackType_t PWRDIST_StackBuf[1024];
static StackType_t RADIO_AUTODL_StackBuf[1024+512];
static StackType_t SERVICES_StackBuf[configMINIMAL_STACK_SIZE];
static StackType_t CRD_DATA_StackBuf[512];
static StackType_t SCRUBBER_StackBuf[configMINIMAL_STACK_SIZE];
static StackType_t INIT_StackBuf[2048];
static StackType_t IDLE_StackBuf[configMINIMAL_STACK_SIZE];
static StackType_t TIMER_StackBuf[configTIMER_TASK_STACK_DEPTH];
#ifdef DEBUG_ENABLED
static StackType_t DEFAULT_StackBuf[2048];
#endif


static StackType_t* const StackBufs[NUM_RTOS_TASKS] = {
    [TASK_TASK_MONITOR] = TASK_MONITOR_StackBuf,
    [TASK_SYSMAN] = SYSMAN_StackBuf,
    [TASK_RADIO_CTL] = RADIO_CTL_StackBuf,
    [TASK_ADCS_CTL] = ADCS_CTL_StackBuf,
    [TASK_ESTTC_UART] = ESTTC_UART_StackBuf,
    [TASK_LOG_WRITER] = LOG_WRITER_StackBuf,
    [TASK_ADCS_DETUMB] = ADCS_DETUMB_StackBuf,
    [TASK_ADCS_GGB] = ADCS_GGB_StackBuf,
    [TASK_ADCS_MEAS] = ADCS_MEAS_StackBuf,
    [TASK_PWRDIST] = PWRDIST_StackBuf,
    [TASK_RADIO_AUTODL] = RADIO_AUTODL_StackBuf,
    [TASK_SERVICES] = SERVICES_StackBuf,
    [TASK_CRD_DATA] = CRD_DATA_StackBuf,
    [TASK_SCRUBBER] = SCRUBBER_StackBuf,
    [TASK_INIT] = INIT_StackBuf,
};

static void SysMan_Task(void* arg);
static void SysInit_Task(void* arg);
// Temporary declaration while we don't have a proper task monitor
// Function defined in `WDGPetter.c'
void WDG_PetterTask(void* argument);

RTOS_TaskInfo_t RTOS_TASKS[NUM_RTOS_TASKS] = {
    [TASK_TASK_MONITOR] = TaskInfo_Initializer(TASK_MONITOR, 9, WDG_PetterTask, 0, L0),
    [TASK_SYSMAN] = TaskInfo_Initializer(SYSMAN, 8, SysMan_Task, 0, L0),
    [TASK_ESTTC_UART] = TaskInfo_Initializer(ESTTC_UART, 5, ESTTC_UART_TASK, 0, L2),
    [TASK_SERVICES] = TaskInfo_Initializer(SERVICES, 2, ServicesTask, 0, L0),
    [TASK_INIT] = TaskInfo_Initializer(INIT, 7, SysInit_Task, 0, L0),
};

// Static data for FreeRTOS built-in tasks,
// which do not participate in SysMan's task infrastructure
static StaticTask_t IDLE_data, TIMER_data;
#ifdef DEBUG_ENABLED
static StaticTask_t DEFAULT_data;
#endif

static QueueHandle_t SysMan_mailbox_handle;
static uint8_t SysMan_mailbox_storage[SYSMAN_MAILBOX_SZ]; // Left-justified
static StaticQueue_t SysMan_mailbox_obj;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// *~~~~~~~~~~~~~~~~~ SOFTWARE RELATED METHODS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

static void startTask(RTOS_TASKS_e task, void* task_arg)
{
    assert(task < NUM_RTOS_TASKS);
    RTOS_TaskInfo_t* const taskInfo = &(RTOS_TASKS[task]);
    assert(taskInfo->handle == NULL); // should not be starting a task that is already running
    taskInfo->handle =
        xTaskCreateStatic(taskInfo->funct, taskInfo->name, taskInfo->stackDepth,
            task_arg,
            taskInfo->prio, StackBufs[task], &(taskInfo->data));
}

static void stopTask(RTOS_TASKS_e task)
{
    assert(task < NUM_RTOS_TASKS);
    RTOS_TaskInfo_t* const taskInfo = &(RTOS_TASKS[task]);
    assert(taskInfo->handle != NULL); // should not be trying to stop a task that is already not running
    vTaskDelete(taskInfo->handle);
    taskInfo->handle = NULL;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INITIALIZATION DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
_Noreturn void Sys_X_Init()
{
    // Create essential tasks
    startTask(TASK_SYSMAN, NULL);
    startTask(TASK_TASK_MONITOR, NULL);
    startTask(TASK_INIT, NULL);
    // Noreturn start RTOS
    vTaskStartScheduler();
    // Should never be reached!
    assert(0);
}

void SysInit_Task(void* arg)
{
    // Essential peripherals init
    EEPROM_Emul_Init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_RTC_Init();
    MX_I2C1_Init();

#if !defined(DEBUG_ENABLED) || defined(DEBUG_SD)
    // Init SD card
    InitCdCard();
#endif
    // Rest of peripherals init
    // Skip ADC and FMC - currently not used
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_SPI1_Init();
    MX_TIM5_Init();
    MX_USART1_Init();
    MX_USART6_Init();
    // Skip other UARTs - currently not used

#ifdef DEBUG_ENABLED
    ESTTC_PrintVersion(DEBUG_COMM);
#endif

#ifdef DEBUG_ENABLED
    debug_msg("%s", "OK+OBC initialization completed");
    xTaskCreateStatic(StartDefaultTask, "DEFAULT",
        sizeof(DEFAULT_StackBuf) / sizeof(StackType_t),
        NULL, 0,
        DEFAULT_StackBuf, &DEFAULT_data);
    Sys_DebugTaskStart(TASK_ESTTC_UART);
#endif

    // This task deletes itself
    vTaskDelete(NULL);
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED CALLBACKS DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Generic handler for all SysMan callbacks.
If `mail' is not NULL, will copy `SYSMAN_MAILBOX_SZ' bytes into the SysMan mailbox.
In any case, will then signal the SysMan task with `req'. */
static void callbackHandler(SysMan_Request_e req, const void* mail) {
    if (mail != NULL) xQueueSendToBack(SysMan_mailbox_handle, mail, 0);
    xTaskNotify(RTOS_TASKS[TASK_SYSMAN].handle, req, eSetValueWithOverwrite);
}

/* Generic function to wait for a request from a task.
Abstracts away the FreeRTOS implementation of requests and mail.
Returns the reuest type and populates mail with any mail received. */
static SysMan_Request_e waitForRequest(uint8_t mail[SYSMAN_MAILBOX_SZ])
{
    uint32_t retstat;
    if (
        xTaskNotifyWait(0, 0, &retstat, portMAX_DELAY)
        == pdTRUE)
    {
        xQueueReceive(SysMan_mailbox_handle, mail, 0);
        return (SysMan_Request_e)retstat;
    }
    else panic("%s", "Unexpected SysMan wakeup");
    return 0; //Never reached
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#ifdef DEBUG_ENABLED

void Sys_DebugTaskStart(RTOS_TASKS_e task)
{
    startTask(task, 0);
}

void Sys_DebugTaskStop(RTOS_TASKS_e task)
{
    stopTask(task);
}

static uint8_t S_expectLogicErr = 0;
static const char* S_expectLogicErr_File;
static int S_expectLogicErr_Line;
static uint8_t* S_expectLogicErr_CB;
void Sys_ExpectLogicError(const char* file, int line, uint8_t* callback)
{
    S_expectLogicErr = 1;
    S_expectLogicErr_File = file;
    S_expectLogicErr_Line = line;
    S_expectLogicErr_CB = callback;
}

static uint8_t S_expectDevErr = 0;
static DevName_e S_expectDevErr_Dev;
static uint8_t S_expectDevErr_filterCode = 0;
static unsigned int S_expectDevErr_Code;
static uint8_t* S_expectDevErr_CB;
void Sys_ExpectDevError(DevName_e dev, const unsigned int* errorStatus, uint8_t* callback)
{
    S_expectDevErr = 1;
    S_expectDevErr_Dev = dev;
    if (errorStatus != NULL)
    {
        S_expectDevErr_filterCode = 1;
        S_expectDevErr_Code = *errorStatus;
    }
    else S_expectDevErr_filterCode = 0;
    S_expectDevErr_CB = callback;
}

__WEAK DevStat_e Sys_GetDevStatus(DevName_e dev)
{
    return DEV_STAT_NORMAL;
}
#endif

void Sys_IAmAlive()
{
    TaskHandle_t callerHandle = xTaskGetCurrentTaskHandle();
    callbackHandler(REQ_IAMALIVE, &callerHandle);
}

void _Sys_RaiseLogicError(const char* file, int line, LogicError_Behavior_e behavior, const char* format, ...)
{
    char mail[SYSMAN_MAILBOX_SZ];
    memcpy(mail, &file, sizeof(file));
    memcpy(mail + sizeof(file), &line, sizeof(line));
    uint8_t _behavior = behavior;
    memcpy(mail + sizeof(file) + sizeof(line), &_behavior, sizeof(_behavior));
    va_list vl; va_start(vl, format);
    vsnprintf(mail + sizeof(file) + sizeof(line) + sizeof(_behavior), SYS_ERR_MAX_LEN, format, vl);
    va_end(vl);

    callbackHandler(REQ_LOGIC_ERR, mail);
}

void Sys_RaiseDevError(DevName_e dev, unsigned int errorStatus)
{
    uint8_t mail[SYSMAN_MAILBOX_SZ];
    memcpy(mail, &dev, sizeof(dev));
    memcpy(mail + sizeof(dev), &errorStatus, sizeof(errorStatus));

    callbackHandler(REQ_DEV_ERR, mail);
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Provide the FreeRTOS kernel with statically allocated memory for the Idle & Timer tasks
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @note              Funcitons provided for use by FreeRTOS kernel
*                    since dynamic memory is disabled.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
    StackType_t** ppxIdleTaskStackBuffer,
    uint32_t* pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &IDLE_data; *ppxIdleTaskStackBuffer = IDLE_StackBuf; *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
    StackType_t** ppxTimerTaskStackBuffer,
    uint32_t* pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer = &TIMER_data; *ppxTimerTaskStackBuffer = TIMER_StackBuf; *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TASK FUNCTION DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static void SysMan_Task(void* arg)
{
    // TODO at some appropriate point in SysMan task init
    // Init mailbox
    SysMan_mailbox_handle = xQueueCreateStatic(1, SYSMAN_MAILBOX_SZ, SysMan_mailbox_storage, &SysMan_mailbox_obj);

    SysMan_Request_e request; uint8_t mail[SYSMAN_MAILBOX_SZ];
    while (1)
    {
        // Sleep until request comes in 
        request = waitForRequest(mail);
        // Deal with request
        switch (request)
        {
        case REQ_LOGIC_ERR:
        {
            char* file;
            memcpy(&file, mail, sizeof(file));
            int line;
            memcpy(&line, mail + sizeof(file), sizeof(line));
            uint8_t _behavior;
            memcpy(&_behavior, mail + sizeof(file) + sizeof(line), sizeof(_behavior));
            LogicError_Behavior_e behavior = _behavior;
            char *msg = (char*)(mail + sizeof(file) + sizeof(line) + sizeof(_behavior));
#ifdef DEBUG_ENABLED
            if (S_expectLogicErr && strcmp(S_expectLogicErr_File, file) == 0 && (S_expectLogicErr_Line == 0 || S_expectLogicErr_Line == line))
            {
                if (S_expectLogicErr_CB != NULL) *S_expectLogicErr_CB = 1;
                debug_L3("Expected logic err @ %s:%d %s", file, line, msg);
                S_expectLogicErr = 0; S_expectLogicErr_File = NULL;
            }
            else 
            {
                switch (behavior)
                {
                case LERR_WARN:
                    debug_L1("Logic err: warn @ %s:%d %s", file, line, msg);
                    break;
                case LERR_FATAL:
                default:
                    panic("Logic err: fatal @ %s:%d: %s", file, line, msg);
                }
            }
#else
#endif
            break;
        }
        case REQ_DEV_ERR:
        {
            DevName_e dev;
            memcpy(&dev, mail, sizeof(dev));
            unsigned int errorStatus;
            memcpy(&errorStatus, mail + sizeof(dev), sizeof(errorStatus));
#ifdef DEBUG_ENABLED
            if (S_expectDevErr
                && S_expectDevErr_Dev == dev
                && (!S_expectDevErr_filterCode || S_expectDevErr_Code == errorStatus))
            {
                if (S_expectDevErr_CB != NULL) *S_expectDevErr_CB = 1;
                debug_L3("Expected dev err for 0x%.2X : 0x%.2X", (unsigned int)dev, errorStatus);
                S_expectDevErr = 0;
            }
            else panic("dev err raised for 0x%.2X : 0x%.2X", (unsigned int)dev, errorStatus);
#else
#endif
            break;
        }
        case REQ_IAMALIVE:
        {
            TaskHandle_t callingHandle;
            memcpy(&callingHandle, mail, sizeof(callingHandle));

            RTOS_TASKS_e callingTask = 0;
            // Linear search through all tasks on the system 
            // to convert FreeRTOS handle into enum value. 
            // There are few enough where the linear search shouldn't be a problem. 
            for (; callingTask < NUM_RTOS_TASKS; callingTask++)
            {
                if (RTOS_TASKS[callingTask].handle == callingHandle) break;
            }
            if (callingTask < NUM_RTOS_TASKS) // found
            {
#ifdef DEBUG_ENABLED
                debug_L3("IAmAlive from %s", RTOS_TASKS[callingTask].name);
#else
#endif
            }
            else // not found - really shouldn't happen
            {
#ifdef DEBUG_ENABLED
                panic("Unrecognized handle %p at IAmAlive", callingHandle);
#else
#endif
            }
            break;
        }
        default:
            panic("Unexpected SysMan req %d", request);
        }
    }
}
