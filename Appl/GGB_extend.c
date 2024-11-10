/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file GGB_extend.c
* @brief GGB extension C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @details           Defines GGB extension task & ISR
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "GGB_extend.h"
#include "system_manager.h"
#include "MCU_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f4xx_hal_exti.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Maximum time for which to wait for an encoder IRQ 
when the GGB is supposed to be moving. */
#define GGB_ENC_IRQ_TO 5000             

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL VARIABLES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
uint8_t G_GGB_atTarget = 1; // Very unsafe global for testing only!

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static SemaphoreHandle_t S_GGB_Enc_IRQ_Semphr = NULL;
static StaticSemaphore_t S_GGB_Enc_IRQ_Semphr_Storage;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void EXTI9_5_IRQHandler()
{
    if (HAL_EXTI_GetPending(&hexti_GGB_Enc, EXTI_TRIGGER_RISING_FALLING))
    {
        // First, give the encoder IRQ semaphore to the GGB extension task
        BaseType_t requestContextSwitch = pdFALSE;
        BaseType_t giveStatus;
        giveStatus = xSemaphoreGiveFromISR(S_GGB_Enc_IRQ_Semphr, &requestContextSwitch);
        assert(giveStatus == pdTRUE);
        // Second, inform HAL of the IRQ
        HAL_EXTI_IRQHandler(&hexti_GGB_Enc);
        // End by forcing a context switch if approriate
        portYIELD_FROM_ISR(requestContextSwitch);
    }
    else panic("%s", "Unexpected EXTI9-5 IRQ");
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void GGBext_X_TaskSetup()
{
    assert(S_GGB_Enc_IRQ_Semphr == NULL);
    S_GGB_Enc_IRQ_Semphr = xSemaphoreCreateBinaryStatic(&S_GGB_Enc_IRQ_Semphr_Storage);
}

void GGBext_X_Task(void *arg)
{
    // Any init necessary before GGB movement goes here

    // Main task loop
    while (1)
    {
        // Wait for rotational encoder IRQ
        assert(S_GGB_Enc_IRQ_Semphr);
        BaseType_t gotSemphr = xSemaphoreTake(S_GGB_Enc_IRQ_Semphr, pdMS_TO_TICKS(GGB_ENC_IRQ_TO));

        // Determine whether IRQ arrived within TO
        if (gotSemphr == pdTRUE)
        {
            // If yes, update extension level and then compare against target
            debug_msg("%s", "Got semaphore");
            if (G_GGB_atTarget) break;
            else continue;
        }
        else 
        {
            // If no, error!
            debug_msg("%s", "Semaphore TO");
            break;
        }

    }

    // Cleanup e.g. turning off motor goes here

    // Inform ADCS Controller and delete yourself
    debug_msg("%s", "Deleting myself");
    vTaskDelete(NULL);
}

void GGBext_X_TaskCleanup()
{
    assert(S_GGB_Enc_IRQ_Semphr);
    vSemaphoreDelete(S_GGB_Enc_IRQ_Semphr);
    S_GGB_Enc_IRQ_Semphr = NULL;
}
