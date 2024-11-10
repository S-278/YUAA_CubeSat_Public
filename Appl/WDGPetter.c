/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file WDGPetter.c
* @brief Watchdog Petter C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B,
* @details           Defines temporary watchdog petter
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "system_manager.h"
#include "MCU_Init.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "stm32f4xx_hal_wwdg.h"
#include "stm32f4xx_hal_iwdg.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* The WWDG window is initialized to between 20ms and 80ms in MUC_Init.c,
   so refresh it every 50ms. */
#define WWDG_REFRESH_PERIOD_MS (50)
/* IWDG period is initialized to 30sec in MCU_Init.c, 
   so refresh it every 20sec. */
#define IWDG_REFRESH_PERIOD_TICK (14 * configTICK_RATE_HZ)

#define errorcheck(retstat) assert(retstat == HAL_OK)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

void WDG_PetterTask(void* argument)
{
    /* This is a temporary, maximally simple watchdog solution,
       intended to be used while EnduroSat TaskMonitor is being re-implemented.
       This task must have the highest priority in the system.
       It wakes up on a period equal to the middle of the WWDG window
       and refreshes the WWDG. When this task's wakeup also coincides with the
       IWDG window, it refreshes the IWDG too. */

    MX_IWDG_Init();
    MX_WWDG_Init();

    TickType_t lastIWDG_Tick = xTaskGetTickCount();
    HAL_StatusTypeDef retstat;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(WWDG_REFRESH_PERIOD_MS));

        retstat = HAL_WWDG_Refresh(&hwwdg); errorcheck(retstat);

        if (xTaskGetTickCount() - lastIWDG_Tick > IWDG_REFRESH_PERIOD_TICK)
        {
            retstat = HAL_IWDG_Refresh(&hiwdg); errorcheck(retstat);
            lastIWDG_Tick = xTaskGetTickCount();
        }
    }
}
