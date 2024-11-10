/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file power_dist.c
* @brief Power Distribution C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elena W., Nich S., Anton M.
* @details           Defines power distribution routine
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "power_dist.h"
#include "EPS.h"
#include "SunDetect.h"
#include "system_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef DEBUG_ENABLED
#include "AppTasks.h"
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Number of data points in circular power consumption buffer.
Every data point represents 5 minutes of measurements, 
buffer goes back 90 min. */
#define CONSBUF_NUM_ELTS (90/5) 
/* Minimum number of data points which must be in the buffer
for base level determination algorithm to run. 
For now, demand the entire buffer. */
#define CONSBUF_MIN_ELTS CONSBUF_NUM_ELTS

#define EPS_CAPACITY (10.2f * 60) // In units of watt-minutes (W*min) TBR!
#define EPS_CRIT_SOC (0.05f) // Critical battery threshold in percent. TBR!
/* Lowest power level the distribution algorithm can ordinarily return. TBR! */
#define GLOBAL_PWRLVL_FLOOR ((Sys_PowerLevel_e)L2) 

#define TASK_PERIOD (1 * 60 * 1000) // In ms.
#define TASK_ALGO_PERIOD_MINUTES (5) // 5 min between distribution algo runs

/* If the phase is measured as `PHASE_DARK' for this many minutes
after a sun phase, consider sun phase to be over. */
#define SUN_PHASE_MAX_TRAILING_MINUTES (3) 

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

typedef enum {
    WAIT,
    RUN
} State;

typedef struct {
    float five_min_consumption[CONSBUF_NUM_ELTS]; // Each element is system power consumption averaged over 5min. In units of W.
    uint8_t index; // Points to next element to be populated. Increments on insert (buffer is left-justified).
    uint8_t len; // Number of valid data points currently stored in buffer
} ConsumptionBuffer;

typedef struct {
    uint8_t duration; // Measured duration of sun phase, in minutes
    float totalGen; // Running integral of measured solar generation, in watt-minutes (W*min)
} SolarData_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL CONSTANTS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Expected power level additional power consumption in W, index using `Sys_PowerLevel_e'.
// `PWRLVL_EXP_CONS[Lx]' is the expected *additional* consumption increase or decrease
// if we switch up to or down from level Lx, respectively.
// Consumption is 0 for L0 since it's the minimum possible consumpiton anyways and it doesn't matter what its actual consumption is
static const float PWRLVL_EXP_CONS[NumPwrLvls] = {
    // TODO update with refined power budget after solar panel sim has been done
    [L0] = 0,
    [L1] = 0,
    [L2] = 0,
    [L3] = 0,
    [L4] = 0,
};

// Constant linear interpolation data 
// used for mapping battery voltage onto battery SoC
static const float SOC_VOLTAGES[] = { 3.0, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.2 };
static const float SOC_VALS[] = { 0.0, 2.16, 3.57, 8.98, 23.15, 58.87, 77.91, 90.91, 100.0 };

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL MEASUREMENTS CACHE
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Deliberately not initialized at link-time - must be initialized as part of task init

static ConsumptionBuffer consumptionBuffer; 
static float battChg_cache;
static ORBIT_PHASE currPhase;
static SolarData_t lastFullSolar, currSolar;
static uint8_t trailingDarkMinutes; // Counts number of times dark phase was measured after sun phase

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

//////////////////////////////////////////////////////////
/// Buffer Functions /////////////////////////////////////
//////////////////////////////////////////////////////////

static void insertConsumption(float batteryChargeBefore, float batteryChargeAfter) {
    // 5 min consumption
    float difference = (batteryChargeAfter - batteryChargeBefore) * EPS_CAPACITY / 5.f;
    consumptionBuffer.five_min_consumption[consumptionBuffer.index] = difference;
    if (consumptionBuffer.len < CONSBUF_NUM_ELTS) consumptionBuffer.len++;
    if (consumptionBuffer.index < CONSBUF_NUM_ELTS - 1) consumptionBuffer.index++;
    else consumptionBuffer.index = 0;
}

static void resetConsBuff()
{
    consumptionBuffer.index = 0;
    consumptionBuffer.len = 0;
    // Only initialize the buffer itself in debug to save processing time for flight
#ifdef DEBUG_ENABLED
    memset(&(consumptionBuffer.five_min_consumption), 0, sizeof(consumptionBuffer.five_min_consumption));
#endif
}

//////////////////////////////////////////////////////////
/// Reset Functions //////////////////////////////////////
//////////////////////////////////////////////////////////

static void discardMeasurements() {
    resetConsBuff();
    battChg_cache = FLT_MAX;
    currPhase = PHASE_UNKNOWN;
    memset(&currSolar, 0, sizeof(currSolar));
    trailingDarkMinutes = 0;
    // Keep last full solar
}

static void initStaticData() {
    discardMeasurements();
    memset(&lastFullSolar, 0, sizeof(lastFullSolar));
}

//////////////////////////////////////////////////////////
/// External Measurement Functions ///////////////////////
//////////////////////////////////////////////////////////

// needs to be implemented with output from solar panels
float getSolarPower() {
    return 0;
}

// needs to be implemented with output from EPS
float getBatteryCharge() {
    return 0;
}

// needs to be implemented with output from sun sensor
float getState() {
    return 0;
}

//////////////////////////////////////////////////////////
/// Power Functions //////////////////////////////////////
//////////////////////////////////////////////////////////

// Calculate the mean of all data in the consumption buffer
static float calculateBufAvg()
{
    float sum = 0; uint8_t num_read = 0;
    for (uint8_t ptr = consumptionBuffer.index;
        num_read < consumptionBuffer.len;
        num_read++, ptr > 0 ? ptr-- : ptr = CONSBUF_NUM_ELTS)
    {
        sum += consumptionBuffer.five_min_consumption[ptr];
    }
    return sum / num_read;
}

// returns total power consumed over 90 min
static float calculateTotalConsumption90() {
    float averageConsumptionSum = 0;
    for (int i = 0; i < CONSBUF_NUM_ELTS; i++) {
        averageConsumptionSum += consumptionBuffer.five_min_consumption[i];
    }

    return averageConsumptionSum;
}

//////////////////////////////////////////////////////////
/// Main Login ///////////////////////////////////////////
//////////////////////////////////////////////////////////

// Calculate base power level, i.e. the maximum power level where
// consumption is below generation. 
// Uses `PWRLVL_EXP_CONS'.
static Sys_PowerLevel_e calculateBasePwrLvl(Sys_PowerLevel_e currPwrLvl, float avgCons, float avgGen) {

    float power_diff = avgGen - avgCons;
    Sys_PowerLevel_e targetLvl = currPwrLvl;

    if (power_diff < 0) { // Will need to reduce power level
        float returned_power = 0;

        // Decrease power level until power difference is positive
        while (targetLvl > GLOBAL_PWRLVL_FLOOR) { 
            // Get back the expected consumption of the target level,
            // then decrease the target level.
            returned_power += PWRLVL_EXP_CONS[targetLvl--]; 
            if (returned_power > power_diff) break;
        }
    }
    else { // Maybe can increase power level
        float spent_power = 0;

        // Increase power level until power difference is negative
        while (targetLvl < NumPwrLvls) {
            // Increase target level,
            // then add the expected consumption of the new target level.
            spent_power += PWRLVL_EXP_CONS[++targetLvl];
            if (spent_power > power_diff) {
                targetLvl--; // Backtrack to stay strictly under generation
                break;
            }
        }
    }
    return targetLvl;
}


//TODO: measure how long it takes to run and take it out of the sleep time

void run_1min() {
    State currentState = WAIT;
    float solarPowerIntegral = 0;
    int count_90min = 0;
    int minute_count = 0;
    int current_power_level = 4;
    float chargeBefore = max_charge;
    float chargeBefore_percent = chargeBefore / max_charge;

    while (1) {
        // State Machine 
        switch (currentState) {
        case WAIT: {
            // Not Measuring
            ORBIT_PHASE orbit_phase = getPhase_Sample();

            if (orbit_phase == PHASE_SUN) {
                currentState = RUN;
                break;
            }
        }
        case RUN: {
            int dark_count = 0;

            ORBIT_PHASE orbit_phase = getPhase_Sample();

            if (dark_count == 3) {
                // Check for 3 DARK phases
                currentState = WAIT;

                break;
            }

            solarPowerIntegral += getSolarPower_Sample(orbit_phase);

            if (orbit_phase == PHASE_SUN) {
                dark_count = 0;
            }
            else if (orbit_phase == PHASE_DARK) {
                dark_count++;
            }
            else {
                dark_count = 0;
            }
        }
        }

        if (count_90min == 90) {
            solarPowerTotal = solarPowerIntegral;
            count_90min = 0;
            solarPowerIntegral = 0;
        }

        count_90min++;
        usleep(100000);
        printf("1min\n");
        minute_count++;
        if (minute_count == 5) {

            // Measures the EPS battery SoC
            float chargeAfter = getBatteryCharge_Sample(current_power_level, chargeBefore);
            float chargeAfter_percent = chargeAfter / max_charge;

            // 5-min-average power consumption in the buffer
            insertConsumption(&consumptionBuffer, chargeBefore_percent, chargeAfter_percent);

            // Update State given new Avarage
            if (consumptionBuffer.five_min_consumption[CONSBUF_NUM_ELTS - 1] != FLT_MAX) {

                ORBIT_PHASE orbit_phase = getPhase_Sample();

                // Runs the Algorithm
                current_power_level = changeState(current_power_level, &consumptionBuffer, orbit_phase);
            }
            printf("Power Level: %d\n", current_power_level);

            // Save for a power diffence
            chargeBefore = chargeAfter;
            chargeAfter = 0;
            minute_count = 0;
        }
    }
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void PowerDist_X_Task(void* argument)
{
    initStaticData();

    HAL_StatusTypeDef EPS_stat; EPS_VoltageType V; float battChg;

    // Task wakes and measures SoC every minute, checking for critical.
    // Additionally, every 5th minute the measurement is cached and the distribution algorithm runs.
    uint8_t minuteCt = 0; // Number of minutes the task has waited since the last algo run

    while (1)
    {
        if (battChg_cache > 100) // If there is no cached measurement yet, take it now
        {
            Sys_TryDev(EPS_stat, EPS_R_Voltages(&V), HAL_OK, DEV_EPS);
            if (EPS_stat != HAL_OK) break; // Cannot continue on EPS fault - exit task
            battChg_cache = PowerDist_GetSoC(V.V_Batt);
            minuteCt = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD)); minuteCt++;

        // Critical check happens every minute, i.e. every time task wakes
        do // Guard in a do-while to be able to repeat critical check after waking from L0
        {
            Sys_TryDev(EPS_stat, EPS_R_Voltages(&V), HAL_OK, DEV_EPS);
            if (EPS_stat != HAL_OK) goto end; // Cannot continue on EPS fault - exit task
            battChg = PowerDist_GetSoC(V.V_Batt);
            if (battChg < EPS_CRIT_SOC)
            {
                // Because we are about to sleep for > 5min,
                // throw away measurements which need to be continuous
                discardMeasurements();
                Sys_X_SetPowerLevel(L0);
                continue; // Repeat critical check after waking from L0
            }
        } while (0);

        // Sun check happens every minute, i.e. every time task wakes
        ORBIT_PHASE newPhase = RunSunDetect();
        if (newPhase == PHASE_SUN)
        {
            trailingDarkMinutes = 0;
            // If last phase was dark and this phase is sun, start new sun phase
            if (currPhase != PHASE_SUN)
            {
                currSolar.duration = 0; currSolar.totalGen = 0;
            }
            // In any case, make new sun generation measurement and update duration
            // TODO update `currSolar' duration and generation integral
            // TODO check for case duration >= 90 min
            currPhase = PHASE_SUN;
        }
        else if (newPhase == PHASE_DARK)
        {
            if (currPhase == PHASE_SUN)
            {
                if (++trailingDarkMinutes >= SUN_PHASE_MAX_TRAILING_MINUTES) // sun phase over
                {
                    currPhase = PHASE_DARK;
                    currSolar.duration -= trailingDarkMinutes; // Deduct miscounted trailing dark minutes
                    trailingDarkMinutes = 0;
                    memcpy(&lastFullSolar, &currSolar, sizeof(SolarData_t));
                }
                else // Sun phase maybe not yet over
                {
                    // TODO update `currSolar' duration and generation integral
                }
            }
        }
        else panic("unexpected SunDeterm result %d", newPhase); // TODO real implementation is TBR based on testing
        
        // Every 5th minute, also run distribution algo
        if (minuteCt >= TASK_ALGO_PERIOD_MINUTES) 
        {
            minuteCt = 0;

            insertConsumption(battChg_cache, battChg); // Insert new measurement into consumption buffer

            if (consumptionBuffer.len >= CONSBUF_MIN_ELTS)
            {
                float consAvg = calculateBufAvg(); // Average consumption data over the past 90 min.

                float solarAvg; // TODO

                Sys_PowerLevel_e currPwrLvl = Sys_GetPowerLevel();
                Sys_PowerLevel_e baseLvl = calculateBasePwrLvl(currPwrLvl, consAvg, solarAvg);

                if (baseLvl != currPwrLvl)
                {
                    resetConsBuff();
                    Sys_X_SetPowerLevel(baseLvl);
                }
            }

            battChg_cache = battChg;
        }
    }
end:
    // Should never get here!
    vTaskDelete(NULL);
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
float PowerDist_GetSoC(float voltage)
{
    if (voltage <= SOC_VOLTAGES[0])
        return SOC_VALS[0];
    if (voltage >= SOC_VOLTAGES[8])
        return SOC_VALS[8];

    int i = 0;
    while (voltage >= SOC_VOLTAGES[i])
    {
        i++;
    }
    float charge = SOC_VALS[i - 1] + (voltage - SOC_VOLTAGES[i - 1]) * (SOC_VALS[i] - SOC_VALS[i - 1]) / (SOC_VOLTAGES[i] - SOC_VOLTAGES[i - 1]);
    return charge;
}

HAL_StatusTypeDef PowerDist_IsBattCritical(uint8_t* isCrit)
{
    HAL_StatusTypeDef EPS_stat; EPS_VoltageType V; 
    Sys_TryDev(EPS_stat, EPS_R_Voltages(&V), HAL_OK, DEV_EPS);
    if (EPS_stat == HAL_OK)
    {
        float SoC = PowerDist_GetSoC(V.V_Batt);
        *isCrit = SoC < EPS_CRIT_SOC;
    }
    return EPS_stat;
}

#ifdef DEBUG_ENABLED
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TEST SET DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

ORBIT_PHASE phases[] = { PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_SUN, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK, PHASE_DARK };
float charge_powers[] = { 0.2, 0, 0, 0, 0.2, 0.5, 0.8, 0.1 };
int testStateCount = 8;
int testState = 0;

float getSolarPower_Sample(ORBIT_PHASE phase) {
    float solarChargeSun = rand() % (5000 - 3000) + 3000;
    float solarChargeDark = 0.;
    if (phase == PHASE_SUN) {
        return solarChargeSun;
    }
    else {
        return solarChargeDark;
    }
}

float getBatteryCharge_Sample(int current_power_level, float previous_charge) {

    int is_detumbling = 0;
    float BatteryCharge = previous_charge;
    switch (current_power_level) {
    case(0): {
        //not from spreadsheet
        // "L0 is the lowest level so consumption doesn't matter"
        BatteryCharge -= 30;
    }
    case(1): {
        BatteryCharge -= 53;
    }
    case(2): {
        BatteryCharge -= 74;
    }
    case(3): {
        if (is_detumbling == 0) {
            BatteryCharge -= 132;
        }
        else {
            BatteryCharge -= 74;
        }
    }
    case(4): {
        if (is_detumbling == 0) {
            BatteryCharge -= 226;
        }
        else {
            BatteryCharge -= 168;
        }
    }
    }
    return BatteryCharge;
}


ORBIT_PHASE getPhase_Sample(void);

ORBIT_PHASE getPhase_Sample() {
    testState++;
    testState = testState % 180;
    if (testState < 90) {
        printf("SUN_Phase");
    }
    else {
        printf("DARK_Phase");
    }

    return phases[testState];
}
#endif
