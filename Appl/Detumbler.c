/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file Detumbler.c
* @brief Defines magnetorquer Bdot implementation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Jack M. and Alex J.
* @version           1.0.0
* @date              2022.09.25
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "Detumbler.h"
#include "ADCS_Meas.h"
#include "EEPROM_emul.h"
#include "panels.h"
#include "arm_math.h"
#include "User_types.h"
#include <string.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Maximum number of ms to wait for new measurements from ADCS measurement task
#define MEAS_WAIT_TO (10*1000)

// Define this symbol to add lowpass Bdot filtering
//#define DETUMB_FILTER_BDOT
#ifdef DETUMB_FILTER_BDOT
/* Dimensionless constant expressing weight of previous measurement when filtering */
#define ALPHA (float)(0.5)             
#endif

#define SCALE (100)	/* Scaling factor between normalized Bdot and MTQ output vector */

#define errorcheck(retval, good_val, fail_lbl) if (retval != good_val) goto fail_lbl

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DETUMB_FILTER_BDOT
static Vec3D_t oldFilteredBdot = { {0,0,0} };
#endif

// Saves the MTQ output, in case MTQ operation needs to be interrupted to measure MTM
static MTQ_out_comp_t S_currMTQoutput[3];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

void processBdot(MTQ_out_comp_t MTQ_output[3], Vec3D_t* Bdot)
{
    Vec3D_t processedBdot;
#ifdef DETUMB_FILTER_BDOT
    arm_scale_f32(&oldFilteredBdot, ALPHA, &oldFilteredBdot, 3);
    arm_scale_f32(Bdot, ((float)1 - ALPHA), Bdot, 3);
    arm_add_f32(&oldFilteredBdot, Bdot, &processedBdot, 3);
    //replace previous value
    memcpy(&oldFilteredBdot, &processedBdot, sizeof(Vec3D_t));
#else
    memcpy(&processedBdot, Bdot, sizeof(Vec3D_t));
#endif // DETUMB_FILTER_BDOT

    //check which vector component has greatest magnitude
    float max_comp;
    max_comp = fabs(processedBdot.X) > fabs(processedBdot.Y) ? processedBdot.X : processedBdot.Y;
    max_comp = fabs(max_comp) > fabs(processedBdot.Z) ? max_comp : processedBdot.Z;

    Vec3D_t outputVec;

    //normalize Bdot such that greatest magnitude component has magnitude 1
    arm_scale_f32(&processedBdot, max_comp, &outputVec, 3);

    //invert and scale bdot
    float32_t scalarFactor = -1 * SCALE;
    if (EEPROM_emul_pDataInfo->MTQ_polarity == MTQ_POLARITY_INVERT) scalarFactor *= -1;
    arm_scale_f32(&outputVec, scalarFactor, &outputVec, 3);

    // Save output vector as uint8_t percentages
    // TODO: EnduroSat method parameters and behavior are ambiguous
    // and must be tested in the lab!!!
    for (uint8_t i = 0; i < 3; i++)
    {
        MTQ_output[i].perc = (uint8_t)fabs(outputVec.Vec[i]);
        if (outputVec.Vec[i] < 0) MTQ_output[i].dir = 0;
        else MTQ_output[i].dir = 1;
    }
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

void Detumbling_X_Task(void* arg)
{
    memset(&oldFilteredBdot, 0, sizeof(Vec3D_t));

    while (1)
    {
        if (ADCS_Meas_WaitForUpdate(ADCS_MEAS_LISTENER_DETUMB, MEAS_WAIT_TO) \
            == E_OK)
        {
            Vec3D_t Bdot;
            ES_ReturnType retstat = ADCS_Meas_GetBdot(&Bdot); errorcheck(retstat, E_OK, Meas_fail);
            MTQ_out_comp_t MTQ_out[3];
            processBdot(MTQ_out, &Bdot);
            memcpy(S_currMTQoutput, MTQ_out, 3 * sizeof(MTQ_out_comp_t));
            status_t MTQ_retstat = Detumbling_X_SetOutput(MTQ_out); errorcheck(MTQ_retstat, SEN_SUCCESS, MTQ_fail);
            continue;
        }
        }
        else // measurement timeout
        {
            // TODO
        }
    Meas_fail:
    // TODO: error reporting
    continue;
MTQ_fail:
    // TODO: error reporting
    continue;

}

void Detumbling_X_RestoreOutput()
{
    for (uint8_t i = 0; i < 3; i++)
    {
        status_t MTQretstat = SetMagnetorque(PAN_X_M + i, S_currMTQoutput[i].perc, S_currMTQoutput[i].dir);
        if (MTQretstat != SEN_SUCCESS)
        {
            //TODO
        }
    }
}

status_t Detumbling_X_SetOutput(MTQ_out_comp_t output[3])
{
    Stop_Magnetorquers();
    for (uint8_t i = 0; i < 3; i++)
    {
        status_t MTQretstat = SetMagnetorque(PAN_X_M + i, output[i].perc, output[i].dir);
        if (MTQretstat != SEN_SUCCESS) return MTQretstat;
    }
}
