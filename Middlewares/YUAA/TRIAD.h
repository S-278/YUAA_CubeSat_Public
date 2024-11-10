/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TRIAD.h
* @brief TRIAD Algorithm Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Wesley A., Lijie Y.
* @version           1.0.0
* @date              2022.09.25
*
* @details           Estimate the attitude from two orthogonal vector observations
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "ADCS_Meas.h"
#include "User_types.h"
#include "arm_math.h"
#include "ff.h"
#include <time.h>
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#define EQUINOX_TIME ((time_t) 1695451800)
/* Radius of the spherical Earth in meters */
#define EARTH_R ((float)6378100)

/* UTC time at which 0deg N and 0deg E was along the X axis in equatorial coordinates.
   This happens at the time of a vernal (spring) equinox.
   (Actually this also happens every day whenever RA 0 crosses the meridian at 0deg N 0deg E,
   but an equinox is a more convenient reference point.) /
#define EQUINOX_TIME ((time_t) 1695451800)
/ Radius of the spherical Earth in meters /
#define EARTH_R ((float)6378100)
/ Angular speed of Earth's rotation about its axis, in rad/s */
#define EARTH_ROT_SPEED ((float)7.2921159E-5)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Run the TRIAD attitude determination algorithm
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 body_B - pointer to B measurement in body frame
* @param[input]		 body_S - pointer to S measurement in body frame
* @param[input]		 t - current UTC time
* @param[output]      attitudeMatrix - pointer to initialized arm math matrix instance type 
*					       to be populated by 3x3 attitude matrix
* @return             FRESULT containing result of SD interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TRIAD_GetAttMatrix(const Vec3D_t* body_B, const Vec3D_t* body_S, time_t* t,
	                        arm_matrix_instance_f32* attitudeMatrix);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute angular velocity vector
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		 pairB - pointer to MeasPair_t containing pair of B measurements 
*                     in body frame
* @param[input]		 pairS - pointer to MeasPair_t containing pair of S measurements 
*                     in body frame
* @param[input]		 t - UTC time of first measurement
* @param[output]      omega - pointer to Vec3D_t to populate with omega vector
* @return             FRESULT containing result of SD interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT TRIAD_GetOmegaVec(const MeasPair_t* pairB, const MeasPair_t* pairS,
                          time_t* t, Vec3D_t* omega);
