/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file Geomag.h
* @brief YUAA Geomag Library Header
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Wesley A.
* @details           Declares the routines for Geomag Model without using malloc
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include "MCU_Init.h"
#include "ff.h"
#include "User_types.h"
#include "arm_math.h"
#include <time.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define MAX_N_MODE 12
#define CALCULATE_NUMTERMS(N)    (N * ( N + 1 ) / 2 + N)

#define MAXLINELENGTH (1024)

#define RAD2DEG(rad)    ((rad)*(180.0L/PI))
#define DEG2RAD(deg)    ((deg)*(PI/180.0L))

#define MAG_GEO_POLE_TOLERANCE  1e-5
#define MAG_USE_GEOID	1    /* 1 Geoid - Ellipsoid difference should be corrected, 0 otherwise */

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

typedef struct
{
    float32_t EditionDate;
    float32_t epoch; /*Base time of Geomagnetic model epoch (yrs)*/
    char ModelName[32];
    float32_t Main_Field_Coeff_G[CALCULATE_NUMTERMS(MAX_N_MODE)+1]; /* C - Gauss coefficients of main geomagnetic model (nT) Index is (n * (n + 1) / 2 + m) */
    float32_t Main_Field_Coeff_H[CALCULATE_NUMTERMS(MAX_N_MODE)+1]; /* C - Gauss coefficients of main geomagnetic model (nT) */
    float32_t Secular_Var_Coeff_G[CALCULATE_NUMTERMS(MAX_N_MODE)+1]; /* CD - Gauss coefficients of secular geomagnetic model (nT/yr) */
    float32_t Secular_Var_Coeff_H[CALCULATE_NUMTERMS(MAX_N_MODE)+1]; /* CD - Gauss coefficients of secular geomagnetic model (nT/yr) */
    int nMax; /* Maximum degree of spherical harmonic model */
    int nMaxSecVar; /* Maximum degree of spherical harmonic secular model */
    int SecularVariationUsed; /* Whether or not the magnetic secular variation vector will be needed by program*/
    float32_t CoefficientFileEndDate; 
} Geomag_MagneticModel_t;

typedef struct {
    float32_t a; /*semi-major axis of the ellipsoid*/
    float32_t b; /*semi-minor axis of the ellipsoid*/
    float32_t fla; /* flattening */
    float32_t epssq; /*first eccentricity squared */
    float32_t eps; /* first eccentricity */
    float32_t re; /* mean radius of  ellipsoid*/
} Geomag_Ellipsoid_t;

typedef struct {
    int NumbGeoidCols; /* 360 degrees of longitude at 15 minute spacing */
    int NumbGeoidRows; /* 180 degrees of latitude  at 15 minute spacing */
    int NumbHeaderItems; /* min, max lat, min, max long, lat, long spacing*/
    int ScaleFactor; /* 4 grid cells per degree at 15 minute spacing  */
    float *GeoidHeightBuffer;
    int NumbGeoidElevs;
    int Geoid_Initialized; /* indicates successful initialization */
    int UseGeoid; /*Is the Geoid being used?*/
} Geomag_Geoid_t;

typedef struct {
    float32_t lambda; /* longitude */
    float32_t phi; /* geodetic latitude */
    float32_t HeightAboveEllipsoid; /* height above the ellipsoid (HaE) */
    float32_t HeightAboveGeoid; /* (height above the EGM96 geoid model ) */
    int UseGeoid;
} Geomag_CoordGeodetic_t;

typedef struct {
    float32_t lambda; /* longitude*/
    float32_t phig; /* geocentric latitude*/
    float32_t r; /* distance from the center of the ellipsoid*/
} Geomag_CoordSpherical_t;

typedef struct {
    float32_t DecimalDate;
} Geomag_Date_t;

typedef struct {
    float32_t Decl; /* 1. Angle between the magnetic field vector and true north, positive east*/
    float32_t Incl; /*2. Angle between the magnetic field vector and the horizontal plane, positive down*/
    float32_t F; /*3. Magnetic Field Strength*/
    float32_t H; /*4. Horizontal Magnetic Field Strength*/
    float32_t Bx; /*5. Northern component of the magnetic field vector*/
    float32_t By; /*6. Eastern component of the magnetic field vector*/
    float32_t Bz; /*7. Downward component of the magnetic field vector*/
} Geomag_GeoMagneticElements_t;

typedef struct {
    float32_t Pcup[CALCULATE_NUMTERMS(MAX_N_MODE)+1]; /* Legendre Function */
    float32_t dPcup[CALCULATE_NUMTERMS(MAX_N_MODE)+1]; /* Derivative of Legendre fcn */
} Geomag_LegendreFunction_t;

typedef struct {
    float32_t RelativeRadiusPower[MAX_N_MODE+1]; /* [earth_reference_radius_km / sph. radius ]^n  */
    float32_t cos_mlambda[MAX_N_MODE+1]; /*cp(m)  - cosine of (m*spherical coord. longitude)*/
    float32_t sin_mlambda[MAX_N_MODE+1]; /* sp(m)  - sine of (m*spherical coord. longitude) */
} Geomag_SphericalHarmonicVariables_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Get magnetic vector in equatorial coordinates for a given sattelite position
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]         t - pointer to time_t containing UTC time at which to compute vector
* @param[input]         SatEquatorial - sattelite position in equatorial coordinates
* @param[output]        MagEquatorial - magnetic vector in equatorial coordinates
* @return               FRESULT containing result of SD interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Geomag_GetMagEquatorial(time_t* t, const Vec3D_t* SatEquatorial, Vec3D_t* MagEquatorial);

#ifdef DEBUG_ENABLED
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* TEST SET DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void testGeomag_HorizontalToEquatorial();
void testGeomag_X_HorizontalToEquatorial();
void testGeomag_GetMagEquatorial();
void testGeomag_LoadWMM();

#endif