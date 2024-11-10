/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file OrbitProp.h
* @brief Orbit Propagation Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Aviv M.
* @version           0.1.0
* @date              2022.10.02
*
* @details           Declares orbit propagation utility
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#pragma once
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "SD_Routines.h"
#include "ff.h"
#include "User_types.h"
#include <time.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
typedef struct {
    double latitude;
    double longitude;
} LatLong_Coord_t;

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define SD_TLE_PATH		SD_ROOT
#define SD_TLE_FILENAME ("tle")
#define TLE_LENGTH		(2*70)

#define SUN_EQ_START_TIME	((time_t)0 /*PLACEHOLDER*/)		// UTC time for which the first entry in the file is valid TODO: SET VALUE
#define SUN_EQ_PATH			SD_ROOT
#define SUN_EQ_FILENAME		("sun_eq")
#define SUN_EQ_INTERVAL     60 /*PLACEHOLDER*/       // s between each entry TODO: set this

#define GND_PATH		SD_ROOT
#define GND_FILENAME	("gnd")

/* UTC time at which 0deg N and 0deg E was along the X axis in equatorial coordinates.
   This happens at the time of a vernal (spring) equinox.
   (Actually this also happens every day whenever RA 0 crosses the meridian at 0deg N 0deg E,
   but an equinox is a more convenient reference point.) */
#define EQUINOX_TIME ((time_t) 1695451800)
/* Radius of the spherical Earth in meters */
#define EARTH_R ((float)6378100)
/* Angular speed of Earth's rotation about its axis, in rad/s */
#define EARTH_ROT_SPEED ((float)7.2921159E-5)

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Check that a TLE string is properly formatted
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		tle_str - pointer to TLE string
* @return           1 if the string pointed to by `tle_str' is a valid TLE, 0 otherwise
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
uint8_t Orbit_IsValidTLE(const char* tle_str);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute vector from Earth to sat in equatorial coordinates
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		t - pointer to time_t containing UTC time at which to compute vector
* @param[output]    result - pointer to Vec3D_t to populate with result
* @return           `E_NOT_INIT' if SD file does not contain valid TLEs, `E_OK' otherwise
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType Orbit_GetSatEquatorial(const time_t* t, Vec3D_t* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Convert Earth surface latitude-longitude coordinates to equatorial coordinates
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]     latlong - pointer to latitude-longitude coordinates
* param[output]     eq - pointer to Vec3D_t to populate with equatorial coordinates
* @param[input]		t - pointer to time_t containing UTC time at which to compute conversion
* @return           none
* @detail           This conversion assumes a perfectly spherical Earth 
*                   and uses the constants `EQUINOX_TIME', `EARTH_R', and `EARTH_ROT_SPEED'
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void Orbit_ConvertLatLongToEq(const LatLong_Coord_t* latlong, Vec3D_t* eq, const time_t* t);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute vector from Earth to the Sun in equatorial coordinates
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		t - pointer to time_t containing UTC time at which to compute vector
* @param[output]    result - pointer to Vec3D_t to populate with result
* @return           FRESULT containing result of SD interface
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Orbit_GetSunEquatorial(const time_t* t, Vec3D_t* result);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute time of next expected pass over gnd
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]		currT - pointer to time_t containing current UTC time
* @param[output]    result - pointer to time_t to populate with UTC time of next expected 
*					pass over gnd. If no pass is found, this is set to 0.
* @param[output]	SD_stat - pointer to FRESULT to populate with SD interface result
* @return           `E_NOT_INIT' if SD file does not contain valid TLEs, `E_OK' otherwise
* @detail			User is responsible for checking both the return value and `SD_stat'
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType Orbit_GetNextPassPred(const time_t* currT, time_t* result, FRESULT* SD_stat);

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Update sat TLEs
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]     tle_str - pointer to string formatted as a TLE
* @return           FRESULT containing result of SD interface
* @detail			This updates both the runtime variable stored in this C file, and 
*					the file stored on the SD card
* @note				Exclusively for TelComm Parser
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Orbit_X_SetTLEs(const char* tle_str);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Initialize orbit propagator library
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return           FRESULT containing result of reading TLEs from SD 
* @detail			This reads the SD TLE file and parses the string into a static runtime
*					variable.
* @note				Exclusively for init routine
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Orbit_X_Init();
