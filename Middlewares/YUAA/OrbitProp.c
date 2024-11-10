/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file OrbitProp.h
* @brief Orbit Propagation C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Aviv M.
* @version           0.1.0
* @date              2022.10.02
*
* @details           Declares orbit propagation utility
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "OrbitProp.h"
#include "system_manager.h"
#include "SD_Routines.h"
#include "VecUtils.h"
#include "SGP4.h"
#include "TLE.h"
#include "User_types.h"
#include "arm_math.h"
#include <math.h>
#include <stdint.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Constants for SD reading
#define SUN_EQ_LINE_SZ (sizeof(Vec3D_t))
#define GND_LINE_SZ (sizeof(LatLong_Coord_t))
// Number of gnd coordinate lines to hold in memory at a time. TODO optimize
#define GND_NUM_LINES_IN_BUF (8)

// Constants for pass prediction
/* Number of seconds to step on each iteration when searching for a pass */
#define PASS_PRED_STEP ((time_t) 60) // TODO: DECIDE THIS
/* Maximum number of steps to take on propagation before giving up */
#define PASS_PRED_MAX_STEPS (3*24*60) // TODO: DECIDE THIS
/* Minimum elevation of sat above the local horizon that is considered a gnd pass, in rad */
#define PASS_PRED_MIN_ELEVATION ((float)(10 * PI / 180))

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// CAUTION: the TLE type is almost 2kB! It is preferred to have only one TLE object, and
// better for it to be in the data segment (as a static global) than on the stack (as a local)
static TLE S_tle; 
static uint8_t S_tleValid = 0;

/* Example TLE for testing:
1 39444U 13066AE  22153.47668570  .00001597  00000+0  19532-3 0  9990
2 39444  97.6236 128.0555 0058478  82.3390 278.4460 14.83359372459309
*/


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Given a sat vector `v' in equatorial and a gnd vector `gnd' also in equatorial,
   returns whether v is above the horizon from the perspective of gnd */ 
static int isSeenFromGnd(const Vec3D_t* v, const Vec3D_t* gnd)
{
    // TODO
    // 1. Find the vector from `gnd' to `v'
    Vec3D_t gndToSat;
    arm_sub_f32((float32_t*)v, (float32_t*)gnd, (float32_t*)&gndToSat, 3);
    // 2. Find the angle between the vector from gnd to v, and gnd. 
    // This angle is 90deg - elevation above local horizon.
    float32_t dotProd;
    arm_dot_prod_f32((float32_t*)&gndToSat, (float32_t*)gnd, 3, &dotProd);
    float32_t mag_gnd, mag_gndToSat;
    arm_dot_prod_f32((float32_t*)gnd, (float32_t*)gnd, 3, &mag_gnd); arm_sqrt_f32(mag_gnd, &mag_gnd);
    arm_dot_prod_f32((float32_t*)(&gndToSat), (float32_t*)(&gndToSat), 3, &mag_gndToSat); arm_sqrt_f32(mag_gndToSat, &mag_gndToSat);
    float32_t cosAngle = dotProd / (mag_gnd * mag_gndToSat);
    float angle = acosf(cosAngle);
    // 3. Find the elevation and compare against threshold
    float elevation = PI / 2 - angle;
    return elevation > PASS_PRED_MIN_ELEVATION;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/


uint8_t Orbit_IsValidTLE(const char* tle_str)
{
    // Specifically, checks: total length, length of each line, no illegal symbols (e.g. no 
    // letters where there are only supposed to be digits), spaces in the correct places. 

    // TODO: is there a function in SGP4 or TLE for this?

}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute vector from Earth to sat in equatorial coordinates
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     result - pointer to Vec3D_t to populate with result
* @return            ES_ReturnType containing status of calculation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
ES_ReturnType Orbit_GetSatEquatorial(const time_t* t, Vec3D_t* result)
{
    if (result == NULL) Sys_RaiseLogicError(__FILE__, __LINE__, "null ptr passed");
    if (!S_tleValid) return E_NOT_INIT;

    double r[3];
    double v[3];
    getRVForDate(&S_tle,(*t)*1000,r,v);

    for (int i = 0; i < 3; i++) result->Vec[i] = r[i];
    return E_OK;
}

/* First converts `latlong' to equatorial at `EQUINOX_TIME',
   then rotates the equatorial vector accordingly
   based on the time difference between `EQUINOX_TIME' and `t'. */
void Orbit_ConvertLatLongToEq(const LatLong_Coord_t* latlong, Vec3D_t* eq, const time_t* t)
{
    // 1. Use standard spherical coordinate conversions to convert latlong to Cartesian coordinates.
    // This gives the equatorial coordinates of latlong at the time of a vernal equinox 
    // (TODO exactly which vernal equinox should be refined right before launch).
    float32_t theta = PI / 2 - (float)(latlong->latitude);
    float32_t phi = (float)(latlong->longitude);
    // x = r * sin(theta) * cos(phi)
    eq->X = EARTH_R * arm_sin_f32(theta) * arm_cos_f32(phi);
    // y = r * sin(theta) * sin(phi)
    eq->Y = EARTH_R * arm_sin_f32(theta) * arm_sin_f32(phi);
    // z = r * cos(theta)
    eq->Z = EARTH_R * arm_cos_f32(theta);

    // 2. Rotate the equatorial vector about the Z axis at the speed of Earth's rotation about its axis
    // for a time equal to the difference between `EQUINOX_TIME' and `t'
    float32_t rotAngle = EARTH_ROT_SPEED * (*t - EQUINOX_TIME);
    Vec_RotateSpher(eq, 0, rotAngle, NULL);
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute vector from Earth to the Sun in equatorial coordinates
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     result - pointer to Vec3D_t to populate with result
* @return            ES_ReturnType containing status of calculation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Orbit_GetSunEquatorial(const time_t* t, Vec3D_t* result)
{
    if (result == NULL) Sys_RaiseLogicError(__FILE__, __LINE__, "null ptr passed");
    if (*t < SUN_EQ_START_TIME) Sys_RaiseLogicError(__FILE__, __LINE__, "time passed is too early");

    // Find unixtime file offset
    time_t time_offset = *t - SUN_EQ_START_TIME;
    uint32_t start_line = time_offset / SUN_EQ_INTERVAL;

    // Read two lines from file
    char file_contents[SUN_EQ_LINE_SZ * 2];
    FRESULT f_result = SD_getFileContentsOffset(SUN_EQ_PATH, SUN_EQ_FILENAME, file_contents, start_line * SUN_EQ_LINE_SZ, SUN_EQ_LINE_SZ * 2);
    if (f_result != FR_OK) return f_result;

    // Set up pointers to treat buffer as two vectors
    Vec3D_t* v0 = (void*)file_contents, *v1 = (void*)(file_contents + SUN_EQ_LINE_SZ);

    // Check if exact time
    time_t remainder = time_offset % SUN_EQ_INTERVAL;
    if (remainder == 0) {

        // Write first line data into result
        memcpy(result, v0, sizeof(Vec3D_t));
        return FR_OK;

    } else { // interpolate

        float scalar = remainder / (float) SUN_EQ_INTERVAL;
        Vec3D_t v1_minus_v0;
        arm_sub_f32((float32_t*)v1, (float32_t*)v0, (float32_t*)&v1_minus_v0, 3);
        arm_scale_f32((float32_t*)&v1_minus_v0, scalar, (float32_t*)&v1_minus_v0, 3);
        arm_add_f32((float32_t*)v0, (float32_t*)&v1_minus_v0, (float32_t*)result, 3);

        return E_OK;
    }
    
}

ES_ReturnType Orbit_GetNextPassPred(const time_t* currT, time_t* result, FRESULT* SD_stat)
{
    if (!S_tleValid) return E_NOT_INIT;

#define errorcheck(fres) if ((FRESULT)fres != FR_OK) goto end

    FIL fd; FRESULT fres = f_open(&fd, GND_FILENAME, FA_READ | FA_OPEN_EXISTING);
    if (fres != FR_OK) { *SD_stat = fres; return E_OK; }
    uint16_t numGndLines = f_size(&fd) / GND_LINE_SZ;
    
    // Buffer to store gnd coordinates read from SD in RAM. It's pretty big.
    LatLong_Coord_t gndCoordBuf[GND_NUM_LINES_IN_BUF];
    time_t earliestPassFound = UINT64_MAX;

    /* TODO assuming for now that SD access is slower than orbit propagation,
       so that it's faster to iterate through the SD file once and iterate through time for each SD iteration.
       If orbit propagation is actually slower, this should instead iterate through time once,
       and iterate through the SD file at each time step. 
       All of this doesn't matter if we have enough memory to store the entire gnd file in RAM at once. */
    for (uint16_t line = 0; line < numGndLines; line += GND_NUM_LINES_IN_BUF)
    {
        // 1. Read gnd coordinate lines into `gndCoordinateBuf'

        // 2. Propagate orbit from `currT' forwards.
        // At each time step, check for each gnd if sat is visible. If it is, record the time and break.
        time_t predictionTime = *currT; Vec3D_t predictionPos, gndPos;
        for (uint32_t step = 0; step < PASS_PRED_MAX_STEPS; step++)
        {
            Orbit_GetSatEquatorial(&predictionTime, &predictionPos);
            for (uint16_t bufLine = 0; bufLine < GND_NUM_LINES_IN_BUF; bufLine++)
            {
                convertLatLongToEq(gndCoordBuf + bufLine, &gndPos, &predictionTime);
                if (isSeenFromGnd(&predictionPos, &gndPos))
                {
                    if (earliestPassFound > predictionTime) earliestPassFound = predictionTime;
                    step = PASS_PRED_MAX_STEPS - 1; break;
                }
            }
            predictionTime += PASS_PRED_STEP;
        }
    }

    if (earliestPassFound != UINT64_MAX) *result = earliestPassFound;
    else *result = 0;

end: ;

    FRESULT fres_temp = f_close(&fd);
    if (fres == FR_OK) fres = fres_temp;
    *SD_stat = fres; return E_OK;

#undef errorcheck
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* RESTRICTED ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Orbit_X_Init()
{
    // Fetch TLE string from SD
    char tle_str[TLE_LENGTH];
    FRESULT fres = SD_getFileContents(SD_TLE_PATH, SD_TLE_FILENAME, tle_str, TLE_LENGTH);
    if (fres != FR_OK)
    {
#ifdef DEBUG_ENABLED
        switch (fres)
        {
        case FR_NO_FILE:
            SD_createFile(SD_TLE_PATH, SD_TLE_FILENAME);
            char placeholder_tle[TLE_LENGTH] = "1 39444U 13066AE  22153.47668570  .00001597  00000+0  19532-3 0  9990\n2 39444  97.6236 128.0555 0058478  82.3390 278.4460 14.83359372459309";
            SD_overwriteData(SD_TLE_PATH, SD_TLE_FILENAME, placeholder_tle, TLE_LENGTH);
            strcpy(tle_str, placeholder_tle);
            break;
        default:
            //TODO: error handling
            return fres;
        }
#else // Debug set up a file with a placeholder TLE
        return fres;
#endif
    }
#ifdef DEBUG_ENABLED
    if (!isValidTLE(tle_str)) Sys_RaiseLogicError(__FILE__, __LINE__, "TLE on SD not formatted");
#endif // Debug check formatting of TLEs on SD
    parseLines(&S_tle, tle_str, tle_str + (TLE_LENGTH / 2));
    S_tleValid = 1;
}

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Update sat TLEs
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]     tle_str - pointer to string formatted as a TLE
* @return           FRESULT containing result of SD interface
* @note				This updates both the runtime variable stored in this C file, and
*					the file stored on the SD card
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
FRESULT Orbit_X_SetTLEs(const char* tle_str)
{
    if (tle_str == NULL) Sys_RaiseLogicError(__FILE__, __LINE__, "null ptr passed");
    FRESULT fres = SD_overwriteData(SD_TLE_PATH, SD_TLE_FILENAME, tle_str, TLE_LENGTH);
    if (fres != FR_OK) return fres;
    parseLines(&S_tle, tle_str, tle_str + (TLE_LENGTH / 2));
    return fres;
}



/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Compute vector from Earth to sat in geodetic coordinates
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[output]     result - pointer to MAGType_CoordSpherical to populate with result
* @return            ES_ReturnType containing status of calculation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
//ES_ReturnType Orbit_GetSatGeodetic(MAGtype_CoordSpherical* result) {
//
//    Vec3D_t equatorialSatPos;
//    Orbit_GetSatEquatorial(&equatorialSatPos);
//
//    // This won't happen, but just in case:
//    if (equatorialSatPos.X.Axis == 0) {
//        equatorialSatPos.X.Axis = 0.0001;
//    }
//    if (equatorialSatPos.Y.Axis == 0) {
//        equatorialSatPos.Y.Axis = 0.0001;
//    }
//    if (equatorialSatPos.Z.Axis == 0) {
//        equatorialSatPos.Z.Axis = 0.0001;
//    }
//
//    double latitude = atan(equatorialSatPos.Z.Axis / sqrt(pow(equatorialSatPos.X.Axis, 2) + pow(equatorialSatPos.Y.Axis, 2)));
//
//    double longitude;
//    if (equatorialSatPos.X.Axis > 0) {
//        longitude  = atan(equatorialSatPos.Y.Axis/equatorialSatPos.X.Axis);
//    } else if (equatorialSatPos.X.Axis < 0 && equatorialSatPos.X.Axis >= 0) {
//        longitude  = atan(equatorialSatPos.Y.Axis/equatorialSatPos.X.Axis) + M_PI;
//    } else if (equatorialSatPos.X.Axis < 0 && equatorialSatPos.X.Axis < 0) {
//        longitude  = atan(equatorialSatPos.Y.Axis/equatorialSatPos.X.Axis) - M_PI;
//    }
//
//    double magnitude = sqrt(pow(equatorialSatPos.X.Axis, 2) + pow(equatorialSatPos.Y.Axis, 2) + pow(equatorialSatPos.Z.Axis, 2));
//    result->lambda = longitude;
//    result->phig = latitude;
//    result->r = magnitude;
//
//    return E_OK;
//}

#ifdef DEBUG_ENABLED

void Test_file_read() {
    char line[100];
    FIL fd; 
    FRESULT fres = f_open(&fd, "test.txt", FA_READ);
    while (f_gets(line, sizeof line, &fil)) {
        fprintf(COMM, line);
    }
    /* Close the file */
    f_close(&fil);

}

#endif