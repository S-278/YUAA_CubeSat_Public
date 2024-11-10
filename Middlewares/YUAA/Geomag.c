/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file Geomag.c
* @brief Implementation of YUAA Geomag Library
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Wesley A.
* @details           Calculate the magnetic vector coordinates. Most of the code was adapted
                     from the World Magnetic Model Library.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include "Geomag.h"
#include "AppTasks.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include "VecUtils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#define WMM_FILENAME ("WMM.COF")

#define ATanH(x)	    (0.5 * log((1 + x) / (1 - x)))

/* UTC time at which 0deg N and 0deg E was along the X axis in equatorial coordinates.
   This happens at the time of a vernal (spring) equinox.
   (Actually this also happens every day whenever RA 0 crosses the meridian at 0deg N 0deg E,
   but an equinox is a more convenient reference point.) */
#define EQUINOX_TIME ((time_t) 1695451800)
/* Radius of the spherical Earth in meters */
#define EARTH_R ((float)6378100)

/* Angular speed of Earth's rotation about its axis, in rad/s */
#define EARTH_ROT_SPEED ((float)7.2921159E-5)

#define errorcheck_ret(fresult) if (fresult != FR_OK) return fresult
#define errorcheck_goto(fresult, lbl) if (fresult != FR_OK) goto lbl

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static void Geomag_InitializeModel(Geomag_MagneticModel_t * MagneticModel) {
    MagneticModel->CoefficientFileEndDate = 0;
    MagneticModel->EditionDate = 0;
    strcpy(MagneticModel->ModelName, "");
    MagneticModel->SecularVariationUsed = 0;
    MagneticModel->epoch = 0;
    MagneticModel->nMax = 0;
    MagneticModel->nMaxSecVar = 0;

    for (int i = 0; i < CALCULATE_NUMTERMS(MAX_N_MODE); i++) {
        MagneticModel->Main_Field_Coeff_G[i] = 0;
        MagneticModel->Main_Field_Coeff_H[i] = 0;
        MagneticModel->Secular_Var_Coeff_G[i] = 0;
        MagneticModel->Secular_Var_Coeff_H[i] = 0;
    }
}

static FRESULT Geomag_ReadMagModel(char* filename, Geomag_MagneticModel_t *MagneticModel) {
    // Type to read each line into
    typedef struct {
        int n, m;
        float32_t gnm, hnm, dgnm, dhnm;
    } WMM_Line;

    // Setup variable
    Geomag_InitializeModel(MagneticModel);
    FIL MAG_COF_File;
    WMM_Line line;
    int index;
    float32_t epoch;
    UINT bytes_read;

    epoch = 2020;

    // Model initial configuration (kept in from original implementation)
    MagneticModel->Main_Field_Coeff_H[0] = 0.0;
    MagneticModel->Main_Field_Coeff_G[0] = 0.0;
    MagneticModel->Secular_Var_Coeff_H[0] = 0.0;
    MagneticModel->Secular_Var_Coeff_G[0] = 0.0;

    float gnm[] = {
        -29404.500000, -1450.699951, -2500.000000, 2982.000000, 1676.800049, 1363.900024, -2381.000000, 1236.199951, 525.700012, 
        903.099976, 809.400024, 86.199997, -309.399994, 47.900002, -234.399994, 363.100006, 187.800003, -140.699997, -151.199997, 
        13.700000, 65.900002, 65.599998, 73.000000, -121.500000, -36.200001, 13.500000, -64.699997, 80.599998, -76.800003, -8.300000, 
        56.500000, 15.800000, 6.400000, -7.200000, 9.800000, 23.600000, 9.800000, -17.500000, -0.400000, -21.100000, 15.300000, 
        13.700000, -16.500000, -0.300000, 5.000000, 8.200000, 2.900000, -1.400000, -1.100000, -13.300000, 1.100000, 8.900000, 
        -9.300000, -11.900000, -1.900000, -6.200000, -0.100000, 1.700000, -0.900000, 0.600000, -0.900000, 1.900000, 1.400000, -2.400000, 
        -3.900000, 3.000000, -1.400000, -2.500000, 2.400000, -0.900000, 0.300000, -0.700000, -0.100000, 1.400000, -0.600000, 0.200000, 
        3.100000, -2.000000, -0.100000, 0.500000, 1.300000, -1.200000, 0.700000, 0.300000, 0.500000, -0.200000, -0.500000, 0.100000, 
        -1.100000, -0.300000
    };
    float hnm[] = {
        0.000000, 4652.899902, 0.000000, -2991.600098, -734.799988, 0.000000, -82.199997, 241.800003, -542.900024, 0.000000, 282.000000, 
        -158.399994, 199.800003, -350.100006, 0.000000, 47.700001, 208.399994, -121.300003, 32.200001, 99.099998, 0.000000, -19.100000, 
        25.000000, 52.700001, -64.400002, 9.000000, 68.099998, 0.000000, -51.400002, -16.799999, 2.300000, 23.500000, -2.200000, -27.200001, 
        -1.900000, 0.000000, 8.400000, -15.300000, 12.800000, -11.800000, 14.900000, 3.600000, -6.900000, 2.800000, 0.000000, -23.299999, 
        11.100000, 9.800000, -5.100000, -6.200000, 7.800000, 0.400000, -1.500000, 9.700000, 0.000000, 3.400000, -0.200000, 3.500000, 4.800000, 
        -8.600000, -0.100000, -4.200000, -3.400000, -0.100000, -8.800000, 0.000000, -0.000000, 2.600000, -0.500000, -0.400000, 0.600000, 
        -0.200000, -1.700000, -1.600000, -3.000000, -2.000000, -2.600000, 0.000000, -1.200000, 0.500000, 1.300000, -1.800000, 0.100000, 0.700000, 
        -0.100000, 0.600000, 0.200000, -0.900000, -0.000000, 0.500000
    };
    float dgnm[] = {
        6.700000, 7.700000, -11.500000, -7.100000, -2.200000, 2.800000, -6.200000, 3.400000, -12.200000, -1.100000, -1.600000, -6.000000, 5.400000, 
        -5.500000, -0.300000, 0.600000, -0.700000, 0.100000, 1.200000, 1.000000, -0.600000, -0.400000, 0.500000, 1.400000, -1.400000, -0.000000, 
        0.800000, -0.100000, -0.300000, -0.100000, 0.700000, 0.200000, -0.500000, -0.800000, 1.000000, -0.100000, 0.100000, -0.100000, 0.500000, 
        -0.100000, 0.400000, 0.500000, 0.000000, 0.400000, -0.100000, -0.200000, -0.000000, 0.400000, -0.300000, -0.000000, 0.300000, -0.000000, 
        -0.000000, -0.400000, 0.000000, -0.000000, -0.000000, 0.200000, -0.100000, -0.200000, -0.000000, -0.100000, -0.200000, -0.100000, -0.000000, 
        -0.000000, -0.100000, -0.000000, 0.000000, -0.000000, -0.100000, 0.000000, -0.000000, -0.100000, -0.100000, -0.100000, -0.100000, 0.000000, 
        -0.000000, -0.000000, 0.000000, -0.000000, -0.000000, 0.000000, -0.000000, 0.000000, -0.000000, -0.000000, -0.000000, -0.100000
    };
    float dhnm[] = {
        0.000000, -25.100000, 0.000000, -30.200001, -23.900000, 0.000000, 5.700000, -1.000000, 1.100000, 0.000000, 0.200000, 6.900000, 3.700000, 
        -5.600000, 0.000000, 0.100000, 2.500000, -0.900000, 3.000000, 0.500000, 0.000000, 0.100000, -1.800000, -1.400000, 0.900000, 0.100000, 1.000000, 
        0.000000, 0.500000, 0.600000, -0.700000, -0.200000, -1.200000, 0.200000, 0.300000, 0.000000, -0.300000, 0.700000, -0.200000, 0.500000, -0.300000, 
        -0.500000, 0.400000, 0.100000, 0.000000, -0.300000, 0.200000, -0.400000, 0.400000, 0.100000, -0.000000, -0.200000, 0.500000, 0.200000, 0.000000, 
        -0.000000, 0.100000, -0.300000, 0.100000, -0.200000, 0.100000, -0.000000, -0.100000, 0.200000, -0.000000, 0.000000, -0.000000, 0.100000, 0.000000, 
        0.200000, -0.000000, 0.000000, 0.100000, -0.000000, -0.100000, 0.000000, -0.000000, 0.000000, -0.000000, 0.000000, -0.100000, 0.100000, -0.000000, 
        0.000000, -0.000000, 0.100000, -0.000000, -0.000000, 0.000000, -0.100000
    };

    for (int i = 0; i < 90; i++) {
        MagneticModel->Main_Field_Coeff_G[i+1] = gnm[i];
        MagneticModel->Secular_Var_Coeff_G[i+1] = dgnm[i];
        MagneticModel->Main_Field_Coeff_H[i+1] = hnm[i];
        MagneticModel->Secular_Var_Coeff_H[i+1] = dhnm[i];
    }

    return FR_OK;
    /*
    // Open file
    FRESULT fres = f_open(&MAG_COF_File, filename, FA_READ | FA_OPEN_EXISTING); errorcheck_ret(fres);

    // Model initial configuration (kept in from original implementation)
    MagneticModel->Main_Field_Coeff_H[0] = 0.0;
    MagneticModel->Main_Field_Coeff_G[0] = 0.0;
    MagneticModel->Secular_Var_Coeff_H[0] = 0.0;
    MagneticModel->Secular_Var_Coeff_G[0] = 0.0;

    // Read epoch
    //FRESULT epoch_read_res = f_read(&MAG_COF_File, &epoch, sizeof(float), &bytes_read);
    //if (epoch_read_res != 0) { fres = f_error(&MAG_COF_File); errorcheck_goto(fres, cleanup); }
    MagneticModel->epoch = epoch;

    // Read lines
    do
    {
        FRESULT line_read_res = f_read(&MAG_COF_File, &line, sizeof(WMM_Line), &bytes_read);
        if (line_read_res != 0) { fres = f_error(&MAG_COF_File); errorcheck_goto(fres, cleanup); }

        if(line.m <= line.n)
        {
            index = (line.n * (line.n + 1) / 2 + line.m);
            MagneticModel->Main_Field_Coeff_G[index] = line.gnm;
            MagneticModel->Secular_Var_Coeff_G[index] = line.dgnm;
            MagneticModel->Main_Field_Coeff_H[index] = line.hnm;
            MagneticModel->Secular_Var_Coeff_H[index] = line.dhnm;
        }
    } while(bytes_read == sizeof(WMM_Line));
cleanup:;
    FRESULT last_fres = f_close(&MAG_COF_File);
    if (fres == FR_OK)
    {
        fres = last_fres; // Keep first encountered error

        MagneticModel->nMax = MAX_N_MODE;
        MagneticModel->nMaxSecVar = MAX_N_MODE;
        MagneticModel->CoefficientFileEndDate = MagneticModel->epoch + 5;
    }

    return fres;
    */
}

static void Geomag_SetDefaults(Geomag_Ellipsoid_t *Ellip, Geomag_Geoid_t *Geoid) {
    Ellip->a = 6378.137; /*semi-major axis of the ellipsoid in */
    Ellip->b = 6356.7523142; /*semi-minor axis of the ellipsoid in */
    Ellip->fla = 1 / 298.257223563; /* flattening */
    arm_sqrt_f32(1 - (Ellip->b * Ellip->b) / (Ellip->a * Ellip->a), &(Ellip->eps)); /*first eccentricity */
    Ellip->epssq = (Ellip->eps * Ellip->eps); /*first eccentricity squared */
    Ellip->re = 6371.2; /* Earth's radius */

    /* Sets EGM-96 model file parameters */
    Geoid->NumbGeoidCols = 1441; /* 360 degrees of longitude at 15 minute spacing */
    Geoid->NumbGeoidRows = 721; /* 180 degrees of latitude  at 15 minute spacing */
    Geoid->NumbHeaderItems = 6; /* min, max lat, min, max long, lat, long spacing*/
    Geoid->ScaleFactor = 4; /* 4 grid cells per degree at 15 minute spacing  */
    Geoid->NumbGeoidElevs = Geoid->NumbGeoidCols * Geoid->NumbGeoidRows;
    Geoid->Geoid_Initialized = 0; /*  Geoid will be initialized only if this is set to zero */
    Geoid->UseGeoid = MAG_USE_GEOID;
}

static void Geomag_CartesianToGeodetic(Geomag_Ellipsoid_t* Ellip, float32_t x, float32_t y, float32_t z, Geomag_CoordGeodetic_t* CoordGeodetic) {
        
    float32_t modified_b,r,e,f,p,q,d,v,g,t,zlong,rlat;

    /*
    *   1.0 compute semi-minor axis and set sign to that of z in order
    *       to get sign of Phi correct
    */

    if (z < 0.0) modified_b = -Ellip->b;
    else  modified_b = Ellip->b;

    /*
    *   2.0 compute intermediate values for latitude
    */
    arm_sqrt_f32(x*x + y*y, &r);
    e= ( modified_b*z - (Ellip->a*Ellip->a - modified_b*modified_b) ) / ( Ellip->a*r );
    f= ( modified_b*z + (Ellip->a*Ellip->a - modified_b*modified_b) ) / ( Ellip->a*r );
    /*
    *   3.0 find solution to:
    *       t^4 + 2*E*t^3 + 2*F*t - 1 = 0
    */
    p= (4.0 / 3.0) * (e*f + 1.0);
    q= 2.0 * (e*e - f*f);
    d= p*p*p + q*q;

    if( d >= 0.0 ) 
        {
        arm_sqrt_f32(d, &d);
        v= pow( (d - q), (1.0 / 3.0) )
            - pow( (d+ q), (1.0 / 3.0) );
        } 
    else 
        {
        float32_t sqrt_p;
        arm_sqrt_f32(-p, &sqrt_p);
        v= 2.0 * sqrt_p
            * arm_cos_f32( acos( q/(p * sqrt_p) ) / 3.0 );
        }
    /*
    *   4.0 improve v
    *       NOTE: not really necessary unless point is near pole
    */
    if( v*v < fabs(p) ) {
            v= -(v*v*v + 2.0*q) / (3.0*p);
    }
    arm_sqrt_f32(e*e + v, &g);
    g = (g + e) / 2.0;
    arm_sqrt_f32(g*g  + (f - v*g)/(2.0*g - e), &t);
    t = t - g;

    arm_atan2_f32((Ellip->a * (1.0 - t * t)), (2.0 * modified_b * t), &rlat); // ORIGINAL: rlat =atan( (Ellip->a*(1.0 - t*t)) / (2.0*modified_b*t) );
    CoordGeodetic->phi = RAD2DEG(rlat);
        
    /*
    *   5.0 compute height above ellipsoid
    */
    CoordGeodetic->HeightAboveEllipsoid = (r - Ellip->a*t) * arm_cos_f32(rlat) + (z - modified_b) * arm_sin_f32(rlat);

    /*
    *   6.0 compute longitude east of Greenwich
    */
    arm_atan2_f32(y, x, &zlong);
    if( zlong < 0.0 )
            zlong= zlong + 2*PI;

    CoordGeodetic->lambda = RAD2DEG(zlong);
    while(CoordGeodetic->lambda > 180)
    {
        CoordGeodetic->lambda-=360;
    }
}

static void Geomag_GeodeticToSpherical(Geomag_Ellipsoid_t* Ellip, Geomag_CoordGeodetic_t* CoordGeodetic, Geomag_CoordSpherical_t *CoordSpherical) {
    float32_t CosLat, SinLat, rc, xp, zp; /*all local variables */

    /*
     ** Convert geodetic coordinates, (defined by the WGS-84
     ** reference ellipsoid), to Earth Centered Earth Fixed Cartesian
     ** coordinates, and then to spherical coordinates.
     */

    CosLat = arm_cos_f32(DEG2RAD(CoordGeodetic->phi));
    SinLat = arm_sin_f32(DEG2RAD(CoordGeodetic->phi));

    /* compute the local radius of curvature on the WGS-84 reference ellipsoid */
    arm_sqrt_f32(1.0 - Ellip->epssq * SinLat * SinLat, &rc);
    rc = Ellip->a / rc;

    /* compute ECEF Cartesian coordinates of specified point (for longitude=0) */

    xp = (rc + CoordGeodetic->HeightAboveEllipsoid) * CosLat;
    zp = (rc * (1.0 - Ellip->epssq) + CoordGeodetic->HeightAboveEllipsoid) * SinLat;

    /* compute spherical radius and angle lambda and phi of specified point */

    arm_sqrt_f32(xp * xp + zp * zp, &(CoordSpherical->r));
    CoordSpherical->phig = RAD2DEG(asin(zp / CoordSpherical->r)); /* geocentric latitude */
    CoordSpherical->lambda = CoordGeodetic->lambda; /* longitude */
}

static float32_t Geomag_TimeToYear(time_t* t) {
    struct tm date;

    // Shared memory guard because gmtime is not thread-safe
    SharedMem_BeginAccess();
    struct tm *temp = gmtime(t);
    memcpy(&date, temp, sizeof(struct tm));
    SharedMem_EndAccess();

    int year = date.tm_year + 1900;

    // TODO: Check approximation
    float32_t days = (year % 4 == 0) ? 366.0 : 365.0;

    return year + (date.tm_yday)/(days);
}

static void Geomag_TimelyModifyMagModel(time_t* t, Geomag_MagneticModel_t *MagneticModel, Geomag_MagneticModel_t *TimedMagneticModel) {
    float32_t DecimalYear = Geomag_TimeToYear(t);
    int n, m, index, a, b;
    TimedMagneticModel->EditionDate = MagneticModel->EditionDate;
    TimedMagneticModel->epoch = MagneticModel->epoch;
    TimedMagneticModel->nMax = MagneticModel->nMax;
    TimedMagneticModel->nMaxSecVar = MagneticModel->nMaxSecVar;
    a = TimedMagneticModel->nMaxSecVar;
    b = (a * (a + 1) / 2 + a);
    strcpy(TimedMagneticModel->ModelName, MagneticModel->ModelName);
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(index <= b)
            {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index] + (DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index] + (DecimalYear - MagneticModel->epoch) * MagneticModel->Secular_Var_Coeff_G[index];
                TimedMagneticModel->Secular_Var_Coeff_H[index] = MagneticModel->Secular_Var_Coeff_H[index];
                TimedMagneticModel->Secular_Var_Coeff_G[index] = MagneticModel->Secular_Var_Coeff_G[index];
            } else
            {
                TimedMagneticModel->Main_Field_Coeff_H[index] = MagneticModel->Main_Field_Coeff_H[index];
                TimedMagneticModel->Main_Field_Coeff_G[index] = MagneticModel->Main_Field_Coeff_G[index];
            }
        }
    }
}

static void Geomag_ComputeSphericalHarmonicVariables(Geomag_Ellipsoid_t* Ellip, Geomag_CoordSpherical_t* CoordSpherical, int nMax, Geomag_SphericalHarmonicVariables_t* SphVariables) {
    float32_t cos_lambda, sin_lambda;
    int m, n;
    cos_lambda = arm_cos_f32(DEG2RAD(CoordSpherical->lambda));
    sin_lambda = arm_sin_f32(DEG2RAD(CoordSpherical->lambda));
    /* for n = 0 ... model_order, compute (Radius of Earth / Spherical radius r)^(n+2)
    for n  1..nMax-1 (this is much faster than calling pow MAX_N+1 times).      */
    SphVariables->RelativeRadiusPower[0] = (Ellip->re / CoordSpherical->r) * (Ellip->re / CoordSpherical->r);
    for(n = 1; n <= nMax; n++)
    {
        SphVariables->RelativeRadiusPower[n] = SphVariables->RelativeRadiusPower[n - 1] * (Ellip->re / CoordSpherical->r);
    }

    /*
     Compute cos(m*lambda), sin(m*lambda) for m = 0 ... nMax
           cos(a + b) = cos(a)*cos(b) - sin(a)*sin(b)
           sin(a + b) = cos(a)*sin(b) + sin(a)*cos(b)
     */
    SphVariables->cos_mlambda[0] = 1.0;
    SphVariables->sin_mlambda[0] = 0.0;

    SphVariables->cos_mlambda[1] = cos_lambda;
    SphVariables->sin_mlambda[1] = sin_lambda;
    for(m = 2; m <= nMax; m++)
    {
        SphVariables->cos_mlambda[m] = SphVariables->cos_mlambda[m - 1] * cos_lambda - SphVariables->sin_mlambda[m - 1] * sin_lambda;
        SphVariables->sin_mlambda[m] = SphVariables->cos_mlambda[m - 1] * sin_lambda + SphVariables->sin_mlambda[m - 1] * cos_lambda;
    }
}

static void Geomag_PcupLow(float32_t *Pcup, float32_t *dPcup, float32_t x, int nMax) {
    int n, m, index, index1, index2, NumTerms;
    float32_t k, z;
    float32_t schmidtQuasiNorm[CALCULATE_NUMTERMS(nMax) + 1];
    Pcup[0] = 1.0;
    dPcup[0] = 0.0;
    /*sin (geocentric latitude) - sin_phi */
    arm_sqrt_f32((1.0 - x) * (1.0 + x), &z);

    /*	 First,	Compute the Gauss-normalized associated Legendre  functions*/
    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            if(n == m)
            {
                index1 = (n - 1) * n / 2 + m - 1;
                Pcup [index] = z * Pcup[index1];
                dPcup[index] = z * dPcup[index1] + x * Pcup[index1];
            } else if(n == 1 && m == 0)
            {
                index1 = (n - 1) * n / 2 + m;
                Pcup[index] = x * Pcup[index1];
                dPcup[index] = x * dPcup[index1] - z * Pcup[index1];
            } else if(n > 1 && n != m)
            {
                index1 = (n - 2) * (n - 1) / 2 + m;
                index2 = (n - 1) * n / 2 + m;
                if(m > n - 2)
                {
                    Pcup[index] = x * Pcup[index2];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2];
                } else
                {
                    k = (float32_t) (((n - 1) * (n - 1)) - (m * m)) / (float32_t) ((2 * n - 1) * (2 * n - 3));
                    Pcup[index] = x * Pcup[index2] - k * Pcup[index1];
                    dPcup[index] = x * dPcup[index2] - z * Pcup[index2] - k * dPcup[index1];
                }
            }
        }
    }
    /* Compute the ration between the the Schmidt quasi-normalized associated Legendre
     * functions and the Gauss-normalized version. */

    schmidtQuasiNorm[0] = 1.0;
    for(n = 1; n <= nMax; n++)
    {
        index = (n * (n + 1) / 2);
        index1 = (n - 1) * n / 2;
        /* for m = 0 */
        schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * (float32_t) (2 * n - 1) / (float32_t) n;

        for(m = 1; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            index1 = (n * (n + 1) / 2 + m - 1);
            float32_t sqrt;
            arm_sqrt_f32((float32_t) ((n - m + 1) * (m == 1 ? 2 : 1)) / (float32_t) (n + m), &sqrt);
            schmidtQuasiNorm[index] = schmidtQuasiNorm[index1] * sqrt;
        }

    }

    /* Converts the  Gauss-normalized associated Legendre
              functions to the Schmidt quasi-normalized version using pre-computed
              relation stored in the variable schmidtQuasiNorm */

    for(n = 1; n <= nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);
            Pcup[index] = Pcup[index] * schmidtQuasiNorm[index];
            dPcup[index] = -dPcup[index] * schmidtQuasiNorm[index];
            /* The sign is changed since the new WMM routines use derivative with respect to latitude
            insted of co-latitude */
        }
    }
}

static void Geomag_AssociatedLegendreFunction(Geomag_CoordSpherical_t* CoordSpherical, int nMax, Geomag_LegendreFunction_t *LegendreFunction) {
    float32_t sin_phi;
    int FLAG = 1;

    sin_phi = arm_sin_f32(DEG2RAD(CoordSpherical->phig)); /* sin  (geocentric latitude) */

    Geomag_PcupLow(LegendreFunction->Pcup, LegendreFunction->dPcup, sin_phi, nMax);
}

static void Geomag_Summation(Geomag_LegendreFunction_t *LegendreFunction, Geomag_MagneticModel_t *MagneticModel, Geomag_SphericalHarmonicVariables_t* SphVariables, Geomag_CoordSpherical_t* CoordSpherical, Geomag_GeoMagneticElements_t *MagneticResults) {
    int m, n, index;
    float32_t cos_phi;
    MagneticResults->Bz = 0.0;
    MagneticResults->By = 0.0;
    MagneticResults->Bx = 0.0;
    for(n = 1; n <= MagneticModel->nMax; n++)
    {
        for(m = 0; m <= n; m++)
        {
            index = (n * (n + 1) / 2 + m);

            /*		    nMax  	(n+2) 	  n     m            m           m
                    Bz =   -SUM (a/r)   (n+1) SUM  [g cos(m p) + h sin(m p)] P (sin(phi))
                                    n=1      	      m=0   n            n           n  */
            /* Equation 12 in the WMM Technical report.  Derivative with respect to radius.*/
            MagneticResults->Bz -= SphVariables->RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables->cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables->sin_mlambda[m])
                    * (float32_t) (n + 1) * LegendreFunction-> Pcup[index];

            /*		  1 nMax  (n+2)    n     m            m           m
                    By =    SUM (a/r) (m)  SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1             m=0   n            n           n  */
            /* Equation 11 in the WMM Technical report. Derivative with respect to longitude, divided by radius. */
            MagneticResults->By += SphVariables->RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables->sin_mlambda[m] -
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables->cos_mlambda[m])
                    * (float32_t) (m) * LegendreFunction-> Pcup[index];
            /*		   nMax  (n+2) n     m            m           m
                    Bx = - SUM (a/r)   SUM  [g cos(m p) + h sin(m p)] dP (sin(phi))
                               n=1         m=0   n            n           n  */
            /* Equation 10  in the WMM Technical report. Derivative with respect to latitude, divided by radius. */

            MagneticResults->Bx -= SphVariables->RelativeRadiusPower[n] *
                    (MagneticModel->Main_Field_Coeff_G[index] * SphVariables->cos_mlambda[m] +
                    MagneticModel->Main_Field_Coeff_H[index] * SphVariables->sin_mlambda[m])
                    * LegendreFunction-> dPcup[index];
        }
    }

    cos_phi = arm_cos_f32(DEG2RAD(CoordSpherical->phig));

    MagneticResults->By = MagneticResults->By / cos_phi;

    // TODO: Should we be checking for geographic poles? I do not think it is likely to be exactly +-90 (If it is, it calls one more function)
}

static void Geomag_RotateMagneticVector(Geomag_CoordSpherical_t* CoordSpherical, Geomag_CoordGeodetic_t* CoordGeodetic, Geomag_GeoMagneticElements_t* MagneticResultsSph, Geomag_GeoMagneticElements_t *MagneticResultsGeo) {
    float32_t Psi;
    /* Difference between the spherical and Geodetic latitudes */
    Psi = (PI / 180) * (CoordSpherical->phig - CoordGeodetic->phi);

    /* Rotate spherical field components to the Geodetic system */
    Vec_RotateSpher((Vec3D_t*)&(MagneticResultsSph->Bx), -Psi, 0, (Vec3D_t*)&(MagneticResultsGeo->Bx));
    /* original:
    MagneticResultsGeo->Bz = MagneticResultsSph->Bx * arm_sin_f32(Psi) + MagneticResultsSph->Bz * arm_cos_f32(Psi);
    MagneticResultsGeo->Bx = MagneticResultsSph->Bx * arm_cos_f32(Psi) - MagneticResultsSph->Bz * arm_sin_f32(Psi);
    MagneticResultsGeo->By = MagneticResultsSph->By;*/
}

static void Geomag_CalculateGeomagneticElements(Geomag_GeoMagneticElements_t *MagneticResultsGeo, Geomag_GeoMagneticElements_t *GeoMagneticElements) {
    arm_sqrt_f32(MagneticResultsGeo->Bx * MagneticResultsGeo->Bx + MagneticResultsGeo->By * MagneticResultsGeo->By, &(GeoMagneticElements->H));
    arm_sqrt_f32(GeoMagneticElements->H * GeoMagneticElements->H + MagneticResultsGeo->Bz * MagneticResultsGeo->Bz, &(GeoMagneticElements->F));
    arm_atan2_f32(GeoMagneticElements->By, GeoMagneticElements->Bx, &(GeoMagneticElements->Decl));
    GeoMagneticElements->Decl = RAD2DEG(GeoMagneticElements->Decl);
    arm_atan2_f32(GeoMagneticElements->Bz, GeoMagneticElements->H, &(GeoMagneticElements->Incl));
    GeoMagneticElements->Incl = RAD2DEG(GeoMagneticElements->Incl);
}

static void Geomag_Compute(Geomag_Ellipsoid_t* Ellip, Geomag_CoordSpherical_t* CoordSpherical, Geomag_CoordGeodetic_t* CoordGeodetic, Geomag_MagneticModel_t* TimedMagneticModel, Geomag_GeoMagneticElements_t *GeomagElements) {
    Geomag_LegendreFunction_t LegendreFunction;
    Geomag_SphericalHarmonicVariables_t SphVariables;
    Geomag_GeoMagneticElements_t MagneticResultsSph, MagneticResultsGeo;

    Geomag_ComputeSphericalHarmonicVariables(Ellip, CoordSpherical, TimedMagneticModel->nMax, &SphVariables); /* Compute Spherical Harmonic variables  */
    Geomag_AssociatedLegendreFunction(CoordSpherical, TimedMagneticModel->nMax, &LegendreFunction); /* Compute ALF  */
    Geomag_Summation(&LegendreFunction, TimedMagneticModel, &SphVariables, CoordSpherical, &MagneticResultsSph); /* Accumulate the spherical harmonic coefficients*/
    Geomag_RotateMagneticVector(CoordSpherical, CoordGeodetic, &MagneticResultsSph, &MagneticResultsGeo); /* Map the computed Magnetic fields to Geodeitic coordinates  */
    Geomag_CalculateGeomagneticElements(&MagneticResultsGeo, GeomagElements); /* Calculate the Geomagnetic elements, Equation 19 , WMM Technical report */
}

static void Geomag_HorizontalToEquatorial(Geomag_GeoMagneticElements_t* MagHorizontal, Vec3D_t *MagEquatorial, float32_t theta, float32_t phi_p) {
    float32_t theta_p = - theta - PI/2;
    //TODO: Check math. Should we indeed sum with Vernal Equinox as we discussed?

    Vec_RotateSpher((Vec3D_t*)&(MagHorizontal->Bx), theta_p, phi_p, MagEquatorial);
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

FRESULT Geomag_GetMagEquatorial(time_t* t, const Vec3D_t* SatEquatorial, Vec3D_t* MagEquatorial) {
    Vec3D_t SatLocal;
    //Angle between prime meridian and vernal equinox
    float32_t rotAngle = EARTH_ROT_SPEED * (- *t + EQUINOX_TIME);
    Vec_RotateSpher((Vec3D_t*)SatEquatorial, 0, rotAngle, &SatLocal);

    // Lattitude and Longitude angles for rotation at the end
    float32_t theta;
    arm_atan2_f32(sqrtf(powf(SatLocal.X, (float32_t)2) + powf(SatLocal.Y, (float32_t)2)), SatLocal.Z, &theta);
    float32_t phi;
    arm_atan2_f32(SatLocal.Y, SatLocal.X, &phi);
    float32_t phi_p = phi - rotAngle;

    Geomag_MagneticModel_t MagneticModel, TimedMagneticModel;
    Geomag_Ellipsoid_t Ellip;
    Geomag_Geoid_t Geoid;
    Geomag_CoordGeodetic_t CoordGeodetic;
    Geomag_CoordSpherical_t CoordSpherical;
    Geomag_GeoMagneticElements_t GeomagElements;

    FRESULT fres = Geomag_ReadMagModel(WMM_FILENAME, &MagneticModel); errorcheck_ret(fres);
    Geomag_InitializeModel(&TimedMagneticModel);
    Geomag_SetDefaults(&Ellip, &Geoid);
    Geoid.Geoid_Initialized = pdTRUE;

    Geomag_CartesianToGeodetic(&Ellip, SatLocal.X, SatLocal.Y, SatLocal.Z, &CoordGeodetic);
    Geomag_GeodeticToSpherical(&Ellip, &CoordGeodetic, &CoordSpherical);
    Geomag_TimelyModifyMagModel(t, &MagneticModel, &TimedMagneticModel);
    Geomag_Compute(&Ellip, &CoordSpherical, &CoordGeodetic, &TimedMagneticModel, &GeomagElements);

    Geomag_HorizontalToEquatorial(&GeomagElements, MagEquatorial, theta, phi_p);

    return FR_OK;
}

#ifdef DEBUG_ENABLED

typedef struct {
    float32_t x_check;
    float32_t y_check;
    float32_t z_check;
    Geomag_GeoMagneticElements_t geomagnetic_element;
    float32_t test_theta;
    float32_t test_phi;
} testGeomag_HorizontalToEquatorial_t;

testGeomag_HorizontalToEquatorial_t testArr[1] = {
        { // ROW 1
            .x_check = 0,
            .y_check = 0,
            .z_check = 1,
            .geomagnetic_element = {
                .Bx = 0.0, // calculated later
                .By = 0.0, // calculated later
                .Bz = 0.0, // calculated later
                .Decl = 0.0,
                .Incl = 0.0,
                .F = 0.0, // calculated later
                .H = 0.0 // calculated later
            },
            .test_theta = 0,
            .test_phi = 2.920407058,
        },
        { // ROW 2
            .x_check = -0.761299972235553,
            .y_check = 0.41282242220372,
            .z_check = 0.5,
            .geomagnetic_element = {
                .Bx = 0.0, // calculated later
                .By = 0.0, // calculated later
                .Bz = 0.0, // calculated later
                .Decl = 0.0,
                .Incl = 0.0,
                .F = 0.0, // calculated later
                .H = 0.0 // calculated later
            },
            .test_theta = 1.0471975511966,
            .test_phi = 5.0148021601414,
        },
        };

void calculateMagneticElements(Geomag_GeoMagneticElements_t* element) {
    element->F = 1.0f;
    element->H = element->F * arm_cos_f32(element->Incl);
    element->Bx = element->H * arm_cos_f32(element->Decl);
    element->By = element->H * arm_sin_f32(element->Decl);
    element->Bz = element->F * arm_sin_f32(element->Incl);
}

void testGeomag_HorizontalToEquatorial() {
    Vec3D_t testeq1;

    for (int i = 0; i < 3; i++) {

        // Zero out the testeq1
        memset(&testeq1, 0, sizeof(testeq1));

        calculateMagneticElements(&testArr[i].geomagnetic_element);
        Geomag_HorizontalToEquatorial(&testArr[i].geomagnetic_element, &testeq1, testArr[i].test_theta, testArr[i].test_phi);
        debug_msg("Equatorial coordinates (x, y, z) for horizontal (%f,%f,%f): \n(%f, %f, %f)", testArr[i].geomagnetic_element.Bx, testArr[i].geomagnetic_element.By, testArr[i].geomagnetic_element.Bz, testeq1.X, testeq1.Y, testeq1.Z);
    }

    // float32_t test_theta;
    // float32_t test_phi;
    // Geomag_GeoMagneticElements_t testhor1;
    // Geomag_GeoMagneticElements_t testhor2;
    // Geomag_GeoMagneticElements_t testhor3;
    // Vec3D_t testeq1;
    // Vec3D_t testeq2;
    // Vec3D_t testeq3;
    // char buffer[100];

    // //sat coordinates (rad) wrt prime meridian
    // test_theta = 0;
    // test_phi = 2.920407058;

    // //test 1:
    // testhor1.Decl = 0; /* Angle between the magnetic field vector and true north, positive east*/
    // testhor1.Incl = 0; /*Angle between the magnetic field vector and the horizontal plane, positive down*/
    // testhor1.F = 1; /*Magnetic Field Strength*/
    // testhor1.H = testhor1.F * arm_cos_f32(testhor1.Incl); /*Horizontal Magnetic Field Strength*/
    // testhor1.Bx = testhor1.H * arm_cos_f32 (testhor1.Decl); /* Northern component of the magnetic field vector */
    // testhor1.By = testhor1.H * arm_sin_f32 (testhor1.Decl); /*Eastern component of the magnetic field vector*/
    // testhor1.Bz= testhor1.F * arm_sin_f32(testhor1.Incl); /*Downward component of the magnetic field vector*/

    // //test 2:
    // testhor2.Decl = 0;
    // testhor2.Incl = -4.188790204786;
    // testhor2.F = 1;
    // testhor2.H = testhor2.F * arm_cos_f32(testhor2.Incl); 
    // testhor2.Bx = testhor2.H * arm_cos_f32 (testhor2.Decl);
    // testhor2.By = testhor2.H * arm_sin_f32 (testhor2.Decl);
    // testhor2.Bz= testhor2.F * arm_sin_f32(testhor2.Incl);

    // //test 3:
    // testhor3.Decl = 2.094395102;
    // testhor3.Incl = -2.094395102;
    // testhor3.F = 1;
    // testhor3.H = testhor3.F * arm_cos_f32(testhor3.Incl); 
    // testhor3.Bx = testhor3.H * arm_cos_f32 (testhor3.Decl);
    // testhor3.By = testhor3.H * arm_sin_f32 (testhor3.Decl);
    // testhor3.Bz= testhor3.F * arm_sin_f32(testhor3.Incl);

    // Geomag_HorizontalToEquatorial(&testhor1, &testeq1, test_theta, test_phi);
    // Geomag_HorizontalToEquatorial(&testhor2, &testeq2, test_theta, test_phi);
    // Geomag_HorizontalToEquatorial(&testhor3, &testeq3, test_theta, test_phi);
    // sprintf(buffer,"Equatorial coordinates (x, y, z) for horizontal (%f,%f,%f): \n(%f, %f, %f)", testhor1.Bx, testhor1.By, testhor1.Bz, testeq1.X, testeq1.Y, testeq1.Z);
    // debug_msg(__FILE__, __LINE__, buffer);
    // sprintf(buffer,"Equatorial coordinates (x, y, z) for horizontal (%f,%f,%f): \n(%f, %f, %f)", testhor2.Bx, testhor2.By, testhor2.Bz, testeq2.X, testeq2.Y, testeq2.Z);
    // debug_msg(__FILE__, __LINE__, buffer);
    // sprintf(buffer,"Equatorial coordinates (x, y, z) for horizontal (%f,%f,%f): \n(%f, %f, %f)", testhor3.Bx, testhor3.By, testhor3.Bz, testeq3.X, testeq3.Y, testeq3.Z);
    // debug_msg(__FILE__, __LINE__, buffer);
}

void testGeomag_GetMagEquatorial(){
    time_t t;
    Vec3D_t testSatEquatorial;
    Vec3D_t testMagEquatorial;

    t = 1713844800;

    //3D vector with vernal equinox as x-axis
    //TODO: Initialize the test sat equatorial coordinates
    testSatEquatorial.X = 0;
    testSatEquatorial.Y = 0;
    testSatEquatorial.Z = 0;

    Geomag_GetMagEquatorial(&t, &testSatEquatorial, &testMagEquatorial);

    debug_msg("Equatorial coordinates (x, y, z): (%f, %f, %f)", testMagEquatorial.X,
    	    testMagEquatorial.Y, testMagEquatorial.Z);
}

void testGeomag_LoadWMM() {
    Geomag_MagneticModel_t model;
    Geomag_InitializeModel(&model);
    Geomag_ReadMagModel(WMM_FILENAME, &model);

    for (int i = 0; i < 91; i++) {
        debug_msg("%f %f %f %f\n",
                model.Main_Field_Coeff_G[i],
                model.Secular_Var_Coeff_G[i],
                model.Main_Field_Coeff_H[i],
                model.Secular_Var_Coeff_H[i]);
    }
}

FRESULT testGeomag_WMMInstanceTest(time_t *t, float32_t lat, float32_t lon, float32_t height) {
	 debug_msg(
	        	"(Params) Time: %d, Lat: %f; Lon: %f; Height: %f\n",
				*t,
	    		lat,//CoordGeodetic.phi,
				lon,//CoordGeodetic.lambda,
				height//CoordGeodetic.HeightAboveEllipsoid
	    );

    Geomag_MagneticModel_t MagneticModel, TimedMagneticModel;
    Geomag_Ellipsoid_t Ellip;
    Geomag_Geoid_t Geoid;
    Geomag_CoordGeodetic_t CoordGeodetic;
    Geomag_CoordSpherical_t CoordSpherical;
    Geomag_GeoMagneticElements_t GeomagElements;

    FRESULT fres = Geomag_ReadMagModel(WMM_FILENAME, &MagneticModel); errorcheck_ret(fres);
    Geomag_InitializeModel(&TimedMagneticModel);
    Geomag_SetDefaults(&Ellip, &Geoid);
    Geoid.Geoid_Initialized = pdTRUE;
    
    CoordGeodetic.phi = lon;
    CoordGeodetic.lambda = lat;
    CoordGeodetic.HeightAboveEllipsoid = height;
    CoordGeodetic.HeightAboveGeoid = 0;
    CoordGeodetic.UseGeoid = 0;

    Geomag_GeodeticToSpherical(&Ellip, &CoordGeodetic, &CoordSpherical);
    debug_msg(
    		"Phi: %f; Lambda: %f; Height: %f\n",
			CoordSpherical.phig,
			CoordSpherical.lambda,
			CoordSpherical.r
	);

    Geomag_TimelyModifyMagModel(t, &MagneticModel, &TimedMagneticModel);
    Geomag_Compute(&Ellip, &CoordSpherical, &CoordGeodetic, &TimedMagneticModel, &GeomagElements);

    debug_msg(
        "D: %f; I: %f; F: %f; H: %f; X: %f; Y: %f; Z: %f\n",
        GeomagElements.Decl,
        GeomagElements.Incl,
        GeomagElements.F,
        GeomagElements.H,
        GeomagElements.Bx,
        GeomagElements.By,
        GeomagElements.Bz
    );

    return FR_OK;
}

void testGeomag_WMM() {
	// 2020 -- 80N 0E 0H -> D: -1.28
	time_t t2020 = 1577854800;
	testGeomag_WMMInstanceTest(&t2020, 80.0, 0.0, 0.0);
}

#endif
