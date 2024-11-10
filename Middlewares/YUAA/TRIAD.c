/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TRIAD.h
* @brief TRIAD Algorithm Implementation
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Wesley A., Lijie Y.
* @version           1.0.0
* @date              2022.10.01
*
* @details           Estimate the attitude from two orthogonal vector observations
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "TRIAD.h"
#include "system_manager.h"
#include "OrbitProp.h"
#include "ff.h"
#include "User_types.h"
#include "arm_math.h"
#include "Geomag.h"
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define errorcheck(fres) if ((FRESULT)fres != FR_OK) return fres
#ifdef ARM_MATH_MATRIX_CHECK
#define mat_errorcheck(arm_math_status) if ((arm_status)arm_math_status != ARM_MATH_SUCCESS) Sys_RaiseLogicErrorMsg("%s", "matrix math error")
#else
#define mat_errorcheck(arm_math_status) ((void)arm_math_status)
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// TODO: naive cross product implementation because for some reason CMSIS-DSP doesn't have one
// See this: https://math.stackexchange.com/questions/1656219/how-to-prove-that-the-cross-product-of-two-vectors-is-a-linear-transformation
// for how to express a vector cross product as a matrix-vector multiplication (which CMSIS-DSP does have)
static void vec3_cross_f32(float32_t* v1, float32_t* v2, float32_t* result)
{
    arm_matrix_instance_f32 transform_matrix;

    float32_t transform_arr[9] = {
        0, -v1[2], v1[1],
        v1[2], 0, -v1[0],
        -v1[1], v1[0], 0
    };

    arm_mat_init_f32(&transform_matrix, 3, 3, transform_arr);

    arm_mat_vec_mult_f32(&transform_matrix, v2, result);
}

// Constructs the s:m:sxm or S:M:SxM matrix (the procedures are identical in either case). 
// Pass body frame vectors and bodyData as the target matrix for the former,
// and the equatorial frame vectors and equatorialData as the target matrix for the latter.
static void TRIAD_ConstructMatrix(const Vec3D_t* sun_vec, const Vec3D_t* mag_vec, float32_t* target_matrix, arm_matrix_instance_f32* target_instance)
{
    float32_t s[3];
    float32_t m[3];
    float32_t sxm[3];

    float32_t s_Modulus;
    arm_dot_prod_f32((float32_t*)sun_vec, (float32_t*)sun_vec, 3, &s_Modulus); arm_sqrt_f32(s_Modulus, &s_Modulus);
    arm_scale_f32((float32_t*)sun_vec, 1.0 / s_Modulus, s, 3);

    vec3_cross_f32((float32_t*)sun_vec, (float32_t*)mag_vec, m);

    float32_t m_Modulus;
    arm_dot_prod_f32(m, m, 3, &m_Modulus); arm_sqrt_f32(m_Modulus, &m_Modulus);
    arm_scale_f32(m, 1.0 / m_Modulus, m, 3);

    vec3_cross_f32(s, m, sxm);

    target_matrix[0 * 3 + 0] = s[0];
    target_matrix[1 * 3 + 0] = s[1];
    target_matrix[2 * 3 + 0] = s[2];

    target_matrix[0 * 3 + 1] = m[0];
    target_matrix[1 * 3 + 1] = m[1];
    target_matrix[2 * 3 + 1] = m[2];

    target_matrix[0 * 3 + 2] = sxm[0];
    target_matrix[1 * 3 + 2] = sxm[1];
    target_matrix[2 * 3 + 2] = sxm[2];

    arm_mat_init_f32(target_instance, 3, 3, target_matrix);
}

FRESULT TRIAD_GetAttMatrix(const Vec3D_t* body_B, const Vec3D_t* body_S, time_t* t,
	                        arm_matrix_instance_f32* attitudeMatrix)
{
    assert(body_B != NULL && body_S != NULL && t != NULL && attitudeMatrix != NULL);
#ifdef ARM_MATH_MATRIX_CHECK
    if (attitudeMatrix->numCols != 3 || attitudeMatrix->numRows != 3)
        Sys_RaiseLogicErrorMsg("%s", "wrong matrix param size");
#endif

    float32_t bodyData[3 * 3];
    arm_matrix_instance_f32 BODY_MATRIX;

    float32_t equatorialData[3 * 3];
    arm_matrix_instance_f32 EQUATORIAL_MATRIX;


    TRIAD_ConstructMatrix(body_S, body_B, bodyData, &BODY_MATRIX);

    //Get equitorial sun vector
    Vec3D_t eq_sun_vec;
    FRESULT retstat = Orbit_GetSunEquatorial(t, &eq_sun_vec); errorcheck(retstat);

    //Get sat position vector
    Vec3D_t eq_sat_vec;
    Orbit_GetSatEquatorial(t, &eq_sat_vec);

    //Get equitorial magnetic vector
    Vec3D_t eq_mag_vec;
    FRESULT retmag = Geomag_GetMagEquatorial(t, &eq_sat_vec, &eq_mag_vec);
    // TODO add FRESULT error checking

    TRIAD_ConstructMatrix(&eq_sun_vec, &eq_mag_vec, equatorialData, &EQUATORIAL_MATRIX);

    float32_t body_t[3 * 3];
    arm_matrix_instance_f32 BODY_T;
    arm_mat_init_f32(&BODY_T, 3, 3, body_t);
    arm_status math_retstat = arm_mat_trans_f32(&BODY_MATRIX, &BODY_T); mat_errorcheck(math_retstat);

    math_retstat = arm_mat_mult_f32(&EQUATORIAL_MATRIX, &BODY_T, attitudeMatrix); mat_errorcheck(math_retstat);

    return retstat;
}

FRESULT TRIAD_GetOmegaVec(const MeasPair_t* pairB, const MeasPair_t* pairS,
                          time_t* t, Vec3D_t* omega)
{
    float32_t attMat0_data[3 * 3];
    arm_matrix_instance_f32 attMat0;
    arm_mat_init_f32(&attMat0, 3, 3, attMat0_data);

    float32_t attMat1_data[3 * 3];
    arm_matrix_instance_f32 attMat1;
    arm_mat_init_f32(&attMat1, 3, 3, attMat1_data);

    FRESULT fres;
    fres = TRIAD_GetAttMatrix(&pairB->M0, &pairS->M0, t, &attMat0); errorcheck(fres);
    fres = TRIAD_GetAttMatrix(&pairB->M1, &pairS->M1, t, &attMat0); errorcheck(fres);

    // 1. Take numerical derivative of the two attitude matrices
    arm_matrix_instance_f32 difference;
    // TODO Assuming that arm_mat_sub does first parameter - second parameter
    arm_mat_sub_f32(&attMat1, &attMat0, &difference);

    assert(pairB->intv == pairS->intv);
    float32_t time_diff = pairB->intv;
    float32_t derivative_data[3*3];
    arm_matrix_instance_f32 derivative;
    arm_mat_init_f32(&derivative, 3, 3, derivative_data);
    arm_mat_scale_f32(&difference, (1 / time_diff), &derivative);

    // 2. Convert that into the omega vector
    float32_t attitude_transpose_data[3*3];
    arm_matrix_instance_f32 attitude_transpose;
    arm_mat_init_f32(&attitude_transpose, 3, 3, attitude_transpose_data);
    arm_mat_trans_f32(&attMat1, &attitude_transpose);
    
    float32_t omega_data[3*3];
    arm_matrix_instance_f32 omega_matrix;
    arm_mat_init_f32(&omega_matrix, 3, 3, omega_data);
    arm_mat_mult_f32(&derivative, &attitude_transpose, &omega_matrix);

    // Assuming that pData returns a flattened array from left to right, then up to down of the matrix
    omega->X = omega_matrix.pData[7];
    omega->Y = omega_matrix.pData[2];
    omega->Z = omega_matrix.pData[3];

    return fres;
}
