/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      Attitude calculation middleware, providing related functions for attitude calculation
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Íê³É
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include "main.h"

void AHRS_get_height(fp32* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

void AHRS_get_latitude(fp32* latitude)
{
    if (latitude != NULL)
    {
        // *latitude = 22.0f; // shenzhen
        *latitude = 47.39f; // university of washington
    }
}

/**
 * @brief          fast inverse square root function
 */
fp32 AHRS_invSqrt(fp32 num)
{
    return 1/sqrtf(num);

//    fp32 halfnum = 0.5f * num;
//    fp32 y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(fp32*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

fp32 AHRS_sinf(fp32 angle)
{
    if (fabs(angle) <= 1e-6f)
    {
        return 0.0f;
    }
    else
    {
        return arm_sin_f32(angle);
    }
}

fp32 AHRS_cosf(fp32 angle)
{
    if (fabs(angle) <= 1e-6f)
    {
        return 1.0f;
    }
    else
    {
        return arm_cos_f32(angle);
    }
}

fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}
/**
 * @brief          asin function for fp32
 * @author         RM
 * @param[in]      max 1.0f, min -1.0f
 * @retval         unit rad
 */
fp32 AHRS_asinf(fp32 sin)
{
    return asinf(sin);
}

fp32 AHRS_acosf(fp32 cos)
{
    return acosf(cos);
}

/**
 * @brief          arctangent
 * @author         RM
 * @param[in]      max inf, min -inf
 * @param[in]      max inf, min -inf
 * @retval         unit rad
 */

fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}
