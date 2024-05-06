
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

#ifndef AHRS_MIDDLEWARE_H
#define AHRS_MIDDLEWARE_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

// Convert angle (degree) to radian
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

// Convert radian to angle (degree)
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(fp32 *high);
extern void AHRS_get_latitude(fp32 *latitude);
extern fp32 AHRS_invSqrt(fp32 num);
extern fp32 AHRS_sinf(fp32 angle);
extern fp32 AHRS_cosf(fp32 angle);
extern fp32 AHRS_tanf(fp32 angle);
extern fp32 AHRS_asinf(fp32 sin);
extern fp32 AHRS_acosf(fp32 cos);
extern fp32 AHRS_atan2f(fp32 y, fp32 x);
#endif
