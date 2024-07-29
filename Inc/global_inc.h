#ifndef _GLOBAL_INC_H
#define _GLOBAL_INC_H

#define INFANTRY_2023_MECANUM 0
#define INFANTRY_2024_MECANUM 1
#define INFANTRY_2023_SWERVE 2
#define INFANTRY_2024_BIPED 3
#define SENTRY_2023_MECANUM 4

/********************* Only Modify this area (start) *********************/
#define ROBOT_TYPE SENTRY_2023_MECANUM
#define CV_INTERFACE 1
#define DEBUG_CV_WITH_USB 0
#define ENABLE_LASER 1
#define USE_SERVO_TO_STIR_AMMO 0
/********************* Only Modify this area (end) *********************/

#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
// Yaw use DaMiao 4310
#define ROBOT_YAW_IS_4310 1
#else
#define ROBOT_YAW_IS_4310 0
#endif

#if ((ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_BIPED))
#define ROBOT_YAW_HAS_SLIP_RING 1
#else
#define ROBOT_YAW_HAS_SLIP_RING 0
#endif

#if ((ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == SENTRY_2023_MECANUM))
#define ROBOT_CHASSIS_USE_MECANUM 1
#else
#define ROBOT_CHASSIS_USE_MECANUM 0
#endif

#if DEBUG_CV_WITH_USB && !CV_INTERFACE
#error "DEBUG_CV_WITH_USB is only for CV_INTERFACE"
#endif

#if (ROBOT_TYPE == SENTRY_2023_MECANUM) && !CV_INTERFACE
#error "SENTRY_2023_MECANUM should use CV_INTERFACE for competition"
#endif

#if (ROBOT_TYPE != INFANTRY_2023_SWERVE) && USE_SERVO_TO_STIR_AMMO
#error "Only INFANTRY_2023_SWERVE supports USE_SERVO_TO_STIR_AMMO"
#endif

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif /* _GLOBAL_INC_H */
