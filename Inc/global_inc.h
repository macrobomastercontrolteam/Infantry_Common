#ifndef _GLOBAL_INC_H
#define _GLOBAL_INC_H

#define INFANTRY_2018_MECANUM 0
#define INFANTRY_2023_MECANUM 1
#define INFANTRY_2023_SWERVE 2
#define INFANTRY_BIPED 3
#define SENTRY_2023_MECANUM 4
#define ENGINEER_2024_MECANUM 5

/********************* Only Modify this area (start) *********************/
#define ROBOT_TYPE ENGINEER_2024_MECANUM
#define SENTRY_HW_TEST 0
#define CV_INTERFACE 0
#define DEBUG_CV_WITH_USB 0
#define TEST_NO_REF 1
/********************* Only Modify this area (end) *********************/

#if (ROBOT_TYPE != SENTRY_2023_MECANUM) && SENTRY_HW_TEST
#error "SENTRY_HW_TEST is only for SENTRY_2023_MECANUM"
#endif

#if DEBUG_CV_WITH_USB && !CV_INTERFACE
#error "DEBUG_CV_WITH_USB is only for CV_INTERFACE"
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
