#ifndef _GLOBAL_INC_H
#define _GLOBAL_INC_H

//Robot type selection
#define INFANTRY_2023_MECANUM 0
#define INFANTRY_2024_MECANUM 1
#define INFANTRY_2023_SWERVE 2
#define INFANTRY_2024_BIPED 3
#define SENTRY_2023_MECANUM 4
#define HERO_2025_MECANUM 5
#define SENTRY_2026_OMNI 6
#define INFANTRY_2026_MECANUM  7
#define INFANTRY_2026_STANDARD 8

//Competition type selection
#define RMUL 0
#define RMUC 1

//Supercapacitor type selection
#define UBC_SUPERCAP 1
#define SJTU_SUPERCAP 2
#define MACRM_SUPERCAP 3

//Power train selection
#define ROBOT_CHASSIS_USE_MECANUM 0
#define ROBOT_CHASSIS_USE_OMNI 1
#define ROBOT_CHASSIS_USE_SWERVE 2
#define ROBOT_CHASSIS_USE_BIPED 3

#define POWER_TRAIN_USE_3508_MOTOR 0
#define POWER_TRAIN_USE_4010_MOTOR 1
#define POWER_TRAIN_USE_SPECIAL_CTRL 2

#define REMOTE_USE_DR16 0
#define REMOTE_USE_VT13 1
/********************* Only Modify this area (start) *********************/
#define ROBOT_TYPE HERO_2025_MECANUM
#define COMPETITION_TYPE RMUL
#define SUPERCAP_TYPE MACRM_SUPERCAP
#define CV_INTERFACE 1
#define DEBUG_CV_WITH_USB 0
#define DEBUG_CV 1 // set to 1 before the game starts for INFANTRY & HERO ONLY
#define ENABLE_LASER 1
#define VOFA_UART_USE 0 // 0 to disable VOFA, 1 for uart6(shown as uart1)(Refree_UART,3_pin), 2 for uart1(shown as uart2) (CV_UART,4_pin),

#define USE_SERVO_TO_STIR_AMMO 0
#define ENABLE_HIGHER_BAUD_RATE_FOR_CV 1

#define REMOTE_TYPE REMOTE_USE_VT13
/********************* Only Modify this area (end) *********************/

#if (VOFA_UART_USE == 1)
#define ENABLE_REFREE_UART_PORT 0
#else
#define ENABLE_REFREE_UART_PORT 1
#endif

#if ((ROBOT_TYPE == INFANTRY_2024_MECANUM)||(ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI))
#define CHASSIS_POWER_CONTROL 1 
#else
#define CHASSIS_POWER_CONTROL 0 
#endif

#if ((ROBOT_TYPE == INFANTRY_2024_MECANUM)||(ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI))
#define CAN_PASS_REF_INFO 1 
#else
#define CAN_PASS_REF_INFO 0 
#endif

#if ((ROBOT_TYPE == INFANTRY_2024_BIPED) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == INFANTRY_2026_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI) || (ROBOT_TYPE == INFANTRY_2026_STANDARD))
// Yaw use DaMiao 4310
#define ROBOT_YAW_IS_4310 1
#else
#define ROBOT_YAW_IS_4310 0
#endif

#if (ROBOT_TYPE == HERO_2025_MECANUM)
// Pitch use DaMiao 4310
#define ROBOT_PITCH_IS_4310 0
#define ROBOT_PITCH_IS_4340 1
#elif (ROBOT_TYPE == SENTRY_2026_OMNI)
#define ROBOT_PITCH_IS_3507 1
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
#define ROBOT_PITCH_IS_4310 1
#define ROBOT_PITCH_IS_4340 0
#else
#define ROBOT_PITCH_IS_4310 0
#define ROBOT_PITCH_IS_4340 0
#endif



#if ((ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_BIPED) || (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == INFANTRY_2026_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI))
#define ROBOT_YAW_HAS_SLIP_RING 1
#else
#define ROBOT_YAW_HAS_SLIP_RING 0
#endif

#if ((ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2026_MECANUM))
#define WHEEL_TYPE ROBOT_CHASSIS_USE_MECANUM 
#define MOTOR_TYPE POWER_TRAIN_USE_3508_MOTOR 
//#warning "INFANTRY_2023_MECANUM, INFANTRY_2024_MECANUM and HERO_2025_MECANUM are using mecanum wheels and 3508 motors, please make sure the chassis is built accordingly and the firmware is configured correctly"
#elif (ROBOT_TYPE == SENTRY_2026_OMNI)
#define WHEEL_TYPE ROBOT_CHASSIS_USE_OMNI 
#define MOTOR_TYPE POWER_TRAIN_USE_4010_MOTOR 
//#warning "SENTRY_2026_OMNI is using omni wheels and 4010 motors, please make sure the chassis is built accordingly and the firmware is configured correctly"
#elif (ROBOT_TYPE == INFANTRY_2026_STANDARD)
#define WHEEL_TYPE ROBOT_CHASSIS_USE_MECANUM 
#define MOTOR_TYPE POWER_TRAIN_USE_4010_MOTOR 
//#warning "INFANTRY_2026_STANDARD is using standard wheels, please make sure the chassis is built accordingly and the firmware is configured correctly"
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define MOTOR_TYPE POWER_TRAIN_USE_3508_MOTOR
#define WHEEL_TYPE ROBOT_CHASSIS_USE_SWERVE
//#warning "INFANTRY_2023_SWERVE is using swerve wheels, please make sure the chassis is built accordingly and the firmware is configured correctly"
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
#define WHEEL_TYPE ROBOT_CHASSIS_USE_BIPED
#define MOTOR_TYPE POWER_TRAIN_USE_SPECIAL_CTRL
//#warning "INFANTRY_2024_BIPED is using biped wheels, please make sure the chassis is built accordingly and the firmware is configured correctly"
#else
#error "No chassis type defined for the selected ROBOT_TYPE"
#endif

#if DEBUG_CV_WITH_USB && !CV_INTERFACE
#error "DEBUG_CV_WITH_USB is only for CV_INTERFACE"
#endif

#if CV_INTERFACE && (VOFA_UART_USE == 2)
//#error "VOFA_UART_USE=2 conflicts with CV_INTERFACE using USART1"
#endif

#if ((ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI)) && !CV_INTERFACE
#error "SENTRY_2023_MECANUM and SENTRY_2026_OMNI should use CV_INTERFACE for competition"
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
