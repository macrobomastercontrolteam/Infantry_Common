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
#define INFANTRY_2024_MECANUM_NEO 8
#define INFANTRY_2026_OMNI 9

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
#define ROBOT_TYPE INFANTRY_2024_MECANUM_NEO
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

/* ============================ Per-robot hardware capability map ============================
 * Each ROBOT_TYPE selects exactly one block below. Every hardware-capability macro for a robot
 * lives in a single place, so a robot can be configured or reviewed without scanning the file.
 * To add a robot: declare its ROBOT_TYPE macro above, then add a matching block here.
 *
 *   CHASSIS_POWER_CONTROL        - firmware runs the active chassis power/energy loop
 *   CAN_PASS_REF_INFO            - referee data is forwarded over CAN (vs. read from local UART)
 *   ROBOT_YAW_IS_4310            - yaw actuator is a DaMiao 4310
 *   ROBOT_PITCH_IS_3507/4310/4340- pitch actuator model (all 0 = legacy GM6020-style pitch)
 *   WHEEL_TYPE / MOTOR_TYPE      - chassis kinematics and drive-motor family
 *
 * Capability matrix (see individual blocks for the authoritative values):
 *   TYPE                       PWR CAN YAW4310 PITCH   WHEEL    MOTOR
 *   INFANTRY_2023_MECANUM       0   0    0     -       MECANUM  3508
 *   INFANTRY_2024_MECANUM       1   1    1     -       MECANUM  3508
 *   INFANTRY_2023_SWERVE        0   0    0     -       SWERVE   3508
 *   INFANTRY_2024_BIPED         0   0    1     -       BIPED    SPECIAL
 *   SENTRY_2023_MECANUM         0   0    0     -       MECANUM  3508
 *   HERO_2025_MECANUM           1   1    1     4340    MECANUM  3508
 *   SENTRY_2026_OMNI            1   1    1     3507    OMNI     4010
 *   INFANTRY_2026_MECANUM       1   0    1     4310    MECANUM  3508
 *   INFANTRY_2024_MECANUM_NEO   1   1    1     3507    MECANUM  4010
 *   INFANTRY_2026_OMNI          1   1    1     3507    OMNI     4010
 * ==========================================================================================*/

/* Every supported robot type carries a yaw slip ring today. */
#define ROBOT_YAW_HAS_SLIP_RING 1

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
    #define CHASSIS_POWER_CONTROL 0
    #define CAN_PASS_REF_INFO     0
    #define ROBOT_YAW_IS_4310     0
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_MECANUM
    #define MOTOR_TYPE            POWER_TRAIN_USE_3508_MOTOR

#elif (ROBOT_TYPE == INFANTRY_2024_MECANUM)
    #define CHASSIS_POWER_CONTROL 1
    #define CAN_PASS_REF_INFO     1
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_MECANUM
    #define MOTOR_TYPE            POWER_TRAIN_USE_3508_MOTOR

#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
    #define CHASSIS_POWER_CONTROL 0
    #define CAN_PASS_REF_INFO     0
    #define ROBOT_YAW_IS_4310     0
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_SWERVE
    #define MOTOR_TYPE            POWER_TRAIN_USE_3508_MOTOR

#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
    #define CHASSIS_POWER_CONTROL 0
    #define CAN_PASS_REF_INFO     0
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_BIPED
    #define MOTOR_TYPE            POWER_TRAIN_USE_SPECIAL_CTRL

#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
    #define CHASSIS_POWER_CONTROL 0
    #define CAN_PASS_REF_INFO     0
    #define ROBOT_YAW_IS_4310     0
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_MECANUM
    #define MOTOR_TYPE            POWER_TRAIN_USE_3508_MOTOR

#elif (ROBOT_TYPE == HERO_2025_MECANUM)
    #define CHASSIS_POWER_CONTROL 1
    #define CAN_PASS_REF_INFO     1
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   1
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_MECANUM
    #define MOTOR_TYPE            POWER_TRAIN_USE_3508_MOTOR

#elif (ROBOT_TYPE == SENTRY_2026_OMNI)
    #define CHASSIS_POWER_CONTROL 1
    #define CAN_PASS_REF_INFO     1
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   1
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_OMNI
    #define MOTOR_TYPE            POWER_TRAIN_USE_4010_MOTOR

#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    #define CHASSIS_POWER_CONTROL 0
    #define CAN_PASS_REF_INFO     1
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   0
    #define ROBOT_PITCH_IS_4310   1
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_MECANUM
    #define MOTOR_TYPE            POWER_TRAIN_USE_3508_MOTOR

#elif (ROBOT_TYPE == INFANTRY_2024_MECANUM_NEO)
    #define CHASSIS_POWER_CONTROL 1
    #define CAN_PASS_REF_INFO     1
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   1
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_MECANUM
    #define MOTOR_TYPE            POWER_TRAIN_USE_4010_MOTOR

#elif (ROBOT_TYPE == INFANTRY_2026_OMNI)
    #define CHASSIS_POWER_CONTROL 1
    #define CAN_PASS_REF_INFO     1
    #define ROBOT_YAW_IS_4310     1
    #define ROBOT_PITCH_IS_3507   1
    #define ROBOT_PITCH_IS_4310   0
    #define ROBOT_PITCH_IS_4340   0
    #define WHEEL_TYPE            ROBOT_CHASSIS_USE_OMNI
    #define MOTOR_TYPE            POWER_TRAIN_USE_4010_MOTOR

#else
    #error "No hardware configuration defined for the selected ROBOT_TYPE"
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

/* Use the toolchain-provided fixed-width integer types. Defining them by hand
 * conflicts with newlib's <stdint.h> under arm-none-eabi-gcc, where int32_t is
 * "long int" rather than "int". <stdint.h> provides matching definitions for
 * both the ARM Compiler and GCC. */
#include <stdint.h>

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif /* _GLOBAL_INC_H */
