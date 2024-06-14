/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef _SHOOT_H
#define _SHOOT_H
#include "global_inc.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "chassis_task.h"

#define SHOOT_CONTROL_TIME_MS GIMBAL_CONTROL_TIME_MS
#define SHOOT_CONTROL_TIME_S GIMBAL_CONTROL_TIME_S

// After the shooting is enabled, the bullet is continuously fired for a period of time, used to clear the bullet
#define RC_S_LONG_TIME              250

// TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO: gear ratio between trigger motor and trigger wheel
// TRIGGER_WHEEL_CAPACITY: ammo per revolution of trigger wheel
#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO  1.0f
#define TRIGGER_WHEEL_CAPACITY 8.0f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == INFANTRY_2024_MECANUM)
#define TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO  (58.0f / 24.0f)
#define TRIGGER_WHEEL_CAPACITY  12.0f
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO  1.0f
#define TRIGGER_WHEEL_CAPACITY  9.0f
#endif

#define TRIGGER_MOTOR_GEAR_RATIO  36.0f
#define TRIGGER_MOTOR_RPM_TO_SPEED  (2.0f * PI / 60.0f / TRIGGER_MOTOR_GEAR_RATIO)
#define TRIGGER_MOTOR_ECD_TO_ANGLE  (2.0f * PI / (float)ECD_RANGE / TRIGGER_MOTOR_GEAR_RATIO / TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO)
#define TRIGGER_MOTOR_ANGLE_THRESHOLD 0.05f
#define TRIGGER_MULTILOOP_FULL_COUNT 18

#define FRICTION_MOTOR_RADIUS 0.03f
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define SPEED_COMPENSATION_RATIO 1.22f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define SPEED_COMPENSATION_RATIO 1.22f
#elif (ROBOT_TYPE == INFANTRY_2024_MECANUM)
#define SPEED_COMPENSATION_RATIO 1.22f
#else
#warning "SPEED_COMPENSATION_RATIO not defined for this robot type, using default value 1.0. If you're sure about this, temporarily uncomment this line."
#define SPEED_COMPENSATION_RATIO 1.0f
#endif
#define FRICTION_MOTOR_RPM_TO_SPEED (2.0f * PI / 60.0f * (FRICTION_MOTOR_RADIUS * SPEED_COMPENSATION_RATIO))
#define FRICTION_MOTOR_SPEED_TO_RPM (1.0f / FRICTION_MOTOR_RPM_TO_SPEED)
#define FRICTION_MOTOR_SPEED_THRESHOLD 0.9f // 10% tolerance

// max speed of M3508 is 26.99m/s for one motor, 26.2m/s for one motor during test
#if ENABLE_SHOOT_REDUNDANT_SWITCH
#define FRICTION_MOTOR_SPEED  26.0f
#else
#define FRICTION_MOTOR_SPEED  1.0f
#endif

// Unit: ammo per minute
#define SEMI_AUTO_FIRE_RATE 1200.0f
#define AUTO_FIRE_RATE     1200.0f
#define READY_TRIGGER_RATE 400.0f

// Speed unit: rpm
// tested approx max spinning speed: 19 rad/s, corresponding to 1632 rpm
#define SEMI_AUTO_FIRE_TRIGGER_SPEED RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * SEMI_AUTO_FIRE_RATE / TRIGGER_WHEEL_CAPACITY)
#define AUTO_FIRE_TRIGGER_SPEED      RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * AUTO_FIRE_RATE / TRIGGER_WHEEL_CAPACITY)
#define READY_TRIGGER_SPEED          RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * READY_TRIGGER_RATE / TRIGGER_WHEEL_CAPACITY)

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

#define BLOCK_TRIGGER_SPEED         1.0f
#define IDLE_TRIGGER_SPEED          2.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define TRIGGER_ANGLE_INCREMENT     (2.0f * PI / TRIGGER_WHEEL_CAPACITY)

#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        100.0f
#define TRIGGER_ANGLE_PID_KD        0.002f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define REVERSE_TRIGGER_DIRECTION 1
#else
#define REVERSE_TRIGGER_DIRECTION 0
#endif

#define SHOOT_HEAT_REMAIN_VALUE     50

//Frictional wheel 1 PID
#define FRICTION_1_SPEED_PID_KP        20.0f
#define FRICTION_1_SPEED_PID_KI        0.0f
#define FRICTION_1_SPEED_PID_KD        0.0f
#define FRICTION_1_SPEED_PID_MAX_OUT   MAX_MOTOR_CAN_CURRENT
#define FRICTION_1_SPEED_PID_MAX_IOUT  200.0f

//Frictional wheel 2 PID
#define FRICTION_2_SPEED_PID_KP        20.0f
#define FRICTION_2_SPEED_PID_KI        0.0f
#define FRICTION_2_SPEED_PID_KD        0.0f
#define FRICTION_2_SPEED_PID_MAX_OUT   MAX_MOTOR_CAN_CURRENT
#define FRICTION_2_SPEED_PID_MAX_IOUT  200.0f

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_TRIGGER,
    SHOOT_READY,
    SHOOT_SEMI_AUTO_FIRE,
    SHOOT_AUTO_FIRE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;

    int16_t fric1_given_current;
    int16_t fric2_given_current;

    pid_type_def friction_motor1_pid;
    fp32 friction_motor1_rpm_set;
    fp32 friction_motor1_rpm;
    // fp32 friction_motor1_angle;

    pid_type_def friction_motor2_pid;
    fp32 friction_motor2_rpm_set;
    fp32 friction_motor2_rpm;
    // fp32 friction_motor2_angle;

	pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;    
    fp32 speed; // unit: rad/s
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;
    
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t left_click_hold_time;

    uint16_t block_time;
    uint16_t reverse_time;

    bool_t key;
    // uint8_t key_time;

    // shooter of this controller itself
    uint16_t heat_limit;
    uint16_t heat;

    uint32_t cv_auto_shoot_start_time;

    // @TODO: automatically adjust SPEED_COMPENSATION_RATIO using ref feedbacks here
    uint8_t launching_frequency[2];
	fp32 bullet_init_speed[2];
} shoot_control_t;

// because the shooting and gimbal use the same can id, the shooting task is also executed in the gimbal task
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

extern shoot_control_t shoot_control;

#endif
