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

#ifndef SHOOT_H
#define SHOOT_H
#include "global_inc.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "chassis_task.h"



#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

// Start friction wheel immediately
#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define SHOOT_FRIC_PWM_ADD_VALUE    2000.0f
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define SHOOT_FRIC_PWM_ADD_VALUE    2000.0f
#else
#define SHOOT_FRIC_PWM_ADD_VALUE    2000.0f
#endif

//Start friction wheel PID


#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

// After the shooting is completed, the bullet is ejected, and the judgment time is to prevent mis-triggering
#define SHOOT_DONE_KEY_OFF_TIME     15
// Determine the long press of the mouse
#define PRESS_LONG_TIME             400
// After the remote control shooting switch is pressed down, the bullet is continuously fired for a period of time, used to clear the bullet
#define RC_S_LONG_TIME              2000
// Friction wheel high speed acceleration time
#define UP_ADD_TIME                 80

#define TRIGGER_MOTOR_RPM_TO_SPEED  0.00290888208665721596153948461415f
#define TRIGGER_MOTOR_ECD_TO_ANGLE  0.000021305288720633905968306772076277f
#define FULL_COUNT                  18

#define FRICTION_MOTOR_RPM_TO_SPEED  (2.0f*PI/60.0f*0.03f)
#define FRICTION_MOTOR_SPEED_TO_RPM  (1.0f/FRICTION_MOTOR_RPM_TO_SPEED)
// max speed of M3508 is 26.99m/s for one motor, 26.2m/s for one motor during test
#define FRICTION_MOTOR_SPEED  26.0f


#define TRIGGER_SPEED               10.0f
#define CONTINUE_TRIGGER_SPEED      15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#if (ROBOT_TYPE == INFANTRY_2018_MECANUM) 
#define TRIGGER_ANGLE_INCREMENT     (PI/7.0f)
#elif (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define TRIGGER_ANGLE_INCREMENT     (PI/8.0f)
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define TRIGGER_ANGLE_INCREMENT     (PI/12.0f)
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define TRIGGER_ANGLE_INCREMENT     (PI/9.0f)
#endif

#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define TRIGGER_TURN
#endif

#define SHOOT_HEAT_REMAIN_VALUE     80

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM) 
//Frictional wheel 1 PID
#define FRICTION_1_SPEED_PID_KP        1000.0f
#define FRICTION_1_SPEED_PID_KI        10.0f
#define FRICTION_1_SPEED_PID_KD        0.0f
#define FRICTION_1_SPEED_PID_MAX_OUT   MAX_MOTOR_CAN_CURRENT
#define FRICTION_1_SPEED_PID_MAX_IOUT  200.0f

//Frictional wheel 2 PID
#define FRICTION_2_SPEED_PID_KP        100.0f
#define FRICTION_2_SPEED_PID_KI        5.0f
#define FRICTION_2_SPEED_PID_KD        0.0f
#define FRICTION_2_SPEED_PID_MAX_OUT   MAX_MOTOR_CAN_CURRENT
#define FRICTION_2_SPEED_PID_MAX_IOUT  200.0f

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY_BULLET,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    uint8_t fIsCvControl;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    const motor_measure_t *fric_1_motor_measure;
    const motor_measure_t *fric_2_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;
    pid_type_def friction_motor1_pid;
    fp32 friction_motor1_rpm_set;
    fp32 friction_motor1_rpm;
    //fp32 friction_motor1_angle;
    
    pid_type_def friction_motor2_pid;
    fp32 friction_motor2_rpm_set;
    fp32 friction_motor2_rpm;
    //fp32 friction_motor2_angle;
    
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
    int16_t fric1_given_current;
    int16_t fric2_given_current;
} shoot_control_t;

// because the shooting and gimbal use the same can id, the shooting task is also executed in the gimbal task
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
extern shoot_control_t shoot_control;
#endif

#endif
#endif
