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
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_BIPED)
#define TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO  (57.0f / 24.0f)
#define TRIGGER_WHEEL_CAPACITY  12.0f
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO  1.0f
#define TRIGGER_WHEEL_CAPACITY  9.0f
#elif(ROBOT_TYPE == HERO_2025_MECANUM)
#define TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO 1.0f
#define TRIGGER_WHEEL_CAPACITY 6.0f
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
#define SPEED_COMPENSATION_RATIO 1.12f
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

//***************2025_Hero *******************/
#if (ROBOT_TYPE == HERO_2025_MECANUM)

#if ENABLE_SHOOT_REDUNDANT_SWITCH
#define HERO_FRICTION_MOTOR_SPEED  15.0f
#else
#define HERO_FRICTION_MOTOR_SPEED  1.0f
#endif

#define HERO_FRICTON_MOTOR_INIT_SPEED 3.0f //m/s

#define READY_TRIGGER_RATE 180.0f //ammo / minute
#define JAM_RESOVE_TRIGGER_RATE 240.0f

//piston
#define PISTON_MOTOR_SPEED 0.6f // Piston motor speed in m/s
#define PISTON_MOTOR_RPM_TO_SPEED (2.0f * PI / 60.0f * 0.005f)
#define PISTON_MOTOR_SPEED_TO_RPM (1.0f / PISTON_MOTOR_RPM_TO_SPEED)

#define BLOCK_PISTON_SPEED         0.1f
#define IDLE_PISTON_SPEED          2.0f
#define PISTON_BLOCK_TIME           200

#define PISTON_FORWARD 1
#define PISTON_BACKWARD -1

#define HORI_FRICTION_MOTOR_RADIUS 0.036f
#define HORI_FRICTION_MOTOR_RPM_TO_SPEED (2.0f * PI / 60.0f * (HORI_FRICTION_MOTOR_RADIUS * SPEED_COMPENSATION_RATIO))
#define HORI_FRICTION_MOTOR_SPEED_TO_RPM (1.0f / HORI_FRICTION_MOTOR_RPM_TO_SPEED)
#define HORI_FRICTION_MOTOR_SPEED_THRESHOLD 0.9f // 10% tolerance

#define VERT_FRICTION_MOTOR_RADIUS 0.03f
#define VERT_FRICTION_MOTOR_RPM_TO_SPEED (2.0f * PI / 60.0f * (VERT_FRICTION_MOTOR_RADIUS * SPEED_COMPENSATION_RATIO))
#define VERT_FRICTION_MOTOR_SPEED_TO_RPM (1.0f / VERT_FRICTION_MOTOR_RPM_TO_SPEED)
#define VERT_FRICTION_MOTOR_SPEED_THRESHOLD 0.9f // 10% tolerance

#endif
//************* */

// Unit: ammo per minute
#define SEMI_AUTO_FIRE_RATE 1800.0f
#define AUTO_FIRE_RATE     1800.0f


// Speed unit: rpm
// tested approx max spinning speed: 19 rad/s, corresponding to 1632 rpm
#define SEMI_AUTO_FIRE_TRIGGER_SPEED RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * SEMI_AUTO_FIRE_RATE / TRIGGER_WHEEL_CAPACITY)
#define AUTO_FIRE_TRIGGER_SPEED      RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * AUTO_FIRE_RATE / TRIGGER_WHEEL_CAPACITY)

#define READY_TRIGGER_SPEED          RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * READY_TRIGGER_RATE / TRIGGER_WHEEL_CAPACITY)
#define JAM_RESOVE_TRIGGER_SPEED     RPM_TO_RADS(TRIGGER_MOTOR_TO_WHEEL_GEAR_RATIO * JAM_RESOVE_TRIGGER_RATE / TRIGGER_WHEEL_CAPACITY)

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1


#if (ROBOT_TYPE == HERO_2025_MECANUM)
#define TRIGGER_BLOCK_TIME          200
#define TRIGGER_REVERSE_TIME        0
#define REVERSE_SPEED_LIMIT         0
#else
#define TRIGGER_BLOCK_TIME          100
#define TRIGGER_REVERSE_TIME        150
#define REVERSE_SPEED_LIMIT         13.0f
#endif


#define TRIGGER_ANGLE_INCREMENT     (2.0f * PI / TRIGGER_WHEEL_CAPACITY)

#if (ROBOT_TYPE == HERO_2025_MECANUM)
#define TRIGGER_ANGLE_PID_KP        2000.0f
#define TRIGGER_ANGLE_PID_KI        0.75f
#define TRIGGER_ANGLE_PID_KD        0.12f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define BLOCK_TRIGGER_SPEED         0.2f //READY_TRIGGER_SPEED * 0.5f
#define IDLE_TRIGGER_SPEED          2.0f
#else
#define TRIGGER_ANGLE_PID_KP        900.0f
#define TRIGGER_ANGLE_PID_KI        100.0f
#define TRIGGER_ANGLE_PID_KD        0.002f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define BLOCK_TRIGGER_SPEED         0.5f
#define IDLE_TRIGGER_SPEED          2.0f
#endif

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define REVERSE_TRIGGER_DIRECTION 1
#else
#define REVERSE_TRIGGER_DIRECTION 0
#endif

#define SHOOT_HEAT_LIMIT_CLEARANCE     60

//Frictional wheel 1 PID (LEFT)
#define FRICTION_1_SPEED_PID_KP        20.0f
#define FRICTION_1_SPEED_PID_KI        0.0f
#define FRICTION_1_SPEED_PID_KD        0.0f
#define FRICTION_1_SPEED_PID_MAX_OUT   MAX_3508_MOTOR_CAN_CURRENT
#define FRICTION_1_SPEED_PID_MAX_IOUT  200.0f

//Frictional wheel 2 PID (RIGHT)
#define FRICTION_2_SPEED_PID_KP        20.0f
#define FRICTION_2_SPEED_PID_KI        0.0f
#define FRICTION_2_SPEED_PID_KD        0.0f
#define FRICTION_2_SPEED_PID_MAX_OUT   MAX_3508_MOTOR_CAN_CURRENT
#define FRICTION_2_SPEED_PID_MAX_IOUT  200.0f

#if (ROBOT_TYPE == HERO_2025_MECANUM)
//DUMMY VALUES
//Frictional wheel 3 PID (UP)
#define FRICTION_3_SPEED_PID_KP        55.0f
#define FRICTION_3_SPEED_PID_KI        0.0f
#define FRICTION_3_SPEED_PID_KD        0.0f
#define FRICTION_3_SPEED_PID_MAX_OUT   MAX_3508_MOTOR_CAN_CURRENT
#define FRICTION_3_SPEED_PID_MAX_IOUT  500.0f

//Frictional wheel 4 PID (DOWN)
#define FRICTION_4_SPEED_PID_KP        55.0f
#define FRICTION_4_SPEED_PID_KI        0.0f
#define FRICTION_4_SPEED_PID_KD        0.0f
#define FRICTION_4_SPEED_PID_MAX_OUT   MAX_3508_MOTOR_CAN_CURRENT
#define FRICTION_4_SPEED_PID_MAX_IOUT  500.0f

//Piston Motor PID
#define PISTON_SPEED_PID_KP            13500.0f
#define PISTON_SPEED_PID_KI            500.0f
#define PISTON_SPEED_PID_KD            1.5f
#define PISTON_SPEED_PID_MAX_OUT       MAX_3508_MOTOR_CAN_CURRENT
#define PISTON_SPEED_PID_MAX_IOUT      1000.0f

#endif

typedef enum
{
    SHOOT_STOP = 0, // Stop all shooting activity.
    SHOOT_READY_FRIC, // Start spinning friction wheels. Actual shooting mode will be set corespondingly inside this mode.
    SHOOT_READY_TRIGGER, //spain trigger motor to load the ammo chain
    SHOOT_READY,
    SHOOT_SEMI_AUTO_FIRE, 
    SHOOT_AUTO_FIRE,

    HERO_INIT_LAUNCHER, //clear and close the launcher
    HERO_LAUNCHER_READY,
    HERO_LAUNCHER_SHOOT,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    shoot_mode_e previous_shoot_mode;

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

    int16_t fric3_given_current;
    int16_t fric4_given_current;
    
    pid_type_def friction_motor3_pid;
    fp32 friction_motor3_rpm_set;
    fp32 friction_motor3_rpm;

    pid_type_def friction_motor4_pid;
    fp32 friction_motor4_rpm_set;
    fp32 friction_motor4_rpm;

    pid_type_def piston_motor_pid; //PID variable for piston motor
    //fp32 piston_speed_target; //Target speed in the state machine
    fp32 piston_speed_set; //Speed defined by the piston motor contro function
    fp32 piston_speed; //Speed feedback from the motor ESC
    //int8_t piston_direction; //Direction of the piston motor, 1 for forward, -1 for reverse
    uint16_t piston_given_current;

	pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;    
    fp32 speed; // unit: rad/s
    fp32 speed_set;
    fp32 trigger_angle;
    fp32 set_angle;
    int16_t cmd_value;
    int8_t trigger_ecd_count;
    
    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t left_click_hold_time;

    uint16_t block_time;
    uint16_t piston_block_time;
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

//flags for launcher status
typedef struct
{
    //=1 for loaded/opened
    uint8_t Launcher_Initialized;
    uint8_t Chain_Loaded;
    uint8_t Launcher_Loaded;
    uint8_t Launcher_Opened;
    uint8_t Launcher_Jamed;

    uint8_t piston_moving;
    uint8_t piston_feedback;

    uint8_t trigger_moving;

    uint8_t init_step;
    
}launcher_status_t;
// because the shooting and gimbal use the same can id, the shooting task is also executed in the gimbal task
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);

extern shoot_control_t shoot_control;
extern launcher_status_t launcher_status;
#endif
