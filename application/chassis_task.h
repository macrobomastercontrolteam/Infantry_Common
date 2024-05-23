/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. finished
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "global_inc.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_WZ_NUM 0.06f
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define CHASSIS_ACCEL_X_NUM 0.06f
#define CHASSIS_ACCEL_Y_NUM 0.06f
#else
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#endif

//joystick value deadline
#define CHASSIS_RC_DEADLINE 10

#define CHASSIS_TEST_MODE 0

#if (ROBOT_TYPE == INFANTRY_2018_MECANUM) || (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
// avoid changing angle too often near zero speed
#define STEER_TURN_X_SPEED_DEADZONE 0.01f
#define STEER_TURN_Y_SPEED_DEADZONE (STEER_TURN_X_SPEED_DEADZONE * NORMAL_MAX_CHASSIS_SPEED_Y / NORMAL_MAX_CHASSIS_SPEED_X)
#define STEER_TURN_W_SPEED_DEADZONE 0.01f
#endif

#if (ROBOT_TYPE == INFANTRY_2018_MECANUM)
#define MOTOR_DISTANCE_TO_CENTER 0.2f
#elif (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define MOTOR_DISTANCE_TO_CENTER 0.2788f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define MOTOR_DISTANCE_TO_CENTER 0.28284271247461906f // sqrt(pow(CHASSIS_Y_DIRECTION_HALF_LENGTH,2)+pow(CHASSIS_X_DIRECTION_HALF_LENGTH,2))
#define CHASSIS_Y_DIRECTION_HALF_LENGTH 0.2f
#define CHASSIS_X_DIRECTION_HALF_LENGTH 0.2f
#define CHASSIS_ANGLE_COS 0.7071067811865475f // (CHASSIS_X_DIRECTION_HALF_LENGTH/MOTOR_DISTANCE_TO_CENTER)
#define CHASSIS_ANGLE_SIN 0.7071067811865475f // (CHASSIS_Y_DIRECTION_HALF_LENGTH/MOTOR_DISTANCE_TO_CENTER)
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define MOTOR_DISTANCE_TO_CENTER 0.2f // @TODO: update this
#endif

#define CHASSIS_CONTROL_TIME_MS 2.0f
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)
#define CHASSIS_CONTROL_FREQUENCE (1.0f / CHASSIS_CONTROL_TIME_S)

//chassis 3508 max motor control current
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//chassis 6020 max motor control voltage
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f
//press the key, chassis will swing
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

// drive wheel parameters
#define M3508_MOTOR_GEAR_RATIO (3591.0f / 187.0f)
#if (ROBOT_TYPE == INFANTRY_2018_MECANUM) || (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define DRIVE_WHEEL_RADIUS 0.07625f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define DRIVE_WHEEL_RADIUS 0.055f
#endif
// Ratio of M3508 speed in rpm to chassis speed in m/s
#define M3508_MOTOR_RPM_TO_VECTOR ((2.0f * PI / 60.0f) * DRIVE_WHEEL_RADIUS / M3508_MOTOR_GEAR_RATIO)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
#define SPRINT_MAX_CHASSIS_SPEED_X 5.0f
//chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f
#define SPRINT_MAX_CHASSIS_SPEED_Y 5.0f
#define NORMAL_MAX_CHASSIS_SPEED_WZ RPM_TO_RADS(60.0f)

// In follow-yaw mode, map joystick value to increment in target yaw angle
#define CHASSIS_ANGLE_Z_RC_CHANGE_TIME_S 2.0f
#define CHASSIS_ANGLE_Z_RC_SEN_INC (PI / 2.0f / CHASSIS_ANGLE_Z_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S / JOYSTICK_HALF_RANGE)
// In not-follow-yaw mode, map joystick value to target yaw speed
#define CHASSIS_WZ_RC_SEN (NORMAL_MAX_CHASSIS_SPEED_WZ / JOYSTICK_HALF_RANGE)
// map rc dial value (max 660) to spinning speed (rad/s)
#define CHASSIS_SPIN_RC_SEN_POSITIVE_INPUT ((NORMAL_MAX_CHASSIS_SPEED_WZ - SPINNING_CHASSIS_LOW_OMEGA) / JOYSTICK_HALF_RANGE)
#define CHASSIS_SPIN_RC_SEN_NEGATIVE_INPUT ((NORMAL_MAX_CHASSIS_SPEED_WZ + SPINNING_CHASSIS_LOW_OMEGA) / JOYSTICK_HALF_RANGE)
#define CHASSIS_SPIN_RC_OFFSET SPINNING_CHASSIS_LOW_OMEGA

// Arbitrary offsets between chassis rotational center and centroid
#if ROBOT_YAW_HAS_SLIP_RING
// slip ring is at the center of chassis
#define CHASSIS_WZ_SET_SCALE 0.0f
#else
// Offset for the official model
#define CHASSIS_WZ_SET_SCALE 0.1f
#endif

//when chassis is not set to move, swing max angle
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 5000.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0002f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   //chassis will follow yaw gimbal motor relative angle (this mode is not stable nor useful, but it may enlighten you on how to make new modes)
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  //chassis will have yaw angle(chassis_yaw) close-looped control
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.
  CHASSIS_VECTOR_SPINNING,            //spinning chassis

} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef union
{
  uint8_t can_buf[8];
  struct
  {
    uint8_t cap_state;
    uint8_t reserve;
    uint16_t cap_voltage;
    float cap_power;
  } cap_message;
} supcap_t;

extern supcap_t cap_message_rx;

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
typedef struct
{
  uint16_t target_ecd; ///< unit encoder unit; range is [0, 8191]; positive direction is clockwise; forward direction of chassis is 0 ecd
} chassis_steer_motor_t;
#endif

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //the point to remote control
  const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle
  const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor
  chassis_mode_e chassis_mode;               //state machine
  chassis_mode_e last_chassis_mode;          //last state machine
  chassis_motor_t motor_chassis[4];          //chassis motor data
  pid_type_def motor_speed_pid[4];             //motor speed PID
  pid_type_def chassis_angle_pid;              //follow angle PID
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
  chassis_steer_motor_t steer_motor_chassis[4];//chassis steering motor data
  pid_type_def steer_motor_angle_pid[4];       //steering motor angle PID
#endif

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point
  first_order_filter_type_t chassis_cmd_slow_set_wz;  //use first order filter to slow set-point

#if !(ROBOT_TYPE == INFANTRY_2023_SWERVE)
  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s
#endif
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s
  fp32 chassis_relative_angle_set;  //the set relative angle
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s
  fp32 vx_min_speed;  //max backward speed, unit m/s
  fp32 vy_max_speed;  //max letf speed, unit m/s
  fp32 vy_min_speed;  //max right speed, unit m/s
  fp32 vx_rc_sen;     //map joystick value to vertical speed
  fp32 vy_rc_sen;     //map joystick value to horizontal speed
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
extern void chassis_task(void const *pvParameters);

/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     vy_set: horizontal speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_rc_to_swerve_control_vector(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

extern chassis_move_t chassis_move;

#endif
