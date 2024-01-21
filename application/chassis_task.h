/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357.0f
#define BIPED_CHASSIS_TASK_INIT_TIME 1000.0f

//chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 2.5f
#define NORMAL_MAX_CHASSIS_SPEED_YAW 2.5f
#define MAX_CHASSIS_ROLL 0.4f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//摇杆死区
#define CHASSIS_RC_DEADLINE 10.0f

#define CHASSIS_JSCOPE_DEBUG 1

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 11.0f
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.01f
//chassis control frequence, no use now.
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 100.0f
//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//chassis 6020 max motor control voltage
//底盘6020最大can发送电压值
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_DIS_RC_SPEED 1.0f // 1m/s
#define CHASSIS_DIS_KEYBOARD_INC (CHASSIS_DIS_RC_SPEED/1000.0f*CHASSIS_CONTROL_TIME_MS)
#define CHASSIS_DIS_RC_SEN_INC (CHASSIS_DIS_RC_SPEED/1000.0f*CHASSIS_CONTROL_TIME_MS/JOYSTICK_HALF_RANGE)

#define CHASSIS_YAW_RC_SEN_INC (NORMAL_MAX_CHASSIS_SPEED_YAW/1000.0f*CHASSIS_CONTROL_TIME_MS/JOYSTICK_HALF_RANGE)

// rise speed is L0 range per CHASSIS_CONTROL_TIME_MS per joystick_range
#define LEG_L0_RC_RISE_TIME_MS 1000.0f
#define LEG_L0_KEYBOARD_INC (LEG_L0_RANGE/LEG_L0_RC_RISE_TIME_MS*CHASSIS_CONTROL_TIME_MS)
#define LEG_L0_RC_SEN_INC (LEG_L0_RANGE/LEG_L0_RC_RISE_TIME_MS*CHASSIS_CONTROL_TIME_MS/JOYSTICK_FULL_RANGE)

#define CHASSIS_ROLL_RC_CHANGE_TIME_MS 2000.0f
#define CHASSIS_ROLL_KEYBOARD_INC (MAX_CHASSIS_ROLL/CHASSIS_ROLL_RC_CHANGE_TIME_MS*CHASSIS_CONTROL_TIME_MS)
#define CHASSIS_ROLL_RC_SEN_INC (MAX_CHASSIS_ROLL/CHASSIS_ROLL_RC_CHANGE_TIME_MS*CHASSIS_CONTROL_TIME_MS/JOYSTICK_FULL_RANGE)

//press the key, chassis will swing
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_RISE_PLATFORM_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_LOWER_PLATFORM_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEAN_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_LEAN_RIGHT_KEY KEY_PRESSED_OFFSET_D

//single chassis motor max speed
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f

// Arbitrary offsets between chassis rotational center and centroid
#if defined(INFANTRY_1) || defined(INFANTRY_2) || defined(INFANTRY_3) || defined(SENTRY_1)
// slip ring is at the center of chassis
#define CHASSIS_WZ_SET_SCALE 0.0f
#else
// Offset for the official model
#define CHASSIS_WZ_SET_SCALE 0.1f
#endif

//when chassis is not set to move, swing max angle
//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

// //chassis motor speed PID
// //底盘电机速度环PID
// #define M3508_MOTOR_SPEED_PID_KP 15000.0f
// #define M3508_MOTOR_SPEED_PID_KI 10.0f
// #define M3508_MOTOR_SPEED_PID_KD 0.0f
// #define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
// #define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

// //chassis follow angle PID
// //底盘旋转跟随PID
// #define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.1f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
// #define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. 底盘有旋转速度控制
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.
  CHASSIS_VECTOR_SPINNING,            //spinning chassis
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  fp32 give_torque;
} chassis_motor_t;

#if defined(INFANTRY_3)
typedef struct
{
  uint16_t target_ecd; ///< unit encoder unit; range is [0, 8191]; positive direction is clockwise; forward direction of chassis is 0 ecd
} chassis_steer_motor_t;
#endif

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
  // const gimbal_motor_t *chassis_yaw_motor;   //will use the relative angle of yaw gimbal motor to calculate the euler angle.底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
  // const gimbal_motor_t *chassis_pitch_motor; //will use the relative angle of pitch gimbal motor to calculate the euler angle.底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
  const fp32 *chassis_INS_speed;             //the point to the euler angular speed of gyro sensor
  // const fp32 *chassis_INS_accel;
  chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
  // pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
  // fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_relative_angle;      //the relative angle between chassis and gimbal.底盘与云台的相对角度，单位 rad
  fp32 chassis_relative_angle_set;  //the set relative angle.设置相对云台控制角度

  // fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  // fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  // fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  // fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s

} chassis_move_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
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
/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector);

fp32 abs_err_handler(fp32 set, fp32 ref);

extern chassis_move_t chassis_move;

#endif
