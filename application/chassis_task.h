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
#include "global_inc.h"
#include "CAN_receive.h"

#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

#define JOINT_0_RC_DIRECTION_GAIN (1.0f)
#define JOINT_1_RC_DIRECTION_GAIN (-1.0f)
#define JOINT_2_RC_DIRECTION_GAIN (1.0f)
#define JOINT_3_RC_DIRECTION_GAIN (1.0f)
#define JOINT_4_RC_DIRECTION_GAIN (1.0f)
#define JOINT_5_RC_DIRECTION_GAIN (-1.0f)
#define JOINT_6_RC_DIRECTION_GAIN (1.0f)

#define END_EFFECTOR_ROLL_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_PITCH_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_YAW_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_X_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_Y_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_Z_RC_DIRECTION_GAIN (1.0f)

#define ARM_JOINT_CLEARANCE 0.08f

#define ARM_JOINT_0_ANGLE_MIN (-PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_0_ANGLE_MAX (PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_0_ANGLE_RANGE (ARM_JOINT_0_ANGLE_MAX - ARM_JOINT_0_ANGLE_MIN)
#define ARM_JOINT_0_ANGLE_HOME 0.0f
#define ARM_JOINT_0_RC_SEN (JOINT_0_RC_DIRECTION_GAIN * ARM_JOINT_0_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_0_ANGLE_RC_CHANGE_TIME_S 3.0f
#define ARM_JOINT_0_KEYBOARD_INC (JOINT_0_RC_DIRECTION_GAIN * ARM_JOINT_0_ANGLE_RANGE / ARM_JOINT_0_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_0_RC_SEN_INC (ARM_JOINT_0_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_JOINT_1_ANGLE_MIN (-30.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_1_ANGLE_MAX (40.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_1_ANGLE_RANGE (ARM_JOINT_1_ANGLE_MAX - ARM_JOINT_1_ANGLE_MIN)
#define ARM_JOINT_1_ANGLE_HOME ARM_JOINT_1_ANGLE_MAX
#define ARM_JOINT_1_RC_SEN (JOINT_1_RC_DIRECTION_GAIN * ARM_JOINT_1_ANGLE_RANGE / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_1_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_1_KEYBOARD_INC (JOINT_1_RC_DIRECTION_GAIN * ARM_JOINT_1_ANGLE_RANGE / ARM_JOINT_1_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_1_RC_SEN_INC (ARM_JOINT_1_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_JOINT_2_ANGLE_MIN (-155.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_2_ANGLE_MAX (0.0f - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_2_ANGLE_RANGE (ARM_JOINT_2_ANGLE_MAX - ARM_JOINT_2_ANGLE_MIN)
#define ARM_JOINT_2_ANGLE_HOME ARM_JOINT_2_ANGLE_MIN
#define ARM_JOINT_2_RC_SEN (JOINT_2_RC_DIRECTION_GAIN * ARM_JOINT_2_ANGLE_RANGE / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_2_ANGLE_RC_CHANGE_TIME_S 1.5f
#define ARM_JOINT_2_KEYBOARD_INC (JOINT_2_RC_DIRECTION_GAIN * ARM_JOINT_2_ANGLE_RANGE / ARM_JOINT_2_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_2_RC_SEN_INC (ARM_JOINT_2_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_JOINT_3_ANGLE_MIN (-80.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_3_ANGLE_MAX (80.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_3_ANGLE_RANGE (ARM_JOINT_3_ANGLE_MAX - ARM_JOINT_3_ANGLE_MIN)
#define ARM_JOINT_3_ANGLE_HOME 0.0f
#define ARM_JOINT_3_RC_SEN (JOINT_3_RC_DIRECTION_GAIN * ARM_JOINT_3_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_3_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_3_KEYBOARD_INC (JOINT_3_RC_DIRECTION_GAIN * ARM_JOINT_3_ANGLE_RANGE / ARM_JOINT_3_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_3_RC_SEN_INC (ARM_JOINT_3_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_JOINT_4_ANGLE_MIN (-0.5f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_4_ANGLE_MAX (0.5f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_4_ANGLE_RANGE (ARM_JOINT_4_ANGLE_MAX - ARM_JOINT_4_ANGLE_MIN)
#define ARM_JOINT_4_ANGLE_HOME 0.0f
#define ARM_JOINT_4_RC_SEN (JOINT_4_RC_DIRECTION_GAIN * ARM_JOINT_4_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_4_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_4_KEYBOARD_INC (JOINT_4_RC_DIRECTION_GAIN * ARM_JOINT_4_ANGLE_RANGE / ARM_JOINT_4_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_4_RC_SEN_INC (ARM_JOINT_4_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_JOINT_5_ANGLE_MIN (10.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_5_ANGLE_MAX (170.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_5_ANGLE_RANGE (ARM_JOINT_5_ANGLE_MAX - ARM_JOINT_5_ANGLE_MIN)
#define ARM_JOINT_5_ANGLE_HOME ARM_JOINT_5_ANGLE_MAX
#define ARM_JOINT_5_RC_SEN (JOINT_5_RC_DIRECTION_GAIN * ARM_JOINT_5_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_5_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_5_KEYBOARD_INC (JOINT_5_RC_DIRECTION_GAIN * ARM_JOINT_5_ANGLE_RANGE / ARM_JOINT_5_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_5_RC_SEN_INC (ARM_JOINT_5_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_JOINT_6_ANGLE_MIN (-PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_6_ANGLE_MAX (PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_6_ANGLE_RANGE (ARM_JOINT_6_ANGLE_MAX - ARM_JOINT_6_ANGLE_MIN)
#define ARM_JOINT_6_ANGLE_HOME 0.0f
#define ARM_JOINT_6_RC_SEN (JOINT_6_RC_DIRECTION_GAIN * ARM_JOINT_6_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_6_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_6_KEYBOARD_INC (JOINT_6_RC_DIRECTION_GAIN * ARM_JOINT_6_ANGLE_RANGE / ARM_JOINT_6_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_6_RC_SEN_INC (ARM_JOINT_6_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_END_EFFECTOR_ROLL_MIN (-PI + ARM_JOINT_CLEARANCE)
#define ARM_END_EFFECTOR_ROLL_MAX (PI - ARM_JOINT_CLEARANCE)
#define ARM_END_EFFECTOR_ROLL_RANGE (ARM_END_EFFECTOR_ROLL_MAX - ARM_END_EFFECTOR_ROLL_MIN)
#define ARM_END_EFFECTOR_ROLL_HOME 0.0f
#define ARM_END_EFFECTOR_ROLL_RC_SEN (END_EFFECTOR_ROLL_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_ROLL_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_END_EFFECTOR_ROLL_RC_CHANGE_TIME_S 1.0f
#define ARM_END_EFFECTOR_ROLL_KEYBOARD_INC (END_EFFECTOR_ROLL_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_ROLL_RANGE / ARM_END_EFFECTOR_ROLL_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_END_EFFECTOR_ROLL_RC_SEN_INC (ARM_END_EFFECTOR_ROLL_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_END_EFFECTOR_PITCH_MIN (-PI + ARM_JOINT_CLEARANCE)
#define ARM_END_EFFECTOR_PITCH_MAX (PI - ARM_JOINT_CLEARANCE)
#define ARM_END_EFFECTOR_PITCH_RANGE (ARM_END_EFFECTOR_PITCH_MAX - ARM_END_EFFECTOR_PITCH_MIN)
#define ARM_END_EFFECTOR_PITCH_HOME 0.0f
#define ARM_END_EFFECTOR_PITCH_RC_SEN (END_EFFECTOR_PITCH_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_PITCH_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_END_EFFECTOR_PITCH_RC_CHANGE_TIME_S 1.0f
#define ARM_END_EFFECTOR_PITCH_KEYBOARD_INC (END_EFFECTOR_PITCH_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_PITCH_RANGE / ARM_END_EFFECTOR_PITCH_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_END_EFFECTOR_PITCH_RC_SEN_INC (ARM_END_EFFECTOR_PITCH_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_END_EFFECTOR_YAW_MIN (-PI + ARM_JOINT_CLEARANCE)
#define ARM_END_EFFECTOR_YAW_MAX (PI - ARM_JOINT_CLEARANCE)
#define ARM_END_EFFECTOR_YAW_RANGE (ARM_END_EFFECTOR_YAW_MAX - ARM_END_EFFECTOR_YAW_MIN)
#define ARM_END_EFFECTOR_YAW_HOME 0.0f
#define ARM_END_EFFECTOR_YAW_RC_SEN (END_EFFECTOR_YAW_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_YAW_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_END_EFFECTOR_YAW_RC_CHANGE_TIME_S 1.0f
#define ARM_END_EFFECTOR_YAW_KEYBOARD_INC (END_EFFECTOR_YAW_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_YAW_RANGE / ARM_END_EFFECTOR_YAW_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_END_EFFECTOR_YAW_RC_SEN_INC (ARM_END_EFFECTOR_YAW_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_END_EFFECTOR_X_MIN (-0.5f)
#define ARM_END_EFFECTOR_X_MAX (0.5f)
#define ARM_END_EFFECTOR_X_RANGE (ARM_END_EFFECTOR_X_MAX - ARM_END_EFFECTOR_X_MIN)
#define ARM_END_EFFECTOR_X_HOME 0.0f
#define ARM_END_EFFECTOR_X_RC_SEN (END_EFFECTOR_X_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_X_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_END_EFFECTOR_X_RC_CHANGE_TIME_S 1.0f
#define ARM_END_EFFECTOR_X_KEYBOARD_INC (END_EFFECTOR_X_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_X_RANGE / ARM_END_EFFECTOR_X_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_END_EFFECTOR_X_RC_SEN_INC (ARM_END_EFFECTOR_X_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_END_EFFECTOR_Y_MIN (-0.5f)
#define ARM_END_EFFECTOR_Y_MAX (0.5f)
#define ARM_END_EFFECTOR_Y_RANGE (ARM_END_EFFECTOR_Y_MAX - ARM_END_EFFECTOR_Y_MIN)
#define ARM_END_EFFECTOR_Y_HOME 0.0f
#define ARM_END_EFFECTOR_Y_RC_SEN (END_EFFECTOR_Y_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_Y_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_END_EFFECTOR_Y_RC_CHANGE_TIME_S 1.0f
#define ARM_END_EFFECTOR_Y_KEYBOARD_INC (END_EFFECTOR_Y_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_Y_RANGE / ARM_END_EFFECTOR_Y_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_END_EFFECTOR_Y_RC_SEN_INC (ARM_END_EFFECTOR_Y_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define ARM_END_EFFECTOR_Z_MIN (-0.5f)
#define ARM_END_EFFECTOR_Z_MAX (0.5f)
#define ARM_END_EFFECTOR_Z_RANGE (ARM_END_EFFECTOR_Z_MAX - ARM_END_EFFECTOR_Z_MIN)
#define ARM_END_EFFECTOR_Z_HOME 0.0f
#define ARM_END_EFFECTOR_Z_RC_SEN (END_EFFECTOR_Z_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_Z_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_END_EFFECTOR_Z_RC_CHANGE_TIME_S 1.0f
#define ARM_END_EFFECTOR_Z_KEYBOARD_INC (END_EFFECTOR_Z_RC_DIRECTION_GAIN * ARM_END_EFFECTOR_Z_RANGE / ARM_END_EFFECTOR_Z_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_END_EFFECTOR_Z_RC_SEN_INC (ARM_END_EFFECTOR_Z_KEYBOARD_INC / JOYSTICK_HALF_RANGE)

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//rocker value deadline
//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define CHASSIS_TEST_MODE 1

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)
//chassis task control time 0.002s
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//chassis 6020 max motor control voltage
//底盘6020最大can发送电压值
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f
//press the key, chassis will swing
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

// #define DRIVE_WHEEL_RADIUS 0.07625f // official iron wheel
#define DRIVE_WHEEL_RADIUS 0.077f // 3rd party wheel with motor 3508 embedded into wheel
#define MOTOR_3508_REDUCTION_RATIO (3591.0f/187.0f)
#define M3508_MOTOR_RPM_TO_VECTOR ((2.0f*PI/60.0f)*DRIVE_WHEEL_RADIUS/MOTOR_3508_REDUCTION_RATIO)

#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//single chassis motor max speed
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f

//rocker value (max 660) change to vertial speed (m/s) 
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN (NORMAL_MAX_CHASSIS_SPEED_X/JOYSTICK_HALF_RANGE)
//rocker value (max 660) change to horizontal speed (m/s)
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN (NORMAL_MAX_CHASSIS_SPEED_Y/JOYSTICK_HALF_RANGE)
//in following yaw angle mode, rocker value add to angle 
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

// Arbitrary offsets between chassis rotational center and centroid
#define CHASSIS_WZ_SET_SCALE 0.0f

//when chassis is not set to move, swing max angle
//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//底盘电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. 底盘有旋转速度控制
  CHASSIS_VECTOR_RAW,                 //control-current will be sent to CAN bus derectly.
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针, the point to remote control
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;               //state machine. 底盘控制状态机
  chassis_mode_e last_chassis_mode;          //last state machine.底盘上次控制状态机
  chassis_motor_t motor_chassis[4];          //chassis motor data.底盘电机数据
  pid_type_def motor_speed_pid[4];             //motor speed PID.底盘电机速度pid
  pid_type_def chassis_angle_pid;              //follow angle PID.底盘跟随角度pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.使用一阶低通滤波减缓设定值

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. 底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.后退方向最大速度 单位m/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.左方向最大速度 单位m/s
  fp32 vy_min_speed;  //max right speed, unit m/s.右方向最大速度 单位m/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.陀螺仪和云台电机叠加的roll角度

  robot_arm_behaviour_e robot_arm_mode;
  end_effector_cmd_t end_effector_cmd;
#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
  fp32 robot_arm_motor_pos[7];
#endif /* INDIVIDUAL_MOTOR_TEST */
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
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
void robot_arm_set_home(void);
#endif

extern chassis_move_t chassis_move;

#endif
