/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357

//the channel num of controlling vertial speed 
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//the channel num of controlling horizontal speed
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0

//in some mode, can use remote control to control rotation speed
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2

#define JOINT_0_RC_DIRECTION_GAIN (1.0f)
#define JOINT_1_RC_DIRECTION_GAIN (-1.0f)
#define JOINT_2_RC_DIRECTION_GAIN (1.0f)
#define JOINT_3_RC_DIRECTION_GAIN (-1.0f)
#define JOINT_4_RC_DIRECTION_GAIN (1.0f)
#define JOINT_5_RC_DIRECTION_GAIN (1.0f)
#define JOINT_6_RC_DIRECTION_GAIN (1.0f)

#define END_EFFECTOR_ROLL_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_PITCH_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_YAW_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_X_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_Y_RC_DIRECTION_GAIN (1.0f)
#define END_EFFECTOR_Z_RC_DIRECTION_GAIN (1.0f)

#define ARM_JOINT_CLEARANCE 0.08f

#define ARM_JOINT_0_ANGLE_MIN (-160.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_0_ANGLE_MAX (160.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_0_ANGLE_RANGE (ARM_JOINT_0_ANGLE_MAX - ARM_JOINT_0_ANGLE_MIN)
#define ARM_JOINT_0_ANGLE_HOME 0.0f
#define ARM_JOINT_0_RC_SEN (JOINT_0_RC_DIRECTION_GAIN * ARM_JOINT_0_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_0_ANGLE_RC_CHANGE_TIME_S 3.0f
#define ARM_JOINT_0_KEYBOARD_INC (JOINT_0_RC_DIRECTION_GAIN * ARM_JOINT_0_ANGLE_RANGE / ARM_JOINT_0_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_0_RC_SEN_INC (ARM_JOINT_0_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_0_ANGLE_STATIC 0.12226295471191406f 
#define ARM_JOINT_0_ANGLE_STORAGE 2.4664f

#define ARM_JOINT_1_ANGLE_MIN (-30.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_1_ANGLE_MAX (40.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_1_ANGLE_RANGE (ARM_JOINT_1_ANGLE_MAX - ARM_JOINT_1_ANGLE_MIN)
#define ARM_JOINT_1_ANGLE_HOME ARM_JOINT_1_ANGLE_MAX
#define ARM_JOINT_1_RC_SEN (JOINT_1_RC_DIRECTION_GAIN * ARM_JOINT_1_ANGLE_RANGE / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_1_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_1_KEYBOARD_INC (JOINT_1_RC_DIRECTION_GAIN * ARM_JOINT_1_ANGLE_RANGE / ARM_JOINT_1_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_1_RC_SEN_INC (ARM_JOINT_1_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_1_ANGLE_STATIC -0.12252672016620636f
#define ARM_JOINT_1_ANGLE_STORAGE 0.1145f

#define ARM_JOINT_2_ANGLE_MIN (-155.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_2_ANGLE_MAX (0.0f - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_2_ANGLE_RANGE (ARM_JOINT_2_ANGLE_MAX - ARM_JOINT_2_ANGLE_MIN)
#define ARM_JOINT_2_ANGLE_HOME ARM_JOINT_2_ANGLE_MIN
#define ARM_JOINT_2_RC_SEN (JOINT_2_RC_DIRECTION_GAIN * ARM_JOINT_2_ANGLE_RANGE / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_2_ANGLE_RC_CHANGE_TIME_S 1.5f
#define ARM_JOINT_2_KEYBOARD_INC (JOINT_2_RC_DIRECTION_GAIN * ARM_JOINT_2_ANGLE_RANGE / ARM_JOINT_2_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_2_RC_SEN_INC (ARM_JOINT_2_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_2_ANGLE_STATIC -0.9702428579330444f
#define ARM_JOINT_2_ANGLE_STORAGE -1.6439f

#define ARM_JOINT_3_ANGLE_MIN (-80.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_3_ANGLE_MAX (80.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_3_ANGLE_RANGE (ARM_JOINT_3_ANGLE_MAX - ARM_JOINT_3_ANGLE_MIN)
#define ARM_JOINT_3_ANGLE_HOME 0.0f
#define ARM_JOINT_3_RC_SEN (JOINT_3_RC_DIRECTION_GAIN * ARM_JOINT_3_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_3_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_3_KEYBOARD_INC (JOINT_3_RC_DIRECTION_GAIN * ARM_JOINT_3_ANGLE_RANGE / ARM_JOINT_3_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_3_RC_SEN_INC (ARM_JOINT_3_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_3_ANGLE_STATIC 0.21572399139404297f
#define ARM_JOINT_3_ANGLE_STORAGE 0.6903f

#define ARM_JOINT_4_ANGLE_MIN (-0.5f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_4_ANGLE_MAX (0.5f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_4_ANGLE_RANGE (ARM_JOINT_4_ANGLE_MAX - ARM_JOINT_4_ANGLE_MIN)
#define ARM_JOINT_4_ANGLE_HOME 0.0f
#define ARM_JOINT_4_RC_SEN (JOINT_4_RC_DIRECTION_GAIN * ARM_JOINT_4_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_4_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_4_KEYBOARD_INC (JOINT_4_RC_DIRECTION_GAIN * ARM_JOINT_4_ANGLE_RANGE / ARM_JOINT_4_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_4_RC_SEN_INC (ARM_JOINT_4_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_4_ANGLE_STATIC 0.0165939331054687f
#define ARM_JOINT_4_ANGLE_STORAGE -0.1104f

#define ARM_JOINT_5_ANGLE_MIN (0.0f / 180.0f * PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_5_ANGLE_MAX (180.0f / 180.0f * PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_5_ANGLE_RANGE (ARM_JOINT_5_ANGLE_MAX - ARM_JOINT_5_ANGLE_MIN)
#define ARM_JOINT_5_ANGLE_HOME ARM_JOINT_5_ANGLE_MAX
#define ARM_JOINT_5_RC_SEN (JOINT_5_RC_DIRECTION_GAIN * ARM_JOINT_5_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_5_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_5_KEYBOARD_INC (JOINT_5_RC_DIRECTION_GAIN * ARM_JOINT_5_ANGLE_RANGE / ARM_JOINT_5_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_5_RC_SEN_INC (ARM_JOINT_5_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_5_ANGLE_STATIC 1.5642404556274414f
#define ARM_JOINT_5_ANGLE_STORAGE 1.2228f

#define ARM_JOINT_6_ANGLE_MIN (-PI + ARM_JOINT_CLEARANCE)
#define ARM_JOINT_6_ANGLE_MAX (PI - ARM_JOINT_CLEARANCE)
#define ARM_JOINT_6_ANGLE_RANGE (ARM_JOINT_6_ANGLE_MAX - ARM_JOINT_6_ANGLE_MIN)
#define ARM_JOINT_6_ANGLE_HOME 0.0f
#define ARM_JOINT_6_RC_SEN (JOINT_6_RC_DIRECTION_GAIN * ARM_JOINT_6_ANGLE_RANGE / JOYSTICK_FULL_RANGE)
#define ARM_JOINT_6_ANGLE_RC_CHANGE_TIME_S 1.0f
#define ARM_JOINT_6_KEYBOARD_INC (JOINT_6_RC_DIRECTION_GAIN * ARM_JOINT_6_ANGLE_RANGE / ARM_JOINT_6_ANGLE_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define ARM_JOINT_6_RC_SEN_INC (ARM_JOINT_6_KEYBOARD_INC / JOYSTICK_HALF_RANGE)
#define ARM_JOINT_6_ANGLE_STATIC -0.8927768468856f
#define ARM_JOINT_6_ANGLE_STORAGE -1.2647f

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
//ҡ������
#define CHASSIS_RC_DEADLINE 10

#define CHASSIS_TEST_MODE 1

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//chassis task control time  2ms
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 3
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)
//chassis task control time 0.002s
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//chassis control frequence, no use now.
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//chassis 3508 max motor control current
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f
//chassis 6020 max motor control voltage
//����6020���can���͵�ѹֵ
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f
//press the key, chassis will swing
//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//chassi forward, back, left, right key
//����ǰ�����ҿ��ư���
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
//�������̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//chassis forward or back max speed
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//chassis left or right max speed
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f

//rocker value (max 660) change to vertial speed (m/s) 
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN (NORMAL_MAX_CHASSIS_SPEED_X/JOYSTICK_HALF_RANGE)
//rocker value (max 660) change to horizontal speed (m/s)
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN (NORMAL_MAX_CHASSIS_SPEED_Y/JOYSTICK_HALF_RANGE)
//in following yaw angle mode, rocker value add to angle 
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//in not following yaw angle mode, rocker value change to rotation speed
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f

// Arbitrary offsets between chassis rotational center and centroid
#define CHASSIS_WZ_SET_SCALE 0.0f

//when chassis is not set to move, swing max angle
//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//when chassis is set to move, swing max angle
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//chassis motor speed PID
//���̵���ٶȻ�PID
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

/************ VTM gimbal configs ************/
#define M3508_VTM_PITCH_ECD_PID_KP 50.0f
#define M3508_VTM_PITCH_ECD_PID_KI 0.0f
#define M3508_VTM_PITCH_ECD_PID_KD 0.0f
#define M3508_VTM_PITCH_ECD_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_VTM_PITCH_ECD_PID_MAX_IOUT 0.0f

#define M3508_VTM_YAW_ECD_PID_KP 50.0f
#define M3508_VTM_YAW_ECD_PID_KI 0.0f
#define M3508_VTM_YAW_ECD_PID_KD 200.0f
#define M3508_VTM_YAW_ECD_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_VTM_YAW_ECD_PID_MAX_IOUT 0.0f

#define VTM_YAW_KEYBOARD_CHANGE_TIME_S 1.0f
#define VTM_YAW_ECD_KEYBOARD_SEN_INC (PI / 2.0f * MOTOR_RAD_TO_ECD / VTM_YAW_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define VTM_PITCH_KEYBOARD_CHANGE_TIME_S 1.0f
#define VTM_PITCH_ECD_KEYBOARD_SEN_INC (PI / 2.0f * MOTOR_RAD_TO_ECD / VTM_PITCH_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
/************ VTM gimbal configs ************/

typedef enum
{
  CHASSIS_VECTOR_NO_FOLLOW_YAW,       //chassis will have rotation speed control. ��������ת�ٶȿ���
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
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��, the point to remote control
  const fp32 *chassis_INS_angle;             //the point to the euler angle of gyro sensor.��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //state machine. ���̿���״̬��
  chassis_mode_e last_chassis_mode;          //last state machine.�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];          //chassis motor data.���̵������
  pid_type_def motor_speed_pid[4];             //motor speed PID.���̵���ٶ�pid
  pid_type_def chassis_angle_pid;              //follow angle PID.���̸���Ƕ�pid

  first_order_filter_type_t chassis_cmd_slow_set_vx;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ
  first_order_filter_type_t chassis_cmd_slow_set_vy;  //use first order filter to slow set-point.ʹ��һ�׵�ͨ�˲������趨ֵ

  fp32 vx;                          //chassis vertical speed, positive means forward,unit m/s. �����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //chassis horizontal speed, positive means letf,unit m/s.�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //chassis rotation speed, positive means counterclockwise,unit rad/s.������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                      //chassis set vertical speed,positive means forward,unit m/s.�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //chassis set horizontal speed,positive means left,unit m/s.�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //chassis set rotation speed,positive means counterclockwise,unit rad/s.�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  //max forward speed, unit m/s.ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //max backward speed, unit m/s.���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  //max letf speed, unit m/s.��������ٶ� ��λm/s
  fp32 vy_min_speed;  //max right speed, unit m/s.�ҷ�������ٶ� ��λm/s
  fp32 chassis_yaw;   //the yaw angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //the pitch angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //the roll angle calculated by gyro sensor and gimbal motor.�����Ǻ���̨������ӵ�roll�Ƕ�
  
  uint8_t fHoming;
  uint8_t fStatic;
  uint8_t fStorage;
  robot_arm_behaviour_e robot_arm_mode;
  end_effector_cmd_t end_effector_cmd;
#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
  fp32 robot_arm_motor_pos[7];
#endif /* INDIVIDUAL_MOTOR_TEST */
} chassis_move_t;

typedef struct
{
  pid_type_def pitch_ecd_pid;
  pid_type_def yaw_ecd_pid;

  int16_t pitch_current_cmd;
  int16_t yaw_current_cmd;

  // Note: encoder of 3508 is not permanent
  // upwards is positive
  fp32 pitch_target_ecd;
  // to the right is positive
  fp32 yaw_target_ecd;
  uint8_t fVtmGimbalPowerEnabled;
} vtm_gimbal_t;

/**
  * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: ��
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
  * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
  *                 
  * @param[out]     vx_set: �����ٶ�ָ��
  * @param[out]     vy_set: �����ٶ�ָ��
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
void robot_arm_set_home(void);
#endif

extern const fp32 joint_angle_min[7];
extern const fp32 joint_angle_max[7];
extern chassis_move_t chassis_move;
extern vtm_gimbal_t vtm_gimbal;

#endif
