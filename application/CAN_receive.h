/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "global_inc.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

#define CAN_CONTROL_ID_BASE 0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define STEER_MOTOR_COUNT (CHASSIS_ID_STEER_4 - CHASSIS_ID_STEER_1 + 1)
#define HIP_MOTOR_COUNT (CHASSIS_ID_HIP_4 - CHASSIS_ID_HIP_1 + 1)

#define M6020_MAX_VOLTAGE 20000.0f
#define MG6012_MAX_TORQUE 6.0f

typedef struct
{
	float set_torque;
	float feedback_abs_angle;
	float feedback_abs_ecd_fp32;
	int16_t set_voltage;
	uint16_t offset_ecd;
	uint16_t feedback_raw_ecd;
	uint16_t target_ecd;
	int16_t rotor_speed;
	// int16_t  torque_current;
	// uint8_t  temperature;
} motor_info_t;

/* CAN send and receive ID */
typedef enum
{
	// Custom IDs
	CAN_STEER_CONTROLLER_RX_ID = 0x112,
	// CAN_CHASSIS_LOAD_SERVO_RX_ID = 0x113, // implementation is in Infantry_3_4Steer, not migrated here unless it becomes demanding
	// receives target chassis platform params: alpha1, alpha2, center height
	CAN_SWERVE_CONTROLLERE_RX_ID = 0x114,
	// sends target derivative of rotational radius of each wheel
	CAN_SWERVE_RADII_DOT_TX_ID = 0x115,
	// sends current chassis platform params: alpha1, alpha2, center height, rotational radius of each wheels
	CAN_SHRINKED_CONTROLLER_TX_ID = 0x116,

	// steer motor tx
	CAN_CHASSIS_GM6020_TX_ID = 0x1FF,
	// hip motor tx
	// CAN_HIP_MOTOR_SINGLECMD_TX_ID = 0x140,
	CAN_HIP_MOTOR_MULTICMD_TX_ID = 0x280,

	CAN_STEER1_RX_ID = 0x205,
	CAN_STEER2_RX_ID = 0x206,
	CAN_STEER3_RX_ID = 0x207,
	CAN_STEER4_RX_ID = 0x208,

	// 6012 motor as hip
	CAN_HIP1_RX_ID = 0x141,
	CAN_HIP2_RX_ID = 0x142,
	CAN_HIP3_RX_ID = 0x143,
	CAN_HIP4_RX_ID = 0x144,
} can_msg_id_e;

typedef enum
{
	CHASSIS_ID_STEER_1 = 0, // right front
	CHASSIS_ID_STEER_2 = 1, // left front
	CHASSIS_ID_STEER_3 = 2, // left back
	CHASSIS_ID_STEER_4 = 3, // right back
	CHASSIS_ID_HIP_1 = 4,   // right front
	CHASSIS_ID_HIP_2 = 5,   // left front
	CHASSIS_ID_HIP_3 = 6,   // left back
	CHASSIS_ID_HIP_4 = 7,   // right back
	CHASSIS_ID_LAST,
} chassis_motor_ID_e;

extern motor_info_t motor_info[CHASSIS_ID_LAST];

void can_user_init(void);
void CAN_cmd_steer_motors(uint8_t id_range, int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4);
uint8_t CAN_cmd_hip_motors(float torque1, float torque2, float torque3, float torque4);
void CAN_send_shrinked_params_to_upper_board(fp32 radius1, fp32 radius2, fp32 radius3, fp32 radius4, fp32 current_alpha1, fp32 current_alpha2, fp32 current_height);
void CAN_send_radius_dot_to_upper_board(fp32 target_radius_dot1, fp32 target_radius_dot2, fp32 target_radius_dot3, fp32 target_radius_dot4);
void encode_6012_multi_motor_torque_control(float torque1, float torque2, float torque3, float torque4);

#endif
