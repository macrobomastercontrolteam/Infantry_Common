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

#define MG_6012 0
#define DM_4340P 1
#define HIP_MOTOR_TYPE DM_4340P

#define HIP_MOTOR_COUNT (CHASSIS_ID_HIP_4 - CHASSIS_ID_HIP_1 + 1)

#define MG6012_MAX_TORQUE 6.0f

typedef struct
{
	float set_torque;
	float feedback_abs_angle;
	float feedback_abs_ecd_fp32;
	uint16_t offset_ecd;
	uint16_t feedback_raw_ecd;
	int16_t rotor_speed;
	// int16_t  torque_current;
	// uint8_t  temperature;
    int16_t speed_rpm;
    int16_t feedback_current;
    uint8_t temperate;
    int16_t last_ecd;
    fp32 velocity;     // rad/s
    fp32 torque;       // Nm
} motor_info_t;

typedef enum
{
    DM_4340 = 0,
    LAST_MIT_CONTROLLED_MOTOR_TYPE,
} MIT_controlled_motor_type_e;

/* CAN send and receive ID */
typedef enum
{
	// Custom IDs
	// receives target chassis platform params: alpha1, alpha2, center height
	CAN_CHASSIS_CONTROLLER_RX_ID = 0x114,
	// sends current chassis platform params: alpha1, alpha2, center height
	CAN_CHASSIS_STATUS_TX_ID = 0x116,
	// hip motor tx
	// CAN_HIP_MOTOR_SINGLECMD_TX_ID = 0x140,
	CAN_HIP_MOTOR_MULTICMD_TX_ID = 0x280,

	#if HIP_MOTOR_TYPE == MG_6012
	// 6012 motor as hip
	CAN_HIP1_RX_ID = 0x141,
	CAN_HIP2_RX_ID = 0x142,
	CAN_HIP3_RX_ID = 0x143,
	CAN_HIP4_RX_ID = 0x144,
	#elif HIP_MOTOR_TYPE == DM_4340P
	// 4340 motor as hip
	CAN_HIP1_TX_ID = 0x001,	//ID used to send can commands
	CAN_HIP2_TX_ID = 0x002,
	CAN_HIP3_TX_ID = 0x003,
	CAN_HIP4_TX_ID = 0x004,

	CAN_HIP1_RX_ID = 0xFC, //ID used to receive motor feedback
	CAN_HIP2_RX_ID = 0xFD,
	CAN_HIP3_RX_ID = 0xFE,
	CAN_HIP4_RX_ID = 0xFF,
	#endif
} can_msg_id_e;

typedef enum
{
	CHASSIS_ID_HIP_1 = 0,   // right front
	CHASSIS_ID_HIP_2 = 1,   // left front
	CHASSIS_ID_HIP_3 = 2,   // left back
	CHASSIS_ID_HIP_4 = 3,   // right back
	CHASSIS_ID_LAST,
} chassis_motor_ID_e;

extern motor_info_t motor_info[CHASSIS_ID_LAST];

void can_user_init(void);
uint8_t CAN_cmd_hip_motors(float torque1, float torque2, float torque3, float torque4);
void encode_6012_multi_motor_torque_control(float torque1, float torque2, float torque3, float torque4);
void CAN_cmd_wrapper(void);

#if HIP_MOTOR_TYPE == DM_4340P
void enable_all_DaMiao_motors(uint8_t _enable);
#endif

#endif
