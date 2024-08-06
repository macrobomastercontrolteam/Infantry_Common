/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

#define LOWER_MOTORS_CAN hcan1
#define UPPER_MOTORS_CAN hcan2

#define ENABLE_JOINT_0_MOTOR 1
#define ENABLE_JOINT_1_MOTOR 1
#define ENABLE_JOINT_2_MOTOR 1
#define ENABLE_JOINT_3_MOTOR 1
#define ENABLE_JOINT_4_MOTOR 1
#define ENABLE_JOINT_5_MOTOR 1
#define ENABLE_JOINT_6_MOTOR 0

#define MOTOR_MG4005_GEAR_RATIO 10.0f
#define MOTOR_MG4005_MAX_CMD 2000.0f

#define MOTOR_MS4005_GEAR_RATIO 1.0f
#define MOTOR_MS4005_MAX_CMD 2000.0f

typedef enum
{
    // DaMiao motors
    // @TODO: use different rx id for different damiao motors
    CAN_JOINT_MOTOR_0_4310_TX_ID = 0x001,
    CAN_JOINT_MOTOR_1_4310_TX_ID = 0x002,
    CAN_JOINT_MOTOR_2_4310_TX_ID = 0x003,

    CAN_JOINT_MOTOR_0_4310_RX_ID = 0x0F1,
    CAN_JOINT_MOTOR_1_4310_RX_ID = 0x0F2,
    CAN_JOINT_MOTOR_2_4310_RX_ID = 0x0F3,

    // KTech motors
    CAN_KTECH_BROADCAST_CURRENT_TX_ID = 0x280,
    // MG4005E-i10
    CAN_JOINT_MOTOR_3_4005_RX_ID = 0x141,
    // MS4005-v3
    CAN_JOINT_MOTOR_4_4005_RX_ID = 0x142,
    CAN_JOINT_MOTOR_5_4005_RX_ID = 0x143,
    CAN_JOINT_MOTOR_6_4005_RX_ID = 0x144,
} can_msg_id_e;

// Consecutive indices corresponding to individual can_msg_id_e
typedef enum
{
    JOINT_ID_0_4310 = 0,
    JOINT_ID_1_4310 = 1,
    JOINT_ID_2_4310 = 2,
    JOINT_ID_3_4005 = 3,
    JOINT_ID_4_4005 = 4,
    JOINT_ID_5_4005 = 5,
    JOINT_ID_6_4005 = 6,
    JOINT_ID_LAST,
} chassis_motor_ID_e;

typedef enum
{
    CAN_KTECH_TORQUE_ID = 0xA1,
    CAN_KTECH_TEMP_DISABLE_MOTOR_ID = 0x81,
    CAN_KTECH_MULTIANGLE_2_ID = 0xA4,
    CAN_9015_MULTIANGLE_MSG_ID = 0x92,
    CAN_9015_SET_CURRENT_ZERO_POINT_MSG_ID = 0x64,
} can_msg_type_e;

typedef enum
{
    DM_8006 = 0,
    MA_9015 = 1,
    DM_4310 = 2,
    LAST_MIT_CONTROLLED_MOTOR_TYPE,
} MIT_controlled_motor_type_e;

// rm motor data
typedef struct
{
    int8_t temperature;
    uint16_t ecd;
    int16_t last_ecd;
    // int16_t speed_rpm;
    // int16_t given_current;
    fp32 output_angle; // rad
    // fp32 input_angle;  // rad
    fp32 velocity; // rad/s
    fp32 torque;   // Nm
    fp32 feedback_current;
} motor_measure_t;

extern motor_measure_t motor_measure[JOINT_ID_LAST];

uint8_t arm_joints_cmd_position(float joint_angle_target_ptr[7], fp32 dt, uint8_t fIsTeaching);
void arm_joints_cmd_torque(const fp32 joint_torques[7]);
void update_joint_6_6020_angle(void);
void CAN_cmd_switch_motor_power(uint8_t _enable);

#ifdef __cplusplus
}
#endif

#endif /* CAN_RECEIVE_H */
