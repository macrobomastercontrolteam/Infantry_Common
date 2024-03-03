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

typedef enum
{
    // main IDs: used for indexing
    CAN_JOINT_MOTOR_0_4310_TX_ID = 0x001,
    CAN_JOINT_MOTOR_1_6012_TX_ID = 0x142,
    CAN_JOINT_MOTOR_2_4010_TX_ID = 0x143,
    CAN_JOINT_MOTOR_3_4310_TX_ID = 0x004,
    CAN_JOINT_MOTOR_4_4310_TX_ID = 0x005,
    CAN_JOINT_MOTOR_5_4310_TX_ID = 0x006,
    CAN_JOINT_MOTOR_6_6020_RX_ID = 0x20B,

    // other IDs
    CAN_DAMIAO_RX_ID = 0x0FF,
    CAN_MOTOR_6020_TX_ID = 0x2FF,
    CAN_JOINT_MOTOR_1_6012_RX_ID = 0x282,
    CAN_JOINT_MOTOR_2_4010_RX_ID = 0x283,

    CAN_INTER_BOARD_POSITION_RX_ID = 0x114,
	CAN_INTER_BOARD_ORIENTATION_RX_ID = 0x115,
	CAN_INTER_BOARD_INDIVIDUAL_MOTOR_1_RX_ID = 0x116,
	CAN_INTER_BOARD_INDIVIDUAL_MOTOR_2_RX_ID = 0x117,
} can_msg_id_e;

// Consecutive indices corresponding to individual can_msg_id_e
typedef enum
{
    JOINT_ID_0_4310 = 0,
    JOINT_ID_1_6012 = 1,
    JOINT_ID_2_4010 = 2,
    JOINT_ID_3_4310 = 3,
    JOINT_ID_4_4310 = 4,
    JOINT_ID_5_4310 = 5,
    JOINT_ID_6_6020 = 6,
    JOINT_ID_LAST,
} chassis_motor_ID_e;

typedef enum
{
    CAN_6012_TORQUE_RX_ID = 0xA1,
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
    int16_t speed_rpm;
    int16_t given_current;
    int16_t last_ecd;
    fp32 output_angle; // rad
    fp32 input_angle;  // rad
    fp32 velocity;     // rad/s
    fp32 torque;       // Nm
    fp32 power;
} motor_measure_t;

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          返回底盘电机 3508电机数据指针
 * @param[in]      i: 电机编号,范围[0,3]
 * @retval         电机数据指针
 */
uint8_t arm_joints_cmd_position(float *joint_angle_target_ptr, fp32 dt);

extern motor_measure_t motor_measure[JOINT_ID_LAST];

void update_joint_6_6020_angle(void);
void enable_all_motor_control(uint8_t _enable);

#ifdef __cplusplus
}
#endif

#endif /* CAN_RECEIVE_H */
