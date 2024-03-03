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

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan1

#define RAD_TO_INT16_SCALE (32767.0f/PI) // (2^15-1)/PI
#define ONE_METER_TO_INT16_SCALE (32767.0f/1.0f) // (2^15-1)/1.0f

/* CAN send and receive ID */
typedef enum
{
	/*******Tx CAN IDs********/
	CAN_CHASSIS_M3508_TX_ID = 0x200,
	CAN_GIMBAL_ALL_TX_ID = 0x1FF,
  
	CAN_GIMBAL_CONTROLLER_POSITION_TX_ID = 0x114,
	CAN_GIMBAL_CONTROLLER_ORIENTATION_TX_ID = 0x115,
	CAN_GIMBAL_CONTROLLER_INDIVIDUAL_MOTOR_1_TX_ID = 0x116,
	CAN_GIMBAL_CONTROLLER_INDIVIDUAL_MOTOR_2_TX_ID = 0x117,

	/*******Chassis CAN IDs********/
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
} can_msg_id_e;

typedef enum
{
	MOTOR_INDEX_3508_M1 = 0,
	MOTOR_INDEX_3508_M2,
	MOTOR_INDEX_3508_M3,
	MOTOR_INDEX_3508_M4,
	MOTOR_LIST_LENGTH,
} can_motor_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef enum
{
  ROBOT_ARM_ENABLED,
  ROBOT_ARM_HOME,
  ROBOT_ARM_FIXED,
  ROBOT_ARM_ZERO_FORCE,
} robot_arm_behaviour_e;

typedef union
{
	struct
	{
    // unit: rad for angle, m for position
		fp32 roll_set;
		fp32 pitch_set;
		fp32 yaw_set;
		fp32 x_set;
		fp32 y_set;
		fp32 z_set;
	} setpoints;
	fp32 setpoints_data[6];
} end_effector_cmd_t;

void CAN_cmd_robot_arm_by_end_effector(end_effector_cmd_t _end_effector_cmd, robot_arm_behaviour_e arm_cmd_type);
void CAN_cmd_robot_arm_by_q(fp32 motor_pos[7], robot_arm_behaviour_e arm_cmd_type);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

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
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t motor_index);


#endif
