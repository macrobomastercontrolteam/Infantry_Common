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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "user_lib.h"
#include "detect_task.h"
#include "chassis_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define ENABLE_ARM_MOTOR_POWER 0
#define ENABLE_DRIVE_MOTOR_POWER 0
#define ENABLE_VTM_MOTOR_POWER 0

void decode_3508_motor_msg(motor_measure_t *ptr, uint8_t data[8]);
void decode_vtm_yaw_motor_msg(motor_measure_t *ptr, uint8_t data[8]);
void decode_vtm_pitch_motor_msg(motor_measure_t *ptr, uint8_t data[8]);

/**
 * @brief motor feedback data
 * Chassis CAN:
 * 0:chassis motor1 3508; 1:chassis motor2 3508; 2:chassis motor3 3508; 3:chassis motor4 3508;
 * 6:trigger motor 2006; 4:yaw gimbal motor 6020;
 * 
 * Gimbal CAN:
 * 5:pitch gimbal motor 6020;
 */
motor_measure_t motor_measure[MOTOR_LIST_LENGTH];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (hcan == &hcan1)
	{
		switch (rx_header.StdId)
			{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_VTM_YAW_ID:
			case CAN_VTM_PITCH_ID:
			{
				uint8_t bMotorId = rx_header.StdId - CAN_3508_M1_ID;
				decode_3508_motor_msg(&motor_measure[bMotorId], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + bMotorId);
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

void decode_3508_motor_msg(motor_measure_t *ptr, uint8_t data[8])
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)((data[0] << 8) | data[1]);
    ptr->speed_rpm = (int16_t)((data[2] << 8) | data[3]);
    ptr->given_current = (int16_t)((data[4] << 8) | data[5]);
    ptr->temperate = data[6];
}

void CAN_cmd_robot_arm_by_end_effector(end_effector_cmd_t _end_effector_cmd, robot_arm_behaviour_e arm_cmd_type, uint8_t fHoming)
{
    uint32_t send_mail_box;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    int16_t cmd_int16[6];
    switch (arm_cmd_type)
    {
#if ENABLE_ARM_MOTOR_POWER
      case ROBOT_ARM_CHANGEABLE:
      case ROBOT_ARM_FIXED:
      case ROBOT_ARM_HOME:
      {
        if (fHoming)
        {
          memset(cmd_int16, 0xFF, sizeof(cmd_int16));
        }
        else
        {
          cmd_int16[0] = _end_effector_cmd.setpoints.roll_set * RAD_TO_INT16_SCALE;
          cmd_int16[1] = _end_effector_cmd.setpoints.pitch_set * RAD_TO_INT16_SCALE;
          cmd_int16[2] = _end_effector_cmd.setpoints.yaw_set * RAD_TO_INT16_SCALE;
          cmd_int16[3] = _end_effector_cmd.setpoints.x_set * ONE_METER_TO_INT16_SCALE;
          cmd_int16[4] = _end_effector_cmd.setpoints.y_set * ONE_METER_TO_INT16_SCALE;
          cmd_int16[5] = _end_effector_cmd.setpoints.z_set * ONE_METER_TO_INT16_SCALE;
        }
		    break;
      }
      case ROBOT_ARM_ZERO_FORCE:
#endif
      default:
      {
        memset(cmd_int16, 0x00, sizeof(cmd_int16));
        break;
      }
    }
	  
    // position
    gimbal_tx_message.StdId = CAN_GIMBAL_CONTROLLER_POSITION_TX_ID;
    memcpy(gimbal_can_send_data, cmd_int16, 3*sizeof(cmd_int16));
    // redundant data in case of special cmd
    gimbal_can_send_data[6] = gimbal_can_send_data[5];
    gimbal_can_send_data[7] = gimbal_can_send_data[5];
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

    osDelay(1);

    // orientation
    gimbal_tx_message.StdId = CAN_GIMBAL_CONTROLLER_ORIENTATION_TX_ID;
    memcpy(gimbal_can_send_data, &cmd_int16[3], 3*sizeof(cmd_int16));
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

void CAN_cmd_robot_arm_by_q(fp32 motor_pos[7], robot_arm_behaviour_e arm_cmd_type, uint8_t fHoming)
{
    uint32_t send_mail_box;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    int16_t cmd_int16[7];
    switch (arm_cmd_type)
    {
#if ENABLE_ARM_MOTOR_POWER
      case ROBOT_ARM_CHANGEABLE:
      case ROBOT_ARM_FIXED:
      case ROBOT_ARM_HOME:
      {
        if (fHoming)
        {
          memset(cmd_int16, 0xFF, sizeof(cmd_int16));
        }
        else
        {
          uint8_t cmd_index;
          for (cmd_index = 0; cmd_index < sizeof(cmd_int16) / sizeof(cmd_int16[0]); cmd_index++)
          {
            cmd_int16[cmd_index] = motor_pos[cmd_index] * RAD_TO_INT16_SCALE;
          }
        }
		    break;
      }
      case ROBOT_ARM_ZERO_FORCE:
#endif
      default:
      {
        memset(cmd_int16, 0x00, sizeof(cmd_int16));
        break;
      }
    }

	  // position
    gimbal_tx_message.StdId = CAN_GIMBAL_CONTROLLER_INDIVIDUAL_MOTOR_1_TX_ID;
    memcpy(gimbal_can_send_data, cmd_int16, sizeof(gimbal_can_send_data));
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

    osDelay(1);

    // orientation
    gimbal_tx_message.StdId = CAN_GIMBAL_CONTROLLER_INDIVIDUAL_MOTOR_2_TX_ID;
    memcpy(gimbal_can_send_data, &cmd_int16[4], sizeof(cmd_int16) - sizeof(gimbal_can_send_data));
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

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
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

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
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    // driver motors (M3508)
    chassis_tx_message.StdId = CAN_CHASSIS_M3508_1_TX_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
#if (ENABLE_DRIVE_MOTOR_POWER == 0)
		memset(chassis_can_send_data, 0, sizeof(chassis_can_send_data));
#else
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
#endif
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_vtm_gimbal(int16_t motor5, int16_t motor6)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_M3508_2_TX_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
#if ENABLE_VTM_MOTOR_POWER
    if (vtm_gimbal.fVtmGimbalPowerEnabled)
    {
      chassis_can_send_data[0] = motor5 >> 8;
      chassis_can_send_data[1] = motor5;
      chassis_can_send_data[2] = motor6 >> 8;
      chassis_can_send_data[3] = motor6;
      // chassis_can_send_data[4] = rev >> 8;
      // chassis_can_send_data[5] = rev;
      // chassis_can_send_data[6] = rev >> 8;
      // chassis_can_send_data[7] = rev;
    }
    else
#endif
    {
      memset(chassis_can_send_data, 0, sizeof(chassis_can_send_data));
    }
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

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
const motor_measure_t *get_chassis_motor_measure_point(uint8_t motor_index)
{
	if (motor_index >= MOTOR_LIST_LENGTH)
	{
		return NULL;
	}
	else
	{
		return &motor_measure[motor_index];
	}
}
