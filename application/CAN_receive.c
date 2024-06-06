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
#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"

#include "detect_task.h"
#include "shoot.h"
#include "string.h"

// Warning: for safety, PLEASE ALWAYS keep those default values as 0 when you commit
// Warning: because #if directive will assume the expression as 0 even if the macro is not defined, positive logic, for example, ENABLE_MOTOR_POWER, is safer that if and only if it's defined and set to 1 that the power is enabled
#define ENABLE_YAW_MOTOR_POWER 0
#define ENABLE_PITCH_MOTOR_POWER 0
#define ENABLE_TRIGGER_MOTOR_POWER 0
#define ENABLE_FRICTION_1_MOTOR_POWER 0
#define ENABLE_FRICTION_2_MOTOR_POWER 0

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/**
 * @brief motor feedback data
 * Chassis CAN:
 * 0:chassis motor1 3508; 1:chassis motor2 3508; 2:chassis motor3 3508; 3:chassis motor4 3508;
 * 6:trigger motor 2006; 4:yaw gimbal motor 6020;
 * 
 * Gimbal CAN:
 * 5:pitch gimbal motor 6020;
 */
motor_measure_t motor_chassis[MOTOR_LIST_LENGTH];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];

void decode_lower_head_data(uint8_t *data);

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t bMotorId = 0;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    
    if (hcan == &GIMBAL_CAN)
	{
		switch (rx_header.StdId)
		{
			case CAN_PIT_MOTOR_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH;
				get_motor_measure(&motor_chassis[bMotorId], rx_data);
				detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				break;
			}
			case CAN_TRIGGER_MOTOR_ID:
			{
				bMotorId = MOTOR_INDEX_TRIGGER;
				get_motor_measure(&motor_chassis[bMotorId], rx_data);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
			case CAN_FRICTION_MOTOR_LEFT_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_LEFT;
				get_motor_measure(&motor_chassis[bMotorId], rx_data);
				detect_hook(FRIC1_MOTOR_TOE);
				break;
			}
			case CAN_FRICTION_MOTOR_RIGHT_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_RIGHT;
				get_motor_measure(&motor_chassis[bMotorId], rx_data);
				detect_hook(FRIC2_MOTOR_TOE);
				break;
			}
			case CAN_YAW_MOTOR_ID:
			{
				bMotorId = MOTOR_INDEX_YAW;
				get_motor_measure(&motor_chassis[bMotorId], rx_data);
				detect_hook(YAW_GIMBAL_MOTOR_TOE);
				break;
			}
			default:
			{
				break;
			}
		}
	}
	else if (hcan == &CHASSIS_CAN)
	{
		switch (rx_header.StdId)
		{
			case CAN_LOWER_HEAD_RX_ID:
			{
				// @TODO: parse data from lower head
				decode_lower_head_data(rx_data);
				detect_hook(LOWER_HEAD_TOE);
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

void decode_lower_head_data(uint8_t *data)
{
    shoot_control.heat_limit = (data[1] << 8) | data[0];
    shoot_control.heat = (data[2] << 8) | data[2];
}

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      trigger: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      fric_left: 3508 motor control current when used as friction motor
  * @param[in]      fric_right: 3508 motor control current when used as friction motor
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t trigger, int16_t fric_left, int16_t fric_right)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_RM_LOW_RANGE_TX_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

#if (ENABLE_YAW_MOTOR_POWER == 0)
    yaw = 0;
#endif
#if (ENABLE_TRIGGER_MOTOR_POWER == 0)
    trigger = 0;
#endif
#if (ENABLE_PITCH_MOTOR_POWER == 0)
    pitch = 0;
#endif
#if ((ENABLE_FRICTION_1_MOTOR_POWER == 0) || (ENABLE_SHOOT_REDUNDANT_SWITCH == 0))
    fric_left = 0;
#endif
#if ((ENABLE_FRICTION_2_MOTOR_POWER == 0) || (ENABLE_SHOOT_REDUNDANT_SWITCH == 0))
    fric_right = 0;
#endif

    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (trigger >> 8);
    gimbal_can_send_data[5] = trigger;
    gimbal_can_send_data[6] = (fric_left >> 8);
    gimbal_can_send_data[7] = fric_left;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

    osDelay(1);
    gimbal_tx_message.StdId = CAN_RM_HIGH_RANGE_TX_ID;
    gimbal_can_send_data[0] = (fric_right >> 8);
    gimbal_can_send_data[1] = fric_right;
    // gimbal_can_send_data[2] = (rev >> 8);
    // gimbal_can_send_data[3] = rev;
    // gimbal_can_send_data[4] = (rev >> 8);
    // gimbal_can_send_data[5] = rev;
    // gimbal_can_send_data[6] = (rev >> 8);
    // gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[MOTOR_INDEX_YAW];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[MOTOR_INDEX_PITCH];
}
