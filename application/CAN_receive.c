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
#include "chassis_task.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "main.h"
#include "remote_control.h"
#include "string.h"

// Warning: for safety, PLEASE ALWAYS keep those default values as 0 when you commit
// Warning: because #if directive will assume the expression as 0 even if the macro is not defined, positive logic, for example, ENABLE_MOTOR_POWER, is safer that if and only if it's defined and set to 1 that the power is enabled
#define ENABLE_STEER_MOTOR_POWER 0
#define ENABLE_HIP_MOTOR_POWER 0

// reverse hip motor direction
#define REVERSE_1_HIP_MOTOR_DIRECTION 1
#define REVERSE_2_HIP_MOTOR_DIRECTION 1
#define REVERSE_3_HIP_MOTOR_DIRECTION 1
#define REVERSE_4_HIP_MOTOR_DIRECTION 1

#define MOTOR_6012_GEAR_RATIO 36.0f
#define MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO 0.225146199f
#define MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO 0.212f
#define MOTOR_6012_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 33.0f * 2048.0f)
#define MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 32.0f * 2000.0f)

#define MG6012_ECD_DELTA_DEADZONE MG6012_ECD_RANGE_90
#define MG6012_SPEED_DPS_DELTA_DEADZONE 100

typedef enum
{
	CAN_6012_TORQUE_FEEDBACK_ID = 0xA1,
} can_msg_type_e;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_info_t motor_info[CHASSIS_ID_LAST];
static CAN_TxHeaderTypeDef can_tx_msg;
static uint8_t can_tx_data[8];
static uint32_t send_mail_box;
const uint8_t abAllFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const fp32 speed_encoding_ratio = (1 << 15) / METER_PER_SEC_ECD_MAX_LIMIT;
const fp32 meter_encoding_ratio = (1 << 16) / METER_ENCODER_MAX_LIMIT;
const fp32 angle_encoding_ratio = (1 << 15) / ANGLE_ECD_MAX_LIMIT;

const fp32 meter_encoding_ratio_shrinked = (1 << 8) / METER_ENCODER_MAX_LIMIT;
const fp32 angle_encoding_ratio_shrinked = (1 << 7) / ANGLE_ECD_MAX_LIMIT;

void decode_6020_motor_feedback(uint8_t *data, uint8_t bMotorId);
void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId);

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	uint8_t bMotorId = 0xFF;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (hcan == &hcan1)
	{
		switch (rx_header.StdId)
		{
			case CAN_STEER1_RX_ID:
			case CAN_STEER2_RX_ID:
			case CAN_STEER3_RX_ID:
			case CAN_STEER4_RX_ID:
			{
				bMotorId = rx_header.StdId - CAN_STEER1_RX_ID + CHASSIS_ID_STEER_1;
				decode_6020_motor_feedback(rx_data, bMotorId);
				break;
			}
			case CAN_HIP1_RX_ID:
			case CAN_HIP2_RX_ID:
			case CAN_HIP3_RX_ID:
			case CAN_HIP4_RX_ID:
			{
				if (rx_data[0] == CAN_6012_TORQUE_FEEDBACK_ID)
				{
					bMotorId = rx_header.StdId - CAN_HIP1_RX_ID + CHASSIS_ID_HIP_1;
					decode_6012_motor_torque_feedback(rx_data, bMotorId);
				}
				break;
			}
			default:
			{
				break;
			}
		}
	}
	else if (hcan == &hcan2)
	{
		// Note: filter is enabled for CAN2, so be careful with the mask config
		switch (rx_header.StdId)
		{
			case CAN_STEER_CONTROLLER_RX_ID:
			{
				chassis_move.fSteerMotorEnabled = (memcmp(rx_data, abAllFF, sizeof(abAllFF)) != 0);
				if (chassis_move.fSteerMotorEnabled)
				{
					for (bMotorId = 0; bMotorId < STEER_MOTOR_COUNT; bMotorId++)
					{
						motor_info[bMotorId].target_ecd = ((rx_data[2 * bMotorId] << 8) | rx_data[2 * bMotorId + 1]);
					}
				}
				break;
			}
			case CAN_SWERVE_CONTROLLERE_RX_ID:
			{
				chassis_move.fHipMotorEnabled = (memcmp(rx_data, abAllFF, sizeof(abAllFF)) != 0);
#if (HEADLESS_HIP_TEST == 0)
				if (chassis_move.fHipMotorEnabled)
				{
					chassis_move.target_alpha1 = (int16_t)((rx_data[0] << 8) | rx_data[1]) / angle_encoding_ratio;
					chassis_move.target_alpha2 = (int16_t)((rx_data[2] << 8) | rx_data[3]) / angle_encoding_ratio;
					chassis_move.target_height = (uint16_t)((rx_data[4] << 8) | rx_data[5]) / meter_encoding_ratio;

					chassis_move.target_height = fp32_constrain(chassis_move.target_height, CHASSIS_H_LOWER_LIMIT, CHASSIS_H_UPPER_LIMIT);

					// calculation for alpha limit: According to the matlab calculation, the available workspace in height-alpha space is triangular, so we assume height target has more priority than alpha target, and calculate alpha limit based on height
					if (chassis_move.target_height >= CHASSIS_H_WORKSPACE_PEAK)
					{
						chassis_move.alpha_upper_limit = (chassis_move.target_height - CHASSIS_H_UPPER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE2;
					}
					else
					{
						chassis_move.alpha_upper_limit = (chassis_move.target_height - CHASSIS_H_LOWER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE1;
					}
					chassis_move.alpha_lower_limit = -chassis_move.alpha_upper_limit;

					chassis_move.target_alpha1 = fp32_constrain(chassis_move.target_alpha1, chassis_move.alpha_lower_limit, chassis_move.alpha_upper_limit);
					chassis_move.target_alpha2 = fp32_constrain(chassis_move.target_alpha2, chassis_move.alpha_lower_limit, chassis_move.alpha_upper_limit);
				}
#endif
				break;
			}
			default:
			{
				break;
			}
		}
	}
}

uint8_t CAN_cmd_hip_motors(float torque1, float torque2, float torque3, float torque4)
{
	uint8_t fValidInput = (((torque1 != torque1) || (torque2 != torque2) || (torque3 != torque3) || (torque4 != torque4)) == 0);
#if ENABLE_HIP_MOTOR
	// if ((chassis_move.fHipMotorEnabled == 0) || (fValidInput == 0) || (chassis_move.fHipDataIsValid == 0))
	if ((chassis_move.fHipMotorEnabled == 0) || (fValidInput == 0))
#endif
	{
		torque1 = 0;
		torque2 = 0;
		torque3 = 0;
		torque4 = 0;
	}

#if REVERSE_3_HIP_MOTOR_DIRECTION
	torque3 *= -1.0f;
#endif

#if REVERSE_2_HIP_MOTOR_DIRECTION
	torque2 *= -1.0f;
#endif

#if REVERSE_4_HIP_MOTOR_DIRECTION
	torque4 *= -1.0f;
#endif

#if REVERSE_1_HIP_MOTOR_DIRECTION
	torque1 *= -1.0f;
#endif

	// 6012 motor as hip
	encode_6012_multi_motor_torque_control(torque1, torque2, torque3, torque4);
	return fValidInput;
}

void CAN_send_shrinked_params_to_upper_board(fp32 current_radius1, fp32 current_radius2, fp32 current_radius3, fp32 current_radius4, fp32 current_alpha1, fp32 current_alpha2, fp32 current_height)
{
	can_tx_msg.StdId = CAN_SHRINKED_CONTROLLER_TX_ID;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 8;

	if (chassis_move.fHipDataIsValid)
	{
		uint8_t current_radius1_uint = fp32_constrain(current_radius1, 0, METER_ENCODER_MAX_LIMIT) * meter_encoding_ratio_shrinked;
		uint8_t current_radius2_uint = fp32_constrain(current_radius2, 0, METER_ENCODER_MAX_LIMIT) * meter_encoding_ratio_shrinked;
		uint8_t current_radius3_uint = fp32_constrain(current_radius3, 0, METER_ENCODER_MAX_LIMIT) * meter_encoding_ratio_shrinked;
		uint8_t current_radius4_uint = fp32_constrain(current_radius4, 0, METER_ENCODER_MAX_LIMIT) * meter_encoding_ratio_shrinked;

		int8_t current_alpha1_int = fp32_abs_constrain(current_alpha1, ANGLE_ECD_MAX_LIMIT) * angle_encoding_ratio_shrinked;
		int8_t current_alpha2_int = fp32_abs_constrain(current_alpha2, ANGLE_ECD_MAX_LIMIT) * angle_encoding_ratio_shrinked;
		uint8_t current_height_uint = fp32_constrain(current_radius4, 0, METER_ENCODER_MAX_LIMIT) * meter_encoding_ratio_shrinked;

		can_tx_data[0] = current_radius1_uint;
		can_tx_data[1] = current_radius2_uint;
		can_tx_data[2] = current_radius3_uint;
		can_tx_data[3] = current_radius4_uint;

		can_tx_data[4] = current_alpha1_int;
		can_tx_data[5] = current_alpha2_int;
		can_tx_data[6] = current_height_uint;
		// reserved
		// can_tx_data[7] = rev;
	}
	else
	{
		memset(can_tx_data, 0xFF, sizeof(can_tx_data));
	}
	HAL_CAN_AddTxMessage(&hcan2, &can_tx_msg, can_tx_data, &send_mail_box);
}

void CAN_send_radius_dot_to_upper_board(fp32 target_radius_dot1, fp32 target_radius_dot2, fp32 target_radius_dot3, fp32 target_radius_dot4)
{
	can_tx_msg.StdId = CAN_SWERVE_RADII_DOT_TX_ID;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 8;

	if (chassis_move.fHipDataIsValid)
	{
		int16_t target_radius_dot1_int = fp32_abs_constrain(target_radius_dot1, METER_PER_SEC_ECD_MAX_LIMIT) * speed_encoding_ratio;
		int16_t target_radius_dot2_int = fp32_abs_constrain(target_radius_dot2, METER_PER_SEC_ECD_MAX_LIMIT) * speed_encoding_ratio;
		int16_t target_radius_dot3_int = fp32_abs_constrain(target_radius_dot3, METER_PER_SEC_ECD_MAX_LIMIT) * speed_encoding_ratio;
		int16_t target_radius_dot4_int = fp32_abs_constrain(target_radius_dot4, METER_PER_SEC_ECD_MAX_LIMIT) * speed_encoding_ratio;

		can_tx_data[0] = *(uint8_t *)(&target_radius_dot1_int);
		can_tx_data[1] = *((uint8_t *)(&target_radius_dot1_int) + 1);
		can_tx_data[2] = *(uint8_t *)(&target_radius_dot2_int);
		can_tx_data[3] = *((uint8_t *)(&target_radius_dot2_int) + 1);
		can_tx_data[4] = *(uint8_t *)(&target_radius_dot3_int);
		can_tx_data[5] = *((uint8_t *)(&target_radius_dot3_int) + 1);
		can_tx_data[6] = *(uint8_t *)(&target_radius_dot4_int);
		can_tx_data[7] = *((uint8_t *)(&target_radius_dot4_int) + 1);
	}
	else
	{
		memset(can_tx_data, 0xFF, sizeof(can_tx_data));
	}
	HAL_CAN_AddTxMessage(&hcan2, &can_tx_msg, can_tx_data, &send_mail_box);
}

void encode_6012_multi_motor_torque_control(float torque1, float torque2, float torque3, float torque4)
{
	can_tx_msg.StdId = CAN_HIP_MOTOR_MULTICMD_TX_ID;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 8;

	int16_t iqControl_1 = torque1 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	int16_t iqControl_2 = torque2 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	int16_t iqControl_3 = torque3 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	int16_t iqControl_4 = torque4 * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;

	can_tx_data[0] = *(uint8_t *)(&iqControl_1);
	can_tx_data[1] = *((uint8_t *)(&iqControl_1) + 1);
	can_tx_data[2] = *(uint8_t *)(&iqControl_2);
	can_tx_data[3] = *((uint8_t *)(&iqControl_2) + 1);
	can_tx_data[4] = *(uint8_t *)(&iqControl_3);
	can_tx_data[5] = *((uint8_t *)(&iqControl_3) + 1);
	can_tx_data[6] = *(uint8_t *)(&iqControl_4);
	can_tx_data[7] = *((uint8_t *)(&iqControl_4) + 1);

	HAL_CAN_AddTxMessage(&hcan1, &can_tx_msg, can_tx_data, &send_mail_box);
}

void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId)
{
	// int16_t iq_int = (data[3] << 8) | data[2];    // A
	int16_t v_int = (data[5] << 8) | data[4];     // deg/s
	uint16_t p_uint = ((data[7] << 8) | data[6]); // 16bit abs encoder

	// motor_info[bMotorId].torque = ((float)iq_int) / MOTOR_6012_CMD_TO_TORQUE_RATIO;
	// motor_info[bMotorId].velocity = ((float)v_int) / 36.0f / 180.0f * PI;
	int16_t temp_rotor_speed = v_int / 36;
	uint16_t temp_feedback_raw_ecd = p_uint;
	// motor_info[bMotorId].temperature = data[1];

#if (REVERSE_1_HIP_MOTOR_DIRECTION || REVERSE_2_HIP_MOTOR_DIRECTION || REVERSE_3_HIP_MOTOR_DIRECTION || REVERSE_4_HIP_MOTOR_DIRECTION)
	switch (bMotorId)
	{
#if REVERSE_1_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_1:
#endif
#if REVERSE_2_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_2:
#endif
#if REVERSE_3_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_3:
#endif
#if REVERSE_4_HIP_MOTOR_DIRECTION
		case CHASSIS_ID_HIP_4:
#endif
		{
			// motor_info[bMotorId].torque *= -1.0f;
			// motor_info[bMotorId].velocity *= -1.0f;
			temp_rotor_speed *= -1;
			temp_feedback_raw_ecd = MG6012_loop_ecd_constrain(MG6012_ECD_RANGE - temp_feedback_raw_ecd);
			break;
		}
		default:
		{
			break;
		}
	}
#endif
	// patch for sudden erroneous ecd change as observed from the motor
	if (MG6012_loop_ecd_constrain(temp_feedback_raw_ecd - motor_info[bMotorId].feedback_raw_ecd) < MG6012_ECD_DELTA_DEADZONE)
	{
		motor_info[bMotorId].feedback_raw_ecd = temp_feedback_raw_ecd;
		motor_info[bMotorId].feedback_abs_ecd_fp32 = MG6012_loop_ecd_constrain((float)motor_info[bMotorId].feedback_raw_ecd - (float)motor_info[bMotorId].offset_ecd);
		motor_info[bMotorId].feedback_abs_angle = ((float)motor_info[bMotorId].feedback_abs_ecd_fp32) / (1 << 16) * 2.0f * PI;

		if (chassis_move.fFatalError == 0)
		{
			chassis_move.fHipDataIsValid = 1;
		}
	}

	int32_t speed_diff = motor_info[bMotorId].rotor_speed - temp_rotor_speed;
	if (abs(speed_diff) < MG6012_SPEED_DPS_DELTA_DEADZONE)
	{
		motor_info[bMotorId].rotor_speed = temp_rotor_speed;
	}
}

void decode_6020_motor_feedback(uint8_t *data, uint8_t bMotorId)
{
	motor_info[bMotorId].feedback_raw_ecd = ((data[0] << 8) | data[1]);
	motor_info[bMotorId].feedback_abs_ecd_fp32 = M6020_loop_ecd_constrain((float)motor_info[bMotorId].feedback_raw_ecd - (float)motor_info[bMotorId].offset_ecd);
	motor_info[bMotorId].rotor_speed = ((data[2] << 8) | data[3]);
	// motor_info[bMotorId].torque_current = ((data[4] << 8) | data[5]);
	// motor_info[bMotorId].temperature    =   data[6];
}

/**
 * @brief  send motor control message through can bus
 * @param  motor voltage 1,2,3,4 or 5,6,7
 * @retval None
 */
void CAN_cmd_steer_motors(uint8_t id_range, int16_t voltage1, int16_t voltage2, int16_t voltage3, int16_t voltage4)
{
	can_tx_msg.StdId = (id_range == 0) ? (CAN_CONTROL_ID_BASE) : (CAN_CONTROL_ID_EXTEND);
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 8;

#if ENABLE_STEER_MOTOR
	if (chassis_move.fSteerMotorEnabled == 0)
#endif
	{
		voltage1 = 0;
		voltage2 = 0;
		voltage3 = 0;
		voltage4 = 0;
	}

	can_tx_data[0] = voltage1 >> 8;
	can_tx_data[1] = voltage1;
	can_tx_data[2] = voltage2 >> 8;
	can_tx_data[3] = voltage2;
	can_tx_data[4] = voltage3 >> 8;
	can_tx_data[5] = voltage3;
	can_tx_data[6] = voltage4 >> 8;
	can_tx_data[7] = voltage4;
	HAL_CAN_AddTxMessage(&hcan1, &can_tx_msg, can_tx_data, &send_mail_box);
}
