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
#include <stdlib.h>
#include "user_lib.h"

// Warning: for safety, PLEASE ALWAYS keep those default values as 0 when you commit
// Warning: because #if directive will assume the expression as 0 even if the macro is not defined, positive logic, for example, ENABLE_MOTOR_POWER, is safer that if and only if it's defined and set to 1 that the power is enabled
#define ENABLE_HIP_MOTOR_POWER 0

// reverse hip motor direction
#define REVERSE_1_HIP_MOTOR_DIRECTION 1
#define REVERSE_2_HIP_MOTOR_DIRECTION 1
#define REVERSE_3_HIP_MOTOR_DIRECTION 1
#define REVERSE_4_HIP_MOTOR_DIRECTION 1

#if HIP_MOTOR_TYPE == MG_6012
#define MOTOR_6012_GEAR_RATIO 36.0f
#define MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO 0.225146199f
#define MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO 0.212f
#define MOTOR_6012_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 33.0f * 2048.0f)
#define MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 32.0f * 2000.0f)

#define MG6012_ECD_DELTA_DEADZONE MG6012_ECD_RANGE_90
#define MG6012_SPEED_DPS_DELTA_DEADZONE 100

#elif HIP_MOTOR_TYPE == DM_4340P
const fp32 MIT_CONTROL_P_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {4.0f*PI};  //value needs to match to which in motor setting software
const fp32 MIT_CONTROL_P_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = { -4.0f*PI};
const fp32 MIT_CONTROL_V_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {30.0f};
const fp32 MIT_CONTROL_V_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-30.0f};
const fp32 MIT_CONTROL_T_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {10.0f};
const fp32 MIT_CONTROL_T_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-10.0f};
const fp32 MIT_CONTROL_KP_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {500.0f};
const fp32 MIT_CONTROL_KP_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f};
const fp32 MIT_CONTROL_KD_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {5.0f};
const fp32 MIT_CONTROL_KD_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f};

fp32 uint_to_fp32_motor(int x_int, fp32 x_min, fp32 x_max, int bits);
int fp32_to_uint_motor(fp32 x, fp32 x_min, fp32 x_max, int bits);
HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef decode_4340_motor_feedback(uint8_t *data, uint8_t bMotorId);
HAL_StatusTypeDef enable_DaMiao_motor(uint32_t id, uint8_t _enable, CAN_HandleTypeDef *hcan_ptr);
void enable_all_DaMiao_motors(uint8_t _enable);
#endif

#define INTER_CTRL_CAN hcan1
#define HIP_CAN hcan2


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

const fp32 meter_encoding_ratio = ((1 << 16)- 1) / METER_ENCODER_MAX_LIMIT;
const fp32 angle_encoding_ratio = ((1 << 15)- 1) / ANGLE_ECD_MAX_LIMIT;

#if HIP_MOTOR_TYPE == MG_6012
void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId);
void encode_6012_motor_torque_control(uint32_t id, float torque_cmd);
#elif HIP_MOTOR_TYPE == DM_4340P
HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef decode_4340_motor_feedback(uint8_t *data, uint8_t bMotorId);
HAL_StatusTypeDef enable_DaMiao_motor(uint32_t id, uint8_t _enable, CAN_HandleTypeDef *hcan_ptr);
#endif
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

	if (hcan == &HIP_CAN)
	{
		switch (rx_header.StdId)
		{
#if HIP_MOTOR_TYPE == MG_6012
			case CAN_HIP1_RX_ID:
			{
				if (rx_data[0] == CAN_6012_TORQUE_FEEDBACK_ID)
				{
					bMotorId = CHASSIS_ID_HIP_1;
					decode_6012_motor_torque_feedback(rx_data, bMotorId);
					detect_hook(CHASSIS_HIP1_TOE);
				}
				break;
			}
			case CAN_HIP2_RX_ID:
			{
				if (rx_data[0] == CAN_6012_TORQUE_FEEDBACK_ID)
				{
					bMotorId = CHASSIS_ID_HIP_2;
					decode_6012_motor_torque_feedback(rx_data, bMotorId);
					detect_hook(CHASSIS_HIP2_TOE);
				}
				break;
			}
			case CAN_HIP3_RX_ID:
			{
				if (rx_data[0] == CAN_6012_TORQUE_FEEDBACK_ID)
				{
					bMotorId = CHASSIS_ID_HIP_3;
					decode_6012_motor_torque_feedback(rx_data, bMotorId);
					detect_hook(CHASSIS_HIP3_TOE);
				}
				break;
			}
			case CAN_HIP4_RX_ID:
			{
				if (rx_data[0] == CAN_6012_TORQUE_FEEDBACK_ID)
				{
					bMotorId = CHASSIS_ID_HIP_4;
					decode_6012_motor_torque_feedback(rx_data, bMotorId);
					detect_hook(CHASSIS_HIP4_TOE);
				}
				break;
			}
#elif HIP_MOTOR_TYPE == DM_4340P
			case CAN_HIP1_RX_ID:
			{
				bMotorId = CHASSIS_ID_HIP_1;
				if(decode_4340_motor_feedback(rx_data, bMotorId)==HAL_OK){
					detect_hook(CHASSIS_HIP1_TOE);
				}
				
				break;
			}
			case CAN_HIP2_RX_ID:
			{
				bMotorId = CHASSIS_ID_HIP_2;
				if(decode_4340_motor_feedback(rx_data, bMotorId)==HAL_OK){
					detect_hook(CHASSIS_HIP2_TOE);
				}
				
				break;
			}
			case CAN_HIP3_RX_ID:
			{
				bMotorId = CHASSIS_ID_HIP_3;
				if(decode_4340_motor_feedback(rx_data, bMotorId)==HAL_OK){
					detect_hook(CHASSIS_HIP3_TOE);
				}
				
				break;
			}
			case CAN_HIP4_RX_ID:
			{
				bMotorId = CHASSIS_ID_HIP_4;
				if(decode_4340_motor_feedback(rx_data, bMotorId)==HAL_OK){
					detect_hook(CHASSIS_HIP4_TOE);
				}
				
				break;
			}
#endif
			default:
			{
				break;
			}
		}
	}
	else if (hcan == &INTER_CTRL_CAN)
	{
		// Note: filter is enabled for CAN2, so be careful with the mask config
		switch (rx_header.StdId)
		{
			case CAN_CHASSIS_CONTROLLER_RX_ID:
			{
				chassis_move.fHipMotorEnabled = (memcmp(rx_data, abAllFF, sizeof(abAllFF)) != 0);
#if (HEADLESS_HIP_TEST == 0)
				if (chassis_move.fHipMotorEnabled)
				{
					chassis_move.target_alpha1 = (int16_t)((rx_data[0] << 8) | rx_data[1]) / angle_encoding_ratio;
					chassis_move.target_height = (uint16_t)((rx_data[2] << 8) | rx_data[3]) / meter_encoding_ratio;
					chassis_move.hip_motor_kp = ((uint16_t)((rx_data[4] << 8) | rx_data[5])) * 0.1f;
					chassis_move.target_height = fp32_constrain(chassis_move.target_height, CHASSIS_H_LOWER_LIMIT, CHASSIS_H_UPPER_LIMIT);

					// Compute workspace alpha limit from the single height command
					if (chassis_move.target_height >= CHASSIS_H_WORKSPACE_PEAK)
						chassis_move.alpha_upper_limit = (chassis_move.target_height - CHASSIS_H_UPPER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE2;
					else
						chassis_move.alpha_upper_limit = (chassis_move.target_height - CHASSIS_H_LOWER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE1;
					chassis_move.alpha_lower_limit = -chassis_move.alpha_upper_limit;

					chassis_move.target_alpha1 = fp32_constrain(chassis_move.target_alpha1, chassis_move.alpha_lower_limit, chassis_move.alpha_upper_limit);
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
#if ENABLE_HIP_MOTOR_POWER
	if ((chassis_move.fHipMotorEnabled == 0) || (fValidInput == 0) || (chassis_move.fHipDataIsValid == 0))
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
	// Broadcast msg would cause signal to be lost if the bus is too long, use individual msg instead
	// encode_6012_multi_motor_torque_control(torque1, torque2, torque3, torque4);

#if HIP_MOTOR_TYPE == MG_6012
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP1_RX_ID, hip_torque1);
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP2_RX_ID, hip_torque2);
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP3_RX_ID, hip_torque3);
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP4_RX_ID, hip_torque4);
#elif HIP_MOTOR_TYPE == DM_4340P
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP1_TX_ID, 0, 0, 0, 0, torque1, DM_4340, &HIP_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP2_TX_ID, 0, 0, 0, 0, torque2, DM_4340, &HIP_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP3_TX_ID, 0, 0, 0, 0, torque3, DM_4340, &HIP_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP4_TX_ID, 0, 0, 0, 0, torque4, DM_4340, &HIP_CAN);
#endif
	return fValidInput;
}


#if HIP_MOTOR_TYPE == MG_6012
void encode_6012_multi_motor_torque_control(float torque1, float torque2, float torque3, float torque4)
{
	can_tx_msg.StdId = CAN_HIP_MOTOR_MULTICMD_TX_ID;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 8;

#if ENABLE_HIP_MOTOR_POWER
	if (chassis_move.fHipMotorEnabled)
#endif
	{
		torque1 = 0;
		torque2 = 0;
		torque3 = 0;
		torque4 = 0;
	}

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

	HAL_CAN_AddTxMessage(&HIP_CAN, &can_tx_msg, can_tx_data, &send_mail_box);
}

void encode_6012_motor_torque_control(uint32_t id, float torque_cmd)
{
	can_tx_msg.StdId = id;
	can_tx_msg.ExtId = 0x00;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 8;

#if (ENABLE_HIP_MOTOR_POWER == 0)
	torque_cmd = 0;
#endif

	int16_t iqControl = torque_cmd * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	memset(can_tx_data, 0, sizeof(can_tx_data));
	can_tx_data[0] = CAN_6012_TORQUE_FEEDBACK_ID;
	can_tx_data[4] = *(uint8_t *)(&iqControl);
	can_tx_data[5] = *((uint8_t *)(&iqControl) + 1);

	HAL_CAN_AddTxMessage(&HIP_CAN, &can_tx_msg, can_tx_data, &send_mail_box);
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


#elif HIP_MOTOR_TYPE == DM_4340P

fp32 uint_to_fp32_motor(int x_int, fp32 x_min, fp32 x_max, int bits)
{
	/// converts unsigned int to fp32, given range and number of bits ///
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	return ((fp32)x_int) * span / ((fp32)((1 << bits) - 1)) + offset;
}

int fp32_to_uint_motor(fp32 x, fp32 x_min, fp32 x_max, int bits)
{
	/// Converts a fp32 to an unsigned int, given range and number of bits///
	fp32 span = x_max - x_min;
	fp32 offset = x_min;
	if (x >= x_max)
	{
		return ((1 << bits) - 1);
	}
	else if (x <= x_min)
	{
		return 0;
	}
	else
	{
		return (int)((x - offset) * ((fp32)((1 << bits) - 1)) / span);
	}
}

HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr)
{
	uint32_t send_mail_box;
	can_tx_msg.StdId = id;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 0x08;

#if DISABLE_ARM_MOTOR_POWER
	_pos = 0;
	_vel = 0;
	_KP = 0;
	_KD = 0;
	_torq = 0;
#endif

	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = fp32_to_uint_motor(_pos, MIT_CONTROL_P_MIN[motor_type], MIT_CONTROL_P_MAX[motor_type], 16);
	vel_tmp = fp32_to_uint_motor(_vel, MIT_CONTROL_V_MIN[motor_type], MIT_CONTROL_V_MAX[motor_type], 12);
	kp_tmp = fp32_to_uint_motor(_KP, MIT_CONTROL_KP_MIN[motor_type], MIT_CONTROL_KP_MAX[motor_type], 12);
	kd_tmp = fp32_to_uint_motor(_KD, MIT_CONTROL_KD_MIN[motor_type], MIT_CONTROL_KD_MAX[motor_type], 12);
	tor_tmp = fp32_to_uint_motor(_torq, MIT_CONTROL_T_MIN[motor_type], MIT_CONTROL_T_MAX[motor_type], 12);

	can_tx_data[0] = (pos_tmp >> 8);
	can_tx_data[1] = pos_tmp;
	can_tx_data[2] = (vel_tmp >> 4);
	can_tx_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	can_tx_data[4] = kp_tmp;
	can_tx_data[5] = (kd_tmp >> 4);
	can_tx_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	can_tx_data[7] = tor_tmp;

	return HAL_CAN_AddTxMessage(hcan_ptr, &can_tx_msg, can_tx_data, &send_mail_box);
}

HAL_StatusTypeDef decode_4340_motor_feedback(uint8_t *data, uint8_t bMotorId)
{
	HAL_StatusTypeDef ret_value = HAL_ERROR;
	// Note: error_id = 0， 1 means motor power is disabled/enabled
	uint8_t error_id = data[0] >> 4;
	if ((error_id != 0) && (error_id != 1))
	{
		ret_value = HAL_ERROR;
	}
	else
	{
		uint16_t p_int = (data[1] << 8) | data[2];		   // rad (+-4*pi)
		uint16_t v_int = (data[3] << 4) | (data[4] >> 4);  // rad/s
		uint16_t t_int = ((data[4] & 0xF) << 8) | data[5];
		motor_info[bMotorId].feedback_abs_angle = uint_to_fp32_motor(p_int, MIT_CONTROL_P_MIN[DM_4340], MIT_CONTROL_P_MAX[DM_4340], 16);
		motor_info[bMotorId].feedback_abs_ecd_fp32 = loop_fp32_constrain(motor_info[bMotorId].feedback_abs_angle, 0, 2 * PI) * DM4340_MOTOR_RAD_TO_ECD; //no actual ecd reading used 
		motor_info[bMotorId].velocity = uint_to_fp32_motor(v_int, MIT_CONTROL_V_MIN[DM_4340], MIT_CONTROL_V_MAX[DM_4340], 12);
		motor_info[bMotorId].torque = uint_to_fp32_motor(t_int, MIT_CONTROL_T_MIN[DM_4340], MIT_CONTROL_T_MAX[DM_4340], 12);
		motor_info[bMotorId].temperate = data[6];

		ret_value = HAL_OK;
	}
	chassis_move.fHipDataIsValid = 1;
	return ret_value;
}

HAL_StatusTypeDef enable_DaMiao_motor(uint32_t id, uint8_t _enable, CAN_HandleTypeDef *hcan_ptr)
{
	uint32_t send_mail_box;
	can_tx_msg.StdId = id;
	can_tx_msg.IDE = CAN_ID_STD;
	can_tx_msg.RTR = CAN_RTR_DATA;
	can_tx_msg.DLC = 0x08;

	memset(can_tx_data, 0xFF, sizeof(can_tx_data));

	if (_enable)
	{
		can_tx_data[7] = 0xFC;
	}
	else
	{
		// disable
		can_tx_data[7] = 0xFD;
	}
	HAL_StatusTypeDef hal_status = HAL_CAN_AddTxMessage(hcan_ptr, &can_tx_msg, can_tx_data, &send_mail_box);
	memset(can_tx_data, 0, sizeof(can_tx_data));
	return hal_status;
}

void enable_all_DaMiao_motors(uint8_t _enable)
{
			enable_DaMiao_motor(CAN_HIP1_TX_ID, _enable, &HIP_CAN);
        osDelay(1);
			enable_DaMiao_motor(CAN_HIP2_TX_ID, _enable, &HIP_CAN);
        osDelay(1);
			enable_DaMiao_motor(CAN_HIP3_TX_ID, _enable, &HIP_CAN);
        osDelay(1);
			enable_DaMiao_motor(CAN_HIP4_TX_ID, _enable, &HIP_CAN);
}

#endif


/**
 * @brief CAN1 and CAN2 tx calls are interlaced so that we can reduce total delay time without affecting the performance
 * @TODO: make helper functions to load CAN msg to the buffer and freely organize their sending order
 */
void CAN_cmd_wrapper(void)
{
	/*********** CAN_cmd_hip_motors ***********/
	float hip_torque1 = motor_info[CHASSIS_ID_HIP_1].set_torque;
	float hip_torque2 = motor_info[CHASSIS_ID_HIP_2].set_torque;
	float hip_torque3 = motor_info[CHASSIS_ID_HIP_3].set_torque;
	float hip_torque4 = motor_info[CHASSIS_ID_HIP_4].set_torque;

	uint8_t fValidInput = (((hip_torque1 != hip_torque1) || (hip_torque2 != hip_torque2) || (hip_torque3 != hip_torque3) || (hip_torque4 != hip_torque4)) == 0);
#if ENABLE_HIP_MOTOR_POWER
	// @TODO: validate this line
	if ((chassis_move.fHipMotorEnabled == 0) || (fValidInput == 0) || (chassis_move.fHipDataIsValid == 0))
	// if ((chassis_move.fHipMotorEnabled == 0) || (fValidInput == 0))
#endif
	{
		hip_torque1 = 0;
		hip_torque2 = 0;
		hip_torque3 = 0;
		hip_torque4 = 0;
	}

#if REVERSE_3_HIP_MOTOR_DIRECTION
	hip_torque3 *= -1.0f;
#endif

#if REVERSE_2_HIP_MOTOR_DIRECTION
	hip_torque2 *= -1.0f;
#endif

#if REVERSE_4_HIP_MOTOR_DIRECTION
	hip_torque4 *= -1.0f;
#endif

#if REVERSE_1_HIP_MOTOR_DIRECTION
	hip_torque1 *= -1.0f;
#endif

	// 6012 motor as hip
	// Broadcast msg would cause signal to be lost if the bus is too long, use individual msg instead
	// encode_6012_multi_motor_torque_control(hip_torque1, hip_torque2, hip_torque3, hip_torque4);
	/*********** CAN_cmd_hip_motors ***********/

#if HIP_MOTOR_TYPE == MG_6012
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP1_RX_ID, hip_torque1);
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP2_RX_ID, hip_torque2);
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP3_RX_ID, hip_torque3);
	osDelay(1);
	encode_6012_motor_torque_control(CAN_HIP4_RX_ID, hip_torque4);
#elif HIP_MOTOR_TYPE == DM_4340P
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP1_TX_ID, chassis_move.hip_cmd[0].pos, chassis_move.hip_cmd[0].vel, chassis_move.hip_motor_kp, chassis_move.hip_cmd[0].KD, chassis_move.hip_cmd[0].torq, DM_4340, &HIP_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP2_TX_ID, chassis_move.hip_cmd[1].pos, chassis_move.hip_cmd[1].vel, chassis_move.hip_motor_kp, chassis_move.hip_cmd[1].KD, chassis_move.hip_cmd[1].torq, DM_4340, &HIP_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP3_TX_ID, chassis_move.hip_cmd[2].pos, chassis_move.hip_cmd[2].vel, chassis_move.hip_motor_kp, chassis_move.hip_cmd[2].KD, chassis_move.hip_cmd[2].torq, DM_4340, &HIP_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_HIP4_TX_ID, chassis_move.hip_cmd[3].pos, chassis_move.hip_cmd[3].vel, chassis_move.hip_motor_kp, chassis_move.hip_cmd[3].KD, chassis_move.hip_cmd[3].torq, DM_4340, &HIP_CAN);
#endif

}

