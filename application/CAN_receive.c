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
#include "detect_task.h"
#include "main.h"
#include "pid.h"
#include "robot_arm_task.h"
#include "user_lib.h"

#define DISABLE_ARM_MOTOR_POWER 0

#define MOTOR_6012_GEAR_RATIO 36.0f
#define MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO 0.225146199f
#define MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO 0.212f
#define MOTOR_6012_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 33.0f * 2048.0f)
#define MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_6012_GEAR_RATIO / MOTOR_6012_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_6012_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 32.0f * 2000.0f)

#define MOTOR_4010_GEAR_RATIO 36.0f
#define MOTOR_4010_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO 0.225146199f // @TODO
#define MOTOR_4010_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO 0.212f      // @TODO
#define MOTOR_4010_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_4010_GEAR_RATIO / MOTOR_4010_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_4010_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 33.0f * 2048.0f)
#define MOTOR_4010_BROADCAST_CMD_TO_TORQUE_RATIO (1.0f / MOTOR_4010_GEAR_RATIO / MOTOR_4010_INPUT_TORQUE_TO_MAIN_CURRENT_RATIO / MOTOR_4010_MAIN_CURRENT_TO_ROTOR_CURRENT_RATIO / 32.0f * 2000.0f)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
// motor data read
#define get_rm_motor_measure(ptr, data)                                \
	{                                                                  \
		(ptr)->last_ecd = (ptr)->ecd;                                  \
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
		(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
		(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
		(ptr)->temperature = (data)[6];                                \
	}

motor_measure_t motor_measure[JOINT_ID_LAST];

static CAN_TxHeaderTypeDef can_tx_message;
static uint8_t can_send_data[8];
uint32_t send_mail_box;

const float MIT_CONTROL_P_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {12.5f, 12.5f, 12.5f};
const float MIT_CONTROL_P_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-12.5f, -12.5f, -12.5f};
const float MIT_CONTROL_V_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {25.0f, 45.0f, 30.0f};
const float MIT_CONTROL_V_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-25.0f, -45.0f, -30.0f};
const float MIT_CONTROL_T_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {20.0f, 24.0f, 10.0f};
const float MIT_CONTROL_T_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-20.0f, -24.0f, -10.0f};
const float MIT_CONTROL_KP_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {500.0f, 500.0f, 500.0f};
const float MIT_CONTROL_KP_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f};
const float MIT_CONTROL_KD_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {5.0f, 5.0f, 5.0f};
const float MIT_CONTROL_KD_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f};

// @TODO: measure 6020 offset angle
const uint16_t joint_6_6020_offset_ecd = 0;

HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq, uint8_t blocking_call, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef encode_6012_motor_position_control(uint32_t id, fp32 maxSpeed_rpm, fp32 angleControl_rad, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef encode_4010_motor_position_control(uint32_t id, fp32 maxSpeed_rpm, fp32 angleControl_rad, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef encode_6012_motor_torque_control(uint32_t id, float torque_cmd, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef encode_4010_motor_torque_control(uint32_t id, float torque_cmd, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef encode_6020_motor_current_control(int16_t current_ch_5, int16_t current_ch_6, int16_t current_ch_7, CAN_HandleTypeDef *hcan_ptr, uint8_t blocking_call);

void decode_chassis_controller_rx(uint8_t *data, uint32_t id);
void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId);
void decode_4010_motor_torque_feedback(uint8_t *data, uint8_t bMotorId);
HAL_StatusTypeDef decode_4310_motor_feedback(uint8_t *data, uint8_t *bMotorIdPtr);

HAL_StatusTypeDef enable_DaMiao_motor(uint32_t id, uint8_t _enable, CAN_HandleTypeDef *hcan_ptr, uint8_t blocking_call);
HAL_StatusTypeDef soft_disable_Ktech_motor(uint32_t id, CAN_HandleTypeDef *hcan_ptr, uint8_t blocking_call);
HAL_StatusTypeDef blocking_can_send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data);
float uint_to_float_motor(int x_int, float x_min, float x_max, int bits);
int float_to_uint_motor(float x, float x_min, float x_max, int bits);
uint8_t arm_joints_cmd_position(float joint_angle_target_ptr[7], fp32 dt);
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
HAL_StatusTypeDef Send_CAN_Cmd(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data, uint8_t blocking_call);

uint8_t canRxIdToMotorArrayId(can_msg_id_e _SINGLE_CAN_ID)
{
	switch (_SINGLE_CAN_ID)
	{
		case CAN_JOINT_MOTOR_1_6012_RX_ID:
		{
			return JOINT_ID_1_6012;
		}
		case CAN_JOINT_MOTOR_2_4010_RX_ID:
		{
			return JOINT_ID_2_4010;
		}
		case CAN_JOINT_MOTOR_6_6020_RX_ID:
		{
			return JOINT_ID_6_6020;
		}
		case CAN_DAMIAO_RX_ID:
		default:
		{
			return 0xFF;
		}
	}
}

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
	uint8_t bMotorId = 0xFF;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	bMotorId = canRxIdToMotorArrayId((can_msg_id_e)rx_header.StdId);
	switch (rx_header.StdId)
	{
		case CAN_DAMIAO_RX_ID:
		{
			decode_4310_motor_feedback(rx_data, &bMotorId);
			break;
		}
		case CAN_JOINT_MOTOR_1_6012_RX_ID:
		{
			if ((rx_data[0] == CAN_KTECH_MULTIANGLE_2_ID) || (rx_data[0] == CAN_KTECH_TORQUE_ID))
			{
				decode_6012_motor_torque_feedback(rx_data, bMotorId);
			}
			else if (rx_data[0] == CAN_KTECH_TEMP_DISABLE_MOTOR_ID)
			{
				; // do nothing except detect_hook
			}
			break;
		}
		case CAN_JOINT_MOTOR_2_4010_RX_ID:
		{
			if ((rx_data[0] == CAN_KTECH_MULTIANGLE_2_ID) || (rx_data[0] == CAN_KTECH_TORQUE_ID))
			{
				decode_4010_motor_torque_feedback(rx_data, bMotorId);
			}
			else if (rx_data[0] == CAN_KTECH_TEMP_DISABLE_MOTOR_ID)
			{
				; // do nothing except detect_hook
			}
			break;
		}
		case CAN_JOINT_MOTOR_6_6020_RX_ID:
		{
			get_rm_motor_measure(&motor_measure[bMotorId], rx_data);
			break;
		}
		case CAN_INTER_BOARD_INDIVIDUAL_MOTOR_1_RX_ID:
		case CAN_INTER_BOARD_INDIVIDUAL_MOTOR_2_RX_ID:
		{
			decode_chassis_controller_rx(rx_data, rx_header.StdId);
			detect_hook(CHASSIS_CONTROLLER_TOE);
			break;
		}
		case CAN_INTER_BOARD_POSITION_RX_ID:
		{
			// @TODO: end-effector control
			detect_hook(CHASSIS_CONTROLLER_TOE);
			break;
		}
		case CAN_INTER_BOARD_ORIENTATION_RX_ID:
		{
			// @TODO: end-effector control
			detect_hook(CHASSIS_CONTROLLER_TOE);
			break;
		}
		default:
		{
			break;
		}
	}

	if (bMotorId < JOINT_ID_LAST)
	{
		detect_hook(bMotorId + JOINT_0_TOE);
	}
}

void decode_chassis_controller_rx(uint8_t *data, uint32_t id)
{
	uint8_t fIsSpecialCmd = 0;
	uint8_t specialCmd = data[0];
	if ((specialCmd == 0x00) || (specialCmd == 0xFF))
	{
		fIsSpecialCmd = 1;
		for (uint8_t i = 1; i < 8; i++)
		{
			if (specialCmd != data[i])
			{
				fIsSpecialCmd = 0;
				break;
			}
		}
	}

	if (fIsSpecialCmd)
	{
		switch (specialCmd)
		{
			case 0x00:
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
				robot_arm.fHoming = 0;
				break;
			}
			case 0xFF:
			{
				robot_arm.fHoming = 1;
				robot_arm_return_to_center(0, JOINT_ID_LAST - 1);
				break;
			}
		}
	}
	else
	{
		robot_arm.fHoming = 0;
		switch (id)
		{
			case CAN_INTER_BOARD_INDIVIDUAL_MOTOR_1_RX_ID:
			{
				robot_arm.joint_angle_target[0] = (int16_t)((data[1] << 8) | data[0]) / RAD_TO_INT16_SCALE;
				robot_arm.joint_angle_target[1] = (int16_t)((data[3] << 8) | data[2]) / RAD_TO_INT16_SCALE;
				robot_arm.joint_angle_target[2] = (int16_t)((data[5] << 8) | data[4]) / RAD_TO_INT16_SCALE;
				robot_arm.joint_angle_target[3] = (int16_t)((data[7] << 8) | data[6]) / RAD_TO_INT16_SCALE;
				break;
			}
			case CAN_INTER_BOARD_INDIVIDUAL_MOTOR_2_RX_ID:
			default:
			{
				robot_arm.joint_angle_target[4] = (int16_t)((data[1] << 8) | data[0]) / RAD_TO_INT16_SCALE;
				robot_arm.joint_angle_target[5] = (int16_t)((data[3] << 8) | data[2]) / RAD_TO_INT16_SCALE;
				robot_arm.joint_angle_target[6] = (int16_t)((data[5] << 8) | data[4]) / RAD_TO_INT16_SCALE;
				break;
			}
		}
	}
}

float uint_to_float_motor(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint_motor(float x, float x_min, float x_max, int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits///
	float span = x_max - x_min;
	float offset = x_min;
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
		return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
	}
}

void decode_6012_motor_torque_feedback(uint8_t *data, uint8_t bMotorId)
{
	int16_t iq_int = (data[3] << 8) | data[2];    // A
	int16_t v_int = (data[5] << 8) | data[4];     // deg/s
	uint16_t p_uint = ((data[7] << 8) | data[6]); // 16bit abs encoder

	motor_measure[bMotorId].torque = ((fp32)iq_int) / MOTOR_6012_CMD_TO_TORQUE_RATIO;
	motor_measure[bMotorId].velocity = ((fp32)v_int) / 36.0f / 180.0f * PI;
	motor_measure[bMotorId].output_angle = ((fp32)p_uint) / (1 << 16) * 2.0f * PI;
	motor_measure[bMotorId].temperature = data[1];
}

void decode_4010_motor_torque_feedback(uint8_t *data, uint8_t bMotorId)
{
	int16_t power_int = (data[3] << 8) | data[2]; // A
	int16_t v_int = (data[5] << 8) | data[4];     // deg/s
	uint16_t p_uint = ((data[7] << 8) | data[6]); // 16bit abs encoder

	motor_measure[bMotorId].torque = ((fp32)power_int) / MOTOR_4010_CMD_TO_TORQUE_RATIO;
	motor_measure[bMotorId].velocity = ((fp32)v_int) / 36.0f / 180.0f * PI;
	motor_measure[bMotorId].output_angle = ((fp32)p_uint) / (1 << 16) * 2.0f * PI;
	motor_measure[bMotorId].temperature = data[1];
}

HAL_StatusTypeDef decode_4310_motor_feedback(uint8_t *data, uint8_t *bMotorIdPtr)
{
	uint8_t error_id = data[0] >> 4;
	if (error_id != 0)
	{
		return HAL_ERROR;
	}
	else
	{
		uint16_t p_int = (data[1] << 8) | data[2];         // rad
		uint16_t v_int = (data[3] << 4) | (data[4] >> 4);  // rad/s
		uint16_t t_int = ((data[4] & 0xF) << 8) | data[5]; // Nm

		*bMotorIdPtr = (data[0] & 0xF) - 1 + JOINT_ID_0_4310;
		motor_measure[*bMotorIdPtr].output_angle = uint_to_float_motor(p_int, MIT_CONTROL_P_MIN[DM_4310], MIT_CONTROL_P_MAX[DM_4310], 16);
		motor_measure[*bMotorIdPtr].velocity = uint_to_float_motor(v_int, MIT_CONTROL_V_MIN[DM_4310], MIT_CONTROL_V_MAX[DM_4310], 12);
		motor_measure[*bMotorIdPtr].torque = uint_to_float_motor(t_int, MIT_CONTROL_T_MIN[DM_4310], MIT_CONTROL_T_MAX[DM_4310], 12);
		motor_measure[*bMotorIdPtr].temperature = data[6];
	}
	return HAL_OK;
}

uint8_t arm_joints_cmd_position(float joint_angle_target_ptr[7], fp32 dt)
{
	uint8_t fValidInput = 1;
	uint8_t pos_index;
	for (pos_index = 0; pos_index < 7; pos_index++)
	{
		if ((joint_angle_target_ptr[pos_index] != joint_angle_target_ptr[pos_index]) || (joint_angle_target_ptr[pos_index] > joint_angle_max[pos_index]) || (joint_angle_target_ptr[pos_index] < joint_angle_min[pos_index]))
		{
			joint_angle_target_ptr[pos_index] = joint_angle_home[pos_index];
			fValidInput = 0;
		}
	}

#if (DISABLE_ARM_MOTOR_POWER == 0)
	uint8_t blocking_call = 1;
	// According to test, CAN_JOINT_MOTOR_0_4310_TX_ID starts diverging with kd>1.5
	encode_MIT_motor_control(CAN_JOINT_MOTOR_0_4310_TX_ID, joint_angle_target_ptr[0], 0, 10, 1.5f, 0, blocking_call, DM_4310, &LOWER_MOTORS_CAN);
	encode_6012_motor_position_control(CAN_JOINT_MOTOR_1_6012_TX_ID, 5.0f, joint_angle_target_ptr[1], blocking_call, &LOWER_MOTORS_CAN);
	osDelay(1);
	encode_4010_motor_position_control(CAN_JOINT_MOTOR_2_4010_TX_ID, 10.0f, joint_angle_target_ptr[2], blocking_call, &LOWER_MOTORS_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_JOINT_MOTOR_3_4310_TX_ID, joint_angle_target_ptr[3], 0, 10, 0.5f, 0, blocking_call, DM_4310, &UPPER_MOTORS_CAN);
	encode_MIT_motor_control(CAN_JOINT_MOTOR_4_4310_TX_ID, joint_angle_target_ptr[4], 0, 5, 0.5f, 0, blocking_call, DM_4310, &UPPER_MOTORS_CAN);
	encode_MIT_motor_control(CAN_JOINT_MOTOR_5_4310_TX_ID, joint_angle_target_ptr[5], 0, 5, 0.5f, 0, blocking_call, DM_4310, &UPPER_MOTORS_CAN);

	// 6020 calculation
	fp32 joint_6_6020_speed_set = PID_calc(&robot_arm.joint_6_6020_angle_pid, motor_measure[JOINT_ID_6_6020].output_angle, joint_angle_target_ptr[6], dt);
	fp32 joint_6_6020_current_set = (int16_t)PID_calc(&robot_arm.joint_6_6020_speed_pid, motor_measure[JOINT_ID_6_6020].speed_rpm * 2 * PI / 60.0f, joint_6_6020_speed_set, dt);
	encode_6020_motor_current_control(0, 0, joint_6_6020_current_set, &LOWER_MOTORS_CAN, blocking_call);
#endif

	return fValidInput;
}

void arm_joints_cmd_torque(float joint_torques[7])
{
	uint8_t blocking_call = 1;
	encode_MIT_motor_control(CAN_JOINT_MOTOR_0_4310_TX_ID, 0, 0, 0, 0, joint_torques[0], blocking_call, DM_4310, &LOWER_MOTORS_CAN);
	encode_6012_motor_torque_control(CAN_JOINT_MOTOR_1_6012_TX_ID, joint_torques[1], blocking_call, &LOWER_MOTORS_CAN);
	osDelay(1);
	// Warning: 4010 current to torque ratio is unknown
	encode_4010_motor_torque_control(CAN_JOINT_MOTOR_2_4010_TX_ID, joint_torques[2], blocking_call, &LOWER_MOTORS_CAN);
	osDelay(1);
	encode_MIT_motor_control(CAN_JOINT_MOTOR_3_4310_TX_ID, 0, 0, 0, 0, joint_torques[3], blocking_call, DM_4310, &UPPER_MOTORS_CAN);
	encode_MIT_motor_control(CAN_JOINT_MOTOR_4_4310_TX_ID, 0, 0, 0, 0, joint_torques[4], blocking_call, DM_4310, &UPPER_MOTORS_CAN);
	encode_MIT_motor_control(CAN_JOINT_MOTOR_5_4310_TX_ID, 0, 0, 0, 0, joint_torques[5], blocking_call, DM_4310, &UPPER_MOTORS_CAN);
	// Warning: 6020 current to torque ratio is unknown
	encode_6020_motor_current_control(0, 0, joint_torques[6], &LOWER_MOTORS_CAN, blocking_call);
}

HAL_StatusTypeDef enable_DaMiao_motor(uint32_t id, uint8_t _enable, CAN_HandleTypeDef *hcan_ptr, uint8_t blocking_call)
{
	can_tx_message.StdId = id;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;

	memset(can_send_data, 0xFF, sizeof(can_send_data));

	if (_enable)
	{
		can_send_data[7] = 0xFC;
	}
	else
	{
		// disable
		can_send_data[7] = 0xFD;
	}
	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef encode_6020_motor_current_control(int16_t current_ch_5, int16_t current_ch_6, int16_t current_ch_7, CAN_HandleTypeDef *hcan_ptr, uint8_t blocking_call)
{
	can_tx_message.StdId = CAN_MOTOR_6020_TX_ID;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;
	can_send_data[0] = (current_ch_5 >> 8);
	can_send_data[1] = current_ch_5;
	can_send_data[2] = (current_ch_6 >> 8);
	can_send_data[3] = current_ch_6;
	can_send_data[4] = (current_ch_7 >> 8);
	can_send_data[5] = current_ch_7;
	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

void enable_all_motor_control(uint8_t _enable)
{
	uint8_t blocking_call = 1;
	enable_DaMiao_motor(CAN_JOINT_MOTOR_0_4310_TX_ID, _enable, &LOWER_MOTORS_CAN, blocking_call);

	if (_enable == 0)
	{
		soft_disable_Ktech_motor(CAN_JOINT_MOTOR_1_6012_TX_ID, &LOWER_MOTORS_CAN, blocking_call);
		soft_disable_Ktech_motor(CAN_JOINT_MOTOR_2_4010_TX_ID, &LOWER_MOTORS_CAN, blocking_call);

		encode_6020_motor_current_control(0, 0, 0, &LOWER_MOTORS_CAN, blocking_call);
		PID_clear(&robot_arm.joint_6_6020_angle_pid);
		PID_clear(&robot_arm.joint_6_6020_speed_pid);
	}

	enable_DaMiao_motor(CAN_JOINT_MOTOR_3_4310_TX_ID, _enable, &UPPER_MOTORS_CAN, blocking_call);
	enable_DaMiao_motor(CAN_JOINT_MOTOR_4_4310_TX_ID, _enable, &UPPER_MOTORS_CAN, blocking_call);
	enable_DaMiao_motor(CAN_JOINT_MOTOR_5_4310_TX_ID, _enable, &UPPER_MOTORS_CAN, blocking_call);
}

HAL_StatusTypeDef soft_disable_Ktech_motor(uint32_t id, CAN_HandleTypeDef *hcan_ptr, uint8_t blocking_call)
{
	can_tx_message.StdId = id;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;

	memset(can_send_data, 0, sizeof(can_send_data));
	can_send_data[0] = CAN_KTECH_TEMP_DISABLE_MOTOR_ID;

	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef encode_4010_motor_position_control(uint32_t id, fp32 maxSpeed_rpm, fp32 angleControl_rad, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr)
{
	uint16_t maxSpeed_dps = fp32_constrain(maxSpeed_rpm, -20.0f, 20.0f) * 360.0f / 60.0f * MOTOR_4010_GEAR_RATIO;
	int32_t angle_deg = fp32_constrain(angleControl_rad, -PI, PI) / PI * 180.0f * 100.0f * MOTOR_4010_GEAR_RATIO;
	can_tx_message.StdId = id;
	can_tx_message.ExtId = 0x00;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.DLC = 8;

	can_send_data[0] = 0xA4;
	can_send_data[1] = 0x00;
	can_send_data[2] = *((uint8_t *)&maxSpeed_dps + 0);
	can_send_data[3] = *((uint8_t *)&maxSpeed_dps + 1);
	can_send_data[4] = *((uint8_t *)&angle_deg + 0);
	can_send_data[5] = *((uint8_t *)&angle_deg + 1);
	can_send_data[6] = *((uint8_t *)&angle_deg + 2);
	can_send_data[7] = *((uint8_t *)&angle_deg + 3);

	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef encode_6012_motor_position_control(uint32_t id, fp32 maxSpeed_rpm, fp32 angleControl_rad, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr)
{
	uint16_t maxSpeed_dps = fp32_constrain(maxSpeed_rpm, -20.0f, 20.0f) * 360.0f / 60.0f * MOTOR_6012_GEAR_RATIO;
	int32_t angle_deg = fp32_constrain(angleControl_rad, -PI, PI) / PI * 180.0f * 100.0f * MOTOR_6012_GEAR_RATIO;
	can_tx_message.StdId = id;
	can_tx_message.ExtId = 0x00;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.DLC = 8;

	can_send_data[0] = 0xA4;
	can_send_data[1] = 0x00;
	can_send_data[2] = *((uint8_t *)&maxSpeed_dps + 0);
	can_send_data[3] = *((uint8_t *)&maxSpeed_dps + 1);
	can_send_data[4] = *((uint8_t *)&angle_deg + 0);
	can_send_data[5] = *((uint8_t *)&angle_deg + 1);
	can_send_data[6] = *((uint8_t *)&angle_deg + 2);
	can_send_data[7] = *((uint8_t *)&angle_deg + 3);

	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef encode_6012_motor_torque_control(uint32_t id, float torque_cmd, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr)
{
	can_tx_message.StdId = id;
	can_tx_message.ExtId = 0x00;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 8;

	int16_t iqControl = torque_cmd * MOTOR_6012_BROADCAST_CMD_TO_TORQUE_RATIO;
	memset(can_send_data, 0, sizeof(can_send_data));
	can_send_data[0] = CAN_KTECH_TORQUE_ID;
	can_send_data[4] = *(uint8_t *)(&iqControl);
	can_send_data[5] = *((uint8_t *)(&iqControl) + 1);

	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef encode_4010_motor_torque_control(uint32_t id, float torque_cmd, uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr)
{
	can_tx_message.StdId = id;
	can_tx_message.ExtId = 0x00;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 8;

	int16_t iqControl = torque_cmd * MOTOR_4010_BROADCAST_CMD_TO_TORQUE_RATIO;
	memset(can_send_data, 0, sizeof(can_send_data));
	can_send_data[0] = CAN_KTECH_TORQUE_ID;
	can_send_data[4] = *(uint8_t *)(&iqControl);
	can_send_data[5] = *((uint8_t *)(&iqControl) + 1);

	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq, uint8_t blocking_call, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr)
{
	can_tx_message.StdId = id;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 0x08;

	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = float_to_uint_motor(_pos, MIT_CONTROL_P_MIN[motor_type], MIT_CONTROL_P_MAX[motor_type], 16);
	vel_tmp = float_to_uint_motor(_vel, MIT_CONTROL_V_MIN[motor_type], MIT_CONTROL_V_MAX[motor_type], 12);
	kp_tmp = float_to_uint_motor(_KP, MIT_CONTROL_KP_MIN[motor_type], MIT_CONTROL_KP_MAX[motor_type], 12);
	kd_tmp = float_to_uint_motor(_KD, MIT_CONTROL_KD_MIN[motor_type], MIT_CONTROL_KD_MAX[motor_type], 12);
	tor_tmp = float_to_uint_motor(_torq, MIT_CONTROL_T_MIN[motor_type], MIT_CONTROL_T_MAX[motor_type], 12);

	can_send_data[0] = (pos_tmp >> 8);
	can_send_data[1] = pos_tmp;
	can_send_data[2] = (vel_tmp >> 4);
	can_send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	can_send_data[4] = kp_tmp;
	can_send_data[5] = (kd_tmp >> 4);
	can_send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	can_send_data[7] = tor_tmp;

	return Send_CAN_Cmd(hcan_ptr, &can_tx_message, can_send_data, blocking_call);
}

HAL_StatusTypeDef Send_CAN_Cmd(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data, uint8_t blocking_call)
{
	if (blocking_call)
	{
		return blocking_can_send(hcan, tx_header, tx_data);
	}
	else
	{
		return HAL_CAN_AddTxMessage(hcan, tx_header, tx_data, &send_mail_box);
	}
}

HAL_StatusTypeDef blocking_can_send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data)
{
	HAL_StatusTypeDef CAN_status = HAL_TIMEOUT;
	uint16_t try_cnt = 0;
	const uint16_t retry_delay_ms = 1;
	const uint16_t retry_timeout_ms = 5000;
	while (1)
	{
		if ((hcan->State == HAL_CAN_STATE_READY) || (hcan->State == HAL_CAN_STATE_LISTENING))
		{
			CAN_status = HAL_CAN_AddTxMessage(hcan, tx_header, tx_data, &send_mail_box);
		}

		if (CAN_status == HAL_OK)
		{
			break;
		}
		else if (try_cnt > retry_timeout_ms / retry_delay_ms)
		{
			CAN_status = HAL_TIMEOUT;
			break;
		}
		try_cnt++;
		HAL_Delay(retry_delay_ms);
	}
	return CAN_status;
}

fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
	int32_t relative_ecd = ecd - offset_ecd;
	if (relative_ecd > HALF_ECD_RANGE)
	{
		relative_ecd -= ECD_RANGE;
	}
	else if (relative_ecd < -HALF_ECD_RANGE)
	{
		relative_ecd += ECD_RANGE;
	}

	return relative_ecd * MOTOR_ECD_TO_RAD;
}

void update_joint_6_6020_angle(void)
{
	motor_measure[JOINT_ID_6_6020].output_angle = motor_ecd_to_angle_change(motor_measure[JOINT_ID_6_6020].ecd, joint_6_6020_offset_ecd);
}
