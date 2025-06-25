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
#include "cv_usart_task.h"
#include "shoot.h"
#include "string.h"

// Warning: for safety, PLEASE ALWAYS keep those default values as 0 when you commit
// Warning: because #if directive will assume the expression as 0 even if the macro is not defined, positive logic, for example, ENABLE_MOTOR_POWER, is safer that if and only if it's defined and set to 1 that the power is enabled
#define ENABLE_YAW_MOTOR_POWER 1
#define ENABLE_PITCH_MOTOR_POWER 1
#define ENABLE_TRIGGER_MOTOR_POWER 1
#define ENABLE_FRICTION_1_MOTOR_POWER 1
#define ENABLE_FRICTION_2_MOTOR_POWER 1

#define BULLET_SPEED_ECD_MAX 40.0f
#define BULLET_SPEED_DECODE_RATIO (BULLET_SPEED_ECD_MAX / 255.0f)

#define HEAT_LIMIT_ECD_MAX 400.0f
#define SHOOT_HEAT_DECODE_RATIO (HEAT_LIMIT_ECD_MAX / 255.0f)

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

const fp32 MIT_CONTROL_P_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {12.5f, 12.5f, 4.0f*PI};  //value needs to match to which in motor setting software
const fp32 MIT_CONTROL_P_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-12.5f, -12.5f, -4.0f*PI};
const fp32 MIT_CONTROL_V_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {25.0f, 45.0f, 30.0f};
const fp32 MIT_CONTROL_V_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-25.0f, -45.0f, -30.0f};
const fp32 MIT_CONTROL_T_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {20.0f, 24.0f, 10.0f};
const fp32 MIT_CONTROL_T_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-20.0f, -24.0f, -10.0f};
const fp32 MIT_CONTROL_KP_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {500.0f, 500.0f, 500.0f};
const fp32 MIT_CONTROL_KP_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f};
const fp32 MIT_CONTROL_KD_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {5.0f, 5.0f, 5.0f};
const fp32 MIT_CONTROL_KD_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f};

void decode_lower_head_data(uint8_t *data);
fp32 uint_to_fp32_motor(int x_int, fp32 x_min, fp32 x_max, int bits);
int fp32_to_uint_motor(fp32 x, fp32 x_min, fp32 x_max, int bits);
HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef decode_4310_motor_feedback(uint8_t *data, uint8_t bMotorId);

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
#if PITCH_IS_4310
			case CAN_PIT_MOTOR_RX_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH;
				if (decode_4310_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				}
				break;
			}
#else
			case CAN_PIT_MOTOR_TX_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH;
				get_motor_measure(&motor_chassis[bMotorId], rx_data);
				detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				break;
			}
#endif

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
    shoot_control.heat_limit = (fp32)data[0] * SHOOT_HEAT_DECODE_RATIO;
    shoot_control.heat = (fp32)data[1] * SHOOT_HEAT_DECODE_RATIO;
    shoot_control.bullet_init_speed = (fp32)data[2] * BULLET_SPEED_DECODE_RATIO;

    uint16_t projectile_allowance = (data[3] << 8) | data[4];
	uint16_t gold_coins = (data[5] << 8) | data[6];
	uint8_t team_color = ((data[7] & (1 << 7)) != 0);
	// 4 represents game start
    uint8_t game_progress = 4;
	uint16_t current_HP = 100;
	uint16_t stage_remain_time = 100;
	CvCmder_set_ref_status(current_HP, team_color, stage_remain_time, game_progress, projectile_allowance, gold_coins, shoot_control.heat_limit, shoot_control.heat);
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

    // GM6020 CAN ID = 0x204 + ID, M2006 and M3508 CAN ID = 0x200 + ID
    // CAN_6020_LOW_RANGE_TX_ID same as CAN_3508_OR_2006_HIGH_RANGE_TX_ID
    gimbal_tx_message.StdId = CAN_6020_LOW_RANGE_TX_ID;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
#if PITCH_IS_4310
    // gimbal_can_send_data[2] = (rev >> 8);
    // gimbal_can_send_data[3] = rev;
#else
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
#endif

    gimbal_can_send_data[4] = (trigger >> 8);
    gimbal_can_send_data[5] = trigger;
    gimbal_can_send_data[6] = (fric_left >> 8);
    gimbal_can_send_data[7] = fric_left;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

    osDelay(1);
    gimbal_tx_message.StdId = CAN_3508_OR_2006_LOW_RANGE_TX_ID;
    // gimbal_can_send_data[0] = (rev >> 8);
    // gimbal_can_send_data[1] = rev;
    // gimbal_can_send_data[2] = (rev >> 8);
    // gimbal_can_send_data[3] = rev;
    // gimbal_can_send_data[4] = (rev >> 8);
    // gimbal_can_send_data[5] = rev;
    gimbal_can_send_data[6] = (fric_right >> 8);
    gimbal_can_send_data[7] = fric_right;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

#if PITCH_IS_4310
    osDelay(1);
	encode_MIT_motor_control(CAN_PIT_MOTOR_TX_ID, 0, 0, 0, 0, pitch, DM_4310, &GIMBAL_CAN);
#endif
}

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

HAL_StatusTypeDef enable_DaMiao_motor(uint32_t id, uint8_t _enable, CAN_HandleTypeDef *hcan_ptr)
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = id;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

	memset(gimbal_can_send_data, 0xFF, sizeof(gimbal_can_send_data));

	if (_enable)
	{
		gimbal_can_send_data[7] = 0xFC;
	}
	else
	{
		// disable
		gimbal_can_send_data[7] = 0xFD;
	}
	return HAL_CAN_AddTxMessage(hcan_ptr, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr)
{
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = id;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;

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

	gimbal_can_send_data[0] = (pos_tmp >> 8);
	gimbal_can_send_data[1] = pos_tmp;
	gimbal_can_send_data[2] = (vel_tmp >> 4);
	gimbal_can_send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	gimbal_can_send_data[4] = kp_tmp;
	gimbal_can_send_data[5] = (kd_tmp >> 4);
	gimbal_can_send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	gimbal_can_send_data[7] = tor_tmp;

	return HAL_CAN_AddTxMessage(hcan_ptr, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

HAL_StatusTypeDef decode_4310_motor_feedback(uint8_t *data, uint8_t bMotorId)
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
		uint16_t t_int = ((data[4] & 0xF) << 8) | data[5]; // Nm

		motor_chassis[bMotorId].output_angle = uint_to_fp32_motor(p_int, MIT_CONTROL_P_MIN[DM_4310], MIT_CONTROL_P_MAX[DM_4310], 16);
		motor_chassis[bMotorId].ecd = loop_fp32_constrain(motor_chassis[bMotorId].output_angle, 0, 2 * PI) * MOTOR_RAD_TO_ECD; //no actual ecd reading used 
		motor_chassis[bMotorId].velocity = uint_to_fp32_motor(v_int, MIT_CONTROL_V_MIN[DM_4310], MIT_CONTROL_V_MAX[DM_4310], 12);
		motor_chassis[bMotorId].torque = uint_to_fp32_motor(t_int, MIT_CONTROL_T_MIN[DM_4310], MIT_CONTROL_T_MAX[DM_4310], 12);
		motor_chassis[bMotorId].temperate = data[6];

		ret_value = HAL_OK;
	}
	return ret_value;
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
