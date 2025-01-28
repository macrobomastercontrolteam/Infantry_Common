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

#include "bsp_rng.h"
#include "main.h"

#include "chassis_task.h"
#include "detect_task.h"
#include "referee.h"
#include "remote_control.h"
#include "chassis_behaviour.h"
#include "string.h"
#include "shoot.h"

// Warning: for safety, PLEASE ALWAYS keep those default values as 0 when you commit
// Warning: because #if directive will assume the expression as 0 even if the macro is not defined, positive logic, for example, ENABLE_MOTOR_POWER, is safer that if and only if it's defined and set to 1 that the power is enabled
#define ENABLE_DRIVE_MOTOR_POWER 1
#define ENABLE_YAW_MOTOR_POWER 0
#define ENABLE_PITCH_MOTOR_POWER 0
// Remember to enable ENABLE_SHOOT_REDUNDANT_SWITCH as well if you want to shoot
#define ENABLE_TRIGGER_MOTOR_POWER 0
#define ENABLE_FRICTION_1_MOTOR_POWER 0
#define ENABLE_FRICTION_2_MOTOR_POWER 0

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define ENABLE_UPPER_HEAD_POWER 0
#elif (ROBOT_TYPE == HERO_2025_SWERVE)
#define ENABLE_STEER_MOTOR_POWER 1
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define ENABLE_STEER_MOTOR_POWER 1
#define ENABLE_HIP_MOTOR_POWER 0
#endif

#define REVERSE_M3508_1 0
#define REVERSE_M3508_2 0
#define REVERSE_M3508_3 0
#define REVERSE_M3508_4 0

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define IS_TRIGGER_ON_GIMBAL 1
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_BIPED) || (ROBOT_TYPE == HERO_2025_SWERVE)
#define IS_TRIGGER_ON_GIMBAL 0
#else
#define IS_TRIGGER_ON_GIMBAL 0
#endif

#define BULLET_SPEED_ECD_MAX 40.0f
#define BULLET_SPEED_RATIO (0xFF / BULLET_SPEED_ECD_MAX)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void CAN_cmd_3508_chassis(void);
fp32 uint_to_fp32_motor(int x_int, fp32 x_min, fp32 x_max, int bits);
int fp32_to_uint_motor(fp32 x, fp32 x_min, fp32 x_max, int bits);
HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef decode_4310_motor_feedback(uint8_t *data, uint8_t bMotorId);
void decode_rm_motor_feedback(uint8_t *data, uint8_t bMotorId);

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

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
const uint8_t abAllFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const fp32 MIT_CONTROL_P_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {12.5f, 12.5f, 12.5f};
const fp32 MIT_CONTROL_P_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-12.5f, -12.5f, -12.5f};
const fp32 MIT_CONTROL_V_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {25.0f, 45.0f, 30.0f};
const fp32 MIT_CONTROL_V_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-25.0f, -45.0f, -30.0f};
const fp32 MIT_CONTROL_T_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {20.0f, 24.0f, 10.0f};
const fp32 MIT_CONTROL_T_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-20.0f, -24.0f, -10.0f};
const fp32 MIT_CONTROL_KP_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {500.0f, 500.0f, 500.0f};
const fp32 MIT_CONTROL_KP_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f};
const fp32 MIT_CONTROL_KD_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {5.0f, 5.0f, 5.0f};
const fp32 MIT_CONTROL_KD_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f};

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE || ROBOT_TYPE == HERO_2025_SWERVE)
#define SWERVE_METER_PER_SEC_ECD_MAX_LIMIT 1.5f
#define SWERVE_METER_ECD_MAX_LIMIT 0.5f
#define SWERVE_ANGLE_ECD_MAX_LIMIT (PI / 12.0f)
#define SWERVE_WHEEL_ROT_RADIUS_DOT_DEADZONE 0.008f

const fp32 swerve_speed_encoding_ratio = (1 << 15) / SWERVE_METER_PER_SEC_ECD_MAX_LIMIT;
const fp32 swerve_meter_encoding_ratio = (1 << 16) / SWERVE_METER_ECD_MAX_LIMIT;
const fp32 swerve_angle_encoding_ratio = (1 << 15) / SWERVE_ANGLE_ECD_MAX_LIMIT;

const fp32 swerve_meter_encoding_ratio_shrinked = (1 << 8) / SWERVE_METER_ECD_MAX_LIMIT;
const fp32 swerve_angle_encoding_ratio_shrinked = (1 << 7) / SWERVE_ANGLE_ECD_MAX_LIMIT;

uint8_t decode_swerve_chassis_target_radius_dot(uint8_t *data);
uint8_t decode_swerve_chassis_feedback(uint8_t *data);

#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
#define BIPED_METER_PER_SEC_ECD_MAX_LIMIT 3.5f
#define BIPED_METER_ECD_MAX_LIMIT 0.5f
#define BIPED_RAD_ECD_MAX_LIMIT PI
#define BIPED_RAD_PER_SEC_ECD_MAX_LIMIT 2.5f

const fp32 biped_speed_encoding_ratio = (1 << 15) / BIPED_METER_PER_SEC_ECD_MAX_LIMIT;
const fp32 biped_meter_encoding_ratio = (1 << 16) / BIPED_METER_ECD_MAX_LIMIT;
const fp32 biped_angle_encoding_ratio = (1 << 15) / BIPED_RAD_ECD_MAX_LIMIT;
const fp32 biped_angle_speed_encoding_ratio = (1 << 15) / BIPED_RAD_PER_SEC_ECD_MAX_LIMIT;

uint8_t decode_biped_chassis_feedback(uint8_t *data);
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
	uint8_t bMotorId = 0;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (hcan == &GIMBAL_CAN)
	{
		switch (rx_header.StdId)
		{
			case CAN_PIT_MOTOR_ID:
			{
        		bMotorId = MOTOR_INDEX_PITCH;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				break;
			}
			case CAN_FRICTION_MOTOR_LEFT_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_LEFT;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(FRIC1_MOTOR_TOE);
				break;
			}
			case CAN_FRICTION_MOTOR_RIGHT_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_RIGHT;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(FRIC2_MOTOR_TOE);
				break;
			}
#if IS_TRIGGER_ON_GIMBAL
			case CAN_TRIGGER_MOTOR_ID:
			{
        		bMotorId = MOTOR_INDEX_TRIGGER;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
#endif
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
			case CAN_3508_M1_ID:
			{
				bMotorId = MOTOR_INDEX_3508_M1;
				decode_rm_motor_feedback(rx_data, bMotorId);
        		detect_hook(CHASSIS_MOTOR1_TOE);
				break;
			}
			case CAN_3508_M2_ID:
			{
        		bMotorId = MOTOR_INDEX_3508_M2;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(CHASSIS_MOTOR2_TOE);
        
				break;
			}
			case CAN_3508_M3_ID:
			{
        		bMotorId = MOTOR_INDEX_3508_M3;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(CHASSIS_MOTOR3_TOE);
				break;
			}
			case CAN_3508_M4_ID:
			{
        		bMotorId = MOTOR_INDEX_3508_M4;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(CHASSIS_MOTOR4_TOE);
				break;
			}
			case SUPCAP_RX_ID:
			{
				memcpy(cap_message_rx.can_buf, rx_data, sizeof(rx_data));
				detect_hook(SUPCAP_TOE);
				break;
			}
#if ROBOT_YAW_IS_4310
			case CAN_YAW_MOTOR_4310_RX_ID:
			{
				bMotorId = MOTOR_INDEX_YAW;
				if (decode_4310_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(YAW_GIMBAL_MOTOR_TOE);
				}
				break;
			}
#else
			case CAN_YAW_MOTOR_6020_RX_ID:
			{
        		bMotorId = MOTOR_INDEX_YAW;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(YAW_GIMBAL_MOTOR_TOE);
				break;
			}
#endif
#if (IS_TRIGGER_ON_GIMBAL == 0)
			case CAN_TRIGGER_MOTOR_ID:
			{
        		bMotorId = MOTOR_INDEX_TRIGGER;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
#endif
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == HERO_2025_SWERVE)
			case CAN_SHRINKED_CONTROLLER_RX_ID:
			{
				if (decode_swerve_chassis_feedback(rx_data))
				{
					detect_hook(SWERVE_CTRL_TOE);
				}
				break;
			}
			case CAN_SWERVE_RADII_DOT_RX_ID:
			{
				if (decode_swerve_chassis_target_radius_dot(rx_data))
				{
					detect_hook(SWERVE_CTRL_TOE);
				}
				break;
			}
#endif
#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
			case CAN_BIPED_CONTROLLER_RX_ID:
			{
				if (decode_biped_chassis_feedback(rx_data))
				{
					detect_hook(BIPED_CTRL_TOE);
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
}

void decode_rm_motor_feedback(uint8_t *data, uint8_t bMotorId)
{
	uint16_t temp_ecd = (uint16_t)(data[0] << 8 | data[1]);
	int16_t temp_speed = (int16_t)(data[2] << 8 | data[3]);
#if (REVERSE_M3508_1 || REVERSE_M3508_2 || REVERSE_M3508_3 || REVERSE_M3508_4)
	switch (bMotorId)
	{
#if REVERSE_M3508_1
		case MOTOR_INDEX_3508_M1:
#endif
#if REVERSE_M3508_2
		case MOTOR_INDEX_3508_M2:
#endif
#if REVERSE_M3508_3
		case MOTOR_INDEX_3508_M3:
#endif
#if REVERSE_M3508_4
		case MOTOR_INDEX_3508_M4:
#endif
		{
			temp_ecd = (temp_ecd + HALF_ECD_RANGE) % ECD_RANGE;
			temp_speed = -temp_speed;
			break;
		}
		default:
		{
			break;
		}
	}
#endif
	motor_chassis[bMotorId].last_ecd = motor_chassis[bMotorId].ecd;
	motor_chassis[bMotorId].ecd = temp_ecd;
	motor_chassis[bMotorId].speed_rpm = temp_speed;
	motor_chassis[bMotorId].feedback_current = (int16_t)(data[4] << 8 | data[5]);
	motor_chassis[bMotorId].temperate = data[6];
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
		uint16_t p_int = (data[1] << 8) | data[2];		   // rad
		uint16_t v_int = (data[3] << 4) | (data[4] >> 4);  // rad/s
		uint16_t t_int = ((data[4] & 0xF) << 8) | data[5]; // Nm

		motor_chassis[bMotorId].output_angle = uint_to_fp32_motor(p_int, MIT_CONTROL_P_MIN[DM_4310], MIT_CONTROL_P_MAX[DM_4310], 16);
		motor_chassis[bMotorId].ecd = loop_fp32_constrain(motor_chassis[bMotorId].output_angle, 0, 2 * PI) * MOTOR_RAD_TO_ECD;
		motor_chassis[bMotorId].velocity = uint_to_fp32_motor(v_int, MIT_CONTROL_V_MIN[DM_4310], MIT_CONTROL_V_MAX[DM_4310], 12);
		motor_chassis[bMotorId].torque = uint_to_fp32_motor(t_int, MIT_CONTROL_T_MIN[DM_4310], MIT_CONTROL_T_MAX[DM_4310], 12);
		motor_chassis[bMotorId].temperate = data[6];

		ret_value = HAL_OK;
	}
	return ret_value;
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == HERO_2025_SWERVE)
uint8_t decode_swerve_chassis_feedback(uint8_t *data)
{
	uint8_t fDataValid = (memcmp(data, abAllFF, sizeof(abAllFF)) != 0);
	if (fDataValid)
	{
		for (uint8_t wheel_id = 0; wheel_id < 4; wheel_id++)
		{
			chassis_move.wheel_rot_radii[wheel_id] = data[wheel_id] / swerve_meter_encoding_ratio_shrinked;
		}
		chassis_move.chassis_platform.feedback_alpha1 = data[4] / swerve_angle_encoding_ratio_shrinked;
		chassis_move.chassis_platform.feedback_alpha2 = data[5] / swerve_angle_encoding_ratio_shrinked;
		chassis_move.chassis_platform.feedback_height = data[6] / swerve_meter_encoding_ratio_shrinked;
		// data[7] reserved
	}
	return fDataValid;
}

uint8_t decode_swerve_chassis_target_radius_dot(uint8_t *data)
{
	uint8_t fDataValid = (memcmp(data, abAllFF, sizeof(abAllFF)) != 0);
	if (fDataValid)
	{
		for (uint8_t wheel_id = 0; wheel_id < 4; wheel_id++)
		{
			int16_t radius_dot = (data[2 * wheel_id + 1] << 8) | data[2 * wheel_id];
			chassis_move.target_wheel_rot_radii_dot[wheel_id] = (fp32)radius_dot / swerve_speed_encoding_ratio;
			fp32_deadzone(&chassis_move.target_wheel_rot_radii_dot[wheel_id], SWERVE_WHEEL_ROT_RADIUS_DOT_DEADZONE);
			// first_order_filter(chassis_move.wheel_rot_radii_dot[wheel_id], chassis_move.wheel_rot_radii_dot_last[wheel_id], 0.8f);
		}
	}
	return fDataValid;
}
#endif

#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
void CAN_cmd_biped_chassis(void)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_BIPED_CONTROLLER_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

	// hip and drive power has to be enabled together for biped
#if ENABLE_DRIVE_MOTOR_POWER
	if (chassis_move.fLegEnabled)
	{
		if (chassis_move.chassis_platform.fBackToHome)
		{
			memset(chassis_can_send_data, 0, sizeof(chassis_can_send_data));
		}
		else
		{
			int16_t yaw_int = rad_format(chassis_move.chassis_platform.target_yaw) * biped_angle_encoding_ratio;
			int16_t L0_dot_int = fp32_constrain(chassis_move.chassis_platform.target_simplified_L0_dot, -BIPED_METER_PER_SEC_ECD_MAX_LIMIT, BIPED_METER_PER_SEC_ECD_MAX_LIMIT) * biped_speed_encoding_ratio;
			int16_t roll_dot_int = fp32_constrain(chassis_move.chassis_platform.target_roll_dot, -BIPED_RAD_PER_SEC_ECD_MAX_LIMIT, BIPED_RAD_PER_SEC_ECD_MAX_LIMIT) * biped_angle_speed_encoding_ratio;
			int16_t dis_dot_int = fp32_constrain(chassis_move.chassis_platform.target_dis_dot, -BIPED_METER_PER_SEC_ECD_MAX_LIMIT, BIPED_METER_PER_SEC_ECD_MAX_LIMIT) * biped_speed_encoding_ratio;

			chassis_can_send_data[0] = *(uint8_t *)(&yaw_int);
			chassis_can_send_data[1] = *((uint8_t *)(&yaw_int) + 1);
			chassis_can_send_data[2] = *(uint8_t *)(&L0_dot_int);
			chassis_can_send_data[3] = *((uint8_t *)(&L0_dot_int) + 1);
			chassis_can_send_data[4] = *(uint8_t *)(&roll_dot_int);
			chassis_can_send_data[5] = *((uint8_t *)(&roll_dot_int) + 1);
			chassis_can_send_data[6] = *(uint8_t *)(&dis_dot_int);
			chassis_can_send_data[7] = *((uint8_t *)(&dis_dot_int) + 1);

			// reset speeds for safety, they will be reassigned in chassis_task immediately before next call of this function
			// chassis_move.chassis_platform.target_yaw_dot = 0;
			chassis_move.chassis_platform.target_simplified_L0_dot = 0;
			chassis_move.chassis_platform.target_roll_dot = 0;
			chassis_move.chassis_platform.target_dis_dot = 0;
		}
	}
	else
#endif
	{
		memset(chassis_can_send_data, 0xFF, sizeof(chassis_can_send_data));
	}
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_biped_chassis_mode(void)
{
	// determine whether to send mode control msg
	static uint8_t bBipedModeTxCounter = 0;
	static uint8_t fLastJumpSignal = 0;
	static uint8_t fLastBackToChairPosture = 0;
	static uint8_t bLastRcLeftSw = RC_SW_DOWN;
	static uint8_t bLastRcRightSw = RC_SW_DOWN;
	uint8_t bRcLeftSw = RC_SW_DOWN;
	uint8_t bRcRightSw = RC_SW_DOWN;
	if (toe_is_error(DBUS_TOE) == 0)
	{
		bRcLeftSw = rc_ctrl.rc.s[RC_LEFT_LEVER_CHANNEL];
		bRcRightSw = rc_ctrl.rc.s[RC_RIGHT_LEVER_CHANNEL];
	}
	
	if ((bLastRcLeftSw != bRcLeftSw) || (bLastRcRightSw != bRcRightSw) || (fLastJumpSignal != chassis_move.chassis_platform.fJumpStart))
	{
		bBipedModeTxCounter = 0;
		bLastRcLeftSw = bRcLeftSw;
		bLastRcRightSw = bRcRightSw;
		fLastJumpSignal = chassis_move.chassis_platform.fJumpStart;
	}
	
	if (fLastBackToChairPosture != chassis_move.chassis_platform.fBackToHome)
	{
		// do not trigger mode control msg on falling edge of fBackToHome
		if (chassis_move.chassis_platform.fBackToHome)
		{
			bBipedModeTxCounter = 0;
		}
		fLastBackToChairPosture = chassis_move.chassis_platform.fBackToHome;
	}

	if (bBipedModeTxCounter < 3)
	{
		// send mode control msg for 3 times to make sure that biped controller receives it
		osDelay(1);

		uint32_t send_mail_box;
		chassis_tx_message.StdId = CAN_BIPED_CONTROLLER_MODE_TX_ID;
		chassis_tx_message.IDE = CAN_ID_STD;
		chassis_tx_message.RTR = CAN_RTR_DATA;
		chassis_tx_message.DLC = 0x08;

		chassis_can_send_data[0] = bRcLeftSw;
		chassis_can_send_data[1] = bRcRightSw;
		chassis_can_send_data[2] = chassis_move.chassis_platform.fBackToHome;
		chassis_can_send_data[3] = chassis_move.chassis_platform.fJumpStart;
		HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
		
		bBipedModeTxCounter++;
	}
	else
	{
		chassis_move.chassis_platform.fBackToHome = 0;
		// Do not reset fJumpStart to avoid noise in the input signal. Note that jumping action of biped lower board will only be trigger upon rising edge 
	}
}

uint8_t decode_biped_chassis_feedback(uint8_t *data)
{
	uint8_t fDataValid = 1;
	chassis_move.chassis_platform.feedback_yaw = fp32_constrain((int16_t)((data[1] << 8) | data[0]) / biped_angle_encoding_ratio, -PI, PI);
	chassis_move.chassis_platform.feedback_simplified_L0 = (int16_t)((data[3] << 8) | data[2]) / biped_meter_encoding_ratio;
	chassis_move.chassis_platform.feedback_roll = (int16_t)((data[5] << 8) | data[4]) / biped_angle_encoding_ratio;
	chassis_move.chassis_platform.feedback_pitch = (int16_t)((data[7] << 8) | data[6]) / biped_angle_encoding_ratio;
	return fDataValid;
}
#endif

/**
 * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
 * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
 * @param[in]      trigger: (0x207) 2006 motor control current, range [-10000,10000]
 * @param[in]      fric_left: 3508 motor control current when used as friction motor
 * @param[in]      fric_right: 3508 motor control current when used as friction motor
 * @retval         none
 */
void CAN_cmd_gimbal(fp32 yaw, fp32 pitch, int16_t trigger, int16_t fric_left, int16_t fric_right)
{
	uint32_t send_mail_box;
	// CAN_6020_LOW_RANGE_TX_ID same as CAN_3508_OR_2006_HIGH_RANGE_TX_ID
	gimbal_tx_message.StdId = CAN_6020_LOW_RANGE_TX_ID;
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

	// control yaw motor and trigger motor
#if ROBOT_YAW_IS_4310
	// gimbal_can_send_data[0] = (rev >> 8);
	// gimbal_can_send_data[1] = rev;
#else
	gimbal_can_send_data[0] = ((int16_t)yaw >> 8);
	gimbal_can_send_data[1] = (int16_t)yaw;
#endif
	// gimbal_can_send_data[2] = (rev >> 8);
	// gimbal_can_send_data[3] = rev;
#if IS_TRIGGER_ON_GIMBAL
	// gimbal_can_send_data[4] = (rev >> 8);
	// gimbal_can_send_data[5] = rev;
#else
	gimbal_can_send_data[4] = (trigger >> 8);
	gimbal_can_send_data[5] = trigger;
#endif
	// gimbal_can_send_data[6] = (rev >> 8);
	// gimbal_can_send_data[7] = rev;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
#if ROBOT_YAW_IS_4310
	osDelay(1);
#endif

	// control pitch motor and fric_left and fric_right
	gimbal_can_send_data[0] = (fric_left >> 8);
	gimbal_can_send_data[1] = fric_left;
	gimbal_can_send_data[2] = ((int16_t)pitch >> 8);
	gimbal_can_send_data[3] = (int16_t)pitch;
#if IS_TRIGGER_ON_GIMBAL
	gimbal_can_send_data[4] = (trigger >> 8);
	gimbal_can_send_data[5] = trigger;
#else
	// gimbal_can_send_data[4] = (rev >> 8);
	// gimbal_can_send_data[5] = rev;
#endif
	gimbal_can_send_data[6] = (fric_right >> 8);
	gimbal_can_send_data[7] = fric_right;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

#if ROBOT_YAW_IS_4310
	encode_MIT_motor_control(CAN_YAW_MOTOR_4310_TX_ID, 0, 0, 0, 0, yaw, DM_4310, &CHASSIS_CAN);
#endif
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

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
#if ROBOT_CHASSIS_USE_MECANUM || (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == HERO_2025_SWERVE)
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
#endif
}

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
void CAN_cmd_upper_head(void)
{
#if ENABLE_UPPER_HEAD_POWER
	if (chassis_move.fUpperHeadEnabled)
	{
		uint32_t send_mail_box;
		chassis_tx_message.StdId = CAN_UPPER_HEAD_TX_ID;
		chassis_tx_message.IDE = CAN_ID_STD;
		chassis_tx_message.RTR = CAN_RTR_DATA;
		chassis_tx_message.DLC = 0x08;

		uint8_t _bullet_speed = 0;
		if (shoot_control.bullet_init_speed[1] > 0)
		{
			if (shoot_control.bullet_init_speed[1] > BULLET_SPEED_ECD_MAX)
			{
				_bullet_speed = BULLET_SPEED_ECD_MAX * BULLET_SPEED_RATIO;
			}
			else
			{
				_bullet_speed = shoot_control.bullet_init_speed[1] * BULLET_SPEED_RATIO;
			}
		}

		uint8_t team_color = get_team_color();
		uint16_t shoot_heat_limit = 0;

		const fp32 shoot_heat_limit_max = 400.0f;
		uint16_t shoot_heat1_int16 = 0;
		get_shoot_heat1_limit_and_heat(&shoot_heat_limit, &shoot_heat1_int16);
		uint8_t shoot_heat_limit_uint8 = fp32_abs_constrain(shoot_heat_limit, shoot_heat_limit_max) / shoot_heat_limit_max * 255.0f;
		uint8_t shoot_heat1_uint8 = fp32_abs_constrain(shoot_heat1_int16, shoot_heat_limit_max) / shoot_heat_limit_max * 255.0f;

		uint16_t blueOutPostHP = get_blue_outpost_HP();
		uint16_t redOutPostHP = get_red_outpost_HP();

		chassis_can_send_data[0] = shoot_heat_limit_uint8;
		chassis_can_send_data[1] = shoot_heat1_uint8;
		chassis_can_send_data[2] = _bullet_speed;
		chassis_can_send_data[3] = (blueOutPostHP >> 8);
		chassis_can_send_data[4] = blueOutPostHP;
		chassis_can_send_data[5] = (redOutPostHP >> 8);
		chassis_can_send_data[6] = redOutPostHP;
		chassis_can_send_data[7] = (team_color << 7);
		HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	}
#endif
}
#endif

void CAN_cmd_chassis(void)
{
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == HERO_2025_SWERVE)
	CAN_cmd_3508_chassis();
	osDelay(1);
	CAN_cmd_swerve_steer();
	osDelay(1);
	CAN_cmd_swerve_hip();
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
	CAN_cmd_3508_chassis();
	osDelay(1);
	CAN_cmd_upper_head();
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
	CAN_cmd_biped_chassis();
	CAN_cmd_biped_chassis_mode();
#else
	CAN_cmd_3508_chassis();
#endif
}

/**
 * @brief          send control current or voltage of motor. Refer to can_msg_id_e for motor IDs
 * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
 * @param[in]      steer_motor1: target encoder value of 6020 motor; it's moved to a bus only controlled by chassis controller to reduce bus load
 * @param[in]      steer_motor2: target encoder value of 6020 motor; it's moved to a bus only controlled by chassis controller to reduce bus load
 * @param[in]      steer_motor3: target encoder value of 6020 motor; it's moved to a bus only controlled by chassis controller to reduce bus load
 * @param[in]      steer_motor4: target encoder value of 6020 motor; it's moved to a bus only controlled by chassis controller to reduce bus load
 * @retval         none
 */
void CAN_cmd_3508_chassis(void)
{
#if (ROBOT_TYPE != INFANTRY_2024_BIPED)
	uint32_t send_mail_box;
	// driver motors (M3508)
	chassis_tx_message.StdId = CAN_3508_OR_2006_LOW_RANGE_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

#if ENABLE_DRIVE_MOTOR_POWER
	int16_t motor1 = chassis_move.motor_chassis[0].give_current;
	int16_t motor2 = chassis_move.motor_chassis[1].give_current;
	int16_t motor3 = chassis_move.motor_chassis[2].give_current;
	int16_t motor4 = chassis_move.motor_chassis[3].give_current;
#else
	int16_t motor1 = 0;
	int16_t motor2 = 0;
	int16_t motor3 = 0;
	int16_t motor4 = 0;
#endif

#if REVERSE_M3508_1
	motor1 = -motor1;
#endif

#if REVERSE_M3508_2
	motor2 = -motor2;
#endif

#if REVERSE_M3508_3
	motor3 = -motor3;
#endif

#if REVERSE_M3508_4
	motor4 = -motor4;
#endif

	chassis_can_send_data[0] = motor1 >> 8;
	chassis_can_send_data[1] = motor1;
	chassis_can_send_data[2] = motor2 >> 8;
	chassis_can_send_data[3] = motor2;
	chassis_can_send_data[4] = motor3 >> 8;
	chassis_can_send_data[5] = motor3;
	chassis_can_send_data[6] = motor4 >> 8;
	chassis_can_send_data[7] = motor4;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
#endif
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == HERO_2025_SWERVE)
void CAN_cmd_swerve_steer(void)
{
	uint32_t send_mail_box;

	// Send target encoder value of steering motors (GM6020) to chassis controller
	chassis_tx_message.StdId = CAN_STEER_CONTROLLER_TX_ID;
#if ENABLE_STEER_MOTOR_POWER
	if ((chassis_behaviour_mode != CHASSIS_ZERO_FORCE) || chassis_move.fHipDisabledEdge)
	{
		uint16_t steer_motor1 = chassis_move.steer_motor_chassis[0].target_ecd;
		uint16_t steer_motor2 = chassis_move.steer_motor_chassis[1].target_ecd;
		uint16_t steer_motor3 = chassis_move.steer_motor_chassis[2].target_ecd;
		uint16_t steer_motor4 = chassis_move.steer_motor_chassis[3].target_ecd;

		chassis_can_send_data[0] = steer_motor1 >> 8;
		chassis_can_send_data[1] = steer_motor1;
		chassis_can_send_data[2] = steer_motor2 >> 8;
		chassis_can_send_data[3] = steer_motor2;
		chassis_can_send_data[4] = steer_motor3 >> 8;
		chassis_can_send_data[5] = steer_motor3;
		chassis_can_send_data[6] = steer_motor4 >> 8;
		chassis_can_send_data[7] = steer_motor4;
	}
	else
#endif
	{
		memset(chassis_can_send_data, 0xFF, sizeof(chassis_can_send_data));
	}
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_swerve_hip(void)
{
	uint32_t send_mail_box;

	chassis_tx_message.StdId = CAN_SWERVE_CONTROLLERE_TX_ID;
#if ENABLE_HIP_MOTOR_POWER
	if (chassis_move.fHipEnabled)
	{
		int16_t target_alpha1_cmd = fp32_abs_constrain(chassis_move.chassis_platform.target_alpha1, SWERVE_ANGLE_ECD_MAX_LIMIT) * swerve_angle_encoding_ratio;
		int16_t target_alpha2_cmd = fp32_abs_constrain(chassis_move.chassis_platform.target_alpha2, SWERVE_ANGLE_ECD_MAX_LIMIT) * swerve_angle_encoding_ratio;
		uint16_t target_height_cmd = fp32_constrain(chassis_move.chassis_platform.target_height, 0, SWERVE_METER_ECD_MAX_LIMIT) * swerve_meter_encoding_ratio;

		chassis_can_send_data[0] = target_alpha1_cmd >> 8;
		chassis_can_send_data[1] = target_alpha1_cmd;
		chassis_can_send_data[2] = target_alpha2_cmd >> 8;
		chassis_can_send_data[3] = target_alpha2_cmd;
		chassis_can_send_data[4] = target_height_cmd >> 8;
		chassis_can_send_data[5] = target_height_cmd;
		// reserved
		// chassis_can_send_data[6] = rev >> 8;
		// chassis_can_send_data[7] = rev;
	}
	else
#endif
	{
		memset(chassis_can_send_data, 0xFF, sizeof(chassis_can_send_data));
	}
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}
#endif

#if USE_SERVO_TO_STIR_AMMO
void CAN_cmd_load_servo(uint8_t fServoSwitch, uint8_t bTrialTimes)
{
	// Turn on/off loading servo motor, by commanding Type-A board on chassis
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_CHASSIS_LOAD_SERVO_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;
	chassis_can_send_data[0] = fServoSwitch;
	for (uint8_t i = 0; i < bTrialTimes; i++)
	{
		HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
		osDelay(1);
	}
}
#endif

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

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t motor_index)
{
	if (motor_index >= MOTOR_LIST_LENGTH)
	{
		return NULL;
	}
	else
	{
		return &motor_chassis[motor_index];
	}
}

void chassis_enable_platform_flag(uint8_t fEnabled)
{
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == HERO_2025_SWERVE)

#if ENABLE_HIP_MOTOR_POWER
	chassis_move.fHipDisabledEdge = ((fEnabled == 0) && chassis_move.fHipEnabled);
	chassis_move.fHipEnabled = fEnabled;
#else
	chassis_move.fHipDisabledEdge = 0;
	chassis_move.fHipEnabled = 0;
#endif

#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)

#if ENABLE_DRIVE_MOTOR_POWER
	chassis_move.fLegEnabled = fEnabled;
#else
	chassis_move.fLegEnabled = 0;
#endif

#endif
}
