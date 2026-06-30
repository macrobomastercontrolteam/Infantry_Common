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
#include "custom_ui_task.h"

// Warning: for safety, PLEASE ALWAYS keep those default values as 0 when you commit
// Warning: because #if directive will assume the expression as 0 even if the macro is not defined, positive logic, for example, ENABLE_MOTOR_POWER, is safer that if and only if it's defined and set to 1 that the power is enabled

//////////////enable for all robot types//////////////////////
#define ENABLE_DRIVE_MOTOR_POWER 1
#define ENABLE_YAW_MOTOR_POWER 1
#define ENABLE_PITCH_MOTOR_POWER 1
///////////////enable fo 2026 standard only begin///////////////////
#define ENABLE_PITCH_BASE_MOTOR_POWER 0
////////////////enable fo 2026 standard only end////////////////////
// Remember to enable ENABLE_SHOOT_REDUNDANT_SWITCH as well if you want to shoot
#define ENABLE_TRIGGER_MOTOR_POWER 0
#define ENABLE_FRICTION_1_MOTOR_POWER 0
#define ENABLE_FRICTION_2_MOTOR_POWER 0
///////////////enable fo 2025 Hero only begin///////////////////
#define ENABLE_FRICTION_3_MOTOR_POWER 0
#define ENABLE_FRICTION_4_MOTOR_POWER 0
#define ENABLE_PISTON_MOTOR_POWER 0
////////////////enable fo 2025 Hero only end////////////////////
///////////////enable fo 2026 Hero only begin///////////////////
#define ENABLE_SECOND_YAW_MOTOR_POWER 1
////////////////enable fo 2026 Hero only end////////////////////

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define ENABLE_UPPER_HEAD_POWER 0
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define ENABLE_STEER_MOTOR_POWER 0
#define ENABLE_HIP_MOTOR_POWER 0
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
#define ENABLE_HIP_MOTOR_POWER 1
#endif

#if (ROBOT_TYPE == INFANTRY_2026_MECANUM) && !ENABLE_PITCH_BASE_MOTOR_POWER && ENABLE_PITCH_MOTOR_POWER
#error "INFANTRY_2026_MECANUM must has its pitch base motor power enabled for normal operation"
#elif (ROBOT_TYPE != INFANTRY_2026_MECANUM) && ENABLE_PITCH_BASE_MOTOR_POWER
#error "This robot type should not enable pitch base motor power"
#endif

#define REVERSE_M3508_1 0
#define REVERSE_M3508_2 0
#define REVERSE_M3508_3 0
#define REVERSE_M3508_4 0

#define REVERSE_MG4010_1 0
#define REVERSE_MG4010_2 0
#define REVERSE_MG4010_3 0
#define REVERSE_MG4010_4 0

// Set to 1 to reverse the MG4010 trigger motor spin direction (command + feedback are both flipped,
// so the firmware "positive = loading" convention stays physically correct).
#define REVERSE_TRIGGER_MG4010 0

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == HERO_2026_OMNI)
#define IS_TRIGGER_ON_GIMBAL 1
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE) || (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_BIPED) || (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI) || (ROBOT_TYPE == INFANTRY_2026_MECANUM)
#define IS_TRIGGER_ON_GIMBAL 0
#else
#define IS_TRIGGER_ON_GIMBAL 0
#endif

#define BULLET_SPEED_ECD_MAX 40.0f
#define BULLET_SPEED_RATIO (0xFF / BULLET_SPEED_ECD_MAX)

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
power_meter_can_rx_t power_meter_can_rx_msg;

#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR) || TRIGGER_MOTOR_IS_4010
static uint8_t can_send_data[8];
static CAN_TxHeaderTypeDef can_tx_message;
uint32_t send_mail_box;

HAL_StatusTypeDef encode_ktech_broadcast_speed_control(const int16_t speedControlCmd[4], uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef Send_CAN_Cmd(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data, uint8_t blocking_call);
HAL_StatusTypeDef blocking_can_send(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *tx_header, uint8_t *tx_data);
void decode_MG_4010_motor_feedback(uint8_t *data, uint8_t bMotorId);
#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)
void CAN_cmd_4010_chassis(void);
#endif
#endif

void CAN_cmd_3508_chassis(void);
fp32 uint_to_fp32_motor(int x_int, fp32 x_min, fp32 x_max, int bits);
int fp32_to_uint_motor(fp32 x, fp32 x_min, fp32 x_max, int bits);
HAL_StatusTypeDef encode_MIT_motor_control(uint16_t id, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq, MIT_controlled_motor_type_e motor_type, CAN_HandleTypeDef *hcan_ptr);
HAL_StatusTypeDef decode_4310_motor_feedback(uint8_t *data, uint8_t bMotorId);
HAL_StatusTypeDef decode_4340_motor_feedback(uint8_t *data, uint8_t bMotorId);
HAL_StatusTypeDef decode_3507_motor_feedback(uint8_t *data, uint8_t bMotorId);

void decode_rm_motor_feedback(uint8_t *data, uint8_t bMotorId);

void decode_power_meter(uint8_t *data);
fp32 get_chassis_power_meter_data(void);
#if (SUPERCAP_TYPE == UBC_SUPERCAP)
capcan_rx_t capcan_rx_msg;
capcan_tx_t capcan_tx_msg;
void decode_ubc_cap_tx_data(uint8_t *data);
#elif (SUPERCAP_TYPE == MACRM_SUPERCAP)
capcan_rx_t capcan_rx_msg;
capcan_tx_t capcan_tx_msg;
#elif (SUPERCAP_TYPE == SJTU_SUPERCAP)
supcap_t cap_message_rx;
#endif

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

can_ref_info_t can_ref_info;

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];
static uint8_t interboard_can_send_data[8]; 
const uint8_t abAllFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const fp32 MIT_CONTROL_P_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {12.5f, 12.5f, 4.0f*PI, 4.0f*PI, 4.0f*PI};  //value needs to match to which in motor setting software
const fp32 MIT_CONTROL_P_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-12.5f, -12.5f, -4.0f*PI, -4.0f*PI, -4.0f*PI};
const fp32 MIT_CONTROL_V_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {25.0f, 45.0f, 30.0f, 30.0f, 50.0f};
const fp32 MIT_CONTROL_V_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-25.0f, -45.0f, -30.0f, -30.0f, -50.0f};
const fp32 MIT_CONTROL_T_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {20.0f, 24.0f, 10.0f, 10.0f, 5.0f};
const fp32 MIT_CONTROL_T_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {-20.0f, -24.0f, -10.0f, -10.0f, -5.0f};
const fp32 MIT_CONTROL_KP_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {500.0f, 500.0f, 500.0f, 500.0f, 500.0f};
const fp32 MIT_CONTROL_KP_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const fp32 MIT_CONTROL_KD_MAX[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
const fp32 MIT_CONTROL_KD_MIN[LAST_MIT_CONTROLLED_MOTOR_TYPE] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
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

#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
#define CHASSIS_METER_PER_SEC_ECD_MAX_LIMIT 1.5f
#define CHASSIS_METER_ECD_MAX_LIMIT 0.5f
#define CHASSIS_ANGLE_ECD_MAX_LIMIT CHASSIS_ALPHA_WORKSPACE_PEAK
#define CHASSIS_WHEEL_ROT_RADIUS_DOT_DEADZONE 0.008f

const fp32 chassis_speed_encoding_ratio = ((1 << 15) - 1) / CHASSIS_METER_PER_SEC_ECD_MAX_LIMIT;
const fp32 chassis_meter_encoding_ratio = ((1 << 16) - 1) / CHASSIS_METER_ECD_MAX_LIMIT;
const fp32 chassis_angle_encoding_ratio = ((1 << 15) - 1) / CHASSIS_ANGLE_ECD_MAX_LIMIT;

const fp32 chassis_meter_encoding_ratio_shrinked = (1 << 8) / CHASSIS_METER_ECD_MAX_LIMIT;
const fp32 chassis_angle_encoding_ratio_shrinked = (1 << 7) / CHASSIS_ANGLE_ECD_MAX_LIMIT;

uint8_t decode_chassis_target_radius_dot(uint8_t *data);
uint8_t decode_chassis_feedback(uint8_t *data);

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
#if ROBOT_PITCH_IS_4340
			case CAN_PITCH_MOTOR_4340_RX_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH;
				if (decode_4340_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				}
				break;
			}
#elif ROBOT_PITCH_IS_3507
			case CAN_PITCH_MOTOR_3507_RX_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH;
				if (decode_3507_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				}
				break;
			}
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
			case CAN_PITCH_MOTOR_4310_RX_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH;
				if (decode_4310_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				}
				break;
			}

			case CAN_PITCH_BASE_MOTOR_4310_RX_ID:
			{
				bMotorId = MOTOR_INDEX_PITCH_BASE;
				if (decode_4310_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(PITCH_BASE_GIMBAL_MOTOR_TOE);
				}
				break;
			}
#else
			case CAN_PIT_MOTOR_ID:
			{
        		bMotorId = MOTOR_INDEX_PITCH;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(PITCH_GIMBAL_MOTOR_TOE);
				break;
			}
#endif
#if (ROBOT_TYPE == HERO_2026_OMNI)
			case CAN_SECOND_YAW_MOTOR_4310_RX_ID:
			{
				bMotorId = MOTOR_INDEX_SECOND_YAW;
				if (decode_4310_motor_feedback(rx_data, bMotorId) == HAL_OK)
				{
					detect_hook(SECOND_YAW_GIMBAL_MOTOR_TOE);
				}
				break;
			}
#endif
			case CAN_FRICTION_MOTOR_LEFT_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_LEFT;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(FRICTIONAL_MOTOR_LEFT_TOE);
				break;
			}
			case CAN_FRICTION_MOTOR_RIGHT_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_RIGHT;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(FRICTIONAL_MOTOR_RIGHT_TOE);
				break;
			}
#if (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == HERO_2026_OMNI)
			case CAN_FRICTION_MOTOR_UP_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_UP;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(FRICTIONAL_MOTOR_UP_TOE);
				break;
			}
			case CAN_FRICTION_MOTOR_DOWN_ID:
			{
				bMotorId = MOTOR_INDEX_FRICTION_DOWN;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(FRICTIONAL_MOTOR_DOWN_TOE);
				break;
			}
			case CAN_PISTON_MOTOR_ID:
			{
				bMotorId = MOTOR_INDEX_PISTON;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(PISTON_MOTOR_TOE);
				break;
			}
#endif

#if IS_TRIGGER_ON_GIMBAL
#if TRIGGER_MOTOR_IS_4010
			case CAN_TRIGGER_MOTOR_ID:
			{
				bMotorId = MOTOR_INDEX_TRIGGER;
				decode_MG_4010_motor_feedback(rx_data, bMotorId);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
#else
			case CAN_TRIGGER_MOTOR_ID:
			{
        		bMotorId = MOTOR_INDEX_TRIGGER;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
#endif
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
#if (MOTOR_TYPE == POWER_TRAIN_USE_3508_MOTOR)
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
#elif (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)
			case CAN_4010_M1_ID:
			{
				bMotorId = MOTOR_INDEX_4010_M1;
				decode_MG_4010_motor_feedback(rx_data, bMotorId);
        		detect_hook(CHASSIS_MOTOR1_TOE);
				break;
			}
			case CAN_4010_M2_ID:
			{
        		bMotorId = MOTOR_INDEX_4010_M2;
				decode_MG_4010_motor_feedback(rx_data, bMotorId);
				detect_hook(CHASSIS_MOTOR2_TOE);
        
				break;
			}
			case CAN_4010_M3_ID:
			{
        		bMotorId = MOTOR_INDEX_4010_M3;
				decode_MG_4010_motor_feedback(rx_data, bMotorId);
				detect_hook(CHASSIS_MOTOR3_TOE);
				break;
			}
			case CAN_4010_M4_ID:
			{
        		bMotorId = MOTOR_INDEX_4010_M4;
				decode_MG_4010_motor_feedback(rx_data, bMotorId);
				detect_hook(CHASSIS_MOTOR4_TOE);
				break;
			}
#endif
			case SUPCAP_RX_ID:
			{
				decode_supercap(rx_data);
				detect_hook(SUPCAP_TOE);
				break;
			}

			case CAN_POWER_METER_RX_ID:
			{
				decode_power_meter(rx_data);
				detect_hook(POWER_METER_TOE);
				break;
			}
#if CAN_PASS_REF_INFO
			case CAN_REF_INFO_PULL_RX_ID:
			{
				decode_ref_info(rx_data);
				detect_hook(REFEREE_TOE);
				break;
			}
#endif

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

#if (IS_TRIGGER_ON_GIMBAL == 0) && (TRIGGER_MOTOR_IS_4010 == 0)
			case CAN_TRIGGER_MOTOR_ID:
			{
        		bMotorId = MOTOR_INDEX_TRIGGER;
				decode_rm_motor_feedback(rx_data, bMotorId);
				detect_hook(TRIGGER_MOTOR_TOE);
				break;
			}
#endif
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
			case CAN_SHRINKED_CONTROLLER_RX_ID:
			{
				if (decode_swerve_chassis_feedback(rx_data))
				{
					detect_hook(SWERVE_CTRL_TOE);
				}
				break;
			}
			case CAN_CHASSIS_RADII_DOT_RX_ID:
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
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
			case CAN_SHRINKED_CONTROLLER_RX_ID:
			{
				if (decode_chassis_feedback(rx_data))
				{
					detect_hook(SWERVE_CTRL_TOE);
				}
				break;
			}
			case CAN_CHASSIS_RADII_DOT_RX_ID:
			{
				if (decode_chassis_target_radius_dot(rx_data))
				{
					detect_hook(SWERVE_CTRL_TOE);
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

#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR) || TRIGGER_MOTOR_IS_4010
void decode_MG_4010_motor_feedback(uint8_t *data, uint8_t bMotorId)
{

	int16_t current_int = (data[3]<<8) | data[2]; //Uniy Ampere
	int16_t v_int = (data[5]<<8) | data[4]; //deg/s
	uint16_t p_int = (data[7]<<8) | data[6]; //encode value

	motor_chassis[bMotorId].feedback_current = (fp32)current_int;
	motor_chassis[bMotorId].speed_rpm = (int16_t)(v_int / 6.0f) / MOTOR_MG4010_GEAR_RATIO;  // convert deg/s to RPM: 360 deg/s = 60 RPM
	motor_chassis[bMotorId].velocity = ((fp32)v_int) / MOTOR_MG4010_GEAR_RATIO / 180.0f * PI;
	motor_chassis[bMotorId].output_angle = ((fp32)p_int) / (1 << 14) * 2.0f * PI;
	motor_chassis[bMotorId].temperature = data[1];

	// Populate ecd/last_ecd (scaled from the 14-bit MG4010 encoder 0..16383 down to ECD_RANGE 0..8191)
	// so the trigger ammo/position counting in shoot.c keeps working like the RM motor path.
	uint16_t ecd_scaled = (uint16_t)(p_int >> 1);
#if TRIGGER_MOTOR_IS_4010 && REVERSE_TRIGGER_MG4010
	if (bMotorId == MOTOR_INDEX_TRIGGER)
	{
		ecd_scaled = (uint16_t)((ECD_RANGE - 1) - ecd_scaled);
		motor_chassis[bMotorId].speed_rpm = -motor_chassis[bMotorId].speed_rpm;
		motor_chassis[bMotorId].velocity = -motor_chassis[bMotorId].velocity;
	}
#endif
	motor_chassis[bMotorId].last_ecd = motor_chassis[bMotorId].ecd;
	motor_chassis[bMotorId].ecd = ecd_scaled;
}

HAL_StatusTypeDef encode_ktech_broadcast_speed_control(const int16_t speedControlCmd[4], uint8_t blocking_call, CAN_HandleTypeDef *hcan_ptr)
{
	can_tx_message.StdId = CAN_KTECH_BROADCAST_SPEED_TX_ID;
	can_tx_message.ExtId = 0x00;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 8;

	can_send_data[0] = (uint8_t)speedControlCmd[0];
	can_send_data[1] = (uint8_t)(speedControlCmd[0] >> 8);
	can_send_data[2] = (uint8_t)speedControlCmd[1];
	can_send_data[3] = (uint8_t)(speedControlCmd[1] >> 8);
	can_send_data[4] = (uint8_t)speedControlCmd[2];
	can_send_data[5] = (uint8_t)(speedControlCmd[2] >> 8);
	can_send_data[6] = (uint8_t)speedControlCmd[3];
	can_send_data[7] = (uint8_t)(speedControlCmd[3] >> 8);

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
		osDelay(retry_delay_ms);
	}
	return CAN_status;
}

#if TRIGGER_MOTOR_IS_4010
/**
  * @brief          send a single-motor speed command to the KTech MG4010 trigger motor (0x217) on GIMBAL_CAN.
  *                 Uses the same int16 deg/s (1 dps/LSB) speed encoding as the chassis MG4010 broadcast.
  * @param[in]      trigger_speed_cmd: motor-rotor speed command in deg/s
  * @retval         none
  */
void CAN_cmd_4010_trigger(int16_t trigger_speed_cmd)
{
#if (ENABLE_TRIGGER_MOTOR_POWER == 0)
	trigger_speed_cmd = 0;
#endif

#if REVERSE_TRIGGER_MG4010
	trigger_speed_cmd = -trigger_speed_cmd;
#endif

	int16_t cmd = (int16_t)fp32_abs_constrain((fp32)trigger_speed_cmd, MOTOR_MG4010_MAX_CMD);

	can_tx_message.StdId = CAN_TRIGGER_MOTOR_ID;
	can_tx_message.ExtId = 0x00;
	can_tx_message.IDE = CAN_ID_STD;
	can_tx_message.RTR = CAN_RTR_DATA;
	can_tx_message.DLC = 8;

	can_send_data[0] = (uint8_t)cmd;
	can_send_data[1] = (uint8_t)(cmd >> 8);
	can_send_data[2] = 0;
	can_send_data[3] = 0;
	can_send_data[4] = 0;
	can_send_data[5] = 0;
	can_send_data[6] = 0;
	can_send_data[7] = 0;

	Send_CAN_Cmd(&GIMBAL_CAN, &can_tx_message, can_send_data, 1);
}
#endif

#endif
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

void CAN_cmd_gimbal_Damiao_motor(MIT_control_motor_t *MIT_control_motor)
{	
#if ENABLE_PITCH_MOTOR_POWER

	MIT_control_variable_t pitch_variable = MIT_control_motor->pitch_MIT_variable;

#if ROBOT_PITCH_IS_4310
	encode_MIT_motor_control(CAN_PITCH_MOTOR_4310_TX_ID, pitch_variable.pos, pitch_variable.vel, pitch_variable.KP, pitch_variable.KD, pitch_variable.torq, DM_4310, &GIMBAL_CAN);
// #elif ROBOT_PITCH_IS_4340
// 	encode_MIT_motor_control(CAN_PITCH_MOTOR_4340_TX_ID, pitch_variable.pos, pitch_variable.vel, pitch_variable.KP, pitch_variable.KD, pitch_variable.torq, DM_4340, &GIMBAL_CAN);
#endif
#endif

#if (ROBOT_TYPE == INFANTRY_2026_MECANUM) && ENABLE_PITCH_BASE_MOTOR_POWER 
	MIT_control_variable_t pitch_base_variable = MIT_control_motor->pitch_base_MIT_variable;
	encode_MIT_motor_control(CAN_PITCH_BASE_MOTOR_4310_TX_ID, pitch_base_variable.pos, pitch_base_variable.vel, pitch_base_variable.KP, pitch_base_variable.KD, pitch_base_variable.torq, DM_4310, &GIMBAL_CAN);
#endif


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
		uint16_t t_int = ((data[4] & 0xF) << 8) | data[5]; // Nm

		motor_chassis[bMotorId].output_angle = uint_to_fp32_motor(p_int, MIT_CONTROL_P_MIN[DM_4340], MIT_CONTROL_P_MAX[DM_4340], 16);
		motor_chassis[bMotorId].ecd = loop_fp32_constrain(motor_chassis[bMotorId].output_angle, 0, 2 * PI) * MOTOR_RAD_TO_ECD; //no actual ecd reading used 
		motor_chassis[bMotorId].velocity = uint_to_fp32_motor(v_int, MIT_CONTROL_V_MIN[DM_4340], MIT_CONTROL_V_MAX[DM_4340], 12);
		motor_chassis[bMotorId].torque = uint_to_fp32_motor(t_int, MIT_CONTROL_T_MIN[DM_4340], MIT_CONTROL_T_MAX[DM_4340], 12);
		motor_chassis[bMotorId].temperate = data[6];

		ret_value = HAL_OK;
	}
	return ret_value;
}

HAL_StatusTypeDef decode_3507_motor_feedback(uint8_t *data, uint8_t bMotorId)
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

		motor_chassis[bMotorId].output_angle = uint_to_fp32_motor(p_int, MIT_CONTROL_P_MIN[DM_3507], MIT_CONTROL_P_MAX[DM_3507], 16);
		motor_chassis[bMotorId].ecd = loop_fp32_constrain(motor_chassis[bMotorId].output_angle, 0, 2 * PI) * MOTOR_RAD_TO_ECD; //no actual ecd reading used 
		motor_chassis[bMotorId].velocity = uint_to_fp32_motor(v_int, MIT_CONTROL_V_MIN[DM_3507], MIT_CONTROL_V_MAX[DM_3507], 12);
		motor_chassis[bMotorId].torque = uint_to_fp32_motor(t_int, MIT_CONTROL_T_MIN[DM_3507], MIT_CONTROL_T_MAX[DM_3507], 12);
		motor_chassis[bMotorId].temperate = data[6];

		ret_value = HAL_OK;
	}
	return ret_value;
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
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

#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
uint8_t decode_chassis_feedback(uint8_t *data)
{
	uint8_t fDataValid = (memcmp(data, abAllFF, sizeof(abAllFF)) != 0);
	if (fDataValid)
	{
		for (uint8_t wheel_id = 0; wheel_id < 4; wheel_id++)
		{
			chassis_move.wheel_rot_radii[wheel_id] = data[wheel_id] / chassis_meter_encoding_ratio_shrinked;
		}
		chassis_move.chassis_platform.feedback_alpha1 = data[4] / chassis_angle_encoding_ratio_shrinked;
		chassis_move.chassis_platform.feedback_alpha2 = data[5] / chassis_angle_encoding_ratio_shrinked;
		chassis_move.chassis_platform.feedback_height = data[6] / chassis_meter_encoding_ratio_shrinked;
		// data[7] reserved
	}
	return fDataValid;
}

uint8_t decode_chassis_target_radius_dot(uint8_t *data)
{
	uint8_t fDataValid = (memcmp(data, abAllFF, sizeof(abAllFF)) != 0);
	if (fDataValid)
	{
		for (uint8_t wheel_id = 0; wheel_id < 4; wheel_id++)
		{
			int16_t radius_dot = (data[2 * wheel_id + 1] << 8) | data[2 * wheel_id];
			chassis_move.target_wheel_rot_radii_dot[wheel_id] = (fp32)radius_dot / chassis_speed_encoding_ratio;
			fp32_deadzone(&chassis_move.target_wheel_rot_radii_dot[wheel_id], CHASSIS_WHEEL_ROT_RADIUS_DOT_DEADZONE);
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
	if (toe_is_error(REMOTE_TOE) == 0)
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
void CAN_cmd_gimbal_upper_can_ID(fp32 yaw, fp32 secondary_yaw, fp32 pitch, int16_t trigger, int16_t fric_left, int16_t fric_right, int16_t piston_motor)
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
#if (ENABLE_SECOND_YAW_MOTOR_POWER == 0)
	secondary_yaw = 0;
#endif
#if (ENABLE_TRIGGER_MOTOR_POWER == 0)
	trigger = 0;
#endif
#if TRIGGER_MOTOR_IS_4010
	// The MG4010 trigger motor is commanded separately via CAN_cmd_4010_trigger(); keep this legacy RM frame slot empty.
	trigger = 0;
#endif
#if (ENABLE_PITCH_MOTOR_POWER == 0)
	pitch = 0;
#endif
#if ((ENABLE_FRICTION_1_MOTOR_POWER == 0))
	fric_left = 0;
#endif
#if ((ENABLE_FRICTION_2_MOTOR_POWER == 0))
	fric_right = 0;
#endif
#if ((ENABLE_PISTON_MOTOR_POWER == 0) || (ENABLE_SHOOT_REDUNDANT_SWITCH == 0))
	piston_motor = 0;
#endif

//**************** Chassis CAN packet ******************
	// control yaw motor and trigger motor
#if ROBOT_YAW_IS_4310
	//encode and send MIT control saperately
#else
	gimbal_can_send_data[0] = ((int16_t)yaw >> 8);
	gimbal_can_send_data[1] = (int16_t)yaw;
#endif
	// gimbal_can_send_data[2] = (open >> 8);
	// gimbal_can_send_data[3] = open;
#if IS_TRIGGER_ON_GIMBAL
	// gimbal_can_send_data[4] = (open >> 8);
	// gimbal_can_send_data[5] = open;
#else
	gimbal_can_send_data[4] = (trigger >> 8);
	gimbal_can_send_data[5] = trigger;
#endif
	// gimbal_can_send_data[6] = (open >> 8);
	// gimbal_can_send_data[7] = open;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

//**************** Gimbal CAN packet *******************
	// control pitch motor and fric_left and fric_right
	gimbal_can_send_data[0] = (fric_left >> 8);
	gimbal_can_send_data[1] = fric_left;

#if (ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_3507 || ROBOT_PITCH_IS_4310)
	//encode and send MIT control saperately
	//gimbal_can_send_data[2] = (open >> 8);
	//gimbal_can_send_data[3] = open;
#else
	gimbal_can_send_data[2] = ((int16_t)pitch >> 8);
	gimbal_can_send_data[3] = (int16_t)pitch;
#endif

#if IS_TRIGGER_ON_GIMBAL
	gimbal_can_send_data[4] = (trigger >> 8);
	gimbal_can_send_data[5] = trigger;
#else
//Piston motor for hero 2023 and 2026
#if (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == HERO_2026_OMNI) //This is for the piston motor with higher can ID 7
	gimbal_can_send_data[4] = (piston_motor >> 8); //Higher 8-bit
	gimbal_can_send_data[5] = piston_motor; //Lower 8-bit
#endif

#endif
	gimbal_can_send_data[6] = (fric_right >> 8);
	gimbal_can_send_data[7] = fric_right;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);

//*************** MIT_control massage******************
#if (ROBOT_YAW_IS_4310||ROBOT_PITCH_IS_4340)
	osDelay(1);//delay 1 ms if need to send MIT cmds in same CAN bus to other standard motor
#endif

#if ROBOT_YAW_IS_4310
	encode_MIT_motor_control(CAN_YAW_MOTOR_4310_TX_ID, 0, 0, 0, 0, yaw, DM_4310, &CHASSIS_CAN);
#endif

#if (ROBOT_TYPE == HERO_2026_OMNI)
	encode_MIT_motor_control(CAN_SECOND_YAW_MOTOR_4310_TX_ID, 0, 0, 0, 0, secondary_yaw, DM_4310, &GIMBAL_CAN);
#endif

#if ROBOT_PITCH_IS_4340
	encode_MIT_motor_control(CAN_PITCH_MOTOR_4340_TX_ID, 0, 0, 0, 0, pitch, DM_4340, &GIMBAL_CAN);
#elif ROBOT_PITCH_IS_3507
	encode_MIT_motor_control(CAN_PITCH_MOTOR_3507_TX_ID, 0, 0, 0, 0, pitch, DM_3507, &GIMBAL_CAN);
#endif
//TODO: delete after test
// #if ENABLE_PITCH_MOTOR_POWER
// 	encode_MIT_motor_control(CAN_PITCH_MOTOR_4310_TX_ID, 0, 0, 0, 0, pitch, DM_4310, &GIMBAL_CAN);
// #if ENABLE_PITCH_BASE_MOTOR_POWER 
// 	encode_MIT_motor_control(CAN_PITCH_BASE_MOTOR_4310_TX_ID, 0, 0, temp_base_kp, temp_base_kd, 0, DM_4310, &GIMBAL_CAN);
// #endif

// #endif
}

void CAN_cmd_gimbal_lower_can_id(int16_t fric_up, int16_t fric_down)
{
#if ((ENABLE_FRICTION_3_MOTOR_POWER == 0) || (ENABLE_SHOOT_REDUNDANT_SWITCH == 0))
	fric_up = 0;
#endif
#if ((ENABLE_FRICTION_4_MOTOR_POWER == 0) || (ENABLE_SHOOT_REDUNDANT_SWITCH == 0))
	fric_down = 0;
#endif
	uint32_t send_mail_box;
	gimbal_tx_message.StdId = CAN_3508_OR_2006_LOW_RANGE_TX_ID;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;


	//Motor ID 1
	// gimbal_can_send_data[0] = (open >> 8);
	// gimbal_can_send_data[1] = open;

	//Motor ID 2
	// gimbal_can_send_data[2] = (open >> 8);
	// gimbal_can_send_data[3] = open;

	//Motor ID 3
	gimbal_can_send_data[4] = (fric_up >> 8);
	gimbal_can_send_data[5] = fric_up;

	//Motor ID 4
	gimbal_can_send_data[6] = (fric_down >> 8);
	gimbal_can_send_data[7] = fric_down;
	HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
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
	HAL_StatusTypeDef hal_status = HAL_CAN_AddTxMessage(hcan_ptr, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
	memset(gimbal_can_send_data, 0, sizeof(gimbal_can_send_data));
	return hal_status;
}

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
#if (WHEEL_TYPE == ROBOT_CHASSIS_USE_MECANUM) || (ROBOT_TYPE == INFANTRY_2023_SWERVE)
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
	if (chassis_move.fUpperHeadEnabled && is_game_started())
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
		uint16_t shoot_heat1_int16 = 0, projectile_allowance_17mm, gold_coins;
		get_shoot_heat1_limit_and_heat(&shoot_heat_limit, &shoot_heat1_int16);
		get_remaining_gold_coins(&gold_coins);
		get_projectile_allowance_17mm(&projectile_allowance_17mm);
		uint8_t shoot_heat_limit_uint8 = fp32_abs_constrain(shoot_heat_limit, shoot_heat_limit_max) / shoot_heat_limit_max * 255.0f;
		uint8_t shoot_heat1_uint8 = fp32_abs_constrain(shoot_heat1_int16, shoot_heat_limit_max) / shoot_heat_limit_max * 255.0f;

		uint16_t blueOutPostHP = get_blue_outpost_HP();
		uint16_t redOutPostHP = get_red_outpost_HP();

		chassis_can_send_data[0] = shoot_heat_limit_uint8;
		chassis_can_send_data[1] = shoot_heat1_uint8;
		chassis_can_send_data[2] = _bullet_speed;
		chassis_can_send_data[3] = (projectile_allowance_17mm >> 8);
		chassis_can_send_data[4] = projectile_allowance_17mm;
		chassis_can_send_data[5] = (gold_coins >> 8);
		chassis_can_send_data[6] = gold_coins;
		chassis_can_send_data[7] = (team_color << 7);
		HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	}
#endif
}
#endif

void CAN_cmd_chassis(void)
{
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
	CAN_cmd_3508_chassis();
	osDelay(1);
	CAN_cmd_swerve_steer();
	osDelay(1);
	CAN_cmd_swerve_hip();
#elif (ROBOT_TYPE == SENTRY_2026_OMNI)
	CAN_cmd_4010_chassis();
	osDelay(1);
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
	CAN_cmd_3508_chassis();
	osDelay(1);
	CAN_cmd_chassis_hip();
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
	CAN_cmd_biped_chassis();
	CAN_cmd_biped_chassis_mode();

#elif (ROBOT_TYPE == HERO_2026_OMNI)
	CAN_cmd_4010_chassis();
	osDelay(1);
#else
#if (MOTOR_TYPE == POWER_TRAIN_USE_3508_MOTOR)
	CAN_cmd_3508_chassis();
	osDelay(1);
#elif (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)
	CAN_cmd_4010_chassis();
#endif
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
#if !((ROBOT_TYPE == INFANTRY_2024_BIPED) || (ROBOT_TYPE == SENTRY_2026_OMNI))
	uint32_t send_mail_box;
	// driver motors (M3508)
	chassis_tx_message.StdId = CAN_3508_OR_2006_LOW_RANGE_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

#if ENABLE_DRIVE_MOTOR_POWER
	int16_t motor1 = chassis_move.motor_chassis[0].give_chassis_motor_cmd;
	int16_t motor2 = chassis_move.motor_chassis[1].give_chassis_motor_cmd;
	int16_t motor3 = chassis_move.motor_chassis[2].give_chassis_motor_cmd;
	int16_t motor4 = chassis_move.motor_chassis[3].give_chassis_motor_cmd;
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

#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR) || (TRIGGER_MOTOR_IS_4010)
void CAN_cmd_4010_chassis(void)
{
	uint8_t blocking_call = 1;

	// MG4010 motors are controlled via speed commands sent to individual motor IDs
#if ENABLE_DRIVE_MOTOR_POWER
	fp32 motor1_speed = chassis_move.motor_chassis[0].give_chassis_motor_cmd;
	fp32 motor2_speed = chassis_move.motor_chassis[1].give_chassis_motor_cmd;
	fp32 motor3_speed = chassis_move.motor_chassis[2].give_chassis_motor_cmd;
	fp32 motor4_speed = chassis_move.motor_chassis[3].give_chassis_motor_cmd;
#else
	fp32 motor1_speed = 0;
	fp32 motor2_speed = 0;
	fp32 motor3_speed = 0;
	fp32 motor4_speed = 0;
#endif

#if REVERSE_MG4010_1
	motor1_speed = -motor1_speed;
#endif

#if REVERSE_MG4010_2
	motor2_speed = -motor2_speed;
#endif

#if REVERSE_MG4010_3
	motor3_speed = -motor3_speed;
#endif

#if REVERSE_MG4010_4
	motor4_speed = -motor4_speed;
#endif

	// Convert speeds from rad/s to dps (deg/s) and then to control command (0.01 dps per LSB)
	// Motor speed in rad/s -> deg/s via gear ratio -> control cmd (multiply by 100 for 0.01 dps per LSB)
	int16_t motor1_cmd = (int16_t)fp32_abs_constrain((motor1_speed) , MOTOR_MG4010_MAX_CMD);
	int16_t motor2_cmd = (int16_t)fp32_abs_constrain((motor2_speed) , MOTOR_MG4010_MAX_CMD);
	int16_t motor3_cmd = (int16_t)fp32_abs_constrain((motor3_speed) , MOTOR_MG4010_MAX_CMD);
	int16_t motor4_cmd = (int16_t)fp32_abs_constrain((motor4_speed) , MOTOR_MG4010_MAX_CMD);

	// Send speed commands to each MG4010 motor using the ktech broadcast function
	int16_t motor_cmds[4] = {motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd};
	encode_ktech_broadcast_speed_control(motor_cmds, blocking_call, &CHASSIS_CAN);
	osDelay(1);
}
#endif

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
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

	chassis_tx_message.StdId = CAN_CHASSIS_CONTROLLERE_TX_ID;
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

#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
int16_t target_alpha_cmd;
uint16_t hight_cmd;
uint16_t hip_kp_cmd;
void CAN_cmd_chassis_hip(void)
{
	uint32_t send_mail_box;

	chassis_tx_message.StdId = CAN_CHASSIS_CONTROLLERE_TX_ID;
#if ENABLE_HIP_MOTOR_POWER
	if (chassis_move.fHipEnabled)
	{
		target_alpha_cmd = fp32_abs_constrain(chassis_move.chassis_platform.target_alpha, CHASSIS_ANGLE_ECD_MAX_LIMIT) * chassis_angle_encoding_ratio;
		hight_cmd = fp32_constrain(chassis_move.chassis_platform.target_height, 0, CHASSIS_METER_ECD_MAX_LIMIT) * chassis_meter_encoding_ratio;
		hip_kp_cmd = (uint16_t)(fp32_constrain(chassis_move.chassis_platform.chassis_hip_kp, HIP_MIT_PROFILE_KP_MIN, HIP_MIT_PROFILE_KP_MAX) * 10.0f);

		chassis_can_send_data[0] = target_alpha_cmd >> 8;
		chassis_can_send_data[1] = target_alpha_cmd;
		chassis_can_send_data[2] = hight_cmd >> 8;
		chassis_can_send_data[3] = hight_cmd;
		chassis_can_send_data[4] = hip_kp_cmd >> 8;
		chassis_can_send_data[5] = hip_kp_cmd;
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
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)

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
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
#if ENABLE_DRIVE_MOTOR_POWER
	chassis_move.fHipEnabled = fEnabled;
#else
	chassis_move.fHipEnabled = 0;
#endif
#endif
}

#if CAN_PASS_REF_INFO
void send_ui_info(void)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_UI_INFO_RX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

	Set_Bit(&ui_info.launcher_flag_byte,UI_TRIGGER_STATE_BIT,ui_info.trigger_state);
	Set_Bit(&ui_info.launcher_flag_byte,UI_FRIC_STATE_BIT,ui_info.firc_state);
	Set_Bit(&ui_info.launcher_flag_byte,UI_AUTO_AIM_STATE_BIT,ui_info.auto_aim_state);
	Set_Bit(&ui_info.launcher_flag_byte,UI_IGNORE_HEAT_LIMIT_BIT,ui_info.Heat_Limit_Ignored);
	Set_Bit(&ui_info.launcher_flag_byte,UI_LAUNCHER_LOADED_BIT,ui_info.Launcher_Loaded);
	Set_Bit(&ui_info.launcher_flag_byte,UI_LAUNCHER_OPENED_BIT,ui_info.Launcher_Opened);
	
	Set_Bit(&ui_info.chassis_flag_byte,UI_SPINNING_STATE_BIT,ui_info.spinning_state);
	Set_Bit(&ui_info.chassis_flag_byte,UI_POWER_SAVING_BIT,ui_info.power_saving);



	interboard_can_send_data[0] = ui_info.chassis_flag_byte;
	interboard_can_send_data[1] = ui_info.launcher_flag_byte;
	
		
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, interboard_can_send_data, &send_mail_box);
}


void pull_ref_info(uint8_t info_code)
{	
	uint32_t send_mail_box;
	chassis_tx_message.StdId = CAN_REF_INFO_PULL_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

	interboard_can_send_data[0] = info_code;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, interboard_can_send_data, &send_mail_box);
}

uint8_t temp_pmm_gimbal = 0;

void decode_ref_info(uint8_t *rx_data)
{

	uint8_t info_code = rx_data[0];

	switch (info_code)
	{
		case BARREL_HEAT_LIMIT_AND_BARREL_1_HEAT:
		{
			memcpy(&can_ref_info.barrel_heat_limit, rx_data + 1, 2);
			memcpy(&can_ref_info.barrel_1_heat, rx_data + 3, 2); //stored saperately from uart-refree data, 42mm or 17mm heat determined in lower board
			break;
		}
		
		case CHASSIS_POWER_INFO:
		{
			memcpy(&can_ref_info.chassis_power_buffer, rx_data + 1, 2);
			memcpy(&can_ref_info.chassis_power_limit, rx_data + 3, 2);
			memcpy(&can_ref_info.encoded_chassis_power, rx_data + 5, 2);
		
			robot_state.power_management_chassis_output = 1;//rx_data[7] & POWER_MANAGEMNT_CHASSIS_BIT;
			robot_state.power_management_shooter_output = 1;//rx_data[7] & POWER_MANAGEMNT_SHOOTER_BIT;
			robot_state.power_management_gimbal_output = 1;
			temp_pmm_gimbal = rx_data[7] & POWER_MANAGEMNT_GIMBAL_BIT;
			break;
		}
	
	}
}

void CAN_get_heat_limit_and_barrel_1_heat(uint16_t *heat_limit, uint16_t *heat)
{
	*heat_limit = can_ref_info.barrel_heat_limit;
	*heat = can_ref_info.barrel_1_heat;
}

void CAN_get_chassis_power_info(fp32 *buffer, fp32 *power_limit)  //safe to convert by fp32 data = uint data
{
	*buffer = can_ref_info.chassis_power_buffer;
	if (can_ref_info.chassis_power_limit > 0)
	{
		*power_limit = can_ref_info.chassis_power_limit;
	}
	else
	{
		*power_limit = 45;
	}
}
#endif


#if (SUPERCAP_TYPE == MACRM_SUPERCAP)
void CAN_cmd_supercap(void)
{
	uint32_t send_mail_box;

	chassis_tx_message.StdId = SUPCAP_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

	fp32 chassis_power_buffer;
    fp32 chassis_power_limit;
    get_chassis_power_data(&chassis_power_buffer, &chassis_power_limit);
	
	capcan_rx_msg.power_target = chassis_power_limit*100;

	chassis_can_send_data[0] = capcan_rx_msg.power_target;
	chassis_can_send_data[1] = capcan_rx_msg.power_target >> 8;
	//chassis_can_send_data[2] = capcan_rx_msg.referee_power;
	//chassis_can_send_data[3] = capcan_rx_msg.referee_power >> 8;
	//chassis_can_send_data[4] = capcan_rx_msg.rsvd1 >> 8;
	//chassis_can_send_data[5] = capcan_rx_msg.rsvd1;
	//chassis_can_send_data[6] = capcan_rx_msg.rsvd2 >> 8;
	//chassis_can_send_data[7] = capcan_rx_msg.rsvd2;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void decode_macrm_cap_tx_data(uint8_t *data)
{
	capcan_tx_msg.current_chassis_power = (data[1] << 8) | data[0];
	capcan_tx_msg.current_battery_power = (data[3] << 8) | data[2];
	capcan_tx_msg.cap_voltage = (data[5] << 8) | data[4];
	capcan_tx_msg.cap_state = (data[7] << 8) | data[6];
}

uint16_t get_current_chassis_power(void)
{
	return capcan_tx_msg.current_chassis_power;
}
uint16_t get_current_battery_power(void)
{
	return capcan_tx_msg.current_battery_power;
}
int16_t get_cap_voltage(void)
{
	return capcan_tx_msg.cap_voltage;
}
uint16_t get_cap_state(void)
{
	return capcan_tx_msg.cap_state;
}
#endif

#if (SUPERCAP_TYPE == UBC_SUPERCAP)
void CAN_cmd_supercap(void)
{
	uint32_t send_mail_box;

	chassis_tx_message.StdId = SUPCAP_TX_ID;
	chassis_tx_message.IDE = CAN_ID_STD;
	chassis_tx_message.RTR = CAN_RTR_DATA;
	chassis_tx_message.DLC = 0x08;

	fp32 chassis_power_buffer;
	fp32 chassis_power_limit;
	fp32 chassis_power_raw;
	chassis_power_raw = get_chassis_power_meter_data();
	get_chassis_power_data(&chassis_power_buffer, &chassis_power_limit);
	capcan_rx_msg.power_target = chassis_power_limit*100;
	capcan_rx_msg.referee_power = chassis_power_raw * 100;
	capcan_rx_msg.rsvd1 = 0x2012;
	capcan_rx_msg.rsvd2 = 0x0712;

	chassis_can_send_data[0] = capcan_rx_msg.power_target;
	chassis_can_send_data[1] = capcan_rx_msg.power_target >> 8;
	chassis_can_send_data[2] = capcan_rx_msg.referee_power;
	chassis_can_send_data[3] = capcan_rx_msg.referee_power >> 8;
	chassis_can_send_data[4] = capcan_rx_msg.rsvd1 >> 8;
	chassis_can_send_data[5] = capcan_rx_msg.rsvd1;
	chassis_can_send_data[6] = capcan_rx_msg.rsvd2 >> 8;
	chassis_can_send_data[7] = capcan_rx_msg.rsvd2;
	HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void decode_ubc_cap_tx_data(uint8_t *data)
{
	capcan_tx_msg.max_discharge_power = (data[1] << 8) | data[0];
	capcan_tx_msg.base_power = (data[3] << 8) | data[2];
	capcan_tx_msg.cap_energy_percentage = (data[5] << 8) | data[4];
	capcan_tx_msg.cap_state = (data[7] << 8) | data[6];
}

uint16_t get_max_discharge_power(void)
{
	return capcan_tx_msg.max_discharge_power;
}
uint16_t get_current_chassis_power(void)
{
	return capcan_tx_msg.base_power;
}
int16_t get_cap_energy_percentage(void)
{
	return capcan_tx_msg.cap_energy_percentage;
}
uint16_t get_cap_state(void)
{
	return capcan_tx_msg.cap_state;
}
#endif

void decode_supercap(uint8_t *data)
{
#if (SUPERCAP_TYPE == SJTU_SUPERCAP)
	memcpy(cap_message_rx.can_buf, rx_data, sizeof(rx_data));
#elif (SUPERCAP_TYPE == MACRM_SUPERCAP)
	decode_macrm_cap_tx_data(data);
#elif (SUPERCAP_TYPE == UBC_SUPERCAP)
	decode_ubc_cap_tx_data(data);
#endif
	detect_hook(SUPCAP_TOE);
}

void decode_power_meter(uint8_t *data)
{
    power_meter_can_rx_msg.chassis_current = (fp32)((int32_t)((data[3] << 8) | (int32_t)(data[2]))) / 100.0f;
	power_meter_can_rx_msg.chassis_voltage = (fp32)((int32_t)((data[1] << 8) | (int32_t)data[0])) / 100.0f;
	power_meter_can_rx_msg.chassis_power = power_meter_can_rx_msg.chassis_current * power_meter_can_rx_msg.chassis_voltage;

}

fp32 get_chassis_power_meter_data(void)
{
	return power_meter_can_rx_msg.chassis_power;
}
