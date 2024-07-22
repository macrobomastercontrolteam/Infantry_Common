/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. finished
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "global_inc.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include "calibrate_task.h"

#define SPINNING_CHASSIS_MAX_OMEGA RPM_TO_RADS(60.0f)
#define SPINNING_CHASSIS_HIGH_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.833f)
#define SPINNING_CHASSIS_MED_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.667f)
#define SPINNING_CHASSIS_LOW_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.583f)
#define SPINNING_CHASSIS_ULTRA_LOW_OMEGA (SPINNING_CHASSIS_MAX_OMEGA * 0.167f)

#define SPINNING_CHASSIS_RANDOM_OMEGA_MIN SPINNING_CHASSIS_MED_OMEGA
#define SPINNING_CHASSIS_RANDOM_OMEGA_MAX SPINNING_CHASSIS_MAX_OMEGA
#define SPINNING_CHASSIS_RANDOM_OMEGA_PARAM_A ((SPINNING_CHASSIS_RANDOM_OMEGA_MAX - SPINNING_CHASSIS_RANDOM_OMEGA_MIN) / 2.0f)
#define SPINNING_CHASSIS_RANDOM_OMEGA_PARAM_B ((SPINNING_CHASSIS_RANDOM_OMEGA_MAX + SPINNING_CHASSIS_RANDOM_OMEGA_MIN) / 2.0f)

// in the beginning of task ,wait a time
#define CHASSIS_TASK_INIT_TIME 357

#define CHASSIS_ACCEL_WZ_NUM 0.06f
#define CHASSIS_ACCEL_X_NUM 0.5f
#define CHASSIS_ACCEL_Y_NUM 0.5f

// joystick value deadline
#define CHASSIS_RC_DEADLINE 20

#define CHASSIS_TEST_MODE 0

// swerve chassis platform parameters
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define CHASSIS_A_LENGTH 0.322815f
#define CHASSIS_HALF_A_LENGTH (CHASSIS_A_LENGTH / 2.0f)
#define CHASSIS_L1_LENGTH 0.12f
#define CHASSIS_L2_LENGTH 0.177353f
#define CHASSIS_THETA_LOWER_LIMIT DEG_TO_RAD(16.1f)
#define CHASSIS_THETA_UPPER_LIMIT DEG_TO_RAD(68.28f)
#define CHASSIS_THETA_LOWER_LIMIT_ECD (CHASSIS_THETA_LOWER_LIMIT * MG6012_MOTOR_RAD_TO_ECD)
#define CHASSIS_THETA_UPPER_LIMIT_ECD (CHASSIS_THETA_UPPER_LIMIT * MG6012_MOTOR_RAD_TO_ECD)

// calculated by Matlab offline
#define CHASSIS_H_LOWER_LIMIT 0.213246f
#define CHASSIS_H_UPPER_LIMIT 0.28699f
#define CHASSIS_H_PREINSPTION_LIMIT 0.276f
#define CHASSIS_H_WORKSPACE_PEAK 0.2518985f
#define CHASSIS_ALPHA_WORKSPACE_PEAK 0.156f
#define CHASSIS_H_WORKSPACE_SLOPE1 0.247767f
#define CHASSIS_H_WORKSPACE_SLOPE2 (-0.224946f)
// @TODO: calculate for roll and pitch limits
#define CHASSIS_ROLL_UPPER_LIMIT (PI / 4.0f)
#define CHASSIS_PITCH_UPPER_LIMIT (PI / 4.0f)
#endif

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define MOTOR_DISTANCE_TO_CENTER_DEFAULT 0.2788f
#elif (ROBOT_TYPE == INFANTRY_2024_MECANUM)
#define MOTOR_DISTANCE_TO_CENTER_DEFAULT 0.25010678f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
// angle going from the right direction to front-right leg
#define CHASSIS_LEG_TO_HORIZONTAL_ANGLE (PI / 4.0f)
#define MOTOR_DISTANCE_TO_CENTER_DEFAULT (CHASSIS_HALF_A_LENGTH + CHASSIS_L1_LENGTH * AHRS_cosf(CHASSIS_THETA_LOWER_LIMIT))
#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define MOTOR_DISTANCE_TO_CENTER_DEFAULT 0.3259247634040715f
#endif

#define CHASSIS_CONTROL_TIME_MS 5.0f
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)
#define CHASSIS_CONTROL_FREQUENCE (1.0f / CHASSIS_CONTROL_TIME_S)

// chassis 3508 max motor control current
#define MAX_3508_MOTOR_CAN_CURRENT 16000.0f
// chassis 6020 max motor control voltage
#define MAX_MOTOR_CAN_VOLTAGE 20000.0f
// // press the key, chassis will swing
// #define SWING_KEY KEY_PRESSED_OFFSET_CTRL
// chassi forward, back, left, right key
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

// drive wheel parameters
#define M3508_MOTOR_GEAR_RATIO (3591.0f / 187.0f)
#if (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define DRIVE_WHEEL_RADIUS 0.0785f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define DRIVE_WHEEL_RADIUS 0.055f
#endif
// Ratio of M3508 speed in rpm to chassis speed in m/s
#define M3508_MOTOR_RPM_TO_VECTOR ((2.0f * PI / 60.0f) * DRIVE_WHEEL_RADIUS / M3508_MOTOR_GEAR_RATIO)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

// single chassis motor max speed
#define MAX_WHEEL_SPEED 4.0f
// chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
#define SPRINT_MAX_CHASSIS_SPEED_X 5.0f
// chassis left or right max speed
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.0f
#define SPRINT_MAX_CHASSIS_SPEED_Y 5.0f
#define NORMAL_MAX_CHASSIS_SPEED_WZ SPINNING_CHASSIS_MAX_OMEGA

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
// avoid changing angle too often near zero speed
#define STEER_TURN_X_SPEED_DEADZONE 0.01f
#define STEER_TURN_Y_SPEED_DEADZONE (STEER_TURN_X_SPEED_DEADZONE * NORMAL_MAX_CHASSIS_SPEED_Y / NORMAL_MAX_CHASSIS_SPEED_X)
#define STEER_TURN_W_SPEED_DEADZONE 0.01f
// Measured approx max in normal mode: 0.159f
#define STEER_TURN_HIP_RADIUS_SPEED_DEADZONE 0.01f
#endif

// In follow-yaw mode, map joystick value to increment in target yaw angle
#define CHASSIS_ANGLE_Z_RC_CHANGE_TIME_S 1.0f
#define CHASSIS_ANGLE_Z_RC_SEN_INC (PI / 2.0f / CHASSIS_ANGLE_Z_RC_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S / JOYSTICK_HALF_RANGE)
// In not-follow-yaw mode, map joystick value to target yaw speed
#define CHASSIS_WZ_RC_SEN (NORMAL_MAX_CHASSIS_SPEED_WZ / JOYSTICK_HALF_RANGE)
// map rc dial value (max 660) to spinning speed (rad/s)
#define CHASSIS_SPIN_RC_OFFSET SPINNING_CHASSIS_LOW_OMEGA
#define CHASSIS_SPIN_RC_SEN_POSITIVE_INPUT ((NORMAL_MAX_CHASSIS_SPEED_WZ - CHASSIS_SPIN_RC_OFFSET) / JOYSTICK_HALF_RANGE)
#define CHASSIS_SPIN_RC_SEN_NEGATIVE_INPUT ((NORMAL_MAX_CHASSIS_SPEED_WZ + CHASSIS_SPIN_RC_OFFSET) / JOYSTICK_HALF_RANGE)

// configuration for pseudo RPY-to-tilt conversion
#define SWERVE_HIP_ROLL_KEYBOARD_CHANGE_TIME_S 0.4f
#define SWERVE_HIP_ROLL_KEYBOARD_SEN_INC (2.0f * CHASSIS_ALPHA_WORKSPACE_PEAK / SWERVE_HIP_ROLL_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define SWERVE_HIP_PITCH_KEYBOARD_CHANGE_TIME_S 0.4f
#define SWERVE_HIP_PITCH_KEYBOARD_SEN_INC (2.0f * CHASSIS_ALPHA_WORKSPACE_PEAK / SWERVE_HIP_ROLL_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)
#define SWERVE_HIP_HEIGHT_KEYBOARD_CHANGE_TIME_S 0.4f
#define SWERVE_HIP_HEIGHT_KEYBOARD_SEN_INC ((CHASSIS_H_UPPER_LIMIT - CHASSIS_H_LOWER_LIMIT) / SWERVE_HIP_ROLL_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S)

// Arbitrary offsets between chassis rotational center and centroid
#if ROBOT_YAW_HAS_SLIP_RING
// slip ring is at the center of chassis
#define CHASSIS_WZ_SET_SCALE 0.0f
#else
// Offset for the official model
#define CHASSIS_WZ_SET_SCALE 0.1f
#endif

// when chassis is not set to move, swing max angle
#define SWING_NO_MOVE_ANGLE 0.7f
// when chassis is set to move, swing max angle
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

// chassis motor speed PID
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
#define M3508_MOTOR_SPEED_PID_KP 35000.0f
#define M3508_MOTOR_SPEED_PID_KI 1000.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_3508_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#elif (ROBOT_TYPE == INFANTRY_2023_MECANUM)
#define M3508_MOTOR_SPEED_PID_KP 30000.0f
#define M3508_MOTOR_SPEED_PID_KI 500.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_3508_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#else
// @TODO: tune pid for other robots
// @TODO: fix drift in spinning mode when power is limited
#define M3508_MOTOR_SPEED_PID_KP 35000.0f
#define M3508_MOTOR_SPEED_PID_KI 1000.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_3508_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
#endif

// chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0002f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

typedef enum
{
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,    // chassis will follow yaw gimbal motor relative angle (this mode is not stable nor useful, but it may enlighten you on how to make new modes)
	CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,   // chassis will have yaw angle(chassis_yaw) close-looped control
	CHASSIS_VECTOR_NO_FOLLOW_YAW,        // chassis will have rotation speed control
	CHASSIS_VECTOR_RAW,                  // control-current will be sent to CAN bus derectly.
	CHASSIS_VECTOR_SPINNING,             // spinning chassis
} chassis_mode_e;

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	fp32 accel;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
} chassis_motor_t;

typedef struct
{
	fp32 target_roll;
	fp32 target_pitch;
	fp32 target_height;

	fp32 target_alpha1;
	fp32 target_alpha2;

	fp32 alpha_lower_limit;
	fp32 alpha_upper_limit;

	fp32 feedback_alpha1;
	fp32 feedback_alpha2;
	fp32 feedback_height;
} chassis_platform_t;

typedef union
{
	uint8_t can_buf[8];
	struct
	{
		// 0: not provide power
		// 1: provide power
		uint8_t cap_state;
		uint8_t reserve;
		uint16_t cap_milivoltage;
		float cap_power;
	} cap_message;
} supcap_t;

extern supcap_t cap_message_rx;

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
typedef struct
{
	uint16_t target_ecd; ///< unit encoder unit; range is [0, 8191]; positive direction is clockwise; forward direction of chassis is 0 ecd
} chassis_steer_motor_t;
#endif

typedef struct
{
	const RC_ctrl_t *chassis_RC;               // the point to remote control
	const gimbal_motor_t *chassis_yaw_motor;   // will use the relative angle of yaw gimbal motor to calculate the euler angle
	const gimbal_motor_t *chassis_pitch_motor; // will use the relative angle of pitch gimbal motor to calculate the euler angle
	const fp32 *chassis_INS_angle;             // the point to the euler angle of gyro sensor
	chassis_mode_e chassis_mode;               // state machine
	chassis_mode_e last_chassis_mode;          // last state machine
	chassis_motor_t motor_chassis[4];          // chassis motor data
	pid_type_def motor_speed_pid[4];           // motor speed PID
	pid_type_def chassis_angle_pid;            // follow angle PID
	fp32 wheel_rot_radii[4];
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
	fp32 target_wheel_rot_radii_dot[4];
	chassis_steer_motor_t steer_motor_chassis[4]; // chassis steering motor data
	chassis_platform_t chassis_platform;
	uint8_t fHipEnabled;
	uint8_t fHipDisabledEdge;
#endif

	first_order_filter_type_t chassis_cmd_slow_set_vx; // use first order filter to slow set-point
	first_order_filter_type_t chassis_cmd_slow_set_vy; // use first order filter to slow set-point
	first_order_filter_type_t chassis_cmd_slow_set_wz; // use first order filter to slow set-point

#if !(ROBOT_TYPE == INFANTRY_2023_SWERVE)
	fp32 vx; // chassis vertical speed, positive means forward,unit m/s
	fp32 vy; // chassis horizontal speed, positive means letf,unit m/s
	fp32 wz; // chassis rotation speed, positive means counterclockwise,unit rad/s
#endif
	fp32 vx_set;                     // chassis set vertical speed,positive means forward,unit m/s
	fp32 vy_set;                     // chassis set horizontal speed,positive means left,unit m/s
	fp32 wz_set;                     // chassis set rotation speed,positive means counterclockwise,unit rad/s
	fp32 chassis_relative_angle_set; // the set relative angle
	fp32 chassis_yaw_set;

	fp32 vx_max_speed; // max forward speed, unit m/s
	fp32 vx_min_speed; // max backward speed, unit m/s
	fp32 vy_max_speed; // max letf speed, unit m/s
	fp32 vy_min_speed; // max right speed, unit m/s
	fp32 vx_rc_sen;    // map joystick value to vertical speed
	fp32 vy_rc_sen;    // map joystick value to horizontal speed

	// @TODO: Currently wz limits are only enforced in CHASSIS_SPINNING, extend that to other modes
	fp32 wz_max_speed; // max spinning speed, unit rad/s.
	fp32 wz_min_speed; // min spinning speed, unit rad/s.

	fp32 chassis_yaw;   // the yaw angle calculated by gyro sensor and gimbal motor
	fp32 chassis_pitch; // the pitch angle calculated by gyro sensor and gimbal motor
	fp32 chassis_roll;  // the roll angle calculated by gyro sensor and gimbal motor

	uint8_t fRandomSpinOn;
	int16_t dial_channel_latched;
	int16_t dial_channel_out;

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	uint8_t fUpperHeadEnabled;
#endif
} chassis_move_t;

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
extern void chassis_task(void const *pvParameters);

/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
 * @retval         none
 */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
void swerve_platform_rc_mapping(void);
void chassis_swerve_back_home(void);
#endif
void swerve_chassis_params_reset(void);

extern chassis_move_t chassis_move;

#endif
