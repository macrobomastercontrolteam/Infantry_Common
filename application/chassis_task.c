/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "AHRS_middleware.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "pid.h"
#include "user_lib.h"
#include <assert.h>

#define CHASSIS_TASK_INIT_TIME 357
#define CHASSIS_CONTROL_TIME_MS 5.0f
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)

#define M6020_MOTOR_ANGLE_PID_KP 80.0f
// #define M6020_MOTOR_ANGLE_PID_KI 1.5f
// #define M6020_MOTOR_ANGLE_PID_KD 2.5f
#define M6020_MOTOR_ANGLE_PID_KI 300.0f
#define M6020_MOTOR_ANGLE_PID_KD 0.0125f
#define M6020_MOTOR_ANGLE_PID_MAX_OUT M6020_MAX_VOLTAGE
#define M6020_MOTOR_ANGLE_PID_MAX_IOUT 10000.0f

// MG6012 hip motor configs
#define MG6012_MOTOR_ANGLE_PID_KP 0.001f
#define MG6012_MOTOR_ANGLE_PID_KI 0.0f
#define MG6012_MOTOR_ANGLE_PID_KD 0.0f
#define MG6012_MOTOR_ANGLE_PID_MAX_OUT 10.0f
#define MG6012_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

#define MG6012_MOTOR_SPEED_PID_KP 5.5f
// #define MG6012_MOTOR_SPEED_PID_KI 0.125f
#define MG6012_MOTOR_SPEED_PID_KI 25.0f
#define MG6012_MOTOR_SPEED_PID_KD 0.0f
#define MG6012_MOTOR_SPEED_PID_MAX_OUT MG6012_MAX_TORQUE
#define MG6012_MOTOR_SPEED_PID_MAX_IOUT MG6012_MAX_TORQUE

#define ROTATE_6020_OFFSET 0
#define M6020_MOTOR_0_ANGLE_ECD_OFFSET ((1679U + ROTATE_6020_OFFSET) % M6020_ECD_RANGE)
#define M6020_MOTOR_1_ANGLE_ECD_OFFSET ((5111U + ROTATE_6020_OFFSET) % M6020_ECD_RANGE)
#define M6020_MOTOR_2_ANGLE_ECD_OFFSET ((4447U + ROTATE_6020_OFFSET) % M6020_ECD_RANGE)
#define M6020_MOTOR_3_ANGLE_ECD_OFFSET ((7769U + ROTATE_6020_OFFSET) % M6020_ECD_RANGE)

#define MG6012_MOTOR_0_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD + 1)
#define MG6012_MOTOR_1_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD + 87)
#define MG6012_MOTOR_2_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD + 56)
#define MG6012_MOTOR_3_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD + 174)

#define CHASSIS_TEST_MODE 1

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

chassis_move_t chassis_move;

void param_asserts(void);
void chassis_init(void);
void chassis_calc_feedbacks(void);
void chassis_calc_targets(void);
void chassis_inv_kine_diagonal(fp32 alpha, fp32 height, fp32 *theta_right, fp32 *theta_left);
void chassis_safe_guard(void);

float M6020_abs_angle_pid_calc(pid_type_def *pid, float feedback_abs_ecd_fp32, float target_ecd);
float MG6012_abs_angle_pid_calc(pid_type_def *pid, float feedback_abs_ecd_fp32, float target_ecd);

#if CHASSIS_TEST_MODE
// fp32 hip1_angle;
// fp32 hip2_angle;
// fp32 hip3_angle;
// fp32 hip4_angle;
// fp32 height_fp32;
// fp32 alpha1_fp32;
// fp32 alpha2_fp32;
// fp32 target_height_fp32;
// fp32 target_alpha1_fp32;
// fp32 target_alpha2_fp32;
// fp32 target_theta1_fp32;
// fp32 target_theta2_fp32;
// fp32 target_theta3_fp32;
// fp32 target_theta4_fp32;
// fp32 rot_radius1_fp32;
// fp32 rot_radius2_fp32;
// fp32 rot_radius3_fp32;
// fp32 rot_radius4_fp32;
// fp32 rot_radius1_dot_fp32;
// fp32 rot_radius2_dot_fp32;
// fp32 rot_radius3_dot_fp32;
// fp32 rot_radius4_dot_fp32;
// fp32 current_theta1_fp32;
// fp32 current_theta2_fp32;
// fp32 current_theta3_fp32;
// fp32 current_theta4_fp32;
// fp32 target_theta1_dot_fp32;
// fp32 target_theta2_dot_fp32;
// fp32 target_theta3_dot_fp32;
// fp32 target_theta4_dot_fp32;
// fp32 set_torque3_fp32;
static void J_scope_chassis_test(void)
{
	// hip1_angle = motor_info[CHASSIS_ID_HIP_1].feedback_abs_angle * 180.0f / PI;
	// hip2_angle = motor_info[CHASSIS_ID_HIP_2].feedback_abs_angle * 180.0f / PI;
	// hip3_angle = motor_info[CHASSIS_ID_HIP_3].feedback_abs_angle * 180.0f / PI;
	// hip4_angle = motor_info[CHASSIS_ID_HIP_4].feedback_abs_angle * 180.0f / PI;

	// height_fp32 = chassis_move.height * 1000.0f;
	// alpha1_fp32 = chassis_move.current_alpha1 * 180.0f / PI;
	// alpha2_fp32 = chassis_move.current_alpha2 * 180.0f / PI;
	// target_height_fp32 = chassis_move.target_height * 1000.0f;
	// target_alpha1_fp32 = chassis_move.target_alpha1 * 180.0f / PI;
	// target_alpha2_fp32 = chassis_move.target_alpha2 * 180.0f / PI;

	// target_theta1_fp32 = chassis_move.target_theta[0] * 180.0f / PI;
	// target_theta2_fp32 = chassis_move.target_theta[1] * 180.0f / PI;
	// target_theta3_fp32 = chassis_move.target_theta[2] * 180.0f / PI;
	// target_theta4_fp32 = chassis_move.target_theta[3] * 180.0f / PI;

	// rot_radius1_fp32 = chassis_move.wheel_rot_radius[0] * 1000.0f;
	// rot_radius2_fp32 = chassis_move.wheel_rot_radius[1] * 1000.0f;
	// rot_radius3_fp32 = chassis_move.wheel_rot_radius[2] * 1000.0f;
	// rot_radius4_fp32 = chassis_move.wheel_rot_radius[3] * 1000.0f;

	// rot_radius1_dot_fp32 = chassis_move.target_wheel_rot_radius_dot[0] * 1000.0f;
	// rot_radius2_dot_fp32 = chassis_move.target_wheel_rot_radius_dot[1] * 1000.0f;
	// rot_radius3_dot_fp32 = chassis_move.target_wheel_rot_radius_dot[2] * 1000.0f;
	// rot_radius4_dot_fp32 = chassis_move.target_wheel_rot_radius_dot[3] * 1000.0f;

	// target_theta1_dot_fp32 = chassis_move.target_theta_dot[0] * 180.0f / PI;
	// target_theta2_dot_fp32 = chassis_move.target_theta_dot[1] * 180.0f / PI;
	// target_theta3_dot_fp32 = chassis_move.target_theta_dot[2] * 180.0f / PI;
	// target_theta4_dot_fp32 = chassis_move.target_theta_dot[3] * 180.0f / PI;

	// current_theta1_fp32 = motor_info[CHASSIS_ID_HIP_1].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;
	// current_theta2_fp32 = motor_info[CHASSIS_ID_HIP_2].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;
	// current_theta3_fp32 = motor_info[CHASSIS_ID_HIP_3].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;
	// current_theta4_fp32 = motor_info[CHASSIS_ID_HIP_4].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;

	// set_torque3_fp32 = motor_info[CHASSIS_ID_HIP_3].set_torque * 50.0f;
}
#endif

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
	uint32_t ulSystemTime = osKernelSysTick();
	uint8_t bMotorId;
	uint8_t bMotorRelativeId;

	param_asserts();
	chassis_init();
	osDelay(CHASSIS_TASK_INIT_TIME);

	while (1)
	{
		chassis_calc_feedbacks();
		chassis_calc_targets();
		chassis_safe_guard();

		// @TODO: reenable this after fixing fatal error
		// if (chassis_move.fFatalError)
		// {
		// 	chassis_move.fSteerMotorEnabled = 0;
		// 	chassis_move.fHipMotorEnabled = 0;
		// }

		// PID calculation
		for (bMotorId = 0; bMotorId < CHASSIS_ID_LAST; bMotorId++)
		{
			switch (bMotorId)
			{
				case CHASSIS_ID_STEER_1:
				case CHASSIS_ID_STEER_2:
				case CHASSIS_ID_STEER_3:
				case CHASSIS_ID_STEER_4:
				{
					bMotorRelativeId = bMotorId - CHASSIS_ID_STEER_1;
					motor_info[bMotorId].set_voltage = PID_calc(&chassis_move.steer_angle_pid[bMotorRelativeId], motor_info[bMotorId].feedback_abs_ecd_fp32, motor_info[bMotorId].target_ecd, CHASSIS_CONTROL_TIME_S);
					break;
				}
				case CHASSIS_ID_HIP_1:
				case CHASSIS_ID_HIP_2:
				case CHASSIS_ID_HIP_3:
				case CHASSIS_ID_HIP_4:
				{
					bMotorRelativeId = bMotorId - CHASSIS_ID_HIP_1;
					chassis_move.target_theta_dot[bMotorRelativeId] = PID_calc(&chassis_move.hip_angle_pid[bMotorRelativeId], motor_info[bMotorId].feedback_abs_ecd_fp32, chassis_move.target_theta[bMotorRelativeId] * MG6012_MOTOR_RAD_TO_ECD, CHASSIS_CONTROL_TIME_S);
					motor_info[bMotorId].set_torque = PID_calc(&chassis_move.hip_speed_pid[bMotorRelativeId], DEG_TO_RAD(motor_info[bMotorId].rotor_speed), chassis_move.target_theta_dot[bMotorRelativeId], CHASSIS_CONTROL_TIME_S);
					break;
				}
				default:
				{
					break;
				}
			}
		}

		/* send motor control message through can bus*/
		CAN_cmd_steer_motors(0, motor_info[CHASSIS_ID_STEER_1].set_voltage, motor_info[CHASSIS_ID_STEER_2].set_voltage, motor_info[CHASSIS_ID_STEER_3].set_voltage, motor_info[CHASSIS_ID_STEER_4].set_voltage);
		CAN_send_shrinked_params_to_upper_board(chassis_move.wheel_rot_radius[0], chassis_move.wheel_rot_radius[1], chassis_move.wheel_rot_radius[2], chassis_move.wheel_rot_radius[3], chassis_move.current_alpha1, chassis_move.current_alpha2, chassis_move.height);
		osDelay(1);
		CAN_cmd_hip_motors(motor_info[CHASSIS_ID_HIP_1].set_torque, motor_info[CHASSIS_ID_HIP_2].set_torque, motor_info[CHASSIS_ID_HIP_3].set_torque, motor_info[CHASSIS_ID_HIP_4].set_torque);
		CAN_send_radius_dot_to_upper_board(chassis_move.target_wheel_rot_radius_dot[0], chassis_move.target_wheel_rot_radius_dot[1], chassis_move.target_wheel_rot_radius_dot[2], chassis_move.target_wheel_rot_radius_dot[3]);

		osDelayUntil(&ulSystemTime, CHASSIS_CONTROL_TIME_MS);

#if CHASSIS_TEST_MODE
		J_scope_chassis_test();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

void param_asserts(void)
{
	assert(CHASSIS_HALF_A_LENGTH + CHASSIS_L1_LENGTH <= METER_ENCODER_MAX_LIMIT);
	assert(CHASSIS_H_UPPER_LIMIT <= METER_ENCODER_MAX_LIMIT);

	assert(CHASSIS_H_LOWER_LIMIT < CHASSIS_H_UPPER_LIMIT);
	assert(CHASSIS_H_LOWER_LIMIT >= CHASSIS_L2_LENGTH);
	assert(CHASSIS_H_UPPER_LIMIT <= CHASSIS_L1_LENGTH + CHASSIS_L2_LENGTH);
	assert((CHASSIS_H_WORKSPACE_PEAK < CHASSIS_H_UPPER_LIMIT) && (CHASSIS_H_WORKSPACE_PEAK > CHASSIS_H_LOWER_LIMIT));
	assert((CHASSIS_H_WORKSPACE_SLOPE1 > 0) && (CHASSIS_H_WORKSPACE_SLOPE1 < 1));
	assert((CHASSIS_H_WORKSPACE_SLOPE2 < 0) && (CHASSIS_H_WORKSPACE_SLOPE2 > -1));
}

void chassis_init(void)
{
	uint8_t bMotorId;
	uint8_t bMotorRelativeId;
	const uint16_t steer_motor_offset_ecd[4] = {M6020_MOTOR_0_ANGLE_ECD_OFFSET, M6020_MOTOR_1_ANGLE_ECD_OFFSET, M6020_MOTOR_2_ANGLE_ECD_OFFSET, M6020_MOTOR_3_ANGLE_ECD_OFFSET};
	const fp32 steer_angle_pid_params[3] = {M6020_MOTOR_ANGLE_PID_KP, M6020_MOTOR_ANGLE_PID_KI, M6020_MOTOR_ANGLE_PID_KD};
	for (bMotorRelativeId = 0; bMotorRelativeId < STEER_MOTOR_COUNT; bMotorRelativeId++)
	{
		bMotorId = bMotorRelativeId + CHASSIS_ID_STEER_1;
		PID_init(&chassis_move.steer_angle_pid[bMotorRelativeId], PID_POSITION, steer_angle_pid_params, M6020_MOTOR_ANGLE_PID_MAX_OUT, M6020_MOTOR_ANGLE_PID_MAX_IOUT, 0, &M6020_ecd_err_handler);
		motor_info[bMotorId].offset_ecd = steer_motor_offset_ecd[bMotorRelativeId];
	}

	const uint16_t hip_motor_offset_ecd[4] = {MG6012_MOTOR_0_ANGLE_ECD_OFFSET, MG6012_MOTOR_1_ANGLE_ECD_OFFSET, MG6012_MOTOR_2_ANGLE_ECD_OFFSET, MG6012_MOTOR_3_ANGLE_ECD_OFFSET};
	const fp32 hip_angle_pid_params[3] = {MG6012_MOTOR_ANGLE_PID_KP, MG6012_MOTOR_ANGLE_PID_KI, MG6012_MOTOR_ANGLE_PID_KD};
	const fp32 hip_speed_pid_params[3] = {MG6012_MOTOR_SPEED_PID_KP, MG6012_MOTOR_SPEED_PID_KI, MG6012_MOTOR_SPEED_PID_KD};
	for (bMotorRelativeId = 0; bMotorRelativeId < HIP_MOTOR_COUNT; bMotorRelativeId++)
	{
		bMotorId = bMotorRelativeId + CHASSIS_ID_HIP_1;
		PID_init(&chassis_move.hip_angle_pid[bMotorRelativeId], PID_POSITION, hip_angle_pid_params, MG6012_MOTOR_ANGLE_PID_MAX_OUT, MG6012_MOTOR_ANGLE_PID_MAX_IOUT, 0, &MG6012_ecd_err_handler);
		PID_init(&chassis_move.hip_speed_pid[bMotorRelativeId], PID_POSITION, hip_speed_pid_params, MG6012_MOTOR_SPEED_PID_MAX_OUT, MG6012_MOTOR_SPEED_PID_MAX_IOUT, 0.8f, &filter_err_handler);
		motor_info[bMotorId].offset_ecd = hip_motor_offset_ecd[bMotorRelativeId];
	}

	chassis_move.target_alpha1 = 0;
	chassis_move.target_alpha2 = 0;
#if HEADLESS_HIP_TEST
	// chassis_move.target_height = CHASSIS_H_WORKSPACE_PEAK;
	chassis_move.target_height = CHASSIS_H_UPPER_LIMIT;
#else
	chassis_move.target_height = CHASSIS_H_LOWER_LIMIT;
#endif
	chassis_move.current_alpha1 = 0;
	chassis_move.current_alpha2 = 0;
	chassis_move.height = CHASSIS_H_LOWER_LIMIT;
	chassis_move.alpha_lower_limit = 0;
	chassis_move.alpha_upper_limit = 0;

	for (bMotorId = 0; bMotorId < STEER_MOTOR_COUNT; bMotorId++)
	{
		chassis_move.wheel_rot_radius[bMotorId] = 0;
		chassis_move.target_wheel_rot_radius_dot[bMotorId] = 0;
		chassis_move.target_theta[bMotorId] = CHASSIS_THETA_LOWER_LIMIT;
		chassis_move.target_theta_dot[bMotorId] = 0;
	}

	chassis_move.fSteerMotorEnabled = 0;
	chassis_move.fHipMotorEnabled = 0;
	chassis_move.fFatalError = 0;
	chassis_move.fHipDataIsValid = 0;
}

void chassis_calc_feedbacks(void)
{
	fp32 current_theta1 = motor_info[CHASSIS_ID_HIP_1].feedback_abs_angle;
	fp32 current_theta2 = motor_info[CHASSIS_ID_HIP_2].feedback_abs_angle;
	fp32 current_theta3 = motor_info[CHASSIS_ID_HIP_3].feedback_abs_angle;
	fp32 current_theta4 = motor_info[CHASSIS_ID_HIP_4].feedback_abs_angle;

	// @TODO: calculate for chassis_move.current_alpha1 and chassis_move.current_alpha2 using IMU data
	fp32 raw_roll = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);
	fp32 raw_pitch = *(INS_angle + INS_PITCH_ADDRESS_OFFSET);
	chassis_move.current_alpha1 = -rad_format(raw_roll + PI);
	chassis_move.current_alpha2 = -raw_pitch;

	// calculation for chassis_move.height: some legs may not be on the ground, so pick the largest height calculated
	// right diagonal
	fp32 temp_heights[4];
	temp_heights[0] = (CHASSIS_L1_LENGTH * AHRS_cosf(current_theta1) + CHASSIS_HALF_A_LENGTH) * AHRS_sinf(chassis_move.current_alpha1) + (CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta1)) * AHRS_cosf(chassis_move.current_alpha1);
	temp_heights[2] = -(CHASSIS_L1_LENGTH * AHRS_cosf(current_theta3) + CHASSIS_HALF_A_LENGTH) * AHRS_sinf(chassis_move.current_alpha1) + (CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta3)) * AHRS_cosf(chassis_move.current_alpha1);
	// left diagonal
	temp_heights[3] = (CHASSIS_L1_LENGTH * AHRS_cosf(current_theta4) + CHASSIS_HALF_A_LENGTH) * AHRS_sinf(chassis_move.current_alpha2) + (CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta4)) * AHRS_cosf(chassis_move.current_alpha2);
	temp_heights[1] = -(CHASSIS_L1_LENGTH * AHRS_cosf(current_theta2) + CHASSIS_HALF_A_LENGTH) * AHRS_sinf(chassis_move.current_alpha2) + (CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta2)) * AHRS_cosf(chassis_move.current_alpha2);
	fp32 temp_heights_max;
	uint32_t temp_heights_max_index;
	arm_max_f32(temp_heights, sizeof(temp_heights) / sizeof(temp_heights[0]), &temp_heights_max, &temp_heights_max_index);
	if (temp_heights_max > 0)
	{
		chassis_move.height = temp_heights_max;
	}

	// calculation for chassis_move.wheel_rot_radius
	// right diagonal
	chassis_move.wheel_rot_radius[0] = (CHASSIS_HALF_A_LENGTH + CHASSIS_L1_LENGTH * AHRS_cosf(current_theta1)) / AHRS_cosf(chassis_move.current_alpha1) - chassis_move.height * AHRS_tanf(chassis_move.current_alpha1);
	chassis_move.wheel_rot_radius[2] = (CHASSIS_HALF_A_LENGTH + CHASSIS_L1_LENGTH * AHRS_cosf(current_theta3)) / AHRS_cosf(chassis_move.current_alpha1) + chassis_move.height * AHRS_tanf(chassis_move.current_alpha1);
	// left diagonal
	chassis_move.wheel_rot_radius[3] = (CHASSIS_HALF_A_LENGTH + CHASSIS_L1_LENGTH * AHRS_cosf(current_theta4)) / AHRS_cosf(chassis_move.current_alpha2) - chassis_move.height * AHRS_tanf(chassis_move.current_alpha2);
	chassis_move.wheel_rot_radius[1] = (CHASSIS_HALF_A_LENGTH + CHASSIS_L1_LENGTH * AHRS_cosf(current_theta2)) / AHRS_cosf(chassis_move.current_alpha2) + chassis_move.height * AHRS_tanf(chassis_move.current_alpha2);
}

void chassis_calc_targets(void)
{
	fp32 current_theta1 = motor_info[CHASSIS_ID_HIP_1].feedback_abs_angle;
	fp32 current_theta2 = motor_info[CHASSIS_ID_HIP_2].feedback_abs_angle;
	fp32 current_theta3 = motor_info[CHASSIS_ID_HIP_3].feedback_abs_angle;
	fp32 current_theta4 = motor_info[CHASSIS_ID_HIP_4].feedback_abs_angle;

	// calculate for chassis_move.target_theta corresponding to (chassis_move.target_alpha1, chassis_move.target_alpha2, chassis_move.target_height)
	// right diagonal
	chassis_inv_kine_diagonal(chassis_move.target_alpha1, chassis_move.target_height, &chassis_move.target_theta[0], &chassis_move.target_theta[2]);
	// left diagonal
	chassis_inv_kine_diagonal(chassis_move.target_alpha2, chassis_move.target_height, &chassis_move.target_theta[3], &chassis_move.target_theta[1]);

	// @TODO: verify calculation for target_wheel_rot_radius_dot using current_theta, target_theta_dot, current_alpha1, current_alpha2
	// right diagonal
	chassis_move.target_wheel_rot_radius_dot[0] = -CHASSIS_L1_LENGTH * AHRS_sinf(current_theta1) / AHRS_cosf(chassis_move.current_alpha1) * chassis_move.target_theta_dot[0];
	chassis_move.target_wheel_rot_radius_dot[2] = -CHASSIS_L1_LENGTH * AHRS_sinf(current_theta3) / AHRS_cosf(chassis_move.current_alpha1) * chassis_move.target_theta_dot[2];
	// left diagonal
	chassis_move.target_wheel_rot_radius_dot[3] = -CHASSIS_L1_LENGTH * AHRS_sinf(current_theta4) / AHRS_cosf(chassis_move.current_alpha2) * chassis_move.target_theta_dot[3];
	chassis_move.target_wheel_rot_radius_dot[1] = -CHASSIS_L1_LENGTH * AHRS_sinf(current_theta2) / AHRS_cosf(chassis_move.current_alpha2) * chassis_move.target_theta_dot[1];
}

void chassis_inv_kine_diagonal(fp32 alpha, fp32 height, fp32 *theta_right, fp32 *theta_left)
{
	fp32 A_right = height / AHRS_cosf(alpha) - CHASSIS_HALF_A_LENGTH * AHRS_tanf(alpha) - CHASSIS_L2_LENGTH;
	fp32 A_left = height / AHRS_cosf(alpha) + CHASSIS_HALF_A_LENGTH * AHRS_tanf(alpha) - CHASSIS_L2_LENGTH;
	fp32 B_right = A_right * AHRS_cosf(alpha) / CHASSIS_L1_LENGTH;
	fp32 B_left = A_left * AHRS_cosf(alpha) / CHASSIS_L1_LENGTH;

	*theta_right = fabs(alpha - AHRS_asinf(B_right));
	*theta_left = fabs(alpha + AHRS_asinf(B_left));
}

void chassis_safe_guard(void)
{
	// safety check for calculation results
	// only check calculated data after data of motor and upper board are both received
	// if (chassis_move.fHipDataIsValid)
	// {
	// 	if (chassis_move.fFatalError == 0)
	// 	{
	// 		if ((chassis_move.current_alpha1 != chassis_move.current_alpha1) || (chassis_move.current_alpha2 != chassis_move.current_alpha2) || (chassis_move.height != chassis_move.height) || (chassis_move.wheel_rot_radius[0] != chassis_move.wheel_rot_radius[0]) || (chassis_move.wheel_rot_radius[1] != chassis_move.wheel_rot_radius[1]) || (chassis_move.wheel_rot_radius[2] != chassis_move.wheel_rot_radius[2]) || (chassis_move.wheel_rot_radius[3] != chassis_move.wheel_rot_radius[3]) || (chassis_move.target_wheel_rot_radius_dot[0] != chassis_move.target_wheel_rot_radius_dot[0]) || (chassis_move.target_wheel_rot_radius_dot[1] != chassis_move.target_wheel_rot_radius_dot[1]) || (chassis_move.target_wheel_rot_radius_dot[2] != chassis_move.target_wheel_rot_radius_dot[2]) || (chassis_move.target_wheel_rot_radius_dot[3] != chassis_move.target_wheel_rot_radius_dot[3]))
	// 		{
	// 			chassis_move.fFatalError = 1;
	// 		}
	// 		else if ((chassis_move.height < CHASSIS_H_LOWER_LIMIT) || (chassis_move.height > CHASSIS_H_UPPER_LIMIT))
	// 		{
	// 			chassis_move.fFatalError = 1;
	// 		}
	// 		else if (chassis_move.alpha_upper_limit > CHASSIS_ALPHA_WORKSPACE_PEAK)
	// 		{
	// 			chassis_move.fFatalError = 1;
	// 		}

	// 		for (uint8_t bMotorRelativeId = 0; bMotorRelativeId < STEER_MOTOR_COUNT; bMotorRelativeId++)
	// 		{
	// 			if ((chassis_move.wheel_rot_radius[bMotorRelativeId] < 0) || (chassis_move.wheel_rot_radius[bMotorRelativeId] > METER_ENCODER_MAX_LIMIT))
	// 			{
	// 				chassis_move.fFatalError = 1;
	// 			}
	// 		}

	// 		for (uint8_t bMotorRelativeId = 0; bMotorRelativeId < HIP_MOTOR_COUNT; bMotorRelativeId++)
	// 		{
	// 			// if ((motor_info[bMotorRelativeId + CHASSIS_ID_HIP_1].feedback_abs_ecd_fp32 < CHASSIS_THETA_LOWER_LIMIT_ECD) || (motor_info[bMotorRelativeId + CHASSIS_ID_HIP_1].feedback_abs_ecd_fp32 > CHASSIS_THETA_UPPER_LIMIT_ECD))
	// 			//{
	// 			//	chassis_move.fFatalError = 1;
	// 			// }
	// 			chassis_move.target_theta[bMotorRelativeId] = fp32_constrain(chassis_move.target_theta[bMotorRelativeId], CHASSIS_THETA_LOWER_LIMIT_WITH_CLEARANCE, CHASSIS_THETA_UPPER_LIMIT);
	// 		}
	// 	}

	// 	// @TODO: add conditionals to check for chassis_move.fHipDataIsValid

	// 	if (chassis_move.fFatalError)
	// 	{
	// 		chassis_move.fHipDataIsValid = 0;
	// 	}
	// }
}
