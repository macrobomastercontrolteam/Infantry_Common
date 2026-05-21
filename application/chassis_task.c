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
#include "mit_pos_profile.h"
#include "pid.h"
#include "user_lib.h"
#include <assert.h>
#include "detect_task.h"

#define CHASSIS_TASK_INIT_TIME 357
#define CHASSIS_CONTROL_TIME_MS 5.0f
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)


#if HIP_MOTOR_TYPE == MG_6012
// MG6012 hip motor configs
#define HIP_MOTOR_ANGLE_PID_KP 0.001f
#define HIP_MOTOR_ANGLE_PID_KI 0.0f
#define HIP_MOTOR_ANGLE_PID_KD 0.0f
#define HIP_MOTOR_ANGLE_PID_MAX_OUT 10.0f
#define HIP_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

#define HIP_MOTOR_SPEED_PID_KP 7.5f
// #define HIP_MOTOR_SPEED_PID_KI 0.125f
#define HIP_MOTOR_SPEED_PID_KI 25.0f
#define HIP_MOTOR_SPEED_PID_KD 0.0f
#define HIP_MOTOR_SPEED_PID_MAX_OUT MG6012_MAX_TORQUE
#define HIP_MOTOR_SPEED_PID_MAX_IOUT MG6012_MAX_TORQUE
#elif HIP_MOTOR_TYPE == DM_4340P
// DM4340_P hip motor configs
#define HIP_MOTOR_ANGLE_PID_KP 2.5f
#define HIP_MOTOR_ANGLE_PID_KI 0.0f
#define HIP_MOTOR_ANGLE_PID_KD 0.0f
#define HIP_MOTOR_ANGLE_PID_MAX_OUT 10.5f
#define HIP_MOTOR_ANGLE_PID_MAX_IOUT 0.0f

#define HIP_MOTOR_SPEED_PID_KP 0.05f
// #define DM_4340_P_MOTOR_SPEED_PID_KI 0.125f
#define HIP_MOTOR_SPEED_PID_KI 0.0f
#define HIP_MOTOR_SPEED_PID_KD 0.0f
#define HIP_MOTOR_SPEED_PID_MAX_OUT 6.5f
#define HIP_MOTOR_SPEED_PID_MAX_IOUT 0.0f

static const fp32 hip_ff_sign[4] = { -1.0f, 1.0f, -1.0f, 1.0f };

// MIT position profile parameters for DM4340P hip motors
#define HIP_MIT_PROFILE_KP            35.0f    ///< position gain (matches prior hardcoded value)
#define HIP_MIT_PROFILE_KP_MIN        10.0f
#define HIP_MIT_PROFILE_KP_MAX        75.0f
#define HIP_MIT_PROFILE_KD            1.0f   ///< damping gain  (matches prior hardcoded value)
#define HIP_MIT_PROFILE_MAX_VEL       0.5f    ///< slew-rate limit (rad/s) — tune as needed
#define HIP_MIT_PROFILE_TORQ_FF_BIAS  5.0f    ///< constant feedforward torque (Nm)
#define HIP_MIT_PROFILE_TORQ_FF_AMP   0.0f    ///< sine FF amplitude (Nm); 0 = disabled
#define HIP_MIT_PROFILE_ANGLE_OFFSET  0.0f    ///< sine FF phase offset (rad)
#define HIP_MIT_PROFILE_TORQ_FF_MIN   (-10.0f)
#define HIP_MIT_PROFILE_TORQ_FF_MAX   (10.0f)
#else
#endif
#if HIP_MOTOR_TYPE == MG_6012
// Zero position of 6012 motors are calibrated to the CHASSIS_THETA_LOWER_LIMIT_ECD
#define MG6012_MOTOR_0_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#define MG6012_MOTOR_1_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#define MG6012_MOTOR_2_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#define MG6012_MOTOR_3_ANGLE_ECD_OFFSET (MG6012_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#elif HIP_MOTOR_TYPE == DM_4340P
#define DM4340_MOTOR_0_ANGLE_ECD_OFFSET (HALF_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#define DM4340_MOTOR_1_ANGLE_ECD_OFFSET (HALF_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#define DM4340_MOTOR_2_ANGLE_ECD_OFFSET (HALF_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#define DM4340_MOTOR_3_ANGLE_ECD_OFFSET (HALF_ECD_RANGE - CHASSIS_THETA_LOWER_LIMIT_ECD)
#else
#endif
#define CHASSIS_TEST_MODE 0

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

chassis_move_t chassis_move;

#if HIP_MOTOR_TYPE == DM_4340P
static mit_pos_profile_t      hip_profile_state[HIP_MOTOR_COUNT];
static mit_pos_profile_param_t hip_profile_param;
#endif

void param_asserts(void);
void chassis_init(void);
void chassis_calc_feedbacks(void);
void chassis_calc_targets(void);
void chassis_inv_kine(fp32 alpha, fp32 height);
void chassis_safe_guard(void);

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
fp32 roll_fp32;
fp32 pitch_fp32;
fp32 yaw_fp32;
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

	// target_theta1_dot_fp32 = chassis_move.target_theta_dot[0] * 180.0f / PI;
	// target_theta2_dot_fp32 = chassis_move.target_theta_dot[1] * 180.0f / PI;
	// target_theta3_dot_fp32 = chassis_move.target_theta_dot[2] * 180.0f / PI;
	// target_theta4_dot_fp32 = chassis_move.target_theta_dot[3] * 180.0f / PI;

	// current_theta1_fp32 = motor_info[CHASSIS_ID_HIP_1].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;
	// current_theta2_fp32 = motor_info[CHASSIS_ID_HIP_2].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;
	// current_theta3_fp32 = motor_info[CHASSIS_ID_HIP_3].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;
	// current_theta4_fp32 = motor_info[CHASSIS_ID_HIP_4].feedback_abs_ecd_fp32 / MG6012_ECD_RANGE_180 * 180.0f;

	// set_torque3_fp32 = motor_info[CHASSIS_ID_HIP_3].set_torque * 50.0f;

	roll_fp32 = *(INS_angle + INS_ROLL_ADDRESS_OFFSET) * 180.0f / PI;
	pitch_fp32 = *(INS_angle + INS_PITCH_ADDRESS_OFFSET) * 180.0f / PI;
	yaw_fp32 = *(INS_angle + INS_YAW_ADDRESS_OFFSET) * 180.0f / PI;
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
#if HIP_MOTOR_TYPE == DM_4340P

	enable_all_DaMiao_motors(1);

#endif
	param_asserts();
	chassis_init();
	osDelay(CHASSIS_TASK_INIT_TIME);
	//chassis_move.fHipDataIsValid = 1;
	while (1)
	{
#if HIP_MOTOR_TYPE == DM_4340P
		hip_profile_param.kp = fp32_constrain(chassis_move.hip_motor_kp, HIP_MIT_PROFILE_KP_MIN, HIP_MIT_PROFILE_KP_MAX);
		if(toe_is_error(CHASSIS_HIP1_TOE) || 
           toe_is_error(CHASSIS_HIP2_TOE) ||  
           toe_is_error(CHASSIS_HIP3_TOE) || 
           toe_is_error(CHASSIS_HIP4_TOE))
		{
			chassis_move.hip_motor_kp = HIP_MIT_PROFILE_KP;
			enable_all_DaMiao_motors(1);
		}
		

#endif
		chassis_calc_feedbacks();
		chassis_calc_targets();
		chassis_safe_guard();


		// @TODO: reenable this after fixing fatal error
		// if (chassis_move.fFatalError)
		// {
		// 	chassis_move.fHipMotorEnabled = 0;
		// }

		// PID calculation
		for (bMotorRelativeId = 0; bMotorRelativeId < HIP_MOTOR_COUNT; bMotorRelativeId++)
		{
			bMotorId = bMotorRelativeId + CHASSIS_ID_HIP_1;
#if HIP_MOTOR_TYPE == DM_4340P
				mit_pos_profile_update(
					&hip_profile_state[bMotorRelativeId],
					&hip_profile_param,
					motor_info[bMotorId].feedback_abs_angle,
					chassis_move.target_theta[bMotorRelativeId],
					0,
					&chassis_move.hip_cmd[bMotorRelativeId]);

					// apply motor-specific FF sign outside
					chassis_move.hip_cmd[bMotorRelativeId].torq *= hip_ff_sign[bMotorRelativeId];
#else
				chassis_move.target_theta_dot[bMotorRelativeId] = PID_calc(&chassis_move.hip_angle_pid[bMotorRelativeId], motor_info[bMotorId].feedback_abs_ecd_fp32, chassis_move.target_theta[bMotorRelativeId] * MG6012_MOTOR_RAD_TO_ECD, CHASSIS_CONTROL_TIME_S);
					motor_info[bMotorId].set_torque = PID_calc(&chassis_move.hip_speed_pid[bMotorRelativeId], DEG_TO_RAD(motor_info[bMotorId].rotor_speed), chassis_move.target_theta_dot[bMotorRelativeId], CHASSIS_CONTROL_TIME_S);
#endif /* HIP_MOTOR_TYPE == DM_4340P */
		}
		CAN_cmd_wrapper();
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
	assert(CHASSIS_H_LOWER_LIMIT >= CHASSIS_WHEEL_REDIUS);
	assert(CHASSIS_H_UPPER_LIMIT <= CHASSIS_L1_LENGTH + CHASSIS_WHEEL_REDIUS);
	assert((CHASSIS_H_WORKSPACE_PEAK < CHASSIS_H_UPPER_LIMIT) && (CHASSIS_H_WORKSPACE_PEAK > CHASSIS_H_LOWER_LIMIT));
	assert((CHASSIS_H_WORKSPACE_SLOPE1 > 0) && (CHASSIS_H_WORKSPACE_SLOPE1 < 1));
	assert((CHASSIS_H_WORKSPACE_SLOPE2 < 0) && (CHASSIS_H_WORKSPACE_SLOPE2 > -1));
}

void chassis_init(void)
{
	uint8_t bMotorId;
	uint8_t bMotorRelativeId;
#if HIP_MOTOR_TYPE == MG_6012
	const uint16_t hip_motor_offset_ecd[4] = {MG6012_MOTOR_0_ANGLE_ECD_OFFSET, MG6012_MOTOR_1_ANGLE_ECD_OFFSET, MG6012_MOTOR_2_ANGLE_ECD_OFFSET, MG6012_MOTOR_3_ANGLE_ECD_OFFSET};
#elif HIP_MOTOR_TYPE == DM_4340P
	const uint16_t hip_motor_offset_ecd[4] = {DM4340_MOTOR_0_ANGLE_ECD_OFFSET, DM4340_MOTOR_1_ANGLE_ECD_OFFSET, DM4340_MOTOR_2_ANGLE_ECD_OFFSET, DM4340_MOTOR_3_ANGLE_ECD_OFFSET};
#else
#endif
	const fp32 hip_angle_pid_params[3] = {HIP_MOTOR_ANGLE_PID_KP, HIP_MOTOR_ANGLE_PID_KI, HIP_MOTOR_ANGLE_PID_KD};
	const fp32 hip_speed_pid_params[3] = {HIP_MOTOR_SPEED_PID_KP, HIP_MOTOR_SPEED_PID_KI, HIP_MOTOR_SPEED_PID_KD};
	for (bMotorRelativeId = 0; bMotorRelativeId < HIP_MOTOR_COUNT; bMotorRelativeId++)
	{
		bMotorId = bMotorRelativeId + CHASSIS_ID_HIP_1;
		PID_init(&chassis_move.hip_angle_pid[bMotorRelativeId], PID_POSITION, hip_angle_pid_params, HIP_MOTOR_ANGLE_PID_MAX_OUT, HIP_MOTOR_ANGLE_PID_MAX_IOUT, 0, &MG6012_ecd_err_handler);
		PID_init(&chassis_move.hip_speed_pid[bMotorRelativeId], PID_POSITION, hip_speed_pid_params, HIP_MOTOR_SPEED_PID_MAX_OUT, HIP_MOTOR_SPEED_PID_MAX_IOUT, 0.8f, &filter_err_handler);
		motor_info[bMotorId].offset_ecd = hip_motor_offset_ecd[bMotorRelativeId];
	}

	chassis_move.target_alpha1 = 0;
	chassis_move.target_alpha2 = 0;
#if HEADLESS_HIP_TEST
	// chassis_move.target_height_front = CHASSIS_H_WORKSPACE_PEAK;
	chassis_move.target_height_front = CHASSIS_H_UPPER_LIMIT;
	chassis_move.target_height_back  = CHASSIS_H_UPPER_LIMIT;
#else
	chassis_move.target_height_front = CHASSIS_H_LOWER_LIMIT;
	chassis_move.target_height_back  = CHASSIS_H_LOWER_LIMIT;
#endif
	chassis_move.current_alpha1 = 0;
	chassis_move.current_alpha2 = 0;
	chassis_move.height = CHASSIS_H_LOWER_LIMIT;
	chassis_move.alpha_lower_limit = 0;
	chassis_move.alpha_upper_limit = 0;

	for (bMotorId = 0; bMotorId < HIP_MOTOR_COUNT; bMotorId++)
	{
		chassis_move.target_theta[bMotorId] = CHASSIS_THETA_LOWER_LIMIT;
		chassis_move.target_theta_dot[bMotorId] = 0;
	}

	chassis_move.fHipMotorEnabled = 0;
	chassis_move.fFatalError = 0;
	chassis_move.fHipDataIsValid = 0;

#if HIP_MOTOR_TYPE == DM_4340P
	// Initialize MIT position profile parameters
	hip_profile_param.kp             = HIP_MIT_PROFILE_KP;
	hip_profile_param.kd             = HIP_MIT_PROFILE_KD;
	hip_profile_param.max_vel        = HIP_MIT_PROFILE_MAX_VEL;
	hip_profile_param.dt             = CHASSIS_CONTROL_TIME_S;
	hip_profile_param.torque_ff_bias = HIP_MIT_PROFILE_TORQ_FF_BIAS;
	hip_profile_param.use_sine_ff    = 0;
	hip_profile_param.torque_ff_amp  = HIP_MIT_PROFILE_TORQ_FF_AMP;
	hip_profile_param.angle_offset   = HIP_MIT_PROFILE_ANGLE_OFFSET;
	hip_profile_param.torque_ff_min  = HIP_MIT_PROFILE_TORQ_FF_MIN;
	hip_profile_param.torque_ff_max  = HIP_MIT_PROFILE_TORQ_FF_MAX;

	chassis_move.hip_motor_kp = HIP_MIT_PROFILE_KP;

	// Clear state — mit_pos_profile_update seeds from measured_pos on first call
	uint8_t bProfileId;
	for (bProfileId = 0; bProfileId < HIP_MOTOR_COUNT; bProfileId++)
	{
		hip_profile_state[bProfileId].initialized  = 0;
		hip_profile_state[bProfileId].pos_cmd      = 0.0f;
		hip_profile_state[bProfileId].last_pos_cmd = 0.0f;
	}
#endif
}

void chassis_calc_feedbacks(void)
{
	fp32 temp_heights[4];
	fp32 current_theta1 = motor_info[CHASSIS_ID_HIP_1].feedback_abs_angle;
	fp32 current_theta2 = motor_info[CHASSIS_ID_HIP_2].feedback_abs_angle;
	fp32 current_theta3 = motor_info[CHASSIS_ID_HIP_3].feedback_abs_angle;
	fp32 current_theta4 = motor_info[CHASSIS_ID_HIP_4].feedback_abs_angle;

	// @TODO: calculate for chassis_move.current_alpha1 and chassis_move.current_alpha2 using IMU data
	fp32 raw_roll = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);
	fp32 raw_pitch = *(INS_angle + INS_PITCH_ADDRESS_OFFSET);
	chassis_move.current_alpha1 = -rad_format(raw_roll);
	chassis_move.current_alpha2 = -raw_pitch;

	// calculation for chassis_move.height: some legs may not be on the ground, so pick the largest height calculated
	// right diagonal
	
	//to do: change the L2 to the wheel radius as the wheel won't change the height offset when hip motor is moving
	//to do: Add 2/sqrt(2) to the chassis L1 parameter
	temp_heights[0] = (CHASSIS_L1_LENGTH * -AHRS_sinf(current_theta1)) + CHASSIS_WHEEL_REDIUS; //(CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta1)) * AHRS_cosf(chassis_move.current_alpha1);
	temp_heights[2] = (CHASSIS_L1_LENGTH * -AHRS_sinf(current_theta3)) + CHASSIS_WHEEL_REDIUS; //(CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta3)) * AHRS_cosf(chassis_move.current_alpha1);
	// left diagonal
	temp_heights[3] = (CHASSIS_L1_LENGTH * AHRS_sinf(current_theta4)) + CHASSIS_WHEEL_REDIUS; //(CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta4)) * AHRS_cosf(chassis_move.current_alpha2);
	temp_heights[1] = (CHASSIS_L1_LENGTH * AHRS_sinf(current_theta2)) + CHASSIS_WHEEL_REDIUS; //(CHASSIS_L2_LENGTH + CHASSIS_L1_LENGTH * AHRS_sinf(current_theta2)) * AHRS_cosf(chassis_move.current_alpha2);
	fp32 temp_heights_max;
	uint32_t temp_heights_max_index;
	arm_max_f32(temp_heights, sizeof(temp_heights) / sizeof(temp_heights[0]), &temp_heights_max, &temp_heights_max_index);
	if (temp_heights_max > 0)
	{
		chassis_move.height = temp_heights_max;
	}

}

void chassis_calc_targets(void)
{
	// Calculate target_theta for all hips in one call.
	chassis_inv_kine(chassis_move.target_alpha1, chassis_move.target_height);

}

void chassis_inv_kine(fp32 alpha, fp32 height)
{
	fp32 sin_alpha_term = ((CHASSIS_A_LENGTH + CHASSIS_L1_LENGTH) / CHASSIS_L1_LENGTH) * AHRS_sinf(alpha);

	fp32 sin_height_term = (height - CHASSIS_WHEEL_REDIUS) / CHASSIS_L1_LENGTH;


	fp32 alpha_term = AHRS_asinf(sin_alpha_term);
	fp32 height_term = AHRS_asinf(sin_height_term);
	fp32 base_theta = height_term + alpha;

	for (uint8_t hip_id = 0; hip_id < HIP_MOTOR_COUNT; hip_id++)
	{
		uint8_t is_rear = (hip_id >= 2);
		fp32 theta = base_theta + (is_rear ? -alpha_term : alpha_term);
		if ((hip_id == 0) || (hip_id == 2))
		{
			theta = -theta;
		}
		chassis_move.target_theta[hip_id] = theta;
	}
}

void chassis_safe_guard(void)
{
	// safety check for calculation results
	// only check calculated data after data of motor and upper board are both received
	// if (chassis_move.fHipDataIsValid)
	// {
	// 	if (chassis_move.fFatalError == 0)
	// 	{
	// 		if ((chassis_move.current_alpha1 != chassis_move.current_alpha1) || (chassis_move.current_alpha2 != chassis_move.current_alpha2) || (chassis_move.height != chassis_move.height))
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
