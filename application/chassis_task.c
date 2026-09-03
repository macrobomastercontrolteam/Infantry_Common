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
#include "chassis_behaviour.h"
#include "gimbal_task.h"

#include "cmsis_os.h"

#include "AHRS_middleware.h"
#include "CAN_receive.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "cv_usart_task.h"
#include "detect_task.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"
#include <assert.h>
#include "referee.h"
#include "vofa_task.h"
#define STEER_MOTOR_UPSIDE_DOWN_MOUNTING 0
#define SWERVE_INVALID_HIP_DATA_RESET_TIMEOUT 1000

fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // unit m/s

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @retval         none
 */
static void chassis_init(void);

/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle, and robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_feedback_update(void);
void chassis_speed_max_adj(void);
/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 *
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_set_control(void);
/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @retval         none
 */
static void chassis_control_loop(void);

#if (ROBOT_TYPE != INFANTRY_2024_BIPED) && (ROBOT_TYPE != INFANTRY_2023_SWERVE)
static void chassis_slip_robust_vel_update(void);
#endif

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
static uint16_t motor_angle_to_ecd_change(fp32 angle);
void swerve_convert_from_rpy_to_alpha(fp32 roll, fp32 pitch, fp32 *alpha1, fp32 *alpha2, fp32 gimbal_chassis_relative_yaw_angle);
void swerve_convert_from_alpha_to_rpy(fp32 *roll, fp32 *pitch, fp32 alpha1, fp32 alpha2, fp32 gimbal_chassis_relative_yaw_angle);
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif
supcap_t cap_message_rx;

chassis_move_t chassis_move;

// encoder-only, slip-mitigated chassis-frame translational velocity (m/s), updated every chassis cycle.
// Always defined so the helper compiles on every robot type; it is only called for 4-wheel chassis builds.
static fp32 chassis_vx_slip_robust = 0.0f;
static fp32 chassis_vy_slip_robust = 0.0f;

#if CHASSIS_TEST_MODE
fp32 rot_radius0;
fp32 rot_radius1;
fp32 rot_radius2;
fp32 rot_radius3;
fp32 rot_radius_dot0;
fp32 rot_radius_dot1;
fp32 rot_radius_dot2;
fp32 rot_radius_dot3;
static void J_scope_chassis_test(void)
{
	rot_radius0 = chassis_move.wheel_rot_radii[0] * 1000.0f;
	rot_radius1 = chassis_move.wheel_rot_radii[1] * 1000.0f;
	rot_radius2 = chassis_move.wheel_rot_radii[2] * 1000.0f;
	rot_radius3 = chassis_move.wheel_rot_radii[3] * 1000.0f;

	rot_radius_dot0 = chassis_move.target_wheel_rot_radii_dot[0] * 1000.0f;
	rot_radius_dot1 = chassis_move.target_wheel_rot_radii_dot[1] * 1000.0f;
	rot_radius_dot2 = chassis_move.target_wheel_rot_radii_dot[2] * 1000.0f;
	rot_radius_dot3 = chassis_move.target_wheel_rot_radii_dot[3] * 1000.0f;
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
	// wait a time
	osDelay(CHASSIS_TASK_INIT_TIME);

	// while (ifToeStatusExist(REMOTE_TOE, CHASSIS_MOTOR4_TOE, TOE_STATUS_OFFLINE, NULL))
	// {
	//     osDelay(CHASSIS_CONTROL_TIME_MS * 2);
	// }

	chassis_init();

	while (1)
	{
		chassis_behaviour_set_mode();
		// operation when behaviour mode changes
		chassis_behaviour_change_transit();
		// chassis data update
		chassis_feedback_update();
		// set chassis control set-point
		chassis_set_control();
		// chassis control pid calculate
		chassis_control_loop();
		// send CAN msg
		CAN_cmd_chassis();
#if (SUPERCAP_TYPE == UBC_SUPERCAP)
		CAN_cmd_supercap();

#elif (SUPERCAP_TYPE == MACRM_SUPERCAP)
		CAN_cmd_supercap();
#endif

		osDelayUntil(&ulSystemTime, CHASSIS_CONTROL_TIME_MS);

#if CHASSIS_TEST_MODE
		J_scope_chassis_test();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @retval         none
 */
static void chassis_init(void)
{
	// chassis angle PID
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

	const static fp32 chassis_wz_order_filter[1] = {CHASSIS_ACCEL_WZ_NUM};

	chassis_move.chassis_coord_sys = CHASSIS_COORDINATE_FOLLOW_CHASSIS_RELATIVE_FRONT;
	chassis_move.chassis_RC = get_remote_control_point();
	chassis_move.chassis_INS_angle = get_INS_angle_point();
	chassis_move.chassis_yaw_motor = get_yaw_motor_point();
	chassis_move.chassis_pitch_motor = get_pitch_motor_point();

#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
	for (uint8_t i = 0; i < sizeof(chassis_move.wheel_rot_radii) / sizeof(chassis_move.wheel_rot_radii[0]); i++)
	{
		chassis_move.wheel_rot_radii[i] = MOTOR_DISTANCE_TO_CENTER_DEFAULT;
	}

#elif (ROBOT_TYPE == SENTRY_2026_OMNI) || (ROBOT_TYPE == INFANTRY_2024_MECANUM_NEO) || (ROBOT_TYPE == INFANTRY_2026_OMNI)
	const static fp32 motor_speed_pid[3] = {MG4010_MOTOR_SPEED_PID_KP, MG4010_MOTOR_SPEED_PID_KI, MG4010_MOTOR_SPEED_PID_KD};
	for (uint8_t i = 0; i < sizeof(chassis_move.wheel_rot_radii) / sizeof(chassis_move.wheel_rot_radii[0]); i++)
	{
		chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move.motor_speed_pid[i], PID_POSITION, motor_speed_pid, MG4010_MOTOR_SPEED_PID_MAX_OUT, MG4010_MOTOR_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
		chassis_move.wheel_rot_radii[i] = MOTOR_DISTANCE_TO_CENTER_DEFAULT;
	}
#else
	const static fp32 motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};
	for (uint8_t i = 0; i < sizeof(chassis_move.wheel_rot_radii) / sizeof(chassis_move.wheel_rot_radii[0]); i++)
	{
		chassis_move.motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move.motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT, 0.35f, &filter_err_handler);
		chassis_move.wheel_rot_radii[i] = MOTOR_DISTANCE_TO_CENTER_DEFAULT;
	}
#endif
	PID_init(&chassis_move.chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, 0, &rad_err_handler);

	ramp_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME_S, NORMAL_MAX_CHASSIS_SPEED_X, -NORMAL_MAX_CHASSIS_SPEED_X);
	ramp_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME_S, NORMAL_MAX_CHASSIS_SPEED_Y, -NORMAL_MAX_CHASSIS_SPEED_Y);
	first_order_filter_init(&chassis_move.chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME_S, chassis_wz_order_filter);
	//low_pass_filter_ema(chassis_move.chassis_cmd_slow_set_vx.out, 0.05f, 1);
	//low_pass_filter_ema(chassis_move.chassis_cmd_slow_set_vy.out, 0.05f, 1);
	chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move.wz_max_speed = SPINNING_CHASSIS_MED_OMEGA;

	chassis_move.dial_channel_latched = 0;

#if (ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI)
	chassis_move.fRandomSpinOn = 1;
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	chassis_move.fUpperHeadEnabled = 0;
#endif
#else
	chassis_move.fRandomSpinOn = 0;
#endif

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
	// special assignment to ensure fHipDisabledEdge not being miscalculated by the garbage data
	chassis_move.fHipEnabled = 0;
	swerve_chassis_params_reset();
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)
	chassis_move.chassis_platform.chassis_hip_kp = HIP_MIT_PROFILE_KP;
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
	biped_chassis_params_reset();
#endif

	// update data
	chassis_feedback_update();
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
void swerve_chassis_back_home(void)
{
	// back to middle height, which has the largest workspace for alpha angle
	chassis_move.chassis_platform.target_roll = 0;
	chassis_move.chassis_platform.target_pitch = 0;
	chassis_move.chassis_platform.target_alpha1 = 0;
	chassis_move.chassis_platform.target_alpha2 = 0;
	chassis_move.chassis_platform.target_height = CHASSIS_H_WORKSPACE_PEAK;
}

void swerve_chassis_params_reset(void)
{
	chassis_move.chassis_platform.target_roll = 0;
	chassis_move.chassis_platform.target_pitch = 0;
	chassis_move.chassis_platform.target_alpha1 = 0;
	chassis_move.chassis_platform.target_alpha2 = 0;
	chassis_move.chassis_platform.target_height = CHASSIS_H_LOWER_LIMIT;
	chassis_move.chassis_platform.feedback_alpha1 = 0;
	chassis_move.chassis_platform.feedback_alpha2 = 0;
	chassis_move.chassis_platform.feedback_height = CHASSIS_H_LOWER_LIMIT;

	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_move.wheel_rot_radii[i] = MOTOR_DISTANCE_TO_CENTER_DEFAULT;
		chassis_move.target_wheel_rot_radii_dot[i] = 0;
	}
	chassis_enable_platform_flag(0);
}
#endif

#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
void biped_chassis_back_home(void)
{
	// back to chair posture
	chassis_move.chassis_platform.target_roll_dot = 0;
	chassis_move.chassis_platform.target_yaw = chassis_move.chassis_platform.feedback_yaw;
	chassis_move.chassis_platform.target_simplified_L0_dot = 0;
	chassis_move.chassis_platform.target_dis_dot = 0;
	chassis_move.chassis_platform.fBackToHome = 1;
}

void biped_chassis_params_reset(void)
{
	chassis_move.chassis_platform.feedback_roll = 0;
	chassis_move.chassis_platform.feedback_pitch = 0;
	// @Warning: be careful with assigning feedback_yaw manually
	chassis_move.chassis_platform.feedback_yaw = 0;
	chassis_move.chassis_platform.feedback_simplified_L0 = BIPED_LEG_L0_MIN;

	chassis_move.chassis_platform.target_roll_dot = 0;
	chassis_move.chassis_platform.target_yaw = chassis_move.chassis_platform.feedback_yaw;
	chassis_move.chassis_platform.target_simplified_L0_dot = 0;
	chassis_move.chassis_platform.target_dis_dot = 0;
	chassis_move.chassis_platform.fBackToHome = 0;
	chassis_move.chassis_platform.fJumpStart = 0;

	chassis_enable_platform_flag(0);
}
#endif

/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle, and robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_feedback_update(void)
{
#if (ROBOT_TYPE != INFANTRY_2024_BIPED)
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		// update motor speed, accel is differential of speed PID
		chassis_move.motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move.motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move.motor_chassis[i].accel = chassis_move.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

#if (ROBOT_TYPE != INFANTRY_2023_SWERVE)
	// update chassis parameters: vertical speed x, horizontal speed y, rotation speed wz, right hand rule
#if (WHEEL_TYPE ==ROBOT_CHASSIS_USE_MECANUM)
	// Mecanum wheel feedback calculation
	chassis_move.vx = (-chassis_move.motor_chassis[0].speed + chassis_move.motor_chassis[1].speed + chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_move.vy = (-chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed + chassis_move.motor_chassis[2].speed + chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	if (chassis_move.wheel_rot_radii[0] > 0.001f)
	{
		chassis_move.wz = (-chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed - chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / chassis_move.wheel_rot_radii[0];
	}
	else
	{
		chassis_move.wz = 0.0f;
	}
#elif (WHEEL_TYPE == ROBOT_CHASSIS_USE_OMNI)

	chassis_move.vx = (-chassis_move.motor_chassis[0].speed + chassis_move.motor_chassis[1].speed + chassis_move.motor_chassis[2].speed - chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_move.vy = (-chassis_move.motor_chassis[0].speed - chassis_move.motor_chassis[1].speed + chassis_move.motor_chassis[2].speed + chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	if (chassis_move.wheel_rot_radii[0] > 0.001f)
	{
		chassis_move.wz = (chassis_move.motor_chassis[0].speed + chassis_move.motor_chassis[1].speed + chassis_move.motor_chassis[2].speed + chassis_move.motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / chassis_move.wheel_rot_radii[0];
	}
	else
	{
		chassis_move.wz = 0.0f;
	}
#endif
	// encoder-only slip-robust chassis velocity estimate (consumed by the CV chassis-velocity report)
	chassis_slip_robust_vel_update();
#endif
#endif

	// calculate chassis euler angle, if chassis have a new gyro sensor,please change this code
	chassis_move.chassis_yaw = rad_format(*(chassis_move.chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move.chassis_yaw_motor->relative_angle);
	chassis_move.chassis_pitch = rad_format(*(chassis_move.chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move.chassis_pitch_motor->relative_angle);
	chassis_move.chassis_roll = *(chassis_move.chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

	// KEY_PRESSED_OFFSET_E toggles random spinning mode
	// no need to debounce because keyboard signal is clean
	static uint8_t fLastKeyESignal = 0;
	uint8_t fIsKeyEPressed = ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_E) != 0);
	if (fLastKeyESignal != fIsKeyEPressed)
	{
		if (fIsKeyEPressed)
		{
			chassis_move.fRandomSpinOn = !chassis_move.fRandomSpinOn;
		}
		fLastKeyESignal = fIsKeyEPressed;
	}
}

void chassis_speed_max_adj(void)
{
	fp32 ref_chassis_power_buffer = 0;
	fp32 ref_chassis_power_limit = 0;

#if CAN_PASS_REF_INFO
	CAN_get_chassis_power_info(&ref_chassis_power_buffer, &ref_chassis_power_limit);
#else
	get_chassis_power_data(&ref_chassis_power_buffer, &ref_chassis_power_limit);
#endif
	
	fp32 vx_speed_limit = 0;
	fp32 vy_speed_limit = 0;
	fp32 wz_speed_limit = 0;
	fp32 wz_scaling_factor = 0;
	fp32 buffer_ratio;
	
	if(ref_chassis_power_buffer < 10.0f)
	{
		vx_speed_limit = LOW_POWER_MAX_CHASSIS_SPEED_X;
		vy_speed_limit = LOW_POWER_MAX_CHASSIS_SPEED_Y;
		wz_scaling_factor = 0.05f;
	}
	else
    {
        buffer_ratio = fp32_constrain((ref_chassis_power_buffer - 10.0f) / 30.0f, 0.0f, 1.0f);

        vx_speed_limit = NORMAL_MAX_CHASSIS_SPEED_X * (0.5f + 0.5f * buffer_ratio);
        vy_speed_limit = NORMAL_MAX_CHASSIS_SPEED_Y * (0.5f + 0.5f * buffer_ratio);
        wz_scaling_factor = 0.2f + 0.8f * buffer_ratio;
    }

	if ((chassis_behaviour_mode == CHASSIS_SPINNING_MODE) || (chassis_behaviour_mode == CHASSIS_CV_CONTROL_MODE))
	{
		vx_speed_limit = vy_speed_limit;
		wz_speed_limit = calc_wz_max_speed(vx_speed_limit, vy_speed_limit, wz_scaling_factor); //further limit the spinning speed in spinning mode to avoid high power consumption
	}
	else
	{
		wz_speed_limit = calc_wz_max_speed(vx_speed_limit, vy_speed_limit, 1.0f);
	}

	chassis_move.vx_max_speed = fp32_abs_constrain(vx_speed_limit, SPRINT_MAX_CHASSIS_SPEED_X);
	chassis_move.vy_max_speed = fp32_abs_constrain(vy_speed_limit, SPRINT_MAX_CHASSIS_SPEED_Y);
	chassis_move.wz_max_speed = fp32_abs_constrain(wz_speed_limit, SPINNING_CHASSIS_MAX_OMEGA);
}

/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set)
{
	if (vx_set == NULL || vy_set == NULL)
	{
		return;
	}

	chassis_speed_max_adj();//

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	// deadline, because some remote control need be calibrated,  the value of joystick is not zero in middle place,
	deadband_limit(chassis_move.chassis_RC->rc.ch[JOYSTICK_LEFT_VERTICAL_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	deadband_limit(chassis_move.chassis_RC->rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

	// (remote controller sensitivity) map joystick value to vertical (vx) and horizontal (vy) speed
	fp32 vx_rc_sen = chassis_move.vx_max_speed / JOYSTICK_HALF_RANGE;
	fp32 vy_rc_sen = chassis_move.vy_max_speed / JOYSTICK_HALF_RANGE;
	vx_set_channel = vx_channel * vx_rc_sen;
	vy_set_channel = vy_channel * -vy_rc_sen;

	// keyboard set speed set-point
	if (chassis_move.chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		vx_set_channel = chassis_move.vx_max_speed;
	}
	else if (chassis_move.chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		vx_set_channel = -chassis_move.vx_max_speed;
	}

	if (chassis_move.chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		vy_set_channel = chassis_move.vy_max_speed;
	}
	else if (chassis_move.chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		vy_set_channel = -chassis_move.vy_max_speed;
	}

	// RC input shaping with ramp: keep this at input layer so other control logic remains intact
	chassis_move.chassis_cmd_slow_set_vx.max_value = chassis_move.vx_max_speed;
	chassis_move.chassis_cmd_slow_set_vx.min_value = -chassis_move.vx_max_speed;
	chassis_move.chassis_cmd_slow_set_vy.max_value = chassis_move.vy_max_speed;
	chassis_move.chassis_cmd_slow_set_vy.min_value = -chassis_move.vy_max_speed;

	// decelerate faster only when command magnitude is reduced (stop/down-ramp)
	fp32 vx_limit = (fabs(vx_set_channel) < fabs(chassis_move.chassis_cmd_slow_set_vx.out)) ? CHASSIS_DECEL_X_NUM : CHASSIS_ACCEL_X_NUM;
	fp32 vx_ramp_input = fp32_constrain(
		(vx_set_channel - chassis_move.chassis_cmd_slow_set_vx.out) / CHASSIS_CONTROL_TIME_S,
		-vx_limit, vx_limit);
	ramp_calc(&chassis_move.chassis_cmd_slow_set_vx, vx_ramp_input);

	fp32 vy_limit = (fabs(vy_set_channel) < fabs(chassis_move.chassis_cmd_slow_set_vy.out)) ? CHASSIS_DECEL_Y_NUM : CHASSIS_ACCEL_Y_NUM;
	fp32 vy_ramp_input = fp32_constrain(
		(vy_set_channel - chassis_move.chassis_cmd_slow_set_vy.out) / CHASSIS_CONTROL_TIME_S,
		-vy_limit, vy_limit);
	ramp_calc(&chassis_move.chassis_cmd_slow_set_vy, vy_ramp_input);

	*vx_set = chassis_move.chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move.chassis_cmd_slow_set_vy.out;
}

#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
void biped_platform_rc_mapping(void)
{
	const fp32 BIPED_PLATFORM_ROLL_KEYBOARD_CHANGE_TIME_S = 0.5f;
	const fp32 BIPED_PLATFORM_ROLL_KEYBOARD_DOT = BIPED_PLATFORM_MAX_ROLL / BIPED_PLATFORM_ROLL_KEYBOARD_CHANGE_TIME_S;
	const fp32 BIPED_PLATFORM_LENGTH_KEYBOARD_CHANGE_TIME_S = 1.25f;
	const fp32 BIPED_PLATFORM_LENGTH_KEYBOARD_DOT = (BIPED_LEG_L0_MAX - BIPED_LEG_L0_MIN) / BIPED_PLATFORM_LENGTH_KEYBOARD_CHANGE_TIME_S;

	if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
	{
		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_C)
		{
			biped_chassis_back_home();
		}
		else
		{
			// height
			if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
			{
				chassis_move.chassis_platform.target_simplified_L0_dot = BIPED_PLATFORM_LENGTH_KEYBOARD_DOT;
			}
			else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
			{
				chassis_move.chassis_platform.target_simplified_L0_dot = -BIPED_PLATFORM_LENGTH_KEYBOARD_DOT;
			}
		}
	}
	else // ctrl not pressed
	{
		// roll
		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_B)
		{
			chassis_move.chassis_platform.target_roll_dot = BIPED_PLATFORM_ROLL_KEYBOARD_DOT;
		}
		else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_C)
		{
			chassis_move.chassis_platform.target_roll_dot = -BIPED_PLATFORM_ROLL_KEYBOARD_DOT;
		}
		else
		{
			chassis_move.chassis_platform.fJumpStart = ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_G) != 0);
		}
	}
}
#endif

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
void swerve_platform_rc_mapping(void)
{
	// configuration for pseudo RPY-to-tilt conversion
	const fp32 SWERVE_PLATFORM_ROLL_KEYBOARD_CHANGE_TIME_S = 0.4f;
	const fp32 SWERVE_PLATFORM_ROLL_KEYBOARD_SEN_INC = 2.0f * CHASSIS_ALPHA_WORKSPACE_PEAK / SWERVE_PLATFORM_ROLL_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S;
	const fp32 SWERVE_PLATFORM_PITCH_KEYBOARD_CHANGE_TIME_S = 0.4f;
	const fp32 SWERVE_PLATFORM_PITCH_KEYBOARD_SEN_INC = 2.0f * CHASSIS_ALPHA_WORKSPACE_PEAK / SWERVE_PLATFORM_PITCH_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S;
	const fp32 SWERVE_PLATFORM_HEIGHT_KEYBOARD_CHANGE_TIME_S = 0.4f;
	const fp32 SWERVE_PLATFORM_HEIGHT_KEYBOARD_SEN_INC = (CHASSIS_H_UPPER_LIMIT - CHASSIS_H_LOWER_LIMIT) / SWERVE_PLATFORM_HEIGHT_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S;

	if ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) && (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_C))
	{
		swerve_chassis_back_home();
	}
	else
	{
		// roll
		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_B)
		{
			chassis_move.chassis_platform.target_roll += SWERVE_PLATFORM_ROLL_KEYBOARD_SEN_INC;
		}
		else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_C)
		{
			chassis_move.chassis_platform.target_roll -= SWERVE_PLATFORM_ROLL_KEYBOARD_SEN_INC;
		}

		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
		{
			// height
			if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
			{
				chassis_move.chassis_platform.target_height += SWERVE_PLATFORM_HEIGHT_KEYBOARD_SEN_INC;
			}
			else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
			{
				chassis_move.chassis_platform.target_height -= SWERVE_PLATFORM_HEIGHT_KEYBOARD_SEN_INC;
			}
		}
		else
		{
			// pitch
			if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
			{
				chassis_move.chassis_platform.target_pitch += SWERVE_PLATFORM_PITCH_KEYBOARD_SEN_INC;
			}
			else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
			{
				chassis_move.chassis_platform.target_pitch -= SWERVE_PLATFORM_PITCH_KEYBOARD_SEN_INC;
			}
		}
		// constrain target roll, pitch, and height values
		chassis_move.chassis_platform.target_roll = fp32_constrain(chassis_move.chassis_platform.target_roll, -CHASSIS_ROLL_UPPER_LIMIT, CHASSIS_ROLL_UPPER_LIMIT);
		chassis_move.chassis_platform.target_pitch = fp32_constrain(chassis_move.chassis_platform.target_pitch, -CHASSIS_PITCH_UPPER_LIMIT, CHASSIS_PITCH_UPPER_LIMIT);
		chassis_move.chassis_platform.target_height = fp32_constrain(chassis_move.chassis_platform.target_height, CHASSIS_H_LOWER_LIMIT, CHASSIS_H_UPPER_LIMIT);

		// chassis platform posture conversion
		swerve_convert_from_rpy_to_alpha(chassis_move.chassis_platform.target_roll, chassis_move.chassis_platform.target_pitch, &(chassis_move.chassis_platform.target_alpha1), &(chassis_move.chassis_platform.target_alpha2), chassis_move.chassis_yaw_motor->relative_angle);

		// constrain target alpha1 and alpha2 values
		// calculation for alpha limit: According to the matlab calculation, the available workspace in height-alpha space is triangular, so we assume height target has more priority than alpha target, and calculate alpha limit based on height
		if (chassis_move.chassis_platform.target_height >= CHASSIS_H_WORKSPACE_PEAK)
		{
			chassis_move.chassis_platform.alpha_upper_limit = (chassis_move.chassis_platform.target_height - CHASSIS_H_UPPER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE2;
		}
		else
		{
			chassis_move.chassis_platform.alpha_upper_limit = (chassis_move.chassis_platform.target_height - CHASSIS_H_LOWER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE1;
		}
		chassis_move.chassis_platform.alpha_lower_limit = -chassis_move.chassis_platform.alpha_upper_limit;

		chassis_move.chassis_platform.target_alpha1 = fp32_constrain(chassis_move.chassis_platform.target_alpha1, chassis_move.chassis_platform.alpha_lower_limit, chassis_move.chassis_platform.alpha_upper_limit);
		chassis_move.chassis_platform.target_alpha2 = fp32_constrain(chassis_move.chassis_platform.target_alpha2, chassis_move.chassis_platform.alpha_lower_limit, chassis_move.chassis_platform.alpha_upper_limit);

		// chassis platform posture inverse conversion to reflect limited alpha values onto the set roll and pitch values
		swerve_convert_from_alpha_to_rpy(&(chassis_move.chassis_platform.target_roll), &(chassis_move.chassis_platform.target_pitch), chassis_move.chassis_platform.target_alpha1, chassis_move.chassis_platform.target_alpha2, chassis_move.chassis_yaw_motor->relative_angle);
	}
}

void swerve_convert_from_rpy_to_alpha(fp32 roll, fp32 pitch, fp32 *alpha1, fp32 *alpha2, fp32 gimbal_chassis_relative_yaw_angle)
{
	// roll: positive tilting right
	// pitch: positive tilting down
	// alpha1: positive tilting front-right
	// alpha2: positive tilting rear-right

	// pseudo conversion
	// *alpha1 = (roll + pitch) / 2.0f;
	// *alpha2 = (roll - pitch) / 2.0f;

	fp32 cos_total = AHRS_cosf(PI / 4.0f + gimbal_chassis_relative_yaw_angle);
	fp32 sin_total = AHRS_sinf(PI / 4.0f + gimbal_chassis_relative_yaw_angle);
	*alpha2 = cos_total * roll - sin_total * pitch;
	*alpha1 = sin_total * roll + cos_total * pitch;

	// @TODO: implement conversion from roll & pitch to alpha1 & alpha2
	// fp32 cos_alpha1;
	// fp32 cos_alpha2;
	// // AHRS_acosf output range is [0, PI]
	// *alpha1 = AHRS_acosf();
}

void swerve_convert_from_alpha_to_rpy(fp32 *roll, fp32 *pitch, fp32 alpha1, fp32 alpha2, fp32 gimbal_chassis_relative_yaw_angle)
{
	// pseudo conversion
	// *roll = alpha1 + alpha2;
	// *pitch = alpha1 - alpha2;

	fp32 cos_total = AHRS_cosf(PI / 4.0f + gimbal_chassis_relative_yaw_angle);
	fp32 sin_total = AHRS_sinf(PI / 4.0f + gimbal_chassis_relative_yaw_angle);
	*roll = cos_total * alpha2 + sin_total * alpha1;
	*pitch = -sin_total * alpha2 + cos_total * alpha1;
}
#elif (ROBOT_TYPE == INFANTRY_2026_MECANUM)

void chassis_back_home(void)
{
	// back to middle height, which has the largest workspace for alpha angle
	chassis_move.chassis_platform.target_roll = 0;
	chassis_move.chassis_platform.target_pitch = 0;
	chassis_move.chassis_platform.target_alpha = 0;
	chassis_move.chassis_platform.target_height = CHASSIS_H_WORKSPACE_PEAK;
}

/**
 * @brief  Ramp chassis height toward CHASSIS_H_LOWER_LIMIT when gimbal is
 *         folded (or folding), and back to CHASSIS_H_WORKSPACE_PEAK when
 *         unfolded. Must be called every chassis control cycle.
 */
static void chassis_platform_fold_height_update(void)
{
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
	fp32 height_target;
	if (gimbal_control.gimbal_folding_status.target == FOLDED ||
	    gimbal_control.gimbal_folding_status.current == FOLDED)
	{
		height_target = CHASSIS_H_LOWER_LIMIT;
	}
	else
	{
		return; // No automatic height change when unfolded
	}

	if (chassis_move.chassis_platform.target_height > height_target)
	{
		chassis_move.chassis_platform.target_height -= CHASSIS_FOLD_HEIGHT_RAMP_RATE;
		if (chassis_move.chassis_platform.target_height < height_target)
		{
			chassis_move.chassis_platform.target_height = height_target;
		}
	}
	else if (chassis_move.chassis_platform.target_height < height_target)
	{
		chassis_move.chassis_platform.target_height += CHASSIS_FOLD_HEIGHT_RAMP_RATE;
		if (chassis_move.chassis_platform.target_height > height_target)
		{
			chassis_move.chassis_platform.target_height = height_target;
		}
	}

#endif
}

void chassis_platform_rc_mapping(void)
{
	chassis_platform_fold_height_update();

	// configuration for pseudo RPY-to-tilt conversion
	const fp32 CHASSIS_PLATFORM_ROLL_KEYBOARD_CHANGE_TIME_S = 0.4f;
	const fp32 CHASSIS_PLATFORM_ROLL_KEYBOARD_SEN_INC = 2.0f * CHASSIS_ALPHA_WORKSPACE_PEAK / CHASSIS_PLATFORM_ROLL_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S;
	const fp32 CHASSIS_PLATFORM_PITCH_KEYBOARD_CHANGE_TIME_S = 0.4f;
	const fp32 CHASSIS_PLATFORM_PITCH_KEYBOARD_SEN_INC = 2.5f * CHASSIS_ALPHA_WORKSPACE_PEAK / CHASSIS_PLATFORM_PITCH_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S;
	const fp32 CHASSIS_PLATFORM_ALPHA_KEYBOARD_SEN_INC = CHASSIS_PLATFORM_PITCH_KEYBOARD_SEN_INC;
	const fp32 CHASSIS_PLATFORM_HEIGHT_KEYBOARD_CHANGE_TIME_S = 0.4f;
	const fp32 CHASSIS_PLATFORM_HEIGHT_KEYBOARD_SEN_INC = (CHASSIS_H_UPPER_LIMIT - CHASSIS_H_LOWER_LIMIT) * 0.2f / CHASSIS_PLATFORM_HEIGHT_KEYBOARD_CHANGE_TIME_S * CHASSIS_CONTROL_TIME_S;

	if ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) && (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_C))
	{
		chassis_back_home();
	}
	else
	{
		// roll
		//if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_B)
		//{
		//	chassis_move.chassis_platform.target_roll += CHASSIS_PLATFORM_ROLL_KEYBOARD_SEN_INC;
		//}
		//else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_C)
		//{
		//	chassis_move.chassis_platform.target_roll -= CHASSIS_PLATFORM_ROLL_KEYBOARD_SEN_INC;
		//}

#if (REMOTE_TYPE == REMOTE_USE_VT13)
		if (chassis_move.chassis_RC->vt13.customizable_button_left)
		{
			chassis_move.chassis_platform.target_height -= CHASSIS_PLATFORM_HEIGHT_KEYBOARD_SEN_INC;
		}
		else if (chassis_move.chassis_RC->vt13.customizable_button_right)
		{
			chassis_move.chassis_platform.target_height += CHASSIS_PLATFORM_HEIGHT_KEYBOARD_SEN_INC;
		}
		else
#endif
		{
			if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
			{
				// height (Ctrl+F/V: increment/decrement)
				if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
				{
					chassis_move.chassis_platform.target_height += CHASSIS_PLATFORM_HEIGHT_KEYBOARD_SEN_INC;
				}
				else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
				{
					chassis_move.chassis_platform.target_height -= CHASSIS_PLATFORM_HEIGHT_KEYBOARD_SEN_INC;
				}
			}
			else
			{
				// F/V: adjust platform alpha
				if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
				{
					chassis_move.chassis_platform.target_alpha += CHASSIS_PLATFORM_ALPHA_KEYBOARD_SEN_INC;
				}
				else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
				{
					chassis_move.chassis_platform.target_alpha -= CHASSIS_PLATFORM_ALPHA_KEYBOARD_SEN_INC;
				}
			}
		}
		// constrain target roll, pitch, and height values
		//chassis_move.chassis_platform.target_roll = fp32_constrain(chassis_move.chassis_platform.target_roll, -CHASSIS_ROLL_UPPER_LIMIT, CHASSIS_ROLL_UPPER_LIMIT);
		//chassis_move.chassis_platform.target_pitch = fp32_constrain(chassis_move.chassis_platform.target_pitch, -CHASSIS_PITCH_UPPER_LIMIT, CHASSIS_PITCH_UPPER_LIMIT);
		chassis_move.chassis_platform.target_height = fp32_constrain(chassis_move.chassis_platform.target_height, CHASSIS_H_LOWER_LIMIT, CHASSIS_H_UPPER_LIMIT);
		chassis_move.chassis_platform.target_alpha = fp32_constrain(chassis_move.chassis_platform.target_alpha, -CHASSIS_ALPHA_WORKSPACE_PEAK, CHASSIS_ALPHA_WORKSPACE_PEAK);

		// chassis platform posture conversion
		//swerve_convert_from_rpy_to_alpha(chassis_move.chassis_platform.target_roll, chassis_move.chassis_platform.target_pitch, &(chassis_move.chassis_platform.target_alpha1), &(chassis_move.chassis_platform.target_alpha2), chassis_move.chassis_yaw_motor->relative_angle);

		// constrain target alpha1 and alpha2 values
		// calculation for alpha limit: According to the matlab calculation, the available workspace in height-alpha space is triangular, so we assume height target has more priority than alpha target, and calculate alpha limit based on height
		//if (chassis_move.chassis_platform.target_height >= CHASSIS_H_WORKSPACE_PEAK)
		//{
		//	chassis_move.chassis_platform.alpha_upper_limit = (chassis_move.chassis_platform.target_height - CHASSIS_H_UPPER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE2;
		//}
		//else
		//{
		//	chassis_move.chassis_platform.alpha_upper_limit = (chassis_move.chassis_platform.target_height - CHASSIS_H_LOWER_LIMIT) / CHASSIS_H_WORKSPACE_SLOPE1;
		//}
		//chassis_move.chassis_platform.alpha_lower_limit = -chassis_move.chassis_platform.alpha_upper_limit;
//
		//chassis_move.chassis_platform.target_alpha1 = fp32_constrain(chassis_move.chassis_platform.target_alpha1, chassis_move.chassis_platform.alpha_lower_limit, chassis_move.chassis_platform.alpha_upper_limit);
		//chassis_move.chassis_platform.target_alpha2 = fp32_constrain(chassis_move.chassis_platform.target_alpha2, chassis_move.chassis_platform.alpha_lower_limit, chassis_move.chassis_platform.alpha_upper_limit);

		// chassis platform posture inverse conversion to reflect limited alpha values onto the set roll and pitch values
		//swerve_convert_from_alpha_to_rpy(&(chassis_move.chassis_platform.target_roll), &(chassis_move.chassis_platform.target_pitch), chassis_move.chassis_platform.target_alpha1, chassis_move.chassis_platform.target_alpha2, chassis_move.chassis_yaw_motor->relative_angle);
	}
}
#endif

/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
static void chassis_set_control(void)
{
	fp32 vx_set = 0;
	fp32 vy_set = 0;
	fp32 wz_set = 0;
	fp32 angle_set = 0;
	chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, &angle_set);

	switch (chassis_move.chassis_coord_sys)
	{
		case CHASSIS_COORDINATE_FOLLOW_CHASSIS_ABSOLUTE_FRONT:
		{
			// direction of vx align with absolute imu front
			fp32 sin_yaw = AHRS_sinf(-chassis_move.chassis_yaw);
			fp32 cos_yaw = AHRS_cosf(-chassis_move.chassis_yaw);
			chassis_move.vx_set = cos_yaw * vx_set + (-sin_yaw) * vy_set;
			chassis_move.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;
			chassis_move.wz_set = wz_set;
			break;
		}
		case CHASSIS_COORDINATE_FOLLOW_GIMBAL:
		{
			fp32 sin_yaw = AHRS_sinf(chassis_move.chassis_yaw_motor->relative_angle);
			fp32 cos_yaw = AHRS_cosf(chassis_move.chassis_yaw_motor->relative_angle);
			chassis_move.vx_set = cos_yaw * vx_set + (-sin_yaw) * vy_set;
			chassis_move.vy_set = sin_yaw * vx_set + cos_yaw * vy_set;

			if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_MODE)
			{
				chassis_move.chassis_relative_angle_set = rad_format(angle_set);
				chassis_move.wz_set = -PID_calc(&chassis_move.chassis_angle_pid, chassis_move.chassis_yaw_motor->relative_angle, chassis_move.chassis_relative_angle_set, CHASSIS_CONTROL_TIME_S);
			}
			else
			{
				chassis_move.wz_set = wz_set;
			}
			break;
		}
		case CHASSIS_COORDINATE_FOLLOW_CHASSIS_RELATIVE_FRONT:
		default:
		{
			chassis_move.vx_set = vx_set;
			chassis_move.vy_set = vy_set;
			chassis_move.wz_set = wz_set;
		}
	}
	// speed limit
	chassis_move.vx_set = fp32_abs_constrain(chassis_move.vx_set, chassis_move.vx_max_speed);
	chassis_move.vy_set = fp32_abs_constrain(chassis_move.vy_set, chassis_move.vy_max_speed);
}

#if (WHEEL_TYPE == ROBOT_CHASSIS_USE_MECANUM)
/**
 * @brief          four mecanum wheels speed is calculated by three param.
 * @param[in]      vx_set: vertial speed
 * @param[in]      vy_set: horizontal speed
 * @param[in]      wz_set: rotation speed
 * @param[out]     wheel_speed: four mecanum wheels speed
 * @retval         none
 */
static void mecanum_chassis_vector_to_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	// because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
	wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * chassis_move.wheel_rot_radii[0] * wz_set;
	wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * chassis_move.wheel_rot_radii[1] * wz_set;
	wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * chassis_move.wheel_rot_radii[2] * wz_set;
	wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * chassis_move.wheel_rot_radii[3] * wz_set;
}
#elif (WHEEL_TYPE == ROBOT_CHASSIS_USE_OMNI)
/**
 * @brief          four omni wheels speed is calculated by three param.
 * @note           Omni wheels (Swedish wheels) are arranged in a square pattern.
 *                 Each wheel can move independently, providing omnidirectional motion
 *                 without the need for rotation of the wheel itself.
 * @param[in]      vx_set: vertical speed (forward/backward)
 * @param[in]      vy_set: horizontal speed (left/right)
 * @param[in]      wz_set: rotation speed (clockwise/counter-clockwise)
 * @param[out]     wheel_speed: four omni wheels speed
 * @retval         none
 */
static void omni_chassis_vector_to_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	// Omni wheel kinematics: each wheel contributes to forward, lateral, and rotational motion
	// Wheel layout: 0=front-right, 1=front-left, 2=rear-left, 3=rear-right
	// The rotation contribution is scaled by CHASSIS_WZ_SET_SCALE to account for gimbal position
	
	// Forward/backward contribution (vx)
	fp32 vx_contrib = vx_set;
	// Lateral contribution (vy)
	fp32 vy_contrib = vy_set;
	// Rotation contribution (wz) - scaled by distance to center
	fp32 wz_contrib = (CHASSIS_WZ_SET_SCALE - 1.0f) * chassis_move.wheel_rot_radii[0] * wz_set;
	
	// Calculate wheel speeds for omni configuration
	// Front-right wheel
	wheel_speed[0] = -vx_contrib - vy_contrib + wz_contrib;
	// Front-left wheel
	wheel_speed[1] = +vx_contrib - vy_contrib + wz_contrib;
	// Rear-left wheel
	wheel_speed[2] = +vx_contrib + vy_contrib + wz_contrib;
	// Rear-right wheel
	wheel_speed[3] = -vx_contrib + vy_contrib + wz_contrib;
}
#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
/**
 * @brief          four drive wheels' speeds and four steering wheels' angles are calculated by three chassis param.
 * @param[in]      vx_set: vertial speed (up is positive)
 * @param[in]      vy_set: horizontal speed (remote controller: left is positive; internal calculation: right is positive)
 * @param[in]      wz_set: rotation speed (counter-clockwise is positive)
 * @param[out]     wheel_speed: four drive wheels speed
 * @param[out]     steer_wheel_angle: four steering wheels angle. Angle between positive y-axis and total velocity
 * @retval         none
 */
void swerve_chassis_vector_to_wheel_vector(fp32 vx_set, fp32 vy_set, fp32 wz_set, fp32 wheel_speed[4], fp32 steer_wheel_angle[4])
{
	// remote controller: left is positive; internal calculation: right is positive
	vy_set = -vy_set;
	// because the gimbal is behind chassis, when chassis rotates, wheel 0 and wheel 1 should be faster and wheel 2 and wheel 3 should be slower
	// CHASSIS_WZ_SET_SCALE makes a coarse adjustment for that
	const fp32 sin_gamma = AHRS_sinf(CHASSIS_LEG_TO_HORIZONTAL_ANGLE);
	const fp32 cos_gamma = AHRS_cosf(CHASSIS_LEG_TO_HORIZONTAL_ANGLE);

	// velocity contributed by wheel rotation
	fp32 vx_by_wz[4];
	fp32 vy_by_wz[4];
	fp32 wz_set_adjusted_front_wheels = (1.0f + CHASSIS_WZ_SET_SCALE) * wz_set;
	fp32 wz_set_adjusted_rear_wheels = (1.0f - CHASSIS_WZ_SET_SCALE) * wz_set;
	// right front
	vx_by_wz[0] = wz_set_adjusted_front_wheels * (chassis_move.wheel_rot_radii[0] * sin_gamma);	 // wz_set * R * cos(gamma)
	vy_by_wz[0] = -wz_set_adjusted_front_wheels * (chassis_move.wheel_rot_radii[0] * cos_gamma); // wz_set * R * sin(gamma)
	// left front
	vx_by_wz[1] = -wz_set_adjusted_front_wheels * (chassis_move.wheel_rot_radii[1] * sin_gamma); // wz_set * R * cos(gamma)
	vy_by_wz[1] = -wz_set_adjusted_front_wheels * (chassis_move.wheel_rot_radii[1] * cos_gamma); // wz_set * R * sin(gamma)
	// left rear
	vx_by_wz[2] = -wz_set_adjusted_rear_wheels * (chassis_move.wheel_rot_radii[2] * sin_gamma); // wz_set * R * cos(gamma)
	vy_by_wz[2] = wz_set_adjusted_rear_wheels * (chassis_move.wheel_rot_radii[2] * cos_gamma);	// wz_set * R * sin(gamma)
	// right rear
	vx_by_wz[3] = wz_set_adjusted_rear_wheels * (chassis_move.wheel_rot_radii[2] * sin_gamma); // wz_set * R * cos(gamma)
	vy_by_wz[3] = wz_set_adjusted_rear_wheels * (chassis_move.wheel_rot_radii[2] * cos_gamma); // wz_set * R * sin(gamma)

	// velocity contributed by hip
	fp32 vx_by_hip[4];
	fp32 vy_by_hip[4];
	if (chassis_move.fHipEnabled)
	{
		// right front
		vx_by_hip[0] = chassis_move.target_wheel_rot_radii_dot[0] * sin_gamma;
		vy_by_hip[0] = chassis_move.target_wheel_rot_radii_dot[0] * cos_gamma;
		// left front
		vx_by_hip[1] = chassis_move.target_wheel_rot_radii_dot[1] * sin_gamma;
		vy_by_hip[1] = -chassis_move.target_wheel_rot_radii_dot[1] * cos_gamma;
		// left rear
		vx_by_hip[2] = -chassis_move.target_wheel_rot_radii_dot[2] * sin_gamma;
		vy_by_hip[2] = -chassis_move.target_wheel_rot_radii_dot[2] * cos_gamma;
		// right rear
		vx_by_hip[3] = -chassis_move.target_wheel_rot_radii_dot[3] * sin_gamma;
		vy_by_hip[3] = chassis_move.target_wheel_rot_radii_dot[3] * cos_gamma;
	}
	else
	{
		fp32 temp_wheel_rot_radii_dot = STEER_TURN_HIP_RADIUS_SPEED_DEADZONE * 0.9f;
		// right front
		vx_by_hip[0] = temp_wheel_rot_radii_dot * sin_gamma;
		vy_by_hip[0] = temp_wheel_rot_radii_dot * cos_gamma;
		// left front
		vx_by_hip[1] = temp_wheel_rot_radii_dot * sin_gamma;
		vy_by_hip[1] = -temp_wheel_rot_radii_dot * cos_gamma;
		// left rear
		vx_by_hip[2] = -temp_wheel_rot_radii_dot * sin_gamma;
		vy_by_hip[2] = -temp_wheel_rot_radii_dot * cos_gamma;
		// right rear
		vx_by_hip[3] = -temp_wheel_rot_radii_dot * sin_gamma;
		vy_by_hip[3] = temp_wheel_rot_radii_dot * cos_gamma;
	}

	// pairs of velocities represented by (x,y)
	fp32 wheel_velocity[4][2] = {
		{vx_set + vx_by_wz[0] + vx_by_hip[0], vy_set + vy_by_wz[0] + vy_by_hip[0]},
		{vx_set + vx_by_wz[1] + vx_by_hip[1], vy_set + vy_by_wz[1] + vy_by_hip[1]},
		{vx_set + vx_by_wz[2] + vx_by_hip[2], vy_set + vy_by_wz[2] + vy_by_hip[2]},
		{vx_set + vx_by_wz[3] + vx_by_hip[3], vy_set + vy_by_wz[3] + vy_by_hip[3]},
	};

#if STEER_MOTOR_UPSIDE_DOWN_MOUNTING
	wheel_velocity[0][1] = -wheel_velocity[0][1];
	wheel_velocity[1][1] = -wheel_velocity[1][1];
	wheel_velocity[2][1] = -wheel_velocity[2][1];
	wheel_velocity[3][1] = -wheel_velocity[3][1];
#endif

	static fp32 last_steer_wheel_angle_target[4];
	static uint8_t reverse_flag[4];
	uint8_t i;
	// On the edge of turning off, vx,vy,vz are all zero yet 6020 still needs to turn
	uint8_t fNoChangeUniversal = ((chassis_move.fHipDisabledEdge == 0) && (fabs(vy_set) <= STEER_TURN_X_SPEED_DEADZONE) && (fabs(vx_set) < STEER_TURN_X_SPEED_DEADZONE) && (fabs(wz_set) < STEER_TURN_W_SPEED_DEADZONE));
	uint8_t fNoChangeUnique;
	for (i = 0; i < 4; i++)
	{
		// drive wheel speed
		wheel_speed[i] = sqrt(pow(wheel_velocity[i][0], 2) + pow(wheel_velocity[i][1], 2));

		// determine whether to calculate steering wheel angle again
		if (chassis_move.fHipEnabled)
		{
			fNoChangeUnique = (fabs(chassis_move.target_wheel_rot_radii_dot[i]) < STEER_TURN_HIP_RADIUS_SPEED_DEADZONE);
		}
		else
		{
			if (chassis_move.fHipDisabledEdge)
			{
				// do change steer to release tension at the edge of reset
				fNoChangeUnique = 0;
			}
			else
			{
				// do not change steer after reset
				fNoChangeUnique = 1;
			}
		}

		if (fNoChangeUniversal && fNoChangeUnique)
		{
			steer_wheel_angle[i] = last_steer_wheel_angle_target[i];
		}
		else
		{
			// (https://en.cppreference.com/w/c/numeric/math/atan2)
			// steer_wheel_angle: unit rad; range is [-PI, PI]; positive direction is clockwise
			steer_wheel_angle[i] = atan2f(wheel_velocity[i][1], wheel_velocity[i][0]);

			reverse_flag[i] = (fabs(rad_format(steer_wheel_angle[i] - last_steer_wheel_angle_target[i])) > PI / 2);
			if (reverse_flag[i])
			{
				// if angle between last and target is greater than 90 deg, simply reverse drive wheel reduces time to turn
				steer_wheel_angle[i] = rad_format(steer_wheel_angle[i] + PI);
			}

			last_steer_wheel_angle_target[i] = steer_wheel_angle[i];
		}

		if (reverse_flag[i])
		{
			wheel_speed[i] = -wheel_speed[i];
		}
	}

	// reverse direction because of special initial direction
	wheel_speed[0] = -wheel_speed[0];
	wheel_speed[3] = -wheel_speed[3];
}
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
void biped_chassis_vector_to_speed(fp32 vx_set, fp32 vy_set, fp32 wz_set)
{
	uint8_t fRotationHasHigherPriority = 0;
	uint8_t fNoPowerMode = 0;
	if (chassis_behaviour_mode == CHASSIS_BASIC_FPV_MODE)
	{
		// wz_set has higher priority than vx_set and vy_set by default
		fRotationHasHigherPriority = (fabs(wz_set) > BIPED_W_SPEED_DEADZONE);
	}
	else if (chassis_behaviour_mode == CHASSIS_SPINNING_MODE)
	{
		// vx_set and vy_set has higher priority than wz_set by default
		fRotationHasHigherPriority = ((fabs(vx_set) < BIPED_X_SPEED_DEADZONE) && (fabs(vy_set) < BIPED_Y_SPEED_DEADZONE));
	}
	else
	{
		fNoPowerMode = 1;
	}

	if (fNoPowerMode)
	{
		// do nothing, all values will be zero
	}
	else if (fRotationHasHigherPriority)
	{
		// wz_set has higher priority
		chassis_move.chassis_platform.target_yaw = rad_format(chassis_move.chassis_platform.target_yaw + wz_set * CHASSIS_CONTROL_TIME_S);
		chassis_move.chassis_platform.target_dis_dot = 0;
	}
	else // vx_set and vy_set has higher priority
	{
		// vy_set: left is positive; vx_set: forward is positive
		// relative_yaw_target: range is [-PI, PI]; positive direction is CCW from chassis to gimbal direction; default value should be 0 which matches initial head state
		// chassis_yaw_motor->relative_angle: range is [-PI, PI]; positive direction is CW
		static fp32 last_relative_yaw_target = 0;
		fp32 relative_yaw_target = 0;
		uint8_t fYawTurnEnabled = 0;
		// avoid frequently changing direction when joystick is closed to center
		if ((fabs(vx_set) < BIPED_X_SPEED_DEADZONE) && (fabs(vy_set) < BIPED_Y_SPEED_DEADZONE))
		{
			relative_yaw_target = last_relative_yaw_target;
		}
		else
		{
			relative_yaw_target = atan2f(-vy_set, vx_set);
			last_relative_yaw_target = relative_yaw_target;
			fYawTurnEnabled = 1;
		}

		// calc relative_yaw_target_diff
		fp32 fMoveDirectionReversed = 1;
		fp32 relative_yaw_target_diff = rad_format(chassis_move.chassis_yaw_motor->relative_angle - relative_yaw_target);
		if (fabs(relative_yaw_target_diff) > PI / 2.0f)
		{
			fMoveDirectionReversed = -1;
			relative_yaw_target_diff = rad_format(relative_yaw_target_diff + PI);
		}

		// yaw turn cmd
		if (fYawTurnEnabled)
		{
			chassis_move.chassis_platform.target_yaw = rad_format(chassis_move.chassis_platform.feedback_yaw + relative_yaw_target_diff);
		}

		// do not translate until chassis align with intended moving direction
		const fp32 BIPED_GIMBAL_ALIGN_ANGLE_DEADZONE = DEG_TO_RAD(10.0f);
		if (fabs(relative_yaw_target_diff) > BIPED_GIMBAL_ALIGN_ANGLE_DEADZONE)
		{
			chassis_move.chassis_platform.target_dis_dot = 0;
		}
		else
		{
			chassis_move.chassis_platform.target_dis_dot = fMoveDirectionReversed * sqrtf(vx_set * vx_set + vy_set * vy_set);
		}
	}
}
#endif

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @retval         none
 */
static void chassis_control_loop(void)
{
#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
	biped_chassis_vector_to_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set);
#else
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	//fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // unit m/s
	uint8_t i = 0;

#if (WHEEL_TYPE == ROBOT_CHASSIS_USE_MECANUM)
	// mecanum chassis inverse kinematics
	mecanum_chassis_vector_to_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);
#elif (WHEEL_TYPE == ROBOT_CHASSIS_USE_OMNI)
	// omni chassis inverse kinematics
	omni_chassis_vector_to_wheel_speed(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed);

#elif (ROBOT_TYPE == INFANTRY_2023_SWERVE)
	// swerve chassis inverse kinematics
	fp32 steer_wheel_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // unit rad
	swerve_chassis_vector_to_wheel_vector(chassis_move.vx_set, chassis_move.vy_set, chassis_move.wz_set, wheel_speed, steer_wheel_angle);
#endif

	if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
	{
		for (i = 0; i < 4; i++)
		{
			chassis_move.motor_chassis[i].give_chassis_motor_cmd = (int16_t)(wheel_speed[i]);
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
			chassis_move.steer_motor_chassis[i].target_ecd = motor_angle_to_ecd_change(steer_wheel_angle[i]);
#endif
		}
	}
	else
	{
		for (i = 0; i < 4; i++)
		{
			// calculate the max speed in four wheels, limit the max speed
			chassis_move.motor_chassis[i].speed_set = wheel_speed[i];
			temp = fabs(chassis_move.motor_chassis[i].speed_set);
			if (max_vector < temp)
			{
				max_vector = temp;
			}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
			chassis_move.steer_motor_chassis[i].target_ecd = motor_angle_to_ecd_change(steer_wheel_angle[i]);
#endif
		}

		if (max_vector > MAX_WHEEL_SPEED)
		{
			vector_rate = MAX_WHEEL_SPEED / max_vector;
			for (i = 0; i < 4; i++)
			{
				chassis_move.motor_chassis[i].speed_set *= vector_rate;
			}
		}

		// calculate pid
		for (i = 0; i < 4; i++)
		{
			PID_calc(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, chassis_move.motor_chassis[i].speed_set, CHASSIS_CONTROL_TIME_S);
		}

#if CHASSIS_POWER_CONTROL
		if ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT) && (can_ref_info.chassis_power_buffer > 30)) // when shift pressed and buffer is above safe value, bypass powercontrol (leave to supercap to control)
		{
			for (i = 0; i < 4; i++)
			{
#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)

				chassis_move.motor_chassis[i].give_chassis_motor_cmd = (int16_t)fp32_constrain(
					(chassis_move.motor_chassis[i].speed_set + chassis_move.motor_speed_pid[i].out) * MOTOR_ROTOR_TO_OUTPUT_CONSTANT,
					-(fp32)MOTOR_MG4010_MAX_CMD, (fp32)MOTOR_MG4010_MAX_CMD);
#else
				chassis_move.motor_chassis[i].give_chassis_motor_cmd = (int16_t)((chassis_move.motor_speed_pid[i].out) * MOTOR_ROTOR_TO_OUTPUT_CONSTANT);
#endif
			}
		}
		else
		{
			chassis_power_control(); // give_chassis_motor_cmd assigned inside
		}

#else
		for (i = 0; i < 4; i++)
		{
			chassis_move.motor_chassis[i].give_chassis_motor_cmd = (int16_t)((chassis_move.motor_speed_pid[i].out) * MOTOR_ROTOR_TO_OUTPUT_CONSTANT);
		}
#endif
	}
#endif
}

fp32 chassis_get_high_wz_limit(void)
{
	return (chassis_move.wz_max_speed * 0.833f);
}

fp32 chassis_get_med_wz_limit(void)
{
	return (chassis_move.wz_max_speed * 0.667f);
}

fp32 chassis_get_low_wz_limit(void)
{
	return (chassis_move.wz_max_speed * 0.583f);
}

fp32 chassis_get_ultra_low_wz_limit(void)
{
	return (chassis_move.wz_max_speed * 0.167f);
}

/**
 * @brief          encoder-only, slip-mitigated chassis translational velocity estimate (chassis frame).
 *                 Runs every chassis cycle from chassis_feedback_update(). No IMU is used.
 *
 * Best-effort slip handling with wheel encoders only:
 *  1. Redundancy residual: a 4-wheel chassis provides 4 speed measurements but only 3 DOF
 *     (vx, vy, wz). The one redundant wheel-speed combination must be ~0 under pure rolling, so a
 *     nonzero value is a direct slip / inconsistency indicator (orthogonal to the vx/vy/wz mixing).
 *  2. Single-wheel rejection: when slip is detected, the wheel with clearly anomalous acceleration
 *     (the classic signature of a wheel losing traction) is corrected back to the value implied by
 *     the other three wheels, which is equivalent to dropping it from the solve.
 *  3. Acceleration clamp: the frame-to-frame velocity change is limited to a physically plausible
 *     chassis acceleration, rejecting the non-physical jumps a brief slip spike produces.
 *  4. Adaptive low-pass: filtering is light when the wheels agree (responsive) and heavy while slip
 *     is present (coast on the previous estimate instead of trusting bad wheel data).
 *
 * Note: a symmetric 4-wheel chassis cannot identify WHICH wheel slips from a single instantaneous
 * sample alone, so step 2 leans on wheel acceleration and only acts when one wheel clearly dominates;
 * otherwise steps 3-4 simply de-weight the estimate. Sustained multi-wheel slip is not recoverable
 * without an independent reference (e.g. IMU), which is intentionally not used here.
 */
#if (ROBOT_TYPE != INFANTRY_2024_BIPED) && (ROBOT_TYPE != INFANTRY_2023_SWERVE)
static void chassis_slip_robust_vel_update(void)
{
	// slip-tuning constants (chassis frame; speeds in m/s, accelerations in m/s^2)
	const fp32 SLIP_RESIDUAL_THRESHOLD = 0.30f;  // |redundant wheel combo| above this => slip suspected
	const fp32 SLIP_ACCEL_THRESHOLD    = 8.0f;   // a wheel must exceed this |accel| to be treated as slipping
	const fp32 SLIP_ACCEL_DOMINANCE    = 2.0f;   // and exceed the 2nd-most-dynamic wheel by this factor
	const fp32 SLIP_MAX_CHASSIS_ACCEL  = 20.0f;  // plausible chassis accel ceiling for the velocity clamp
	const fp32 SLIP_TAU_NORMAL         = 0.015f; // low-pass time constant when wheels agree
	const fp32 SLIP_TAU_SLIP           = 0.080f; // heavier low-pass while slipping

	fp32 s[4];
	fp32 a[4];
	for (uint8_t i = 0; i < 4; i++)
	{
		s[i] = chassis_move.motor_chassis[i].speed; // wheel surface speed, m/s (encoder)
		a[i] = chassis_move.motor_chassis[i].accel; // wheel accel, m/s^2 (encoder-derived)
	}

	// residual sign pattern: orthogonal to the vx/vy/wz mixing used in chassis_feedback_update,
	// so it captures only the slip/inconsistency component of the four wheel speeds
#if (WHEEL_TYPE == ROBOT_CHASSIS_USE_MECANUM)
	const fp32 r[4] = {1.0f, -1.0f, 1.0f, -1.0f};
#elif (WHEEL_TYPE == ROBOT_CHASSIS_USE_OMNI)
	// redundant (null-space) wheel combination for this omni layout: orthogonal to vx, vy and rot,
	// so it isolates slip only. {1,1,-1,-1} would instead equal -4*vy (a real motion axis) and mis-fire.
	const fp32 r[4] = {1.0f, -1.0f, 1.0f, -1.0f};
#else
	const fp32 r[4] = {0.0f, 0.0f, 0.0f, 0.0f};
#endif

	fp32 residual = r[0] * s[0] + r[1] * s[1] + r[2] * s[2] + r[3] * s[3];
	uint8_t slip_detected = (fabs(residual) > SLIP_RESIDUAL_THRESHOLD);

	if (slip_detected)
	{
		// find the most dynamic wheel and the runner-up; only act if one clearly dominates
		uint8_t k = 0;
		uint8_t second = 1;
		for (uint8_t i = 1; i < 4; i++)
		{
			if (fabs(a[i]) > fabs(a[k]))
			{
				second = k;
				k = i;
			}
		}
		if ((fabs(a[k]) > SLIP_ACCEL_THRESHOLD) &&
		    (fabs(a[k]) > SLIP_ACCEL_DOMINANCE * fabs(a[second])) &&
		    (r[k] != 0.0f))
		{
			// correct the slipping wheel to the speed implied by the other three (drops it from the solve)
			s[k] -= residual * r[k];
		}
	}

	// chassis-frame velocity from the (possibly corrected) wheel speeds, same mixing as chassis_feedback_update
	fp32 vx_raw = 0.0f;
	fp32 vy_raw = 0.0f;
#if (WHEEL_TYPE == ROBOT_CHASSIS_USE_MECANUM)
	vx_raw = (-s[0] + s[1] + s[2] - s[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	vy_raw = (-s[0] - s[1] + s[2] + s[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
#elif (WHEEL_TYPE == ROBOT_CHASSIS_USE_OMNI)
	vx_raw = (-s[0] + s[1] + s[2] - s[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	vy_raw = (-s[0] - s[1] + s[2] + s[3]) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
#endif

	// acceleration clamp: reject the non-physical velocity jumps a slip spike produces
	const fp32 dt = CHASSIS_CONTROL_TIME_S;
	fp32 max_dv = SLIP_MAX_CHASSIS_ACCEL * dt;
	fp32 vx_clamped = chassis_vx_slip_robust + fp32_constrain(vx_raw - chassis_vx_slip_robust, -max_dv, max_dv);
	fp32 vy_clamped = chassis_vy_slip_robust + fp32_constrain(vy_raw - chassis_vy_slip_robust, -max_dv, max_dv);

	// adaptive low-pass: trust the encoders less while slip is present
	fp32 tau = slip_detected ? SLIP_TAU_SLIP : SLIP_TAU_NORMAL;
	fp32 alpha = dt / (tau + dt);
	chassis_vx_slip_robust += alpha * (vx_clamped - chassis_vx_slip_robust);
	chassis_vy_slip_robust += alpha * (vy_clamped - chassis_vy_slip_robust);
}
#endif

/**
 * @brief          chassis translational velocity expressed in the gimbal-yaw frame, derived purely from
 *                 chassis motor encoder feedback (no IMU). vx points where the yaw axis is aimed, vy is 90deg to its left.
 * @param[out]     vx: forward speed in the gimbal-yaw frame, unit m/s
 * @param[out]     vy: left speed in the gimbal-yaw frame, unit m/s
 * @retval         none
 */
void get_chassis_vel_in_gimbal_frame(fp32 *vx, fp32 *vy)
{
	if (vx == NULL || vy == NULL)
	{
		return;
	}
#if (ROBOT_TYPE == INFANTRY_2024_BIPED) || (ROBOT_TYPE == INFANTRY_2023_SWERVE)
	// these builds do not reconstruct chassis vx/vy from 4-wheel encoder feedback
	*vx = 0.0f;
	*vy = 0.0f;
#else
	// rotate the encoder-only, slip-mitigated chassis-frame velocity into the gimbal-yaw frame using the
	// yaw motor relative angle (gimbal yaw relative to chassis front); inverse of the gimbal->chassis
	// rotation applied in chassis_set_control
	fp32 angle = chassis_move.chassis_yaw_motor->relative_angle;
	fp32 sin_yaw = AHRS_sinf(angle);
	fp32 cos_yaw = AHRS_cosf(angle);
	*vx = cos_yaw * chassis_vx_slip_robust + sin_yaw * chassis_vy_slip_robust;
	*vy = -sin_yaw * chassis_vx_slip_robust + cos_yaw * chassis_vy_slip_robust;
#endif
}

fp32 calc_wz_max_speed(fp32 vx_speed_limit, fp32 vy_speed_limit, fp32 wz_scaling_factor)
{
	const fp32 WZ_MIN_RATIO = 0.25f; // keep 25% turning ability at high speed

	fp32 wz_max_speed = 0.0f;
 
	fp32 vx_norm = chassis_move.vx_set / fp32_constrain(vx_speed_limit, 0.01f, NORMAL_MAX_CHASSIS_SPEED_X);;
	fp32 vy_norm = chassis_move.vy_set / fp32_constrain(vy_speed_limit, 0.01f, NORMAL_MAX_CHASSIS_SPEED_Y);;

	fp32 v_norm = sqrtf(vx_norm * vx_norm + vy_norm * vy_norm);
	v_norm = fp32_constrain(v_norm, 0.0f, 1.0f);

	fp32 wz_decay_by_v_coeff =
		WZ_MIN_RATIO + (1.0f - WZ_MIN_RATIO) * (1.0f - v_norm);

	wz_max_speed = SPINNING_CHASSIS_MAX_OMEGA * wz_decay_by_v_coeff * wz_scaling_factor;
	return wz_max_speed;
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
/**
 * @brief Convert motor angle from radian to encoder unit
 * Requirements:
 *    0 rad = 0 ecd
 *    input and output increase in the same clockwise direction
 * @param[in] angle range [-PI, PI]
 * @param[in] ecd range [0, ECD_RANGE-1]
 */
static uint16_t motor_angle_to_ecd_change(fp32 angle)
{
	return (uint16_t)(loop_fp32_constrain(angle, 0.0f, 2 * PI) * MOTOR_RAD_TO_ECD);
}
#endif
