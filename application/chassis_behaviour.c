/**
****************************(C) COPYRIGHT 2019 DJI****************************
* @file       chassis_behaviour.c/h
* @brief      according to remote control, change the chassis behaviour.
* @note
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-26-2018     RM              1. done
*  V1.1.0     Nov-11-2019     RM              1. add some annotation
*
****************************(C) COPYRIGHT 2019 DJI****************************
*/

#include "chassis_behaviour.h"
#include "AHRS_middleware.h"
#include "bsp_rng.h" // for random number generator
#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "referee.h"

#include "cv_usart_task.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"

// random spin mode parameters
#define MIN_SPIN_PARAM_CHANGE_PERIOD 1.0f
#define NORMAL_SPIN_PARAM_CHANGE_PERIOD 5.0f
#define DELTA_SPIN_PARAM_CHANGE_PERIOD 2.0f
#define MIN_SPIN_SPEED_CHANGE_PERIOD 250.0f
#define MID_SPIN_SPEED_CHANGE_PERIOD 1000.0f
#define DELTA_SPIN_SPEED_CHANGE_PERIOD (MID_SPIN_SPEED_CHANGE_PERIOD / 2.0f)

#define MOUSE_SCROLL_TO_DIAL_SEN_INC -(JOYSTICK_HALF_RANGE / MOUSE_X_EFFECTIVE_SPEED * 30)
#define MOUSE_SCROLL_FILTER_COEFF 0.6f
#define CHASSIS_WZ_CMD_DEADZONE 0.15f

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis control mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @retval         none
 */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set);

/**
 * @brief          when chassis behaviour mode is CHASSIS_FOLLOW_GIMBAL_MODE, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @retval         none
 */
static void chassis_follow_gimbal_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);

static void chassis_cv_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

/**
 * @brief          when chassis behaviour mode is CHASSIS_BASIC_FPV_MODE, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @retval         none
 */
static void chassis_basic_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, uint8_t fSpinningOn);
void chassis_align_to_gimbal(fp32* wz_set);
void chassis_align_to_imu_front(fp32* wz_set);

void chassis_spinning_speed_manager(fp32 *wz_set);
void dial_channel_manager(void);

// Watchout for the default value of chassis behaviour mode
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

/**
 * @brief          logical judgementssign "chassis_behaviour_mode" variable to which mode
 * @retval         none
 */
void chassis_behaviour_set_mode(void)
{
	if ((chassis_behaviour_mode == CHASSIS_CV_CONTROL_MODE) && toe_is_error(DBUS_TOE))
	{
		; // CV fully automatic mode without RC: do not switch out of cv state
	}
	else if (gimbal_emergency_stop() || gimbal_cmd_to_chassis_stop())
	{
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	}
	else if ((toe_is_error(REFEREE_TOE) == 0) && (robot_state.power_management_chassis_output == 0))
	{
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	}
	else
	{
		switch (chassis_move.chassis_RC->rc.s[RC_RIGHT_LEVER_CHANNEL])
		{
			case RC_SW_UP:
			{
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
				chassis_behaviour_mode = CHASSIS_CV_CONTROL_MODE;
#else
				chassis_behaviour_mode = CHASSIS_SPINNING_MODE;
#endif
				break;
			}
			case RC_SW_MID:
			{
				// Remember to change gimbal_behaviour logic correspondingly
				chassis_behaviour_mode = CHASSIS_BASIC_FPV_MODE;
				break;
			}
			case RC_SW_DOWN:
			default:
			{
				chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
				break;
			}
		}
	}

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	CvCmder_ChangeMode(CV_MODE_AUTO_AIM_BIT | CV_MODE_AUTO_MOVE_BIT, (chassis_behaviour_mode == CHASSIS_CV_CONTROL_MODE));
	chassis_move.fUpperHeadEnabled = (chassis_behaviour_mode == CHASSIS_CV_CONTROL_MODE);
#endif

	switch (chassis_behaviour_mode)
	{
		case CHASSIS_FOLLOW_GIMBAL_MODE:
		case CHASSIS_BASIC_FPV_MODE:
		case CHASSIS_SPINNING_MODE:
		{
			chassis_move.chassis_coord_sys = CHASSIS_COORDINATE_FOLLOW_GIMBAL;
			break;
		}
		case CHASSIS_CV_CONTROL_MODE:
		{
			chassis_move.chassis_coord_sys = CHASSIS_COORDINATE_FOLLOW_CHASSIS_ABSOLUTE_FRONT;
			break;
		}
		case CHASSIS_ZERO_FORCE:
		default:
		{
			chassis_move.chassis_coord_sys = CHASSIS_COORDINATE_FOLLOW_CHASSIS_RELATIVE_FRONT;
			break;
		}
	}
}

void chassis_behaviour_change_transit(void)
{
	static chassis_behaviour_e last_chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	if (last_chassis_behaviour_mode != chassis_behaviour_mode)
	{
		if (last_chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
		{
			chassis_enable_platform_flag(1);
		}

		switch (chassis_behaviour_mode)
		{
			case CHASSIS_FOLLOW_GIMBAL_MODE:
			{
				chassis_move.chassis_relative_angle_set = 0;
				break;
			}
			case CHASSIS_CV_CONTROL_MODE:
			{
#if CV_INTERFACE
				// safety guard
				CvCmder_ChangeMode(CV_MODE_CHASSIS_SPINNING_BIT, 0);
#endif
				break;
			}
			case CHASSIS_SPINNING_MODE:
			{
				// Relative angle implementation for chassis spinning mode
				// chassis_move.chassis_relative_angle_set = chassis_move.chassis_yaw_motor->relative_angle;
				break;
			}
			case CHASSIS_BASIC_FPV_MODE:
			{
				chassis_move.chassis_yaw_set = chassis_move.chassis_yaw;
				break;
			}
			case CHASSIS_ZERO_FORCE:
			{
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
				swerve_chassis_params_reset();
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
				biped_chassis_params_reset();
#endif
				break;
			}
			default:
			{
				break;
			}
		}
#if (ROBOT_TYPE == INFANTRY_2024_BIPED)
		chassis_move.chassis_platform.target_yaw = chassis_move.chassis_platform.feedback_yaw;
#endif
		chassis_move.dial_channel_latched = 0;
		last_chassis_behaviour_mode = chassis_behaviour_mode;
	}
}

/**
 * @brief          set control set-point. three movement param, according to difference control mode,
 *                 will control corresponding movement.in the function, usually call different control function.
 * @param[out]     vx_set, usually controls vertical speed.
 * @param[out]     vy_set, usually controls horizotal speed.
 * @param[out]     wz_set, usually controls rotation speed.
 * @retval         none
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, fp32 *angle_set)
{
	if ((vx_set == NULL || vy_set == NULL || wz_set == NULL || angle_set == NULL) == 0)
	{
		dial_channel_manager();

		switch (chassis_behaviour_mode)
		{
			case CHASSIS_FOLLOW_GIMBAL_MODE:
			{
				chassis_follow_gimbal_control(vx_set, vy_set, angle_set);
				break;
			}
			case CHASSIS_BASIC_FPV_MODE:
			{
				uint8_t fSpinningOn = 0;
				chassis_basic_control(vx_set, vy_set, wz_set, fSpinningOn);
				break;
			}
			case CHASSIS_SPINNING_MODE:
			{
				uint8_t fSpinningOn = 1;
				chassis_basic_control(vx_set, vy_set, wz_set, fSpinningOn);
				break;
			}
			case CHASSIS_CV_CONTROL_MODE:
			{
				chassis_cv_control(vx_set, vy_set, wz_set);
				break;
			}
			case CHASSIS_ZERO_FORCE:
			default:
			{
				chassis_zero_force_control(vx_set, vy_set, wz_set);
				break;
			}
		}
	}
}

void dial_channel_manager(void)
{
	int16_t dial_channel_raw;
	deadband_limit(chassis_move.chassis_RC->rc.ch[RC_DIAL_CHANNEL], dial_channel_raw, CHASSIS_RC_DEADLINE);
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	if (chassis_behaviour_mode == CHASSIS_CV_CONTROL_MODE)
	{
		// if (toe_is_error(DBUS_TOE))
		// {
		// 	// CV fully automatic mode without RC
		// 	chassis_move.dial_channel_out = chassis_move.dial_channel_latched;
		// }
		// else
		// {
		// 	chassis_move.dial_channel_latched = dial_channel_raw;
		// 	chassis_move.dial_channel_out = dial_channel_raw;
		// }
		// hardcode fastest spinning
		chassis_move.dial_channel_out = JOYSTICK_HALF_RANGE;
	}
	else
	{
		chassis_move.dial_channel_out = dial_channel_raw;
	}
#else
	if (toe_is_error(DBUS_TOE) == 0)
	{
		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
		{
			chassis_move.dial_channel_latched = dial_channel_raw;
		}

		if (dial_channel_raw == 0)
		{
			chassis_move.dial_channel_out = chassis_move.dial_channel_latched;
		}
		else
		{
			chassis_move.dial_channel_out = dial_channel_raw;
		}
	}
	else
	{
		chassis_move.dial_channel_out = 0;
	}
	// Add mouse scroll input
	static fp32 last_mouse_z_ch = 0;
	fp32 mouse_z_ch = first_order_filter(chassis_move.chassis_RC->mouse.z, last_mouse_z_ch, MOUSE_SCROLL_FILTER_COEFF);
	chassis_move.dial_channel_out += mouse_z_ch * MOUSE_SCROLL_TO_DIAL_SEN_INC;
#endif
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @retval         none
 */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set)
{
	if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL)
	{
		return;
	}
	*vx_can_set = 0;
	*vy_can_set = 0;
	*wz_can_set = 0;
	chassis_move.chassis_cmd_slow_set_vx.out = 0;
	chassis_move.chassis_cmd_slow_set_vy.out = 0;
	chassis_move.chassis_cmd_slow_set_wz.out = 0;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_FOLLOW_GIMBAL_MODE, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @retval         none
 */
static void chassis_follow_gimbal_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
	{
		return;
	}

	// Convert joystick and keyboard input to commands
	chassis_rc_to_control_vector(vx_set, vy_set);

	// // swing angle is generated by sin function, swing_time is the input to sine function
	// static fp32 swing_time = 0.0f;
	// static fp32 swing_angle = 0.0f;
	// // max_angle is the max angle that chassis will rotate
	// static fp32 max_angle = SWING_NO_MOVE_ANGLE;
	// // swing_time increment by add_time per control cycle
	// static fp32 const add_time = PI * 0.5f / CHASSIS_CONTROL_TIME_S;

	// static uint8_t swing_flag = 0;

	// // judge if are to swing
	// if (chassis_move.chassis_RC->key.v & SWING_KEY)
	// {
	// 	if (swing_flag == 0)
	// 	{
	// 		swing_flag = 1;
	// 		swing_time = 0.0f;
	// 	}
	// }
	// else
	// {
	// 	swing_flag = 0;
	// }

	// // judge if keyboard is controlling the chassis, if yes, reduce the max_angle
	// if (chassis_move.chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move.chassis_RC->key.v & CHASSIS_BACK_KEY ||
	//     chassis_move.chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move.chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	// {
	// 	max_angle = SWING_MOVE_ANGLE;
	// }
	// else
	// {
	// 	max_angle = SWING_NO_MOVE_ANGLE;
	// }

	// if (swing_flag)
	// {
	// 	swing_angle = max_angle * AHRS_sinf(swing_time);
	// 	swing_time += add_time;
	// }
	// else
	// {
	// 	swing_angle = 0.0f;
	// }

	// // swing_time range [0, 2*PI]
	// if (swing_time > 2 * PI)
	// {
	// 	swing_time -= 2 * PI;
	// }

	// *angle_set = swing_angle;
}

void chassis_spinning_speed_manager(fp32* wz_set)
{
	// Randomly spin or manually change spin speed
	static fp32 spinning_speed = SPINNING_CHASSIS_LOW_OMEGA;
	if (chassis_move.fRandomSpinOn)
	{
		// Dial changes: range of possible speed and interval to change speed; positive dial value more rapid, negative less rapid
		static uint32_t ulLastUpdateTime = 0;
		static fp32 random_wz_max_speed = SPINNING_CHASSIS_MED_OMEGA;
		static fp32 random_wz_min_speed = SPINNING_CHASSIS_LOW_OMEGA;
		static uint8_t param_change_counter = 0;
		static uint32_t speed_change_period = MID_SPIN_SPEED_CHANGE_PERIOD;
		static uint8_t param_change_period = NORMAL_SPIN_PARAM_CHANGE_PERIOD;
		static fp32 spinning_sign = 1;
		fp32 dial_ratio = chassis_move.dial_channel_out / JOYSTICK_HALF_RANGE;

		// once per speed_change_period, update spinning speed to a random number in between random_wz_min_speed and random_wz_max_speed
		if (osKernelSysTick() - ulLastUpdateTime >= speed_change_period)
		{
			spinning_speed = spinning_sign * RNG_get_random_range_fp32(random_wz_min_speed, random_wz_max_speed);
			ulLastUpdateTime = osKernelSysTick();
			param_change_counter++;

			// change the speed changing period after certain number of changes
			if (param_change_counter >= param_change_period)
			{
				// calc random_wz_max_speed
				random_wz_min_speed = chassis_get_med_wz_limit();
				fp32 random_wz_param_a = ((chassis_move.wz_max_speed - random_wz_min_speed) / 2.0f);
				fp32 random_wz_param_b = ((chassis_move.wz_max_speed + random_wz_min_speed) / 2.0f);
				random_wz_max_speed = random_wz_param_a * (-dial_ratio) + random_wz_param_b;

				// change direction
				//spinning_sign = RNG_get_random_range_int32(0, 1) ? -1 : 1;

				// calc periods
				param_change_period = RNG_get_random_range_int32(MIN_SPIN_PARAM_CHANGE_PERIOD, roundf(NORMAL_SPIN_PARAM_CHANGE_PERIOD + dial_ratio * DELTA_SPIN_PARAM_CHANGE_PERIOD));
				fp32 param_change_period_max = MID_SPIN_SPEED_CHANGE_PERIOD + dial_ratio * DELTA_SPIN_SPEED_CHANGE_PERIOD;
				speed_change_period = RNG_get_random_range_fp32(MIN_SPIN_SPEED_CHANGE_PERIOD, param_change_period_max);

				param_change_counter = 0;
			}
		}
	}
	else
	{
		fp32 spin_rc_offset = chassis_get_low_wz_limit();
		// piecewise linear mapping
		if (chassis_move.dial_channel_out > 0)
		{
			fp32 spin_rc_sen_positive = ((chassis_move.wz_max_speed - spin_rc_offset) / JOYSTICK_HALF_RANGE);
			spinning_speed = chassis_move.dial_channel_out * spin_rc_sen_positive + spin_rc_offset;
		}
		else
		{
			fp32 spin_rc_sen_negative = ((chassis_move.wz_max_speed + spin_rc_offset) / JOYSTICK_HALF_RANGE);
			spinning_speed = chassis_move.dial_channel_out * spin_rc_sen_negative + spin_rc_offset;
		}
	}
	*wz_set = spinning_speed;
}

static void chassis_cv_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
	{
		return;
	}

	// manual test immediately before game start
	if (is_game_started() == 0)
	{
		chassis_rc_to_control_vector(vx_set, vy_set);
	}

#if CV_INTERFACE
	if (toe_is_error(CV_TOE))
	{
		*vx_set = 0;
		*vy_set = 0;
		*wz_set = 0;
	}
	else
	{
		// chassis_task should maintain previous speed if cv is offline for a short time
		*vx_set = CvCmdHandler.CvCmdMsg.xSpeed;
		*vy_set = CvCmdHandler.CvCmdMsg.ySpeed;

		// @TODO: implement CV enemy detection mode
		if (is_game_started() && CvCmder_GetMode(CV_MODE_CHASSIS_SPINNING_BIT))
		{
			chassis_spinning_speed_manager(wz_set);
		}
		else if (CvCmder_GetMode(CV_MODE_CHASSIS_ALIGN_TO_IMU_FRONT_BIT))
		{
			chassis_align_to_imu_front(wz_set);
		}
	}
#endif
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_BASIC_FPV_MODE, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @retval         none
 */
static void chassis_basic_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, uint8_t fSpinningOn)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL)
	{
		return;
	}

	chassis_rc_to_control_vector(vx_set, vy_set);
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
	swerve_platform_rc_mapping();
#elif (ROBOT_TYPE == INFANTRY_2024_BIPED)
	biped_platform_rc_mapping();
#endif

	if (fSpinningOn)
	{
		chassis_spinning_speed_manager(wz_set);
	}
	else
	{
		chassis_align_to_gimbal(wz_set);
	}
}

void chassis_align_to_gimbal(fp32* wz_set)
{
#if (ROBOT_TYPE != INFANTRY_2024_BIPED)
	if ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_R) && ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) == 0))
	{
		// Keep rotating until chassis align with gimbal
		const fp32 gimbal_align_angle_deadzone = DEG_TO_RAD(3.5f);
		if (fabs(gimbal_control.gimbal_yaw_motor.relative_angle) > gimbal_align_angle_deadzone)
		{
			*wz_set = (chassis_move.wz_max_speed - chassis_get_low_wz_limit()) * (fabs(gimbal_control.gimbal_yaw_motor.relative_angle) / (PI / 2.0f)) + chassis_get_low_wz_limit();
			if (gimbal_control.gimbal_yaw_motor.relative_angle < 0)
			{
				*wz_set *= -1;
			}
		}
		else
		{
			*wz_set = 0;
		}
	}
	else
	{
		*wz_set = (chassis_move.wz_max_speed / JOYSTICK_HALF_RANGE) * chassis_move.dial_channel_out;
	}
#endif
}

void chassis_align_to_imu_front(fp32 *wz_set)
{
	// Keep rotating until chassis align with IMU abs front
	const fp32 chassis_align_abs_angle_deadzone = DEG_TO_RAD(10.0f);
	if (fabs(chassis_move.chassis_yaw) > chassis_align_abs_angle_deadzone)
	{
		*wz_set = ((chassis_move.wz_max_speed - chassis_get_ultra_low_wz_limit()) * (fabs(chassis_move.chassis_yaw) / (PI / 2.0f)) + chassis_get_ultra_low_wz_limit()) * 0.4f;
		if (chassis_move.chassis_yaw > 0)
		{
			*wz_set *= -1;
		}
	}
	else
	{
		*wz_set = 0;
	}
}
