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
@verbatim
==============================================================================
  add a chassis behaviour mode
  1. in chassis_behaviour.h , add a new behaviour name in chassis_behaviour
  erum
  {
      ...
      ...
      CHASSIS_XXX_XXX, // new add
  }chassis_behaviour_e,
  2. implement new function. chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
      "vx, vy, wz" param is chassis movement control input.
      first param: 'vx' usually means  vertical speed,
          positive value means forward speed, negative value means backward speed.
      second param: 'vy' usually means horizotal speed,
          positive value means letf speed, negative value means right speed
      third param: 'wz' can be rotation speed set or angle set,

      in this new function, you can assign speed to "vx","vy",and "wz",as your wish
  3.  in "chassis_behaviour_mode_set" function, add new logical judgement to assign CHASSIS_XXX_XXX to  "chassis_behaviour_mode" variable,
      and in the last of the function, add "else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)"
      choose a chassis control mode.
      four mode:
      CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control relative angle
          between chassis and gimbal. you can name third param to 'xxx_angle_set' other than 'wz'
      CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control absolute angle calculated by gyro
          you can name third param to 'xxx_angle_set.
      CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy' are speed control, 'wz' is rotation speed control.
      CHASSIS_VECTOR_RAW : will use 'vx' 'vy' and 'wz'  to linearly calculate four wheel current set,
          current set will be derectly sent to can bus.
  4. in the last of "chassis_behaviour_control_set" function, add
      else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
      {
          chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
      }
==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2019 DJI****************************
*/

#include "chassis_behaviour.h"
#include "AHRS_middleware.h"
#include "bsp_rng.h" // for random number generator
#include "chassis_task.h"
#include "cmsis_os.h"
#include "user_lib.h"

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

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis control mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
 *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

static void chassis_spinning_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

static void chassis_cv_spinning_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
 *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle[-PI, PI]
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
static void swerve_chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void swerve_chassis_spinning_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
#endif

/**
 * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
 *                 set value will be sent to can bus.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

fp32 chassis_spinning_speed_manager(void);

// Watchout for the default value of chassis behaviour mode
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

/**
 * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
 * @param[in]      chassis_move_mode: chassis data
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}

	if ((chassis_behaviour_mode == CHASSIS_CV_CONTROL_SPINNING) && toe_is_error(DBUS_TOE))
	{
		; // CV fully automatic mode without RC: do not switch out of cv state
	}
	else if (gimbal_emergency_stop())
	{
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
	}
	// when gimbal is in some mode, such as init mode, chassis must not move
	else if (gimbal_cmd_to_chassis_stop())
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	}
	else
	{
		switch (chassis_move_mode->chassis_RC->rc.s[RC_RIGHT_LEVER_CHANNEL])
		{
			case RC_SW_UP:
			{
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
				chassis_behaviour_mode = CHASSIS_CV_CONTROL_SPINNING;
#else
				chassis_behaviour_mode = CHASSIS_SPINNING;
#endif
				break;
			}
			case RC_SW_MID:
			{
				// can change to CHASSIS_ZERO_FORCE,CHASSIS_NO_MOVE,CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
				// CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,CHASSIS_NO_FOLLOW_YAW,CHASSIS_OPEN
				// Remember to change gimbal_behaviour logic correspondingly
				chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
				break;
			}
			case RC_SW_DOWN:
			default:
			{
				chassis_behaviour_mode = CHASSIS_NO_MOVE;
				break;
			}
		}
	}

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	if (chassis_behaviour_mode == CHASSIS_CV_CONTROL_SPINNING)
	{
		CvCmder_ChangeMode(CV_MODE_AUTO_AIM_BIT | CV_MODE_AUTO_MOVE_BIT, 1);
#if SENTRY_UPPER_HEAD_TEST
		chassis_move.fUpperHeadEnabled = 1;
#else
		chassis_move.fUpperHeadEnabled = is_game_started();
#endif
	}
	else
	{
		CvCmder_ChangeMode(CV_MODE_AUTO_AIM_BIT | CV_MODE_AUTO_MOVE_BIT, 0);
		chassis_move.fUpperHeadEnabled = 0;
	}
#endif

	switch (chassis_behaviour_mode)
	{
		case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
		{
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
			break;
		}
		case CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW:
		{
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
			break;
		}
		case CHASSIS_NO_MOVE:
		case CHASSIS_NO_FOLLOW_YAW:
		{
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
			break;
		}
		case SWERVE_CHASSIS_NO_FOLLOW_YAW:
		{
			chassis_move_mode->chassis_mode = SWERVE_CHASSIS_VECTOR_NO_FOLLOW_YAW;
			break;
		}
		case CHASSIS_SPINNING:
		case CHASSIS_CV_CONTROL_SPINNING:
		{
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_SPINNING;
			break;
		}
		case SWERVE_CHASSIS_SPINNING:
		{
			chassis_move_mode->chassis_mode = SWERVE_CHASSIS_VECTOR_SPINNING;
			break;
		}
		case CHASSIS_ZERO_FORCE:
		case CHASSIS_OPEN:
		default:
		{
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
			break;
		}
	}
}

/**
 * @brief          set control set-point. three movement param, according to difference control mode,
 *                 will control corresponding movement.in the function, usually call different control function.
 * @param[out]     vx_set, usually controls vertical speed.
 * @param[out]     vy_set, usually controls horizotal speed.
 * @param[out]     wz_set, usually controls rotation speed.
 * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
 * @retval         none
 */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	switch (chassis_behaviour_mode)
	{
		case CHASSIS_NO_MOVE:
		{
			chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW:
		{
			chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW:
		{
			chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case CHASSIS_NO_FOLLOW_YAW:
		{
			chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
		case SWERVE_CHASSIS_NO_FOLLOW_YAW:
		{
			swerve_chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case SWERVE_CHASSIS_SPINNING:
		{
			swerve_chassis_spinning_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
#endif
		case CHASSIS_OPEN:
		{
			chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case CHASSIS_SPINNING:
		{
			chassis_spinning_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case CHASSIS_CV_CONTROL_SPINNING:
		{
			chassis_cv_spinning_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
		case CHASSIS_ZERO_FORCE:
		default:
		{
			chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			break;
		}
	}
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
 *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
 *                 will be sent to CAN bus derectly, and the function will set all speed zero.
 * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
 * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
 * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	*vx_can_set = 0.0f;
	*vy_can_set = 0.0f;
	*wz_can_set = 0.0f;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
 *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	*vx_set = 0.0f;
	*vy_set = 0.0f;
	*wz_set = 0.0f;
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
 *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle difference between chassis and gimbal
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	// Convert joystick and keyboard input to commands
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	// // swing angle is generated by sin function, swing_time is the input to sine function
	// static fp32 swing_time = 0.0f;
	// static fp32 swing_angle = 0.0f;
	// // max_angle is the max angle that chassis will rotate
	// static fp32 max_angle = SWING_NO_MOVE_ANGLE;
	// // swing_time increment by add_time per control cycle
	// static fp32 const add_time = PI * 0.5f / CHASSIS_CONTROL_TIME_S;

	// static uint8_t swing_flag = 0;

	// // judge if are to swing
	// if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
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
	// if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
	//     chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
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

fp32 chassis_spinning_speed_manager(void)
{
	int16_t dial_channel = 0;
	static fp32 spinning_speed = SPINNING_CHASSIS_LOW_OMEGA;
	if (toe_is_error(DBUS_TOE) == 0)
	{
		deadband_limit(chassis_move.chassis_RC->rc.ch[RC_DIAL_CHANNEL], dial_channel, CHASSIS_RC_DEADLINE);
	}

	// Randomly spin or manually change spin speed
	if (chassis_move.fRandomSpinOn)
	{
		// Dial changes: range of possible speed and interval to change speed; positive dial value more rapid, negative less rapid
		static uint32_t ulLastUpdateTime = 0;
		static uint8_t param_change_counter = 0;
		static uint32_t speed_change_period = MID_SPIN_SPEED_CHANGE_PERIOD;
		static uint8_t param_change_period = NORMAL_SPIN_PARAM_CHANGE_PERIOD;
		static fp32 spinning_sign = 1;
		fp32 dial_ratio = dial_channel / JOYSTICK_HALF_RANGE;

		// once per speed_change_period, update spinning speed to a random number in between wz_min_speed and wz_max_speed
		if (osKernelSysTick() - ulLastUpdateTime >= speed_change_period)
		{
			spinning_speed = spinning_sign * RNG_get_random_range_fp32(chassis_move.wz_min_speed, chassis_move.wz_max_speed);
			ulLastUpdateTime = osKernelSysTick();
			param_change_counter++;

			// change the speed changing period after certain number of changes
			if (param_change_counter >= param_change_period)
			{
				param_change_period = RNG_get_random_range_int32(MIN_SPIN_PARAM_CHANGE_PERIOD, roundf(NORMAL_SPIN_PARAM_CHANGE_PERIOD + dial_ratio * DELTA_SPIN_PARAM_CHANGE_PERIOD));
				chassis_move.wz_max_speed = SPINNING_CHASSIS_HIGH_OMEGA * (-dial_ratio / 2 + 1);
				fp32 param_change_period_max = MID_SPIN_SPEED_CHANGE_PERIOD + dial_ratio * DELTA_SPIN_SPEED_CHANGE_PERIOD;
				speed_change_period = RNG_get_random_range_fp32(MIN_SPIN_SPEED_CHANGE_PERIOD, param_change_period_max);
				spinning_sign = RNG_get_random_range_int32(0, 1) ? -1 : 1;
				param_change_counter = 0;
			}
		}
	}
	else
	{
		// piecewise linear mapping
		if (dial_channel > 0)
		{
			spinning_speed = dial_channel * CHASSIS_SPIN_RC_SEN_POSITIVE_INPUT + CHASSIS_SPIN_RC_OFFSET;
		}
		else
		{
			spinning_speed = dial_channel * CHASSIS_SPIN_RC_SEN_NEGATIVE_INPUT + CHASSIS_SPIN_RC_OFFSET;
		}
	}
	return spinning_speed;
}

static void chassis_spinning_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	// Convert joystick and keyboard input to commands
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	*angle_set = chassis_spinning_speed_manager();
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
static void swerve_chassis_spinning_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	// Convert joystick and keyboard input to commands
	fp32 dummy_wz_set;
	swerve_chassis_rc_to_control_vector(vx_set, vy_set, &dummy_wz_set, chassis_move_rc_to_vector, 0);
	*angle_set = chassis_spinning_speed_manager();
}
#endif

static void chassis_cv_spinning_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL)
	{
		return;
	}

#if CV_INTERFACE
	if (toe_is_error(CV_TOE))
	{
		*vx_set = 0;
		*vy_set = 0;
	}
	else
	{
		// chassis_task should maintain previous speed if cv is offline for a short time
		*vx_set = CvCmdHandler.CvCmdMsg.xSpeed;
		*vy_set = CvCmdHandler.CvCmdMsg.ySpeed;

		// @TODO: add enemy detection (controlled by CV)
		// if (CvCmder_GetMode(CV_MODE_ENEMY_DETECTED_BIT))
		// {
		// 	spinning_speed = SPINNING_CHASSIS_HIGH_OMEGA;
		// }
		// else
		// {
		// 	spinning_speed = SPINNING_CHASSIS_MED_OMEGA;
		// }
		*angle_set = chassis_spinning_speed_manager();
	}
#endif
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
 *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     angle_set: control angle[-PI, PI]
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	*angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN_INC * chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL]);
}

/**
 * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
 *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

	int16_t dial_channel;
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[RC_DIAL_CHANNEL], dial_channel, CHASSIS_RC_DEADLINE);
	*wz_set = CHASSIS_WZ_RC_SEN * dial_channel;
}

#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
static void swerve_chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	swerve_chassis_rc_to_control_vector(vx_set, vy_set, wz_set, chassis_move_rc_to_vector, 1);
}
#endif

/**
 * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
 *                 set value will be sent to can bus.
 * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
 * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
 * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
 * @param[in]      chassis_move_rc_to_vector: chassis data
 * @retval         none
 */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	*vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_RIGHT_VERTICAL_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
	*vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_RIGHT_HORIZONTAL_CHANNEL] * CHASSIS_OPEN_RC_SCALE;

	int16_t dial_channel;
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[RC_DIAL_CHANNEL], dial_channel, CHASSIS_RC_DEADLINE);
	*wz_set = dial_channel * CHASSIS_OPEN_RC_SCALE;
	return;
}
