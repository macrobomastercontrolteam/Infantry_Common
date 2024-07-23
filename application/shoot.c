/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "bsp_fric.h"
#include "bsp_laser.h"
#include "referee.h"
#include "user_lib.h"

#include "CAN_receive.h"
#include "cv_usart_task.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "pid.h"

// microswitch
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)
#define AUTOAIM_READY_TIMEOUT 4000
#define TRIGGER_ANTI_STALL_BY_WAIT 1

/**
 * @brief          Set the mode of the shoot control state machine. Corresponding states for left lever of remote controller: up - fast shooting, middle - idle, down - disabled
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void);
/**
 * @brief          Update shoot data
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(void);
/**
 * @brief          Handle stall by oscillating the trigger motor
 * @param[in]      void
 * @retval         void
 */
static void trigger_motor_stall_handler(void);

bool_t isOverheated(void);

shoot_control_t shoot_control;

/**
 * @brief          Initialize the shoot control, including PID, remote control pointer, and motor pointer
 * @param[in]      void
 */
void shoot_init(void)
{

	shoot_control.shoot_mode = SHOOT_STOP;
	shoot_control.shoot_rc = get_remote_control_point();

	// initialize PID
	static const fp32 shoot_speed_pid1[3] = {FRICTION_1_SPEED_PID_KP, FRICTION_1_SPEED_PID_KI, FRICTION_1_SPEED_PID_KD};
	static const fp32 shoot_speed_pid2[3] = {FRICTION_2_SPEED_PID_KP, FRICTION_2_SPEED_PID_KI, FRICTION_2_SPEED_PID_KD};
	static const fp32 trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
	PID_init(&shoot_control.friction_motor1_pid, PID_POSITION, shoot_speed_pid1, FRICTION_1_SPEED_PID_MAX_OUT, FRICTION_1_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
	PID_init(&shoot_control.friction_motor2_pid, PID_POSITION, shoot_speed_pid2, FRICTION_2_SPEED_PID_MAX_OUT, FRICTION_2_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
	PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT, 0, &raw_err_handler);

	// update data
	shoot_feedback_update();

	shoot_control.friction_motor1_rpm_set = 0.0f;
	shoot_control.friction_motor2_rpm_set = 0.0f;
	shoot_control.ecd_count = 0;
	shoot_control.angle = motor_chassis[MOTOR_INDEX_TRIGGER].ecd * TRIGGER_MOTOR_ECD_TO_ANGLE;
	shoot_control.cmd_value = 0;
	shoot_control.set_angle = shoot_control.angle;
	shoot_control.speed = 0.0f;
	shoot_control.speed_set = 0.0f;
	// shoot_control.key_time = 0;

	shoot_control.press_l = 0;
	shoot_control.press_r = 0;
	shoot_control.last_press_l = 0;
	shoot_control.last_press_r = 0;
	shoot_control.left_click_hold_time = 0;

	shoot_control.block_time = 0;
	shoot_control.reverse_time = 0;

	shoot_control.heat_limit = 0;
	shoot_control.heat = 0;
	shoot_control.cv_auto_shoot_start_time = 0;

	memset(&shoot_control.launching_frequency, 0, sizeof(shoot_control.launching_frequency));
	memset(&shoot_control.bullet_init_speed, 0, sizeof(shoot_control.bullet_init_speed));
}

/**
 * @brief          Shoot mode state machine
 * @param[in]      void
 * @retval         void
 */
int16_t shoot_control_loop(void)
{
	shoot_set_mode();
	shoot_feedback_update();

	// detect case switch edge
	static shoot_mode_e pre_shoot_mode = SHOOT_STOP;
	if (pre_shoot_mode != shoot_control.shoot_mode)
	{
		// laser_enable(shoot_control.shoot_mode != SHOOT_STOP);
		switch (shoot_control.shoot_mode)
		{
			case SHOOT_STOP:
			{
#if USE_SERVO_TO_STIR_AMMO
				CAN_cmd_load_servo(0, 3);
#endif
				break;
			}
			case SHOOT_READY_FRIC:
			{
#if USE_SERVO_TO_STIR_AMMO
				CAN_cmd_load_servo(1, 3);
#endif
				shoot_control.trigger_speed_set = 0;

				PID_clear(&shoot_control.friction_motor1_pid);
				PID_clear(&shoot_control.friction_motor2_pid);

				shoot_control.friction_motor1_pid.max_out = FRICTION_1_SPEED_PID_MAX_OUT;
				shoot_control.friction_motor2_pid.max_out = FRICTION_2_SPEED_PID_MAX_OUT;

				shoot_control.friction_motor1_rpm_set = -FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
				shoot_control.friction_motor2_rpm_set = FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
				break;
			}
			case SHOOT_AUTO_FIRE:
			{
				break;
			}
			case SHOOT_SEMI_AUTO_FIRE:
			{
				shoot_control.set_angle = rad_format(shoot_control.angle + TRIGGER_ANGLE_INCREMENT);
				shoot_control.trigger_speed_set = SEMI_AUTO_FIRE_TRIGGER_SPEED;
				break;
			}
			default:
			{
				break;
			}
		}
		pre_shoot_mode = shoot_control.shoot_mode;
	}

	switch (shoot_control.shoot_mode)
	{
		case SHOOT_STOP:
		{
			shoot_control.friction_motor1_rpm_set = 0.0f;
			shoot_control.friction_motor2_rpm_set = 0.0f;
			shoot_control.trigger_speed_set = 0;
			if ((fabs(shoot_control.friction_motor1_rpm) < 60) && (fabs(shoot_control.friction_motor2_rpm) < 60))
			{
				shoot_control.friction_motor1_pid.max_out = 0;
				shoot_control.friction_motor2_pid.max_out = 0;
			}
			else
			{
				shoot_control.friction_motor1_pid.max_out = 1000;
				shoot_control.friction_motor2_pid.max_out = 1000;
			}
			break;
		}
		case SHOOT_READY_FRIC:
		{
			if ((fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_LEFT].speed_rpm / shoot_control.friction_motor1_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD) && (fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_RIGHT].speed_rpm / shoot_control.friction_motor2_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD))
			{
				if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT)) // Auto aim
				{
					if (CvCmder_GetMode(CV_MODE_SHOOT_BIT))
					{
						shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
					}
				}
				else // Manual control
				{
					if (shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL] == RC_SW_UP)
					{
						shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
					}
					else if (shoot_control.press_l)
					{
						if (shoot_control.left_click_hold_time >= RC_S_LONG_TIME)
						{
							shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
						}
						else
						{
							shoot_control.shoot_mode = SHOOT_SEMI_AUTO_FIRE;
						}
					}
				}
			}
			break;
		}
		// rotate trigger motor until a bullet is loaded and ready to fire, therefore, requires a microswitch to function
		case SHOOT_READY_TRIGGER:
		{
			if (shoot_control.key == SWITCH_TRIGGER_OFF)
			{
				shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
			}
			// else
			// {
			//     shoot_control.trigger_speed_set = 0.0f;
			//     shoot_control.shoot_mode = SHOOT_READY;
			// }
			break;
		}
		case SHOOT_READY:
		{
			if (shoot_control.key == SWITCH_TRIGGER_OFF)
			{
				shoot_control.shoot_mode = SHOOT_READY_TRIGGER;
			}
			break;
		}
		case SHOOT_SEMI_AUTO_FIRE:
		{
			if (shoot_control.press_r == 0)
			{
				shoot_control.shoot_mode = SHOOT_STOP;
			}
			else if (shoot_control.left_click_hold_time >= RC_S_LONG_TIME)
			{
				shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
			}
			else if ((rad_format(shoot_control.set_angle - shoot_control.angle) <= TRIGGER_MOTOR_ANGLE_THRESHOLD) || isOverheated())
			{
				shoot_control.trigger_speed_set = 0.0f;
				shoot_control.set_angle = shoot_control.angle;
				shoot_control.shoot_mode = SHOOT_READY_FRIC;
			}
			break;
		}
		case SHOOT_AUTO_FIRE:
		{
			if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT)) // auto aim
			{
				if (CvCmder_GetMode(CV_MODE_SHOOT_BIT) == 0)
				{
					shoot_control.shoot_mode = SHOOT_READY_FRIC;
				}
			}
			else
			{
				if (shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL] == RC_SW_UP)
				{
					; // stay in auto fire mode
				}
				else
				{
					if ((shoot_control.press_r == 0)) // && (shoot_control.key == SWITCH_TRIGGER_OFF)
					{
						shoot_control.shoot_mode = SHOOT_STOP;
					}
					else if ((shoot_control.press_l == 0))
					{
						// shoot_control.trigger_speed_set = 0;
						shoot_control.shoot_mode = SHOOT_READY_FRIC;
					}
				}
			}

			if (isOverheated())
			{
				shoot_control.trigger_speed_set = 0;
			}
			else
			{
				shoot_control.trigger_speed_set = AUTO_FIRE_TRIGGER_SPEED;
			}
			break;
		}
	}

	trigger_motor_stall_handler();

	PID_calc(&shoot_control.friction_motor1_pid, shoot_control.friction_motor1_rpm, shoot_control.friction_motor1_rpm_set, SHOOT_CONTROL_TIME_S);
	shoot_control.fric1_given_current = (int16_t)(shoot_control.friction_motor1_pid.out);

	PID_calc(&shoot_control.friction_motor2_pid, shoot_control.friction_motor2_rpm, shoot_control.friction_motor2_rpm_set, SHOOT_CONTROL_TIME_S);
	shoot_control.fric2_given_current = (int16_t)(shoot_control.friction_motor2_pid.out);

	PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set, SHOOT_CONTROL_TIME_S);
	shoot_control.cmd_value = (int16_t)(shoot_control.trigger_motor_pid.out);

	return shoot_control.cmd_value;
}

/**
 * @brief          Set the shoot mode state machine, swich down as safe, swich middle for mouse control, swich up for fast shooting
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{

	// normal RC control
	if (gimbal_cmd_to_shoot_stop() || toe_is_error(FRIC1_MOTOR_TOE) || toe_is_error(FRIC2_MOTOR_TOE) || toe_is_error(TRIGGER_MOTOR_TOE))
	{
		shoot_control.shoot_mode = SHOOT_STOP;
	}
	else if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT)) // auto aim mode
	{
		if (is_game_started() == 0)
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		else if (CvCmder_GetMode(CV_MODE_SHOOT_BIT))
		{
			if (shoot_control.shoot_mode != SHOOT_AUTO_FIRE)
			{
				shoot_control.shoot_mode = SHOOT_READY_FRIC; // prepare to shoot
			}
			// reset the start time during shooting
			shoot_control.cv_auto_shoot_start_time = osKernelSysTick();
		}
		else if ((osKernelSysTick() - shoot_control.cv_auto_shoot_start_time) > AUTOAIM_READY_TIMEOUT)
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
	}
	else
	{
		// remote controller S1 switch logic
		static int8_t last_s = RC_SW_UP;
		int8_t new_s = shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL];
		switch (new_s)
		{
			case RC_SW_UP:
			{
				if (last_s != RC_SW_UP)
				{
					shoot_control.shoot_mode = SHOOT_READY_FRIC;
				}
				break;
			}
			case RC_SW_MID:
			{
				if (shoot_control.press_r)
				{
					if (shoot_control.last_press_r == 0)
					{
						shoot_control.shoot_mode = SHOOT_READY_FRIC; // start rotatiing friction wheel
					}
				}
				else
				{
					shoot_control.shoot_mode = SHOOT_STOP;
				}
				break;
			}
			case RC_SW_DOWN:
			default:
			{
				shoot_control.shoot_mode = SHOOT_STOP;
				break;
			}
		}
		last_s = new_s;
	}
}

static void shoot_feedback_update(void)
{

	static fp32 speed_fliter_1 = 0.0f;
	static fp32 speed_fliter_2 = 0.0f;
	static fp32 speed_fliter_3 = 0.0f;

	// filter coefficient for trigger motor speed
	static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

	// second-order low-pass filter
	speed_fliter_1 = speed_fliter_2;
	speed_fliter_2 = speed_fliter_3;
	speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (motor_chassis[MOTOR_INDEX_TRIGGER].speed_rpm * TRIGGER_MOTOR_RPM_TO_SPEED) * fliter_num[2];
	shoot_control.speed = speed_fliter_3;

	shoot_control.friction_motor1_rpm = first_order_filter(motor_chassis[MOTOR_INDEX_FRICTION_LEFT].speed_rpm, shoot_control.friction_motor1_rpm, 0.8f);
	shoot_control.friction_motor2_rpm = first_order_filter(motor_chassis[MOTOR_INDEX_FRICTION_RIGHT].speed_rpm, shoot_control.friction_motor2_rpm, 0.8f);

	// reset the motor count, because when the output shaft rotates one turn, the motor shaft rotates 36 turns, process the motor shaft data into output shaft data, used to control the output shaft angle
	if (motor_chassis[MOTOR_INDEX_TRIGGER].ecd - motor_chassis[MOTOR_INDEX_TRIGGER].last_ecd > HALF_ECD_RANGE)
	{
		shoot_control.ecd_count--;
	}
	else if (motor_chassis[MOTOR_INDEX_TRIGGER].ecd - motor_chassis[MOTOR_INDEX_TRIGGER].last_ecd < -HALF_ECD_RANGE)
	{
		shoot_control.ecd_count++;
	}

	if (shoot_control.ecd_count == TRIGGER_MULTILOOP_FULL_COUNT)
	{
		shoot_control.ecd_count = -(TRIGGER_MULTILOOP_FULL_COUNT - 1);
	}
	else if (shoot_control.ecd_count == -TRIGGER_MULTILOOP_FULL_COUNT)
	{
		shoot_control.ecd_count = TRIGGER_MULTILOOP_FULL_COUNT - 1;
	}

	shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + motor_chassis[MOTOR_INDEX_TRIGGER].ecd) * TRIGGER_MOTOR_ECD_TO_ANGLE;
	shoot_control.key = BUTTEN_TRIG_PIN;
	shoot_control.last_press_l = shoot_control.press_l;
	shoot_control.last_press_r = shoot_control.press_r;
	shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
	shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;

	if ((shoot_control.shoot_mode == SHOOT_READY) || (shoot_control.press_l))
	{
		if (shoot_control.left_click_hold_time < RC_S_LONG_TIME)
		{
			shoot_control.left_click_hold_time += SHOOT_CONTROL_TIME_MS;
		}
	}
	else
	{
		shoot_control.left_click_hold_time = 0;
	}
	get_shoot_heat0_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
}

static void trigger_motor_stall_handler(void)
{
#if REVERSE_TRIGGER_DIRECTION
	shoot_control.speed_set = -shoot_control.trigger_speed_set;
#else
	shoot_control.speed_set = shoot_control.trigger_speed_set;
#endif

	if (fabs(shoot_control.trigger_speed_set) < BLOCK_TRIGGER_SPEED)
	{
		shoot_control.block_time = 0;
		shoot_control.reverse_time = 0;
		if (fabs(shoot_control.speed) < IDLE_TRIGGER_SPEED)
		{
			shoot_control.trigger_motor_pid.max_out = 0;
		}
		else
		{
			shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
		}
	}
	else
	{
		shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;

		if (shoot_control.block_time >= BLOCK_TIME)
		{
#if TRIGGER_ANTI_STALL_BY_WAIT
			shoot_control.speed_set = 0;
#else
			shoot_control.speed_set = -shoot_control.speed_set;
#endif
		}
		// jam detection
		if ((fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED) && (shoot_control.block_time < BLOCK_TIME))
		{
			shoot_control.block_time += SHOOT_CONTROL_TIME_MS;
			shoot_control.reverse_time = 0;
		}
		else if ((shoot_control.block_time >= BLOCK_TIME) && (shoot_control.reverse_time < REVERSE_TIME))
		{
			shoot_control.reverse_time += SHOOT_CONTROL_TIME_MS;
		}
		else
		{
			shoot_control.block_time = 0;
		}
	}
}

bool_t isOverheated(void)
{
	bool_t out = 0;
	if (toe_is_error(REFEREE_TOE) == 0)
	{
		if (SHOOT_HEAT_LIMIT_CLEARANCE > shoot_control.heat_limit)
		{
			if (shoot_control.heat + SHOOT_HEAT_LIMIT_CLEARANCE / 2 > shoot_control.heat_limit)
			{
				out = 1;
			}
		}
		else if (shoot_control.heat + SHOOT_HEAT_LIMIT_CLEARANCE > shoot_control.heat_limit)
		{
			out = 1;
		}
	}
	return out;
}
