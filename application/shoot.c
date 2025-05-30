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

#if (ROBOT_TYPE == HERO_2025_MECANUM)
#define TRIGGER_ANTI_STALL_BY_WAIT 1
#else
#define TRIGGER_ANTI_STALL_BY_WAIT 0
#endif
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
static void piston_motor_control(void);

bool_t isOverheated(void);

shoot_control_t shoot_control;

launcher_status_t launcher_status;

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
#if (ROBOT_TYPE == HERO_2025_MECANUM)
	static const fp32 shoot_speed_pid3[3] = {FRICTION_3_SPEED_PID_KP, FRICTION_3_SPEED_PID_KI, FRICTION_3_SPEED_PID_KD};
	static const fp32 shoot_speed_pid4[3] = {FRICTION_4_SPEED_PID_KP, FRICTION_4_SPEED_PID_KI, FRICTION_4_SPEED_PID_KD};
#endif
	static const fp32 trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
	PID_init(&shoot_control.friction_motor1_pid, PID_POSITION, shoot_speed_pid1, FRICTION_1_SPEED_PID_MAX_OUT, FRICTION_1_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
	PID_init(&shoot_control.friction_motor2_pid, PID_POSITION, shoot_speed_pid2, FRICTION_2_SPEED_PID_MAX_OUT, FRICTION_2_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
	PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT, 0, &raw_err_handler);
#if (ROBOT_TYPE == HERO_2025_MECANUM)
	PID_init(&shoot_control.friction_motor3_pid, PID_POSITION, shoot_speed_pid3, FRICTION_3_SPEED_PID_MAX_OUT, FRICTION_3_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
	PID_init(&shoot_control.friction_motor4_pid, PID_POSITION, shoot_speed_pid4, FRICTION_4_SPEED_PID_MAX_OUT, FRICTION_4_SPEED_PID_MAX_IOUT, 0, &raw_err_handler);
#endif
	// update data
	shoot_feedback_update();

	shoot_control.friction_motor1_rpm_set = 0.0f;
	shoot_control.friction_motor2_rpm_set = 0.0f;
#if (ROBOT_TYPE == HERO_2025_MECANUM)
	shoot_control.friction_motor3_rpm_set = 0.0f;
	shoot_control.friction_motor4_rpm_set = 0.0f;
#endif
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

	launcher_status.Chain_Loaded = 0;
	launcher_status.Launcher_Loaded = 0;
	launcher_status.Launcher_Opened = 0;
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
	// This block executes when the shooting mode changes.
	// It performs initial setup for the new mode.
	static shoot_mode_e pre_shoot_mode = SHOOT_STOP;
	if (pre_shoot_mode != shoot_control.shoot_mode)
	{
		// laser_enable(shoot_control.shoot_mode != SHOOT_STOP);
		switch (shoot_control.shoot_mode)
		{
			case SHOOT_STOP:
			{
			// Actions for when shooting stops.
#if USE_SERVO_TO_STIR_AMMO
				CAN_cmd_load_servo(0, 3); // Stop ammo stirring servo if used.
#endif
				break;
			}
			case SHOOT_READY_FRIC:
			{
				// Actions for preparing friction wheels.
#if USE_SERVO_TO_STIR_AMMO
				CAN_cmd_load_servo(1, 3); // Start ammo stirring servo if used.
#endif
				shoot_control.trigger_speed_set = 0; // Ensure trigger motor is stopped.

                // Clear PID controllers for friction motors.
				PID_clear(&shoot_control.friction_motor1_pid);
				PID_clear(&shoot_control.friction_motor2_pid);

                // Set max output for friction motor PIDs.
				shoot_control.friction_motor1_pid.max_out = FRICTION_1_SPEED_PID_MAX_OUT;
				shoot_control.friction_motor2_pid.max_out = FRICTION_2_SPEED_PID_MAX_OUT;

                // Set target RPM for friction motors.
				shoot_control.friction_motor1_rpm_set = -FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
				shoot_control.friction_motor2_rpm_set = FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;

#if (ROBOT_TYPE == HERO_2025_MECANUM)
                // Additional friction motors for HERO_2025_MECANUM.
				PID_clear(&shoot_control.friction_motor3_pid);
				PID_clear(&shoot_control.friction_motor4_pid);

				shoot_control.friction_motor3_pid.max_out = FRICTION_3_SPEED_PID_MAX_OUT;
				shoot_control.friction_motor4_pid.max_out = FRICTION_4_SPEED_PID_MAX_OUT;

				shoot_control.friction_motor3_rpm_set = -FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
				shoot_control.friction_motor4_rpm_set = FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
#endif

				break;
			}
			case SHOOT_AUTO_FIRE:
			{
                // No specific setup needed when entering auto fire directly,
                // as friction wheels should already be ready.
				break;
			}
			case SHOOT_SEMI_AUTO_FIRE:
			{
                // Setup for semi-automatic fire.
                // Set target angle for trigger motor to advance one bullet.
				shoot_control.set_angle = rad_format(shoot_control.angle + TRIGGER_ANGLE_INCREMENT);
                // Set trigger motor speed for semi-auto.
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

    // Main state machine logic based on the current shoot_mode.
	switch (shoot_control.shoot_mode)
	{
		case SHOOT_STOP:
		{
            // In SHOOT_STOP mode:
            // Set all friction motor RPM targets to 0.
			shoot_control.friction_motor1_rpm_set = 0.0f;
			shoot_control.friction_motor2_rpm_set = 0.0f;
			shoot_control.trigger_speed_set = 0; // Stop trigger motor.
#if (ROBOT_TYPE == HERO_2025_MECANUM)
			shoot_control.friction_motor3_rpm_set = 0.0f;
			shoot_control.friction_motor4_rpm_set = 0.0f;
#endif
            // If friction motors are almost stopped, set PID max_out to 0 to prevent oscillation.
            // Otherwise, set a small max_out to allow them to brake.
#if (ROBOT_TYPE == HERO_2025_MECANUM)
			if ((fabs(shoot_control.friction_motor1_rpm) < 60) && (fabs(shoot_control.friction_motor2_rpm) < 60) && (fabs(shoot_control.friction_motor3_rpm) < 60) && (fabs(shoot_control.friction_motor4_rpm) < 60))
			{
				shoot_control.friction_motor1_pid.max_out = 0;
				shoot_control.friction_motor2_pid.max_out = 0;
				shoot_control.friction_motor3_pid.max_out = 0;
				shoot_control.friction_motor4_pid.max_out = 0;
			}
			else
			{
				shoot_control.friction_motor1_pid.max_out = 1000;
				shoot_control.friction_motor2_pid.max_out = 1000;
				shoot_control.friction_motor3_pid.max_out = 1000;
				shoot_control.friction_motor4_pid.max_out = 1000;
			}
#else
            // Standard robot types have two friction motors.
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
#endif
			break;
		}
		case SHOOT_READY_FRIC:
		{
            // In SHOOT_READY_FRIC mode:
            // Wait for friction wheels to reach target speed.
#if (ROBOT_TYPE == HERO_2025_MECANUM)
            // HERO_2025_MECANUM has four friction motors.
			if ((fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_LEFT].speed_rpm / shoot_control.friction_motor1_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD) && (fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_RIGHT].speed_rpm / shoot_control.friction_motor2_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD) && (fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_UP].speed_rpm / shoot_control.friction_motor3_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD) && (fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_DOWN].speed_rpm / shoot_control.friction_motor4_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD))
#else
            // Standard robot types have two friction motors.
			if ((fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_LEFT].speed_rpm / shoot_control.friction_motor1_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD) && (fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_RIGHT].speed_rpm / shoot_control.friction_motor2_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD))
#endif
			{	
                // Friction wheels are ready. Check for shooting command.
				if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT)) // Auto aim mode
				{
					if (CvCmder_GetMode(CV_MODE_SHOOT_BIT)) // CV requests shooting
					{
						// If CV is requesting shoot, transition to auto fire mode.
#if (ROBOT_TYPE == HERO_2025_MECANUM)
						shoot_control.shoot_mode = HERO_LAUNCHER_SHOOT; // HERO_2025_MECANUM has a specific launcher shoot mode.
#else
                        shoot_control.shoot_mode = SHOOT_AUTO_FIRE; // Standard robots go to auto fire.
#endif
					}
				}
				else // Manual control
				{

#if !(ROBOT_TYPE == HERO_2025_MECANUM) //NOT FOR HERO_2025_MECANUM
					// Check for left lever position on remote controller.
					if (shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL] == RC_SW_UP)
					{
						shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
					}
					else if (shoot_control.press_l)
					{
                        // Standard robots: differentiate between short and long press for semi-auto/auto.
						if (shoot_control.left_click_hold_time >= RC_S_LONG_TIME)
						{
							shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
						}
						else
						{
							shoot_control.shoot_mode = SHOOT_SEMI_AUTO_FIRE;
						}
					}
#else
                    // Check for left mouse button press.
					if (shoot_control.press_l)
					{
						shoot_control.shoot_mode = HERO_LAUNCHER_SHOOT; // HERO_2025_MECANUM specific launcher mode.
					}
#endif
				}
			}
			break;
		}
        // rotate trigger motor until a bullet is loaded and ready to fire, therefore, requires a microswitch to function
		case SHOOT_READY_TRIGGER: // This mode is primarily for HERO_2025_MECANUM or robots with similar loading mechanisms.
		{
            // In SHOOT_READY_TRIGGER mode:
            // If the ammo chain is not loaded, run the trigger motor to load it.
			if (launcher_status.Chain_Loaded == 0)
			{
				shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
			}
            // Once Chain_Loaded becomes 1 (presumably by a sensor/microswitch),
            // the trigger motor will stop due to trigger_motor_stall_handler logic or mode change.
			break;
		}
		case HERO_LAUNCHER_SHOOT: // Specific shooting mode for HERO_2025_MECANUM.
		{
            // Logic for HERO_LAUNCHER_SHOOT mode.

			break;
		}
		case SHOOT_SEMI_AUTO_FIRE: // Standard robot semi-automatic fire.
		{
            // In SHOOT_SEMI_AUTO_FIRE mode:
			if (shoot_control.press_r == 0) // Right mouse button released (typically used to stop/cancel)
			{
				shoot_control.shoot_mode = SHOOT_STOP;
			}
			else if (shoot_control.left_click_hold_time >= RC_S_LONG_TIME) // Left click held long enough
			{
				shoot_control.shoot_mode = SHOOT_AUTO_FIRE; // Switch to auto fire.
			}
            // If the trigger motor has reached the target angle for one shot, or if overheated:
			else if ((rad_format(shoot_control.set_angle - shoot_control.angle) <= TRIGGER_MOTOR_ANGLE_THRESHOLD) || isOverheated())
			{
				shoot_control.trigger_speed_set = 0.0f; // Stop trigger motor.
				shoot_control.set_angle = shoot_control.angle; // Update set_angle to current angle.
				shoot_control.shoot_mode = SHOOT_READY_FRIC; // Return to ready state.
			}
            // Otherwise, trigger motor continues to run at SEMI_AUTO_FIRE_TRIGGER_SPEED (set when entering this state).
			break;
		}
		case SHOOT_AUTO_FIRE: // Standard robot automatic fire.
		{
            // In SHOOT_AUTO_FIRE mode:
			if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT)) // Auto aim mode
			{
				if (CvCmder_GetMode(CV_MODE_SHOOT_BIT) == 0) // CV stops requesting shoot
				{
					shoot_control.shoot_mode = SHOOT_READY_FRIC; // Return to ready state.
				}
                // If CV_MODE_SHOOT_BIT is 1, stay in auto fire.
			}
			else // Manual control mode
			{
				if (shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL] == RC_SW_UP) // Remote controller left lever is UP (force auto fire)
				{
					// stay in auto fire mode
				}
				else // Remote controller left lever is not UP
				{
					if ((shoot_control.press_r == 0)) // Right mouse button released (stop/cancel)
					{
						shoot_control.shoot_mode = SHOOT_STOP;
					}
					else if ((shoot_control.press_l == 0)) // Left mouse button released
					{
						// shoot_control.trigger_speed_set = 0; // Stop trigger motor.
						shoot_control.shoot_mode = SHOOT_READY_FRIC; // Return to ready state.
					}
					// If left mouse button is still pressed, stay in auto fire.
				}
			}

            // Handle overheating for auto fire.
			if (isOverheated())
			{
				shoot_control.trigger_speed_set = 0; // Stop trigger motor if overheated.
			}
			else
			{
				shoot_control.trigger_speed_set = AUTO_FIRE_TRIGGER_SPEED; // Set trigger motor speed for auto fire.
			}
			break;
		}
	}

    // Handle trigger motor stall and set its speed based on trigger_speed_set.
	trigger_motor_stall_handler();

    // Calculate PID outputs for friction motors.
	PID_calc(&shoot_control.friction_motor1_pid, shoot_control.friction_motor1_rpm, shoot_control.friction_motor1_rpm_set, SHOOT_CONTROL_TIME_S);
	shoot_control.fric1_given_current = (int16_t)(shoot_control.friction_motor1_pid.out);

	PID_calc(&shoot_control.friction_motor2_pid, shoot_control.friction_motor2_rpm, shoot_control.friction_motor2_rpm_set, SHOOT_CONTROL_TIME_S);
	shoot_control.fric2_given_current = (int16_t)(shoot_control.friction_motor2_pid.out);

    // Calculate PID output for trigger motor.
    // Note: PID is calculated based on 'shoot_control.speed' (filtered actual speed) and 'shoot_control.speed_set' (target speed from stall handler).
	PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set, SHOOT_CONTROL_TIME_S);
	shoot_control.cmd_value = (int16_t)(shoot_control.trigger_motor_pid.out); // This is the command value for the trigger motor.

#if (ROBOT_TYPE == HERO_2025_MECANUM)
    // Calculate PID outputs for additional friction motors on HERO_2025_MECANUM.
	PID_calc(&shoot_control.friction_motor3_pid, shoot_control.friction_motor3_rpm, shoot_control.friction_motor3_rpm_set, SHOOT_CONTROL_TIME_S);
	shoot_control.fric3_given_current = (int16_t)(shoot_control.friction_motor3_pid.out);


	PID_calc(&shoot_control.friction_motor4_pid, shoot_control.friction_motor4_rpm, shoot_control.friction_motor4_rpm_set, SHOOT_CONTROL_TIME_S);
	shoot_control.fric4_given_current = (int16_t)(shoot_control.friction_motor4_pid.out);
#endif

	return shoot_control.cmd_value; // Returns the command value for the trigger motor.
}

/**
 * @brief          Set the shoot mode state machine, swich down as safe, swich middle for mouse control, swich up for fast shooting
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_mode(void)
{

	// normal RC control
#if (ROBOT_TYPE == HERO_2025_MECANUM) 
	// HERO_2025_MECANUM: Check for gimbal command to stop or errors in any of the 4 friction motors or trigger motor.
	if (gimbal_cmd_to_shoot_stop() || toe_is_error(FRICTIONAL_MOTOR_LEFT_TOE) || toe_is_error(FRICTIONAL_MOTOR_RIGHT_TOE) || toe_is_error(FRICTIONAL_MOTOR_UP_TOE) || toe_is_error(FRICTIONAL_MOTOR_DOWN_TOE) || toe_is_error(TRIGGER_MOTOR_TOE))	
#else
	// Standard Robot: Check for gimbal command to stop or errors in the 2 friction motors or trigger motor.
	if (gimbal_cmd_to_shoot_stop() || toe_is_error(FRICTIONAL_MOTOR_LEFT_TOE) || toe_is_error(FRICTIONAL_MOTOR_RIGHT_TOE) || toe_is_error(TRIGGER_MOTOR_TOE))
#endif	
	{
		shoot_control.shoot_mode = SHOOT_STOP; // Set mode to STOP if any critical error or stop command.
	}
	// If referee system is online and shooter power output is 0 (e.g., due to power limits).
	else if ((toe_is_error(REFEREE_TOE) == 0) && (robot_state.power_management_shooter_output == 0))
	{
		shoot_control.shoot_mode = SHOOT_STOP; // Stop shooting if power management disables shooter.
	}
	else if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT)) // Auto aim mode is active.
	{
		if (is_game_started() == 0) // If game has not started.
		{
			shoot_control.shoot_mode = SHOOT_STOP; // Stop shooting before game starts.
		}
		else if (CvCmder_GetMode(CV_MODE_SHOOT_BIT)) // CV requests shooting.
		{
            // If not already in auto fire, transition to ready friction wheels.
            // This ensures friction wheels spin up before firing.
			if (shoot_control.shoot_mode != SHOOT_AUTO_FIRE)
			{
				shoot_control.shoot_mode = SHOOT_READY_FRIC; // prepare to shoot
			}
			// reset the auto shoot start time while CV is requesting shoot.
			// This is used for a timeout if CV stops requesting shoot.
			shoot_control.cv_auto_shoot_start_time = osKernelSysTick();
		}
		// If CV is not requesting shoot and it has been some time since it last requested.
		else if ((osKernelSysTick() - shoot_control.cv_auto_shoot_start_time) > AUTOAIM_READY_TIMEOUT)
		{
			shoot_control.shoot_mode = SHOOT_STOP; // Timeout: stop shooting if CV doesn't request for too long.
		}
		// If CV_MODE_SHOOT_BIT is 0 but timeout hasn't occurred, maintain current mode (likely SHOOT_READY_FRIC or SHOOT_AUTO_FIRE from previous cycle).
	}
	else // Manual control mode (not auto aim).
	{
		// Logic based on remote controller's left lever (S1 switch).
		static int8_t last_s = RC_SW_UP; // Stores the previous state of the S1 switch.
		int8_t new_s = shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL]; // Current state of S1 switch.
		switch (new_s)
		{
			case RC_SW_UP: // Lever is UP (typically continuous/auto fire mode by RC).
			{
				// If the switch was just moved to UP.
				if (last_s != RC_SW_UP)
				{
					shoot_control.shoot_mode = SHOOT_READY_FRIC; // Prepare friction wheels.
                                                                 // Actual auto fire will be triggered by mouse press in SHOOT_READY_FRIC or directly in SHOOT_AUTO_FIRE.
				}
				// If already in RC_SW_UP, the mode (e.g. SHOOT_AUTO_FIRE if mouse is pressed) is handled by the main switch in shoot_control_loop.
				break;
			}
			case RC_SW_MID: // Lever is MIDDLE (typically mouse-controlled fire mode).
			{
#if (ROBOT_TYPE == HERO_2025_MECANUM)
                // For HERO_2025_MECANUM, if ammo chain is not loaded, enter SHOOT_READY_TRIGGER to load it.
				if (launcher_status.Chain_Loaded == 0)
				{
					shoot_control.shoot_mode = SHOOT_READY_TRIGGER;
				}
				else // Chain is loaded, proceed to check mouse input.
#endif
				{
                    // Check right mouse button press (often used for aiming or enabling shooting).
                    // Note: The code uses shoot_control.press_r, which is typically right mouse.
                    // However, shooting is usually triggered by press_l (left mouse). This might be specific logic
                    // where right mouse press enables the friction wheels.
					if (shoot_control.press_r) // If right mouse is pressed.
					{
                        // If right mouse was just pressed.
						if (shoot_control.last_press_r == 0)
						{
							shoot_control.shoot_mode = SHOOT_READY_FRIC; // Start spinning friction wheels.
                                                                         // Actual shooting (semi/auto) will be triggered by left mouse in SHOOT_READY_FRIC.
						}
                        // If right mouse is held, maintain current mode (likely SHOOT_READY_FRIC or a shooting state if left mouse is also active).
					}
					else // Right mouse is not pressed.
					{
						shoot_control.shoot_mode = SHOOT_STOP; // Stop shooting if right mouse is released.
					}
				}

				break;
			}
			case RC_SW_DOWN: // Lever is DOWN (typically safe/disabled mode).
			default: // Default case, also treated as safe mode.
			{
				shoot_control.shoot_mode = SHOOT_STOP; // Stop all shooting activity.
				break;
			}
		}
		last_s = new_s; // Update last switch state.
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
#if (ROBOT_TYPE == HERO_2025_MECANUM) 
	shoot_control.friction_motor3_rpm = first_order_filter(motor_chassis[MOTOR_INDEX_FRICTION_UP].speed_rpm, shoot_control.friction_motor3_rpm, 0.8f);
	shoot_control.friction_motor4_rpm = first_order_filter(motor_chassis[MOTOR_INDEX_FRICTION_DOWN].speed_rpm, shoot_control.friction_motor4_rpm, 0.8f);
#endif


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
#if CAN_PASS_REF_INFO
	
	CAN_get_heat_limit_and_barrel_1_heat(&shoot_control.heat_limit, &shoot_control.heat);
#else
	get_shoot_heat0_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
#endif
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

		if (shoot_control.block_time >= TRIGGER_BLOCK_TIME)
		{
#if TRIGGER_ANTI_STALL_BY_WAIT
			shoot_control.speed_set = 0;
			launcher_status.Chain_Loaded = 1;
#else
			shoot_control.speed_set = -shoot_control.speed_set;
#endif
		}
		// jam detection
		if ((fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED) && (shoot_control.block_time < TRIGGER_BLOCK_TIME))
		{
			shoot_control.block_time += SHOOT_CONTROL_TIME_MS;
			shoot_control.reverse_time = 0;
		}
		else if ((shoot_control.block_time >= TRIGGER_BLOCK_TIME) && (shoot_control.reverse_time < TRIGGER_REVERSE_TIME))
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

static void piston_motor_control(void)
{
	uint8_t isPistonBlocked = 0;
	do
	{
#if REVERSE_PISTON_DIRECTION
		shoot_control.piston_speed_set = (-shoot_control.piston_speed_target) * shoot_control.piston_direction;
#else
		shoot_control.piston_speed_set = shoot_control.piston_speed_target * shoot_control.piston_direction;
#endif
		// Jam detection
		if (fabs(shoot_control.speed) < BLOCK_PISTON_SPEED)
		{

			if (shoot_control.piston_block_time < PISTON_BLOCK_TIME) // Time constraint for perventing false positive)
			{
				shoot_control.piston_block_time += SHOOT_CONTROL_TIME_MS;
			}
			else if (shoot_control.piston_block_time > PISTON_BLOCK_TIME)
			{
				shoot_control.piston_speed_set = 0;									  // Stop the piston motor if reached the end
				shoot_control.piston_direction = shoot_control.piston_direction * -1; // Reverse the direction of the piston
				shoot_control.piston_block_time = 0;
				isPistonBlocked = 1;
			}
		}
	} while (!isPistonBlocked); // Start the piston motor once the function is called

	isPistonBlocked = 0; // Reset the piston block flag
	return;
}
