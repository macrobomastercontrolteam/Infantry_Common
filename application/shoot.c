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

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "cv_usart_task.h"

#define USE_SERVO_TO_STIR_AMMO 0
#define TEST_SHOOT_WITH_REF 0

#define shoot_laser_on() laser_on()
#define shoot_laser_off() laser_off()
//microswitch
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)

/**
 * @brief Set the mode of the shoot control state machine
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
static void trigger_motor_turn_back(void);

/**
  * @brief          Shoot control, control the angle of the trigger motor to complete a single burst shot
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

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
    PID_init(&shoot_control.friction_motor1_pid, PID_POSITION, shoot_speed_pid1, FRICTION_1_SPEED_PID_MAX_OUT, FRICTION_1_SPEED_PID_MAX_IOUT, &raw_err_handler);
    PID_init(&shoot_control.friction_motor2_pid, PID_POSITION, shoot_speed_pid2, FRICTION_2_SPEED_PID_MAX_OUT, FRICTION_2_SPEED_PID_MAX_IOUT, &raw_err_handler);
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT, &raw_err_handler);

    //update data
    shoot_feedback_update();

    shoot_control.friction_motor1_rpm_set = 0.0f;
    shoot_control.friction_motor2_rpm_set = 0.0f;
    shoot_control.ecd_count = 0;
    shoot_control.angle = motor_chassis[MOTOR_INDEX_TRIGGER].ecd * TRIGGER_MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.fIsCvControl = 0;
}

/**
  * @brief          Set the shoot mode state machine, pull up once on the remote control to turn on, pull up again to turn off, pull down once to shoot one, always down to continue shooting, used to clean up bullets during the 3-minute preparation time
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
        switch (shoot_control.shoot_mode)
        {
            case SHOOT_STOP:
            {
#if USE_SERVO_TO_STIR_AMMO
                CAN_cmd_load_servo(0, 3);
#endif

                shoot_control.friction_motor1_rpm_set = 0;
                shoot_control.friction_motor2_rpm_set = 0;
                break;
            }
            case SHOOT_READY_FRIC:
            {
#if USE_SERVO_TO_STIR_AMMO
                CAN_cmd_load_servo(1, 3);
#endif
                shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
                shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;

                PID_clear(&shoot_control.friction_motor1_pid);
                PID_clear(&shoot_control.friction_motor2_pid);
                
                shoot_control.friction_motor1_pid.max_out = FRICTION_1_SPEED_PID_MAX_OUT;
                shoot_control.friction_motor1_pid.max_iout = FRICTION_1_SPEED_PID_MAX_IOUT;

                shoot_control.friction_motor2_pid.max_out = FRICTION_2_SPEED_PID_MAX_OUT;
                shoot_control.friction_motor2_pid.max_iout = FRICTION_2_SPEED_PID_MAX_IOUT;

                shoot_control.friction_motor1_rpm_set = -FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
                shoot_control.friction_motor2_rpm_set = FRICTION_MOTOR_SPEED * FRICTION_MOTOR_SPEED_TO_RPM;
                break;
            }
            case SHOOT_AUTO_FIRE:
            case SHOOT_SEMI_AUTO_FIRE:
            {
                shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
                shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
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
		if (toe_is_error(FRIC1_MOTOR_TOE) || toe_is_error(FRIC2_MOTOR_TOE))
		{
			shoot_control.shoot_mode = SHOOT_STOP;
		}
		else if ((fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_LEFT].speed_rpm / shoot_control.friction_motor1_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD) && (fabs((float)motor_chassis[MOTOR_INDEX_FRICTION_RIGHT].speed_rpm / shoot_control.friction_motor2_rpm_set) > FRICTION_MOTOR_SPEED_THRESHOLD))
		{
			if (shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL] == RC_SW_UP)
			{
				shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
			}
			else
			{
				shoot_control.shoot_mode = SHOOT_SEMI_AUTO_FIRE;
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
            trigger_motor_turn_back();
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
//         else if (shoot_control.fIsCvControl == 0)
//         {
//             // long press mouse to rapid fire
//             if ((shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
//             {
//                 shoot_control.shoot_mode = SHOOT_SEMI_AUTO_FIRE;
//             }
//         }
        break;
    }
    case SHOOT_SEMI_AUTO_FIRE:
    {
#if (TEST_SHOOT_WITH_REF)
        get_shoot_heat_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
        if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
        {
            shoot_control.trigger_motor_pid.max_out = 0;
            shoot_control.trigger_motor_pid.max_iout = 0;
        }
        else
        {
            shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
            shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        }
#endif
        shoot_bullet_control();

        // hold shoot command to enter auto mode
		if (shoot_control.shoot_hold_time == RC_S_LONG_TIME)
		{
            shoot_control.shoot_mode = SHOOT_AUTO_FIRE;
		}
        break;
    }
    case SHOOT_AUTO_FIRE:
    {
        // 设置拨弹轮的拨动速度,并开启堵转反转处理
#if (TEST_SHOOT_WITH_REF)
        get_shoot_heat_limit_and_heat(&shoot_control.heat_limit, &shoot_control.heat);
        if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
        {
            shoot_control.trigger_motor_pid.max_out = 0;
            shoot_control.trigger_motor_pid.max_iout = 0;
        }
        else
        {
            shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
            shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        }
#endif
        shoot_control.trigger_speed_set = AUTO_FIRE_TRIGGER_SPEED;
        trigger_motor_turn_back();
        break;
    }
    case SHOOT_DONE:
    {
        if (shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time++;
            if (shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_TRIGGER;
            }
        }
        // else
        // {
        //     shoot_control.key_time = 0;
        //     shoot_control.shoot_mode = SHOOT_SEMI_AUTO_FIRE;
        // }
        break;
    }
    }

	if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        shoot_control.friction_motor1_rpm_set = 0.0f;
        shoot_control.friction_motor2_rpm_set = 0.0f;
    }
    else
    {
        shoot_laser_on();

		// trigger motor PID
		if (shoot_control.shoot_mode < SHOOT_READY_TRIGGER)
		{
			shoot_control.given_current = 0;
		}
		else
		{
			PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
			shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
		}
    }

    PID_calc(&shoot_control.friction_motor1_pid, shoot_control.friction_motor1_rpm, shoot_control.friction_motor1_rpm_set);
    shoot_control.fric1_given_current = (int16_t)(shoot_control.friction_motor1_pid.out);

    PID_calc(&shoot_control.friction_motor2_pid, shoot_control.friction_motor2_rpm, shoot_control.friction_motor2_rpm_set);
    shoot_control.fric2_given_current = (int16_t)(shoot_control.friction_motor2_pid.out);

    return shoot_control.given_current;
}

/**
  * @brief          Set the shoot mode state machine, pull up once on the remote control to turn on, pull up again to turn off, pull down once to shoot one, always down to continue shooting, used to clean up bullets during the 3-minute preparation time
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	shoot_control.fIsCvControl = (shoot_control.shoot_rc->rc.s[RC_RIGHT_LEVER_CHANNEL] == RC_SW_UP);
#else
    // shoot_control.fIsCvControl = 0; // It is 0 by default
#endif

#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
	if (shoot_control.fIsCvControl)
	{
		static uint8_t lastCvShootMode = 0;
		uint8_t CvShootMode = CvCmder_GetMode(CV_MODE_SHOOT_BIT);
		if (CvShootMode != lastCvShootMode)
		{
			if (CvShootMode == 1)
			{
				shoot_control.shoot_mode = SHOOT_READY_FRIC;
			}
			else
			{
				shoot_control.shoot_mode = SHOOT_STOP;
			}
			lastCvShootMode = CvShootMode;
		}
	}
	else
#endif
	{
        // normal RC control
        if (gimbal_cmd_to_shoot_stop() || toe_is_error(FRIC1_MOTOR_TOE) || toe_is_error(FRIC2_MOTOR_TOE))
		{
			shoot_control.shoot_mode = SHOOT_STOP;
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
					if (shoot_control.press_l)
					{
						if (shoot_control.last_press_l == 0)
						{
							shoot_control.shoot_mode = SHOOT_READY_FRIC;
						}
					}
					else
					{
						shoot_control.shoot_mode = SHOOT_STOP;
					}
					break;
				}
				case RC_SW_DOWN:
				{
					if (last_s != RC_SW_DOWN)
					{
						if (shoot_control.shoot_mode == SHOOT_READY)
						{
							// burst fire
							shoot_control.shoot_mode = SHOOT_SEMI_AUTO_FIRE;
						}
					}
					break;
				}
			}
			last_s = new_s;
		}
	}
}
/**
  * @brief          Update shooting data
  * @param[in]      void
  * @retval         void
  */
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

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + motor_chassis[MOTOR_INDEX_TRIGGER].ecd) * TRIGGER_MOTOR_ECD_TO_ANGLE;
    shoot_control.key = BUTTEN_TRIG_PIN;
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;

    if ((shoot_control.shoot_mode > SHOOT_READY_TRIGGER) && (shoot_control.shoot_mode < SHOOT_DONE))
    {
        if (shoot_control.shoot_hold_time < RC_S_LONG_TIME)
        {
            shoot_control.shoot_hold_time++;
        }
    }
    else
    {
        shoot_control.shoot_hold_time = 0;
    }
}

static void trigger_motor_turn_back(void)
{
#if TRIGGER_TURN
    shoot_control.speed_set = -shoot_control.trigger_speed_set;
#else
    shoot_control.speed_set = shoot_control.trigger_speed_set;
#endif
    if( shoot_control.block_time >= BLOCK_TIME)
    {
        shoot_control.speed_set = -shoot_control.trigger_speed_set;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}


static void shoot_bullet_control(void)
{
    // Rotate by TRIGGER_ANGLE_INCREMENT every time
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + TRIGGER_ANGLE_INCREMENT);
        shoot_control.move_flag = 1;
    }

    if ((shoot_control.press_l == 0) && (shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL] != RC_SW_UP) && (shoot_control.key == SWITCH_TRIGGER_OFF))
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }

    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        shoot_control.trigger_speed_set = SEMI_AUTO_FIRE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}

