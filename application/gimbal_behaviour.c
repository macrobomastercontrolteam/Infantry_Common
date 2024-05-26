/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_behaviour.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and encoder mode
  *             gyro mode: use euler angle to control, encond mode: use encoder
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a gimbal behaviour mode
    1. in gimbal_behaviour.h , add a new behaviour name in gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // new add
    }gimbal_behaviour_e,
    2. implement new function. gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" param is gimbal movement control input. 
        first param: 'yaw' usually means  yaw axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.
        second param: 'pitch' usually means pitch axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.

        in this new function, you can assign set-point to "yaw" and "pitch",as your wish
    3.  in "gimbal_behavour_set" function, add new logical judgement to assign GIMBAL_XXX_XXX to  "gimbal_behaviour" variable,
        and in the last of the "gimbal_behaviour_mode_set" function, add "else if(gimbal_behaviour == GIMBAL_XXX_XXX)" 
        choose a gimbal control mode.
        four mode:
        GIMBAL_MOTOR_RAW : will use 'yaw' and 'pitch' as motor current set,  derectly sent to can bus.
        GIMBAL_MOTOR_ENCODER : 'yaw' and 'pitch' are angle increment,  control encoder relative angle.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' are angle increment,  control gyro absolute angle.
    4. in the last of "gimbal_behaviour_control_set" function, add
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

#include "user_lib.h"
#include "cv_usart_task.h"

#define CV_ABS_ANGLE_INPUT 1 // 1 means abs angle input from cv, 0 means delta angle input from cv

//when gimbal is being calibrated, set buzzer frequency and strength
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

/**
  * @brief          judge if gimbal reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time) += GIMBAL_CONTROL_TIME_MS;                                    \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

#if CV_INTERFACE
static void gimbal_cv_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_cv_control_patrol(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
#endif

/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

// Watchout for the default value
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

#if GIMBAL_TEST_MODE
uint8_t gimbal_behaviour_global;
static void J_scope_gimbal_behavior_test(void)
{
    gimbal_behaviour_global = gimbal_behaviour;
}
#endif

/**
  * @brief          the function is called by gimbal_set_mode function in gimbal_task.c
  *                 the function set gimbal_behaviour variable, and set motor mode.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:
        case GIMBAL_CALI:
        {
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
            break;
        }
        case GIMBAL_ABSOLUTE_ANGLE:
        {
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
            break;
        }
        case GIMBAL_AUTO_AIM:
        case GIMBAL_AUTO_AIM_PATROL:
        {
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_CAMERA;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_CAMERA;
            break;
        }
        case GIMBAL_INIT:
        case GIMBAL_RELATIVE_ANGLE:
        case GIMBAL_MOTIONLESS:
        default:
        {
            gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER;
            gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER;
            break;
        }
    }

#if GIMBAL_TEST_MODE
    J_scope_gimbal_behavior_test();
#endif
}

/**
  * @brief          the function is called by gimbal_set_control function in gimbal_task.c
  *                 accoring to the gimbal_behaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_pitch:pitch axis increment angle,unit rad
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    switch (gimbal_behaviour)
    {
        case GIMBAL_ZERO_FORCE:
        {
            gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_INIT:
        {
            gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_CALI:
        {
            gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_ABSOLUTE_ANGLE:
        {
            gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_AUTO_AIM:
        {
            gimbal_cv_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_AUTO_AIM_PATROL:
        {
            gimbal_cv_control_patrol(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_RELATIVE_ANGLE:
        {
            gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
        case GIMBAL_MOTIONLESS:
        default:
        {
            gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
            break;
        }
    }
}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;

    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //in cali mode, return
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }
    //if other operate make step change to start, means enter cali mode
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP && !toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //init mode, judge if gimbal is in middle place
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //The initialization process is terminated due to exceeding the maximum initialization time, reaching a stable median value for an extended period, or the switch is down, or the remote control is offline
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[RC_RIGHT_LEVER_CHANNEL]) && !toe_is_error(DBUS_TOE))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    // remote controller logic
    if(gimbal_emergency_stop())
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
	else
	{
		switch (gimbal_mode_set->gimbal_rc_ctrl->rc.s[RC_RIGHT_LEVER_CHANNEL])
		{
			case RC_SW_UP:
			{
#if (ROBOT_TYPE == SENTRY_2023_MECANUM)
                if (CvCmder_GetMode(CV_MODE_AUTO_AIM_BIT))
                {
                    gimbal_behaviour = GIMBAL_AUTO_AIM;
                }
                else
#endif
                {
                    gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
                }
				break;
			}
			case RC_SW_MID:
			{
				gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
				break;
			}
			case RC_SW_DOWN:
			default:
			{
				gimbal_behaviour = GIMBAL_ZERO_FORCE;
				break;
			}
		}
	}

    //enter init mode (gimbal back to center)
	if ((last_gimbal_behaviour == GIMBAL_ZERO_FORCE) && (gimbal_behaviour != GIMBAL_ZERO_FORCE) && (gimbal_behaviour != GIMBAL_ABSOLUTE_ANGLE))
	{
		gimbal_behaviour = GIMBAL_INIT;
	}
	last_gimbal_behaviour = gimbal_behaviour;
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // calculate for init parameters
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;
	switch (gimbal_control_set->gimbal_cali.step)
	{
		case GIMBAL_CALI_PITCH_MAX_STEP:
		{
			*pitch = GIMBAL_CALI_MOTOR_SET;
			*yaw = 0;
			// judge gyro data, and record max and min angle data
			gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
			                       gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
			                       gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
			break;
		}
		case GIMBAL_CALI_PITCH_MIN_STEP:
		{
			*pitch = -GIMBAL_CALI_MOTOR_SET;
			*yaw = 0;

			gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
			                       gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
			                       gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
#if ROBOT_YAW_HAS_SLIP_RING
            gimbal_control_set->gimbal_cali.max_yaw = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
            gimbal_control_set->gimbal_cali.max_yaw_ecd = gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd;
            gimbal_control_set->gimbal_cali.max_yaw = gimbal_control_set->gimbal_cali.min_yaw;
            gimbal_control_set->gimbal_cali.max_yaw_ecd = gimbal_control_set->gimbal_cali.min_yaw_ecd;
#endif
            break;
		}
    #if(!ROBOT_YAW_HAS_SLIP_RING)
		case GIMBAL_CALI_YAW_MAX_STEP:
		{
			*pitch = 0;
			*yaw = GIMBAL_CALI_MOTOR_SET;

			gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
			                       gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
			                       gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
			break;
		}
		case GIMBAL_CALI_YAW_MIN_STEP:
		{
			*pitch = 0;
			*yaw = -GIMBAL_CALI_MOTOR_SET;

			gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
			                       gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
			                       gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
			break;
		}
    #endif
		case GIMBAL_CALI_END_STEP:
		{
			cali_time = 0;
			break;
		}
		default:
		{
			break;
		}
	}
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL], yaw_channel, RC_DEADBAND);
    deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[JOYSTICK_LEFT_VERTICAL_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN_INC + gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_RC_MOUSE_SEN_INC;
    *pitch = pitch_channel * PITCH_RC_SEN_INC + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_RC_MOUSE_SEN_INC;


    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                // save the target value to turn
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            // control to the target value of turning, positive or reverse rotation is random
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TURN_SPEED;
            }
            else
            {
                *yaw -= TURN_SPEED;
            }
        }
        // stop after reaching pi
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}

#if CV_INTERFACE
/**
  * @brief          GIMBAL_AUTO_AIM mode: gimbal_motor_mode is GIMBAL_MOTOR_GYRO, gimbal_behaviour is GIMBAL_ABSOLUTE_ANGLE
  *                 Search for enemy in current camera frame without sweeping yaw and pitch angle, as opposite to GIMBAL_AUTO_AIM_PATROL mode
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_cv_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    // Positive Directions
    // CvCmdHandler.CvCmdMsg.xAngle: right
    // CvCmdHandler.CvCmdMsg.yAngle: up
    // yaw_target_adjustment : left (same direction as IMU)
    // pitch_target_adjustment: down (same direction as IMU)
    fp32 yaw_target_adjustment = 0;
    fp32 pitch_target_adjustment = 0;
    if (checkAndResetFlag(&CvCmdHandler.fCvCmdValid))
    {
#if CV_ABS_ANGLE_INPUT
		yaw_target_adjustment = rad_format(CvCmdHandler.CvCmdMsg.xAngle + gimbal_control_set->gimbal_yaw_motor.absolute_angle_offset - gimbal_control_set->gimbal_yaw_motor.absolute_angle_set);        
		pitch_target_adjustment = rad_format(CvCmdHandler.CvCmdMsg.yAngle + gimbal_control_set->gimbal_pitch_motor.absolute_angle_offset - gimbal_control_set->gimbal_pitch_motor.absolute_angle_set);
#else
		yaw_target_adjustment = -CvCmdHandler.CvCmdMsg.xAngle - rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle);
		pitch_target_adjustment = -CvCmdHandler.CvCmdMsg.yAngle - rad_format(gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle);
#endif
	}
    // brakeband_limit(yaw_target_adjustment, yaw_target_adjustment, CV_CAMERA_YAW_BRAKEBAND);
    // brakeband_limit(pitch_target_adjustment, pitch_target_adjustment, CV_CAMERA_PITCH_BRAKEBAND);
    // *yaw = moving_average_calc(yaw_target_adjustment, &(gimbal_control_set->gimbal_yaw_motor.CvCmdAngleFilter), MOVING_AVERAGE_CALC);
    // *pitch = moving_average_calc(pitch_target_adjustment, &(gimbal_control_set->gimbal_pitch_motor.CvCmdAngleFilter), MOVING_AVERAGE_CALC);
    *yaw = yaw_target_adjustment;
    *pitch = pitch_target_adjustment;
}

/**
 * @brief          GIMBAL_AUTO_AIM_PATROL mode: gimbal_motor_mode is GIMBAL_MOTOR_GYRO, gimbal_behaviour is GIMBAL_ABSOLUTE_ANGLE
 *                 Search for enemy while sweeping yaw and pitch angle for cv to detect, as opposite to GIMBAL_AUTO_AIM mode
 * @param[out]     yaw: yaw axia absolute angle increment, unit rad
 * @param[out]     pitch: pitch axia absolute angle increment,unit rad
 * @param[in]      gimbal_control_set: gimbal data
 * @retval         none
 */
static void gimbal_cv_control_patrol(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL)
    {
        return;
    }

    // @TODO: patrol mode state machine
    *yaw = 0.0f;
    *pitch = 0.0f;
}
#endif

/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL], yaw_channel, RC_DEADBAND);
    deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[JOYSTICK_LEFT_VERTICAL_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN_INC + gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_RC_MOUSE_SEN_INC;
    *pitch = pitch_channel * PITCH_RC_SEN_INC + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_RC_MOUSE_SEN_INC;


}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
