/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

#if DISABLE_SHOOT_MOTOR_POWER
#define shoot_fric1_on(pwm) fric_off() //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric_off() //摩擦轮2pwm宏定义
#else
#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#endif

#define shoot_fric_off()    fric_off()      //关闭两个摩擦轮

#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)




/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);



shoot_control_t shoot_control;          //射击数据


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP_INIT;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //初始化PID
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT, &raw_err_handler);
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * TRIGGER_MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
    shoot_control.fIsCvControl = 0;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据

    shoot_control.speed_set = 0.0f;
    switch (shoot_control.shoot_mode)
    {
    case SHOOT_STOP_INIT:
    {
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
        CAN_cmd_load_servo(0);
        CAN_cmd_load_servo(0);
        CAN_cmd_load_servo(0);
#endif
        shoot_control.shoot_mode = SHOOT_STOP;
        // no break; directly go to next state
    }
    case SHOOT_STOP:
    {
        break;
    }
    case SHOOT_READY_FRIC:
    {
        if ((shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value) && (shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value))
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET_INIT;
        }
        break;
    }
    case SHOOT_READY_BULLET_INIT:
    {
#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
        CAN_cmd_load_servo(1);
        CAN_cmd_load_servo(1);
        CAN_cmd_load_servo(1);
#endif
		shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
        // no break; directly go to next state
    }
    case SHOOT_READY_BULLET:
    {
        if (shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            // 设置拨弹轮的拨动速度,并开启堵转反转处理
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
            shoot_control.shoot_mode = SHOOT_READY_BULLET_INIT;
        }
//         else if (shoot_control.fIsCvControl == 0)
//         {
//             // long press mouse to rapid fire
//             if ((shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
//             {
//                 shoot_control.shoot_mode = SHOOT_BULLET;
//             }
//         }
        break;
    }
    case SHOOT_BULLET:
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();

        get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
        if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
        {
            shoot_control.shoot_mode = SHOOT_READY_BULLET_INIT;
        }
        break;
    }
    case SHOOT_CONTINUE_BULLET:
    {
        // 设置拨弹轮的拨动速度,并开启堵转反转处理
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
        shoot_control.shoot_mode = SHOOT_READY_BULLET_INIT;
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
                shoot_control.shoot_mode = SHOOT_READY_BULLET_INIT;
            }
        }
        // else
        // {
        //     shoot_control.key_time = 0;
        //     shoot_control.shoot_mode = SHOOT_BULLET;
        // }
        break;
    }
    }

    // Continue bullet logic: must be used after state machine, since SHOOT_CONTINUE_BULLET switch back to SHOOT_READY_BULLET if not triggered
	if (shoot_control.shoot_mode > SHOOT_READY_FRIC)
	{
		if (shoot_control.fIsCvControl)
		{
			shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
		}
		// 鼠标长按一直进入射击状态 保持连发
		else if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
		{
			get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
			if (!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
			{
				shoot_control.shoot_mode = SHOOT_READY_BULLET_INIT;
			}
			else
			{
				shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
			}
		}
	}

	if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_laser_off();
        shoot_control.given_current = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY_BULLET_INIT)
        {
            shoot_control.given_current = 0;
        }
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);
    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
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
				shoot_control.shoot_mode = SHOOT_STOP_INIT;
			}
			lastCvShootMode = CvShootMode;
		}
	}
	else
#endif
	{
        // normal RC control
		if (gimbal_cmd_to_shoot_stop())
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
					if (shoot_control.shoot_mode != SHOOT_STOP)
					{
						shoot_control.shoot_mode = SHOOT_STOP_INIT;
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
							shoot_control.shoot_mode = SHOOT_BULLET;
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
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * TRIGGER_MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
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

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * TRIGGER_MOTOR_ECD_TO_ANGLE;
    //微动开关
    shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[RC_LEFT_LEVER_CHANNEL]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
    static uint16_t up_time = 0;
    if (shoot_control.press_r)
    {
        up_time = UP_ADD_TIME;
    }

    if (up_time > 0)
    {
        shoot_control.fric1_ramp.max_value = FRIC_UP;
        shoot_control.fric2_ramp.max_value = FRIC_UP;
        up_time--;
    }
    else
    {
        // always fast speed
        shoot_control.fric1_ramp.max_value = FRIC_UP;
        shoot_control.fric2_ramp.max_value = FRIC_UP;
        // shoot_control.fric1_ramp.max_value = FRIC_DOWN;
        // shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    }


}

static void trigger_motor_turn_back(void)
{
#if defined(TRIGGER_TURN)
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

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    // Rotate by TRIGGER_ANGLE_INCREMENT every time
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + TRIGGER_ANGLE_INCREMENT);
        shoot_control.move_flag = 1;
    }
    if(shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }
    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}

