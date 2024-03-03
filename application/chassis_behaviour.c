  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             根据遥控器的值，决定底盘行为。
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

        
    如果要添加一个新的行为模式
    1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // 新添加的
    }chassis_behaviour_e,

    2. 实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"


#include "cv_usart_task.h"
#include "detect_task.h"

#define RPM_TO_RADS(_ROUND_PER_MIN) (_ROUND_PER_MIN*0.10471975511965977f)
#define SPINNING_CHASSIS_LOW_OMEGA (RPM_TO_RADS(25.0f))
#define SPINNING_CHASSIS_MED_OMEGA (RPM_TO_RADS(30.0f))
#define SPINNING_CHASSIS_HIGH_OMEGA (RPM_TO_RADS(35.0f))

#if CHASSIS_TEST_MODE
int32_t chassis_behaviour_mode_int;
static void J_scope_chassis_behavior_test(void)
{
    chassis_behaviour_mode_int = (int32_t)(chassis_behaviour_mode);
}
#endif

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
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


//highlight, the variable chassis behaviour mode 
//留意，这个底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}

	// safety guard
	uint8_t fIsError = 0;
	uint8_t bToeIndex;
	for (bToeIndex = DBUS_TOE; bToeIndex <= CHASSIS_MOTOR4_TOE; bToeIndex++)
	{
		if (toe_is_error(bToeIndex))
		{
			fIsError = 1;
			break;
		}
	}

	if (fIsError)
	{
		chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
	}
	else
	{
		switch (chassis_move_mode->chassis_RC->rc.s[RIGHT_LEVER_CHANNEL])
		{
			case RC_SW_UP:
			case RC_SW_MID:
			{
				if (chassis_move_mode->chassis_RC->rc.s[LEFT_LEVER_CHANNEL] == RC_SW_UP)
				{
					chassis_behaviour_mode = CHASSIS_NO_MOVE;
          chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
				}
				else
				{
					chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
          chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
				}
				break;
			}
			case RC_SW_DOWN:
			default:
			{
				chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
				break;
			}
		}
	}

  // arm control mode
	switch (chassis_behaviour_mode)
	{
		case CHASSIS_NO_FOLLOW_YAW:
    case CHASSIS_NO_MOVE:
		{
			switch (chassis_move_mode->chassis_RC->rc.s[LEFT_LEVER_CHANNEL])
			{
				case RC_SW_UP:
				{
					chassis_move_mode->robot_arm_mode = ROBOT_ARM_ENABLED;
					break;
				}
				case RC_SW_MID:
				{
					chassis_move_mode->robot_arm_mode = ROBOT_ARM_FIXED;
					break;
				}
				case RC_SW_DOWN:
				default:
				{
					chassis_move_mode->robot_arm_mode = ROBOT_ARM_HOME;
					break;
				}
			}
			break;
		}
		case CHASSIS_ZERO_FORCE:
		default:
		{
			chassis_move_mode->robot_arm_mode = ROBOT_ARM_ZERO_FORCE;
			break;
		}
	}

#if CHASSIS_TEST_MODE
	J_scope_chassis_behavior_test();
#endif
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
/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
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
		case CHASSIS_NO_FOLLOW_YAW:
		{
			chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
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
/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}
