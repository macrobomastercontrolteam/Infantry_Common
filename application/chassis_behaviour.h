#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "chassis_task.h"
#include "struct_typedef.h"

typedef enum
{
	CHASSIS_ZERO_FORCE,    // chassis will be like no power,底盘无力, 跟没上电那样
	CHASSIS_NO_MOVE,       // chassis will be stop,底盘保持不动
	CHASSIS_NO_FOLLOW_YAW, // chassis does not follow angle, angle is open-loop,but wheels have closed-loop speed
	                       // 底盘不跟随角度，角度是开环的，但轮子是有速度环
	CHASSIS_SPINNING, // @TODO: Chassis spinning mode
} chassis_behaviour_e;

#define CHASSIS_OPEN_RC_SCALE 10.0f // in CHASSIS_OPEN mode, multiply the value. 在chassis_open 模型下，遥控器乘以该比例发送到can上

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
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

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

extern void chassis_behaviour_control_set(chassis_move_t *chassis_move_rc_to_vector);

extern chassis_behaviour_e chassis_behaviour_mode;

#endif
