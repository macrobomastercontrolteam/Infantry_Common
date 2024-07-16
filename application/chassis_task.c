/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
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

#include "cmsis_os.h"

#include "CAN_receive.h"
#include "INS_task.h"
#include "arm_math.h"
#include "biped.h"
#include "chassis_power_control.h"
#include "detect_task.h"
#include "pid.h"
#include "remote_control.h"
#include "cv_usart_task.h"

static void wait_until_motors_online(void);
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_rc_parse(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if defined(INFANTRY_3)
static uint16_t motor_angle_to_ecd_change(fp32 angle);
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

// 底盘运动数据
chassis_move_t chassis_move;
fp32 chassis_task_loop_delay;

#if CHASSIS_JSCOPE_DEBUG
// fp32 leg_L_drive_multiangle = 0;
// fp32 leg_R_drive_multiangle = 0;
// fp32 leg_L_drive_input_angle = 0;
// fp32 leg_R_drive_input_angle = 0;
// fp32 out_LF = 0;
// fp32 out_LB = 0;
// fp32 out_RF = 0;
// fp32 out_RB = 0;
// fp32 angle_L1 = 0;
// fp32 angle_L4 = 0;
// fp32 angle_R1 = 0;
// fp32 angle_R4 = 0;
// fp32 leg_R_TL_set = 0;
// fp32 leg_R_TR_set = 0;
// fp32 leg_L_TL_set = 0;
// fp32 leg_L_TR_set = 0;
// fp32 leg_L_TWheel_set = 0;
// fp32 leg_R_TWheel_set = 0;
fp32 leg_l_l0_set = 0;
fp32 leg_r_l0_set = 0;
fp32 leg_l_l0_now = 0;
fp32 leg_r_l0_now = 0;
fp32 leg_sim_angle0_now = 0;
fp32 leg_sim_angle0_dot = 0;
fp32 leg_sim_dis_diff = 0;
int32_t leg_sim_dis_dot = 0;
fp32 biped_pitch_now = 0;
fp32 biped_pitch_dot = 0;
fp32 biped_yaw_now = 0;
fp32 biped_yaw_set = 0;
fp32 biped_roll_now = 0;
fp32 biped_roll_set = 0;
uint32_t loop_delay = 0;
uint8_t jump_state;
uint8_t brake_state;
// int32_t cvDisDelta_int = 0;
static void jscope_chassis_test(void)
{
	// leg_L_drive_input_angle = motor_measure[CHASSIS_ID_DRIVE_LEFT].input_angle;
	// leg_R_drive_input_angle = motor_measure[CHASSIS_ID_DRIVE_RIGHT].input_angle;

	// leg_L_drive_multiangle = motor_measure[CHASSIS_ID_DRIVE_LEFT].output_angle;
	// leg_R_drive_multiangle = motor_measure[CHASSIS_ID_DRIVE_RIGHT].output_angle;

	// out_LF = motor_measure[CHASSIS_ID_HIP_LF].output_angle * 180.0f / PI;
	// out_LB = motor_measure[CHASSIS_ID_HIP_LB].output_angle * 180.0f / PI;
	// out_RF = motor_measure[CHASSIS_ID_HIP_RF].output_angle * 180.0f / PI;
	// out_RB = motor_measure[CHASSIS_ID_HIP_RB].output_angle * 180.0f / PI;

	// angle_L1 = biped.leg_L.angle1*180.0f/PI;
	// angle_L4 = biped.leg_L.angle4*180.0f/PI;
	// angle_R1 = biped.leg_R.angle1*180.0f/PI;
	// angle_R4 = biped.leg_R.angle4*180.0f/PI;

	// leg_R_TL_set = biped.leg_R.TL_set;
	// leg_R_TR_set = biped.leg_R.TR_set;
	// leg_L_TL_set = biped.leg_L.TL_set;
	// leg_L_TR_set = biped.leg_L.TR_set;

	// leg_L_TWheel_set = biped.leg_L.TWheel_set;
	// leg_R_TWheel_set = biped.leg_R.TWheel_set;
	leg_l_l0_set = biped.leg_L.L0.set * 1000.0f;
	leg_r_l0_set = biped.leg_R.L0.set * 1000.0f;
	leg_l_l0_now = biped.leg_L.L0.now * 1000.0f;
	leg_r_l0_now = biped.leg_R.L0.now * 1000.0f;

	leg_sim_angle0_now = biped.leg_simplified.angle0.now * 180.0f / PI;
	leg_sim_angle0_dot = biped.leg_simplified.angle0.dot * 180.0f / PI;
	leg_sim_dis_diff = biped_get_dis_diff() * 1000.0f;
	leg_sim_dis_dot = biped.leg_simplified.dis.dot * 1000.0f;

	biped_pitch_now = biped.pitch.now * 180.0f / PI;
	biped_pitch_dot = biped.pitch.dot * 180.0f / PI;

	biped_yaw_now = biped.yaw.now * 180.0f / PI;
	biped_yaw_set = biped.yaw.set * 180.0f / PI;

	biped_roll_now = biped.roll.now * 180.0f / PI;
	biped_roll_set = biped.roll.set * 180.0f / PI;

	loop_delay = HAL_GetTick() - biped.time_ms;

	jump_state = biped.jumpState;
	brake_state = biped.brakeState;

	// cvDisDelta_int = CvCmdHandler.CvCmdMsg.disDelta * 1000;
}
#endif

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS
 * @param[in]      pvParameters: null
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
	static uint32_t ulPreviousOsWakeTime;
	ulPreviousOsWakeTime = osKernelSysTick();
	osDelay(BIPED_CHASSIS_TASK_INIT_TIME);

	chassis_init(&chassis_move);
	biped_init();
	biped_status_update();

	wait_until_motors_online();

	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		biped_status_update();
		chassis_rc_parse(&chassis_move);
		chassis_control_loop(&chassis_move);

		if (biped.fBipedEnable)
		{
			uint8_t bToeIndex;
			for (bToeIndex = DBUS_TOE; bToeIndex <= CHASSIS_DRIVE_MOTOR2_TOE; bToeIndex++)
			{
				if (toe_is_error(bToeIndex))
				{
					biped.fBipedEnable = 0;
					break;
				}
			}

			// flipped over
			// if (fabs(biped.pitch.now) >= 80.0f / 180.0f * PI)
			// {
			// 	biped.fBipedEnable = 0;
			// }

			// switching edge
			if (biped.fBipedEnable == 0)
			{
				enable_all_8006_motors(0);
			}
		}

		if (biped.fBipedEnable == 0)
		{
			biped.HipTorque_MaxLimit = 0;
			biped.DriveTorque_MaxLimit = 0;
		}

		// hip torques are supposed to be positive to lift the robot
		biped.leg_L.TL_set = fp32_constrain(biped.leg_L.TL_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
		biped.leg_L.TR_set = fp32_constrain(biped.leg_L.TR_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
		biped.leg_R.TL_set = fp32_constrain(biped.leg_R.TL_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
		biped.leg_R.TR_set = fp32_constrain(biped.leg_R.TR_set, -biped.HipTorque_MaxLimit, biped.HipTorque_MaxLimit);
		biped.leg_L.TWheel_set = fp32_constrain(biped.leg_L.TWheel_set, -biped.DriveTorque_MaxLimit, biped.DriveTorque_MaxLimit);
		biped.leg_R.TWheel_set = fp32_constrain(biped.leg_R.TWheel_set, -biped.DriveTorque_MaxLimit, biped.DriveTorque_MaxLimit);

		uint8_t blocking_call = 1;
		uint8_t fValidCmd = 1;
		fValidCmd &= drive_motor_set_torque(biped.leg_R.TWheel_set, biped.leg_L.TWheel_set, blocking_call);
		fValidCmd &= hip_motor_set_torque(biped.leg_R.TR_set, biped.leg_L.TR_set, biped.leg_L.TL_set, biped.leg_R.TL_set, blocking_call);

		osDelayUntil(&ulPreviousOsWakeTime, CHASSIS_CONTROL_TIME_MS);

#if CHASSIS_JSCOPE_DEBUG
		jscope_chassis_test();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

static void wait_until_motors_online(void)
{
	// keep dialling until all chassis motors are online
	uint8_t blocking_call = 1;
	uint8_t bToeIndex;
	hip_motor_set_torque(0, 0, 0, 0, blocking_call);
	drive_motor_set_torque(0, 0, blocking_call);
	osDelay(2);
	for (bToeIndex = DBUS_TOE; bToeIndex <= CHASSIS_DRIVE_MOTOR2_TOE; bToeIndex++)
	{
		while (toe_is_error(bToeIndex))
		{
			switch (bToeIndex)
			{
				case CHASSIS_HIP_MOTOR1_TOE:
				case CHASSIS_HIP_MOTOR2_TOE:
				case CHASSIS_HIP_MOTOR3_TOE:
				case CHASSIS_HIP_MOTOR4_TOE:
				{
					enable_motor_control_8006(bToeIndex - CHASSIS_HIP_MOTOR1_TOE + CAN_HIP1_TX_ID, 1);
					hip_motor_set_torque(0, 0, 0, 0, blocking_call);
					break;
				}
				case CHASSIS_DRIVE_MOTOR1_TOE:
				{
					encode_motor_control(CAN_DRIVE1_PVT_TX_ID, 0, 0, 0, 0, 0, blocking_call, MA_9015);
					break;
				}
				case CHASSIS_DRIVE_MOTOR2_TOE:
				{
					encode_motor_control(CAN_DRIVE2_PVT_TX_ID, 0, 0, 0, 0, 0, blocking_call, MA_9015);
					break;
				}
				case DBUS_TOE:
				{
					break;
				}
			}
			osDelay(10);
		}
	}

	// retrigger motor feedback to get them online
	hip_motor_set_torque(0, 0, 0, 0, blocking_call);
	drive_motor_set_torque(0, 0, blocking_call);
	osDelay(5);
	// fake online
	// uint8_t repeat_index;
	// for (repeat_index = 0; repeat_index < 2; repeat_index++)
	// {
	// 	for (bToeIndex = DBUS_TOE; bToeIndex <= CHASSIS_DRIVE_MOTOR2_TOE; bToeIndex++)
	// 	{
	// 		detect_hook(bToeIndex);
	// 	}
	// 	osDelay(15);
	// }
}

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @param[out]     chassis_move_init: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
 * @param[out]     chassis_move_init:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}

	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	chassis_move_init->last_chassis_mode = chassis_move_init->chassis_mode;

	chassis_move_init->chassis_RC = get_remote_control_point();
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	chassis_move_init->chassis_INS_speed = get_gyro_data_point();
	// chassis_move_init->chassis_INS_accel = get_accel_data_point();
	// chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
	// chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
}

/**
 * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
 * @param[out]     chassis_move_mode: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
 * @param[out]     chassis_move_mode:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}
	// in file "chassis_behaviour.c"
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
 * @param[out]     chassis_move_transit: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
 * @param[out]     chassis_move_transit:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
	if (chassis_move_transit == NULL)
	{
		return;
	}

	if (chassis_move_transit->last_chassis_mode != chassis_move_transit->chassis_mode)
	{
		switch (chassis_move_transit->chassis_mode)
		{
			case CHASSIS_VECTOR_NO_FOLLOW_YAW:
			case CHASSIS_VECTOR_CV_NO_FOLLOW_YAW:
			{
				enable_all_8006_motors(1);
				
				// biped_init();
				biped.fBipedEnable = 1;
				biped.fCvBrakeEnable = 1;
				biped.isJumpInTheAir = 0;
				biped.jumpState = JUMP_IDLE;
				biped.brakeState = BRAKE_IDLE;

				biped.yaw.set = biped.yaw.now;

				biped.pitch.set = 0.0f;
				biped.roll.set = 0.0f;
				// @TODO: biped.balance_angle

				// biped.velocity.set = 0;
				// biped.velocity.last = biped.velocity.now;

				biped.leg_L.dis.now = 0;
				biped.leg_R.dis.now = 0;
				biped.leg_simplified.dis.now = 0;

				biped_set_dis(biped.leg_simplified.dis.now, (biped.pitch.now < 0));

				biped.leg_L.dis.last = biped.leg_L.dis.now;
				biped.leg_R.dis.last = biped.leg_R.dis.now;
				biped.leg_simplified.dis.last = biped.leg_simplified.dis.now;

				biped.leg_L.angle0.last = biped.leg_L.angle0.now;
				biped.leg_R.angle0.last = biped.leg_R.angle0.now;

				biped.leg_L.L0.set = LEG_L0_MIN;
				biped.leg_R.L0.set = LEG_L0_MIN;
				biped.leg_L.L0.last = biped.leg_L.L0.now;
				biped.leg_R.L0.last = biped.leg_R.L0.now;

				biped.leg_L.fResetMultiAngleOffset = 1;
				biped.leg_R.fResetMultiAngleOffset = 1;

				biped.HipTorque_MaxLimit = HIP_TORQUE_MAX;
				biped.DriveTorque_MaxLimit = DRIVE_TORQUE_MAX;				
				break;
			}
			default:
			{
				biped.fBipedEnable = 0;
				biped.fCvBrakeEnable = 1;

				biped.isJumpInTheAir = 0;
				biped.jumpState = JUMP_IDLE;
				biped.brakeState = BRAKE_IDLE;

				biped.HipTorque_MaxLimit = 0.0f;
				biped.DriveTorque_MaxLimit = 0.0f;
				biped.leg_L.fResetMultiAngleOffset = 1;
				biped.leg_R.fResetMultiAngleOffset = 1;

				biped.leg_simplified.dis.now = 0;
				biped_set_dis(biped.leg_simplified.dis.now, (biped.pitch.now < 0));
				break;
			}
		}

		chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
	}
}

void chassis_cv_to_control_vector(chassis_move_t *chassis_move_ptr, fp32* pDistanceDelta)
{
	if (chassis_move_ptr == NULL)
	{
		return;
	}

#if defined(CV_INTERFACE)
	if (toe_is_error(CV_TOE))
	{
		// use braking logic
		*pDistanceDelta = 0;
		biped.fCvBrakeEnable = 1;
	}
	else if (CvCmder_CheckAndResetFlag(&CvCmdHandler.fCvCmdValid))
	{
		*pDistanceDelta = CvCmdHandler.CvCmdMsg.disDelta;
		biped.yaw.set = CvCmdHandler.CvCmdMsg.yawSet;
		biped.fCvBrakeEnable = 1;
	}
	else
	{
		// avoid brake enabled by zero command
		*pDistanceDelta = 0;
		biped.fCvBrakeEnable = 0;
	}
#endif
}

/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          根据遥控器通道值，计算纵向和横移速度
 *
 * @param[out]     vx_set: 纵向速度指针
 * @param[out]     vy_set: 横向速度指针
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
 * @retval         none
 */
void chassis_rc_to_control_vector(chassis_move_t *chassis_move_rc_to_vector, fp32* pDistanceDelta)
{
	if (chassis_move_rc_to_vector == NULL)
	{
		return;
	}

	int16_t dis_channel_int16, yaw_channel_int16, l0_channel_int16, roll_channel_int16;
	fp32 dis_channel_fp32, yaw_channel_fp32, l0_channel_fp32, roll_channel_fp32;
	// deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
	// 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_RIGHT_VERTICAL_CHANNEL], dis_channel_int16, CHASSIS_RC_DEADLINE);
	// for biped, Y channel is for rotation
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_RIGHT_HORIZONTAL_CHANNEL], yaw_channel_int16, CHASSIS_RC_DEADLINE);
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_LEFT_VERTICAL_CHANNEL], l0_channel_int16, CHASSIS_RC_DEADLINE);
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL], roll_channel_int16, CHASSIS_RC_DEADLINE);

	dis_channel_fp32 = dis_channel_int16 * CHASSIS_DIS_RC_SEN_INC * biped_limitVelocity(NORMAL_MAX_CHASSIS_SPEED_X, biped.leg_simplified.L0.now);
	yaw_channel_fp32 = yaw_channel_int16 * -CHASSIS_YAW_RC_SEN_INC;
	l0_channel_fp32 = l0_channel_int16 * LEG_L0_RC_SEN_INC;
	roll_channel_fp32 = roll_channel_int16 * CHASSIS_ROLL_RC_SEN_INC;

	// keyboard set speed set-point
	// 键盘控制
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RISE_PLATFORM_KEY)
	{
		biped.leg_L.L0.set += LEG_L0_KEYBOARD_INC;
		biped.leg_R.L0.set += LEG_L0_KEYBOARD_INC;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LOWER_PLATFORM_KEY)
	{
		biped.leg_L.L0.set -= LEG_L0_KEYBOARD_INC;
		biped.leg_R.L0.set -= LEG_L0_KEYBOARD_INC;
	}

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEAN_LEFT_KEY)
	{
		biped.roll.set -= CHASSIS_ROLL_KEYBOARD_INC;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEAN_RIGHT_KEY)
	{
		biped.roll.set += CHASSIS_ROLL_KEYBOARD_INC;
	}

	*pDistanceDelta = dis_channel_fp32;
	biped.yaw.set += yaw_channel_fp32;
	biped.yaw.set = rad_format(biped.yaw.set);

	biped.leg_L.L0.set += l0_channel_fp32;
	biped.leg_R.L0.set += l0_channel_fp32;
	
	biped.roll.set += roll_channel_fp32;
}

static void chassis_rc_parse(chassis_move_t *chassis_move_control)
{

	if (chassis_move_control == NULL)
	{
		return;
	}

	chassis_behaviour_control_set(chassis_move_control);
}

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	switch (chassis_move_control_loop->chassis_mode)
	{
		case CHASSIS_VECTOR_RAW:
		{
			// safety guard
			biped.HipTorque_MaxLimit = 0.0f;
			biped.DriveTorque_MaxLimit = 0.0f;
			break;
		}
		case CHASSIS_VECTOR_NO_FOLLOW_YAW:
		case CHASSIS_VECTOR_CV_NO_FOLLOW_YAW:
		{
			// 1ms processing time
			inv_pendulum_ctrl();
			torque_ctrl();

			if (biped.leg_L.fResetMultiAngleOffset || biped.leg_R.fResetMultiAngleOffset)
			{
				biped.HipTorque_MaxLimit = 0;
				biped.DriveTorque_MaxLimit = 0;
			}
			else
			{
				biped.HipTorque_MaxLimit = HIP_TORQUE_MAX;
				biped.DriveTorque_MaxLimit = DRIVE_TORQUE_MAX;
			}
			break;
		}
		default:
		{
			// no delay so that it blocks LED flow
			break;
		}
	}

	// @TODO: power control according to ref
	// chassis_power_control(chassis_move_control_loop);
}
