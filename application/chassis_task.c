/**
    ****************************(C) COPYRIGHT 2019 DJI****************************
    * @file       chassis.c/h
    * @brief      chassis control task,
    *             ���̿�������
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

#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"

#include "user_lib.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"

void wait_until_all_necessary_modules_online(void);
/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @param[out]     chassis_move_init: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @param[out]     chassis_move_init:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
 * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
 * @param[out]     chassis_move_mode: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);

/**
 * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
 * @param[out]     chassis_move_transit: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
 * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
 * @retval         none
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 *
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_control(chassis_move_t *chassis_move_control);
/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void relay_signal_manager(void);
void robot_arm_control(void);
void CAN_cmd_robot_arm(void);
void vtm_gimbal_control(void);
void set_to_home(void);
void set_static_mode(void);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

const fp32 joint_angle_min[7] = {ARM_JOINT_0_ANGLE_MIN, ARM_JOINT_1_ANGLE_MIN, ARM_JOINT_2_ANGLE_MIN, ARM_JOINT_3_ANGLE_MIN, ARM_JOINT_4_ANGLE_MIN, ARM_JOINT_5_ANGLE_MIN, ARM_JOINT_6_ANGLE_MIN};
const fp32 joint_angle_max[7] = {ARM_JOINT_0_ANGLE_MAX, ARM_JOINT_1_ANGLE_MAX, ARM_JOINT_2_ANGLE_MAX, ARM_JOINT_3_ANGLE_MAX, ARM_JOINT_4_ANGLE_MAX, ARM_JOINT_5_ANGLE_MAX, ARM_JOINT_6_ANGLE_MAX};
const fp32 joint_angle_home[7] = {ARM_JOINT_0_ANGLE_HOME, ARM_JOINT_1_ANGLE_HOME, ARM_JOINT_2_ANGLE_HOME, ARM_JOINT_3_ANGLE_HOME, ARM_JOINT_4_ANGLE_HOME, ARM_JOINT_5_ANGLE_HOME, ARM_JOINT_6_ANGLE_HOME};
const fp32 arm_end_min[6] = {ARM_END_EFFECTOR_ROLL_MIN, ARM_END_EFFECTOR_PITCH_MIN, ARM_END_EFFECTOR_YAW_MIN, ARM_END_EFFECTOR_X_MIN, ARM_END_EFFECTOR_Y_MIN, ARM_END_EFFECTOR_Z_MIN};
const fp32 joint_angle_static[7] = {ARM_JOINT_0_ANGLE_STATIC, ARM_JOINT_1_ANGLE_STATIC, ARM_JOINT_2_ANGLE_STATIC, ARM_JOINT_3_ANGLE_STATIC, ARM_JOINT_4_ANGLE_STATIC, ARM_JOINT_5_ANGLE_STATIC, ARM_JOINT_6_ANGLE_STATIC};
const fp32 arm_end_max[6] = {ARM_END_EFFECTOR_ROLL_MAX, ARM_END_EFFECTOR_PITCH_MAX, ARM_END_EFFECTOR_YAW_MAX, ARM_END_EFFECTOR_X_MAX, ARM_END_EFFECTOR_Y_MAX, ARM_END_EFFECTOR_Z_MAX};
const fp32 arm_end_home[6] = {ARM_END_EFFECTOR_ROLL_HOME, ARM_END_EFFECTOR_PITCH_HOME, ARM_END_EFFECTOR_YAW_HOME, ARM_END_EFFECTOR_X_HOME, ARM_END_EFFECTOR_Y_HOME, ARM_END_EFFECTOR_Z_HOME};

//�����˶�����
chassis_move_t chassis_move;
vtm_gimbal_t vtm_gimbal;

#if CHASSIS_TEST_MODE
static void J_scope_chassis_test(void)
{
	;
}
#endif

/**
 * @brief          chassis task, osDelay CHASSIS_CONTROL_TIME_MS (2ms)
 * @param[in]      pvParameters: null
 * @retval         none
 */
/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
	uint32_t ulSystemTime = osKernelSysTick();
	osDelay(CHASSIS_TASK_INIT_TIME);
	chassis_init(&chassis_move);
	wait_until_all_necessary_modules_online();

	while (1)
	{
		chassis_set_mode(&chassis_move);
		chassis_mode_change_control_transit(&chassis_move);
		chassis_feedback_update(&chassis_move);
		chassis_set_control(&chassis_move);
		chassis_control_loop(&chassis_move);

		set_to_home();
		robot_arm_control();
		vtm_gimbal_control();
		relay_signal_manager();

		CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current, chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		osDelay(1);
		CAN_cmd_vtm_gimbal(vtm_gimbal.yaw_current_cmd, vtm_gimbal.pitch_current_cmd);
		CAN_cmd_robot_arm();

		osDelayUntil(&ulSystemTime, CHASSIS_CONTROL_TIME_MS);

#if CHASSIS_TEST_MODE
		J_scope_chassis_test();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

fp32 vtm_pitch_error_dot_filter_coeff = 0.75f;
fp32 vtm_yaw_error_dot_filter_coeff = 0.6f;
void vtm_gimbal_control(void)
{
	static uint8_t fLastVtmGimbalPowerEnabled = 0;
	vtm_gimbal.fVtmGimbalPowerEnabled = (chassis_behaviour_mode != CHASSIS_ZERO_FORCE);
	// edge detection
	if (fLastVtmGimbalPowerEnabled != vtm_gimbal.fVtmGimbalPowerEnabled)
	{
		if (vtm_gimbal.fVtmGimbalPowerEnabled)
		{
			vtm_gimbal.yaw_target_ecd = M3508_loop_ecd_constrain(motor_measure[MOTOR_INDEX_VTM_YAW].ecd);
			vtm_gimbal.pitch_target_ecd = M3508_loop_ecd_constrain(motor_measure[MOTOR_INDEX_VTM_PITCH].ecd);
			PID_clear(&vtm_gimbal.yaw_ecd_pid);
			PID_clear(&vtm_gimbal.pitch_ecd_pid);
		}
		fLastVtmGimbalPowerEnabled = vtm_gimbal.fVtmGimbalPowerEnabled;
	}

	if (vtm_gimbal.fVtmGimbalPowerEnabled)
	{
		if ((chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) == 0)
		{
			if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
			{
				vtm_gimbal.yaw_target_ecd = M3508_loop_ecd_constrain(vtm_gimbal.yaw_target_ecd - VTM_YAW_ECD_KEYBOARD_SEN_INC);
			}
			else if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)
			{
				vtm_gimbal.yaw_target_ecd = M3508_loop_ecd_constrain(vtm_gimbal.yaw_target_ecd + VTM_YAW_ECD_KEYBOARD_SEN_INC);
			}

			if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
			{
				vtm_gimbal.pitch_target_ecd = M3508_loop_ecd_constrain(vtm_gimbal.pitch_target_ecd - VTM_PITCH_ECD_KEYBOARD_SEN_INC);
			}
			else if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
			{
				vtm_gimbal.pitch_target_ecd = M3508_loop_ecd_constrain(vtm_gimbal.pitch_target_ecd + VTM_PITCH_ECD_KEYBOARD_SEN_INC);
			}
		}
		vtm_gimbal.yaw_target_ecd = fp32_constrain(vtm_gimbal.yaw_target_ecd, -HALF_ECD_RANGE, HALF_ECD_RANGE);
		vtm_gimbal.pitch_target_ecd = fp32_constrain(vtm_gimbal.pitch_target_ecd, -HALF_ECD_RANGE, HALF_ECD_RANGE);
		vtm_gimbal.yaw_current_cmd = PID_calc_with_dot_filter(&vtm_gimbal.yaw_ecd_pid, motor_measure[MOTOR_INDEX_VTM_YAW].ecd, vtm_gimbal.yaw_target_ecd, vtm_yaw_error_dot_filter_coeff);
		vtm_gimbal.pitch_current_cmd = PID_calc_with_dot_filter(&vtm_gimbal.pitch_ecd_pid, motor_measure[MOTOR_INDEX_VTM_PITCH].ecd, vtm_gimbal.pitch_target_ecd, vtm_pitch_error_dot_filter_coeff);
	}
}

void set_static_mode(void)
{
    if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_R)
    {
        chassis_move.robot_arm_mode = ROBOT_ARM_STATIC;
    }
}

void set_to_home(void)
{
	if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_X)
	{
		chassis_move.robot_arm_mode = ROBOT_ARM_HOME;
	}
}


void relay_signal_manager(void)
{
	if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL)
	{
		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_B)
		{
			head_pump_control(1);
		}
		else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_V)
		{
			head_pump_control(0);
		}

		if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_G)
		{
			storage_pump_control(1);
		}
		else if (chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_F)
		{
			storage_pump_control(0);
		}
	}
}

void robot_arm_set_home(void)
{
#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
	for (uint8_t i = 0; i < 7; i++)
	{
		chassis_move.robot_arm_motor_pos[i] = joint_angle_home[i];
	}
#else
	for (uint8_t i = 0; i < 6; i++)
	{
		chassis_move.end_effector_cmd.setpoints[i] = arm_end_home[i];
	}
#endif
	chassis_move.fHoming = 1;
}

void robot_arm_set_static(void)
{
#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
	for (uint8_t i = 0; i < 7; i++)
	{
		chassis_move.robot_arm_motor_pos[i] = joint_angle_static[i];
	}
#endif
	chassis_move.fStatic = 1;
}

void robot_arm_control(void)
{
#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
	switch (chassis_move.robot_arm_mode)
	{
		case ROBOT_ARM_HOME:
		{
			robot_arm_set_home();
			break;
		}
		case ROBOT_ARM_STATIC:
		{
			robot_arm_set_static();
			break;
		}
		case ROBOT_ARM_CHANGEABLE:
		{
			fp32 right_horiz_channel, right_vert_channel, left_horiz_channel, left_vert_channel;
			deadband_limit(chassis_move.chassis_RC->rc.ch[JOYSTICK_RIGHT_HORIZONTAL_CHANNEL], right_horiz_channel, CHASSIS_RC_DEADLINE);
			deadband_limit(chassis_move.chassis_RC->rc.ch[JOYSTICK_RIGHT_VERTICAL_CHANNEL], right_vert_channel, CHASSIS_RC_DEADLINE);
			deadband_limit(chassis_move.chassis_RC->rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL], left_horiz_channel, CHASSIS_RC_DEADLINE);
			deadband_limit(chassis_move.chassis_RC->rc.ch[JOYSTICK_LEFT_VERTICAL_CHANNEL], left_vert_channel, CHASSIS_RC_DEADLINE);

			if (chassis_move.fHoming && (right_horiz_channel || right_vert_channel || left_horiz_channel || left_vert_channel))
			{
				chassis_move.fHoming = 0;
			}
			if (chassis_move.fStatic && (right_horiz_channel || right_vert_channel || left_horiz_channel || left_vert_channel))
			{
				chassis_move.fStatic = 0;
			}

			switch (chassis_move.chassis_RC->rc.s[RIGHT_LEVER_CHANNEL])
			{
				case RC_SW_UP:
				{
					// upper 4 joints control
					chassis_move.robot_arm_motor_pos[3] += right_vert_channel * ARM_JOINT_3_RC_SEN_INC;
					chassis_move.robot_arm_motor_pos[4] += right_horiz_channel * ARM_JOINT_4_RC_SEN_INC;
					chassis_move.robot_arm_motor_pos[5] += left_vert_channel * ARM_JOINT_5_RC_SEN_INC;
					chassis_move.robot_arm_motor_pos[6] += left_horiz_channel * ARM_JOINT_6_RC_SEN_INC;
					break;
				}
				case RC_SW_MID:
				{
					// lower 3 joints control
					chassis_move.robot_arm_motor_pos[0] += right_horiz_channel * ARM_JOINT_0_RC_SEN_INC;
					chassis_move.robot_arm_motor_pos[1] += right_vert_channel * ARM_JOINT_1_RC_SEN_INC;
					chassis_move.robot_arm_motor_pos[2] += left_horiz_channel * ARM_JOINT_2_RC_SEN_INC;
					break;
				}
				default:
				{
					// should not reach here
					break;
				}
			}

			for (uint8_t i = 0; i < 7; i++)
			{
				chassis_move.robot_arm_motor_pos[i] = fp32_constrain(chassis_move.robot_arm_motor_pos[i], joint_angle_min[i], joint_angle_max[i]);
			}
			break;
		}
		case ROBOT_ARM_FIXED:
		case ROBOT_ARM_ZERO_FORCE:
		default:
		{
			break;
		}
	}
#else /* INDIVIDUAL_MOTOR_TEST == 0 */
	switch (chassis_move.robot_arm_mode)
	{
		case ROBOT_ARM_HOME:
		{
			robot_arm_set_home();
			break;
		}
		case ROBOT_ARM_CHANGEABLE:
		{
			// @TODO: end effector mode
			for (uint8_t i = 0; i < 6; i++)
			{
				chassis_move.end_effector_cmd.setpoints[i] = fp32_constrain(chassis_move.end_effector_cmd.setpoints[i], arm_end_min[i], arm_end_max[i]);
			}
			break;
		}
		case ROBOT_ARM_ZERO_FORCE:
		case ROBOT_ARM_FIXED:
		default:
		{
			break;
		}
	}
#endif
}

void wait_until_all_necessary_modules_online(void)
{
	uint8_t fIsError = 1;
	while (fIsError)
	{
		fIsError = is_error_exist_in_range(DBUS_TOE, CHASSIS_MOTOR4_TOE);
		osDelay(2 * CHASSIS_CONTROL_TIME_MS);
	}
}

/**
 * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
 *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
 * @param[out]     chassis_move_init: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @param[out]     chassis_move_init:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
	if (chassis_move_init == NULL)
	{
		return;
	}

	//chassis drive motor (3508) speed PID
	//�����������ٶȻ�pidֵ
	const static fp32 motor_speed_pid[3] = {M3508_MOTOR_SPEED_PID_KP, M3508_MOTOR_SPEED_PID_KI, M3508_MOTOR_SPEED_PID_KD};

	//chassis angle PID
	//���̽Ƕ�pidֵ
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	uint8_t i;

	//in beginning�� chassis mode is raw
	//���̿���״̬Ϊԭʼ
	chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
	//get remote control point
	//��ȡң����ָ��
	chassis_move_init->chassis_RC = get_remote_control_point();
	//get gyro sensor euler angle point
	//��ȡ��������̬��ָ��
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();

	//get chassis motor data point,  initialize motor speed PID
	//��ȡ���̵������ָ�룬��ʼ��PID
	for (i = 0; i < 4; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
		PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3508_MOTOR_SPEED_PID_MAX_OUT, M3508_MOTOR_SPEED_PID_MAX_IOUT, &raw_err_handler);
	}
	//initialize angle PID
	//��ʼ���Ƕ�PID
	PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT, &rad_err_handler);

	//first order low-pass filter  replace ramp function
	//��һ���˲�����б����������
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	//max and min speed
	//��� ��С�ٶ�
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

	chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	chassis_move_init->robot_arm_mode = ROBOT_ARM_ZERO_FORCE;
	chassis_move_init->fHoming = 0;
	chassis_move_init->fStatic = 0;
	robot_arm_set_home();

	// init vtm gimbal params
	const static fp32 vtm_pitch_ecd_pid_coeffs[3] = {M3508_VTM_PITCH_ECD_PID_KP, M3508_VTM_PITCH_ECD_PID_KI, M3508_VTM_PITCH_ECD_PID_KD};
	const static fp32 vtm_yaw_ecd_pid_coeffs[3] = {M3508_VTM_YAW_ECD_PID_KP, M3508_VTM_YAW_ECD_PID_KI, M3508_VTM_YAW_ECD_PID_KD};
	PID_init(&vtm_gimbal.pitch_ecd_pid, PID_POSITION, vtm_pitch_ecd_pid_coeffs, M3508_VTM_PITCH_ECD_PID_MAX_OUT, M3508_VTM_PITCH_ECD_PID_MAX_IOUT, &M3508_ecd_err_handler);
	PID_init(&vtm_gimbal.yaw_ecd_pid, PID_POSITION, vtm_yaw_ecd_pid_coeffs, M3508_VTM_YAW_ECD_PID_MAX_OUT, M3508_VTM_YAW_ECD_PID_MAX_IOUT, &M3508_ecd_err_handler);

	vtm_gimbal.fVtmGimbalPowerEnabled = 0;
	vtm_gimbal.pitch_target_ecd = 0;
	vtm_gimbal.yaw_target_ecd = 0;
	vtm_gimbal.pitch_current_cmd = 0;
	vtm_gimbal.yaw_current_cmd = 0;

	//update data
	//����һ������
	chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
 * @param[out]     chassis_move_mode: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
	if (chassis_move_mode == NULL)
	{
		return;
	}
	//in file "chassis_behaviour.c"
	chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
 * @param[out]     chassis_move_transit: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
 * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
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
			{
				chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
				break;
			}
			default:
			{
				break;
			}
		}
		chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
	}
}

/**
 * @brief          chassis some measure data updata, such as motor speed, euler angle�� robot speed
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	if (chassis_move_update == NULL)
	{
		return;
	}

	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		//update motor speed, accel is differential of speed PID
		//���µ���ٶȣ����ٶ����ٶȵ�PID΢��
		chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
		chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

	//calculate vertical speed, horizontal speed ,rotation speed, left hand rule
	//���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
	chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

	chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
	chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
	chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
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
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 *
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}

	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	//deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
	vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

	//keyboard set speed set-point
	//���̿���
	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	{
		vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	}

	if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
	}
	else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
	{
		vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
	}

	//first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
	//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
	//stop command, need not slow change, set zero derectly
	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_control(chassis_move_t *chassis_move_control)
{

	if (chassis_move_control == NULL)
	{
		return;
	}


	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	//get three control set-point, ��ȡ������������ֵ
	chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

	if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
	{
		//"angle_set" is rotation speed set-point
		// ��angle_set�� ����ת�ٶȿ���
		chassis_move_control->wz_set = angle_set;
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	}
	else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
	{
		// in raw mode, set-point is sent to CAN bus
		// ��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
		chassis_move_control->wz_set = angle_set;
		chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
		chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
	}
}

/**
 * @brief          four mecanum wheels speed is calculated by three param.
 * @param[in]      vx_set: vertial speed
 * @param[in]      vy_set: horizontal speed
 * @param[in]      wz_set: rotation speed
 * @param[out]     wheel_speed: four mecanum wheels speed
 * @retval         none
 */
/**
 * @brief          �ĸ������ٶ���ͨ�������������������
 * @param[in]      vx_set: �����ٶ�
 * @param[in]      vy_set: �����ٶ�
 * @param[in]      wz_set: ��ת�ٶ�
 * @param[out]     wheel_speed: �ĸ������ٶ�
 * @retval         none
 */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	//because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
	//��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
	wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // unit m/s
	uint8_t i = 0;

	//mecanum wheel speed calculation
	chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
	                                      chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

	if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
	{
		for (i = 0; i < 4; i++)
		{
			chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
		}
	}
	else
	{
		for (i = 0; i < 4; i++)
		{
			// calculate the max speed in four wheels, limit the max speed
			// �������ӿ�������ٶȣ�������������ٶ�
			chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
			temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
			if (max_vector < temp)
			{
				max_vector = temp;
			}
		}

		if (max_vector > MAX_WHEEL_SPEED)
		{
			vector_rate = MAX_WHEEL_SPEED / max_vector;
			for (i = 0; i < 4; i++)
			{
				chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
			}
		}

		// calculate pid
		// ����pid
		for (i = 0; i < 4; i++)
		{
			PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
		}

		// ��ֵ����ֵ
		for (i = 0; i < 4; i++)
		{
			chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
		}
	}
}

void CAN_cmd_robot_arm(void)
{
#if (ENGINEER_CONTROL_MODE == INDIVIDUAL_MOTOR_TEST)
	CAN_cmd_robot_arm_by_q(chassis_move.robot_arm_motor_pos, chassis_move.robot_arm_mode, chassis_move.fHoming, chassis_move.fStatic);
#else  /* INDIVIDUAL_MOTOR_TEST */
	CAN_cmd_robot_arm_by_end_effector(chassis_move.end_effector_cmd, chassis_move.robot_arm_mode, chassis_move.fHoming);
#endif /* INDIVIDUAL_MOTOR_TEST */
}
