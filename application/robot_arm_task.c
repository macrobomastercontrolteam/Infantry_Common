#include "robot_arm_task.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "detect_task.h"
#include "user_lib.h"

#define ROBOT_ARM_RC_TEST 0

#define ARM_STATE_MOVING_HOLD_TIME_MS 500
#define ARM_STATE_FIXED_HOLD_TIME_MS 500
#define ARM_STATE_ZERO_FORCE_HOLD_TIME_MS 500

void robot_arm_init(void);
void robot_arm_status_update(void);
void robot_arm_behaviour_set(void);
void robot_arm_control(void);
void robot_arm_state_transition(void);
uint8_t is_joint_target_reached(fp32 tol, uint8_t *notReachedJointPtr);
void robot_arm_assign_current_as_target(void);

const fp32 joint_angle_min[7] = {ARM_JOINT_0_ANGLE_MIN, ARM_JOINT_1_ANGLE_MIN, ARM_JOINT_2_ANGLE_MIN, ARM_JOINT_3_ANGLE_MIN, ARM_JOINT_4_ANGLE_MIN, ARM_JOINT_5_ANGLE_MIN, ARM_JOINT_6_ANGLE_MIN};
const fp32 joint_angle_max[7] = {ARM_JOINT_0_ANGLE_MAX, ARM_JOINT_1_ANGLE_MAX, ARM_JOINT_2_ANGLE_MAX, ARM_JOINT_3_ANGLE_MAX, ARM_JOINT_4_ANGLE_MAX, ARM_JOINT_5_ANGLE_MAX, ARM_JOINT_6_ANGLE_MAX};
const fp32 joint_angle_home[7] = {ARM_JOINT_0_ANGLE_HOME, ARM_JOINT_1_ANGLE_HOME, ARM_JOINT_2_ANGLE_HOME, ARM_JOINT_3_ANGLE_HOME, ARM_JOINT_4_ANGLE_HOME, ARM_JOINT_5_ANGLE_HOME, ARM_JOINT_6_ANGLE_HOME};
const fp32 arm_end_min[6] = {ARM_END_EFFECTOR_ROLL_MIN, ARM_END_EFFECTOR_PITCH_MIN, ARM_END_EFFECTOR_YAW_MIN, ARM_END_EFFECTOR_X_MIN, ARM_END_EFFECTOR_Y_MIN, ARM_END_EFFECTOR_Z_MIN};
const fp32 arm_end_max[6] = {ARM_END_EFFECTOR_ROLL_MAX, ARM_END_EFFECTOR_PITCH_MAX, ARM_END_EFFECTOR_YAW_MAX, ARM_END_EFFECTOR_X_MAX, ARM_END_EFFECTOR_Y_MAX, ARM_END_EFFECTOR_Z_MAX};
const fp32 arm_end_home[6] = {ARM_END_EFFECTOR_ROLL_HOME, ARM_END_EFFECTOR_PITCH_HOME, ARM_END_EFFECTOR_YAW_HOME, ARM_END_EFFECTOR_X_HOME, ARM_END_EFFECTOR_Y_HOME, ARM_END_EFFECTOR_Z_HOME};

// const fp32 yaw_offset = 0;
// const fp32 pitch_offset = 0;
// const fp32 roll_offset = 0;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t robot_arm_high_water;
#endif

robot_arm_t robot_arm;

#if ROBOT_ARM_JSCOPE_DEBUG
// uint8_t isTargetReached = 0;
uint8_t notReachedJoint = 0;
fp32 angle_diff[7];
static void jscope_robot_arm_test(void)
{
	// isTargetReached = is_joint_target_reached(0.05f, &notReachedJoint);
	for (uint8_t i = 0; i < 7; i++)
	{
		angle_diff[i] = RAD_TO_DEG(rad_format(robot_arm.joint_angle_target[i] - motor_measure[i].output_angle));
	}
}
#endif

void robot_arm_task(void const *pvParameters)
{
	uint32_t ulSystemTime = osKernelSysTick();
	osDelay(ROBOT_ARM_TASK_INIT_TIME);

	robot_arm_init();
	osDelay(ROBOT_ARM_TASK_INIT_TIME);
	robot_arm_status_update();
	// wait_until_motors_online();

	while (1)
	{
		robot_arm_status_update();
		robot_arm_behaviour_set();
		robot_arm_state_transition();
		robot_arm_control();
		osDelayUntil(&ulSystemTime, ROBOT_ARM_CONTROL_TIME_MS);

#if ROBOT_ARM_JSCOPE_DEBUG
		jscope_robot_arm_test();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
		robot_arm_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

void robot_arm_state_transition(void)
{
	static robot_arm_state_e prev_arm_state = ARM_STATE_ZERO_FORCE;
	if (prev_arm_state != robot_arm.arm_state)
	{
		if (prev_arm_state == ARM_STATE_ZERO_FORCE)
		{
			CAN_cmd_switch_motor_power(prev_arm_state == ARM_STATE_ZERO_FORCE);
			robot_arm_assign_current_as_target();
		}
		else if (robot_arm.arm_state == ARM_STATE_ZERO_FORCE)
		{
			CAN_cmd_switch_motor_power(prev_arm_state == ARM_STATE_ZERO_FORCE);
		}

		switch (robot_arm.arm_state)
		{
			case ARM_STATE_TEACHING:
			{
				// @TODO: teaching pid
				break;
			}
			case ARM_STATE_FIXED:
			{
				memcpy(robot_arm.joint_angle_real_arm, robot_arm.joint_angle_target, sizeof(robot_arm.joint_angle_target));
				break;
			}
			case ARM_STATE_SYNCING:
			{
				memcpy(robot_arm.joint_angle_target_before_sync, robot_arm.joint_angle_target, sizeof(robot_arm.joint_angle_target));
				break;
			}
			case ARM_STATE_ZERO_FORCE:
			default:
			{
				break;
			}
		}
		robot_arm.prevStateSwitchTime = robot_arm.time_ms;
		prev_arm_state = robot_arm.arm_state;
	}
}

void robot_arm_behaviour_set(void)
{
	if (toe_is_error(DBUS_TOE) == 0)
	{
		static int16_t iLastRightLeverCh = RC_SW_DOWN;
		int16_t iRightLeverCh = rc_ctrl.rc.s[RIGHT_LEVER_CHANNEL];
		switch (iRightLeverCh)
		{
			case RC_SW_UP:
			{
				if (iLastRightLeverCh != iRightLeverCh)
				{
					robot_arm.fTeaching = 1;
				}
				break;
			}
			case RC_SW_MID:
			{
				if (iLastRightLeverCh == RC_SW_DOWN)
				{
					robot_arm_switch_on_power();
				}
				robot_arm.fTeaching = 0;
				break;
			}
			case RC_SW_DOWN:
			default:
			{
				robot_arm.fMasterSwitch = 0;
				robot_arm.fTeaching = 0;
				break;
			}
		}
		iLastRightLeverCh = iRightLeverCh;

#if ROBOT_ARM_RC_TEST
		if (robot_arm.fMasterSwitch && (robot_arm.arm_state == ARM_STATE_FIXED))
		{
			fp32 right_horiz_channel, right_vert_channel, left_horiz_channel, left_vert_channel;
			deadband_limit(rc_ctrl.rc.ch[JOYSTICK_RIGHT_HORIZONTAL_CHANNEL], right_horiz_channel, RC_JOYSTICK_DEADLINE);
			deadband_limit(rc_ctrl.rc.ch[JOYSTICK_RIGHT_VERTICAL_CHANNEL], right_vert_channel, RC_JOYSTICK_DEADLINE);
			deadband_limit(rc_ctrl.rc.ch[JOYSTICK_LEFT_HORIZONTAL_CHANNEL], left_horiz_channel, RC_JOYSTICK_DEADLINE);
			deadband_limit(rc_ctrl.rc.ch[JOYSTICK_LEFT_VERTICAL_CHANNEL], left_vert_channel, RC_JOYSTICK_DEADLINE);

			switch (rc_ctrl.rc.ch[LEFT_LEVER_CHANNEL])
			{
				case RC_SW_UP:
				{
					// upper 4 joints control
					robot_arm.joint_angle_target[3] += right_vert_channel * ARM_JOINT_3_RC_SEN_INC;
					robot_arm.joint_angle_target[4] += right_horiz_channel * ARM_JOINT_4_RC_SEN_INC;
					robot_arm.joint_angle_target[5] += left_vert_channel * ARM_JOINT_5_RC_SEN_INC;
					robot_arm.joint_angle_target[6] += left_horiz_channel * ARM_JOINT_6_RC_SEN_INC;
					break;
				}
				case RC_SW_MID:
				{
					// lower 3 joints control
					robot_arm.joint_angle_target[0] += right_horiz_channel * ARM_JOINT_0_RC_SEN_INC;
					robot_arm.joint_angle_target[1] += right_vert_channel * ARM_JOINT_1_RC_SEN_INC;
					robot_arm.joint_angle_target[2] += left_horiz_channel * ARM_JOINT_2_RC_SEN_INC;
					break;
				}
				case RC_SW_DOWN:
				default:
				{
					break;
				}
			}

			for (uint8_t i = 0; i < 7; i++)
			{
				robot_arm.joint_angle_target[i] = fp32_constrain(robot_arm.joint_angle_target[i], joint_angle_min[i], joint_angle_max[i]);
			}
		}
#endif
	}
}

void robot_arm_control(void)
{
	// Update state hold switch
	// uint32_t ulHoldTimePeriod;
	// switch (robot_arm.arm_state)
	// {
	// 	case ARM_STATE_SYNCING:
	// 	{
	// 		ulHoldTimePeriod = ARM_STATE_SYNCING_HOLD_TIME_MS;
	// 		break;
	// 	}
	// 	case ARM_STATE_FIXED:
	// 	{
	// 		ulHoldTimePeriod = ARM_STATE_FIXED_HOLD_TIME_MS;
	// 		break;
	// 	}
	// 	case ARM_STATE_ZERO_FORCE:
	// 	default:
	// 	{
	// 		ulHoldTimePeriod = ARM_STATE_ZERO_FORCE_HOLD_TIME_MS;
	// 		break;
	// 	}
	// }
	// uint8_t fIsStateHoldTimePassed = (robot_arm.time_ms - robot_arm.prevStateSwitchTime > ulHoldTimePeriod);
	uint8_t fIsStateHoldTimePassed = 1;
	const fp32 ROBOT_ARM_SYNCING_TIME_MS = 1000.0f;

	// safety guard
	if (robot_arm.fMasterSwitch && is_error_exist_in_range(JOINT_0_TOE, JOINT_6_TOE))
	{
		robot_arm.fMasterSwitch = 0;
	}

	switch (robot_arm.arm_state)
	{
		case ARM_STATE_ZERO_FORCE:
		default:
		{
			if (fIsStateHoldTimePassed && robot_arm.fMasterSwitch)
			{
				robot_arm.arm_state = ARM_STATE_SYNCING;
			}
			else
			{
				const fp32 all_0_torque[7] = {0, 0, 0, 0, 0, 0, 0};
				arm_joints_cmd_torque(all_0_torque);
			}
			break;
		}
		case ARM_STATE_SYNCING:
		{
			if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
				robot_arm.fTeaching = 0;
				robot_arm_all_motors_return_home(robot_arm.joint_angle_target);
			}
			else if (robot_arm.fTeaching && fIsStateHoldTimePassed && is_joint_target_reached(0.05f, NULL))
			{
				robot_arm.arm_state = ARM_STATE_TEACHING;
			}
			else if (robot_arm.time_ms - robot_arm.prevStateSwitchTime <= ROBOT_ARM_SYNCING_TIME_MS)
			{
				// gradually change joint_angle_target towards joint_angle_real_arm
				for (uint8_t i = 0; i < 7; i++)
				{
					// rad_format(robot_arm.joint_angle_real_arm[i] - robot_arm.joint_angle_target_before_sync[i]) makes motor move in the shortest path
					robot_arm.joint_angle_target[i] = rad_format(robot_arm.joint_angle_target_before_sync[i] + rad_format(robot_arm.joint_angle_real_arm[i] - robot_arm.joint_angle_target_before_sync[i]) * (robot_arm.time_ms - robot_arm.prevStateSwitchTime) / ROBOT_ARM_SYNCING_TIME_MS);
				}
			}
			arm_joints_cmd_position(robot_arm.joint_angle_target, ROBOT_ARM_CONTROL_TIME_S, 0);
			break;
		}
		case ARM_STATE_FIXED:
		{
			if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
				robot_arm.fTeaching = 0;
				robot_arm_all_motors_return_home(robot_arm.joint_angle_target);
			}
			else if (robot_arm.fTeaching && fIsStateHoldTimePassed && is_joint_target_reached(0.05f, NULL))
			{
				robot_arm.arm_state = ARM_STATE_TEACHING;
			}
			else if ((is_joint_target_reached(DEG_TO_RAD(10.0f), NULL) == 0))
			{
				// fall back to gradual syncing mode to avoid sudden movement
				robot_arm.arm_state = ARM_STATE_SYNCING;
			}
			else
			{
				arm_joints_cmd_position(robot_arm.joint_angle_target, ROBOT_ARM_CONTROL_TIME_S, 0);
			}
			break;
		}
		case ARM_STATE_TEACHING:
		{
			if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
				robot_arm.fTeaching = 0;
				robot_arm_all_motors_return_home(robot_arm.joint_angle_target);
			}
			else if (robot_arm.fTeaching == 0)
			{
				robot_arm.arm_state = ARM_STATE_FIXED;
			}
			else
			{
				arm_joints_cmd_position(robot_arm.joint_angle_target, ROBOT_ARM_CONTROL_TIME_S, 1);
			}
			break;
		}
	}
}

void robot_arm_status_update(void)
{
	robot_arm.time_ms = osKernelSysTick();
}

void robot_arm_init(void)
{
	const static fp32 joint_0_angle_pid_coeffs[3] = {JOINT_0_ANGLE_PID_KP, JOINT_0_ANGLE_PID_KI, JOINT_0_ANGLE_PID_KD};
	const static fp32 joint_1_angle_pid_coeffs[3] = {JOINT_1_ANGLE_PID_KP, JOINT_1_ANGLE_PID_KI, JOINT_1_ANGLE_PID_KD};
	const static fp32 joint_2_angle_pid_coeffs[3] = {JOINT_2_ANGLE_PID_KP, JOINT_2_ANGLE_PID_KI, JOINT_2_ANGLE_PID_KD};
	const static fp32 joint_3_angle_pid_coeffs[3] = {JOINT_3_ANGLE_PID_KP, JOINT_3_ANGLE_PID_KI, JOINT_3_ANGLE_PID_KD};
	const static fp32 joint_4_angle_pid_coeffs[3] = {JOINT_4_ANGLE_PID_KP, JOINT_4_ANGLE_PID_KI, JOINT_4_ANGLE_PID_KD};
	const static fp32 joint_5_angle_pid_coeffs[3] = {JOINT_5_ANGLE_PID_KP, JOINT_5_ANGLE_PID_KI, JOINT_5_ANGLE_PID_KD};
	const static fp32 joint_6_angle_pid_coeffs[3] = {JOINT_6_ANGLE_PID_KP, JOINT_6_ANGLE_PID_KI, JOINT_6_ANGLE_PID_KD};

	PID_init(&robot_arm.joint_angle_pid[0], PID_POSITION, joint_0_angle_pid_coeffs, JOINT_0_ANGLE_PID_MAX_OUT, JOINT_0_ANGLE_PID_MAX_IOUT, 0.8f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_pid[1], PID_POSITION, joint_1_angle_pid_coeffs, JOINT_1_ANGLE_PID_MAX_OUT, JOINT_1_ANGLE_PID_MAX_IOUT, 0.8f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_pid[2], PID_POSITION, joint_2_angle_pid_coeffs, JOINT_2_ANGLE_PID_MAX_OUT, JOINT_2_ANGLE_PID_MAX_IOUT, 0.8f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_pid[3], PID_POSITION, joint_3_angle_pid_coeffs, JOINT_3_ANGLE_PID_MAX_OUT, JOINT_3_ANGLE_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_pid[4], PID_POSITION, joint_4_angle_pid_coeffs, JOINT_4_ANGLE_PID_MAX_OUT, JOINT_4_ANGLE_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_pid[5], PID_POSITION, joint_5_angle_pid_coeffs, JOINT_5_ANGLE_PID_MAX_OUT, JOINT_5_ANGLE_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_pid[6], PID_POSITION, joint_6_angle_pid_coeffs, JOINT_6_ANGLE_PID_MAX_OUT, JOINT_6_ANGLE_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);

	const static fp32 joint_0_angle_teach_pid_coeffs[3] = {JOINT_0_ANGLE_TEACH_PID_KP, JOINT_0_ANGLE_TEACH_PID_KI, JOINT_0_ANGLE_TEACH_PID_KD};
	const static fp32 joint_1_angle_teach_pid_coeffs[3] = {JOINT_1_ANGLE_TEACH_PID_KP, JOINT_1_ANGLE_TEACH_PID_KI, JOINT_1_ANGLE_TEACH_PID_KD};
	const static fp32 joint_2_angle_teach_pid_coeffs[3] = {JOINT_2_ANGLE_TEACH_PID_KP, JOINT_2_ANGLE_TEACH_PID_KI, JOINT_2_ANGLE_TEACH_PID_KD};
	const static fp32 joint_3_angle_teach_pid_coeffs[3] = {JOINT_3_ANGLE_TEACH_PID_KP, JOINT_3_ANGLE_TEACH_PID_KI, JOINT_3_ANGLE_TEACH_PID_KD};
	const static fp32 joint_4_angle_teach_pid_coeffs[3] = {JOINT_4_ANGLE_TEACH_PID_KP, JOINT_4_ANGLE_TEACH_PID_KI, JOINT_4_ANGLE_TEACH_PID_KD};
	const static fp32 joint_5_angle_teach_pid_coeffs[3] = {JOINT_5_ANGLE_TEACH_PID_KP, JOINT_5_ANGLE_TEACH_PID_KI, JOINT_5_ANGLE_TEACH_PID_KD};
	const static fp32 joint_6_angle_teach_pid_coeffs[3] = {JOINT_6_ANGLE_TEACH_PID_KP, JOINT_6_ANGLE_TEACH_PID_KI, JOINT_6_ANGLE_TEACH_PID_KD};

	PID_init(&robot_arm.joint_angle_teach_pid[0], PID_POSITION, joint_0_angle_teach_pid_coeffs, JOINT_0_ANGLE_TEACH_PID_MAX_OUT, JOINT_0_ANGLE_TEACH_PID_MAX_IOUT, 0.8f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_teach_pid[1], PID_POSITION, joint_1_angle_teach_pid_coeffs, JOINT_1_ANGLE_TEACH_PID_MAX_OUT, JOINT_1_ANGLE_TEACH_PID_MAX_IOUT, 0.8f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_teach_pid[2], PID_POSITION, joint_2_angle_teach_pid_coeffs, JOINT_2_ANGLE_TEACH_PID_MAX_OUT, JOINT_2_ANGLE_TEACH_PID_MAX_IOUT, 0.8f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_teach_pid[3], PID_POSITION, joint_3_angle_teach_pid_coeffs, JOINT_3_ANGLE_TEACH_PID_MAX_OUT, JOINT_3_ANGLE_TEACH_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_teach_pid[4], PID_POSITION, joint_4_angle_teach_pid_coeffs, JOINT_4_ANGLE_TEACH_PID_MAX_OUT, JOINT_4_ANGLE_TEACH_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_teach_pid[5], PID_POSITION, joint_5_angle_teach_pid_coeffs, JOINT_5_ANGLE_TEACH_PID_MAX_OUT, JOINT_5_ANGLE_TEACH_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);
	PID_init(&robot_arm.joint_angle_teach_pid[6], PID_POSITION, joint_6_angle_teach_pid_coeffs, JOINT_6_ANGLE_TEACH_PID_MAX_OUT, JOINT_6_ANGLE_TEACH_PID_MAX_IOUT, 1.0f, &filter_rad_err_handler);

	robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
	robot_arm.fTeaching = 0;
	robot_arm.fMasterSwitch = 0;
	robot_arm.prevStateSwitchTime = osKernelSysTick();

	robot_arm_all_motors_return_home(robot_arm.joint_angle_target);
	robot_arm_all_motors_return_home(robot_arm.joint_angle_real_arm);
	CAN_cmd_switch_motor_power(1);
}

void robot_arm_all_motors_return_home(fp32 arm_angles[7])
{
	robot_arm_motors_return_home(arm_angles, 0, JOINT_ID_LAST - 1);
}

void robot_arm_motors_return_home(fp32 arm_angles[7], uint8_t _start, uint8_t _end)
{
	// inclusively from index _start to index _end
	if (_start < _end)
	{
		memcpy(&arm_angles[_start], &joint_angle_home[_start], (_end - _start + 1) * sizeof(joint_angle_home[0]));
	}
}

void robot_arm_assign_current_as_target(void)
{
	for (uint8_t i = 0; i < sizeof(robot_arm.joint_angle_target) / sizeof(robot_arm.joint_angle_target[0]); i++)
	{
		robot_arm.joint_angle_target[i] = motor_measure[i].output_angle;
	}
}

void robot_arm_switch_on_power(void)
{
	if (robot_arm.fMasterSwitch == 0)
	{
		if (is_error_exist_in_range(JOINT_0_TOE, JOINT_6_TOE) == 0)
		{
			robot_arm.fMasterSwitch = 1;
		}
	}
}

uint8_t is_joint_target_reached(fp32 tol, uint8_t *notReachedJointPtr)
{
	uint8_t fTargetReached = 1;
	for (uint8_t i = 0; i < sizeof(robot_arm.joint_angle_target) / sizeof(robot_arm.joint_angle_target[0]); i++)
	{
		if (fabs(robot_arm.joint_angle_target[i] - motor_measure[i].output_angle) > tol)
		{
			fTargetReached = 0;
			if (notReachedJointPtr != NULL)
			{
				*notReachedJointPtr = i;
			}
			break;
		}
	}

	if (fTargetReached && (notReachedJointPtr != NULL))
	{
		*notReachedJointPtr = 0xFF;
	}
	return fTargetReached;
}
