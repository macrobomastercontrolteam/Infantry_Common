#include "robot_arm_task.h"
// #include "INS_task.h"
#include "cmsis_os.h"
#include "detect_task.h"

void robot_arm_init(void);
void robot_arm_status_update(void);
void robot_arm_control(void);
void robot_arm_state_transition(void);
uint8_t is_joint_target_reached(fp32 tol, uint8_t *notReachedJointPtr);
void robot_arm_assign_current_as_target(void);
float debug_threshold = 5.6;
const fp32 joint_angle_min[7] = {ARM_JOINT_0_ANGLE_MIN, ARM_JOINT_1_ANGLE_MIN, ARM_JOINT_2_ANGLE_MIN, ARM_JOINT_3_ANGLE_MIN, ARM_JOINT_4_ANGLE_MIN, ARM_JOINT_5_ANGLE_MIN, ARM_JOINT_6_ANGLE_MIN};
const fp32 joint_angle_max[7] = {ARM_JOINT_0_ANGLE_MAX, ARM_JOINT_1_ANGLE_MAX, ARM_JOINT_2_ANGLE_MAX, ARM_JOINT_3_ANGLE_MAX, ARM_JOINT_4_ANGLE_MAX, ARM_JOINT_5_ANGLE_MAX, ARM_JOINT_6_ANGLE_MAX};
const fp32 joint_angle_home[7] = {ARM_JOINT_0_ANGLE_HOME, ARM_JOINT_1_ANGLE_HOME, ARM_JOINT_2_ANGLE_HOME, ARM_JOINT_3_ANGLE_HOME, ARM_JOINT_4_ANGLE_HOME, ARM_JOINT_5_ANGLE_HOME, ARM_JOINT_6_ANGLE_HOME};
const fp32 arm_end_min[6] = {ARM_END_EFFECTOR_ROLL_MIN, ARM_END_EFFECTOR_PITCH_MIN, ARM_END_EFFECTOR_YAW_MIN, ARM_END_EFFECTOR_X_MIN, ARM_END_EFFECTOR_Y_MIN, ARM_END_EFFECTOR_Z_MIN};
const fp32 arm_end_max[6] = {ARM_END_EFFECTOR_ROLL_MAX, ARM_END_EFFECTOR_PITCH_MAX, ARM_END_EFFECTOR_YAW_MAX, ARM_END_EFFECTOR_X_MAX, ARM_END_EFFECTOR_Y_MAX, ARM_END_EFFECTOR_Z_MAX};
const fp32 arm_end_home[6] = {ARM_END_EFFECTOR_ROLL_HOME, ARM_END_EFFECTOR_PITCH_HOME, ARM_END_EFFECTOR_YAW_HOME, ARM_END_EFFECTOR_X_HOME, ARM_END_EFFECTOR_Y_HOME, ARM_END_EFFECTOR_Z_HOME};
const fp32 joint_angle_static[7] = {ARM_JOINT_0_ANGLE_STATIC, ARM_JOINT_1_ANGLE_STATIC, ARM_JOINT_2_ANGLE_STATIC, ARM_JOINT_3_ANGLE_STATIC, ARM_JOINT_4_ANGLE_STATIC, ARM_JOINT_5_ANGLE_STATIC, ARM_JOINT_6_ANGLE_STATIC};
const fp32 joint_angle_storage[7] = {ARM_JOINT_0_ANGLE_STORAGE, ARM_JOINT_1_ANGLE_STORAGE, ARM_JOINT_2_ANGLE_STORAGE, ARM_JOINT_3_ANGLE_STORAGE, ARM_JOINT_4_ANGLE_STORAGE, ARM_JOINT_5_ANGLE_STORAGE, ARM_JOINT_6_ANGLE_STORAGE};

// const fp32 yaw_offset = 0;
// const fp32 pitch_offset = 0;
// const fp32 roll_offset = 0;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t robot_arm_high_water;
#endif

robot_arm_t robot_arm;

#if ROBOT_ARM_JSCOPE_DEBUG
uint8_t isTargetReached = 0;
uint8_t notReachedJoint = 0;
static void jscope_robot_arm_test(void)
{
	isTargetReached = is_joint_target_reached(0.05f, &notReachedJoint);
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
		if ((prev_arm_state == ARM_STATE_ZERO_FORCE) || (robot_arm.arm_state == ARM_STATE_ZERO_FORCE))
		{
			CAN_cmd_switch_motor_power(prev_arm_state == ARM_STATE_ZERO_FORCE);
		}

		switch (robot_arm.arm_state)
		{
			case ARM_STATE_HOMING:
			{
				robot_arm_all_motors_return_home();
				break;
			}
			case ARM_STATE_STATIC:
            {
                robot_arm_return_static(0, JOINT_ID_LAST - 1);
                break;
            } 
			case ARM_STATE_STORAGE:
			{
				robot_arm_return_storage(0,JOINT_ID_LAST - 1);
			}  
			case ARM_STATE_ZERO_FORCE:
			case ARM_STATE_MOVE:
			default:
			{
				break;
			}
		}
		robot_arm.prevStateSwitchTime = robot_arm.time_ms;
		prev_arm_state = robot_arm.arm_state;
	}
}

void robot_arm_control(void)
{
	// const fp32 ARM_STATE_HOMING_HOLD_TIME_MS = 1000.0f;
	// const fp32 ARM_STATE_MOVE_HOLD_TIME_MS = 1000.0f;
	// const fp32 ARM_STATE_ZERO_FORCE_HOLD_TIME_MS = 1000.0f;
	// // Update state hold switch
	// uint32_t ulHoldTimePeriod;
	// switch (robot_arm.arm_state)
	// {
	// 	case ARM_STATE_HOMING:
	// 	{
	// 		ulHoldTimePeriod = ARM_STATE_HOMING_HOLD_TIME_MS;
	// 		break;
	// 	}
	// 	case ARM_STATE_MOVE:
	// 	{
	// 		ulHoldTimePeriod = ARM_STATE_MOVE_HOLD_TIME_MS;
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

	// safety guard
	if (robot_arm.fMasterSwitch && is_error_exist_in_range(JOINT_0_TOE, CHASSIS_CONTROLLER_TOE))
	{
		robot_arm.fdebug = 9;
		robot_arm.fMasterSwitch = 0;
	}

	switch (robot_arm.arm_state)
	{
		case ARM_STATE_MOVE:
		{
			if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			}
			else if (fIsStateHoldTimePassed && robot_arm.fHoming && (robot_arm.fStatic == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_HOMING;
			}
			else if (fIsStateHoldTimePassed && robot_arm.fStatic && (robot_arm.fHoming == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_STATIC;
			}
			else if (fIsStateHoldTimePassed && robot_arm.fStorage && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 0))
			{
				robot_arm.arm_state = ARM_STATE_STORAGE;
			}
			arm_joints_cmd_position(robot_arm.joint_angle_target, robot_arm.time_step_s);
			break;
		}
		case ARM_STATE_HOMING:
		{
			if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.fdebug = 7;
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			}
			else if (fIsStateHoldTimePassed && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_MOVE;
			}
			else if(fIsStateHoldTimePassed && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 1) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_STATIC;
				robot_arm.fStaticPrev = 1;
			}
			else if (fIsStateHoldTimePassed && is_joint_target_reached(DEG_TO_RAD(debug_threshold), NULL))
			{
				robot_arm.fdebug = 8;
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			}
			else if (fIsStateHoldTimePassed && robot_arm.fStorage && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 0))
			{
				robot_arm.arm_state = ARM_STATE_STORAGE;
			}
			arm_joints_cmd_position(robot_arm.joint_angle_target, robot_arm.time_step_s);
			break;
		}
		case ARM_STATE_STATIC:
        {
            if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			}
            else if (fIsStateHoldTimePassed && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_MOVE;
			}
			else if (fIsStateHoldTimePassed && robot_arm.fHoming && (robot_arm.fStatic == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_HOMING;
			}
			else if (fIsStateHoldTimePassed && robot_arm.fStorage && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 0))
			{
				robot_arm.arm_state = ARM_STATE_STORAGE;
			}                 
			//else if (fIsStateHoldTimePassed && is_joint_target_reached(DEG_TO_RAD(5.45f), NULL))
			//{
				//robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			//}
			arm_joints_cmd_position(robot_arm.joint_angle_target, robot_arm.time_step_s);
			break;
        }
		case ARM_STATE_STORAGE:
		{
			if (robot_arm.fMasterSwitch == 0)
			{
				robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			}
			else if (fIsStateHoldTimePassed && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_MOVE;
			}
			else if(fIsStateHoldTimePassed && (robot_arm.fHoming == 0) && (robot_arm.fStatic == 1) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_STATIC;
			}
			//else if (fIsStateHoldTimePassed && is_joint_target_reached(DEG_TO_RAD(15.45f), NULL))
			//{
				//robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
			//}
			else if (fIsStateHoldTimePassed && robot_arm.fHoming && (robot_arm.fStatic == 0) && (robot_arm.fStorage == 0))
			{
				robot_arm.arm_state = ARM_STATE_HOMING;
			}
			arm_joints_cmd_position(robot_arm.joint_angle_target, robot_arm.time_step_s);
			break;
		}
		case ARM_STATE_ZERO_FORCE:
		default:
		{
			fp32 all_0_torque[7] = {0, 0, 0, 0, 0, 0, 0};
			arm_joints_cmd_motors(all_0_torque);

			if (robot_arm.fMasterSwitch && fIsStateHoldTimePassed)
			{
				if (robot_arm.fHoming)
				{
					// if (is_joint_target_reached(DEG_TO_RAD(15.0f), NULL) == 0)
					// {
					// 	robot_arm.arm_state = ARM_STATE_HOMING;
					// }
				}
				else
				{
					robot_arm.arm_state = ARM_STATE_MOVE;
				}
			}
			break;
		}
	}
}

void robot_arm_status_update(void)
{
	robot_arm.time_step_s = (fp32)(osKernelSysTick() - robot_arm.time_ms) / 1000.0f;
	robot_arm.time_ms = osKernelSysTick();

	for (uint8_t i = 0; i < 7; i++)
	{
		motor_measure[i].velocity_manual = (motor_measure[i].output_angle - motor_measure[i].last_output_angle) / robot_arm.time_step_s;
		motor_measure[i].last_output_angle = motor_measure[i].output_angle;
	}

	// robot_arm.roll.now = *(robot_arm.arm_INS_angle + INS_ROLL_ADDRESS_OFFSET);
	// robot_arm.roll.now -= roll_offset;
	// robot_arm.roll.dot = *(robot_arm.arm_INS_speed + INS_GYRO_X_ADDRESS_OFFSET);
	// robot_arm.roll.last = robot_arm.roll.now;

	// robot_arm.pitch.now = *(robot_arm.arm_INS_angle + INS_PITCH_ADDRESS_OFFSET);
	// robot_arm.pitch.now -= pitch_offset;
	// robot_arm.pitch.dot = *(robot_arm.arm_INS_speed + INS_GYRO_Y_ADDRESS_OFFSET);
	// robot_arm.pitch.last = robot_arm.pitch.now;

	// robot_arm.yaw.now = *(robot_arm.arm_INS_angle + INS_YAW_ADDRESS_OFFSET);
	// robot_arm.yaw.now -= yaw_offset;
	// robot_arm.yaw.dot = *(robot_arm.arm_INS_speed + INS_GYRO_Z_ADDRESS_OFFSET);
	// robot_arm.yaw.last = robot_arm.yaw.now;

	// robot_arm.accel_x = *(robot_arm.arm_INS_accel + INS_ACCEL_X_ADDRESS_OFFSET);
	// robot_arm.accel_y = *(robot_arm.arm_INS_accel + INS_ACCEL_Y_ADDRESS_OFFSET);
	// robot_arm.accel_z = *(robot_arm.arm_INS_accel + INS_ACCEL_Z_ADDRESS_OFFSET);

	update_joint_6_6020_angle();
}

void robot_arm_init(void)
{
	const fp32 joint_angle_pid_coeffs[7][3] = {
		{JOINT_0_ANGLE_PID_KP, JOINT_0_ANGLE_PID_KI, JOINT_0_ANGLE_PID_KD},
		{JOINT_1_ANGLE_PID_KP, JOINT_1_ANGLE_PID_KI, JOINT_1_ANGLE_PID_KD},
		{JOINT_2_ANGLE_PID_KP, JOINT_2_ANGLE_PID_KI, JOINT_2_ANGLE_PID_KD},
		{JOINT_3_ANGLE_PID_KP, JOINT_3_ANGLE_PID_KI, JOINT_3_ANGLE_PID_KD},
		{JOINT_4_ANGLE_PID_KP, JOINT_4_ANGLE_PID_KI, JOINT_4_ANGLE_PID_KD},
		{JOINT_5_ANGLE_PID_KP, JOINT_5_ANGLE_PID_KI, JOINT_5_ANGLE_PID_KD},
		{JOINT_6_ANGLE_PID_KP, JOINT_6_ANGLE_PID_KI, JOINT_6_ANGLE_PID_KD}};
	const fp32 joint_angle_pid_max_out[7] = {JOINT_0_ANGLE_PID_MAX_OUT, JOINT_1_ANGLE_PID_MAX_OUT, JOINT_2_ANGLE_PID_MAX_OUT, JOINT_3_ANGLE_PID_MAX_OUT, JOINT_4_ANGLE_PID_MAX_OUT, JOINT_5_ANGLE_PID_MAX_OUT, JOINT_6_ANGLE_PID_MAX_OUT};
	const fp32 joint_angle_pid_max_iout[7] = {JOINT_0_ANGLE_PID_MAX_IOUT, JOINT_1_ANGLE_PID_MAX_IOUT, JOINT_2_ANGLE_PID_MAX_IOUT, JOINT_3_ANGLE_PID_MAX_IOUT, JOINT_4_ANGLE_PID_MAX_IOUT, JOINT_5_ANGLE_PID_MAX_IOUT, JOINT_6_ANGLE_PID_MAX_IOUT};

	const fp32 joint_speed_pid_coeffs[7][3] = {
		{JOINT_0_SPEED_PID_KP, JOINT_0_SPEED_PID_KI, JOINT_0_SPEED_PID_KD},
		{JOINT_1_SPEED_PID_KP, JOINT_1_SPEED_PID_KI, JOINT_1_SPEED_PID_KD},
		{JOINT_2_SPEED_PID_KP, JOINT_2_SPEED_PID_KI, JOINT_2_SPEED_PID_KD},
		{JOINT_3_SPEED_PID_KP, JOINT_3_SPEED_PID_KI, JOINT_3_SPEED_PID_KD},
		{JOINT_4_SPEED_PID_KP, JOINT_4_SPEED_PID_KI, JOINT_4_SPEED_PID_KD},
		{JOINT_5_SPEED_PID_KP, JOINT_5_SPEED_PID_KI, JOINT_5_SPEED_PID_KD},
		{JOINT_6_SPEED_PID_KP, JOINT_6_SPEED_PID_KI, JOINT_6_SPEED_PID_KD}};
	const fp32 joint_speed_pid_max_out[7] = {JOINT_0_SPEED_PID_MAX_OUT, JOINT_1_SPEED_PID_MAX_OUT, JOINT_2_SPEED_PID_MAX_OUT, JOINT_3_SPEED_PID_MAX_OUT, JOINT_4_SPEED_PID_MAX_OUT, JOINT_5_SPEED_PID_MAX_OUT, JOINT_6_SPEED_PID_MAX_OUT};
	const fp32 joint_speed_pid_max_iout[7] = {JOINT_0_SPEED_PID_MAX_IOUT, JOINT_1_SPEED_PID_MAX_IOUT, JOINT_2_SPEED_PID_MAX_IOUT, JOINT_3_SPEED_PID_MAX_IOUT, JOINT_4_SPEED_PID_MAX_IOUT, JOINT_5_SPEED_PID_MAX_IOUT, JOINT_6_SPEED_PID_MAX_IOUT};

	for (uint8_t i = 0; i < 7; i++)
	{
		PID_init(&robot_arm.joint_angle_pid[i], PID_POSITION, joint_angle_pid_coeffs[i], joint_angle_pid_max_out[i], joint_angle_pid_max_iout[i], 1.0f, &rad_err_handler);
		PID_init(&robot_arm.joint_speed_pid[i], PID_POSITION, joint_speed_pid_coeffs[i], joint_speed_pid_max_out[i], joint_speed_pid_max_iout[i], 0.6f, &filter_err_handler);
	}

	// robot_arm.arm_INS_angle = get_INS_angle_point();
	// robot_arm.arm_INS_speed = get_gyro_data_point();
	// robot_arm.arm_INS_accel = get_accel_data_point();
	robot_arm.arm_state = ARM_STATE_ZERO_FORCE;
	robot_arm.fHoming = 0;
	robot_arm.fStatic = 0;
	robot_arm.fStorage = 0;
	robot_arm.fStaticPrev = 0;
	robot_arm.fMasterSwitch = 0;
	robot_arm.fdebug = 0;
	robot_arm.prevStateSwitchTime = osKernelSysTick();

	robot_arm_all_motors_return_home();
	CAN_cmd_switch_motor_power(1);
}

void robot_arm_all_motors_return_home(void)
{
	robot_arm_motors_return_home(0, JOINT_ID_LAST - 1);
}

void robot_arm_motors_return_home(uint8_t _start, uint8_t _end)
{
	// inclusively from index _start to index _end
	if (_start < _end)
	{
		memcpy(&robot_arm.joint_angle_target[_start], &joint_angle_home[_start], (_end - _start + 1) * sizeof(joint_angle_home[0]));
	}
}

void robot_arm_return_static(uint8_t _start, uint8_t _end)
{
    memcpy(&robot_arm.joint_angle_target[_start], &joint_angle_static[_start], (_end - _start + 1) * sizeof(joint_angle_static[0]));
}

void robot_arm_return_storage(uint8_t _start, uint8_t _end)
{
	memcpy(&robot_arm.joint_angle_target[_start], &joint_angle_storage[_start], (_end - _start + 1) * sizeof(joint_angle_storage[0]));
}

void robot_arm_assign_current_as_target(void)
{
	uint8_t i;
	for (i = 0; i < sizeof(robot_arm.joint_angle_target) / sizeof(robot_arm.joint_angle_target[0]); i++)
	{
		robot_arm.joint_angle_target[i] = motor_measure[i].output_angle;
	}
}

void robot_arm_switch_on_power(void)
{
	if (robot_arm.fMasterSwitch == 0)
	{
		if (is_error_exist_in_range(JOINT_0_TOE, CHASSIS_CONTROLLER_TOE) == 0)
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
		if (fabs(rad_format(robot_arm.joint_angle_target[i] - motor_measure[i].output_angle)) > tol)
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
