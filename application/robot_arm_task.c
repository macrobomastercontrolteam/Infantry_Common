#include "robot_arm_task.h"
#include "INS_task.h"
#include "cmsis_os.h"

/*********** Robot Configs Start ***********/
// @TODO
#define DISABLE_JOINT_0_POWER 0
#define DISABLE_JOINT_1_POWER 0
#define DISABLE_JOINT_2_POWER 0
#define DISABLE_JOINT_3_POWER 0
#define DISABLE_JOINT_4_POWER 0
#define DISABLE_JOINT_5_POWER 0
#define DISABLE_JOINT_6_POWER 0

#define ROBOT_ARM_JSCOPE_DEBUG 1
/*********** Robot Configs End ***********/

#define ROBOT_ARM_TASK_INIT_TIME 500.0f
// task loop delay time
#define ROBOT_ARM_CONTROL_TIME_MS 5.0f
#define ROBOT_ARM_CONTROL_TIME_S (ROBOT_ARM_CONTROL_TIME_MS / 1000.0f)

#define ARM_JOINT_0_ANGLE_MIN (-PI)
#define ARM_JOINT_0_ANGLE_MAX PI
#define ARM_JOINT_0_ANGLE_REST 0.0f

#define ARM_JOINT_1_ANGLE_MIN (-30.0f / 180.0f * PI)
#define ARM_JOINT_1_ANGLE_MAX (35.0f / 180.0f * PI)
#define ARM_JOINT_1_ANGLE_REST ARM_JOINT_1_ANGLE_MAX

#define ARM_JOINT_2_ANGLE_MIN (-120.0f / 180.0f * PI)
#define ARM_JOINT_2_ANGLE_MAX 0.0f
#define ARM_JOINT_2_ANGLE_REST ARM_JOINT_2_ANGLE_MIN

#define ARM_JOINT_3_ANGLE_MIN (-80.0f / 180.0f * PI)
#define ARM_JOINT_3_ANGLE_MAX (80.0f / 180.0f * PI)
#define ARM_JOINT_3_ANGLE_REST 0.0f

#define ARM_JOINT_4_ANGLE_MIN (-0.5f * PI)
#define ARM_JOINT_4_ANGLE_MAX (0.5f * PI)
#define ARM_JOINT_4_ANGLE_REST 0.0f

#define ARM_JOINT_5_ANGLE_MIN (10.0f / 180.0f * PI)
#define ARM_JOINT_5_ANGLE_MAX (170.0f / 180.0f * PI)
#define ARM_JOINT_5_ANGLE_REST (90.0f / 180.0f * PI)

#define ARM_JOINT_6_ANGLE_MIN (-PI)
#define ARM_JOINT_6_ANGLE_MAX PI
#define ARM_JOINT_6_ANGLE_REST 0.0f

void robot_arm_init(void);
void robot_arm_status_update(void);
void robot_arm_control(void);
void robot_arm_reset_position(void);

const fp32 joint_angle_min[7] = {ARM_JOINT_0_ANGLE_MIN, ARM_JOINT_1_ANGLE_MIN, ARM_JOINT_2_ANGLE_MIN, ARM_JOINT_3_ANGLE_MIN, ARM_JOINT_4_ANGLE_MIN, ARM_JOINT_5_ANGLE_MIN, ARM_JOINT_6_ANGLE_MIN};
const fp32 joint_angle_max[7] = {ARM_JOINT_0_ANGLE_MAX, ARM_JOINT_1_ANGLE_MAX, ARM_JOINT_2_ANGLE_MAX, ARM_JOINT_3_ANGLE_MAX, ARM_JOINT_4_ANGLE_MAX, ARM_JOINT_5_ANGLE_MAX, ARM_JOINT_6_ANGLE_MAX};
const fp32 joint_angle_rest[7] = {ARM_JOINT_0_ANGLE_REST, ARM_JOINT_1_ANGLE_REST, ARM_JOINT_2_ANGLE_REST, ARM_JOINT_3_ANGLE_REST, ARM_JOINT_4_ANGLE_REST, ARM_JOINT_5_ANGLE_REST, ARM_JOINT_6_ANGLE_REST};

const fp32 yaw_offset, pitch_offset, roll_offset;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t robot_arm_high_water;
#endif

robot_arm_t robot_arm;
fp32 robot_arm_task_loop_delay;

#if ROBOT_ARM_JSCOPE_DEBUG
uint32_t loop_delay = 0;
static void jscope_robot_arm_test(void)
{
	loop_delay = HAL_GetTick() - robot_arm.time_ms;
}
#endif

void robot_arm_task(void const *pvParameters)
{
	osDelay(ROBOT_ARM_TASK_INIT_TIME);

	robot_arm_init();
	robot_arm_status_update();
	// wait_until_motors_online();

	while (1)
	{
		robot_arm_status_update();
		robot_arm_control();
		arm_joints_cmd_position(robot_arm.joint_angle_target);

		robot_arm_task_loop_delay = xTaskGetTickCount() - robot_arm.time_ms;
		if (robot_arm_task_loop_delay < ROBOT_ARM_CONTROL_TIME_MS)
		{
			osDelay(ROBOT_ARM_CONTROL_TIME_MS - robot_arm_task_loop_delay);
		}

#if ROBOT_ARM_JSCOPE_DEBUG
		jscope_robot_arm_test();
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
		robot_arm_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}

void robot_arm_control(void)
{
	// @TODO
}

void robot_arm_status_update(void)
{
	robot_arm.time_ms = xTaskGetTickCount();

	robot_arm.roll.now = *(robot_arm.arm_INS_angle + INS_ROLL_ADDRESS_OFFSET);
	robot_arm.roll.now -= roll_offset;
	robot_arm.roll.dot = *(robot_arm.arm_INS_speed + INS_GYRO_X_ADDRESS_OFFSET);
	robot_arm.roll.last = robot_arm.roll.now;

	robot_arm.pitch.now = *(robot_arm.arm_INS_angle + INS_PITCH_ADDRESS_OFFSET);
	robot_arm.pitch.now -= pitch_offset;
	robot_arm.pitch.dot = *(robot_arm.arm_INS_speed + INS_GYRO_Y_ADDRESS_OFFSET);
	robot_arm.pitch.last = robot_arm.pitch.now;

	robot_arm.yaw.now = *(robot_arm.arm_INS_angle + INS_YAW_ADDRESS_OFFSET);
	robot_arm.yaw.now -= yaw_offset;
	robot_arm.yaw.dot = *(robot_arm.arm_INS_speed + INS_GYRO_Z_ADDRESS_OFFSET);
	robot_arm.yaw.last = robot_arm.yaw.now;

	// robot_arm.accel_x = *(robot_arm.arm_INS_accel + INS_ACCEL_X_ADDRESS_OFFSET);
	// robot_arm.accel_y = *(robot_arm.arm_INS_accel + INS_ACCEL_Y_ADDRESS_OFFSET);
	// robot_arm.accel_z = *(robot_arm.arm_INS_accel + INS_ACCEL_Z_ADDRESS_OFFSET);
}

void robot_arm_init(void)
{
	robot_arm.arm_INS_angle = get_INS_angle_point();
	robot_arm.arm_INS_speed = get_gyro_data_point();
	// robot_arm.arm_INS_accel = get_accel_data_point();

	robot_arm_reset_position();
	enable_all_motor_control(1);
}

void robot_arm_reset_position(void)
{
	memcpy(robot_arm.joint_angle_target, joint_angle_rest, sizeof(joint_angle_rest));
}
