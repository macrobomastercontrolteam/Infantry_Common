#ifndef _ROBOT_ARM_TASK_H
#define _ROBOT_ARM_TASK_H

#include "CAN_receive.h"
#include "user_lib.h"
#include "pid.h"

typedef struct
{
	fp32 time_step_s; // second
	uint32_t time_ms; // millisecond
	pid_type_def joint_6_6020_angle_pid;

	fp32 joint_angle_target[7];

	const fp32 *arm_INS_angle;
	const fp32 *arm_INS_speed;
	// const fp32 *arm_INS_accel;
    variable_status_t yaw, pitch, roll;
    // fp32 accel_x, accel_y, accel_z;

} robot_arm_t;

extern const fp32 joint_angle_min[7];
extern const fp32 joint_angle_max[7];
extern const fp32 joint_angle_rest[7];
extern void robot_arm_task(void const *pvParameters);

extern robot_arm_t robot_arm;

#endif /* _ROBOT_ARM_TASK_H */
