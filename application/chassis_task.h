/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. finished
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "CAN_receive.h"
#include "global_inc.h"
#include "pid.h"
#include "user_lib.h"

#define HEADLESS_HIP_TEST 0

#define METER_PER_SEC_ECD_MAX_LIMIT 1.5f
#define METER_ENCODER_MAX_LIMIT 0.5f
#define ANGLE_ECD_MAX_LIMIT (PI / 12.0f)

#define CHASSIS_A_LENGTH 0.322815f
#define CHASSIS_HALF_A_LENGTH (CHASSIS_A_LENGTH / 2.0f)
#define CHASSIS_L1_LENGTH 0.12f
#define CHASSIS_L2_LENGTH 0.177353f
#define CHASSIS_THETA_LOWER_LIMIT 0.0f
#define CHASSIS_THETA_UPPER_LIMIT DEG_TO_RAD(68.28f)
#define CHASSIS_THETA_LOWER_LIMIT_ECD (CHASSIS_THETA_LOWER_LIMIT * MG6012_MOTOR_RAD_TO_ECD)
#define CHASSIS_THETA_UPPER_LIMIT_ECD (CHASSIS_THETA_UPPER_LIMIT * MG6012_MOTOR_RAD_TO_ECD)

#define CHASSIS_THETA_CLEARANCE DEG_TO_RAD(3.0f)
#define CHASSIS_THETA_LOWER_LIMIT_WITH_CLEARANCE (CHASSIS_THETA_LOWER_LIMIT + CHASSIS_THETA_CLEARANCE)

// calculated by Matlab offline
#define CHASSIS_H_LOWER_LIMIT 0.177353f
#define CHASSIS_H_UPPER_LIMIT 0.287656f
#define CHASSIS_H_WORKSPACE_PEAK 0.239777f
#define CHASSIS_ALPHA_WORKSPACE_PEAK 0.206667f
#define CHASSIS_H_WORKSPACE_SLOPE1 0.302051f
#define CHASSIS_H_WORKSPACE_SLOPE2 (-0.231672f)

typedef struct
{
	// use float in internal calculation, but int or uint in communication
	// Upper board command values
	fp32 target_alpha1;                                  ///< unit rad
	fp32 target_alpha2;                                  ///< unit rad
	fp32 target_height;                                  ///< unit m
	                                                     // feedback values back to Upper board
	fp32 current_alpha1;                                 ///< unit rad
	fp32 current_alpha2;                                 ///< unit rad
	fp32 height;                                         ///< unit m
	fp32 wheel_rot_radius[STEER_MOTOR_COUNT];            ///< unit m
	fp32 target_wheel_rot_radius_dot[STEER_MOTOR_COUNT]; ///< unit m

	// internal control values
	fp32 target_theta[HIP_MOTOR_COUNT];
	fp32 target_theta_dot[HIP_MOTOR_COUNT]; ///< unit rad/s

	fp32 alpha_lower_limit;
	fp32 alpha_upper_limit;

	uint8_t fSteerMotorEnabled;
	uint8_t fHipMotorEnabled;
	uint8_t fFatalError;
	// lower board notify upper board whether to use the feedback of chassis platform from lower board
	uint8_t fHipDataIsValid;

	pid_type_def steer_angle_pid[STEER_MOTOR_COUNT];
	pid_type_def hip_angle_pid[HIP_MOTOR_COUNT]; // outputs target speed
	pid_type_def hip_speed_pid[HIP_MOTOR_COUNT]; // outputs target torque
} chassis_move_t;

extern void chassis_task(void const *pvParameters);
extern chassis_move_t chassis_move;

#endif
