#ifndef _BIPED_H
#define _BIPED_H

#include "biped_leg.h"
#include "pid.h"
#include "user_lib.h"
#include "arm_math.h"
#include "CAN_receive.h"

typedef enum
{
	JUMP_IDLE = 0,
	JUMP_CHARGE_INIT = 1,
	JUMP_CHARGE = 2,
	JUMP_LAUNCH = 3,
	JUMP_SHRINK = 4,
} jumpState_e;

typedef enum
{
	BRAKE_IDLE = 0,
	BRAKE_ENABLE = 1,
} brakeState_e;

typedef struct
{
	fp32 time_step_s; // second
	uint32_t time_ms; // millisecond

	pid_type_def turn_pid;
	pid_type_def split_pid;
	pid_type_def roll_pid;
	pid_type_def invPendulumInAir_pid;
	pid_type_def wheelBrakeInAirL_pid;
	pid_type_def wheelBrakeInAirR_pid;

	fp32 K_coeff[12][4];
	fp32 K_coeff_inAir[12][4];

	// fp32 acc_up_max, acc_down_max, acc_now;
	fp32 balance_angle;
	fp32 HipTorque_MaxLimit;
	fp32 DriveTorque_MaxLimit;
	// fp32 yaw_dot_last;
	variable_status_t yaw, pitch, roll;
	// fp32 accel_x, accel_y, accel_z;

	LegClass_t leg_L, leg_R, leg_simplified;
	uint8_t isJumpInTheAir;
	uint8_t fBipedEnable;	
	jumpState_e jumpState;
	brakeState_e brakeState;
} biped_t;

extern biped_t biped;

void biped_init(void);
void inv_pendulum_ctrl(void);
void torque_ctrl(void);
void biped_status_update(void);
uint8_t biped_jumpStart(void);
void biped_jumpManager(void);
fp32 biped_limitVelocity(fp32 speed_set, fp32 L0);

#endif /* _BIPED_H */
