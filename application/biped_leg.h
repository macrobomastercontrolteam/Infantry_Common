/*
 * @Description: Leg Types
 * @Version: 2.0
 * @Author: Dandelion
 * @Date: 2023-03-24 17:20:36
 * @LastEditTime: 2023-04-18 18:28:10
 * @FilePath: \webots_sim\controllers\dynamic_lqr\Leg.h
 */
#ifndef _BIPED_LEG_H
#define _BIPED_LEG_H

#include "user_lib.h"
#include "pid.h"

typedef struct
{
	/* data */
	fp32 now;
	fp32 last;
	fp32 set;
	// fp32 set_dot;
	fp32 dot;  // derivative
	// fp32 ddot; // second derivative
} variable_status_t;

typedef struct
{
	// data
	fp32 angle1, angle2, angle3, angle4; // radians
	variable_status_t angle0;
	variable_status_t L0;
	variable_status_t dis;
	// Coordinates (in the coordinate system of the five-bar linkage, with the origin on the bisector of the five-bar linkage)
	fp32 xa, ya;
	fp32 xb, yb;
	fp32 xc, yc;
	fp32 xd, yd;
	fp32 xe, ye;
	// Force and Torque
	fp32 TL_set, TR_set, TWheel_set;
	fp32 F_set;  // get through PD control based on leg length, with the initial value being the gravity of the upper structure
	fp32 Tp_set; // according to the state feedback matrix

	fp32 K[2][6];     // feedback matrix
	fp32 X[6][1]; // state matrix, [theta, theta_dot, x, x_dot, phi, phi_dot]
    fp32 Xd[6][1]; // target state matrix, [theta, theta_dot, x, x_dot, phi, phi_dot]
	pid_type_def supportF_pid;
	pid_type_def supportFInAir_pid; // PID when in air
	pid_type_def supportFCharge_pid; // PID when in JUMP_CHARGE

	uint8_t fResetMultiAngleOffset;
} LegClass_t;

void LegClass_t_Init(LegClass_t* leg);
void LegClass_t_InvKinematics(LegClass_t* leg, const fp32 xc, const fp32 yc);
void LegClass_t_ForwardKinematics(LegClass_t* leg, const fp32 pitch);
void LegClass_t_VMC(LegClass_t* leg, const fp32 F, const fp32 Tp, fp32 result[2][1]);
void LegClass_t_Inv_VMC(LegClass_t* leg, const fp32 TL, const fp32 TR, fp32 result[2][1]);

#endif /* _BIPED_LEG_H */