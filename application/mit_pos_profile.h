/**
 * @file       mit_pos_profile.h
 * @brief      MIT-style position profile controller
 *             Provides velocity-limited position ramp, velocity feedforward,
 *             and optional sine-based gravity/load feedforward torque.
 *             Output: {pos, vel, KP, KD, torq} ready for encode_MIT_motor_control().
 */
#ifndef MIT_POS_PROFILE_H
#define MIT_POS_PROFILE_H

#include "global_inc.h"
#include <stdint.h>

/** MIT motor command output — maps directly to encode_MIT_motor_control() args. */
typedef struct
{
    fp32 pos;   ///< profiled position command (rad)
    fp32 vel;   ///< velocity feedforward (rad/s)
    fp32 KP;    ///< position gain
    fp32 KD;    ///< derivative / damping gain
    fp32 torq;  ///< feedforward torque (Nm)
} mit_cmd_t;

/** Persistent ramp state — one instance per motor axis. */
typedef struct
{
    fp32    pos_cmd;       ///< current ramped position command
    fp32    last_pos_cmd;  ///< previous cycle position command (for vel_cmd derivation)
    uint8_t initialized;   ///< 0 until first update; forces seed from measured_pos
} mit_pos_profile_t;

/** Tuning parameters — shared across axes or per-axis. */
typedef struct
{
    fp32    kp;               ///< position gain forwarded to motor
    fp32    kd;               ///< damping gain forwarded to motor
    fp32    max_vel;          ///< slew-rate limit (rad/s)
    fp32    dt;               ///< control loop period (s)
    fp32    torque_ff_bias;   ///< constant feedforward torque offset (Nm)
    uint8_t use_sine_ff;      ///< enable sinusoidal gravity compensation
    fp32    torque_ff_amp;    ///< amplitude of sine gravity comp term
    fp32    angle_offset;     ///< phase offset for sine gravity comp (rad)
    fp32    torque_ff_min;    ///< lower clamp on feedforward torque
    fp32    torque_ff_max;    ///< upper clamp on feedforward torque
} mit_pos_profile_param_t;

/**
 * @brief  Run one control cycle of the MIT position profile controller.
 *
 * @param  state        Persistent ramp state (one per axis).
 * @param  param        Tuning parameters.
 * @param  measured_pos Current motor position (rad).
 * @param  target_pos   Desired final position (rad).
 * @param  init         Non-zero forces re-seeding of the ramp from measured_pos
 *                      (use on mode-switch or large target jump).
 * @param  cmd          Output MIT command to pass to encode_MIT_motor_control().
 */
void mit_pos_profile_update(
    mit_pos_profile_t             *state,
    const mit_pos_profile_param_t *param,
    fp32                           measured_pos,
    fp32                           target_pos,
    uint8_t                        init,
    mit_cmd_t                     *cmd);

#endif /* MIT_POS_PROFILE_H */
