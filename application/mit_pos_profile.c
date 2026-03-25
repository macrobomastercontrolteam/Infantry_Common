/**
 * @file  mit_pos_profile.c
 * @brief MIT-style position profile controller implementation.
 */
#include "mit_pos_profile.h"
#include "user_lib.h"   /* fp32_constrain */
#include "arm_math.h"   /* arm_sin_f32    */

void mit_pos_profile_update(
    mit_pos_profile_t             *state,
    const mit_pos_profile_param_t *param,
    fp32                           measured_pos,
    fp32                           target_pos,
    uint8_t                        init,
    mit_cmd_t                     *cmd)
{
    if ((state == 0) || (param == 0) || (cmd == 0) || (param->dt <= 0.0f))
    {
        return;
    }

    /* Seed the ramp from current measured position on first call or forced re-init */
    if ((!state->initialized) || init)
    {
        state->pos_cmd      = measured_pos;
        state->last_pos_cmd = measured_pos;
        state->initialized  = 1;
    }

    /* -----------------------------------------------------------------
     * 1) Position ramp / slew limit
     * ----------------------------------------------------------------- */
    fp32 max_step = param->max_vel * param->dt;
    fp32 err      = target_pos - state->pos_cmd;

    if (err > max_step)
    {
        state->pos_cmd += max_step;
    }
    else if (err < -max_step)
    {
        state->pos_cmd -= max_step;
    }
    else
    {
        state->pos_cmd = target_pos;
    }

    /* -----------------------------------------------------------------
     * 2) Velocity feedforward from ramp derivative
     * ----------------------------------------------------------------- */
    fp32 vel_cmd = (state->pos_cmd - state->last_pos_cmd) / param->dt;
    state->last_pos_cmd = state->pos_cmd;

    /* -----------------------------------------------------------------
     * 3) Feedforward torque (optional sine gravity / load compensation)
     * ----------------------------------------------------------------- */
    fp32 torq_ff = param->torque_ff_bias;
    if (param->use_sine_ff)
    {
        torq_ff += param->torque_ff_amp * arm_sin_f32(measured_pos + param->angle_offset);
    }
    torq_ff = fp32_constrain(torq_ff, param->torque_ff_min, param->torque_ff_max);

    /* -----------------------------------------------------------------
     * 4) Fill command output
     * ----------------------------------------------------------------- */
    cmd->pos  = state->pos_cmd;
    cmd->vel  = vel_cmd;
    cmd->KP   = param->kp;
    cmd->KD   = param->kd;
    cmd->torq = torq_ff;
}
