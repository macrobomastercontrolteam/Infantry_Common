/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control
  * @note       Dual-method chassis power limiter (one global scale k per cycle):
  *               - M3508 (current-controlled): predict power from the commanded
  *                 current with the dataset model M3508_POLYMODEL[] and solve one
  *                 scale k so the summed predicted power stays within budget.
  *               - 4010  (speed-controlled, has current feedback): estimate the
  *                 present power as P = 24V * sum|I_feedback| and run a receding-
  *                 horizon MPC that chooses the speed scale k to maximize tracking
  *                 while keeping the predicted buffer energy above a safety floor.
  *             Budget comes from compute_p_ref() (energy / buffer aware).
  *
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.0.0     2025            ---             2. energy-based planning +
  *                                                quadratic k_limit solver
  *  V3.0.0     2025            ---             3. per-motor polynomial model,
  *                                                error-based power allocation
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "chassis_task.h"
#include "vofa_task.h"
#include "cmsis_os.h"
#include <string.h>

// M3508 electrical power model fitted from the power_estimator dataset:
//   P[W] = K0 + K1*I + K2*w + K3*I*w + K4*I^2 + K5*w^2
//   I = rotor current [A] (signed),  w = rotor speed [rad/s] (signed)
#if CHASSIS_POWER_CONTROL && (MOTOR_TYPE == POWER_TRAIN_USE_3508_MOTOR)
static const fp32 M3508_POLYMODEL[6] = {
    2.1599749f,       // K0
    0.0015595104f,    // K1
    -0.00017931848f,  // K2
    0.016782476f,     // K3
    0.11301039f,      // K4
    1.1531789e-05f    // K5
};
#endif

// telemetry
fp32 chassis_power_limit;
fp32 chassis_power;
fp32 chassis_power_buffer;

#if CHASSIS_POWER_CONTROL

/**
  * @brief          compute desired chassis power from remaining buffer energy.
  *                 linear in energy, clamped to [P_ref_min, P_ref_max].
  *                 returns P_ref_min when energy is critically low (keeps a
  *                 minimal safe command rather than an abrupt zero).
  * @param[in]      remaining_energy: buffer / supercap remaining energy [J]
  * @param[in]      refree_power_limit: referee power limit [W]
  * @retval         desired chassis power [W]
  */
static fp32 compute_p_ref(fp32 remaining_energy, fp32 refree_power_limit)
{
    fp32 p_ref;
    fp32 p_max;
    fp32 p_min;

    p_max = refree_power_limit * SPRINT_POWER_RATIO;
    p_min = refree_power_limit * WARNING_POWER_RATIO;

    if (remaining_energy < POWER_BUFF_DANGER)
        return p_min;

    p_ref = refree_power_limit + ENERGY_POWER_RATIO * (remaining_energy - POWER_BUFF_RESERVE);
    
    return fp32_constrain(p_ref, p_min, p_max);
}

void motor_power_init(motor_power_t *mp, const motor_power_init_t *init_data)
{
    mp->K0 = init_data->k0;
    mp->K1 = init_data->k1;
    mp->K2 = init_data->k2;
    mp->K3 = init_data->k3;
    mp->K4 = init_data->k4;
    mp->K5 = init_data->k5;
    mp->current_conversion = init_data->real_current_conversion;
    if (mp->current_conversion == 0.0f)
    {
        mp->current_conversion = 1.0f; // prevent divide-by-zero; config error
    }
    mp->feedback_power = 0.0f;
    mp->predict_not_limit_power = 0.0f;
    mp->predict_power = 0.0f;
    mp->power_limit = 0.0f;
    mp->is_limiting = 0;
}

fp32 motor_power_get_real_current(const motor_power_t *mp, fp32 current)
{
    if (mp->current_conversion == 0.0f)
    {
        return 0.0f;
    }
    return current / mp->current_conversion;
}

fp32 motor_power_update(motor_power_t *mp, fp32 current, fp32 speed,
                        predict_status_e predict_status, motor_power_negative_e negative_status)
{
    fp32 product = current * speed;
    fp32 power_sign = 1.0f;
    fp32 power;

    if (negative_status == MOTOR_PWR_NEGATIVE_ENABLED)
    {
        if (product < 0.0f)
            power_sign = -1.0f;
    }

    current = fabs(motor_power_get_real_current(mp, current));
    speed = fabs(speed);

    power = (mp->K0 + mp->K1 * current + mp->K2 * speed + mp->K3 * current * speed + mp->K4 * current * current + mp->K5 * speed * speed) * power_sign;

    if (predict_status == MOTOR_PWR_PREDICT_ENABLED)
        mp->predict_power = power;
    else if (predict_status == MOTOR_PWR_PREDICT_DISABLED)
        mp->feedback_power = power;
    else
        mp->predict_not_limit_power = power;

    return power;
}

/**
  * @brief          scale *desired_current so that predicted motor power stays
  *                 within motor_power_limit.  Modifies *desired_current in place.
  *
  *                 Solves  a*k^2 + b*k + c = 0  where:
  *                   a = K4 * I_d^2
  *                   b = (K1 + K3*|v|) * I_d
  *                   c = K0 + K2*|v| + K5*v^2 - limit
  *                 and I_d = |desired_current| / current_conversion.
  *                 Picks the largest valid root k in (0, 1).
  *
  * @param[in/out]  mp: motor power instance
  * @param[in/out]  desired_current: PID output (CAN units), scaled in-place
  * @param[in]      current_speed: actual wheel speed [m/s]
  * @param[in]      motor_power_limit: per-motor power budget [W]
  * @retval         scaling coefficient k in [0, 1]
  */
fp32 motor_power_limiter(motor_power_t *mp, fp32 *desired_current,
    fp32 current_speed, fp32 motor_power_limit)
{
    fp32 desired_current_val;
    fp32 real_desired_current;
    fp32 real_speed;
    fp32 predicted_power;
    fp32 a, b, c;
    fp32 discriminant, sqrt_disc;
    fp32 k1, k2, k;

    if (desired_current == NULL)
        return 0.0f;

    mp->power_limit = motor_power_limit;

    if (motor_power_limit < 0.0f)
    {
        *desired_current = 0.0f;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return 0.0f;
    }

    desired_current_val = *desired_current;
    real_desired_current = fabs(motor_power_get_real_current(mp, desired_current_val));
    real_speed = fabs(current_speed);

    // check whether full demand already fits within budget
    predicted_power = motor_power_update(mp, desired_current_val, current_speed,
        MOTOR_PWR_PREDICT_NOT_LIMIT, MOTOR_PWR_NEGATIVE_DISABLED);

    if (predicted_power <= motor_power_limit)
    {
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return 1.0f;
    }

    // quadratic coefficients: a*k^2 + b*k + c = 0
    a = mp->K4 * real_desired_current * real_desired_current;
    b = (mp->K1 + mp->K3 * real_speed) * real_desired_current;
    c = mp->K0 + mp->K2 * real_speed + mp->K5 * real_speed * real_speed - motor_power_limit;

    // degenerate: no quadratic term, solve linear b*k + c = 0
    if (fabs(a) < 1e-5f)
    {
        if (fabs(b) < 1e-5f)
        {
            // constant model: within limit or not
            if (c <= 1e-5f)
            {
                motor_power_update(mp, *desired_current, current_speed,
                    MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
                return 1.0f;
            }
            else
            {
                *desired_current = 0.0f;
                motor_power_update(mp, *desired_current, current_speed,
                    MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
                return 0.0f;
            }
        }
        k = -c / b;
        if (k < 0.0f)
        {
            *desired_current = 0.0f;
            motor_power_update(mp, *desired_current, current_speed,
                MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
            return 0.0f;
        }
        if (k > 1.0f)
        {
            motor_power_update(mp, *desired_current, current_speed,
                MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
            return 1.0f;
        }
        *desired_current *= k; //scale down output current
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return k;
    }

    discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        *desired_current = 0.0f;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return 0.0f;
    }

    if (fabs(discriminant) < 1e-5f)
    {
        k = -b / (2.0f * a);
        if (k < 0.0f)
        {
            *desired_current = 0.0f;
            motor_power_update(mp, *desired_current, current_speed,
                MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
            return 0.0f;
        }
        if (k > 1.0f)
        {
            motor_power_update(mp, *desired_current, current_speed,
                MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
            return 1.0f;
        }
        *desired_current *= k;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return k;
    }

    arm_sqrt_f32(discriminant, &sqrt_disc);
    k1 = (-b - sqrt_disc) / (2.0f * a);
    k2 = (-b + sqrt_disc) / (2.0f * a);

    // pick the largest k in (0, 1) to maximise output while staying within budget
    if ((k1 > 0.0f && k1 < 1.0f) && (k2 > 0.0f && k2 < 1.0f))
    {
        k = (k1 > k2) ? k1 : k2;
        *desired_current *= k;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return k;
    }
    else if ((k1 > 0.0f && k1 < 1.0f) && (k2 >= 1.0f || k2 <= 0.0f))
    {
        *desired_current *= k1;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return k1;
    }
    else if ((k1 <= 0.0f || k1 >= 1.0f) && (k2 > 0.0f && k2 < 1.0f))
    {
        *desired_current *= k2;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return k2;
    }
    else
    {
        *desired_current = 0.0f;
        motor_power_update(mp, *desired_current, current_speed,
            MOTOR_PWR_PREDICT_ENABLED, MOTOR_PWR_NEGATIVE_DISABLED);
        return 0.0f;
    }
}

// distribute total_power_limit across NUM_DRIVE_MOTORS motors in proportion
// to their error magnitudes.
void power_allocation_by_error(fp32 motor_errors[NUM_DRIVE_MOTORS], fp32 total_power_limit, fp32 buffer_power_attenuation, fp32 result[NUM_DRIVE_MOTORS])
{
    uint8_t i = 0;
    fp32 total_error;
    fp32 ratio;

    fp32 total_reserved_power;
    fp32 distributable_power;

    total_power_limit *= (1.0f - POWER_COMPENSATION_ALPHA);
    total_power_limit *= buffer_power_attenuation;

    if (total_power_limit <= 1e-5f)
    {
        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            result[i] = 0.0f;
        }
        return;
    }

    total_error = motor_errors[0] + motor_errors[1] + motor_errors[2] + motor_errors[3];
    total_reserved_power = (fp32)NUM_DRIVE_MOTORS * PER_MOTOR_RESERVED_POWER;

    // if total demand is very small (robot near-stationary), share equally
    if ((total_error < TOO_SMALL_ALL_ERRORS)||(total_power_limit < total_reserved_power))
    {
        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            result[i] = total_power_limit / (fp32)NUM_DRIVE_MOTORS;
        }
    }
    else 
    {
        distributable_power = total_power_limit - total_reserved_power;

        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            ratio = motor_errors[i] / total_error;
            result[i] = PER_MOTOR_RESERVED_POWER + ratio * distributable_power;
        }
    }
}

void chassis_pm_init(chassis_power_manager_t *cpm,
    motor_power_t *m0, motor_power_t *m1,
    motor_power_t *m2, motor_power_t *m3)
{
    uint8_t i = 0;
    cpm->motors[0] = m0;
    cpm->motors[1] = m1;
    cpm->motors[2] = m2;
    cpm->motors[3] = m3;
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        cpm->motor_errors[i] = 0.0f;
    }
}

void chassis_pm_update_error(chassis_power_manager_t *cpm, uint8_t index, fp32 error)
{
    if (index >= NUM_DRIVE_MOTORS)
        return;
    cpm->motor_errors[index] = fabs(error);
}

void chassis_pm_allocate_power(chassis_power_manager_t *cpm,
    fp32 total_power_limit, fp32 buffer_power_attenuation)
{
    uint8_t i = 0;
    fp32 errors_copy[NUM_DRIVE_MOTORS];
    fp32 allocated[NUM_DRIVE_MOTORS];

    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        errors_copy[i] = cpm->motor_errors[i];
    }

    power_allocation_by_error(errors_copy, total_power_limit, buffer_power_attenuation, allocated);

    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        cpm->motors[i]->power_limit = allocated[i];
    }
}

fp32 chassis_pm_get_total_predict_not_limit_power(const chassis_power_manager_t *cpm)
{
    uint8_t i = 0;
    fp32 total = 0.0f;
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        total += cpm->motors[i]->predict_not_limit_power;
    }
    return total;
}

fp32 chassis_pm_get_total_power_limit(const chassis_power_manager_t *cpm)
{
    uint8_t i = 0;
    fp32 total = 0.0f;
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        total += cpm->motors[i]->power_limit;
    }
    return total;
}

fp32 chassis_pm_get_total_predict_power(const chassis_power_manager_t *cpm)
{
    uint8_t i = 0;
    fp32 total = 0.0f;
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        total += cpm->motors[i]->predict_power;
    }
    return total;
}


fp32 rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, fp32 alpha)
{
    fp32 translation = sqrtf((fp32)vx * (fp32)vx + (fp32)vy * (fp32)vy);
    fp32 adjusted_rotate;

    if (fabs(translation) <= 1e-5f)
        return (fp32)rotate;

    adjusted_rotate = fabs((fp32)rotate) - alpha * translation;
    if (adjusted_rotate < 0.0f)
        return 0.0f;

    return (rotate < 0) ? -adjusted_rotate : adjusted_rotate;
}

void rotate_theta_forwardfeed(fp32 *theta, fp32 rotate, fp32 translation, fp32 kp)
{
    if (translation != 0.0f)
        *theta = *theta - kp * rotate;
}

void maf_init(moving_avg_filter_t *f, uint8_t size)
{
    if (size > MOVING_AVG_FILTER_MAX_SIZE)
        size = MOVING_AVG_FILTER_MAX_SIZE;
    if (size == 0)
        size = 1;
    memset(f->buffer, 0, sizeof(f->buffer));
    f->size = size;
    f->index = 0;
    f->count = 0;
    f->sum = 0.0f;
}

fp32 maf_update(moving_avg_filter_t *f, fp32 new_value)
{
    f->sum -= f->buffer[f->index];
    f->buffer[f->index] = new_value;
    f->sum += new_value;
    f->index = (f->index + 1) % f->size;
    if (f->count < f->size)
        f->count++;
    return f->sum / (fp32)f->count;
}

#if (MOTOR_TYPE == POWER_TRAIN_USE_3508_MOTOR)
/**
  * @brief          M3508 power at a fixed rotor speed is quadratic in current:
  *                 P = c0 + c1*I + c2*I^2. Fill coefs[] from the dataset model.
  */
static void m3508_current_power_coefs(fp32 omega, fp32 coefs[3])
{
    coefs[0] = M3508_POLYMODEL[0] + M3508_POLYMODEL[2] * omega + M3508_POLYMODEL[5] * omega * omega;
    coefs[1] = M3508_POLYMODEL[1] + M3508_POLYMODEL[3] * omega;
    coefs[2] = M3508_POLYMODEL[4];
}

/**
  * @brief          largest scaling factor lambda in [0, 1] such that scaling every
  *                 wheel current uniformly (I_i -> lambda*I_i) keeps the summed
  *                 predicted power within target_power. Returns 1.0 when the
  *                 unscaled demand already fits within budget.
  * @param[in]      currents: per-wheel commanded current [A]
  * @param[in]      omegas:   per-wheel rotor speed [rad/s]
  * @param[in]      target_power: total chassis drive-power budget [W]
  */
static fp32 m3508_chassis_power_scaling(const fp32 currents[NUM_DRIVE_MOTORS],
                                        const fp32 omegas[NUM_DRIVE_MOTORS],
                                        fp32 target_power)
{
    fp32 coefs[3];
    fp32 a = 0.0f, b = 0.0f, c = 0.0f;
    fp32 unscaled_power, disc, sqrt_disc, lambda1, lambda2;
    uint8_t i;

    // total power as a function of lambda: a*lambda^2 + b*lambda + c
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        m3508_current_power_coefs(omegas[i], coefs);
        c += coefs[0];
        b += coefs[1] * currents[i];
        a += coefs[2] * currents[i] * currents[i];
    }

    unscaled_power = a + b + c;
    if (unscaled_power <= target_power)
        return 1.0f;

    // solve a*lambda^2 + b*lambda + c = target_power
    if (a < 1e-6f)
    {
        // degenerates to linear b*lambda + c = target_power
        if (b > 1e-6f || b < -1e-6f)
            return fp32_constrain((target_power - c) / b, 0.0f, 1.0f);
        return 0.0f; // power independent of lambda yet already over target
    }

    disc = b * b - 4.0f * a * (c - target_power);
    if (disc < 0.0f)
        return 0.0f; // over target even at lambda = 0

    arm_sqrt_f32(disc, &sqrt_disc);
    lambda1 = (-b - sqrt_disc) / (2.0f * a);
    lambda2 = (-b + sqrt_disc) / (2.0f * a);

    // maximise output: prefer the larger valid root within [0, 1]
    if (lambda2 >= 0.0f && lambda2 <= 1.0f)
        return lambda2;
    if (lambda1 >= 0.0f && lambda1 <= 1.0f)
        return lambda1;
    return 0.0f;
}
#endif // MOTOR_TYPE == POWER_TRAIN_USE_3508_MOTOR

#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)
/**
  * @brief          MPC-style single-input speed-scale optimizer for 4010 power trains.
  *                 Models future buffer energy over a receding horizon as a function
  *                 of the global speed scale k, then chooses k to maximize speed
  *                 tracking while respecting power and energy constraints.
  *
  *                 Power model (affine around the previous operating point):
  *                   P(k) = P_idle + (P_meas - P_idle) * (k / k_prev)
  *                 P_idle captures controller + motor idle/friction losses, so the
  *                 model does not predict zero power at zero command.
  *
  *                 Constraints:
  *                   1) Instantaneous: P(k) <= p_ref
  *                   2) Predictive: buffer energy stays above E_safe over horizon N
  *                 Cost:
  *                   min (1-k)^2 + lambda*(k-k_prev)^2  subject to k <= k_max
  *                 The result is rate-limited and clamped to [0,1].
  *
  * @param[in]      p_ref: power budget from compute_p_ref() [W]
  * @param[in]      p_meas: present electrical power from current feedback [W]
  * @param[in/out]  k_prev: previous cycle's applied speed scale (updated in place)
  * @param[in]      e_now: current buffer energy [J]
  * @retval         speed scale k in [0, 1]
  */
static fp32 mpc_4010_speed_scale(fp32 p_ref, fp32 p_meas, fp32 *k_prev, fp32 e_now)
{
    fp32 k_opt, k_pred, k_inst, k_track;
    fp32 dt = CHASSIS_CONTROL_TIME_S;
    fp32 horizon_s = (fp32)CHASSIS_4010_MPC_HORIZON * dt;
    fp32 p_excess = p_meas - CHASSIS_4010_MPC_IDLE_POWER;
    fp32 p_budget  = p_ref  - CHASSIS_4010_MPC_IDLE_POWER;

    // Default to pass-through when no prior scale exists or no useful dynamics.
    if (*k_prev < 1e-3f)
        *k_prev = 1.0f;

    k_opt = 1.0f;

    if (p_excess > 1e-3f && p_budget > 1e-3f)
    {
        // Instantaneous power constraint: P(k) <= p_ref.
        k_inst = *k_prev * p_budget / p_excess;

        // Predictive energy constraint:
        //   E[j] = e_now + j*dt*(p_ref - P(k))
        // For a discharging scenario the minimum energy occurs at j=N.
        // Require e_now + horizon_s*(p_ref - P(k)) >= E_safe.
        k_pred = *k_prev * (p_budget + (e_now - CHASSIS_4010_MPC_SAFE_ENERGY) / horizon_s) / p_excess;

        k_opt = (k_inst < k_pred) ? k_inst : k_pred;
    }
    else if (p_budget <= 1e-3f)
    {
        // The power budget is at or below idle; no sustained motion is possible.
        k_opt = 0.0f;
    }

    k_opt = fp32_constrain(k_opt, 0.0f, 1.0f);

    // Quadratic tracking + smoothness cost. The unconstrained optimum is:
    //   k* = (TRACK_WEIGHT*1 + SMOOTH_WEIGHT*k_prev) / (TRACK_WEIGHT + SMOOTH_WEIGHT)
    // Clamp to the feasible region k <= k_opt.
    k_track = (CHASSIS_4010_MPC_TRACKING_COST * 1.0f
             + CHASSIS_4010_MPC_SMOOTH_COST * (*k_prev))
            / (CHASSIS_4010_MPC_TRACKING_COST + CHASSIS_4010_MPC_SMOOTH_COST);

    if (k_track > k_opt)
        k_track = k_opt;

    // Rate limiting for smooth actuator commands.
    if (k_track > *k_prev + CHASSIS_4010_MPC_MAX_RISE_STEP)
        k_track = *k_prev + CHASSIS_4010_MPC_MAX_RISE_STEP;
    if (k_track < *k_prev - CHASSIS_4010_MPC_MAX_FALL_STEP)
        k_track = *k_prev - CHASSIS_4010_MPC_MAX_FALL_STEP;

    k_track = fp32_constrain(k_track, 0.0f, 1.0f);

    *k_prev = k_track;
    return k_track;
}
#endif // MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR

/**
  * @brief          limit the power, mainly limit driver motor current
  * @retval         none
  */
void chassis_power_control(void)
{
    uint8_t i = 0;
    fp32 p_ref;
    fp32 k = 1.0f;
    fp32 pid_out_raw[NUM_DRIVE_MOTORS];

#if CAN_PASS_REF_INFO
    chassis_power_buffer = (fp32)can_ref_info.chassis_power_buffer;
    chassis_power_limit = (fp32)can_ref_info.chassis_power_limit;
    chassis_power = can_ref_info.PowerMeter_reading;
#else
    chassis_power_buffer = 0.0f;
    chassis_power_limit = 0.0f;
    chassis_power = 0.0f;
#endif

    // Energy-buffer-aware power budget. compute_p_ref() may return more than the
    // referee limit while buffer energy remains: the 60 J buffer 30 J usable
    // before the referee cuts chassis power) deliberately tolerates brief overshoot.
    p_ref = compute_p_ref(chassis_power_buffer, chassis_power_limit);

    if (p_ref <= 0.0f)
    {
        // critically low energy: cut all output
        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            chassis_move.motor_chassis[i].give_chassis_motor_cmd = 0;
        }
        return;
    }

    // Command each wheel would receive with no power limiting (identical to the
    // unlimited control path); the limiter only scales these uniformly by k.
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
#if (MOTOR_TYPE == POWER_TRAIN_USE_4010_MOTOR)
        pid_out_raw[i] = chassis_move.motor_chassis[i].speed_set + chassis_move.motor_speed_pid[i].out;
        pid_out_raw[i] = fp32_constrain(pid_out_raw[i],
                                        -(fp32)MOTOR_MG4010_MAX_CMD / MOTOR_ROTOR_TO_OUTPUT_CONSTANT,
                                        (fp32)MOTOR_MG4010_MAX_CMD / MOTOR_ROTOR_TO_OUTPUT_CONSTANT);
#else
        pid_out_raw[i] = chassis_move.motor_speed_pid[i].out;
#endif
    }

#if (MOTOR_TYPE == POWER_TRAIN_USE_3508_MOTOR)
    // M3508 wheels are current-controlled (MOTOR_ROTOR_TO_OUTPUT_CONSTANT == 1, so
    // the CAN command equals pid_out_raw). Predict power from the commanded current
    // with the dataset model and solve one global scaling factor for all four wheels.
    {
        fp32 currents_a[NUM_DRIVE_MOTORS];
        fp32 rotor_omega[NUM_DRIVE_MOTORS];
        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            currents_a[i] = pid_out_raw[i] * M3508_CAN_CMD_TO_CURRENT_A;
            rotor_omega[i] = (fp32)motor_chassis[i].speed_rpm * RPM_TO_RAD_S;
        }
        k = m3508_chassis_power_scaling(currents_a, rotor_omega,
                                        p_ref * (1.0f - POWER_COMPENSATION_ALPHA));
    }
#else
    {
        static fp32 k_mpc = 1.0f;   // previous cycle's MPC speed scale
        fp32 v_cmd = 0.0f;          // peak commanded wheel speed this cycle [m/s]
        fp32 p_now = 0.0f;
        fp32 mag;

        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            p_now += CHASSIS_BUS_VOLTAGE
                   * (fp32)fabs((fp32)motor_chassis[i].feedback_current * MG4010_FEEDBACK_CURRENT_TO_A);
            mag = fabs(pid_out_raw[i]);
            if (mag > v_cmd) v_cmd = mag;
        }

        if (v_cmd < 1e-3f)
        {
            // No commanded motion: reset the MPC state and pass the command through.
            k_mpc = 1.0f;
            k = 1.0f;
        }
        else
        {
            // Predictive governor: choose a global speed scale that maximizes tracking
            // while keeping the predicted buffer energy above the safety floor over the
            // MPC horizon. The MPC replaces the previous v_cap / accel-step heuristic.
            k = mpc_4010_speed_scale(p_ref, p_now, &k_mpc, chassis_power_buffer);
        }
    }
#endif

    {
        static fp32 prev_buffer = POWER_BUFF_TOTAL;
        static fp32 decreasing_ms = 0.0f;
        static fp32 flat_ms = 0.0f;
        static fp32 slow_scale = 1.0f;

        if (chassis_power_buffer < prev_buffer - CHASSIS_BUFFER_DROP_EPS)
        {
            // buffer dropped: count this step plus any preceding flat gap as drain time
            decreasing_ms += flat_ms + CHASSIS_CONTROL_TIME_MS;
            flat_ms = 0.0f;
        }
        else if (chassis_power_buffer > prev_buffer + CHASSIS_BUFFER_DROP_EPS)
        {
            // buffer rose: the drain ended, reset the trend detector
            decreasing_ms = 0.0f;
            flat_ms = 0.0f;
        }
        else
        {
            // buffer flat (normal between referee updates): tolerate a short gap, but a
            // long steady stretch means the drain has stopped
            flat_ms += CHASSIS_CONTROL_TIME_MS;
            if (flat_ms >= CHASSIS_BUFFER_FLAT_TIMEOUT_MS)
            {
                decreasing_ms = 0.0f;
                flat_ms = 0.0f;
            }
        }
        prev_buffer = chassis_power_buffer;

        if (decreasing_ms >= CHASSIS_BUFFER_DROP_TIME_MS)
            slow_scale -= CHASSIS_BUFFER_SLOW_STEP; // sustained drain: gradually slow down
        else
            slow_scale += CHASSIS_BUFFER_SLOW_STEP; // drain stopped: gradually resume
        slow_scale = fp32_constrain(slow_scale, CHASSIS_BUFFER_SLOW_MIN, 1.0f);

        k *= slow_scale;
    }

    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        chassis_move.motor_chassis[i].give_chassis_motor_cmd =
            (int16_t)(pid_out_raw[i] * k * MOTOR_ROTOR_TO_OUTPUT_CONSTANT);
    }
}
#endif // CHASSIS_POWER_CONTROL

bool_t chassis_power_control_mode_change(uint8_t fIsKeyPressed)
{
    static uint8_t fLastKeyPressed = 0;
    static uint8_t toggle_mode = 0;
    if (fLastKeyPressed != fIsKeyPressed)
    {
        fLastKeyPressed = fIsKeyPressed;
        if (fIsKeyPressed)
            toggle_mode = !toggle_mode;
    }
    return toggle_mode;
}

bool_t get_chassis_overpower(void)
{
#if CAN_PASS_REF_INFO
    return (bool_t)((can_ref_info.PowerMeter_reading > can_ref_info.chassis_power_limit)
        && (can_ref_info.chassis_power_buffer <= WARNING_POWER_BUFF));
#else
    return 0;
#endif
}
