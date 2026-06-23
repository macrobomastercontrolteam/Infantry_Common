/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control
  * @note       Per-motor polynomial power model (ported from power_manager):
  *               P_i = K0 + K1*I + K2*|v| + K3*I*v + K4*I^2 + K5*v^2
  *             with I = raw_cmd / current_conversion [A], v in m/s.
  *
  *             Flow:
  *               1. compute_p_ref()              energy-based total power budget
  *               2. chassis_pm_allocate_power()  distribute budget by error
  *               3. motor_power_limiter()         scale each motor's PID output
  *
  *             Default K values map to the previous k1/k2/k3 model:
  *               K0 = p_bias/4, K2 = k3, K3 = k1, K4 = k2, K1 = K5 = 0
  *             Tune per motor on your robot.
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

// motor model default coefficients
// P_i = K0 + K1*I + K2*|v| + K3*I*v + K4*I^2 + K5*v^2
// I [A] = CAN_cmd / current_conversion,  v [m/s]
#if((ROBOT_TYPE == INFANTRY_2024_MECANUM)||(ROBOT_TYPE == HERO_2025_MECANUM))
const static motor_power_init_t motor_power_init_data = {
    .k0 = M3508_power_param_K0,
    .k1 = M3508_power_param_K1,
    .k2 = M3508_power_param_K2,
    .k3 = M3508_power_param_K3,
    .k4 = M3508_power_param_K4,
    .k5 = M3508_power_param_K5,
    .real_current_conversion = M3508_Current_Convertion
};
#elif(ROBOT_TYPE == SENTRY_2026_OMNI)
const static motor_power_init_t motor_power_init_data = {
    .k0 = MG4010_power_param_K0,
    .k1 = MG4010_power_param_K1,
    .k2 = MG4010_power_param_K2,
    .k3 = MG4010_power_param_K3,
    .k4 = MG4010_power_param_K4,
    .k5 = MG4010_power_param_K5,
    .real_current_conversion = MG4010_Current_Convertion
};
#endif     

// telemetry
fp32 chassis_power_limit;
fp32 chassis_power;
fp32 chassis_power_buffer;

// module-private instances
static motor_power_t chassis_motor_power[NUM_DRIVE_MOTORS];
static chassis_power_manager_t chassis_power_manager;

/**
  * @brief          compute desired chassis power from remaining buffer energy.
  *                 linear in energy, clamped to [P_ref_min, P_ref_max].
  *                 returns 0 when energy is critically low.
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
        return p_min; //to handle case where buffer energy couldn't be successfy received TODO:validate if would disrupt power control  //0.0f;

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
    mp->feedback_power = 0.0f;
    mp->predict_not_limit_power = 0.0f;
    mp->predict_power = 0.0f;
    mp->power_limit = 0.0f;
    mp->is_limiting = 0;
}

fp32 motor_power_get_real_current(const motor_power_t *mp, fp32 current)
{
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

/**
  * @brief          limit the power, mainly limit driver motor current
  * @retval         none
  */
void chassis_power_control(void)
{
    static uint8_t fInitialized = 0;
    uint8_t i = 0;
    fp32 p_ref;
    fp32 pid_out;
    fp32 k;
    fp32 pid_out_raw[NUM_DRIVE_MOTORS];
    fp32 k_arr[NUM_DRIVE_MOTORS];
    fp32 feedback_speed[4];
    fp32 normalized_error;

    // one-time initialisation of motor model instances
    if (!fInitialized)
    {
        for (i = 0; i < NUM_DRIVE_MOTORS; i++)
        {
            motor_power_init(&chassis_motor_power[i], &motor_power_init_data);
        }

        chassis_pm_init(&chassis_power_manager,
            &chassis_motor_power[0], &chassis_motor_power[1],
            &chassis_motor_power[2], &chassis_motor_power[3]);
        fInitialized = 1;
    }


    chassis_power_buffer = (fp32)can_ref_info.chassis_power_buffer;
    chassis_power_limit = (fp32)can_ref_info.chassis_power_limit;
    chassis_power = can_ref_info.PowerMeter_reading;

    // 1. energy-based desired power planning
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

    // 2. distribute budget across motors by PID demand magnitude
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        normalized_error =  chassis_move.motor_speed_pid[i].out / chassis_move.motor_speed_pid[i].max_out;
        chassis_pm_update_error(&chassis_power_manager, i, normalized_error);
    }
    chassis_pm_allocate_power(&chassis_power_manager, p_ref, 1.0f);

    // 3. per-motor limiting: scale each PID output to stay within allocated budget
    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
#if ((ROBOT_TYPE == INFANTRY_2024_MECANUM) || (ROBOT_TYPE == HERO_2025_MECANUM))
        feedback_speed[i] = motor_chassis[i].speed_rpm;
#elif (ROBOT_TYPE == SENTRY_2026_OMNI)
        feedback_speed[i] = motor_chassis[i].velocity;
#else
#error "undefined feedback speed data"
#endif
        pid_out_raw[i] = chassis_move.motor_speed_pid[i].out; // copy the pid out to local to isolate from pid control loop
        pid_out = pid_out_raw[i];                             // throwaway copy, motor_power_limiter scales it in place

        k = motor_power_limiter(&chassis_motor_power[i], &pid_out,
                                feedback_speed[i], chassis_motor_power[i].power_limit);

        k_arr[i] = k;
    }

    for (i = 0; i < NUM_DRIVE_MOTORS; i++)
    {
        chassis_move.motor_chassis[i].give_chassis_motor_cmd = (int16_t)(pid_out_raw[i] * k_arr[i] * MOTOR_ROTOR_TO_OUTPUT_CONSTANT);//(int16_t)(pid_out_raw[i] * MOTOR_ROTOR_TO_OUTPUT_CONSTANT);//
    }

}

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
    return (bool_t)((can_ref_info.PowerMeter_reading > can_ref_info.chassis_power_limit)
        && (can_ref_info.chassis_power_buffer <= WARNING_POWER_BUFF));
}
