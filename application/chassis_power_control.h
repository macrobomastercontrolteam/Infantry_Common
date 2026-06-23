/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_power_control.c/h
  * @brief      chassis power control
  * @note       Per-motor polynomial power model (ported from power_manager):
  *               P_i = K0 + K1*I + K2*|v| + K3*I*v + K4*I^2 + K5*v^2
  *             with I = raw_cmd / current_conversion [A], v in m/s.
  *
  *             Power is first planned via energy-based compute_p_ref(), then
  *             distributed across motors by error magnitude via
  *             power_allocation_by_error(), and finally each motor's PID output
  *             is independently limited via motor_power_limiter().
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
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "chassis_task.h"
#include "main.h"

// supercap
#define SUPCAP_VOLTAGE_MIN 3.0f
#define SUPCAP_VOLTAGE_LOWER_USE_THRESHOLD SUPCAP_VOLTAGE_MIN

// power control
#define SPRINT_POWER_RATIO 1.2f
#define WARNING_POWER_RATIO 0.7f

#define POWER_BUFF_TOTAL 60.0f
#define POWER_BUFF_RESERVE 20.0f
#define POWER_BUFF_DANGER 10.0f // energy [J] below which output is zeroed
#define ENERGY_POWER_RATIO 1.5f // slope of energy-to-power ramp [W/J]





#if (ROBOT_TYPE == INFANTRY_2023_SWERVE)
#define WARNING_POWER_BUFF (POWER_BUFF_TOTAL / 2.0f)
#else
#define WARNING_POWER_BUFF (POWER_BUFF_TOTAL / 3.0f)
#endif

#define NO_JUDGE_TOTAL_CURRENT_LIMIT (MAX_3508_MOTOR_CAN_CURRENT * CAN_TO_CURRENT_SCALE * 4.0f)
#define NUM_DRIVE_MOTORS 4

// RM motor CAN current conversion
#define CURRENT_TO_CAN_SCALE (1638.4f / 2000.0f)
#define CAN_TO_CURRENT_SCALE (1.0f / CURRENT_TO_CAN_SCALE)
#define CURRENT_TO_CAN(_mA) ((int16_t)((_mA) * CURRENT_TO_CAN_SCALE))
#define CAN_TO_CURRENT(_cmd) ((fp32)(_cmd) * CAN_TO_CURRENT_SCALE)

// current to power conversion
#define M3508_CURRENT_TO_POWER_RATIO 0.025f
#define M3508_CURRENT_TO_POWER(_current) ((_current) * M3508_CURRENT_TO_POWER_RATIO)
#define M3508_POWER_TO_CURRENT(_power) ((fp32)(_power) / M3508_CURRENT_TO_POWER_RATIO)
#define M3508_POWER_TO_CAN_CURRENT(_power) (((fp32)(_power) / M3508_CURRENT_TO_POWER_RATIO) * CURRENT_TO_CAN_SCALE)

#define M3508_POWER_CONTROL_DELAY_S (0.5f)
#define M3508_POWER_CONTROL_SHRINKED_DELAY_S (M3508_POWER_CONTROL_DELAY_S / 2.0f)
#define CURRENT_LIMIT_FILTER_COEFF 0.9f

// per-motor polynomial model parameters (ported from power_manager)
#define POWER_COMPENSATION_ALPHA 0.02f      // 2% safety margin applied to total power
#define PER_MOTOR_RESERVED_POWER 8.0f       // guaranteed per-motor power floor [W]
#define TOO_SMALL_ALL_ERRORS 0.04f         // if sum of all motor errors < this, share power equally
#define MOVING_AVG_FILTER_MAX_SIZE 32       // maximum buffer length for moving-average filter

#if(ROBOT_TYPE == INFANTRY_2024_MECANUM)
#define M3508_power_param_K0 1.548523f
#define M3508_power_param_K1 0.008882f
#define M3508_power_param_K2 0.000040f
#define M3508_power_param_K3 0.000054f
#define M3508_power_param_K4 0.000524f
#define M3508_power_param_K5 0.000001f
#define M3508_Current_Convertion 1638.4f

#elif(ROBOT_TYPE == HERO_2025_MECANUM)
#define M3508_power_param_K0 1.7790400f
#define M3508_power_param_K1 0.0027178f   
#define M3508_power_param_K2 0.0004475f   // |v| term  [W/(m/s)]
#define M3508_power_param_K3 0.0000410f   // I*v term  [W/(A*(m/s))]
#define M3508_power_param_K4 0.0005230f   // I^2 term  [W/A^2]
#define M3508_power_param_K5 0.0000010f
#define M3508_Current_Convertion 1638.4f // 16384 CAN = 10 A  =>  1 A / 1638.4 CAN

#elif(ROBOT_TYPE == SENTRY_2026_OMNI)
#define MG4010_power_param_K0 2.800243f
#define MG4010_power_param_K1 13.025089f
#define MG4010_power_param_K2 -1.086247f
#define MG4010_power_param_K3 -4.242602f
#define MG4010_power_param_K4 70.976102f
#define MG4010_power_param_K5 -0.052372f
#define MG4010_Current_Convertion 1.0f

#endif

#define POWER_LIMIT_HYSTERESIS 0.05f  // 5% 

typedef enum
{
    MOTOR_PWR_NEGATIVE_DISABLED = 0,
    MOTOR_PWR_NEGATIVE_ENABLED,
} motor_power_negative_e;

typedef enum
{
    MOTOR_PWR_PREDICT_DISABLED = 0,
    MOTOR_PWR_PREDICT_ENABLED,
    MOTOR_PWR_PREDICT_NOT_LIMIT,
} predict_status_e;

// initialisation parameters for one motor's power model
typedef struct
{
    fp32 k0, k1, k2, k3, k4, k5;
    fp32 real_current_conversion; // raw_cmd / this = current in Amperes
} motor_power_init_t;

// per-motor power state
typedef struct
{
    fp32 K0, K1, K2, K3, K4, K5;
    fp32 current_conversion;
    fp32 feedback_power;
    fp32 predict_not_limit_power;
    fp32 predict_power;
    fp32 power_limit;
    uint8_t is_limiting;
} motor_power_t;

// chassis-level power manager for NUM_DRIVE_MOTORS motors
typedef struct
{
    motor_power_t *motors[NUM_DRIVE_MOTORS];
    fp32 motor_errors[NUM_DRIVE_MOTORS];
} chassis_power_manager_t;

// simple moving-average filter with compile-time fixed buffer
typedef struct
{
    fp32 buffer[MOVING_AVG_FILTER_MAX_SIZE];
    uint8_t size;
    uint8_t index;
    uint8_t count;
    fp32 sum;
} moving_avg_filter_t;

/**
  * @brief          initialise a motor_power_t from a motor_power_init_t parameter struct
  * @param[out]     mp: motor power instance
  * @param[in]      init_data: initialisation parameters
  * @retval         none
  */
void motor_power_init(motor_power_t *mp, const motor_power_init_t *init_data);

/**
  * @brief          convert raw command to real current in Amperes
  * @param[in]      mp: motor power instance
  * @param[in]      current: raw command value
  * @retval         real current [A]
  */
fp32 motor_power_get_real_current(const motor_power_t *mp, fp32 current);

/**
  * @brief          evaluate the motor power model and store the result
  * @param[in/out]  mp: motor power instance (result stored in appropriate field)
  * @param[in]      current: raw motor command
  * @param[in]      speed: wheel linear speed [m/s]
  * @param[in]      predict_status: selects which field to store result in
  * @param[in]      negative_status: whether to account for regenerative (negative) power
  * @retval         computed power [W]
  */
fp32 motor_power_update(motor_power_t *mp, fp32 current, fp32 speed,
    predict_status_e predict_status, motor_power_negative_e negative_status);

/**
  * @brief          scale *desired_current so predicted motor power stays within budget;
  *                 modifies *desired_current in place
  * @param[in/out]  mp: motor power instance
  * @param[in/out]  desired_current: PID output (CAN units), scaled in-place
  * @param[in]      current_speed: actual wheel speed [m/s]
  * @param[in]      motor_power_limit: per-motor power budget [W]
  * @retval         scaling coefficient k in [0, 1]
  */
fp32 motor_power_limiter(motor_power_t *mp, fp32 *desired_current,
    fp32 current_speed, fp32 motor_power_limit);

/**
  * @brief          initialise a chassis_power_manager_t with four motor instances
  * @param[out]     cpm: chassis power manager instance
  * @param[in]      m0..m3: pointers to the four motor_power_t instances
  * @retval         none
  */
void chassis_pm_init(chassis_power_manager_t *cpm,
    motor_power_t *m0, motor_power_t *m1,
    motor_power_t *m2, motor_power_t *m3);

/**
  * @brief          update the error for one motor (used for proportional power distribution)
  * @param[in/out]  cpm: chassis power manager instance
  * @param[in]      index: motor index [0, NUM_DRIVE_MOTORS)
  * @param[in]      error: motor error value (magnitude will be taken internally)
  * @retval         none
  */
void chassis_pm_update_error(chassis_power_manager_t *cpm, uint8_t index, fp32 error);

/**
  * @brief          distribute total_power_limit across all motors by error proportion
  *                 and write each motor's share to its power_limit field
  * @param[in/out]  cpm: chassis power manager instance
  * @param[in]      total_power_limit: total chassis power budget [W]
  * @param[in]      buffer_power_attenuation: [0,1] scale factor for buffer depletion
  * @retval         none
  */
void chassis_pm_allocate_power(chassis_power_manager_t *cpm,
    fp32 total_power_limit, fp32 buffer_power_attenuation);

fp32 chassis_pm_get_total_predict_not_limit_power(const chassis_power_manager_t *cpm);
fp32 chassis_pm_get_total_power_limit(const chassis_power_manager_t *cpm);
fp32 chassis_pm_get_total_predict_power(const chassis_power_manager_t *cpm);

/**
  * @brief          distribute total_power_limit across NUM_DRIVE_MOTORS motors in
  *                 proportion to their error magnitudes; motor_errors[] is modified in-place
  * @param[in/out]  motor_errors: per-motor error magnitudes (abs taken internally)
  * @param[in]      total_power_limit: total power to distribute [W]
  * @param[in]      buffer_power_attenuation: [0,1] scale factor
  * @param[out]     result: allocated power per motor [W]
  * @retval         none
  */
void power_allocation_by_error(fp32 motor_errors[NUM_DRIVE_MOTORS],
    fp32 total_power_limit, fp32 buffer_power_attenuation,
    fp32 result[NUM_DRIVE_MOTORS]);

/**
  * @brief          split total_power between servo (steer) and wheel groups for swerve drive
  * @param[in]      total_power: total available power [W]
  * @param[in]      servo_rate: fraction of power reserved for servos (0, 1]
  * @param[in]      servo_predict_want_power: servo predicted demand [W]
  * @param[out]     result: [0] servo power, [1] wheel power
  * @retval         none
  */
void allocate_sw_power(fp32 total_power, fp32 servo_rate,
    fp32 servo_predict_want_power, fp32 result[2]);

/**
  * @brief          reduce rotate speed when translating to leave power headroom
  * @param[in]      vx, vy: chassis translation velocity components
  * @param[in]      rotate: raw rotation speed command
  * @param[in]      alpha: attenuation coefficient
  * @retval         adjusted rotation speed
  */
fp32 rotate_speed_allocation(int16_t vx, int16_t vy, int16_t rotate, fp32 alpha);

/**
  * @brief          compensate chassis yaw setpoint for power-limited rotation
  * @param[in/out]  theta: yaw setpoint to adjust
  * @param[in]      rotate: rotation speed
  * @param[in]      translation: translation speed magnitude
  * @param[in]      kp: feedforward gain
  * @retval         none
  */
void rotate_theta_forwardfeed(fp32 *theta, fp32 rotate, fp32 translation, fp32 kp);

/**
  * @brief          initialise a moving_avg_filter_t instance
  * @param[out]     f: filter instance
  * @param[in]      size: window length (clamped to MOVING_AVG_FILTER_MAX_SIZE)
  * @retval         none
  */
void maf_init(moving_avg_filter_t *f, uint8_t size);

/**
  * @brief          push a new sample and return the updated moving average
  * @param[in/out]  f: filter instance
  * @param[in]      new_value: newest sample
  * @retval         current moving average
  */
fp32 maf_update(moving_avg_filter_t *f, fp32 new_value);

/**
  * @brief          limit the power, mainly limit driver motor current
  * @retval         none
  */
extern void chassis_power_control(void);
extern bool_t chassis_power_control_mode_change(uint8_t fIsKeyPressed);
extern bool_t get_chassis_overpower(void);

#endif
