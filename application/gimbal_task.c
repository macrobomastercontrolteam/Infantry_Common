/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and encoder mode
  *             gyro mode: use euler angle to control, encond mode: use encoder
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"

#include "can.h"
#include "main.h"

#include "cmsis_os.h"

#include "AHRS_middleware.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "bsp_laser.h"
#include "pid.h"
#include "cv_usart_task.h"
#include "custom_ui_task.h"

//motor encoder value format, range[0-8191]
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE - 1)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_yaw_pid_clear(gimbal_clear)                                                     \
    {                                                                                          \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_speed_pid);                    \
    }
#if (ROBOT_TYPE == HERO_2026_OMNI)
#define gimbal_second_yaw_pid_clear(gimbal_clear)                                               \
    {                                                                                           \
        PID_clear(&(gimbal_clear)->gimbal_second_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_second_yaw_motor.gimbal_motor_speed_pid);            \
    }
#endif
#define gimbal_pitch_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_speed_pid);                  \
    }

#define GIMBAL_YAW_MOTOR 0
#define GIMBAL_PITCH_MOTOR 1

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

static void gimbal_pitch_abs_angle_PID_init(gimbal_control_t *init);
static void gimbal_yaw_abs_angle_PID_init(gimbal_control_t *init);
static void gimbal_safety_manager(fp32 *yaw_can_set_value_ptr, fp32 *secondary_yaw_can_set_value_ptr, fp32 *pitch_can_set_value_ptr, int16_t *trigger_set_current_ptr, int16_t *fric1_set_current_ptr, int16_t *fric2_set_current_ptr);

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);
/**
  * @brief          gimbal some measure data updata, such as motor encoder, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     mode_change: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle, unit rad
  */
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

static void gimbal_motor_zero_force_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          limit angle set in GIMBAL_MOTOR_GYRO mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, uint8_t motor_select);
/**
  * @brief          limit angle set in GIMBAL_MOTOR_ENCODER mode, avoid exceeding the max angle
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, uint8_t motor_select);

/**
  * @brief          gimbal calibration calculate
  * @param[in]      gimbal_cali: cali data
  * @param[out]     yaw_offset:yaw motor middle place encode
  * @param[out]     pitch_offset:pitch motor middle place encode
  * @param[out]     max_yaw:yaw motor max machine angle
  * @param[out]     min_yaw: yaw motor min machine angle
  * @param[out]     max_pitch: pitch motor max machine angle
  * @param[out]     min_pitch: pitch motor min machine angle
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

#if GIMBAL_TEST_MODE
static void J_scope_gimbal_test(void);
#endif

gimbal_control_t gimbal_control;
static fp32 yaw_can_set_value = 0;
static fp32 secondary_yaw_can_set_value = 0;
static fp32 pitch_can_set_value = 0;
static int16_t trigger_set_current = 0;

#if (ROBOT_TYPE == HERO_2026_OMNI)
/**
  * @brief  Dual-yaw (coarse/fine) allocator for the 2026 hero.
  *
  *  Geometry / sensing:
  *    theta  = q1 + q2     launcher inertial heading (q1 = primary/carrier heading,
  *                         q2 = secondary angle relative to the carrier)
  *    The IMU sits on the secondary stage, so gimbal_yaw_motor.absolute_angle == theta
  *    and gimbal_yaw_motor.motor_gyro == theta_dot directly (no offset subtraction here).
  *    q2 / q2_dot come from the secondary 4310 encoder (already centred in feedback_update).
  *
  *  FINE stage (secondary): cascade angle->rate closed on the IMU drives the launcher to
  *    the aim target. Because it closes on the IMU it rejects chassis motion and the
  *    offset-CoG lever-arm kick within its bandwidth/stroke.
  *  COARSE stage (primary): a gentle, damped PD slews the big yaw so the secondary returns
  *    to the centre of its travel (q2 -> 0). This is the "big yaw follows the small yaw".
  */
static void hero_2026_dual_yaw_allocate(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    gimbal_motor_t *primary = &control_loop->gimbal_yaw_motor;
    gimbal_motor_t *second  = &control_loop->gimbal_second_yaw_motor;

    // Inertial launcher states (IMU is on the secondary/launcher stage).
    const fp32 theta        = primary->absolute_angle;      // launcher inertial heading
    const fp32 theta_target = primary->absolute_angle_set;  // aim setpoint
    const fp32 theta_dot    = primary->motor_gyro;          // launcher inertial yaw rate (IMU)
    const fp32 q2           = second->relative_angle;       // secondary angle rel. to carrier (centred)
    const fp32 q2_dot       = second->motor_gyro;           // secondary relative rate
    const fp32 q1_dot       = theta_dot - q2_dot;           // carrier (primary) inertial rate

    const fp32 soft_limit   = SECOND_YAW_MECH_LIMIT_RAD - SECOND_YAW_SOFT_LIMIT_MARGIN_RAD;

    // Lever-arm (offset-CoG) disturbance feedforward: the body-Y accelerometer (pitch axis,
    // stays horizontal) reads the chassis acceleration perpendicular to the aim, which is what
    // produces the disturbing yaw torque. Counter it -- mostly on the fast secondary stage.
    // While the fine loop is holding aim the launcher is not rotating, so this is almost pure
    // chassis disturbance rather than the gimbal's own motion.
    static fp32 lever_accel_filt = 0.0f;
    lever_accel_filt = first_order_filter(get_accel_data_point()[LEVER_ARM_FF_ACCEL_AXIS],
                                          lever_accel_filt, LEVER_ARM_FF_FILTER_COEFF);
    const fp32 lever_ff_sec = fp32_constrain(LEVER_ARM_FF_GAIN * lever_accel_filt,
                                             -LEVER_ARM_FF_MAX, LEVER_ARM_FF_MAX);
    const fp32 lever_ff_pri = fp32_constrain(LEVER_ARM_FF_PRIMARY_GAIN * lever_accel_filt,
                                             -LEVER_ARM_FF_MAX, LEVER_ARM_FF_MAX);

    // ---------- FINE STAGE (secondary): inertially aim the launcher ----------
    second->absolute_angle     = theta;        // mirrored for telemetry / J-scope
    second->absolute_angle_set = theta_target;
    second->motor_gyro_set = PID_calc_with_dot(&second->gimbal_motor_absolute_angle_pid,
                                               theta, theta_target,
                                               GIMBAL_CONTROL_TIME_S, theta_dot);
    fp32 sec_cmd = PID_calc(&second->gimbal_motor_speed_pid,
                            theta_dot, second->motor_gyro_set,
                            GIMBAL_CONTROL_TIME_S);
    sec_cmd += lever_ff_sec;                    // lever-arm disturbance feedforward

    // Stroke protection: never command the secondary further into a hard stop. The coarse
    // stage recenters it, so any saturation here is only transient.
    if ((q2 >=  soft_limit && sec_cmd > 0.0f) ||
        (q2 <= -soft_limit && sec_cmd < 0.0f))
    {
        sec_cmd = 0.0f;
    }
    second->cmd_value = sec_cmd;

    // ---------- COARSE STAGE (primary): lead via feedforward, then recenter ----------
    // Velocity feedforward: drive the big yaw at the commanded aim rate so it carries the
    // gross motion instead of waiting for the secondary to deflect first. This is the pure
    // command rate (d/dt of the target), independent of the fine loop; clamped against the
    // one-tick spike on a target snap (mode change / CV step).
    static fp32 theta_target_prev = 0.0f;
    static uint8_t ff_init = 0;
    fp32 aim_rate_ff = 0.0f;
    if (ff_init)
    {
        aim_rate_ff = rad_format(theta_target - theta_target_prev) / GIMBAL_CONTROL_TIME_S;
    }
    theta_target_prev = theta_target;
    ff_init = 1;
    aim_rate_ff = fp32_constrain(aim_rate_ff, -PRIMARY_FF_RATE_MAX, PRIMARY_FF_RATE_MAX);

    // Recenter PD: q2 = theta - q1, so a positive q2 needs a positive primary rate to null
    // it. +KD*q2_dot damps the slew (closed loop q2_dot = -KP/(1+KD)*q2): monotonic decay
    // with NO overshoot, so the small yaw never reverses on stick release.
    second->relative_angle_set = 0.0f;         // recenter target (telemetry/intent)
    fp32 primary_rate_set = PRIMARY_FF_GAIN * aim_rate_ff
                          + PRIMARY_RECENTER_KP * q2
                          + PRIMARY_RECENTER_KD * q2_dot;
    primary_rate_set = fp32_constrain(primary_rate_set, -PRIMARY_SLEW_RATE_MAX, PRIMARY_SLEW_RATE_MAX);
    primary->motor_gyro_set = primary_rate_set;
    primary->cmd_value = PID_calc(&primary->gimbal_motor_speed_pid,
                                  q1_dot, primary_rate_set,
                                  GIMBAL_CONTROL_TIME_S)
                       + lever_ff_pri;          // optional lever-arm feedforward share
}
#endif

uint8_t fLastKeyVSignal = 0;
fp32 cvAidedX, cvAidedY, debugx, debugy;
fp32 cv_coeff_x = 0.5, cv_coeff_y = 0.5;

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME_MS (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
void gimbal_task(void const *pvParameters)
{
    uint32_t ulSystemTime = osKernelSysTick();
    osDelay(GIMBAL_TASK_INIT_TIME);
    gimbal_init(&gimbal_control);
    shoot_init();
    //wait until all motors are online

    do
    {
#if ROBOT_YAW_IS_4310
        enable_DaMiao_motor(CAN_YAW_MOTOR_4310_TX_ID, 1, &CHASSIS_CAN);
#if (ROBOT_TYPE == HERO_2026_OMNI)
    enable_DaMiao_motor(CAN_SECOND_YAW_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN);
#endif
#endif
#if ROBOT_PITCH_IS_4340
        enable_DaMiao_motor(CAN_PITCH_MOTOR_4340_TX_ID, 1, &GIMBAL_CAN);
#elif  ROBOT_PITCH_IS_3507
        enable_DaMiao_motor(CAN_PITCH_MOTOR_3507_TX_ID, 1, &GIMBAL_CAN);
#endif
#if ROBOT_PITCH_IS_4310
    if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
    }
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    if (toe_is_error(PITCH_BASE_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_BASE_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
    }
#endif
    
#endif
#if ROBOT_PITCH_IS_4310
    if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
    }
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    if (toe_is_error(PITCH_BASE_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_BASE_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
    }
#endif
    
#endif
        CAN_cmd_gimbal_upper_can_ID(0, 0, 0, 0, 0, 0, 0);
#if (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == HERO_2026_OMNI)
        CAN_cmd_gimbal_lower_can_id(0, 0);
#endif
        osDelay(GIMBAL_CONTROL_TIME_MS);
        gimbal_feedback_update(&gimbal_control);
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)        
    } while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_BASE_GIMBAL_MOTOR_TOE));
#elif (ROBOT_TYPE == HERO_2026_OMNI)
    } while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(SECOND_YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE));
#else
    } while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE));
#endif

    while (1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        trigger_set_current = shoot_control_loop();
        gimbal_safety_manager(&yaw_can_set_value, &secondary_yaw_can_set_value, &pitch_can_set_value, &trigger_set_current, &shoot_control.fric1_given_current, &shoot_control.fric2_given_current);
        
        CAN_cmd_gimbal_upper_can_ID(yaw_can_set_value, secondary_yaw_can_set_value, pitch_can_set_value, trigger_set_current, shoot_control.fric1_given_current, shoot_control.fric2_given_current, shoot_control.piston_given_current);
#if TRIGGER_MOTOR_IS_4010
        CAN_cmd_4010_trigger(trigger_set_current);
#endif
#if ((ROBOT_PITCH_IS_4310 || ROBOT_PITCH_IS_4340) && (ROBOT_TYPE == INFANTRY_2026_MECANUM))
        CAN_cmd_gimbal_Damiao_motor(&gimbal_control.MIT_control_motor);
#endif

#if (ROBOT_TYPE == HERO_2025_MECANUM) || (ROBOT_TYPE == HERO_2026_OMNI)
        CAN_cmd_gimbal_lower_can_id(shoot_control.fric3_given_current, shoot_control.fric4_given_current);
#endif

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        osDelayUntil(&ulSystemTime, GIMBAL_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void gimbal_safety_manager(fp32 *yaw_can_set_value_ptr, fp32 *secondary_yaw_can_set_value_ptr, fp32 *pitch_can_set_value_ptr, int16_t *trigger_set_current_ptr, int16_t *fric1_set_current_ptr, int16_t *fric2_set_current_ptr)
{
    //TODO: uncomment before-push
    //safety for gimbal  
//     if (gimbal_emergency_stop() || toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
//     {
//         *yaw_can_set_value_ptr = 0;
//         *pitch_can_set_value_ptr = 0;
// #if ROBOT_PITCH_IS_4310
//         MIT_control_motor_init(&gimbal_control.MIT_control_motor);
// #endif
//     }
//     else
    {
#if YAW_REVERSED
        *yaw_can_set_value_ptr = -gimbal_control.gimbal_yaw_motor.cmd_value;
#else
        *yaw_can_set_value_ptr = gimbal_control.gimbal_yaw_motor.cmd_value;
#endif

#if (ROBOT_TYPE == HERO_2026_OMNI)
#if SECOND_YAW_REVERSED
        *secondary_yaw_can_set_value_ptr = -gimbal_control.gimbal_second_yaw_motor.cmd_value;
#else
        *secondary_yaw_can_set_value_ptr = gimbal_control.gimbal_second_yaw_motor.cmd_value;
#endif
#if PITCH_REVERSED
        *pitch_can_set_value_ptr = -gimbal_control.gimbal_pitch_motor.cmd_value;
#else
        *pitch_can_set_value_ptr = gimbal_control.gimbal_pitch_motor.cmd_value;
#endif
#endif
    }

    // safety for shoot
    if (toe_is_error(TRIGGER_MOTOR_TOE) || toe_is_error(FRICTIONAL_MOTOR_LEFT_TOE) || toe_is_error(FRICTIONAL_MOTOR_RIGHT_TOE))
    {
        *fric1_set_current_ptr = 0;
        *fric2_set_current_ptr = 0;
        *trigger_set_current_ptr = 0;
    }
}

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


/**
  * @brief          gimbal cali calculate, return motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step             = GIMBAL_CALI_START_STEP;
        // save the data when enter the cali mode, as the start data, to determine the max and min value
        gimbal_control.gimbal_cali.max_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch        = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd    = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw          = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd      = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
#if !ROBOT_YAW_HAS_SLIP_RING
        (*max_yaw) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_yaw) += GIMBAL_CALI_REDUNDANT_ANGLE;
#endif
        (*max_pitch) -= GIMBAL_CALI_REDUNDANT_ANGLE;
        (*min_pitch) += GIMBAL_CALI_REDUNDANT_ANGLE;
        gimbal_control.gimbal_yaw_motor.offset_ecd              = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle      = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle      = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd            = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle    = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle    = *min_pitch;
        gimbal_control.gimbal_cali.step = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          calc motor offset encode, max and min relative angle
  * @param[out]     yaw_offse:yaw middle place encode
  * @param[out]     pitch_offset:pitch place encode
  * @param[out]     max_yaw:yaw max relative angle
  * @param[out]     min_yaw:yaw min relative angle
  * @param[out]     max_yaw:pitch max relative angle
  * @param[out]     min_yaw:pitch min relative angle
  * @retval         none
  */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if ROBOT_YAW_HAS_SLIP_RING
	*yaw_offset = gimbal_cali->min_yaw_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
#else

#if YAW_REVERSED
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ECD_RANGE;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);
    
    ecd_format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);
#endif

#endif

#if PITCH_REVERSED

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / MOTOR_ECD_TO_RAD);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / MOTOR_ECD_TO_RAD);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ecd_format(temp_max_ecd);
    ecd_format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > HALF_ECD_RANGE)
    {
        temp_ecd -= ECD_RANGE;
    }
    else if (temp_ecd < -HALF_ECD_RANGE)
    {
        temp_ecd += ECD_RANGE;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ecd_format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief change pid parameters for absolute angle control mode
 */
static void gimbal_yaw_abs_angle_PID_init(gimbal_control_t *init)
{
    fp32 yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    fp32 yaw_angle_pid[3] = {YAW_ANGLE_PID_KP, YAW_ANGLE_PID_KI, YAW_ANGLE_PID_KD};
    fp32 yaw_camera_speed_pid[3] = {YAW_CAMERA_SPEED_PID_KP, YAW_CAMERA_SPEED_PID_KI, YAW_CAMERA_SPEED_PID_KD};
    fp32 yaw_camera_angle_pid[3] = {YAW_CAMERA_ANGLE_PID_KP, YAW_CAMERA_ANGLE_PID_KI, YAW_CAMERA_ANGLE_PID_KD};
    
    fp32* speed_pid_ptr;
    fp32* angle_pid_ptr;
    fp32 angle_pid_max_out;
    fp32 angle_pid_max_iout;
    fp32 speed_pid_max_out;
    fp32 speed_pid_max_iout;
    switch (init->gimbal_yaw_motor.gimbal_motor_mode)
    {
    case GIMBAL_MOTOR_CAMERA:
    {
        speed_pid_ptr = yaw_camera_speed_pid;
        angle_pid_ptr = yaw_camera_angle_pid;
        angle_pid_max_out = YAW_CAMERA_ANGLE_PID_MAX_OUT;
        angle_pid_max_iout = YAW_CAMERA_ANGLE_PID_MAX_IOUT;
        speed_pid_max_out = YAW_CAMERA_SPEED_PID_MAX_OUT;
        speed_pid_max_iout = YAW_CAMERA_SPEED_PID_MAX_IOUT;
        break;
    }
    case GIMBAL_MOTOR_GYRO:
    default:
    {
        speed_pid_ptr = yaw_speed_pid;
        angle_pid_ptr = yaw_angle_pid;
        angle_pid_max_out = YAW_ANGLE_PID_MAX_OUT;
        angle_pid_max_iout = YAW_ANGLE_PID_MAX_IOUT;
        speed_pid_max_out = YAW_SPEED_PID_MAX_OUT;
        speed_pid_max_iout = YAW_SPEED_PID_MAX_IOUT;
        break;
    }
    }
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, angle_pid_ptr, angle_pid_max_out, angle_pid_max_iout, 0, &rad_err_handler);
    // yaw speed is fast, so benefit of filtering on noise is insignificant comparing to the delay effect
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, speed_pid_ptr, speed_pid_max_out, speed_pid_max_iout, 0.85f, &filter_err_handler);
}

/**
 * @brief change pid parameters for absolute angle control mode
 */
static void gimbal_pitch_abs_angle_PID_init(gimbal_control_t *init)
{
    fp32 pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    fp32 pitch_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};
    fp32 pitch_camera_speed_pid[3] = {PITCH_CAMERA_SPEED_PID_KP, PITCH_CAMERA_SPEED_PID_KI, PITCH_CAMERA_SPEED_PID_KD};
    fp32 pitch_camera_angle_pid[3] = {PITCH_CAMERA_ANGLE_PID_KP, PITCH_CAMERA_ANGLE_PID_KI, PITCH_CAMERA_ANGLE_PID_KD};

    fp32* speed_pid_ptr;
    fp32* angle_pid_ptr;
    fp32 angle_pid_max_out;
    fp32 angle_pid_max_iout;
    fp32 speed_pid_max_out;
    fp32 speed_pid_max_iout;
    switch (init->gimbal_pitch_motor.gimbal_motor_mode)
    {
    case GIMBAL_MOTOR_CAMERA:
    {
        speed_pid_ptr = pitch_camera_speed_pid;
        angle_pid_ptr = pitch_camera_angle_pid;
        angle_pid_max_out = PITCH_CAMERA_ANGLE_PID_MAX_OUT;
        angle_pid_max_iout = PITCH_CAMERA_ANGLE_PID_MAX_IOUT;
        speed_pid_max_out = PITCH_CAMERA_SPEED_PID_MAX_OUT;
        speed_pid_max_iout = PITCH_CAMERA_SPEED_PID_MAX_IOUT;
        break;
    }
    case GIMBAL_MOTOR_GYRO:
    default:
    {
        speed_pid_ptr = pitch_speed_pid;
        angle_pid_ptr = pitch_angle_pid;
        angle_pid_max_out = PITCH_ANGLE_PID_MAX_OUT;
        angle_pid_max_iout = PITCH_ANGLE_PID_MAX_IOUT;
        speed_pid_max_out = PITCH_SPEED_PID_MAX_OUT;
        speed_pid_max_iout = PITCH_SPEED_PID_MAX_IOUT;
        break;
    }
    }
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, angle_pid_ptr, angle_pid_max_out, angle_pid_max_iout, 0, &rad_err_handler);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_speed_pid, PID_POSITION, speed_pid_ptr, speed_pid_max_out, speed_pid_max_iout, 0.85f, &filter_err_handler);
}

/**
  * @brief          "gimbal_control" valiable initialization, include pid initialization, remote control data point initialization, gimbal motors
  *                 data point initialization, and gyro sensor angle point initialization.
  * @param[out]     init: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{
#if (ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_4310) && (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    MIT_control_motor_init(&init->MIT_control_motor); //init variable for MIT controled motors
#endif
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
#if (ROBOT_TYPE == HERO_2026_OMNI)
    init->gimbal_second_yaw_motor.gimbal_motor_measure = &motor_chassis[MOTOR_INDEX_SECOND_YAW];
#endif
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->gimbal_rc_ctrl = get_remote_control_point();
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
#if (ROBOT_TYPE == HERO_2026_OMNI)
    init->gimbal_second_yaw_motor.gimbal_motor_mode = init->gimbal_second_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_ENCODER;
#endif

    static const fp32 yaw_encode_relative_angle_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};
#if (ROBOT_TYPE == HERO_2026_OMNI)
    static const fp32 second_yaw_fine_angle_pid[3] = {SECOND_YAW_FINE_ANGLE_PID_KP, SECOND_YAW_FINE_ANGLE_PID_KI, SECOND_YAW_FINE_ANGLE_PID_KD};
    static const fp32 second_yaw_speed_pid[3] = {SECOND_YAW_SPEED_PID_KP, SECOND_YAW_SPEED_PID_KI, SECOND_YAW_SPEED_PID_KD};
#endif
    static const fp32 pitch_encode_relative_angle_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, PID_POSITION, yaw_encode_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, 0, &rad_err_handler);
#if (ROBOT_TYPE == HERO_2026_OMNI)
    // Fine stage: launcher inertial angle (IMU) -> launcher rate (outer), then -> secondary torque (inner).
    PID_init(&init->gimbal_second_yaw_motor.gimbal_motor_absolute_angle_pid, PID_POSITION, second_yaw_fine_angle_pid, SECOND_YAW_FINE_ANGLE_PID_MAX_OUT, SECOND_YAW_FINE_ANGLE_PID_MAX_IOUT, 0, &rad_err_handler);
    PID_init(&init->gimbal_second_yaw_motor.gimbal_motor_speed_pid, PID_POSITION, second_yaw_speed_pid, SECOND_YAW_SPEED_PID_MAX_OUT, SECOND_YAW_SPEED_PID_MAX_IOUT, 0.85f, &filter_err_handler);
    // Coarse/recenter stage uses a manual PD in hero_2026_dual_yaw_allocate() feeding the primary speed PID.
#endif
    gimbal_yaw_abs_angle_PID_init(init);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PID_POSITION, pitch_encode_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, 0, &rad_err_handler);
    gimbal_pitch_abs_angle_PID_init(init);

  #if CV_INTERFACE
    init->gimbal_pitch_motor.CvCmdAngleFilter.size = CV_ANGLE_FILTER_SIZE;
    init->gimbal_pitch_motor.CvCmdAngleFilter.cursor = 0;
    init->gimbal_pitch_motor.CvCmdAngleFilter.ring = init->gimbal_pitch_motor.CvCmdAngleFilterBuffer;
    init->gimbal_pitch_motor.CvCmdAngleFilter.sum = 0;

    init->gimbal_yaw_motor.CvCmdAngleFilter.size = CV_ANGLE_FILTER_SIZE;
    init->gimbal_yaw_motor.CvCmdAngleFilter.cursor = 0;
    init->gimbal_yaw_motor.CvCmdAngleFilter.ring = init->gimbal_yaw_motor.CvCmdAngleFilterBuffer;
    init->gimbal_yaw_motor.CvCmdAngleFilter.sum = 0;
  #endif

    gimbal_yaw_pid_clear(init);
    gimbal_pitch_pid_clear(init);

    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.absolute_angle_offset = 0;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;

#if (ROBOT_TYPE == HERO_2026_OMNI)
    init->gimbal_second_yaw_motor.offset_ecd = 0;
    init->gimbal_second_yaw_motor.max_relative_angle = SECOND_YAW_MECH_LIMIT_RAD;
    init->gimbal_second_yaw_motor.min_relative_angle = -SECOND_YAW_MECH_LIMIT_RAD;
    // relative_angle / motor_gyro were already populated (and centred) by gimbal_feedback_update() above.
    init->gimbal_second_yaw_motor.relative_angle_set = 0.0f;                               // recenter target
    init->gimbal_second_yaw_motor.absolute_angle = init->gimbal_yaw_motor.absolute_angle;  // launcher heading (theta)
    init->gimbal_second_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_second_yaw_motor.motor_gyro_set = 0.0f;
    init->gimbal_second_yaw_motor.cmd_value = 0.0f;
#endif


    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.absolute_angle_offset = 0;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    init->gimbal_folding_status.target = UNFOLDED;
    init->gimbal_folding_status.current = FOLDED;

	init->gimbal_folding_status.gimbal_centered = 0;
    init->gimbal_folding_status.gimbal_fold_control_cmd = 0;
    init->gimbal_folding_status.gimbal_fold_in_progress = 0;
    init->gimbal_folding_status.gimbal_folding_step = 0;
#endif
#if ENABLE_LASER
    laser_enable(1);
#endif
}



void MIT_control_variable_set(MIT_control_variable_t *var, fp32 _pos, fp32 _vel, fp32 _KP, fp32 _KD, fp32 _torq)
{
    var->pos = _pos;
    var->vel = _vel;
    var->KP  = _KP;
    var->KD  = _KD;
    var->torq = _torq;
}

void MIT_control_motor_init(MIT_control_motor_t *motor)
{
    MIT_control_variable_set(&motor->pitch_MIT_variable, 0, 0, 0, 0, 0);
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    MIT_control_variable_set(&motor->pitch_base_MIT_variable, 0, 0, 0, 0, 0);
#endif
}

void MIT_motor_angle_control_config(MIT_control_motor_t *motor)
{
    MIT_control_variable_set(&motor->pitch_MIT_variable, 0, 0, PITCH_KP, PITCH_KD, 0);
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    MIT_control_variable_set(&motor->pitch_base_MIT_variable, PITCH_BASE_UNFOLD_POS, 0, PITCH_BASE_KP, PITCH_BASE_KD, 0);
#endif
}

void MIT_motor_torque_control_config(MIT_control_motor_t *motor)
{
    MIT_control_variable_set(&motor->pitch_MIT_variable, 0, 0, 0, 0, 0);
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    MIT_control_variable_set(&motor->pitch_base_MIT_variable, PITCH_BASE_UNFOLD_POS, 0, PITCH_BASE_KP, PITCH_BASE_KD, 0); //unfold pitch base in normal operation
#endif
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}
/**
  * @brief          gimbal some measure data updata, such as motor encoder, euler angle, gyro
  * @param[out]     gimbal_feedback_update: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

#if ROBOT_YAW_IS_4310
    if (toe_is_error(YAW_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_YAW_MOTOR_4310_TX_ID, 1, &CHASSIS_CAN); // attempt re-enable yaw motor when offline
    }
#if (ROBOT_TYPE == HERO_2026_OMNI)
    if (toe_is_error(SECOND_YAW_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_SECOND_YAW_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable second yaw motor when offline
    }
#endif
#endif

#if ROBOT_PITCH_IS_4310
    if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
        enable_DaMiao_motor(CAN_PITCH_BASE_MOTOR_4310_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
#endif
    }
#endif

#if ROBOT_PITCH_IS_4340
    if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_MOTOR_4340_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
    }
#elif ROBOT_PITCH_IS_3507
    if (toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        enable_DaMiao_motor(CAN_PITCH_MOTOR_3507_TX_ID, 1, &GIMBAL_CAN); // attempt re-enable pitch motor when offline
    }
#endif
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    if (gimbal_control.gimbal_folding_status.current == FOLDED)
    {
        goto gimbal_feedback_update_sensors;
    }
    gimbal_feedback_update_sensors:
#endif
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
#if PITCH_REVERSED
    //feedback_update->gimbal_pitch_motor.relative_angle = -motor_chassis[MOTOR_INDEX_PITCH].output_angle;
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#else
        feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
    feedback_update->gimbal_pitch_base_motor.relative_angle = motor_chassis[MOTOR_INDEX_PITCH_BASE].output_angle;
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_REVERSED
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = AHRS_cosf(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - AHRS_sinf(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
#if (ROBOT_TYPE == HERO_2026_OMNI)
    // Secondary (offset) yaw feedback, expressed relative to the calibrated mechanical centre
    // so q2 == 0 means the launcher is aligned with the carrier.
#if SECOND_YAW_REVERSED
    feedback_update->gimbal_second_yaw_motor.relative_angle = -motor_chassis[MOTOR_INDEX_SECOND_YAW].output_angle - SECOND_YAW_CENTER_OFFSET_RAD;
    feedback_update->gimbal_second_yaw_motor.motor_gyro = -motor_chassis[MOTOR_INDEX_SECOND_YAW].velocity;
#else
    feedback_update->gimbal_second_yaw_motor.relative_angle = motor_chassis[MOTOR_INDEX_SECOND_YAW].output_angle - SECOND_YAW_CENTER_OFFSET_RAD;
    feedback_update->gimbal_second_yaw_motor.motor_gyro = motor_chassis[MOTOR_INDEX_SECOND_YAW].velocity;
#endif
    // NOTE: the IMU is mounted on the secondary (launcher) stage, so gimbal_yaw_motor.absolute_angle
    // is ALREADY the launcher's inertial heading (theta). Do NOT subtract the secondary offset here:
    // the coarse/fine split lives in hero_2026_dual_yaw_allocate(). (The previous code subtracted it
    // here AND again in the allocator -- a double-count that corrupted both yaw loops.)
#endif
}

/**
  * @brief          calculate the relative angle between ecd and offset_ecd
  * @param[in]      ecd: motor now encode
  * @param[in]      offset_ecd: gimbal offset encode
  * @retval         relative angle; unit rad; range is [-PI, PI]; positive direction is clockwise; forward direction is angle=0
  */
fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
  * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

    // yaw motor mode change
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode)
    {
        switch (gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode)
        {
        case GIMBAL_MOTOR_ZERO_FORCE:
        case GIMBAL_MOTOR_RAW:
        {
            gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.cmd_value;
            break;
        }
        case GIMBAL_MOTOR_GYRO:
        {
            // change pid parameters, which depends on motor control mode
            gimbal_yaw_abs_angle_PID_init(gimbal_mode_change);
            gimbal_yaw_pid_clear(gimbal_mode_change);
            gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
#if (ROBOT_TYPE == HERO_2026_OMNI)
            gimbal_mode_change->gimbal_second_yaw_motor.relative_angle_set = 0.0f; // coarse stage recenters q2 -> 0
            gimbal_mode_change->gimbal_second_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
            gimbal_second_yaw_pid_clear(gimbal_mode_change);
#endif
            break;
        }
        case GIMBAL_MOTOR_CAMERA:
        {
            // change pid parameters, which depends on motor control mode
            gimbal_yaw_abs_angle_PID_init(gimbal_mode_change);
            gimbal_yaw_pid_clear(gimbal_mode_change);
            gimbal_mode_change->gimbal_yaw_motor.absolute_angle_offset = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
            gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
#if (ROBOT_TYPE == HERO_2026_OMNI)
            gimbal_mode_change->gimbal_second_yaw_motor.relative_angle_set = 0.0f; // coarse stage recenters q2 -> 0
            gimbal_mode_change->gimbal_second_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
            gimbal_second_yaw_pid_clear(gimbal_mode_change);
#endif
            break;
        }
        case GIMBAL_MOTOR_ENCODER:
        {
            gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
            break;
        }
        }
        gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;
    }

    // pitch motor mode change
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode)
    {
#if ((ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_4310) && (ROBOT_TYPE == INFANTRY_2026_MECANUM))
        MIT_control_motor_init(&gimbal_mode_change->MIT_control_motor); //init to set all param to 0 first 
#endif

        switch (gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode)
        {
        case GIMBAL_MOTOR_ZERO_FORCE:
        {
#if ((ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_4310) && (ROBOT_TYPE == INFANTRY_2026_MECANUM))
            // set control params to 0
            MIT_control_motor_init(&gimbal_mode_change->MIT_control_motor);
#endif
            break;
        }
        case GIMBAL_MOTOR_RAW:
        {
#if (ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_4310) && (ROBOT_TYPE == INFANTRY_2026_MECANUM)
            //config for torque control
            MIT_motor_torque_control_config(&gimbal_mode_change->MIT_control_motor);
#endif
            gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.cmd_value;
            break;
        }
        case GIMBAL_MOTOR_GYRO:
        {
#if (ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_4310) && (ROBOT_TYPE == INFANTRY_2026_MECANUM)
            //config for torque control
            MIT_motor_torque_control_config(&gimbal_mode_change->MIT_control_motor);
#endif
            // change pid parameters, which depends on motor control mode
            gimbal_pitch_abs_angle_PID_init(gimbal_mode_change);
            gimbal_pitch_pid_clear(gimbal_mode_change);
            gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
            break;
        }
        case GIMBAL_MOTOR_CAMERA:
        {
            // change pid parameters, which depends on motor control mode
            gimbal_pitch_abs_angle_PID_init(gimbal_mode_change);
            gimbal_pitch_pid_clear(gimbal_mode_change);
            gimbal_mode_change->gimbal_pitch_motor.absolute_angle_offset = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
            gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
            break;
        }
#if (ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_4310) && (ROBOT_TYPE == INFANTRY_2026_MECANUM)
        case GIMBAL_MOTOR_MIT_ANGLE:
        {
            MIT_motor_angle_control_config(&gimbal_mode_change->MIT_control_motor);   
            gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
            break;
        }
#endif
        case GIMBAL_MOTOR_ENCODER:
        {
            gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
            break;
        }
        }
        gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
    }
}
/**
  * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".         
  * @param[out]     gimbal_set_control: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

#if !((ROBOT_TYPE == SENTRY_2023_MECANUM) || (ROBOT_TYPE == SENTRY_2026_OMNI))

		if (CvCmder_GetMode(CV_MODE_ASSIST_BIT) && fCvAutoAim())
		{
                cvAidedX = -CvCmdHandler.CvCmdMsg.xAimError * YAW_RC_CV_SEN_INC * 0.85f;
                cvAidedY = CvCmdHandler.CvCmdMsg.yAimError * PITCH_RC_CV_SEN_INC * 0.85f;
                // cvAidedX = debugx * YAW_RC_CV_SEN_INC;
                // cvAidedY = debugy * PITCH_RC_CV_SEN_INC;
                
                ui_info.auto_aim_state = 1;      
		}
        else{
            cvAidedX = 0.0f;
            cvAidedY = 0.0f;

            ui_info.auto_aim_state = 0;
        }
#else
#if DEBUG_CV
        if(chassis_move.chassis_RC->rc.s[RC_RIGHT_LEVER_CHANNEL] == RC_SW_UP)
#else
        if(toe_is_error(REMOTE_TOE) && gimbal_behaviour == GIMBAL_AUTO_AIM)
#endif
        {
            cvAidedX = -CvCmdHandler.CvCmdMsg.xAimError * YAW_RC_CV_SEN_INC*0.35f;
            cvAidedY = CvCmdHandler.CvCmdMsg.yAimError * PITCH_RC_CV_SEN_INC *0.35f;
        }
        else{
            cvAidedX = 0.0f;
            cvAidedY = 0.0f;
        }
#endif

    
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    add_yaw_angle += cvAidedX;
    add_pitch_angle += cvAidedY; 
    // yaw motor mode control
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // send control value directly in raw mode
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if ((set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) || (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_CAMERA))
    {
        // gyro mode control
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle, GIMBAL_YAW_MOTOR);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle, GIMBAL_YAW_MOTOR);
    }

    // pitch motor mode control
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if ((set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) || (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_CAMERA))
    {
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle, GIMBAL_PITCH_MOTOR);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle, GIMBAL_PITCH_MOTOR);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_MIT_ANGLE)
    {

    }
}


/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, uint8_t motor_select)
{
    static fp32 bias_angle;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //present angle error
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
#if ROBOT_YAW_HAS_SLIP_RING
    // Remove yaw motor limit for robots with slip ring
    if (motor_select != GIMBAL_YAW_MOTOR)
#endif
    {
        //relative angle + angle error + add_angle > max_relative angle
        if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
        {
            // if true, turn towards the maximum mechanical angle
            if (add > 0.0f)
            {
                // calculate for the max add_angle
                add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
            }
        }
        else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
        {
            if (add < 0.0f)
            {
                add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
            }
        }
    }
    gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);

    // Relative angle implementation for chassis spinning mode
    // if ((motor_select == GIMBAL_YAW_MOTOR) && ((chassis_behaviour_mode == CHASSIS_SPINNING_MODE))
    // {
    //     chassis_move.chassis_relative_angle_set = rad_format(chassis_move.chassis_relative_angle_set + add);
    // }
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add, uint8_t motor_select)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
#if ROBOT_YAW_HAS_SLIP_RING
    // Remove yaw motor limit for robots with slip ring
    if (motor_select != GIMBAL_YAW_MOTOR)
#endif
    {
        // Whether it exceeds the maximum and minimum values
        if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
        }
        else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
        }
    }
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor
  * @param[out]     gimbal_control_loop: "gimbal_control" valiable point
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    if(control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE)
    {
        gimbal_motor_zero_force_control(&control_loop->gimbal_yaw_motor);
#if (ROBOT_TYPE == HERO_2026_OMNI)
        control_loop->gimbal_second_yaw_motor.cmd_value = 0.0f;
#endif
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
#if (ROBOT_TYPE == HERO_2026_OMNI)
        control_loop->gimbal_second_yaw_motor.cmd_value = 0.0f;
#endif
    }
    else if ((control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) || (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_CAMERA))
    {
    #if (ROBOT_TYPE == HERO_2026_OMNI)
        hero_2026_dual_yaw_allocate(control_loop);
    #else
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    #endif
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
#if (ROBOT_TYPE == HERO_2026_OMNI)
        control_loop->gimbal_second_yaw_motor.cmd_value = 0.0f;
#endif
    }

    if(control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE)
    {
        gimbal_motor_zero_force_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
#if ROBOT_PITCH_IS_4310  
        MIT_motor_set_torq(control_loop);
#endif
    }
    else if ((control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) || (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_CAMERA))
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
#if ROBOT_PITCH_IS_4310
        MIT_motor_set_torq(control_loop);
#endif
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_MIT_ANGLE)
    {
#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)
        foldable_pitch_control(control_loop);
#endif
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    // cascade pid: angle loop & speed loop
    gimbal_motor->motor_gyro_set = PID_calc_with_dot(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, GIMBAL_CONTROL_TIME_S, gimbal_motor->motor_gyro);
    gimbal_motor->cmd_value = PID_calc(&gimbal_motor->gimbal_motor_speed_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set, GIMBAL_CONTROL_TIME_S);
}
/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_ENCODER, use the encode relative angle  to control. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    // cascade pid: angle loop & speed loop
    gimbal_motor->motor_gyro_set = PID_calc_with_dot(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, GIMBAL_CONTROL_TIME_S, gimbal_motor->motor_gyro);
    gimbal_motor->cmd_value = PID_calc(&gimbal_motor->gimbal_motor_speed_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set, GIMBAL_CONTROL_TIME_S);
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_RAW, current  is sent to CAN bus. 
  * @param[out]     gimbal_motor: yaw motor or pitch motor
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->cmd_value = gimbal_motor->raw_cmd_current;
}

static void gimbal_motor_zero_force_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->cmd_value = 0.0f;
}

#if GIMBAL_TEST_MODE
fp32 yaw_cv_delta_fp32;
fp32 yaw_ins_fp32, pitch_ins_fp32;
fp32 yaw_ins_set_fp32, pitch_ins_set_fp32;
fp32 pitch_relative_set_fp32, pitch_relative_angle_fp32;
fp32 yaw_speed_fp32, pitch_speed_fp32;
fp32 yaw_speed_set_fp32, pitch_speed_set_fp32;
fp32 yaw_speed_pid_err;
fp32 pitch_speed_pid_err;
uint32_t gimbal_last_update_time;
uint32_t gimbal_update_interval;
fp32 fric_diff_fp32;
uint8_t fChassisSpinning;
static void J_scope_gimbal_test(void)
{
#if CV_INTERFACE
    yaw_cv_delta_fp32 = -CvCmdHandler.CvCmdMsg.xAngle * 180.0f / PI;
#endif
    yaw_ins_fp32 = gimbal_control.gimbal_yaw_motor.absolute_angle * 180.0f / PI;
    yaw_ins_set_fp32 = gimbal_control.gimbal_yaw_motor.absolute_angle_set * 180.0f / PI;
    yaw_speed_fp32 = gimbal_control.gimbal_yaw_motor.motor_gyro * 180.0f / PI;
    yaw_speed_set_fp32 = gimbal_control.gimbal_yaw_motor.motor_gyro_set * 180.0f / PI;

    pitch_ins_fp32 = gimbal_control.gimbal_pitch_motor.absolute_angle * 180.0f / PI;
    pitch_ins_set_fp32 = gimbal_control.gimbal_pitch_motor.absolute_angle_set * 180.0f / PI;
    pitch_speed_fp32 = gimbal_control.gimbal_pitch_motor.motor_gyro * 180.0f / PI;
    pitch_speed_set_fp32 = gimbal_control.gimbal_pitch_motor.motor_gyro_set * 180.0f / PI;
    pitch_relative_angle_fp32 = gimbal_control.gimbal_pitch_motor.relative_angle * 180.0f / PI;
    pitch_relative_set_fp32 = gimbal_control.gimbal_pitch_motor.relative_angle_set * 180.0f / PI;

    yaw_speed_pid_err = gimbal_control.gimbal_yaw_motor.gimbal_motor_speed_pid.error[0] * 180.0f / PI;
    pitch_speed_pid_err = gimbal_control.gimbal_pitch_motor.gimbal_motor_speed_pid.error[0] * 180.0f / PI;

    gimbal_update_interval = osKernelSysTick() - gimbal_last_update_time;
    gimbal_last_update_time = osKernelSysTick();

    fric_diff_fp32 = shoot_control.friction_motor1_rpm + shoot_control.friction_motor2_rpm;

    fChassisSpinning = CvCmder_GetMode(CV_MODE_CHASSIS_SPINNING_BIT);
}
#endif

/**
 * @brief Emergency stop condition for sentry
 * @return bool_t: true if E-stop
 */
bool_t gimbal_emergency_stop(void)
{
    uint8_t fEStop = 1;
    static uint8_t fFatalError = 0;
    uint8_t current_fault = 0;
#if ROBOT_YAW_IS_4310
    #if (ROBOT_PITCH_IS_4340 || ROBOT_PITCH_IS_3507 || ROBOT_PITCH_IS_4310)
        current_fault = ((fabs(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->torque) >= YAW_4310_MOTOR_TORQUE_LIMIT) || (int_abs(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->feedback_current) >= PITCH_4310_MOTOR_TORQUE_LIMIT));
    #else
        current_fault = ((fabs(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->torque) >= YAW_4310_MOTOR_TORQUE_LIMIT) || (int_abs(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->feedback_current) >= PITCH_MOTOR_CURRENT_LIMIT));
    #endif
#else
    current_fault = ((int_abs(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->feedback_current) >= YAW_6020_MOTOR_CURRENT_LIMIT) || (int_abs(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->feedback_current) >= PITCH_MOTOR_CURRENT_LIMIT));
#endif
    if (current_fault)
    {
        fFatalError = 1;
    }
	else if (!toe_is_error(REMOTE_TOE))
	{
		fFatalError = 0;
	}

	if (fFatalError)
	{
		fEStop = 1;
	}
	else
	{
		fEStop = ((gimbal_behaviour != GIMBAL_AUTO_AIM) && (gimbal_behaviour != GIMBAL_AUTO_AIM_PATROL) && toe_is_error(REMOTE_TOE));
	}
	return fEStop;
}

fp32 get_gimbal_relative_yaw_angle(void)
{
    return (gimbal_control.gimbal_yaw_motor.absolute_angle - gimbal_control.gimbal_yaw_motor.absolute_angle_offset);
}

fp32 get_gimbal_relative_pitch_angle(void)
{
    return (gimbal_control.gimbal_pitch_motor.absolute_angle - gimbal_control.gimbal_pitch_motor.absolute_angle_offset);
}

fp32 get_gimbal_ecd_yaw_angle(void)
{
    return gimbal_control.gimbal_yaw_motor.relative_angle;
}

/**
  * @brief  Yaw angle of the gimbal's POINTING direction relative to the chassis front.
  *         For the dual-yaw hero the launcher points at the primary yaw angle PLUS the
  *         secondary offset q2, so the chassis must use this sum (not just the primary
  *         encoder) to drive where the gimbal is actually aimed. Identical to the primary
  *         relative angle on single-yaw robots.
  */
fp32 get_gimbal_yaw_to_chassis_angle(void)
{
#if (ROBOT_TYPE == HERO_2026_OMNI)
    return gimbal_control.gimbal_yaw_motor.relative_angle + gimbal_control.gimbal_second_yaw_motor.relative_angle;
#else
    return gimbal_control.gimbal_yaw_motor.relative_angle;
#endif
}

fp32 get_gimbal_ecd_pitch_angle(void)
{
    return gimbal_control.gimbal_pitch_motor.relative_angle;
}

#if (ROBOT_TYPE == INFANTRY_2026_MECANUM)

void MIT_motor_set_torq(gimbal_control_t *control_loop)
{
#if PITCH_REVERSED
    control_loop->MIT_control_motor.pitch_MIT_variable.torq = -control_loop->gimbal_pitch_motor.cmd_value;
#else
    control_loop->MIT_control_motor.pitch_MIT_variable.torq = control_loop->gimbal_pitch_motor.cmd_value;
#endif
}

void foldable_pitch_control(gimbal_control_t *gimbal_control_set)
{
    if (gimbal_control_set == NULL)
    {
        return;
    }

    MIT_control_motor_t* MIT_control_motor = &gimbal_control_set->MIT_control_motor;
    static uint8_t fFoldFilterInit = 0;
    static uint8_t bLastFoldTarget = UNFOLDED;
    static fp32 pitch_base_pos_cmd_filtered = 0.0f;
    static fp32 pitch_pos_cmd_filtered = 0.0f;
    static uint32_t folded_start_tick = 0;
    static uint8_t folded_disable_done = 0;
    static uint8_t last_fold_target = UNFOLDED;
    static uint8_t last_fold_step = 0;
    static fp32 fold_base_start_angle = 0.0f;

    fp32 pitch_motor_angle = motor_chassis[MOTOR_INDEX_PITCH].output_angle; //both have to be unprocessed motor feedback angle
    fp32 pitch_base_motor_angle = motor_chassis[MOTOR_INDEX_PITCH_BASE].output_angle;
    fp32 pitch_base_motor_vel = motor_chassis[MOTOR_INDEX_PITCH_BASE].velocity;
    fp32 pitch_base_pos_cmd_target = pitch_base_motor_angle;
    fp32 pitch_pos_cmd_target = pitch_motor_angle;

    uint8_t target_changed = (!fFoldFilterInit) || (bLastFoldTarget != gimbal_control_set->gimbal_folding_status.target);

    if (target_changed)
    {
        pitch_base_pos_cmd_filtered = pitch_base_motor_angle;
        pitch_pos_cmd_filtered = pitch_motor_angle;
        fFoldFilterInit = 1;
        bLastFoldTarget = gimbal_control_set->gimbal_folding_status.target;
    }

    //target=1 for center gimbal first and fold, target=0 for unfold
    if(gimbal_control_set->gimbal_folding_status.target == FOLDED) // fold cmd, target=folded
    {
        MIT_motor_angle_control_config(MIT_control_motor);                  // config for fold/unfold
        fp32 pitch_max_rel_target = gimbal_control_set->gimbal_pitch_motor.max_relative_angle;
        if (target_changed)
        {
            //ensure the correct step base on current angle TODO:improve this logic or pact as function
            if (pitch_base_motor_angle > PITCH_BASE_HALF_FOLD_POS)
            {
                gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 0;
            }
            else if (pitch_base_motor_angle > PITCH_BASE_FOLD_POS)
            {
                gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 2;
            }
            else
            {
                gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 3;
            }
        }
        //folding control
        if (gimbal_control_set->gimbal_folding_status.gimbal_folding_step != last_fold_step)
        {
            if (gimbal_control_set->gimbal_folding_status.gimbal_folding_step == 1)
            {
                fold_base_start_angle = pitch_base_motor_angle;
            }
            last_fold_step = gimbal_control_set->gimbal_folding_status.gimbal_folding_step;
        }
        switch (gimbal_control_set->gimbal_folding_status.gimbal_folding_step)
        {
            case 0:
            {
                pitch_base_pos_cmd_target = pitch_base_motor_angle;
                pitch_pos_cmd_target = pitch_max_rel_target;
                if (fabs(pitch_motor_angle - pitch_max_rel_target) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND)
                {
                    gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 1;
                }
                break;
            }
            case 1:
            {
                pitch_base_pos_cmd_target = PITCH_BASE_HALF_FOLD_POS; // fold to half first 
                if ((fabs(pitch_base_motor_angle - PITCH_BASE_HALF_FOLD_POS) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND) && (fabs(pitch_motor_angle - (-PITCH_BASE_HALF_FOLD_POS)) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND))
                {
                    gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 2;
                }
                break;
            }
            case 2:
            {
                pitch_base_pos_cmd_target = PITCH_BASE_FOLD_POS;
                if ((fabs(pitch_base_motor_angle - PITCH_BASE_FOLD_POS) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND) && (fabs(pitch_motor_angle - (-PITCH_BASE_FOLD_POS)) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND))
                {
                    gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 3;
                }
                break;
            }
            case 3:
            {
                pitch_base_pos_cmd_target = PITCH_BASE_FULLY_FOLD_POS;
                if ((fabs(pitch_base_motor_angle - PITCH_BASE_FULLY_FOLD_POS) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND) && (fabs(pitch_motor_angle - (-PITCH_BASE_FULLY_FOLD_POS)) <= GIMBAL_FOLD_ZERO_FORCE_DEADBAND))
                {
                    gimbal_control_set->gimbal_folding_status.current = FOLDED;
                }
                break;
            }
            
            }
        MIT_control_motor->pitch_base_MIT_variable.torq = -1.7f * sinf(pitch_base_motor_angle); //negative to counter gravity gain
        
        //pitch
        if(gimbal_control_set->gimbal_folding_status.gimbal_folding_step == 3)
        {
            //pitch_pos_cmd_target = -(pitch_base_motor_angle) + 0.2f - PITCH_FOLD_UP_BIAS; // adjust up to avoid armor plate contact
            pitch_pos_cmd_target = -(pitch_base_motor_angle) + 0.2f; // adjust up to avoid armor plate contact
        }
        else if (gimbal_control_set->gimbal_folding_status.gimbal_folding_step == 1)
        {
            fp32 denom = (PITCH_BASE_HALF_FOLD_POS - fold_base_start_angle);
            fp32 blend = (denom != 0.0f) ? ((pitch_base_motor_angle - fold_base_start_angle) / denom) : 1.0f;
            blend = fp32_constrain(blend, 0.0f, 1.0f);
            pitch_pos_cmd_target = (1.0f - blend) * pitch_max_rel_target + blend * (-(pitch_base_motor_angle));
        }
        else if (gimbal_control_set->gimbal_folding_status.gimbal_folding_step != 0)
        {
            //pitch_pos_cmd_target = -(pitch_base_motor_angle) - PITCH_FOLD_UP_BIAS;
            pitch_pos_cmd_target = -(pitch_base_motor_angle);
        }
        if (gimbal_control_set->gimbal_folding_status.gimbal_folding_step == 0)
        {
            MIT_control_motor->pitch_MIT_variable.vel = 0.0f;
        }
        else
        {
            MIT_control_motor->pitch_MIT_variable.vel = -(pitch_base_motor_vel); //reverse conter base so vel also negative
        }
        
    }
    else//unfold
    {
        folded_start_tick = 0;
        folded_disable_done = 0;
        gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 0;
        last_fold_step = 0;
        //pitch_base
        pitch_base_pos_cmd_target = PITCH_BASE_UNFOLD_POS; // TODO:check actual tgt fold angle
        MIT_control_motor->pitch_base_MIT_variable.torq = -1.7f * sinf(pitch_base_motor_angle); //negative to counter gravity gain
        
        //pitch
        pitch_pos_cmd_target = -(pitch_base_motor_angle);
        MIT_control_motor->pitch_MIT_variable.vel = -(pitch_base_motor_vel);
        

        if(pitch_base_motor_angle >= PITCH_BASE_HALF_FOLD_POS) //if reached half range start to reset pitch to level
        {
            pitch_pos_cmd_target = PITCH_UNFOLD_POS; //return to level after base fully unfolded
            if(pitch_motor_angle >= GIMBAL_FOLD_ZERO_FORCE_DEADBAND)
            {
                MIT_motor_torque_control_config(MIT_control_motor); //unfold finished, config back for pitch motor torque control
                //gimbal_control_set->gimbal_folding_status.gimbal_folding_step = 0;
                gimbal_control_set->gimbal_folding_status.current = UNFOLDED;
            }
        }
    }

    if (gimbal_control_set->gimbal_folding_status.target == FOLDED && last_fold_target == UNFOLDED)
    {
        folded_start_tick = osKernelSysTick();
        folded_disable_done = 0;
    }

    if (gimbal_control_set->gimbal_folding_status.target == FOLDED)
    {
        if (!folded_disable_done && ((osKernelSysTick() - folded_start_tick) >= 2000U))
        {
            enable_DaMiao_motor(CAN_PITCH_BASE_MOTOR_4310_TX_ID, 0, &GIMBAL_CAN);
            enable_DaMiao_motor(CAN_PITCH_MOTOR_4310_TX_ID, 0, &GIMBAL_CAN);
            enable_DaMiao_motor(CAN_YAW_MOTOR_4310_TX_ID, 0, &CHASSIS_CAN);
            folded_disable_done = 1;
        }
    }
    else
    {
        folded_start_tick = 0;
        folded_disable_done = 0;
    }
    last_fold_target = gimbal_control_set->gimbal_folding_status.target;

    {
        fp32 pitch_base_pos_cmd_next = first_order_filter(pitch_base_pos_cmd_target, pitch_base_pos_cmd_filtered, FOLD_POS_FILTER_COEFF);
        fp32 pitch_pos_cmd_next = first_order_filter(pitch_pos_cmd_target, pitch_pos_cmd_filtered, FOLD_POS_FILTER_COEFF);

        pitch_base_pos_cmd_filtered += fp32_abs_constrain(pitch_base_pos_cmd_next - pitch_base_pos_cmd_filtered, FOLD_BASE_POS_MAX_STEP);
        pitch_pos_cmd_filtered += fp32_abs_constrain(pitch_pos_cmd_next - pitch_pos_cmd_filtered, FOLD_PITCH_POS_MAX_STEP);
    }
	MIT_control_motor->pitch_base_MIT_variable.pos = pitch_base_pos_cmd_filtered;
	MIT_control_motor->pitch_MIT_variable.pos = pitch_pos_cmd_filtered;
}
#endif
