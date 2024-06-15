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

#include "main.h"

#include "cmsis_os.h"

#include "AHRS_middleware.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "bsp_laser.h"
#include "pid.h"
#include "cv_usart_task.h"

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
static void gimbal_safety_manager(int16_t *yaw_can_set_current_ptr, int16_t *pitch_can_set_current_ptr, int16_t *trigger_set_current_ptr, int16_t *fric1_set_current_ptr, int16_t *fric2_set_current_ptr);

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
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0, trigger_set_current = 0;

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
    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        osDelay(GIMBAL_CONTROL_TIME_MS);
        gimbal_feedback_update(&gimbal_control);
    }

    while (1)
    {
        gimbal_set_mode(&gimbal_control);
        gimbal_mode_change_control_transit(&gimbal_control);
        gimbal_feedback_update(&gimbal_control);
        gimbal_set_control(&gimbal_control);
        gimbal_control_loop(&gimbal_control);
        trigger_set_current = shoot_control_loop();
        gimbal_safety_manager(&yaw_can_set_current, &pitch_can_set_current, &trigger_set_current, &shoot_control.fric1_given_current, &shoot_control.fric2_given_current);
        CAN_cmd_gimbal(yaw_can_set_current, pitch_can_set_current, trigger_set_current, shoot_control.fric1_given_current, shoot_control.fric2_given_current);

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        osDelayUntil(&ulSystemTime, GIMBAL_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

void gimbal_safety_manager(int16_t *yaw_can_set_current_ptr, int16_t *pitch_can_set_current_ptr, int16_t *trigger_set_current_ptr, int16_t *fric1_set_current_ptr, int16_t *fric2_set_current_ptr)
{
    // safety for gimbal
    if (gimbal_emergency_stop() || toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        *yaw_can_set_current_ptr = 0;
        *pitch_can_set_current_ptr = 0;
    }
    else
    {
#if YAW_TURN
        *yaw_can_set_current_ptr = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        *yaw_can_set_current_ptr = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        *pitch_can_set_current_ptr = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        *pitch_can_set_current_ptr = gimbal_control.gimbal_pitch_motor.given_current;
#endif
    }

    // safety for shoot
    if (toe_is_error(TRIGGER_MOTOR_TOE) || toe_is_error(FRIC1_MOTOR_TOE) || toe_is_error(FRIC2_MOTOR_TOE))
    {
        *fric1_set_current_ptr = 0;
        *fric1_set_current_ptr = 0;
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

#if YAW_TURN
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

#if PITCH_TURN

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
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->gimbal_rc_ctrl = get_remote_control_point();
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    static const fp32 yaw_encode_relative_angle_pid[3] = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};
    static const fp32 pitch_encode_relative_angle_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, PID_POSITION, yaw_encode_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, 0, &rad_err_handler);
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


    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.absolute_angle_offset = 0;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
#if ENABLE_LASER
    laser_enable(1);
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
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);

#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
#endif
    feedback_update->gimbal_yaw_motor.motor_gyro = AHRS_cosf(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - AHRS_sinf(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
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
        case GIMBAL_MOTOR_RAW:
        {
            gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
            break;
        }
        case GIMBAL_MOTOR_GYRO:
        {
            // change pid parameters, which depends on motor control mode
            gimbal_yaw_abs_angle_PID_init(gimbal_mode_change);
            gimbal_yaw_pid_clear(gimbal_mode_change);
            gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
            break;
        }
        case GIMBAL_MOTOR_CAMERA:
        {
            // change pid parameters, which depends on motor control mode
            gimbal_yaw_abs_angle_PID_init(gimbal_mode_change);
            gimbal_yaw_pid_clear(gimbal_mode_change);
            gimbal_mode_change->gimbal_yaw_motor.absolute_angle_offset = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
            gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
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
        switch (gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode)
        {
        case GIMBAL_MOTOR_RAW:
        {
            gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
            break;
        }
        case GIMBAL_MOTOR_GYRO:
        {
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

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
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
    // if ((motor_select == GIMBAL_YAW_MOTOR) && ((chassis_move.chassis_mode == CHASSIS_VECTOR_SPINNING))
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
    
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if ((control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) || (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_CAMERA))
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if ((control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO) || (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_CAMERA))
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
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
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_speed_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set, GIMBAL_CONTROL_TIME_S);

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
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
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_speed_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set, GIMBAL_CONTROL_TIME_S);

    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
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
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
fp32 yaw_cv_cmd_fp32;
fp32 yaw_ins_fp32, pitch_ins_fp32;
fp32 yaw_ins_set_fp32, pitch_ins_set_fp32;
fp32 pitch_relative_set_fp32, pitch_relative_angle_fp32;
fp32 yaw_speed_fp32, pitch_speed_fp32;
fp32 yaw_speed_set_fp32, pitch_speed_set_fp32;
static void J_scope_gimbal_test(void)
{
#if CV_INTERFACE
    yaw_cv_cmd_fp32 = CvCmdHandler.CvCmdMsg.xAngle * 180.0f / PI;
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
    if (fFatalError)
    {
        // do nothing
    }
    else if ((int_abs(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->given_current) >= YAW_MOTOR_CURRENT_LIMIT) || (int_abs(gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->given_current) >= PITCH_MOTOR_CURRENT_LIMIT))
    {
        fFatalError = 1;
    }
    else
    {
        fEStop = (toe_is_error(DBUS_TOE) && toe_is_error(LOWER_HEAD_TOE));
    }
    return fEStop;
}
