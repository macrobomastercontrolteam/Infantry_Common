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

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "global_inc.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"
#include "user_lib.h"

#define GIMBAL_CONTROL_TIME_MS 3.0f
#define GIMBAL_CONTROL_TIME_S (GIMBAL_CONTROL_TIME_MS / 1000.0f)

//test mode, 0 close, 1 open
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0

#define int_abs(x) ((x) > 0 ? (x) : (-x))

#if (ROBOT_TYPE == INFANTRY_2023_MECANUM) || (ROBOT_TYPE == INFANTRY_2024_MECANUM)
//pitch speed close-loop PID params, max out and max iout
#define PITCH_SPEED_PID_KP        10000.0f // pitch starts shaking at 2600
#define PITCH_SPEED_PID_KI        100000.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  5000.0f

//yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP        27000.0f
#define YAW_SPEED_PID_KI        11000.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
#define PITCH_ANGLE_PID_KP 30.0f
#define PITCH_ANGLE_PID_KI 0.0f
#define PITCH_ANGLE_PID_KD 0.0f
#define PITCH_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_ANGLE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
#define YAW_ANGLE_PID_KP        23.0f
#define YAW_ANGLE_PID_KI        0.0f
#define YAW_ANGLE_PID_KD        0.0f
#define YAW_ANGLE_PID_MAX_OUT   10.0f
#define YAW_ANGLE_PID_MAX_IOUT  0.0f

#elif (ROBOT_TYPE == SENTRY_2023_MECANUM)

//pitch speed close-loop PID params, max out and max iout
#define PITCH_SPEED_PID_KP        10000.0f // pitch starts shaking at 2600
#define PITCH_SPEED_PID_KI        0.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP        20000.0f
#define YAW_SPEED_PID_KI        1000.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  10000.0f

//pitch gyro angle close-loop PID params, max out and max iout
#define PITCH_ANGLE_PID_KP 30.0f
#define PITCH_ANGLE_PID_KI 10.0f
#define PITCH_ANGLE_PID_KD 0.0f
#define PITCH_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_ANGLE_PID_MAX_IOUT 10.0f

//yaw gyro angle close-loop PID params, max out and max iout
#define YAW_ANGLE_PID_KP        10.0f
#define YAW_ANGLE_PID_KI        0.0f
#define YAW_ANGLE_PID_KD        0.0f
#define YAW_ANGLE_PID_MAX_OUT   10.0f
#define YAW_ANGLE_PID_MAX_IOUT  10.0f

#else

//pitch speed close-loop PID params, max out and max iout
#define PITCH_SPEED_PID_KP        2250.0f // pitch starts shaking at 2600
#define PITCH_SPEED_PID_KI        25000.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw speed close-loop PID params, max out and max iout
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20000.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
#define PITCH_ANGLE_PID_KP 20.0f
#define PITCH_ANGLE_PID_KI 0.0f
#define PITCH_ANGLE_PID_KD 0.0f
#define PITCH_ANGLE_PID_MAX_OUT 10.0f
#define PITCH_ANGLE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
#define YAW_ANGLE_PID_KP        26.0f
#define YAW_ANGLE_PID_KI        0.0f
#define YAW_ANGLE_PID_KD        0.3f
#define YAW_ANGLE_PID_MAX_OUT   10.0f
#define YAW_ANGLE_PID_MAX_IOUT  0.0f
#endif

#define PITCH_CAMERA_SPEED_PID_KP 10000.0f
#define PITCH_CAMERA_SPEED_PID_KI 0.0f
#define PITCH_CAMERA_SPEED_PID_KD 0.0f
#define PITCH_CAMERA_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_CAMERA_SPEED_PID_MAX_IOUT 10000.0f

#define YAW_CAMERA_SPEED_PID_KP 20000.0f
#define YAW_CAMERA_SPEED_PID_KI 1000.0f
#define YAW_CAMERA_SPEED_PID_KD 0.0f
#define YAW_CAMERA_SPEED_PID_MAX_OUT 30000.0f
#define YAW_CAMERA_SPEED_PID_MAX_IOUT 10000.0f

#define PITCH_CAMERA_ANGLE_PID_KP 30.0f
#define PITCH_CAMERA_ANGLE_PID_KI 10.0f
#define PITCH_CAMERA_ANGLE_PID_KD 0.0f
#define PITCH_CAMERA_ANGLE_PID_MAX_OUT 10.0f // max speed (rad/s)
#define PITCH_CAMERA_ANGLE_PID_MAX_IOUT 10.0f

#define YAW_CAMERA_ANGLE_PID_KP 10.0f
#define YAW_CAMERA_ANGLE_PID_KI 0.0f
#define YAW_CAMERA_ANGLE_PID_KD 0.0f
#define YAW_CAMERA_ANGLE_PID_MAX_OUT 10.0f // max speed (rad/s)
#define YAW_CAMERA_ANGLE_PID_MAX_IOUT 10.0f

//pitch encode angle close-loop PID params, max out and max iout
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

#define PITCH_MOTOR_CURRENT_LIMIT  30000
#define YAW_MOTOR_CURRENT_LIMIT  30000

#define GIMBAL_TASK_INIT_TIME 201

//turn 180 degrees
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
#define TURN_SPEED    0.04f
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//joystick value deadband
#define RC_DEADBAND   10

#if CV_INTERFACE
// #define CV_CAMERA_YAW_DEADBAND   0.0174533f // 1 degree
// #define CV_CAMERA_PITCH_DEADBAND   0.0174533f // 1 degree
#define CV_CAMERA_YAW_DEADBAND   0.0f
#define CV_CAMERA_PITCH_DEADBAND   0.0f
#define CV_ANGLE_FILTER_SIZE 10

// //camera control angle PID
// //Feedback: commanded pitch/yaw delta angle from cv interface, unit rad
// //Actuator: pitch/yaw delta angle, unit rad
// #define CV_CONTROL_ANGLE_PID_KP 5.0f
// #define CV_CONTROL_ANGLE_PID_KI 0.0f
// #define CV_CONTROL_ANGLE_PID_KD 0.1f
// #define CV_CONTROL_ANGLE_PID_MAX_OUT 100.0f
// #define CV_CONTROL_ANGLE_PID_MAX_IOUT 100.0f
#endif

/**
 * @brief Positive Directions
 * If Type-C board is facing up, and USB port to the front,
 * IMU yaw: CCW (cannon to the left)
 * IMU pitch: cannon down
 * 
 * If bottom of yaw and pitch GM6020 motors point towards main gimbal body
 * Encoder yaw: CCW
 * Encoder pitch: cannon down
 * 
 * Mouse x velocity: right
 * Mouse y velocity: down
 * 
 * Remote controller left joystick vertical (pitch): up
 * Remote controller left joystick horizontal: right
 * Remote controller right joystick vertical: up
 * Remote controller right joystick horizontal: left
 * 
 * Camera yaw angle: right
 * Camera pitch angle: up
 */
#define YAW_RC_CHANGE_TIME_S 1.0f
#define YAW_RC_SEN_INC -(PI / YAW_RC_CHANGE_TIME_S * GIMBAL_CONTROL_TIME_S / JOYSTICK_HALF_RANGE)
#define YAW_RC_MOUSE_SEN_INC -(PI / YAW_RC_CHANGE_TIME_S * GIMBAL_CONTROL_TIME_S / MOUSE_X_EFFECTIVE_SPEED)

#define PITCH_RC_CHANGE_TIME_S 0.4f
#define PITCH_RC_SEN_INC -(PI / 4.0f / PITCH_RC_CHANGE_TIME_S * GIMBAL_CONTROL_TIME_S / JOYSTICK_HALF_RANGE)
#define PITCH_RC_MOUSE_SEN_INC (PI / 4.0f / PITCH_RC_CHANGE_TIME_S * GIMBAL_CONTROL_TIME_S / MOUSE_Y_EFFECTIVE_SPEED)

#define GIMBAL_INIT_ANGLE_ERROR     0.05f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
#define GIMBAL_INIT_PITCH_SPEED     0.005f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

#define GIMBAL_CALI_MOTOR_SET   6000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#if(!ROBOT_YAW_HAS_SLIP_RING)
#define GIMBAL_CALI_END_STEP    5
#else
#define GIMBAL_CALI_END_STEP    3
#endif

#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

typedef enum
{
    GIMBAL_MOTOR_RAW = 0,
    GIMBAL_MOTOR_GYRO,
    GIMBAL_MOTOR_ENCODER,
    GIMBAL_MOTOR_CAMERA,  //GIMBAL_MOTOR_GYRO but with target angle adjusted by computer vision input (enemy angle within camera frame)
} gimbal_motor_mode_e;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_absolute_angle_pid;
    pid_type_def gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_speed_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 absolute_angle_offset; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    // fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

#if CV_INTERFACE
    moving_average_type_t CvCmdAngleFilter;
    fp32 CvCmdAngleFilterBuffer[CV_ANGLE_FILTER_SIZE];
#endif
} gimbal_motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */

extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME_MS (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
extern void gimbal_task(void const *pvParameters);

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
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

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
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

extern bool_t gimbal_emergency_stop(void);
extern gimbal_control_t gimbal_control;
#endif
