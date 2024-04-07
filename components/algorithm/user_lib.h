#ifndef USER_LIB_H
#define USER_LIB_H
#include "arm_math.h"
#include "struct_typedef.h"

/** A compile time assertion check.
 *
 *  Validate at compile time that the predicate is true without
 *  generating code. This can be used at any point in a source file
 *  where typedef is legal.
 *
 *  On success, compilation proceeds normally.
 *
 *  On failure, attempts to typedef an array type of negative size. The
 *  offending line will look like
 *      typedef assertion_failed_file_h_42[-1]
 *  where file is the content of the second parameter which should
 *  typically be related in some obvious way to the containing file
 *  name, 42 is the line number in the file on which the assertion
 *  appears, and -1 is the result of a calculation based on the
 *  predicate failing.
 *
 *  \param predicate The predicate to test. It must evaluate to
 *  something that can be coerced to a normal C boolean.
 *
 *  \param file A sequence of legal identifier characters that should
 *  uniquely identify the source file in which this condition appears.
 */
#define STATIC_ASSERT(predicate) _impl_CASSERT_LINE(predicate, __LINE__, __FILE__)
#define _impl_PASTE(a, b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
	typedef char _impl_PASTE(assertion_failed_##file##_, line)[2 * !!(predicate)-1];

/******************************* Constants *******************************/
#define MOVING_AVERAGE_RESET 1
#define MOVING_AVERAGE_CALC 0

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8192
#define MOTOR_RAD_TO_ECD 1303.7972938088067f // 8192/(2*PI)
#define MOTOR_ECD_TO_RAD 0.000766990394f     //      2*PI/8192

#define G_gravity 9.81f

#define DRIVE_WHEEL_RADIUS 0.0675f

/******************************* Robot control enables *******************************/
// switches between original, RM, capstone models
#define MODEL_ORIG_RM_CAP 1

// reverse hip motor direction
#define REVERSE_LB_HIP_MOTOR_DIRECTION 0
#define REVERSE_LF_HIP_MOTOR_DIRECTION 1
#define REVERSE_RB_HIP_MOTOR_DIRECTION 1
#define REVERSE_RF_HIP_MOTOR_DIRECTION 0
#define REVERSE_LEFT_DRIVE_MOTOR_DIRECTION 1
#define REVERSE_RIGHT_DRIVE_MOTOR_DIRECTION 0

/******************************* Robot control configs *******************************/
#define NORMAL_MAX_CHASSIS_SPEED_X 2.5f // chassis forward or back max speed
#define NORMAL_MAX_CHASSIS_SPEED_YAW 2.5f
#define MAX_CHASSIS_ROLL (10.0f / 180.0f * PI)

#define MOTOR_TORQUE_CLEARANCE 0.2f
#define HIP_TORQUE_BURST_MAX (35.0f - MOTOR_TORQUE_CLEARANCE)
// #define HIP_TORQUE_MAX 8.0f
#define HIP_TORQUE_MAX 3.0f
#define DRIVE_TORQUE_MAX 3.0f

#define UNLOADED_ROBOT_MASS 5.0f                                            // unit is kg
#define UNLOADED_ROBOT_HALF_WEIGHT (UNLOADED_ROBOT_MASS / 2.0f * G_gravity) // unit is N

#if (MODEL_ORIG_RM_CAP == 0)
#define LEG_L0_MIN 0.2f
#define LEG_L0_MAX 0.35f
#elif ((MODEL_ORIG_RM_CAP == 1) || (MODEL_ORIG_RM_CAP == 2))
#define LEG_L0_MIN 0.15f
#define LEG_L0_MAX 0.34f
#endif
#define LEG_L0_MID ((LEG_L0_MAX + LEG_L0_MIN) / 2.0f)
#define LEG_L0_RANGE (LEG_L0_MAX - LEG_L0_MIN)

#define LEG_L0_MIN_THRESHOLD (LEG_L0_MIN + LEG_L0_RANGE * 0.2f)
#define LEG_L0_MAX_THRESHOLD (LEG_L0_MAX - LEG_L0_RANGE * 0.2f)

/**
 * @brief          remote control dealline solve,because the value of rocker is not zero in middle place,
 * @param          input:the raw channel value
 * @param          output: the processed channel value
 * @param          deadline
 */
#define deadband_limit(input, output, deadline)            \
	{                                                      \
		if ((input) > (deadline) || (input) < -(deadline)) \
		{                                                  \
			(output) = (input);                            \
		}                                                  \
		else                                               \
		{                                                  \
			(output) = 0;                                  \
		}                                                  \
	}

typedef __packed struct
{
	fp32 input;        // 输入数据
	fp32 out;          // 输出数据
	fp32 min_value;    // 限幅最小值
	fp32 max_value;    // 限幅最大值
	fp32 frame_period; // 时间间隔
} ramp_function_source_t;

typedef __packed struct
{
	fp32 input;        // 输入数据
	fp32 out;          // 滤波输出的数据
	fp32 num[1];       // 滤波参数
	fp32 frame_period; // 滤波的时间间隔 单位 s
} first_order_filter_type_t;

typedef struct
{
	uint8_t size;
	uint8_t cursor;
	fp32 *ring;
	fp32 sum;
} moving_average_type_t;
// 快速开方
extern fp32 invSqrt(fp32 num);

// 斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

// 斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
// 一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
// 一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
// moving average
extern fp32 moving_average_calc(fp32 input, moving_average_type_t *moving_average_type, uint8_t fInit);
// 浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
// int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
// 限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
// 限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
// 角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);
extern uint8_t matrixMultiplication(uint8_t m1_rows, uint8_t m1_cols, uint8_t m2_rows, uint8_t m2_cols, fp32 m1[m1_rows][m1_cols], fp32 m2[m2_rows][m2_cols], fp32 result[m1_rows][m2_cols]);
extern void LimitMax(fp32 *num, fp32 Limit);
extern fp32 first_order_filter(fp32 input, fp32 prev_output, fp32 coeff);
extern fp32 second_order_filter(fp32 input, fp32 output_prev1, fp32 output_prev2, fp32 coeff1, fp32 coeff2);
fp32 brakezone(fp32 input, fp32 threshold, fp32 order);
fp32 brakezone_symmetric(fp32 input, fp32 threshold, fp32 order);

// 弧度格式化为-PI~PI
#ifndef rad_format
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)
#endif /* rad_format */

#ifndef Limit
#define Limit(x, max, min) ((x) > (MAX(min, max)) ? (MAX(min, max)) : ((x) < (MIN(min, max)) ? (MIN(min, max)) : (x)))
#endif /* Limit */

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef ABS
#define ABS(x) ((x) > 0 ? (x) : (-x))
#endif /* ABS */

#ifndef SIGN
#define SIGN(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#endif /* SIGN */

#ifndef Deadzone
#define Deadzone(input, threshold) ((fabs(input) < threshold) ? 0 : input)
#endif /* Deadzone */

#endif /* USER_LIB_H */
