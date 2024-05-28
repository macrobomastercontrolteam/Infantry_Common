#ifndef USER_LIB_H
#define USER_LIB_H
#include "global_inc.h"
#include "arm_math.h"

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
#define STATIC_ASSERT(predicate) _impl_CASSERT_LINE(predicate,__LINE__,__FILE__)
#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
    typedef char _impl_PASTE(assertion_failed_##file##_,line)[2*!!(predicate)-1];

#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8192
#define MOTOR_RAD_TO_ECD ((fp32)(HALF_ECD_RANGE) / PI)
#define MOTOR_ECD_TO_RAD (PI / (fp32)(HALF_ECD_RANGE))

#define HALF_ECD_RANGE_SHRINKED 128
#define ECD_RANGE_SHRINKED 256
#define MOTOR_RAD_TO_ECD_SHRINKED ((fp32)(HALF_ECD_RANGE_SHRINKED) / PI)
#define MOTOR_ECD_TO_RAD_SHRINKED (PI / (fp32)(HALF_ECD_RANGE_SHRINKED))

#define MOVING_AVERAGE_RESET 1
#define MOVING_AVERAGE_CALC 0

/**
  * @brief          remote control dealline solve,because the value of joystick is not zero in middle place,
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

#define brakeband_limit(input, output, deadline)                                                                                \
    {                                                                                                                           \
        if ((input) > (deadline) || (input) < -(deadline))                                                                      \
        {                                                                                                                       \
            (output) = (input);                                                                                                 \
        }                                                                                                                       \
        else                                                                                                                    \
        {                                                                                                                       \
            (output) = ((input) * (input) * (input) * (input) * (input)) / ((deadline) * (deadline) * (deadline) * (deadline)); \
        }                                                                                                                       \
    }

#define RPM_TO_RADS(_ROUND_PER_MIN) (_ROUND_PER_MIN * 2.0f * PI / 60.0f)

typedef __packed struct
{
    fp32 input;        // Input data
    fp32 out;          // Output data
    fp32 min_value;    // Minimum limit value
    fp32 max_value;    // Maximum limit value
    fp32 frame_period; // Time interval
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        // Input data
    fp32 out;          // Output data after filtering
    fp32 num[1];       // Filter parameters
    fp32 frame_period; // Time interval for filtering in seconds
} first_order_filter_type_t;

typedef struct
{
    uint8_t size;
    uint8_t cursor;
    fp32 *ring;
    fp32 sum;
} moving_average_type_t;

void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
extern fp32 moving_average_calc(fp32 input, moving_average_type_t* moving_average_type, uint8_t fInit);
extern fp32 sign(fp32 value);
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
uint8_t checkAndResetFlag(uint8_t *pbFlag);
fp32 first_order_filter(fp32 input, fp32 prev_output, fp32 coeff);

extern fp32 invSqrt(fp32 num);
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
extern fp32 theta_format(fp32 Ang);

#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#define DEG_TO_RAD(_deg) (_deg * PI / 180.0f)

#endif
