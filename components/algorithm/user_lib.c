#include "user_lib.h"
#include "arm_math.h"

fp32 first_order_filter(fp32 input, fp32 output_prev, fp32 coeff)
{
    if (coeff > 1)
    {
        return NAN;
    }
    fp32 output = (1-coeff)*output_prev + coeff*input;
    return output;
}

// fast inverse square root
fp32 invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
    * @brief          Ramp function initialization
    * @author         RM
    * @param[in]      Ramp function structure
    * @param[in]      Time interval, in seconds
    * @param[in]      Maximum value
    * @param[in]      Minimum value
    * @retval         None
    */
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
    * @brief          Ramp function calculation, adds the input value based on the given rate, input unit is /s (per second)
    * @param[in]      ramp function structure
    * @param[in]      input value
    * @param[in]      filter parameter
    * @retval         None
    */
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
    * @brief          First-order low-pass filter initialization
    * @author         RM
    * @param[in]      First-order low-pass filter structure
    * @param[in]      Time interval, in seconds
    * @param[in]      Filter parameter
    * @retval         None
    */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
    * @brief          First-order low-pass filter calculation
    * @author         RM
    * @param[in]      First-order low-pass filter structure
    * @param[in]      Time interval, in seconds
    * @retval         None
    */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

fp32 moving_average_calc(fp32 input, moving_average_type_t* moving_average_type, uint8_t fInit)
{
    fp32 output;
    if (fInit == MOVING_AVERAGE_RESET)
    {
        moving_average_type->sum = input * moving_average_type->size;
        for (uint8_t i = 0; i < (moving_average_type->size); i++)
        {
            moving_average_type->ring[i] = input;
        }
        moving_average_type->cursor = 0;
        output = input;
    }
    else
    {
        // history[cursor] is the current oldest history in the ring
        moving_average_type->sum = moving_average_type->sum - moving_average_type->ring[moving_average_type->cursor] + input;
        moving_average_type->ring[moving_average_type->cursor] = input;
        moving_average_type->cursor = (moving_average_type->cursor + 1) % moving_average_type->size;
        output = (moving_average_type->sum) / ((fp32)(moving_average_type->size));
    }
    return output;
}

// determine the sign of the input value
fp32 sign(fp32 value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

void fp32_deadzone(fp32* in, fp32 deadzone)
{
    if (fabs(*in) < deadzone)
    {
        *in = 0;
    }
}

int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

fp32 fp32_abs_constrain(fp32 in, fp32 absValue)
{
    return fp32_constrain(in, -absValue, absValue);
}

int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

uint8_t checkAndResetFlag(uint8_t *pbFlag)
{
    uint8_t temp = *pbFlag;
    *pbFlag = 0;
    return temp;
}

float MG6012_loop_ecd_constrain(float Input)
{
	if (Input > MG6012_ECD_RANGE_180)
	{
		while (Input > MG6012_ECD_RANGE_180)
		{
			Input -= MG6012_ECD_RANGE;
		}
	}
	else if (Input < -MG6012_ECD_RANGE_180)
	{
		while (Input < -MG6012_ECD_RANGE_180)
		{
			Input += MG6012_ECD_RANGE;
		}
	}
	return Input;
}

float M6020_loop_ecd_constrain(float Input)
{
	if (Input > M6020_ECD_RANGE_180)
	{
		while (Input > M6020_ECD_RANGE_180)
		{
			Input -= M6020_ECD_RANGE;
		}
	}
	else if (Input < -M6020_ECD_RANGE_180)
	{
		while (Input < -M6020_ECD_RANGE_180)
		{
			Input += M6020_ECD_RANGE;
		}
	}
	return Input;
}
