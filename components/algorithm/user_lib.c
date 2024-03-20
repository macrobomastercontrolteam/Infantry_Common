#include "user_lib.h"
#include <math.h>

extern int time;

#include "arm_math.h"
#include "cmsis_os.h" // For xTaskGetTickCount()

//快速开方
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
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
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
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
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
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

/**
  * @brief          Moving average
  * @author         2022 MacFalcons
  * @param[in]      input
  * @param[in]      handler
  * @retval         average
  */
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

//判断符号位
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

//浮点死区
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
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

//弧度格式化为-PI~PI

//角度格式化为-180~180
fp32 theta_format(fp32 Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

// Static Arr Implementation

// Filling the array
void fill_buffer(fp32 input_angle, circular_buffer_t *circular_buffer)
{
    if (circular_buffer->current_buffer_index != circular_buffer->buffer_size - 1)
    {
        circular_buffer->angle_buffer[circular_buffer->current_buffer_index] = input_angle;
        circular_buffer->current_buffer_index++;
    }
    else if (circular_buffer->current_buffer_index == circular_buffer->buffer_size - 1)
    {
        circular_buffer->buffer_full_flag = 1;
        circular_buffer->angle_buffer[circular_buffer->current_buffer_index] = input_angle;
        circular_buffer->current_buffer_index = 0;
    }
}

// Accessing Angle
fp32 access_angle(uint16_t target_timestamp, circular_buffer_t *circular_buffer)
{
    uint16_t signal_interval = 1;
    uint16_t current_time = time; //xTaskGetTickCount();
    float time_delta = current_time - target_timestamp;
    int n = (int)(fabs(time_delta) / (float)signal_interval);
    int time_before = current_time - signal_interval * (n + 1);

    if ((n + 1) < circular_buffer->current_buffer_index)
    {
        float angle_after = circular_buffer->angle_buffer[circular_buffer->current_buffer_index - (n + 1)];
        float angle_before = circular_buffer->angle_buffer[circular_buffer->current_buffer_index - (n + 2)];
        return (angle_before + (target_timestamp - time_before) * (angle_after - angle_before) / (signal_interval));
    }
    else if ((n + 1) == circular_buffer->current_buffer_index)
    {
        if (circular_buffer->buffer_full_flag)
        {
            float angle_after = circular_buffer->angle_buffer[circular_buffer->current_buffer_index - (n + 1)];
            float angle_before = circular_buffer->angle_buffer[circular_buffer->current_buffer_index - (n + 2) + circular_buffer->buffer_size];
            return (angle_before + (target_timestamp - time_before) * (angle_after - angle_before) / (signal_interval));
        }
        else
        {
            return circular_buffer->angle_buffer[0];
        }
    }
    else if ((n + 1) > circular_buffer->current_buffer_index)
    {
        if ((n + 2) <= circular_buffer->buffer_size)
        {
            if (circular_buffer->buffer_full_flag)
            {
                float angle_after = circular_buffer->angle_buffer[circular_buffer->current_buffer_index - (n + 1) + circular_buffer->buffer_size];
                float angle_before = circular_buffer->angle_buffer[circular_buffer->current_buffer_index - (n + 2) + circular_buffer->buffer_size];
                return (angle_before + (target_timestamp - time_before) * (angle_after - angle_before) / (signal_interval));
            }
            else
            {
                return circular_buffer->angle_buffer[0];
            }
        }
        else
        {
            if (circular_buffer->buffer_full_flag)
            {
                return circular_buffer->angle_buffer[circular_buffer->current_buffer_index]; // current_buffer_index is 1 past the index of the latest datapoint so no need to add 1, also current_buffer_index < BUFF_SIZE so no modulo needed
            }
            else
            {
                return circular_buffer->angle_buffer[0];
            }
        }
    }
    return -1; // Only because return required at end of function
}

uint8_t checkAndResetFlag(uint8_t *pbFlag)
{
    uint8_t temp = *pbFlag;
    *pbFlag = 0;
    return temp;
}
