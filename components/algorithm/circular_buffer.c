typedef float fp32;

#include <stdint.h>
#include <math.h>
#include <stdio.h>

typedef struct
{
    fp32 *angle_buffer;            // Initializing space for static list
    uint16_t buffer_size;          // Size of the buffer
    uint16_t current_buffer_index; // Current index at fill position
    uint8_t buffer_full_flag;      // Flag to indicate filled arrays
} circular_buffer_t;

typedef struct
{
    circular_buffer_t yaw_angle;
    circular_buffer_t pitch_angle;
} gimbal_control_t;

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
    uint16_t signal_interval = 50;
    uint16_t current_time = 500;
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

gimbal_control_t gimbal_control_1;
fp32 buffer_yaw[10];
fp32 buffer_pitch[10];

int main() {
    gimbal_control_1.yaw_angle.angle_buffer = buffer_yaw;
    gimbal_control_1.pitch_angle.angle_buffer = buffer_pitch;

    gimbal_control_1.pitch_angle.buffer_size = 10;
    gimbal_control_1.pitch_angle.current_buffer_index = 0;
    gimbal_control_1.pitch_angle.buffer_full_flag = 0;

    gimbal_control_1.yaw_angle.buffer_size = 10;
    gimbal_control_1.yaw_angle.current_buffer_index = 0;
    gimbal_control_1.yaw_angle.buffer_full_flag = 0;

    fill_buffer(0.5, &gimbal_control_1.yaw_angle);
    fill_buffer(1.5, &gimbal_control_1.yaw_angle);
    fill_buffer(2.5, &gimbal_control_1.yaw_angle);
    fill_buffer(3.5, &gimbal_control_1.yaw_angle);
    fill_buffer(4.5, &gimbal_control_1.yaw_angle);
    fill_buffer(5.5, &gimbal_control_1.yaw_angle);
    fill_buffer(6.5, &gimbal_control_1.yaw_angle);
    fill_buffer(7.5, &gimbal_control_1.yaw_angle);
    fill_buffer(8.5, &gimbal_control_1.yaw_angle);
    fill_buffer(9.5, &gimbal_control_1.yaw_angle);

    printf("%f\n", access_angle(500, &gimbal_control_1.yaw_angle));
}