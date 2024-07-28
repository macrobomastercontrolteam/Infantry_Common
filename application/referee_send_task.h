#ifndef _REFEREE_SEND_TASK_H
#define _REFEREE_SEND_TASK_H

typedef struct __attribute__((packed))
{
    struct __attribute__((packed))
    {
        uint8_t sof;
        uint16_t data_length;
        uint8_t seq;
        uint8_t crc8;
    } frame_header;
    uint16_t cmd_id;
    uint8_t data[30];
    uint16_t frame_tail;
} Controller_t;

void referee_send_task(void const *argument);

#endif
