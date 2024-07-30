#ifndef VOFA_H
#define VOFA_H
#include "usart.h"
#include "main.h"
#include "global_inc.h"

typedef struct {
    float data[8];
    char tail[4];
} DataPacket;

void vofa_send(void);
void vofa_receive(void);
fp32 vofa_data_get(uint8_t place);
void vofa_update(uint8_t place, fp32 data);
#endif
