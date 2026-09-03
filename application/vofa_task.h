#ifndef VOFA_H
#define VOFA_H
#include "usart.h"
#include "main.h"
#include "global_inc.h"
#include "math.h"
#include "string.h"
#include  "cmsis_os.h"

#if (VOFA_UART_USE == 1)
#define VOFA_UART_AD (&huart6)
#define VOFA_UART (huart6)
#elif (VOFA_UART_USE == 2)
#define VOFA_UART_AD (&huart1)
#define VOFA_UART (huart1)
#endif

typedef struct {
    fp32 data[8];
    char tail[4];
} DataPacket;

void vofa_send(void);

fp32 get_data(void);
uint8_t get_place(void);

extern fp32 vofa_return_data(uint8_t place);
extern void vofa_update(uint8_t place, fp32 data);

extern void vofa_task(void const * argument);
#endif
