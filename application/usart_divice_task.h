#ifndef USART_DIVICE_TASK_H
#define USART_DIVICE_TASK_H
#include "usart.h"
#include "main.h"
#include "global_inc.h"
#include "math.h"
#include "string.h"
#include  "cmsis_os.h"


void uart_init(void);
void uart_send(void);
bool_t data_check(void);
void data_update(void);
extern uint32_t get_distence(void);
extern void usart_divice_task(void const * argument);

#endif
