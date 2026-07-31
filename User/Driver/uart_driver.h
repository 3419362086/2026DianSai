#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "ringbuffer.h"

#define BUFFER_SIZE 256 // ��������С

#define UART6_RING_BUFFER_SIZE 512U // USART6 camera RX ring buffer

void Uart_Tx_Init(void);
int Uart_Printf(UART_HandleTypeDef *huart, const char *format, ...);  
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


#endif
