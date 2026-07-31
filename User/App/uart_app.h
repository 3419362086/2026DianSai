#ifndef __UART_APP_H__
#define __UART_APP_H__

#include "MyDefine.h"

#define DEBUG_UART &huart1
#define wireless_UART &huart5

void Uart_Init(void);
void Uart1_Task(void);
void Uart2_Task(void);
void Uart4_Task(void);
void Uart5_Task(void);
void Uart6_Task(void);
void System_State_Uart_Print(void);


/* 串口 1 */
extern uint8_t uart1_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

extern uint8_t uart1_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
extern struct rt_ringbuffer uart1_ring_buffer; // 环形缓冲区

extern uint8_t uart1_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 2 */
extern uint8_t uart2_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

extern uint8_t uart2_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
extern struct rt_ringbuffer uart2_ring_buffer; // 环形缓冲区

extern uint8_t uart2_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 4 */
extern uint8_t uart4_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

extern uint8_t uart4_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
extern struct rt_ringbuffer uart4_ring_buffer; // 环形缓冲区

extern uint8_t uart4_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 5 */
extern uint8_t uart5_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

extern uint8_t uart5_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
extern struct rt_ringbuffer uart5_ring_buffer; // 环形缓冲区

extern uint8_t uart5_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 6 */
extern uint8_t uart6_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

extern uint8_t uart6_ring_buffer_input[UART6_RING_BUFFER_SIZE]; // 环形缓冲区对应的线性数组
extern struct rt_ringbuffer uart6_ring_buffer; // 环形缓冲区

extern uint8_t uart6_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区


#endif
