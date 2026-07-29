#include "uart_app.h"
void Uart_Init(void)
{
  Uart_Printf(DEBUG_UART, "Uart_Init ......\r\n");
  
  /* 串口 1 */
  rt_ringbuffer_init(&uart1_ring_buffer, uart1_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 2 */
  rt_ringbuffer_init(&uart2_ring_buffer, uart2_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 4 */
  rt_ringbuffer_init(&uart4_ring_buffer, uart4_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 5 */
  rt_ringbuffer_init(&uart5_ring_buffer, uart5_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 6 */
  rt_ringbuffer_init(&uart6_ring_buffer, uart6_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
}

/* 串口 1 */
void Uart1_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart1_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart1_ring_buffer, uart1_data_buffer, uart_data_len);
    uart1_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART1 Ringbuffer:%s\r\n", uart1_data_buffer);
    Uart_Printf(&huart1, "UART1 Ringbuffer:%s\r\n", uart1_data_buffer);
    
    memset(uart1_data_buffer, 0, uart_data_len);
  }
}

/* 串口 2 */
void Uart2_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart2_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart2_ring_buffer, uart2_data_buffer, uart_data_len);
    uart2_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART2 Ringbuffer:%s\r\n", uart2_data_buffer);
    Uart_Printf(&huart2, "UART2 Ringbuffer:%s\r\n", uart2_data_buffer);
    
    memset(uart2_data_buffer, 0, uart_data_len);
  }
}

/* 串口 4 */
void Uart4_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart4_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart4_ring_buffer, uart4_data_buffer, uart_data_len);
    uart4_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART4 Ringbuffer:%s\r\n", uart4_data_buffer);
    Uart_Printf(&huart4, "UART4 Ringbuffer:%s\r\n", uart4_data_buffer);
    
    memset(uart4_data_buffer, 0, uart_data_len);
  }
}

/* 串口 5 */
void Uart5_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart5_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart5_ring_buffer, uart5_data_buffer, uart_data_len);
    uart5_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    // Uart_Printf(DEBUG_UART, "UART5 Ringbuffer:%s\r\n", uart5_data_buffer);
    // Uart_Printf(&huart5, "UART5 Ringbuffer:%s\r\n", uart5_data_buffer);
    // Blue_Key_Parse(uart5_data_buffer);
    
    memset(uart5_data_buffer, 0, uart_data_len);
  }
}

/* 串口 6 */
void Uart6_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart6_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart6_ring_buffer, uart6_data_buffer, uart_data_len);
    uart6_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART6 Ringbuffer:%s\r\n", uart6_data_buffer);
    Uart_Printf(&huart6, "UART6 Ringbuffer:%s\r\n", uart6_data_buffer);
    
    memset(uart6_data_buffer, 0, uart_data_len);
  }
}

void System_State_Uart_Print(void)
{
//  Uart_Printf(DEBUG_UART, "\r\n========== System State ==========\r\n");
//  
//  Uart_Printf(DEBUG_UART, "[RTC] 20%02d-%02d-%02d  %02d:%02d:%02d\r\n", RTC_Day.Year, RTC_Day.Month, RTC_Day.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
//  
//  Uart_Printf(DEBUG_UART, "[PID State] %d\r\n", pid_running);
//  
//  Uart_Printf(DEBUG_UART, "[LED] %d-%d-%d-%d\r\n", led_buf[0], led_buf[1], led_buf[2], led_buf[3]);
//  
//  Uart_Printf(DEBUG_UART, "[Grayscale] %d-%d-%d-%d-%d-%d-%d-%d\r\n",(gray_digtal>>0)&0x01,(gray_digtal>>1)&0x01,(gray_digtal>>2)&0x01,(gray_digtal>>3)&0x01,
//                                                                   (gray_digtal>>4)&0x01,(gray_digtal>>5)&0x01,(gray_digtal>>6)&0x01,(gray_digtal>>7)&0x01);

//	Uart_Printf(DEBUG_UART, "[Encoder] left:%.2fcm/s    right:%.2fcm/s\r\n", left_encoder.speed_cm_s, right_encoder.speed_cm_s);
//	
//  Uart_Printf(DEBUG_UART, "[Motor] left:%d    right:%d\r\n", left_motor.speed, right_motor.speed);

//	Uart_Printf(DEBUG_UART, "[Step_Motor] X:%d    Y:%d\r\n", Get_X_Step_Motor_Speed(), Get_Y_Step_Motor_Speed());

//  #if BNO08x_ON == 0
//  Uart_Printf(DEBUG_UART, "[ICM20608] %.2f, %.2f, %.2f\n", icm20608.Roll, icm20608.Pitch, icm20608.Yaw);
//  #else
//  Uart_Printf(DEBUG_UART, "[BNO08x] %.2f, %.2f, %.2f\n", bno08x.Roll, bno08x.Pitch, bno08x.Yaw);
//  #endif
//  Uart_Printf(DEBUG_UART, "[Temperature] %.2f \r\n", Humiture.Temp);

//  Uart_Printf(DEBUG_UART, "[Humidity] %.2f\r\n", Humiture.RH);
//  
//  Uart_Printf(DEBUG_UART, "[Now] %s\r\n", menu_get_current_page_name(navigator));
//  
//  Uart_Printf(DEBUG_UART, "[Restart Count] %d\r\n", start_count);
//  
//  Uart_Printf(DEBUG_UART, "==================================\r\n");
}

