#include "encoder_app.h"

// 编码器实例
Encoder left_encoder;
Encoder right_encoder;

/**
 * @brief 初始化编码器应用
 */
void Encoder_Init(void)
{
  Uart_Printf(DEBUG_UART, "Encoder_Init ......\r\n");
  
  Encoder_Driver_Init(&left_encoder, &htim3, 1);
  Encoder_Driver_Init(&right_encoder, &htim4, 1);
}

/**
 * @brief 编码器应用运行任务 (应由调度器周期性调用)
 */
void Encoder_Task(void)
{
  Encoder_Driver_Update(&left_encoder);
  Encoder_Driver_Update(&right_encoder);
}
