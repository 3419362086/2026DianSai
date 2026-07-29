#include "gray_app.h"

uint8_t gray_digtal[GRAY_CHANNEL_COUNT] = {0}; // 灰度传感器开关量

void Gray_Init(void)
{
  Uart_Printf(DEBUG_UART, "Gray_Init ......\r\n");
}

void Gray_Task(void)
{
    //获取传感器开关量结果
    deal_IRdata(&gray_digtal[0], &gray_digtal[1], &gray_digtal[2], &gray_digtal[3],
                &gray_digtal[4], &gray_digtal[5], &gray_digtal[6], &gray_digtal[7]);
    Uart_Printf(DEBUG_UART, "x1:%d, x2:%d, x3:%d, x4:%d, x5:%d, x6:%d, x7:%d, x8:%d\r\n", gray_digtal[0], gray_digtal[1], gray_digtal[2], gray_digtal[3], gray_digtal[4], gray_digtal[5], gray_digtal[6], gray_digtal[7]);
}
