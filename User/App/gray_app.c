#include "gray_app.h"

unsigned char gray_digtal; // 灰度传感器开关量

void Gray_Init(void)
{
  Uart_Printf(DEBUG_UART, "Gray_Init ......\r\n");
}

void Gray_Task(void)
{
    //获取传感器开关量结果
    gray_digtal = ~IIC_Get_Digtal();
}
