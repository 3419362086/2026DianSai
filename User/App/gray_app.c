#include "gray_app.h"

uint8_t gray_digtal[GRAY_CHANNEL_COUNT] = {0}; // 灰度传感器开关量
volatile float g_line_position_error =0.0f;/* 巡线横向偏差（供外部 PID 控制器读取），范围约 [-4, +4] */
/* ------------------------------------------------------------------ */
/* 权重表：用于加权平均法计算横向偏差                                    */
/* 负值 → 黑线偏左，正值 → 黑线偏右                                     */
/* ch0(+4) … ch3(+1) | ch4(-1) … ch7(-4)                              */
/* ------------------------------------------------------------------ */
float gray_weight[8] = {4.0f, 3.0f, 2.0f, 0.5f, -0.5f, -2.0f, -3.0f, -4.0f};

void Gray_Init(void)
{
  Uart_Printf(DEBUG_UART, "Gray_Init ......\r\n");
}

float BlackLine_SetTurn(void)
{
    float Err = 0.0f;
    float weight_sum = 0.0f;
    uint8_t blackline_count = 0;

    for (uint8_t i = 0; i < GRAY_CHANNEL_COUNT; i++)
    {
        /* 数组元素为 1，表示对应通道检测到黑线。 */
        if (gray_digtal[i] == 1)
        {
            weight_sum += gray_weight[i];   /* 累加该通道权重 */
            blackline_count++;
        }
    }

    if (blackline_count > 0)
        Err = weight_sum / blackline_count; /* 加权平均得到偏差 */
    else
        Err = g_line_position_error;

    return Err;
}

void Gray_Task(void)
{
    //获取传感器开关量结果
    deal_IRdata(&gray_digtal[0], &gray_digtal[1], &gray_digtal[2], &gray_digtal[3],
                &gray_digtal[4], &gray_digtal[5], &gray_digtal[6], &gray_digtal[7]);
    g_line_position_error = BlackLine_SetTurn();
    // Uart_Printf(DEBUG_UART,"Black line position error: %.2f\r\n", g_line_position_error);
    // Uart_Printf(DEBUG_UART, "x1:%d, x2:%d, x3:%d, x4:%d, x5:%d, x6:%d, x7:%d, x8:%d\r\n", gray_digtal[0], gray_digtal[1], gray_digtal[2], gray_digtal[3], gray_digtal[4], gray_digtal[5], gray_digtal[6], gray_digtal[7]);
}
