#include "motor_app.h"

MOTOR left_motor;
MOTOR right_motor;

void Motor_Init(void)
{
    Uart_Printf(DEBUG_UART, "Motor_Init ......\r\n");

    /* 起转补偿由速度环前馈负责，驱动层不再二次抬升低 PWM 命令。 */
    Motor_Config_Init(&left_motor, &htim1, TIM_CHANNEL_2, &htim1, TIM_CHANNEL_1, 1, 0);
    Motor_Config_Init(&right_motor, &htim1, TIM_CHANNEL_4, &htim1, TIM_CHANNEL_3, 0, 0);
    // Motor_Set_Speed(&left_motor, 700);
    // Motor_Set_Speed(&right_motor, 700);
}

void Motor_Task(void)
{
    // Motor_Set_Speed(&left_motor, 700);
    // Motor_Set_Speed(&right_motor, 700);
}
