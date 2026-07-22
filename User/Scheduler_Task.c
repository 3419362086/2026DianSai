/*
 * @环境：本程序基于米醋 STM32F407VET6 电控板 及 HAL 库进行编写
 * @接线：
 * @功能：包含控制板除舵机外所有外设的菜单示例程序\
 * @注意：默认陀螺仪采用 ICM20608 如果需要开启 BNO08x 在 gyroscope_app.h 中将 BNO08x_ON 改为 1 即可
 *       菜单的相关配置在 User/Module/Easy_Menu/Easy_Menu_User.c 中修改
*/

#include "Scheduler_Task.h"

void System_Init(void)
{
  Uart_Tx_Init();
  Uart_Printf(DEBUG_UART, "==== System Init ====\r\n");
  Led_Init();
  Key_Init();
  Uart_Init();
  Oled_Init();
  Gray_Init();
  Motor_Init();
  Encoder_Init();
  Gyroscope_Init();
  ZDT_Motor_Init();
  PID_Init();
  Aht20_Init();
  Flash_Init();
  RTC_Init();
  Uart_Printf(DEBUG_UART, "==== Finish ====\r\n");
  HAL_TIM_Base_Start_IT(&htim2);
}

unsigned char key_timer10ms = 0;
unsigned char timer10ms = 0;

// TIM2 中断服务函数（1ms 中断）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance != htim2.Instance) return;
  
  /* 按键扫描任务 */
  if(++key_timer10ms >= 10)
  {
    key_timer10ms = 0;
//    Uart5_Task();
    Key_Task();
  }
  
  /* 实时任务 */
  if(++timer10ms >= 10)
  {
    timer10ms = 0;
    Encoder_Task();
    Gyroscope_Task();
    PID_Task();
  }
}

/* =================================== 自定义工具函数 =================================== */

/* 微秒级非阻塞延时 */
void Delay_Us(unsigned int us)
{
	__HAL_TIM_SET_COUNTER(&htim14, 0);
  HAL_TIM_Base_Start(&htim14);  
	while (__HAL_TIM_GET_COUNTER(&htim14) < us);
  HAL_TIM_Base_Stop(&htim14);
}

