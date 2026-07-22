#ifndef __ZDT_MOTOR_DRIVER_H__
#define __ZDT_MOTOR_DRIVER_H__

#include "main.h"
#include "Emm_V5.h"

/* 电机控制宏定义 */
#define MOTOR_X_ADDR        0x00          // X轴电机地址
#define MOTOR_Y_ADDR        0x00          // Y轴电机地址
#define MOTOR_X_UART        &huart2        // X轴电机串口
#define MOTOR_Y_UART        &huart4        // Y轴电机串口
#define MOTOR_SYNC_FLAG     false         // 电机同步标志
  
void Motor_Vel_Synchronous_Control(int x_vel, int y_vel);
int Get_X_Step_Motor_Speed(void);
int Get_Y_Step_Motor_Speed(void);

#endif

