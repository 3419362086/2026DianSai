#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "main.h"

// DRV8871DDA 电机配置结构体
typedef struct MOTOR_Config
{
    struct 
    {
        TIM_HandleTypeDef *htim; // 定时器
        uint32_t pwm_channel; // PWM 通道
    } in1, in2; // 对应 DRV8871DDA 中的 AIN1 或 AIN2
    unsigned char reverse; // 电机的方向是否反转。0-正常，1-反转
} MOTOR_Config;


// 电机结构体
typedef struct MOTOR
{
    MOTOR_Config config;
    int speed;
    int dead_band_speed; // 死区（当 -dead_band_speed < speed < dead_band_speed 时电机不会转动）
} MOTOR;



// 电机初始化函数(绑定相关的PWM通道)
void Motor_Config_Init(MOTOR* motor, 
                       TIM_HandleTypeDef *htim_in1, uint32_t pwm_channel_in1,
                       TIM_HandleTypeDef *htim_in2, uint32_t pwm_channel_in2,
                       unsigned char reverse, int dead_band_speed);

// 速度控制
void Motor_Set_Speed(MOTOR* motor, int speed);

// 电机停止  
void Motor_Stop(MOTOR* motor);          

// 电机刹车
void Motor_Brake(MOTOR* motor);         

#endif
