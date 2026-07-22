#include "motor_driver.h"

#define Motor_ABS(x) ((x) >= 0 ? (x) : -(x))

void Motor_Config_Init(MOTOR* motor, 
                       TIM_HandleTypeDef *htim_in1, uint32_t pwm_channel_in1 ,
                       TIM_HandleTypeDef *htim_in2, uint32_t pwm_channel_in2,
                       unsigned char reverse, int dead_band_speed)
{
    // 配置IN1通道
    motor->config.in1.htim = htim_in1;
    motor->config.in1.pwm_channel = pwm_channel_in1;
    
    // 配置IN2通道
    motor->config.in2.htim = htim_in2;
    motor->config.in2.pwm_channel = pwm_channel_in2;
    
    motor->config.reverse = reverse;
    motor->dead_band_speed = dead_band_speed;
    motor->speed = 0;

    // 启动两个PWM通道并初始化为停止状态
    HAL_TIM_PWM_Start(motor->config.in1.htim, motor->config.in1.pwm_channel);
    HAL_TIM_PWM_Start(motor->config.in2.htim, motor->config.in2.pwm_channel);
    
    __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, 0);
}

// 电机速度限幅
int Motor_Limit_Speed(MOTOR* motor, int speed, int max_speed, int min_speed)
{
    if(speed > max_speed)
        speed = max_speed;
    else if(speed < min_speed)
        speed = min_speed;

    return speed;
}

// 电机死区补偿
int Motor_Dead_Compensation(MOTOR* motor)
{
    if(motor->speed > 0 && motor->speed < motor->dead_band_speed)
        return motor->dead_band_speed;
    else if(motor->speed < 0 && motor->speed > -motor->dead_band_speed) 
        return -motor->dead_band_speed;
    else
        return motor->speed;
}

// 速度控制
void Motor_Set_Speed(MOTOR* motor, int speed)
{
    // 电机速度限幅
    int max_speed = motor->config.in1.htim->Init.Period; // 使用IN1的定时器周期作为最大速度
    motor->speed = Motor_Limit_Speed(motor, speed, max_speed, -max_speed);

    // 电机死区补偿
    motor->speed = Motor_Dead_Compensation(motor);
    
    // 计算实际PWM值（绝对值）
    uint32_t pwm_value = Motor_ABS(motor->speed);
    
    // 根据方向和反转标志设置双PWM通道
    if((motor->speed >= 0 && motor->config.reverse == 0) || 
       (motor->speed < 0 && motor->config.reverse == 1)) 
    {
        // 正向旋转：IN1=PWM, IN2=0
        __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, pwm_value);
        __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, 0);
    }
    else 
    {
        // 反向旋转：IN1=0, IN2=PWM
        __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, 0);
        __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, pwm_value);
    }
}

// 电机停止
void Motor_Stop(MOTOR* motor)
{
    // 两个通道都设置为0（低电平）
    __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, 0);
    __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, 0);
    
    motor->speed = 0;
}

// 电机刹车
void Motor_Brake(MOTOR* motor)
{
    // 获取最大PWM值（定时器周期值）
    uint32_t max_pwm = motor->config.in1.htim->Init.Period;
    
    // 两个通道都设置为最大值（高电平）
    __HAL_TIM_SET_COMPARE(motor->config.in1.htim, motor->config.in1.pwm_channel, max_pwm);
    __HAL_TIM_SET_COMPARE(motor->config.in2.htim, motor->config.in2.pwm_channel, max_pwm);
    
    motor->speed = 0;
}
