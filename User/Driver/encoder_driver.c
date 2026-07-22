#include "encoder_driver.h"

/**
 * @brief 初始化编码器驱动
 */
void Encoder_Driver_Init(Encoder* encoder, TIM_HandleTypeDef *htim, unsigned char reverse)
{
  encoder->htim = htim;
  encoder->reverse = reverse;
  
  // 启动定时器的编码器模式
  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);

  // 清零计数器
  __HAL_TIM_SetCounter(encoder->htim, 0);

  // 初始化数据结构
  encoder->count = 0;
  encoder->total_count = 0;
  encoder->speed_cm_s = 0.0f;
  encoder->rpm = 0.0f;
}

/**
 * @brief 更新编码器数据 (应周期性调用, 例如10ms一次)
 */
void Encoder_Driver_Update(Encoder* encoder)
{
  // 1. 读取原始计数值
  encoder->count = (int16_t)__HAL_TIM_GetCounter(encoder->htim);
  
  // 2. 处理编码器反向
  encoder->count = encoder->reverse == 0 ? encoder->count : -encoder->count;

  // 3. 清零硬件计数器，为下个周期做准备
  __HAL_TIM_SetCounter(encoder->htim, 0);

  // 4. 累计总数
  encoder->total_count += encoder->count;

  // 5. 计算速度 (cm/s)
  // 速度 = (计数值 / PPR) * 周长 / 采样时间
  encoder->speed_cm_s = (float)encoder->count / ENCODER_PPR * WHEEL_CIRCUMFERENCE_CM / SAMPLING_TIME_S;
  
  // 6. 计算RPM (每分钟转数)
  // RPM = (计数值 / PPR) * (60 / 采样时间)
  encoder->rpm = (float)encoder->count / ENCODER_PPR * (60.0f / SAMPLING_TIME_S);
}
