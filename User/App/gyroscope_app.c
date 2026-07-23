#include "gyroscope_app.h"

uint8_t first_gyroscope_flag = 0;

float frist_roll = 0;
float frist_pitch = 0;
float frist_yaw = 0;

#if BNO08x_ON == 0

ICM20608 icm20608;
static uint32_t last_gyro_time = 0;

void Gyroscope_Init(void)
{
  Uart_Printf(DEBUG_UART, "ICM20608 Gyroscope_Init ......\r\n");
  
  // 初始化ICM20608???
  ICM206xx_Init();
  
  // 初始化驱动程序
  Gyroscope_Driver_Init();
  
  // 开始校准
  Gyroscope_Calibrate_Start();
}

void Gyroscope_Task(void)
{
  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - last_gyro_time) / 1000.0f;  // 转换为秒
  
  // 初始化时间
  if (last_gyro_time == 0) {
    last_gyro_time = current_time;
    return;
  }
  
  // 限制时间间隔范围
  if (dt > 0.1f) dt = 0.01f;  // 最大100ms，防止异常
  if (dt < 0.001f) return;    // 最小1ms，避免过于频繁
  
  // 读取原始数据
  ICM206xx_Read_Data(&icm20608.gyro, &icm20608.accel, &icm20608.temperature);
  
  // 静态变量记录上次校准状态
  static uint8_t was_calibrating = 0;
  
  // 检查校准状态变化
  if (was_calibrating && !gyro_calibration.is_calibrating && euler_angles.calibrated) {
    // 校准刚完成，初始化姿态
    Gyroscope_Initialize_Attitude(&icm20608.accel);
  }
  was_calibrating = gyro_calibration.is_calibrating;
  
  // 更新欧拉角解算
  Gyroscope_Update_Euler(&icm20608.gyro, &icm20608.accel, dt);
  
  // 获取解算后的欧拉角
  Gyroscope_Get_Euler_Angles(&icm20608.Roll,&icm20608.Pitch, &icm20608.Yaw);
  
  // 抵消初始的 -179
  if(icm20608.Roll < 0)
    icm20608.Roll = 180 + icm20608.Roll;
  else
    icm20608.Roll = icm20608.Roll - 180;
  
    
  // 记录第一个有效欧拉角
  if (!first_gyroscope_flag && euler_angles.calibrated) {
    frist_roll = icm20608.Roll;
    frist_pitch = icm20608.Pitch;
    frist_yaw = icm20608.Yaw;
    first_gyroscope_flag = 1;
  }
  
  Uart_Printf(DEBUG_UART,"Roll=%.2f Pitch=%.2f Yaw=%.2f\r\n",icm20608.Roll,icm20608.Pitch,icm20608.Yaw);  

  last_gyro_time = current_time;
}

#else

BNO08x bno08x;

void Gyroscope_Init(void)
{
  Uart_Printf(DEBUG_UART, "BNO08x Gyroscope_Init ......\r\n");

  if (BNO080_HardwareReset() == 0) {
      Uart_Printf(DEBUG_UART, "BNO080硬件复位成功\n");
  } else {
      Uart_Printf(DEBUG_UART, "BNO080硬件复位失败，尝试软件复位\n");
      // 备用方案：软件复位
      softReset();
      HAL_Delay(100);
      Uart_Printf(DEBUG_UART, "BNO080软件复位完成\n");
  }
  
  enableRotationVector(100);
  enableGameRotationVector(100);
  enableAccelerometer(100);
  enableLinearAccelerometer(100);
  enableGyro(100);
  enableMagnetometer(100);
  enableStepCounter(100);
  enableStabilityClassifier(100);

  for(unsigned int i  = 0; i < 1000; i++)
  {
    if (dataAvailable()) 
    {
        float q0, q1, q2, q3;
        
        q0 = getQuatReal();
        q1 = getQuatI();
        q2 = getQuatJ();
        q3 = getQuatK();
        
        bno08x.Roll = atan2( 2 * ( q0 * q1 + q2 * q3 ) ,  1- 2 * ( q1 * q1 + q2 * q2 ) ) * 57.3;
        bno08x.Pitch = asin( 2 * ( q0 * q2 - q3 * q1 ) ) * 57.3;
        bno08x.Yaw = atan2( 2 * ( q0 * q3 + q1 * q2 ) ,  1 - 2 * ( q2 * q2 + q3 * q3 ) ) * 57.3;
        
        bno08x.gyro.x = getGyroX();
        bno08x.gyro.y = getGyroY();
        bno08x.gyro.z = getGyroZ();
        
        bno08x.accel.x = getAccelX();
        bno08x.accel.y = getAccelY();
        bno08x.accel.z = getAccelZ();
        
        bno08x.mag.x = getMagX();
        bno08x.mag.y = getMagY();
        bno08x.mag.z = getMagZ();
    }
  }
    
  /* 将当前位置归零 */
  if(first_gyroscope_flag == 0)
  {
    first_gyroscope_flag = 1;
    frist_roll = bno08x.Roll;
    frist_pitch = bno08x.Pitch;
    frist_yaw = bno08x.Yaw;
  }
  
//  calibrateAll();
}

void Gyroscope_Task(void)
{
    if (dataAvailable()) 
    {
      float q0, q1, q2, q3;
      
      q0 = getQuatReal();
      q1 = getQuatI();
      q2 = getQuatJ();
      q3 = getQuatK();
      
      bno08x.Roll = atan2( 2 * ( q0 * q1 + q2 * q3 ) ,  1- 2 * ( q1 * q1 + q2 * q2 ) ) * 57.3;
      bno08x.Pitch = asin( 2 * ( q0 * q2 - q3 * q1 ) ) * 57.3;
      bno08x.Yaw = atan2( 2 * ( q0 * q3 + q1 * q2 ) ,  1 - 2 * ( q2 * q2 + q3 * q3 ) ) * 57.3;
      
      bno08x.gyro.x = getGyroX();
      bno08x.gyro.y = getGyroY();
      bno08x.gyro.z = getGyroZ();
      
      bno08x.accel.x = getAccelX();
      bno08x.accel.y = getAccelY();
      bno08x.accel.z = getAccelZ();
      
      bno08x.mag.x = getMagX();
      bno08x.mag.y = getMagY();
      bno08x.mag.z = getMagZ();
      
      bno08x.Roll = bno08x.Roll - frist_roll;
      bno08x.Pitch = bno08x.Pitch - frist_pitch;
      bno08x.Yaw = bno08x.Yaw - frist_yaw;
    
      bno08x.Yaw = convert_to_continuous_yaw(bno08x.Yaw); 
  }
}

#endif

