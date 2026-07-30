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
  
  // ï¿½ï¿½Ê¼ï¿½ï¿½ICM20608???
  ICM206xx_Init();
  
  // ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
  Gyroscope_Driver_Init();
  
  // ï¿½ï¿½Ê¼Ð£×¼
  Gyroscope_Calibrate_Start();
}

void Gyroscope_Task(void)
{
  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - last_gyro_time) / 1000.0f;  // ×ªï¿½ï¿½Îªï¿½ï¿½
  
  // ï¿½ï¿½Ê¼ï¿½ï¿½Ê±ï¿½ï¿½
  if (last_gyro_time == 0) {
    last_gyro_time = current_time;
    return;
  }
  
  /* ÈÎÎñÓÉÖ÷Ñ­»·Ã¿ 100 ms µ÷¶È£»·¢Éú¶¶¶¯Ê±ÏÞÖÆ×î´ó»ý·Ö²½³¤£¬±ÜÃâ×ËÌ¬Í»Ìø¡£ */
  if (dt > 0.1f) dt = 0.1f;
  if (dt < 0.001f) return;    // ï¿½ï¿½Ð¡1msï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æµï¿½ï¿?
  
  // ï¿½ï¿½È¡Ô­Ê¼ï¿½ï¿½ï¿½ï¿½
  ICM206xx_Read_Data(&icm20608.gyro, &icm20608.accel, &icm20608.temperature);
  
  // ï¿½ï¿½Ì¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Â¼ï¿½Ï´ï¿½Ð£×¼×´Ì¬
  static uint8_t was_calibrating = 0;
  
  // ï¿½ï¿½ï¿½Ð£×¼×´Ì?ï¿½ä»¯
  if (was_calibrating && !gyro_calibration.is_calibrating && euler_angles.calibrated) {
    // Ð£×¼ï¿½ï¿½ï¿½ï¿½É£ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½Ì?
    Gyroscope_Initialize_Attitude(&icm20608.accel);
  }
  was_calibrating = gyro_calibration.is_calibrating;
  
  // ï¿½ï¿½ï¿½ï¿½Å·ï¿½ï¿½ï¿½Ç½ï¿½ï¿½ï¿½
  Gyroscope_Update_Euler(&icm20608.gyro, &icm20608.accel, dt);
  
  // ï¿½ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Å·ï¿½ï¿½ï¿½ï¿½
  Gyroscope_Get_Euler_Angles(&icm20608.Roll,&icm20608.Pitch, &icm20608.Yaw);
  
  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ -179
  if(icm20608.Roll < 0)
    icm20608.Roll = 180 + icm20608.Roll;
  else
    icm20608.Roll = icm20608.Roll - 180;

  // ç»Ÿä¸€è½¦ä½“æ–¹å‘çº¦å®šï¼šå·¦å€?/å·¦è½¬ä¸ºè´Ÿï¼Œå³å€?/å³è½¬ä¸ºæ?£ã€?
  icm20608.Roll = -icm20608.Roll;
  icm20608.Yaw = -icm20608.Yaw;
  
    
  // ï¿½ï¿½Â¼ï¿½ï¿½Ò»ï¿½ï¿½ï¿½ï¿½Ð§Å·ï¿½ï¿½ï¿½ï¿½
  if (!first_gyroscope_flag && euler_angles.calibrated) {
    frist_roll = icm20608.Roll;
    frist_pitch = icm20608.Pitch;
    frist_yaw = icm20608.Yaw;
    first_gyroscope_flag = 1;
  }

  // Uart_Printf(DEBUG_UART, "Roll=%.2f Pitch=%.2f Yaw=%.2f\r\n",
  //             icm20608.Roll, icm20608.Pitch, icm20608.Yaw);
  // Uart_Printf(DEBUG_UART, "GyroX=%.3fdeg/s GyroY=%.3fdeg/s GyroZ=%.3fdeg/s\r\n",
  //             icm20608.gyro.x, icm20608.gyro.y, icm20608.gyro.z);

  last_gyro_time = current_time;
}

#else

BNO08x bno08x;

void Gyroscope_Init(void)
{
  Uart_Printf(DEBUG_UART, "BNO08x Gyroscope_Init ......\r\n");

  if (BNO080_HardwareReset() == 0) {
      Uart_Printf(DEBUG_UART, "BNO080Ó²ï¿½ï¿½ï¿½ï¿½Î»ï¿½É¹ï¿½\n");
  } else {
      Uart_Printf(DEBUG_UART, "BNO080Ó²ï¿½ï¿½ï¿½ï¿½Î»Ê§ï¿½Ü£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»\n");
      // ï¿½ï¿½ï¿½Ã·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»
      softReset();
      HAL_Delay(100);
      Uart_Printf(DEBUG_UART, "BNO080ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î»ï¿½ï¿½ï¿½\n");
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
    
  /* ï¿½ï¿½ï¿½ï¿½Ç°Î»ï¿½Ã¹ï¿½ï¿½ï¿½ */
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

