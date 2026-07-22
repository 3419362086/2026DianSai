#ifndef __GYROSCOPE_DRIVER_H__
#define __GYROSCOPE_DRIVER_H__

#include "main.h"
#include "icm20608.h"
#include "bno08x_hal.h"
#include "stdbool.h"


// 姿态解算相关宏定义
#define IS_FINITE(x) (!((x) != (x)) && (x) > -1e30f && (x) < 1e30f)
#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.017453292f

// 姿态解算结构体
typedef struct {
    float roll;     // 横滚角 (度)
    float pitch;    // 俯仰角 (度) 
    float yaw;      // 偏航角 (度)
    float q0, q1, q2, q3;  // 四元数
    float gx_bias, gy_bias, gz_bias;  // 陀螺仪零偏
    uint8_t calibrated;  // 校准状态标志
    uint32_t last_update_time;  // 上次更新时间
} EulerAngles_t;

// 校准参数结构体
typedef struct {
    uint32_t calibration_count;
    float gyro_sum_x, gyro_sum_y, gyro_sum_z;
    uint8_t is_calibrating;
    uint32_t calibration_start_time;
} GyroCalibration_t;

// 外部变量声明
extern EulerAngles_t euler_angles;
extern GyroCalibration_t gyro_calibration;

// 函数声明
void Gyroscope_Driver_Init(void);
void Gyroscope_Update_Euler(Vector3f *gyro, Vector3f *accel, float dt);
void Gyroscope_Calibrate_Start(void);
void Gyroscope_Calibrate_Update(Vector3f *gyro);
void Gyroscope_Get_Euler_Angles(float *roll, float *pitch, float *yaw);
void Gyroscope_Reset_Attitude(void);
void Gyroscope_Initialize_Attitude(Vector3f *accel);
float Gyroscope_Get_Yaw_Rate(void);
void Gyroscope_Print_Status(void);
void Gyroscope_Print_Raw_Data(Vector3f *gyro, Vector3f *accel);

float convert_to_continuous_yaw(float current_yaw);

#endif

