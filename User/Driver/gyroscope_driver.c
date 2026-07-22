#include "gyroscope_driver.h"
#include "uart_driver.h"

// 全局变量定义
EulerAngles_t euler_angles = {0};
GyroCalibration_t gyro_calibration = {0};

// 互补滤波器参数
static float alpha = 0.85f;  // 初始滤波系数，校准后调整为0.02f
static const float CALIBRATION_TIME_MS = 2000.0f;  // 校准时间2秒
static const float STATIC_THRESHOLD = 5.0f;       // 静态检测阈值 (°/s)
static const float DRIFT_THRESHOLD = 10.0f;       // 漂移检测阈值 (°/s)

/**
 * @brief 陀螺仪驱动初始化
 */
void Gyroscope_Driver_Init(void)
{
    // 初始化欧拉角结构体
    euler_angles.roll = 0.0f;
    euler_angles.pitch = 0.0f;
    euler_angles.yaw = 0.0f;
    
    // 初始化四元数为单位四元数
    euler_angles.q0 = 1.0f;
    euler_angles.q1 = 0.0f;
    euler_angles.q2 = 0.0f;
    euler_angles.q3 = 0.0f;
    
    // 初始化零偏
    euler_angles.gx_bias = 0.0f;
    euler_angles.gy_bias = 0.0f;
    euler_angles.gz_bias = 0.0f;
    
    euler_angles.calibrated = 0;
    euler_angles.last_update_time = HAL_GetTick();
    
    // 初始化校准参数
    gyro_calibration.calibration_count = 0;
    gyro_calibration.gyro_sum_x = 0.0f;
    gyro_calibration.gyro_sum_y = 0.0f;
    gyro_calibration.gyro_sum_z = 0.0f;
    gyro_calibration.is_calibrating = 0;
}

/**
 * @brief 开始陀螺仪校准
 */
void Gyroscope_Calibrate_Start(void)
{
    gyro_calibration.calibration_count = 0;
    gyro_calibration.gyro_sum_x = 0.0f;
    gyro_calibration.gyro_sum_y = 0.0f;
    gyro_calibration.gyro_sum_z = 0.0f;
    gyro_calibration.is_calibrating = 1;
    gyro_calibration.calibration_start_time = HAL_GetTick();
    
    alpha = 0.85f;  // 重置滤波系数
}

/**
 * @brief 更新陀螺仪校准
 * @param gyro 陀螺仪数据
 */
void Gyroscope_Calibrate_Update(Vector3f *gyro)
{
    if (!gyro_calibration.is_calibrating) return;
    
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - gyro_calibration.calibration_start_time;
    
    // 检查校准时间
    if (elapsed_time >= CALIBRATION_TIME_MS) {
        if (gyro_calibration.calibration_count > 0) {
            // 计算平均零偏
            euler_angles.gx_bias = gyro_calibration.gyro_sum_x / gyro_calibration.calibration_count;
            euler_angles.gy_bias = gyro_calibration.gyro_sum_y / gyro_calibration.calibration_count;
            euler_angles.gz_bias = gyro_calibration.gyro_sum_z / gyro_calibration.calibration_count;
            
            euler_angles.calibrated = 1;
            alpha = 0.02f;  // 校准后调整滤波系数
                       
            // 校准完成后重置姿态为初始状态
            Gyroscope_Reset_Attitude();
        }
        
        gyro_calibration.is_calibrating = 0;
        return;
    }
    
    // 检查数据是否在静态阈值内
    if (fabs(gyro->x) < STATIC_THRESHOLD && 
        fabs(gyro->y) < STATIC_THRESHOLD && 
        fabs(gyro->z) < STATIC_THRESHOLD) {
        
        gyro_calibration.gyro_sum_x += gyro->x;
        gyro_calibration.gyro_sum_y += gyro->y;
        gyro_calibration.gyro_sum_z += gyro->z;
        gyro_calibration.calibration_count++;
        
        // 显示校准进度
        if (gyro_calibration.calibration_count % 50 == 0) {
            float progress = (float)elapsed_time / CALIBRATION_TIME_MS * 100.0f;
        }
    }
}

/**
 * @brief 数据有效性检查
 * @param gyro 陀螺仪数据
 * @param accel 加速度计数据
 * @return 1-有效, 0-无效
 */
static uint8_t data_validity_check(Vector3f *gyro, Vector3f *accel)
{
    // 检查数据是否为有限值
    if (!IS_FINITE(gyro->x) || !IS_FINITE(gyro->y) || !IS_FINITE(gyro->z) ||
        !IS_FINITE(accel->x) || !IS_FINITE(accel->y) || !IS_FINITE(accel->z)) {
        return 0;
    }
    
    // 检查陀螺仪数据是否超出合理范围 (±2000°/s)
    if (fabs(gyro->x) > 2000.0f || fabs(gyro->y) > 2000.0f || fabs(gyro->z) > 2000.0f) {
        return 0;
    }
    
    // 检查加速度计数据是否超出合理范围 (±16g)
    if (fabs(accel->x) > 16.0f || fabs(accel->y) > 16.0f || fabs(accel->z) > 16.0f) {
        return 0;
    }
    
    return 1;
}

/**
 * @brief 四元数归一化
 * @param q0,q1,q2,q3 四元数分量指针
 */
static void quaternion_normalize(float *q0, float *q1, float *q2, float *q3)
{
    float norm = sqrtf((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        *q0 *= inv_norm;
        *q1 *= inv_norm;
        *q2 *= inv_norm;
        *q3 *= inv_norm;
    }
}

/**
 * @brief 四元数转换为欧拉角
 * @param q0,q1,q2,q3 四元数分量
 * @param roll,pitch,yaw 欧拉角输出指针
 */
static void quaternion_to_euler(float q0, float q1, float q2, float q3, 
                                float *roll, float *pitch, float *yaw)
{
    // Roll (x轴旋转)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    // Pitch (y轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);  // 使用90度当|sin(pitch)| >= 1
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    // Yaw (z轴旋转)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/**
 * @brief 互补滤波器更新
 * @param gyro 陀螺仪数据 (°/s)
 * @param accel 加速度计数据 (g)
 * @param dt 时间间隔 (s)
 */
static void complementary_filter_update(Vector3f *gyro, Vector3f *accel, float dt)
{
    // 应用零偏补偿
    float gx = (gyro->x - euler_angles.gx_bias) * DEG_TO_RAD;
    float gy = (gyro->y - euler_angles.gy_bias) * DEG_TO_RAD;
    float gz = (gyro->z - euler_angles.gz_bias) * DEG_TO_RAD;
    
    // 加速度计数据归一化
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (accel_norm == 0.0f) return;
    
    float ax = accel->x / accel_norm;
    float ay = accel->y / accel_norm;
    float az = accel->z / accel_norm;
    
    // 当前四元数
    float q0 = euler_angles.q0;
    float q1 = euler_angles.q1;
    float q2 = euler_angles.q2;
    float q3 = euler_angles.q3;
    
    // 如果还未校准完成，使用加速度计初始化姿态
    if (!euler_angles.calibrated) {
        // 使用加速度计数据初始化四元数（假设初始时刻只有重力）
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if (norm > 0.1f) {
            // 计算初始Roll和Pitch
            float init_roll = atan2f(ay, az);
            float init_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
            
            // 转换为四元数（Yaw设为0）
            float cr = cosf(init_roll * 0.5f);
            float sr = sinf(init_roll * 0.5f);
            float cp = cosf(init_pitch * 0.5f);
            float sp = sinf(init_pitch * 0.5f);
            
            euler_angles.q0 = cr * cp;
            euler_angles.q1 = sr * cp;
            euler_angles.q2 = cr * sp;
            euler_angles.q3 = 0.0f;  // Yaw = 0
            
            // 归一化
            quaternion_normalize(&euler_angles.q0, &euler_angles.q1, &euler_angles.q2, &euler_angles.q3);
        }
        return;
    }
    
    // 陀螺仪积分更新四元数
    float q0_gyro = q0 + 0.5f * dt * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_gyro = q1 + 0.5f * dt * ( q0 * gx - q3 * gy + q2 * gz);
    float q2_gyro = q2 + 0.5f * dt * ( q3 * gx + q0 * gy - q1 * gz);
    float q3_gyro = q3 + 0.5f * dt * (-q2 * gx + q1 * gy + q0 * gz);
    
    // 计算当前重力方向 (四元数表示)
    float gx_est = 2.0f * (q1 * q3 - q0 * q2);
    float gy_est = 2.0f * (q0 * q1 + q2 * q3);
    float gz_est = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 计算误差向量 (叉积)
    float ex = ay * gz_est - az * gy_est;
    float ey = az * gx_est - ax * gz_est;
    float ez = ax * gy_est - ay * gx_est;
    
    // 动态调整积分增益
    float kp = 0.5f;  // 比例增益
    float ki = 0.01f; // 积分增益（用于消除长期误差）
    
    // 静态变量存储积分误差
    static float ex_int = 0.0f, ey_int = 0.0f, ez_int = 0.0f;
    
    // 积分误差累积（有界积分）
    ex_int += ex * dt;
    ey_int += ey * dt;
    ez_int += ez * dt;
    
    // 限制积分项
    float int_limit = 1.0f;
    if (ex_int > int_limit) ex_int = int_limit;
    if (ex_int < -int_limit) ex_int = -int_limit;
    if (ey_int > int_limit) ey_int = int_limit;
    if (ey_int < -int_limit) ey_int = -int_limit;
    if (ez_int > int_limit) ez_int = int_limit;
    if (ez_int < -int_limit) ez_int = -int_limit;
    
    // PI控制器校正
    float gx_corrected = gx + kp * ex + ki * ex_int;
    float gy_corrected = gy + kp * ey + ki * ey_int;
    float gz_corrected = gz + kp * ez + ki * ez_int;
    
    // 使用校正后的陀螺仪数据更新四元数
    euler_angles.q0 = q0 + 0.5f * dt * (-q1 * gx_corrected - q2 * gy_corrected - q3 * gz_corrected);
    euler_angles.q1 = q1 + 0.5f * dt * ( q0 * gx_corrected - q3 * gy_corrected + q2 * gz_corrected);
    euler_angles.q2 = q2 + 0.5f * dt * ( q3 * gx_corrected + q0 * gy_corrected - q1 * gz_corrected);
    euler_angles.q3 = q3 + 0.5f * dt * (-q2 * gx_corrected + q1 * gy_corrected + q0 * gz_corrected);
    
    // 归一化四元数
    quaternion_normalize(&euler_angles.q0, &euler_angles.q1, &euler_angles.q2, &euler_angles.q3);
    
    // 转换为欧拉角
    quaternion_to_euler(euler_angles.q0, euler_angles.q1, euler_angles.q2, euler_angles.q3,
                       &euler_angles.roll, &euler_angles.pitch, &euler_angles.yaw);
}

/**
 * @brief 更新欧拉角 (主要接口函数)
 * @param gyro 陀螺仪数据 (°/s)
 * @param accel 加速度计数据 (g)
 * @param dt 时间间隔 (s)
 */
void Gyroscope_Update_Euler(Vector3f *gyro, Vector3f *accel, float dt)
{
    if (!gyro || !accel) return;
    
    // 数据有效性检查
    if (!data_validity_check(gyro, accel)) {
        return;
    }
    
    // 如果正在校准，更新校准数据
    if (gyro_calibration.is_calibrating) {
        Gyroscope_Calibrate_Update(gyro);
        return;
    }
    
    // 检查是否需要重新校准 (检测到较大漂移)
    if (euler_angles.calibrated) {
        float gyro_magnitude = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
        if (gyro_magnitude > DRIFT_THRESHOLD) {
            static uint32_t last_drift_check = 0;
            uint32_t current_time = HAL_GetTick();
            
            // 每5分钟检查一次漂移
            if (current_time - last_drift_check > 300000) {
                last_drift_check = current_time;
            }
        }
    }
    
    // 更新姿态
    complementary_filter_update(gyro, accel, dt);
    
    euler_angles.last_update_time = HAL_GetTick();
}

/**
 * @brief 获取欧拉角
 * @param roll,pitch,yaw 输出欧拉角指针
 */
void Gyroscope_Get_Euler_Angles(float *roll, float *pitch, float *yaw)
{
    if (roll) *roll = euler_angles.roll;
    if (pitch) *pitch = euler_angles.pitch;
    if (yaw) *yaw = euler_angles.yaw;
}

/**
 * @brief 重置姿态
 */
void Gyroscope_Reset_Attitude(void)
{
    euler_angles.q0 = 1.0f;
    euler_angles.q1 = 0.0f;
    euler_angles.q2 = 0.0f;
    euler_angles.q3 = 0.0f;
    
    euler_angles.roll = 0.0f;
    euler_angles.pitch = 0.0f;
    euler_angles.yaw = 0.0f;
}

/**
 * @brief 初始化姿态（使用加速度计数据）
 * @param accel 加速度计数据
 */
void Gyroscope_Initialize_Attitude(Vector3f *accel)
{
    if (!accel) return;
    
    // 加速度计数据归一化
    float norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (norm < 0.1f) {
        return;
    }
    
    float ax = accel->x / norm;
    float ay = accel->y / norm;
    float az = accel->z / norm;
    
    // 计算初始Roll和Pitch（假设仅有重力加速度）
    float init_roll = atan2f(ay, az);
    float init_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    
    // 转换为四元数（Yaw设为0）
    float cr = cosf(init_roll * 0.5f);
    float sr = sinf(init_roll * 0.5f);
    float cp = cosf(init_pitch * 0.5f);
    float sp = sinf(init_pitch * 0.5f);
    
    euler_angles.q0 = cr * cp;
    euler_angles.q1 = sr * cp;
    euler_angles.q2 = cr * sp;
    euler_angles.q3 = 0.0f;  // Yaw = 0
    
    // 归一化四元数
    quaternion_normalize(&euler_angles.q0, &euler_angles.q1, &euler_angles.q2, &euler_angles.q3);
    
    // 转换为欧拉角
    quaternion_to_euler(euler_angles.q0, euler_angles.q1, euler_angles.q2, euler_angles.q3,
                       &euler_angles.roll, &euler_angles.pitch, &euler_angles.yaw);
}

/**
 * @brief 获取偏航角速率
 * @return 偏航角速率 (°/s)
 */
float Gyroscope_Get_Yaw_Rate(void)
{
    return euler_angles.gz_bias;  // 返回校正后的Z轴角速度
}

/**
 * @brief 打印传感器原始数据（调试用）
 * @param gyro 陀螺仪数据
 * @param accel 加速度计数据
 */
void Gyroscope_Print_Raw_Data(Vector3f *gyro, Vector3f *accel)
{
    if (!gyro || !accel) return;
    
    // 计算陀螺仪幅值和加速度计幅值
    float gyro_mag = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
    float accel_mag = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
}

// 使用静态变量来保存上一次的状态
float g_last_yaw = 0.0f;
int g_revolution_count = 0;
bool g_is_yaw_initialized = false;

/**
 * @brief 将一个在[-180, 180]范围内的yaw角度转换为连续的角度值。
 * 
 * @param current_yaw 从传感器读取的当前yaw值 (-180 to 180)。
 * @return float 连续的yaw角度值 (例如 370, -450 等)。
 */
float convert_to_continuous_yaw(float current_yaw) 
{
    // 定义一个阈值来检测“跳变”。这个值应该大于180，通常取270或300比较安全。
    const float WRAP_AROUND_THRESHOLD = 300.0f;

    // 首次调用时进行初始化
    if (!g_is_yaw_initialized) {
        g_last_yaw = current_yaw;
        g_is_yaw_initialized = true;
        g_revolution_count = 0;
    }

    // 计算与上次读数的差异
    float diff = current_yaw - g_last_yaw;

    // 检测是否发生了“跳变”
    if (diff > WRAP_AROUND_THRESHOLD) {
        // 从正角度跳到负角度 (例如, 从 170° 到 -175°), 实际是向右转, 圈数应该增加
        // 此时 diff 接近 -360 (例如 -175 - 170 = -345)
        // 这段逻辑处理的是从-180跳变到+180的情况，说明是向左转过界
        g_revolution_count--;
    } else if (diff < -WRAP_AROUND_THRESHOLD) {
        // 从负角度跳到正角度 (例如, 从 -170° 到 175°), 实际是向左转, 圈数应该减小
        // 此时 diff 接近 360 (例如 175 - (-170) = 345)
        // 这段逻辑处理的是从+180跳变到-180的情况，说明是向右转过界
        g_revolution_count++;
    }

    // 更新上次的yaw值以备下次调用
    g_last_yaw = current_yaw;

    // 计算连续的yaw值
    float continuous_yaw = current_yaw + (float)g_revolution_count * 360.0f;

    return continuous_yaw;
}
