BNO08X硬件复位功能说明：

/**
 * @brief BNO080_HardwareReset() - 硬件复位函数
 * @note 该函数通过控制nRESET引脚(PD4)进行硬件复位
 * @note 复位序列：拉低nRESET 20ms -> 释放引脚 -> 等待Product ID Response
 * @note 相比软件复位更可靠，能完全重置传感器状态
 * @retval 0: 复位成功并收到Product ID Response
 * @retval -1: 复位超时失败（1秒内未收到响应）
 *
 * 使用场景：
 * - 传感器初始化时的首选复位方式
 * - 传感器异常时的恢复操作
 * - 需要完全重置传感器状态时
 */

初始化后，调用即可

/**
 * @brief BNO080传感器完整初始化和配置函数
 * @note 包含硬件初始化、复位、传感器配置等完整流程
 * @note 用户可通过注释选择性启用不同功能模块
 * @retval 0: 成功, -1: 失败
 */
int8_t my_bno080_init(void)
{
    my_printf(&huart1, "开始初始化BNO080传感器...\n");

    // 1. 硬件初始化
    BNO080_Init(&hi2c1, BNO080_DEFAULT_ADDRESS);
    my_printf(&huart1, "BNO080 I2C初始化完成\n");

    // 2. 硬件复位（推荐使用，更可靠）
    if (BNO080_HardwareReset() == 0) {
        my_printf(&huart1, "BNO080硬件复位成功\n");
    } else {
        my_printf(&huart1, "BNO080硬件复位失败，尝试软件复位\n");
        // 备用方案：软件复位
        softReset();
        HAL_Delay(100);
        my_printf(&huart1, "BNO080软件复位完成\n");
    }

    // 3. 基础传感器配置（用户可选择性注释/取消注释）
    enableRotationVector(100);        // 启用旋转向量，100ms间隔
    my_printf(&huart1, "已启用旋转向量报告 (100ms)\n");

    // enableAccelerometer(50);          // 启用加速度计，50ms间隔
    // my_printf(&huart1, "已启用加速度计报告 (50ms)\n");

    // enableGyro(50);                   // 启用陀螺仪，50ms间隔
    // my_printf(&huart1, "已启用陀螺仪报告 (50ms)\n");

    // enableMagnetometer(100);          // 启用磁力计，100ms间隔
    // my_printf(&huart1, "已启用磁力计报告 (100ms)\n");

    // enableLinearAccelerometer(50);    // 启用线性加速度，50ms间隔
    // my_printf(&huart1, "已启用线性加速度报告 (50ms)\n");

    // enableStepCounter(1000);          // 启用步数计数，1000ms间隔
    // my_printf(&huart1, "已启用步数计数器 (1000ms)\n");

    // 4. 高级功能配置（可选）
    // enableGameRotationVector(20);     // 游戏模式旋转向量，20ms间隔
    // my_printf(&huart1, "已启用游戏旋转向量 (20ms)\n");

    // enableStabilityClassifier(500);   // 稳定性分类器，500ms间隔
    // my_printf(&huart1, "已启用稳定性分类器 (500ms)\n");

    HAL_Delay(200); // 等待传感器配置生效
    my_printf(&huart1, "BNO080传感器初始化完成！\n");

    return 0; // 初始化成功
}

/**
 * @brief 硬件复位功能详细说明
 *
 * 新增函数：BNO080_HardwareReset()
 *
 * 功能特点：
 * 1. 通过nRESET引脚(PD4)进行硬件级复位
 * 2. 自动等待并验证Product ID Response (0xF8)
 * 3. 相比软件复位更彻底，能解决更多异常情况
 * 4. 包含超时保护机制（1秒）
 *
 * 复位时序：
 * 1. 拉低nRESET引脚20ms（确保充分复位）
 * 2. 释放nRESET引脚，传感器开始启动
 * 3. 等待100ms让传感器内部初始化
 * 4. 轮询接收Product ID Response
 * 5. 清空启动过程中的其他数据包
 *
 * 返回值：
 * - 0: 复位成功，传感器正常启动
 * - -1: 复位失败或超时
 *
 * 使用建议：
 * - 优先使用硬件复位，失败时再尝试软件复位
 * - 适用于传感器初始化和异常恢复
 * - 可配合resetReason()获取复位原因
 *
 * 示例代码见：bno08x_hardware_reset_example.c
 */

/**
 * @brief BNO080传感器数据读取和处理任务
 * @note 适用于调度器系统，建议50-100ms执行周期
 * @note 用户可通过注释选择性启用不同数据输出
 */
void bno080_task(void)
{

    // 检查是否有新数据
    if (dataAvailable()) {

        // 1. 四元数和姿态数据（基础功能）
        float quat_i = getQuatI();
        float quat_j = getQuatJ();
        float quat_k = getQuatK();
        float quat_real = getQuatReal();
        float roll, pitch, yaw;
        QuaternionToEulerAngles(quat_i, quat_j, quat_k, quat_real, &roll, &pitch, &yaw);
        my_printf(&huart1, "Euler: %.2f, %.2f, %.2f\n", roll, pitch, yaw);

        // 2. 加速度数据（可选）
        // float accel_x = getAccelX();
        // float accel_y = getAccelY();
        // float accel_z = getAccelZ();
        // my_printf(&huart1, "Accel: %.3f, %.3f, %.3f g\n", accel_x, accel_y, accel_z);

        // 3. 陀螺仪数据（可选）
        // float gyro_x = getGyroX();
        // float gyro_y = getGyroY();
        // float gyro_z = getGyroZ();
        // my_printf(&huart1, "Gyro: %.3f, %.3f, %.3f rad/s\n", gyro_x, gyro_y, gyro_z);

        // 4. 磁力计数据（可选）
        // float mag_x = getMagX();
        // float mag_y = getMagY();
        // float mag_z = getMagZ();
        // my_printf(&huart1, "Mag: %.1f, %.1f, %.1f μT\n", mag_x, mag_y, mag_z);

        // 5. 精度监控（调试用）
        // uint8_t quat_accuracy = getQuatAccuracy();
        // uint8_t accel_accuracy = getAccelAccuracy();
        // uint8_t gyro_accuracy = getGyroAccuracy();
        // uint8_t mag_accuracy = getMagAccuracy();
        // my_printf(&huart1, "Accuracy: Q=%d A=%d G=%d M=%d\n",
        //           quat_accuracy, accel_accuracy, gyro_accuracy, mag_accuracy);

        // 6. 高级功能数据（可选）
        // uint16_t steps = getStepCount();
        // uint8_t stability = getStabilityClassifier();
        // my_printf(&huart1, "Steps: %d, Stability: %d\n", steps, stability);
    }
}
