#include "vehicle_run_app.h"

#define VEHICLE_RUN_STOP_YAW_DEGREES  (300.0f)
#define VEHICLE_RUN_STOP_GRAY_COUNT   (3U)

/* 运行状态只在本模块内部使用，避免把题二状态机泄漏给其他业务模块。 */
typedef enum
{
    VEHICLE_RUN_PAUSED = 0,
    VEHICLE_RUN_RUNNING
} Vehicle_Run_State_t;

/*
 * 状态和计时量会分别在主循环按键路径、TIM2 的 10 ms 实时路径和 OLED 查询
 * 路径访问，因此声明为 volatile，防止编译器缓存跨上下文读取的旧值。
 */
/* 当前题二状态；启动/复位/灰度采集路径写入，OLED 查询路径读取。 */
static volatile Vehicle_Run_State_t vehicle_run_state = VEHICLE_RUN_PAUSED;

/* 本轮按下按键 1 时的 HAL tick，单位 ms；仅在运行期间作为计时基准。 */
static volatile uint32_t vehicle_run_start_tick;

/* 暂停时锁存的本轮用时，单位 ms；自动停车保留，按键 2 复位清零。 */
static volatile uint32_t vehicle_run_elapsed_ms;

/* 上一次灰度采集完成时的航向角，单位度，用于累计相邻采样间的净转角。 */
static float vehicle_run_last_yaw;

/* 从本轮启动开始累计的净航向变化，单位度，支持正、负两个转向方向。 */
static float vehicle_run_accumulated_yaw;

/* 达到 300 度后保持有效，避免小幅反向修正反复关闭灰度停车检测。 */
static unsigned char vehicle_run_gray_stop_armed;

static float Vehicle_Run_Get_Current_Yaw(void)
{
#if BNO08x_ON == 0
    return icm20608.Yaw;
#else
    return bno08x.Yaw;
#endif
}

/*
 * 灰度停车条件成立时立即执行刹车并锁存时间。先关闭 pid_running，再将双电机
 * 切换为刹车状态，后续 PID_Task 会因暂停直接返回，不能覆盖刹车输出。
 */
static void Vehicle_Run_Stop_And_Latch_Time(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();

    pid_running = 0U;
    Motor_Brake(&left_motor);
    Motor_Brake(&right_motor);
    vehicle_run_elapsed_ms = HAL_GetTick() - vehicle_run_start_tick;
    vehicle_run_state = VEHICLE_RUN_PAUSED;

    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}

void Vehicle_Run_Start(void)
{
    /* 保存进入临界区前的全局中断状态，退出时只恢复原本开启的中断。 */
    uint32_t interrupt_state;

    /* ebtn 长按会周期性重复发送按键输入，运行中忽略重复启动以免重新计时。 */
    if(vehicle_run_state == VEHICLE_RUN_RUNNING)
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    /*
     * 启动操作来自主循环，而 PID_Task 位于定时器中断。整个状态切换放在临界区，
     * 防止 PID 在控制器复位一半时执行，或沿用上一轮积分、微分和电机输出。
     */
    pid_running = 0U;
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);
    pid_reset(&pid_speed_left);
    pid_reset(&pid_speed_right);
    pid_reset(&pid_angle);
    pid_reset(&pid_line);

    vehicle_run_last_yaw = Vehicle_Run_Get_Current_Yaw();
    vehicle_run_accumulated_yaw = 0.0f;
    vehicle_run_gray_stop_armed = 0U;
    vehicle_run_elapsed_ms = 0U;
    vehicle_run_start_tick = HAL_GetTick();
    vehicle_run_state = VEHICLE_RUN_RUNNING;

    /* 所有起始状态准备完成后，最后启用 PID，保证首个控制周期看到完整新状态。 */
    pid_running = 1U;

    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}

void Vehicle_Run_Reset(void)
{
    /* 保存进入临界区前的全局中断状态，避免误开启调用前已关闭的中断。 */
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();

    /*
     * 复位可能发生在运行中，必须先禁用 PID，再清控制器并显式停止左右电机。
     * 这与 Start 使用同一临界区规则，避免中断与按键回调竞争修改 PWM 和状态。
     */
    pid_running = 0U;
    pid_reset(&pid_speed_left);
    pid_reset(&pid_speed_right);
    pid_reset(&pid_angle);
    pid_reset(&pid_line);
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);

    vehicle_run_start_tick = 0U;
    vehicle_run_elapsed_ms = 0U;
    vehicle_run_last_yaw = Vehicle_Run_Get_Current_Yaw();
    vehicle_run_accumulated_yaw = 0.0f;
    vehicle_run_gray_stop_armed = 0U;
    vehicle_run_state = VEHICLE_RUN_PAUSED;

    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}

void Vehicle_Run_Task(void)
{
    float current_yaw;
    float yaw_delta;
    uint8_t active_gray_count = 0U;
    uint8_t channel_index;

    if(vehicle_run_state != VEHICLE_RUN_RUNNING)
    {
        return;
    }

    current_yaw = Vehicle_Run_Get_Current_Yaw();
    yaw_delta = current_yaw - vehicle_run_last_yaw;

    /* 航向角在正负 180 度附近回绕，补偿跨界差值后再累计本轮净转角。 */
    if(yaw_delta > 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    else if(yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }

    vehicle_run_last_yaw = current_yaw;
    vehicle_run_accumulated_yaw += yaw_delta;

    if((vehicle_run_accumulated_yaw >= VEHICLE_RUN_STOP_YAW_DEGREES) ||
       (vehicle_run_accumulated_yaw <= -VEHICLE_RUN_STOP_YAW_DEGREES))
    {
        vehicle_run_gray_stop_armed = 1U;
    }

    if(vehicle_run_gray_stop_armed == 0U)
    {
        return;
    }

    for(channel_index = 0U; channel_index < GRAY_CHANNEL_COUNT; channel_index++)
    {
        if(gray_digtal[channel_index] == 1U)
        {
            active_gray_count++;
        }
    }

    if(active_gray_count >= VEHICLE_RUN_STOP_GRAY_COUNT)
    {
        Vehicle_Run_Stop_And_Latch_Time();
    }
}

unsigned char Vehicle_Run_Get_State(void)
{
    return (vehicle_run_state == VEHICLE_RUN_RUNNING) ? 1U : 0U;
}

uint32_t Vehicle_Run_Get_Elapsed_Ms(void)
{
    /* HAL tick 使用无符号减法，系统 tick 回绕时仍能得到正确的时间差。 */
    if(vehicle_run_state == VEHICLE_RUN_RUNNING)
    {
        return HAL_GetTick() - vehicle_run_start_tick;
    }

    return vehicle_run_elapsed_ms;
}
