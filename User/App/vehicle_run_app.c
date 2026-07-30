#include "vehicle_run_app.h"

/* ============================== 题二用户可调参数 ============================== */
/*
 * 自动停车所需的累计净航向角绝对值，单位：度。
 *
 * 修改示例：
 *   360.0f：累计到 +360 度或 -360 度时停车（一圈）。
 *   720.0f：累计到 +720 度或 -720 度时停车（两圈）。
 *
 * 必须填写大于 0 的浮点数并保留 f 后缀。程序会同时判断正、负两个方向，
 * 因此这里只配置一个正数阈值，不要为了反向行驶而把宏改成负数。
 */
#define VEHICLE_RUN_LAP_YAW_DEGREES  (360.0f)
/* ============================================================================ */

/* 运行状态只在本模块内部使用，避免把题二状态机泄漏给其他业务模块。 */
typedef enum
{
    VEHICLE_RUN_PAUSED = 0,
    VEHICLE_RUN_RUNNING
} Vehicle_Run_State_t;

/*
 * 状态和计时量会分别在主循环按键路径、TIM2 的 10 ms 实时路径和 OLED 查询
 * 路径访问，因此声明为 volatile，防止编译器缓存跨上下文读取的旧值。
 * 航向历史量只在启动/复位临界区和 10 ms 实时任务中访问。
 */
/* 当前题二状态；启动/复位路径写入，10 ms 任务和 OLED 查询路径读取。 */
static volatile Vehicle_Run_State_t vehicle_run_state = VEHICLE_RUN_PAUSED;

/* 本轮按下按键 1 时的 HAL tick，单位 ms；仅在运行期间作为计时基准。 */
static volatile uint32_t vehicle_run_start_tick;

/* 暂停时锁存的本轮用时，单位 ms；自动停车保留，按键 2 复位清零。 */
static volatile uint32_t vehicle_run_elapsed_ms;

/* 上一个 10 ms 周期的传感器航向角，单位度，用于计算相邻帧角度差。 */
static float vehicle_run_last_yaw;

/* 从发车开始累计的连续净航向变化，单位度；允许正、负两个转向方向。 */
static float vehicle_run_accumulated_yaw;

/* 一圈完成锁止标志：为 1 时拒绝再次启动，直到按键 2 明确复位。 */
static volatile unsigned char vehicle_run_lap_completed;

/*
 * 屏蔽具体姿态传感器，使题二逻辑始终使用项目当前启用的车体航向角。
 * 返回单位为度；ICM20608 当前输出为正负 180 度附近回绕的角度。
 */
static float Vehicle_Run_Get_Current_Yaw(void)
{
#if BNO08x_ON == 0
    return icm20608.Yaw;
#else
    return bno08x.Yaw;
#endif
}

/*
 * 一圈完成时立即执行安全停车并锁存时间。
 * 调用位置在 10 ms 实时任务内且早于 PID_Task：先关闭 pid_running 并将双电机
 * PWM 置零，随后同周期 PID_Task 会因暂停直接返回，不能再次写入电机输出。
 */
static void Vehicle_Run_Stop_And_Latch_Time(void)
{
    pid_running = 0U;
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);
    vehicle_run_elapsed_ms = HAL_GetTick() - vehicle_run_start_tick;
    vehicle_run_state = VEHICLE_RUN_PAUSED;
    vehicle_run_lap_completed = 1U;
}

void Vehicle_Run_Start(void)
{
    /* 保存进入临界区前的全局中断状态，退出时只恢复原本开启的中断。 */
    uint32_t interrupt_state;

    /*
     * ebtn 长按会周期性重复发送按键输入。运行中忽略重复启动，避免开始时间
     * 被反复改写；完成一圈后也保持锁止，要求按键 2 明确复位后才能开始第二轮。
     */
    if((vehicle_run_state == VEHICLE_RUN_RUNNING) ||
       (vehicle_run_lap_completed != 0U))
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

    /* 清除完成锁止后，下一次按键 1 才允许开始第二轮。 */
    vehicle_run_lap_completed = 0U;
    vehicle_run_state = VEHICLE_RUN_PAUSED;

    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}

void Vehicle_Run_Task(void)
{
    /* 本周期从当前启用的姿态传感器读取到的车体航向角，单位度。 */
    float current_yaw;

    /* 当前航向与上一周期航向的差值；跨正负 180 度边界后会被解包。 */
    float yaw_delta;

    if(vehicle_run_state != VEHICLE_RUN_RUNNING)
    {
        return;
    }

    current_yaw = Vehicle_Run_Get_Current_Yaw();
    yaw_delta = current_yaw - vehicle_run_last_yaw;

    /*
     * 航向角在正负 180 度处回绕。例如上一帧为 +179 度、当前帧为 -179 度，
     * 直接相减会得到 -358 度，但真实变化只有 +2 度。将跨界差值补偿 360 度后，
     * 每个 10 ms 周期得到连续的小角度变化，再累计为本轮净转角。
     */
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

    /* 同时支持顺时针和逆时针行驶，任一方向累计达到一圈都触发停车。 */
    if((vehicle_run_accumulated_yaw >= VEHICLE_RUN_LAP_YAW_DEGREES) ||
       (vehicle_run_accumulated_yaw <= -VEHICLE_RUN_LAP_YAW_DEGREES))
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
