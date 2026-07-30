#include "vehicle_run_app.h"
#include "Easy_Menu_User.h"

#define VEHICLE_RUN_QUESTION_TWO_BASE_RPM   (90)
#define VEHICLE_RUN_QUESTION_FOUR_BASE_RPM  (75)
#define VEHICLE_RUN_QUESTION_FOUR_STOP_MS   (7000U)
#define VEHICLE_RUN_STOP_YAW_DEGREES        (250.0f)
#define VEHICLE_RUN_STOP_GRAY_COUNT         (3U)

typedef enum
{
    VEHICLE_RUN_PAUSED = 0,
    VEHICLE_RUN_RUNNING
} Vehicle_Run_State_t;

typedef enum
{
    VEHICLE_RUN_OWNER_NONE = 0,
    VEHICLE_RUN_OWNER_QUESTION_TWO,
    VEHICLE_RUN_OWNER_QUESTION_FOUR
} Vehicle_Run_Owner_t;

typedef struct
{
    volatile Vehicle_Run_State_t state;
    volatile uint32_t start_tick;
    volatile uint32_t elapsed_ms;
} Vehicle_Run_Context_t;

/* 题二和题四分别保存状态与计时，只共享实际电机和 PID 控制资源。 */
static Vehicle_Run_Context_t question_two_run = {
    .state = VEHICLE_RUN_PAUSED,
    .start_tick = 0U,
    .elapsed_ms = 0U,
};

static Vehicle_Run_Context_t question_four_run = {
    .state = VEHICLE_RUN_PAUSED,
    .start_tick = 0U,
    .elapsed_ms = 0U,
};

/* 当前占用电机和 PID 的题目；任何时刻最多只能有一个所有者。 */
static volatile Vehicle_Run_Owner_t vehicle_run_owner = VEHICLE_RUN_OWNER_NONE;

/* 以下转角和灰度状态仅属于题二，题四不会读取或修改。 */
static float question_two_last_yaw;
static float question_two_accumulated_yaw;
static unsigned char question_two_gray_stop_armed;

static float Vehicle_Run_Get_Current_Yaw(void)
{
#if BNO08x_ON == 0
    return icm20608.Yaw;
#else
    return bno08x.Yaw;
#endif
}

/*
 * 在中断关闭期间调用。目标转速与 PID 目标必须整体更新，避免 TIM2 中断
 * 在左右目标只更新一半时执行控制周期。
 */
static void Vehicle_Run_Apply_Base_Target(int base_target_rpm)
{
    basic_speed = base_target_rpm;
    target_speed_left = base_target_rpm;
    target_speed_right = base_target_rpm;
    pid_set_target(&pid_speed_left, target_speed_left);
    pid_set_target(&pid_speed_right, target_speed_right);
    Easy_Menu_Ui_Data.left_target = base_target_rpm;
    Easy_Menu_Ui_Data.right_target = base_target_rpm;
}

/* 调用者必须已经关闭中断，并确保 context 是当前电机所有者。 */
static void Vehicle_Run_Brake_And_Latch(Vehicle_Run_Context_t *context)
{
    pid_running = 0U;
    Motor_Brake(&left_motor);
    Motor_Brake(&right_motor);
    context->elapsed_ms = HAL_GetTick() - context->start_tick;
    context->state = VEHICLE_RUN_PAUSED;
    vehicle_run_owner = VEHICLE_RUN_OWNER_NONE;
}

/* 在另一道题接管电机前急停并锁存原运行时间。调用者必须已关闭中断。 */
static void Vehicle_Run_Release_Current_Owner(Vehicle_Run_Owner_t next_owner)
{
    if((vehicle_run_owner == VEHICLE_RUN_OWNER_QUESTION_TWO) &&
       (next_owner != VEHICLE_RUN_OWNER_QUESTION_TWO) &&
       (question_two_run.state == VEHICLE_RUN_RUNNING))
    {
        Vehicle_Run_Brake_And_Latch(&question_two_run);
    }
    else if((vehicle_run_owner == VEHICLE_RUN_OWNER_QUESTION_FOUR) &&
            (next_owner != VEHICLE_RUN_OWNER_QUESTION_FOUR) &&
            (question_four_run.state == VEHICLE_RUN_RUNNING))
    {
        Vehicle_Run_Brake_And_Latch(&question_four_run);
    }
}

static void Vehicle_Run_Start_Context(Vehicle_Run_Context_t *context,
                                      Vehicle_Run_Owner_t owner,
                                      int base_target_rpm)
{
    uint32_t interrupt_state;

    /* ebtn 长按会重复发送输入，同一道题运行中忽略重复启动，避免重新计时。 */
    if((vehicle_run_owner == owner) && (context->state == VEHICLE_RUN_RUNNING))
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();

    Vehicle_Run_Release_Current_Owner(owner);
    pid_running = 0U;
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);
    pid_reset(&pid_speed_left);
    pid_reset(&pid_speed_right);
    pid_reset(&pid_angle);
    pid_reset(&pid_line);
    Vehicle_Run_Apply_Base_Target(base_target_rpm);

    if(owner == VEHICLE_RUN_OWNER_QUESTION_TWO)
    {
        question_two_last_yaw = Vehicle_Run_Get_Current_Yaw();
        question_two_accumulated_yaw = 0.0f;
        question_two_gray_stop_armed = 0U;
    }

    context->elapsed_ms = 0U;
    context->start_tick = HAL_GetTick();
    context->state = VEHICLE_RUN_RUNNING;
    vehicle_run_owner = owner;

    /* 所有状态准备完成后最后启用 PID，首个控制周期会看到完整新状态。 */
    pid_running = 1U;

    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}

static void Vehicle_Run_Reset_Context(Vehicle_Run_Context_t *context,
                                      Vehicle_Run_Owner_t owner,
                                      int base_target_rpm)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();

    /*
     * PID 和目标转速属于共享物理资源。复位当前题前先释放另一题，避免在另一题
     * 仍运行时写入当前题的基础目标值。
     */
    Vehicle_Run_Release_Current_Owner(owner);
    pid_running = 0U;
    pid_reset(&pid_speed_left);
    pid_reset(&pid_speed_right);
    pid_reset(&pid_angle);
    pid_reset(&pid_line);
    Motor_Stop(&left_motor);
    Motor_Stop(&right_motor);
    Vehicle_Run_Apply_Base_Target(base_target_rpm);

    if(owner == VEHICLE_RUN_OWNER_QUESTION_TWO)
    {
        question_two_last_yaw = Vehicle_Run_Get_Current_Yaw();
        question_two_accumulated_yaw = 0.0f;
        question_two_gray_stop_armed = 0U;
    }

    context->start_tick = 0U;
    context->elapsed_ms = 0U;
    context->state = VEHICLE_RUN_PAUSED;
    vehicle_run_owner = VEHICLE_RUN_OWNER_NONE;

    if(interrupt_state == 0U)
    {
        __enable_irq();
    }
}

static uint32_t Vehicle_Run_Get_Context_Elapsed_Ms(const Vehicle_Run_Context_t *context)
{
    /* HAL tick 使用无符号减法，系统 tick 回绕时仍能得到正确时间差。 */
    if(context->state == VEHICLE_RUN_RUNNING)
    {
        return HAL_GetTick() - context->start_tick;
    }

    return context->elapsed_ms;
}

void Vehicle_Run_Start(void)
{
    Vehicle_Run_Start_Context(&question_two_run,
                              VEHICLE_RUN_OWNER_QUESTION_TWO,
                              VEHICLE_RUN_QUESTION_TWO_BASE_RPM);
}

void Vehicle_Run_Reset(void)
{
    Vehicle_Run_Reset_Context(&question_two_run,
                              VEHICLE_RUN_OWNER_QUESTION_TWO,
                              VEHICLE_RUN_QUESTION_TWO_BASE_RPM);
}

void Vehicle_Run_Question4_Start(void)
{
    Vehicle_Run_Start_Context(&question_four_run,
                              VEHICLE_RUN_OWNER_QUESTION_FOUR,
                              VEHICLE_RUN_QUESTION_FOUR_BASE_RPM);
}

void Vehicle_Run_Question4_Reset(void)
{
    Vehicle_Run_Reset_Context(&question_four_run,
                              VEHICLE_RUN_OWNER_QUESTION_FOUR,
                              VEHICLE_RUN_QUESTION_FOUR_BASE_RPM);
}

void Vehicle_Run_Task(void)
{
    float current_yaw;
    float yaw_delta;
    uint8_t active_gray_count = 0U;
    uint8_t channel_index;
    uint32_t interrupt_state;

    if((vehicle_run_owner == VEHICLE_RUN_OWNER_QUESTION_FOUR) &&
       (question_four_run.state == VEHICLE_RUN_RUNNING))
    {
        if((HAL_GetTick() - question_four_run.start_tick) >=
           VEHICLE_RUN_QUESTION_FOUR_STOP_MS)
        {
            interrupt_state = __get_PRIMASK();
            __disable_irq();
            Vehicle_Run_Brake_And_Latch(&question_four_run);

            if(interrupt_state == 0U)
            {
                __enable_irq();
            }
        }
        return;
    }

    if((vehicle_run_owner != VEHICLE_RUN_OWNER_QUESTION_TWO) ||
       (question_two_run.state != VEHICLE_RUN_RUNNING))
    {
        return;
    }

    current_yaw = Vehicle_Run_Get_Current_Yaw();
    yaw_delta = current_yaw - question_two_last_yaw;

    /* 航向角在正负 180 度附近回绕，补偿跨界差值后再累计本轮净转角。 */
    if(yaw_delta > 180.0f)
    {
        yaw_delta -= 360.0f;
    }
    else if(yaw_delta < -180.0f)
    {
        yaw_delta += 360.0f;
    }

    question_two_last_yaw = current_yaw;
    question_two_accumulated_yaw += yaw_delta;

    if((question_two_accumulated_yaw >= VEHICLE_RUN_STOP_YAW_DEGREES) ||
       (question_two_accumulated_yaw <= -VEHICLE_RUN_STOP_YAW_DEGREES))
    {
        question_two_gray_stop_armed = 1U;
    }

    if(question_two_gray_stop_armed == 0U)
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
        interrupt_state = __get_PRIMASK();
        __disable_irq();
        Vehicle_Run_Brake_And_Latch(&question_two_run);

        if(interrupt_state == 0U)
        {
            __enable_irq();
        }
    }
}

unsigned char Vehicle_Run_Get_State(void)
{
    return (question_two_run.state == VEHICLE_RUN_RUNNING) ? 1U : 0U;
}

uint32_t Vehicle_Run_Get_Elapsed_Ms(void)
{
    return Vehicle_Run_Get_Context_Elapsed_Ms(&question_two_run);
}

unsigned char Vehicle_Run_Question4_Get_State(void)
{
    return (question_four_run.state == VEHICLE_RUN_RUNNING) ? 1U : 0U;
}

uint32_t Vehicle_Run_Question4_Get_Elapsed_Ms(void)
{
    return Vehicle_Run_Get_Context_Elapsed_Ms(&question_four_run);
}
