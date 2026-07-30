#include "pid_app.h"
#include "Easy_Menu_User.h"

/*
 * 根据 12 组正转实测数据做最小二乘拟合：
 * 左轮 40~151 rpm、右轮 45~156 rpm，对应 PWM 均为 575~685。
 * 公式为 base_pwm = slope * |target_rpm| + intercept，左右轮分别拟合，
 * 用于补偿同一 PWM 下左右轮转速不一致；速度 PID 只修正剩余误差。
 */
#define LEFT_BASE_PWM_SLOPE       0.9262202f
#define LEFT_BASE_PWM_INTERCEPT   537.11115f
#define RIGHT_BASE_PWM_SLOPE      0.9850767f
#define RIGHT_BASE_PWM_INTERCEPT  531.65649f
#define LINE_OUTPUT_LPF_ALPHA     0.35f  /* 新值权重，越大则响应越快、滤波越弱 */
#define ANGLE_INTEGRAL_LIMIT      100.0f
#define YAW_RATE_JUMP_LIMIT       65.0f

static float target_yaw_rate_lpf;                    /* 滤波后的外环目标角速度 */
static unsigned char line_output_filter_initialized; /* 首帧初始化标志 */
static float last_valid_yaw_rate;                    /* 中环上一次有效角速度 */
static unsigned char yaw_rate_filter_initialized;    /* 角速度跳变过滤初始化标志 */

int basic_speed = 120; // 基础速度（单位：rpm）
volatile int target_speed_left;
volatile int target_speed_right;

/* PID 控制器实例 */
PID_T pid_speed_left;  // 左轮速度环
PID_T pid_speed_right; // 右轮速度环
PID_T pid_angle;       // 角速度环，输出轮速差（rpm）
PID_T pid_line;        // 循迹外环，输出目标角速度（deg/s）

/* PID 参数定义 */
PidParams_t pid_params_left = {
    .kp = 4.0f,        
    .ki = 0.0000f,      
    .kd = 0.5f,      
    .out_min = -999.0f,
    .out_max = 999.0f,
};

PidParams_t pid_params_right = {
    .kp = 5.0f,        
    .ki = 0.0000f,      
    .kd = 0.4f,      
    .out_min = -999.0f,
    .out_max = 999.0f,
};

/* 外环初值仅用于建立控制链路，需结合实车单位和响应继续调参。 */
PidParams_t pid_params_angle = {
    .kp = 10.0f,
    .ki = 0.075f,
    .kd = 1.0f,
    .out_min = -70.0f,
    .out_max = 70.0f,
};

PidParams_t pid_params_line = {
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_min = -80.0f,
    .out_max = 80.0f,
};

/*
 * 零转速必须直接返回 0，避免拟合截距在停车目标下仍驱动电机。
 * 超出实测转速范围时公式属于外推，因此最终按对应定时器周期限幅。
 * 当前没有反转实测数据，负目标暂时复用正转曲线并改变输出符号。
 */
static int PID_Calculate_BasePwm(float target_rpm,
                                 float slope,
                                 float intercept,
                                 int pwm_limit)
{
    float absolute_target_rpm;
    float base_pwm;
    int rounded_pwm;

    if(target_rpm == 0.0f) return 0;

    absolute_target_rpm = target_rpm > 0.0f ? target_rpm : -target_rpm;
    base_pwm = slope * absolute_target_rpm + intercept;
    base_pwm = pid_constrain(base_pwm, 0.0f, (float)pwm_limit);
    rounded_pwm = (int)(base_pwm + 0.5f);

    /* 当前只有正转实测数据，反转暂时复用同一拟合曲线并改变符号。 */
    return target_rpm > 0.0f ? rounded_pwm : -rounded_pwm;
}

void PID_Init(void)
{
    pid_init(&pid_speed_left,
            pid_params_left.kp, pid_params_left.ki, pid_params_left.kd,
            0.0f, pid_params_left.out_max);
    
    pid_init(&pid_speed_right,
            pid_params_right.kp, pid_params_right.ki, pid_params_right.kd,
            0.0f, pid_params_right.out_max);

    pid_init(&pid_angle,
            pid_params_angle.kp, pid_params_angle.ki, pid_params_angle.kd,
            0.0f, pid_params_angle.out_max);

    pid_init(&pid_line,
            pid_params_line.kp, pid_params_line.ki, pid_params_line.kd,
            0.0f, pid_params_line.out_max);

    target_yaw_rate_lpf = 0.0f;
    line_output_filter_initialized = 0U;
    last_valid_yaw_rate = 0.0f;
    yaw_rate_filter_initialized = 0U;
    
    
    target_speed_left = basic_speed;
    target_speed_right = basic_speed;
    pid_set_target(&pid_speed_left, target_speed_left);
    pid_set_target(&pid_speed_right, target_speed_right);

    Easy_Menu_Ui_Data.left_target = basic_speed;
    Easy_Menu_Ui_Data.left_kp = pid_params_left.kp;
    Easy_Menu_Ui_Data.left_ki = pid_params_left.ki;
    Easy_Menu_Ui_Data.left_kd = pid_params_left.kd;
    Easy_Menu_Ui_Data.right_target = basic_speed;
    Easy_Menu_Ui_Data.right_kp = pid_params_right.kp;
    Easy_Menu_Ui_Data.right_ki = pid_params_right.ki;
    Easy_Menu_Ui_Data.right_kd = pid_params_right.kd;
}

volatile unsigned char pid_running = 0; // 默认暂停，等待计时页面启动新一轮运行

void PID_Task(void)
{
    if(pid_running == 0)
    {
        line_output_filter_initialized = 0U;
        yaw_rate_filter_initialized = 0U;
        return;
    }
    
    float target_yaw_rate;
    float wheel_speed_diff;
    float current_yaw_rate;
    float left_target_rpm;
    float right_target_rpm;
    float output_left_float;
    float output_right_float;
    int left_base_pwm;
    int right_base_pwm;
    int output_left;
    int output_right;
  
    /*
     * 级联控制：循迹误差为正表示车辆偏右，因此循迹环输出负的目标角速度
     * 进行左转纠偏。角速度反馈统一为右转为正，并扣除已完成的静态偏置。
     */
    target_yaw_rate = pid_calculate_positional(&pid_line, g_line_position_error);
    target_yaw_rate = pid_constrain(target_yaw_rate,
                                    pid_params_line.out_min,
                                    pid_params_line.out_max);
    if(line_output_filter_initialized == 0U)
    {
        /* 首帧直接跟随当前输出，避免每次启动时从零缓慢爬升。 */
        target_yaw_rate_lpf = target_yaw_rate;
        line_output_filter_initialized = 1U;
    }
    else
    {
        target_yaw_rate_lpf += LINE_OUTPUT_LPF_ALPHA *
                               (target_yaw_rate - target_yaw_rate_lpf);
    }
    target_yaw_rate = target_yaw_rate_lpf;
    pid_set_target(&pid_angle, target_yaw_rate);
    current_yaw_rate = icm20608.gyro.z - euler_angles.gz_bias;

    if(yaw_rate_filter_initialized == 0U)
    {
        last_valid_yaw_rate = current_yaw_rate;
        yaw_rate_filter_initialized = 1U;
    }
    else if((current_yaw_rate - last_valid_yaw_rate > YAW_RATE_JUMP_LIMIT) ||
            (last_valid_yaw_rate - current_yaw_rate > YAW_RATE_JUMP_LIMIT))
    {
        /* 单周期跳变过大时沿用上一次有效值，避免尖峰使中环瞬时反向饱和。 */
        current_yaw_rate = last_valid_yaw_rate;
    }
    else
    {
        last_valid_yaw_rate = current_yaw_rate;
    }

    wheel_speed_diff = pid_calculate_positional(&pid_angle, current_yaw_rate);

    /* 中环积分只用于补偿稳态偏差，避免持续小误差将轮速差推至输出限幅。 */
    pid_app_limit_integral(&pid_angle,
                           -ANGLE_INTEGRAL_LIMIT,
                           ANGLE_INTEGRAL_LIMIT);
    wheel_speed_diff = pid_constrain(wheel_speed_diff,
                                     pid_params_angle.out_min,
                                     pid_params_angle.out_max);
    // Uart_Printf(DEBUG_UART, "Target Yaw Rate: %.2f, Wheel Speed Diff: %.2f, Current Yaw Rate: %.2f\r\n",
    //         target_yaw_rate, wheel_speed_diff, current_yaw_rate);

    /* 右转为正：左轮加速、右轮减速，轮速差单位为 rpm。 */
    left_target_rpm = (float)target_speed_left + wheel_speed_diff;
    right_target_rpm = (float)target_speed_right - wheel_speed_diff;
    pid_set_target(&pid_speed_left, left_target_rpm);
    pid_set_target(&pid_speed_right, right_target_rpm);

    /*
     * 使用叠加轮速差后的最终左右轮目标计算前馈，使转向时两侧基础 PWM
     * 能分别跟随各自目标转速变化，再由速度 PID 修正剩余误差。
     */
    left_base_pwm = PID_Calculate_BasePwm(left_target_rpm,
                                          LEFT_BASE_PWM_SLOPE,
                                          LEFT_BASE_PWM_INTERCEPT,
                                          (int)left_motor.config.in1.htim->Init.Period);
    right_base_pwm = PID_Calculate_BasePwm(right_target_rpm,
                                           RIGHT_BASE_PWM_SLOPE,
                                           RIGHT_BASE_PWM_INTERCEPT,
                                           (int)right_motor.config.in1.htim->Init.Period);

    // 使用位置式 PID 计算左右轮速度环输出
    output_left_float = pid_calculate_positional(&pid_speed_left, left_encoder.rpm);
    output_right_float = pid_calculate_positional(&pid_speed_right, right_encoder.rpm);

    // PID_T 仍使用对称内部限幅，这里按串口配置的独立上下限做最终限幅。
    output_left = (int)pid_constrain(output_left_float,
                                     pid_params_left.out_min,
                                     pid_params_left.out_max);
    output_right = (int)pid_constrain(output_right_float,
                                      pid_params_right.out_min,
                                      pid_params_right.out_max);
  
    // 设置电机速度
    Motor_Set_Speed(&left_motor, left_base_pwm + output_left);
    Motor_Set_Speed(&right_motor, right_base_pwm + output_right);
}
