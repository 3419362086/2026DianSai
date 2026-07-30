#include "pid_app.h"
#include "Easy_Menu_User.h"

/*
 * 由实测正转数据（PWM 575~700）最小二乘拟合得到左右轮前馈曲线。
 * 按当前控制需求，目标转速超出实测范围后仍沿直线外推，不在 RPM 端点截断；
 * 前馈与 PID 修正量相加后，再按电机定时器周期执行最终 PWM 限幅。
 */
#define LEFT_SPEED_FF_SLOPE       1.3820629f
#define LEFT_SPEED_FF_INTERCEPT 538.4571232f
#define RIGHT_SPEED_FF_SLOPE      1.3961710f
#define RIGHT_SPEED_FF_INTERCEPT 544.9268843f

/* 用于初始化 PID 控制器的参数 */
//60跑一圈27s
//90跑一圈18s
//115跑一圈14s
//60跑半圈
//
int basic_speed = 115; // 基础速度（单位：rpm）
volatile int target_speed_left;
volatile int target_speed_right;

/* PID 控制器实例 */
PID_T pid_speed_left;  // 左轮速度环
PID_T pid_speed_right; // 右轮速度环
PID_T pid_angle;       // 角速度环保留，当前控制链路不调用
PID_T pid_line;        // 循迹环，直接输出轮速差（rpm）

/* PID 参数定义 */
PidParams_t pid_params_left = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_min = -500.0f,
    .out_max = 500.0f,
};

/* 左右速度环当前使用纯前馈，PID 修正参数保留为零。 */
PidParams_t pid_params_right = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_min = -500.0f,
    .out_max = 500.0f,
};

/* 外环初值仅用于建立控制链路，需结合实车单位和响应继续调参。 */
PidParams_t pid_params_angle = {
    .kp = 10.0f,
    .ki = 0.075f,
    .kd = 1.0f,
    .out_min = -65.0f,
    .out_max = 65.0f,
};

/* 灰度权重范围为 -3~+3；当前纯 P 最大输出约 +/-90 rpm，限幅保留到 +/-200 rpm。 */
PidParams_t pid_params_line = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .out_min = -200.0f,
    .out_max = 200.0f,
};

static float PID_Calculate_Speed_Feedforward(float target_rpm,
                                             float slope,
                                             float intercept)
{
    float absolute_target_rpm;
    float feedforward_pwm;

    /* 停车目标不能带入拟合截距，否则 0 rpm 时仍会输出约 540 PWM。 */
    if(target_rpm == 0.0f) return 0.0f;

    absolute_target_rpm = target_rpm > 0.0f ? target_rpm : -target_rpm;
    feedforward_pwm = slope * absolute_target_rpm + intercept;

    /* 尚无反转实测曲线，当前暂按正转曲线对称处理，需后续实车验证。 */
    return target_rpm > 0.0f ? feedforward_pwm : -feedforward_pwm;
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

static float PID_Run_Line_Loop(void)
{
    float wheel_speed_diff;

    wheel_speed_diff = pid_calculate_positional(&pid_line, g_line_position_error);
    return pid_constrain(wheel_speed_diff,
                         pid_params_line.out_min,
                         pid_params_line.out_max);
}

/* 保留角速度环实现，当前按用户要求旁路，不参与 PID_Task 控制链路。 */
static float __attribute__((unused)) PID_Run_Angle_Loop(float target_yaw_rate)
{
    float current_yaw_rate;
    float wheel_speed_diff;

    pid_set_target(&pid_angle, target_yaw_rate);
    current_yaw_rate = icm20608.gyro.z - euler_angles.gz_bias;
    wheel_speed_diff = pid_calculate_positional(&pid_angle, current_yaw_rate);

    return pid_constrain(wheel_speed_diff,
                         pid_params_angle.out_min,
                         pid_params_angle.out_max);
}

static void PID_Run_Speed_Loop(float wheel_speed_diff)
{
    float left_target_rpm;
    float right_target_rpm;
    float left_feedforward_pwm;
    float right_feedforward_pwm;
    float left_pid_correction;
    float right_pid_correction;
    float left_command_pwm;
    float right_command_pwm;
    int left_pwm_limit;
    int right_pwm_limit;
    int output_left;
    int output_right;

    /* 循迹环直接给出轮速差；角速度环当前保持旁路。 */
    left_target_rpm = (float)target_speed_left + wheel_speed_diff;
    right_target_rpm = (float)target_speed_right - wheel_speed_diff;
    pid_set_target(&pid_speed_left, left_target_rpm);
    pid_set_target(&pid_speed_right, right_target_rpm);

    left_feedforward_pwm = PID_Calculate_Speed_Feedforward(
                               left_target_rpm,
                               LEFT_SPEED_FF_SLOPE,
                               LEFT_SPEED_FF_INTERCEPT);
    right_feedforward_pwm = PID_Calculate_Speed_Feedforward(
                                right_target_rpm,
                                RIGHT_SPEED_FF_SLOPE,
                                RIGHT_SPEED_FF_INTERCEPT);

    /* PID 修正量与前馈相加，再统一按各自 PWM 周期限幅。 */
    left_pid_correction = pid_calculate_positional(&pid_speed_left,
                                                    left_encoder.rpm);
    right_pid_correction = pid_calculate_positional(&pid_speed_right,
                                                     right_encoder.rpm);
    left_pid_correction = pid_constrain(left_pid_correction,
                                        pid_params_left.out_min,
                                        pid_params_left.out_max);
    right_pid_correction = pid_constrain(right_pid_correction,
                                         pid_params_right.out_min,
                                         pid_params_right.out_max);

    left_command_pwm = left_feedforward_pwm + left_pid_correction;
    right_command_pwm = right_feedforward_pwm + right_pid_correction;
    left_pwm_limit = (int)left_motor.config.in1.htim->Init.Period;
    right_pwm_limit = (int)right_motor.config.in1.htim->Init.Period;
    output_left = (int)pid_constrain(left_command_pwm,
                                     (float)-left_pwm_limit,
                                     (float)left_pwm_limit);
    output_right = (int)pid_constrain(right_command_pwm,
                                      (float)-right_pwm_limit,
                                      (float)right_pwm_limit);

    Motor_Set_Speed(&left_motor, output_left);
    Motor_Set_Speed(&right_motor, output_right);
}

void PID_Task(void)
{
    float wheel_speed_diff;

    if(pid_running == 0) return;

    wheel_speed_diff = PID_Run_Line_Loop();
    PID_Run_Speed_Loop(wheel_speed_diff);
}
