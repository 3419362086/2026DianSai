#include "ball_control.h"

#include "main.h"
#include "uart_driver.h"
#include "usart.h"
#include "zdt_motor_driver.h"

#include <string.h>

/* 已确认的机械、传动和视觉量程参数。 */
#define BALL_CONTROL_POSITION_LIMIT_CM              12.5f  /* 摄像头坐标范围，单位 cm。 */
#define BALL_CONTROL_ROD_ANGLE_LIMIT_DEG             5.0f  /* 软件限位只禁止继续向外，不禁止回中心。 */
#define BALL_CONTROL_MOTOR_RPM_TO_ROD_DEG_S          0.6f  /* 1 RPM 电机转速对应的摆杆角速度。 */

/*
 * 视觉观测器首版参数，当前只通过编译验证，必须根据 BC 遥测实测后再定稿。
 * 可信度保留在通信协议中，但不作为帧门槛，也不参与观测器增益计算。
 * 观测器只在上电或 Ball_Control_Reset 后重新初始化，不按帧间隔或超时退出。
 */
#define BALL_CONTROL_VISION_MAX_SPEED_CM_S          200.0f /* 用于拒绝不合理的单帧位置跳变。 */
#define BALL_CONTROL_OBSERVER_ALPHA                   0.75f /* 固定位置残差融合增益。 */
#define BALL_CONTROL_OBSERVER_BETA                    0.30f /* 提高速度对位置残差的跟随，降低动态滞后。 */
#define BALL_CONTROL_ACCELERATION_FILTER_GAIN          0.20f /* 加速度一阶低通系数。 */

/*
 * 第 4 阶段速度中环手动调参区：修改后重新编译下载，不通过串口改参数。
 * 首轮保持 v_ref=0、Ka=0，只验证零速制动方向；随后依次测试 +2、-2 cm/s。
 * Kv 的单位为 (deg/s)/(cm/s)，Ka 的单位为 (deg/s)/(cm/s^2)。
 */
#define BALL_CONTROL_VELOCITY_TARGET_CM_S               0.0f
#define BALL_CONTROL_VELOCITY_KV                         0.20f
#define BALL_CONTROL_ACCELERATION_KA                      0.20f

/* 仅保护 ZDT 命令的 uint16_t 速度字段，不是机械最大转速限制。 */
#define BALL_CONTROL_MOTOR_PROTOCOL_MAX_RPM          65535.0f

/* 调试遥测走 USART1，限频以避免影响 10 ms 控制任务。 */
#define BALL_CONTROL_TELEMETRY_ENABLED                 1U
#define BALL_CONTROL_TELEMETRY_INTERVAL_MS           200U

/*
 * 已确认的执行器约束：
 * 1. Y 轴电机正转时摆杆车尾侧升高，定义为摆杆正角度。
 * 2. 齿轮接触线速度近似相等，电机半径 2 cm、摆杆半径 20 cm，
 *    因此摆杆角速度为电机角速度的 0.1 倍，即 1 RPM = 0.6 deg/s。
 * 3. 摆杆机械范围为 +/-15 deg，软件禁止继续向外的阈值为 +/-5 deg。
 * 4. 复位键触发电机内部回零；用户确认杆已停止后按启动，不使用固定等待时间。
 * 5. 闭环运行期间 Y 轴命令只能由本模块产生，其他模块不得修改 Y 轴速度。
 *
 * 摆杆角度按以下公式对本模块最后发送的目标转速积分估算：
 * rod_angle_deg += commanded_motor_rpm * 0.6f * dt_s
 * 该估计只有在回零完成且没有其他模块发送 Y 轴命令时才有效。
 */

typedef struct
{
  bool initialized;                       /* 是否已有第一帧可用位置。 */
  bool valid;                             /* 初始化后保持有效，直到 Reset 清空观测器。 */
  uint32_t last_camera_timestamp_ms;      /* 上次融合帧的摄像头时间戳。 */
  float last_measurement_position_cm;     /* 跳变检查使用的上次原始位置。 */
  float position_cm;                      /* alpha-beta 融合位置。 */
  float velocity_cm_s;                    /* alpha-beta 融合速度。 */
  float acceleration_cm_s2;               /* 速度差分并低通后的加速度。 */
} Ball_Observer_t;

/* 模块唯一实例；所有字段只允许由本文件内函数修改。 */
typedef struct
{
  Ball_Control_State_t state;             /* 对外可见的控制状态。 */
  Ball_Vision_Sample_t latest_vision;     /* Push 接口复制得到的最新帧。 */
  Ball_Observer_t observer;               /* 小球位置、速度和加速度估计。 */
  float target_position_cm;               /* 位置外环目标，单位 cm。 */
  float rod_angle_estimate_deg;           /* 由目标 RPM 积分得到的摆杆角度。 */
  int32_t commanded_motor_rpm;            /* 本模块最后发给 Y 电机的目标转速。 */
  uint32_t last_actuator_update_ms;        /* 上次角度积分使用的 HAL tick。 */
  uint32_t last_telemetry_ms;              /* 遥测限频时间基准。 */
  bool start_requested;                    /* 上层已经请求启动。 */
  bool vision_sample_pending;              /* Task 尚未消费最新视觉帧。 */
  bool vision_frame_received;              /* 至少收到过一帧合法协议数据。 */
  bool rod_zero_valid;                     /* 水平零位是否可信。 */
  bool homing_in_progress;                 /* 复位已触发回零，正在等待用户按启动。 */
  bool zero_after_start_stop;               /* 启动停止命令发出后再建立软件零位。 */
  bool stop_command_pending;               /* 主循环中待发送一次停止命令。 */
} Ball_Control_Context_t;

static Ball_Control_Context_t ball_control;

/* 避免为简单绝对值引入额外数学库依赖。 */
static float Ball_Control_Abs_Float(float value)
{
  return (value < 0.0f) ? -value : value;
}

static void Ball_Control_Clear_State(uint32_t now_ms)
{
  /* 清空历史估计后，必须重新等待机械回零，不能沿用旧角度。 */
  memset(&ball_control, 0, sizeof(ball_control));
  ball_control.state = BALL_CONTROL_STOPPED;
  ball_control.last_actuator_update_ms = now_ms;
  ball_control.last_telemetry_ms = now_ms;
  ball_control.homing_in_progress = true;
}

static void Ball_Control_Initialize_Observer(const Ball_Vision_Sample_t *sample)
{
  /* 第一帧只能确定位置，速度和加速度从 0 开始等待后续帧收敛。 */
  ball_control.observer.initialized = true;
  ball_control.observer.valid = true;
  ball_control.observer.last_camera_timestamp_ms = sample->timestamp_ms;
  ball_control.observer.last_measurement_position_cm =
      (float)sample->position_centi_cm * 0.01f;
  ball_control.observer.position_cm =
      ball_control.observer.last_measurement_position_cm;
  ball_control.observer.velocity_cm_s = 0.0f;
  ball_control.observer.acceleration_cm_s2 = 0.0f;
}

static bool Ball_Control_Process_Vision(void)
{
  Ball_Vision_Sample_t sample;
  uint32_t frame_interval_ms;
  float frame_interval_s;
  float measurement_cm;
  float measured_speed_cm_s;
  float predicted_position_cm;
  float position_residual_cm;
  float previous_velocity_cm_s;
  float raw_acceleration_cm_s2;

  if (ball_control.vision_sample_pending)
  {
    sample = ball_control.latest_vision;
    ball_control.vision_sample_pending = false;

    if (sample.detected != 0U)
    {
      if (!ball_control.observer.initialized)
      {
        Ball_Control_Initialize_Observer(&sample);
        return true;
      }
      else
      {
        frame_interval_ms = sample.timestamp_ms -
                            ball_control.observer.last_camera_timestamp_ms;

        /* dt=0 无法计算速度；除此之外不按帧间隔重置，复位键负责重新初始化。 */
        if (frame_interval_ms != 0U)
        {
          frame_interval_s = (float)frame_interval_ms * 0.001f;
          measurement_cm = (float)sample.position_centi_cm * 0.01f;
          measured_speed_cm_s =
              Ball_Control_Abs_Float(measurement_cm -
                                     ball_control.observer.last_measurement_position_cm) /
              frame_interval_s;

          /* 单帧跳变超过钢球的合理速度时丢弃，不污染位置和速度估计。 */
          if (measured_speed_cm_s <= BALL_CONTROL_VISION_MAX_SPEED_CM_S)
          {
            /*
             * alpha-beta 更新顺序：先按上一帧速度预测本帧位置，再用位置残差
             * 同时修正位置和速度，增益固定，不读取协议中的可信度字段。
             */
            predicted_position_cm = ball_control.observer.position_cm +
                                    ball_control.observer.velocity_cm_s *
                                    frame_interval_s;
            position_residual_cm = measurement_cm - predicted_position_cm;
            previous_velocity_cm_s = ball_control.observer.velocity_cm_s;

            ball_control.observer.position_cm = predicted_position_cm +
                                                BALL_CONTROL_OBSERVER_ALPHA *
                                                position_residual_cm;
            ball_control.observer.velocity_cm_s = previous_velocity_cm_s +
                                                  BALL_CONTROL_OBSERVER_BETA *
                                                  position_residual_cm /
                                                  frame_interval_s;

            /* 加速度直接由速度差分后低通，不再额外限幅。 */
            raw_acceleration_cm_s2 =
                (ball_control.observer.velocity_cm_s - previous_velocity_cm_s) /
                frame_interval_s;
            ball_control.observer.acceleration_cm_s2 +=
                BALL_CONTROL_ACCELERATION_FILTER_GAIN *
                (raw_acceleration_cm_s2 -
                 ball_control.observer.acceleration_cm_s2);

            ball_control.observer.last_camera_timestamp_ms = sample.timestamp_ms;
            ball_control.observer.last_measurement_position_cm = measurement_cm;
            ball_control.observer.valid = true;
            return true;
          }
        }
      }
    }
  }

  return false;
}

static int32_t Ball_Control_Convert_Motor_Rpm(float motor_rpm)
{
  /* NaN 不能安全转换为整数；发生计算异常时输出 0 RPM。 */
  if (motor_rpm != motor_rpm)
  {
    return 0;
  }

  /*
   * ZDT 速度命令使用独立方向位和 uint16_t 幅值。这里只防止浮点转整数
   * 或协议字段溢出，不额外施加机械最大转速限制。
   */
  if (motor_rpm >= BALL_CONTROL_MOTOR_PROTOCOL_MAX_RPM)
  {
    return 65535;
  }
  if (motor_rpm <= -BALL_CONTROL_MOTOR_PROTOCOL_MAX_RPM)
  {
    return -65535;
  }

  /* C 语言强制转换会向 0 截断，先补偿 0.5 以得到最接近的整数 RPM。 */
  return (motor_rpm >= 0.0f) ? (int32_t)(motor_rpm + 0.5f)
                             : (int32_t)(motor_rpm - 0.5f);
}

static void Ball_Control_Send_Motor_Rpm(int32_t requested_motor_rpm)
{
  int32_t motor_rpm = requested_motor_rpm;
  uint16_t motor_rpm_magnitude;
  uint8_t motor_direction;

  /* 到达软件角度边界后只禁止继续向外，反向返回中心的命令仍然有效。 */
  if (((ball_control.rod_angle_estimate_deg >=
        BALL_CONTROL_ROD_ANGLE_LIMIT_DEG) &&
       (motor_rpm > 0)) ||
      ((ball_control.rod_angle_estimate_deg <=
        -BALL_CONTROL_ROD_ANGLE_LIMIT_DEG) &&
       (motor_rpm < 0)))
  {
    motor_rpm = 0;
  }

  /* 目标未改变时不重复占用阻塞式 UART4。 */
  if (motor_rpm == ball_control.commanded_motor_rpm)
  {
    return;
  }

  motor_direction = (motor_rpm >= 0) ? 0U : 1U;
  motor_rpm_magnitude = (uint16_t)((motor_rpm >= 0) ? motor_rpm : -motor_rpm);

  /* 加速度参数 0 表示直接更新速度；阻尼由 Ka 项产生，不使用驱动器斜坡。 */
  Emm_V5_Vel_Control(MOTOR_Y_UART,
                     MOTOR_Y_ADDR,
                     motor_direction,
                     motor_rpm_magnitude,
                     0U,
                     MOTOR_SYNC_FLAG);
  ball_control.commanded_motor_rpm = motor_rpm;
}

static void Ball_Control_Run_Velocity_Loop(void)
{
  float velocity_error_cm_s;
  float rod_angular_velocity_reference_deg_s;
  float motor_rpm;

  velocity_error_cm_s = BALL_CONTROL_VELOCITY_TARGET_CM_S -
                        ball_control.observer.velocity_cm_s;

  /*
   * 实测视觉正方向指向车头。电机正转抬高车尾侧，使钢球向车头正方向
   * 加速，因此 Kv 项与速度误差同号；Ka 项取反，用于抵消当前加速度。
   */
  rod_angular_velocity_reference_deg_s =
      BALL_CONTROL_VELOCITY_KV * velocity_error_cm_s -
      BALL_CONTROL_ACCELERATION_KA *
          ball_control.observer.acceleration_cm_s2;

  /* 已确认 1 motor RPM = 0.6 deg/s rod，故反算电机目标转速。 */
  motor_rpm = rod_angular_velocity_reference_deg_s /
              BALL_CONTROL_MOTOR_RPM_TO_ROD_DEG_S;
  Ball_Control_Send_Motor_Rpm(Ball_Control_Convert_Motor_Rpm(motor_rpm));
}

static void Ball_Control_Update_Actuator_Estimate(uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - ball_control.last_actuator_update_ms;

  ball_control.last_actuator_update_ms = now_ms;

  if (ball_control.homing_in_progress)
  {
    /* 复位后的回零运动由电机内部执行，用户按启动前不进行角度积分。 */
    return;
  }

  if (!ball_control.rod_zero_valid)
  {
    return;
  }

  /* 电机在主控调度间隔内持续保持目标转速，因此按实际 elapsed_ms 积分。 */
  ball_control.rod_angle_estimate_deg +=
      (float)ball_control.commanded_motor_rpm *
      BALL_CONTROL_MOTOR_RPM_TO_ROD_DEG_S *
      ((float)elapsed_ms * 0.001f);

  /* 到达边界后只禁止继续向外，反向命令仍可使摆杆返回中心。 */
  if (((ball_control.rod_angle_estimate_deg >=
        BALL_CONTROL_ROD_ANGLE_LIMIT_DEG) &&
       (ball_control.commanded_motor_rpm > 0)) ||
      ((ball_control.rod_angle_estimate_deg <=
        -BALL_CONTROL_ROD_ANGLE_LIMIT_DEG) &&
       (ball_control.commanded_motor_rpm < 0)))
  {
    ball_control.stop_command_pending = true;
  }
}

static void Ball_Control_Service_Stop_Command(void)
{
  if (ball_control.homing_in_progress || !ball_control.stop_command_pending)
  {
    return;
  }

  /* ZDT 发送接口为阻塞式，只能从本主循环任务调用。 */
  Emm_V5_Stop_Now(MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
  ball_control.commanded_motor_rpm = 0;
  ball_control.stop_command_pending = false;

  if (ball_control.zero_after_start_stop)
  {
    /* 用户确认回零完成并按启动；停止残余运动后建立角度积分零点。 */
    ball_control.rod_angle_estimate_deg = 0.0f;
    ball_control.rod_zero_valid = true;
    ball_control.zero_after_start_stop = false;
  }
}

static void Ball_Control_Reject_External_Y_Command(void)
{
  if (!ball_control.start_requested || ball_control.homing_in_progress ||
      ball_control.zero_after_start_stop ||
      (Get_Y_Step_Motor_Speed() == 0))
  {
    return;
  }

  /*
   * 旧菜单接口会同时发送 X/Y 目标；闭环期间发现其改写 Y 轴后，保留当前
   * X 轴目标并把 Y 轴缓存和命令恢复为 0。该路径只处理违规写入，不周期发送。
   */
  Motor_Vel_Synchronous_Control(Get_X_Step_Motor_Speed(), 0);
  ball_control.commanded_motor_rpm = 0;
  ball_control.stop_command_pending = false;
}

static void Ball_Control_Update_State(void)
{
  if (ball_control.state == BALL_CONTROL_FAULT)
  {
    return;
  }

  /* 启动请求、可信零位和有效视觉三者同时满足后才能进入 RUNNING。 */
  if (!ball_control.start_requested)
  {
    ball_control.state = BALL_CONTROL_STOPPED;
  }
  else if (!ball_control.rod_zero_valid || !ball_control.observer.valid)
  {
    ball_control.state = BALL_CONTROL_WAITING_FOR_VISION;
  }
  else
  {
    ball_control.state = BALL_CONTROL_RUNNING;
  }
}

static void Ball_Control_Report_Telemetry(uint32_t now_ms)
{
#if BALL_CONTROL_TELEMETRY_ENABLED
  if (!ball_control.vision_frame_received ||
      ((uint32_t)(now_ms - ball_control.last_telemetry_ms) <
       BALL_CONTROL_TELEMETRY_INTERVAL_MS))
  {
    return;
  }

  ball_control.last_telemetry_ms = now_ms;

  /*
   * 字段：state, vision_initialized, zero_valid, x_ref, x, v, a, angle,
   * motor_rpm。没有新帧时重复输出最后一次融合结果，不做主控时间外推。
   */
  Uart_Printf(&huart1,
              "BC,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%ld\r\n",
              (unsigned int)ball_control.state,
              ball_control.observer.valid ? 1U : 0U,
              ball_control.rod_zero_valid ? 1U : 0U,
              ball_control.target_position_cm,
              ball_control.observer.position_cm,
              ball_control.observer.velocity_cm_s,
              ball_control.observer.acceleration_cm_s2,
              ball_control.rod_angle_estimate_deg,
              (long)ball_control.commanded_motor_rpm);
#else
  (void)now_ms;
#endif
}

void Ball_Control_Init(void)
{
  Ball_Control_Clear_State(HAL_GetTick());
}

void Ball_Control_Task(void)
{
  uint32_t now_ms = HAL_GetTick();
  bool observer_updated;

  /*
   * 顺序不能随意调整：先累计上一周期执行器运动，再消费视觉并更新状态；
   * 之后处理外部违规命令和待发送停止命令，最后才允许新视觉帧触发中环。
   * 所有阻塞 UART 均留在主循环，不在 USART6 接收路径中驱动电机。
   */
  Ball_Control_Update_Actuator_Estimate(now_ms);
  observer_updated = Ball_Control_Process_Vision();
  Ball_Control_Update_State();
  Ball_Control_Reject_External_Y_Command();

  if ((ball_control.state != BALL_CONTROL_RUNNING) &&
      (ball_control.commanded_motor_rpm != 0))
  {
    ball_control.stop_command_pending = true;
  }

  Ball_Control_Service_Stop_Command();

  if ((ball_control.state == BALL_CONTROL_RUNNING) && observer_updated)
  {
    Ball_Control_Run_Velocity_Loop();
  }

  Ball_Control_Report_Telemetry(now_ms);
}

void Ball_Control_Start(void)
{
  uint32_t now_ms;

  if (ball_control.state == BALL_CONTROL_FAULT)
  {
    return;
  }

  /*
   * 复位后由用户确认杆子已经回到水平。启动时先排队停止残余回零运动，
   * 停止命令发出后立即建立软件零位，不再固定等待 2 s。
   */
  if (ball_control.homing_in_progress)
  {
    now_ms = HAL_GetTick();
    ball_control.homing_in_progress = false;
    ball_control.zero_after_start_stop = true;
    ball_control.rod_zero_valid = false;
    ball_control.commanded_motor_rpm = 0;
    ball_control.last_actuator_update_ms = now_ms;
  }

  /* 先清除可能由其他模块遗留的 Y 轴速度，再等待视觉和零位有效。 */
  ball_control.start_requested = true;
  ball_control.stop_command_pending = true;
  ball_control.state = BALL_CONTROL_WAITING_FOR_VISION;
}

void Ball_Control_Stop(void)
{
  /* 回零未完成时收到停止命令，当前机械位置不能再视为可信零位。 */
  if (ball_control.homing_in_progress || ball_control.zero_after_start_stop)
  {
    ball_control.homing_in_progress = false;
    ball_control.zero_after_start_stop = false;
    ball_control.rod_zero_valid = false;
  }

  ball_control.start_requested = false;
  ball_control.stop_command_pending = true;
  ball_control.state = BALL_CONTROL_STOPPED;
}

void Ball_Control_Reset(void)
{
  /* 调用方已触发机械回零；等待用户确认杆已停止后按启动建立软件零位。 */
  Ball_Control_Clear_State(HAL_GetTick());
}

bool Ball_Control_Set_Target(float target_cm)
{
  /* NaN 与自身不相等，利用该特性可避免为此额外调用浮点库函数。 */
  if (target_cm != target_cm ||
      target_cm < -BALL_CONTROL_POSITION_LIMIT_CM ||
      target_cm > BALL_CONTROL_POSITION_LIMIT_CM)
  {
    return false;
  }

  ball_control.target_position_cm = target_cm;
  return true;
}

void Ball_Control_Push_Vision(const Ball_Vision_Sample_t *sample)
{
  if (sample == NULL)
  {
    return;
  }

  /* Push 和 Task 当前都在协作式主循环中运行，不存在 ISR 并发写入。 */
  ball_control.latest_vision = *sample;
  ball_control.vision_sample_pending = true;
  ball_control.vision_frame_received = true;
}

Ball_Control_State_t Ball_Control_Get_State(void)
{
  return ball_control.state;
}
