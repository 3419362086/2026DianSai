#include "uart_app.h"
#include "Easy_Menu_User.h"

typedef void (*Uart5_CommandHandler)(const char *line,
                                     const char *arguments,
                                     const void *context);

typedef struct
{
  const char *name;
  Uart5_CommandHandler handler;
  const void *context;
} Uart5_CommandEntry_t;

typedef struct
{
  const char *wheel_name;
  PidParams_t *params;
  PID_T *pid;
  float *ui_kp;
  float *ui_ki;
  float *ui_kd;
  signed int *ui_target;
} Uart5_PidCommandContext_t;

#define UART5_PWM_MEASURE_SETTLE_MS 2000U
#define UART5_PWM_MEASURE_SAMPLE_MS 2000U

#define UART6_FRAME_DATA_MAX_LENGTH 64U
/* 定点单位与协议小数位数一致，避免在通信路径中解析浮点文本。 */
#define UART6_POSITION_LIMIT_CENTI_CM 1250U
#define UART6_CONFIDENCE_MAX_MILLI 1000U
#define UART6_ERROR_REPORT_INTERVAL_MS 1000U
#define UART6_FORWARD_TO_WIRELESS 0U /* 协议联调逐帧输出已停用。 */

typedef enum
{
  UART5_PWM_MEASURE_IDLE = 0,
  UART5_PWM_MEASURE_SETTLING,
  UART5_PWM_MEASURE_SAMPLING
} Uart5_PwmMeasureState_t;

typedef struct
{
  Uart5_PwmMeasureState_t state;
  uint32_t phase_started_ms;
  int requested_left_pwm;
  int requested_right_pwm;
  int applied_left_pwm;
  int applied_right_pwm;
  int32_t left_start_count;
  int32_t right_start_count;
} Uart5_PwmMeasure_t;

typedef enum
{
  UART6_WAIT_FIRST_HASH = 0,
  UART6_WAIT_SECOND_HASH,
  UART6_READ_FRAME_DATA,
  UART6_WAIT_SECOND_DOLLAR,
  UART6_WAIT_RESTART_HASH
} Uart6_FrameState_t;

typedef enum
{
  UART6_FRAME_VALID = 0,
  UART6_FRAME_FORMAT_ERROR,
  UART6_FRAME_CHECKSUM_ERROR
} Uart6_FrameResult_t;

typedef struct
{
  int16_t position_centi_cm;
  uint32_t timestamp_ms;
  uint16_t confidence_milli;
  uint8_t detected;
} Uart6_VisionFrame_t;

typedef struct
{
  uint32_t framing_errors;
  uint32_t overflow_errors;
  uint32_t checksum_errors;
  uint32_t content_errors;
} Uart6_ParserStats_t;

static char uart5_command_line[BUFFER_SIZE];
static uint16_t uart5_command_line_length;
static uint8_t uart5_command_line_overflow;
static Uart5_PwmMeasure_t uart5_pwm_measure;

static char uart6_frame_data[UART6_FRAME_DATA_MAX_LENGTH + 1U];
static uint16_t uart6_frame_data_length;
static Uart6_FrameState_t uart6_frame_state;
static Uart6_ParserStats_t uart6_parser_stats;
static Uart6_ParserStats_t uart6_reported_stats;
static uint32_t uart6_last_error_report_ms;

static const Uart5_PidCommandContext_t uart5_pid_left_context =
{
  "left", &pid_params_left, &pid_speed_left,
  &Easy_Menu_Ui_Data.left_kp,
  &Easy_Menu_Ui_Data.left_ki,
  &Easy_Menu_Ui_Data.left_kd,
  &Easy_Menu_Ui_Data.left_target
};

static const Uart5_PidCommandContext_t uart5_pid_right_context =
{
  "right", &pid_params_right, &pid_speed_right,
  &Easy_Menu_Ui_Data.right_kp,
  &Easy_Menu_Ui_Data.right_ki,
  &Easy_Menu_Ui_Data.right_kd,
  &Easy_Menu_Ui_Data.right_target
};

static const Uart5_PidCommandContext_t uart5_pid_angle_context =
{
  "angle", &pid_params_angle, &pid_angle,
  NULL, NULL, NULL, NULL
};

static const Uart5_PidCommandContext_t uart5_pid_line_context =
{
  "line", &pid_params_line, &pid_line,
  NULL, NULL, NULL, NULL
};

static void Uart5_Read_Encoder_Total_Counts(int32_t *left_count,
                                            int32_t *right_count)
{
  uint32_t interrupt_state = __get_PRIMASK();

  /* Encoder_Task 在 10 ms 中断中依次更新两侧累计计数，快照必须来自同一周期。 */
  __disable_irq();
  *left_count = left_encoder.total_count;
  *right_count = right_encoder.total_count;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
}

static void Uart5_Pwm_Measure_Task(void)
{
  uint32_t now_ms;
  uint32_t elapsed_ms;
  int32_t left_end_count;
  int32_t right_end_count;
  int32_t left_delta_count;
  int32_t right_delta_count;
  float left_average_rpm;
  float right_average_rpm;

  if (uart5_pwm_measure.state == UART5_PWM_MEASURE_IDLE) return;

  if (pid_running != 0U)
  {
    uart5_pwm_measure.state = UART5_PWM_MEASURE_IDLE;
    Uart_Printf(wireless_UART, "PWM MEASURE CANCELED: PID enabled\r\n");
    return;
  }

  if ((left_motor.speed != uart5_pwm_measure.applied_left_pwm) ||
      (right_motor.speed != uart5_pwm_measure.applied_right_pwm))
  {
    uart5_pwm_measure.state = UART5_PWM_MEASURE_IDLE;
    Uart_Printf(wireless_UART, "PWM MEASURE CANCELED: motor output changed\r\n");
    return;
  }

  now_ms = HAL_GetTick();
  elapsed_ms = now_ms - uart5_pwm_measure.phase_started_ms;

  if (uart5_pwm_measure.state == UART5_PWM_MEASURE_SETTLING)
  {
    if (elapsed_ms < UART5_PWM_MEASURE_SETTLE_MS) return;

    Uart5_Read_Encoder_Total_Counts(&uart5_pwm_measure.left_start_count,
                                    &uart5_pwm_measure.right_start_count);
    uart5_pwm_measure.phase_started_ms = now_ms;
    uart5_pwm_measure.state = UART5_PWM_MEASURE_SAMPLING;
    return;
  }

  if (elapsed_ms < UART5_PWM_MEASURE_SAMPLE_MS) return;

  Uart5_Read_Encoder_Total_Counts(&left_end_count, &right_end_count);
  left_delta_count = left_end_count - uart5_pwm_measure.left_start_count;
  right_delta_count = right_end_count - uart5_pwm_measure.right_start_count;
  left_average_rpm = ((float)left_delta_count * 60000.0f) /
                     ((float)ENCODER_PPR * (float)elapsed_ms);
  right_average_rpm = ((float)right_delta_count * 60000.0f) /
                      ((float)ENCODER_PPR * (float)elapsed_ms);

  uart5_pwm_measure.state = UART5_PWM_MEASURE_IDLE;
  Motor_Stop(&left_motor);
  Motor_Stop(&right_motor);
  Uart_Printf(wireless_UART,
              "PWM MEASURE OK: requested_left=%d applied_left=%d avg_left_rpm=%.2f "
              "requested_right=%d applied_right=%d avg_right_rpm=%.2f sample_ms=%lu\r\n",
              uart5_pwm_measure.requested_left_pwm,
              uart5_pwm_measure.applied_left_pwm,
              left_average_rpm,
              uart5_pwm_measure.requested_right_pwm,
              uart5_pwm_measure.applied_right_pwm,
              right_average_rpm,
              (unsigned long)elapsed_ms);
}

static void Uart5_Handle_SetPid(const char *line,
                                const char *arguments,
                                const void *context)
{
  const Uart5_PidCommandContext_t *pid_context =
      (const Uart5_PidCommandContext_t *)context;
  float kp, ki, kd, out_min, out_max;
  float symmetric_limit;
  char *tail;
  uint32_t interrupt_state;
  int consumed = 0;
  int parsed_count;

  parsed_count = sscanf(arguments, "%f,%f,%f,%f,%f)%n",
                        &kp, &ki, &kd, &out_min, &out_max, &consumed);
  if (parsed_count != 5 || consumed <= 0)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPID ERROR: %s format, expected 5 numbers\r\n",
                line, pid_context->wheel_name);
    return;
  }

  tail = (char *)arguments + consumed;
  while (*tail == ' ' || *tail == '\t') tail++;
  if (*tail == ';')
  {
    tail++;
    while (*tail == ' ' || *tail == '\t') tail++;
  }
  if (*tail != '\0')
  {
    Uart_Printf(wireless_UART, "RX:%s\r\nPID ERROR: trailing characters\r\n", line);
    return;
  }

  if (out_min > out_max || out_max < 0.0f)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPID ERROR: require out_min<=out_max and out_max>=0\r\n",
                line);
    return;
  }

  symmetric_limit = out_min < 0.0f ? -out_min : out_min;
  if (out_max > symmetric_limit) symmetric_limit = out_max;

  /* PID_Task 在 10 ms 中断中运行，参数组必须作为一个整体更新。 */
  interrupt_state = __get_PRIMASK();
  __disable_irq();
  pid_context->params->kp = kp;
  pid_context->params->ki = ki;
  pid_context->params->kd = kd;
  pid_context->params->out_min = out_min;
  pid_context->params->out_max = out_max;
  pid_set_params(pid_context->pid, kp, ki, kd);
  pid_set_limit(pid_context->pid, symmetric_limit);
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  if (pid_context->ui_kp != NULL)
  {
    *pid_context->ui_kp = kp;
    *pid_context->ui_ki = ki;
    *pid_context->ui_kd = kd;
    Easy_Menu_Display_Refresh();
  }

  Uart_Printf(wireless_UART,
              "RX:%s\r\nPID OK: %s kp=%.2f ki=%.2f kd=%.2f out_min=%.2f out_max=%.2f\r\n",
              line, pid_context->wheel_name, kp, ki, kd, out_min, out_max);
}

static void Uart5_Handle_SetTargetRpm(const char *line,
                                      const char *arguments,
                                      const void *context)
{
  const Uart5_PidCommandContext_t *pid_context =
      (const Uart5_PidCommandContext_t *)context;
  char *tail;
  int target_rpm;
  int consumed = 0;
  int parsed_count;

  parsed_count = sscanf(arguments, "%d)%n", &target_rpm, &consumed);
  if (parsed_count != 1 || consumed <= 0)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nRPM ERROR: %s format, expected 1 integer\r\n",
                line, pid_context->wheel_name);
    return;
  }

  tail = (char *)arguments + consumed;
  while (*tail == ' ' || *tail == '\t') tail++;
  if (*tail == ';')
  {
    tail++;
    while (*tail == ' ' || *tail == '\t') tail++;
  }
  if (*tail != '\0')
  {
    Uart_Printf(wireless_UART, "RX:%s\r\nRPM ERROR: trailing characters\r\n", line);
    return;
  }

  pid_set_target(pid_context->pid, (float)target_rpm);
  if (pid_context->pid == &pid_speed_left)
  {
    target_speed_left = target_rpm;
  }
  else if (pid_context->pid == &pid_speed_right)
  {
    target_speed_right = target_rpm;
  }
  *pid_context->ui_target = target_rpm;
  Easy_Menu_Display_Refresh();

  Uart_Printf(wireless_UART,
              "RX:%s\r\nRPM OK: %s target=%d\r\n",
              line, pid_context->wheel_name, target_rpm);
}

static void Uart5_Handle_SetBasicRpm(const char *line,
                                     const char *arguments,
                                     const void *context)
{
  char *tail;
  uint32_t interrupt_state;
  int target_rpm;
  int consumed = 0;
  int parsed_count;

  (void)context;

  parsed_count = sscanf(arguments, "%d)%n", &target_rpm, &consumed);
  if (parsed_count != 1 || consumed <= 0)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nRPM ERROR: basic format, expected 1 integer\r\n",
                line);
    return;
  }

  tail = (char *)arguments + consumed;
  while (*tail == ' ' || *tail == '\t') tail++;
  if (*tail == ';')
  {
    tail++;
    while (*tail == ' ' || *tail == '\t') tail++;
  }
  if (*tail != '\0')
  {
    Uart_Printf(wireless_UART, "RX:%s\r\nRPM ERROR: trailing characters\r\n", line);
    return;
  }

  /* PID_Task 在中断中读取左右目标，三项基础速度状态必须整体更新。 */
  interrupt_state = __get_PRIMASK();
  __disable_irq();
  basic_speed = target_rpm;
  target_speed_left = target_rpm;
  target_speed_right = target_rpm;
  pid_set_target(&pid_speed_left, (float)target_rpm);
  pid_set_target(&pid_speed_right, (float)target_rpm);
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  Easy_Menu_Ui_Data.left_target = target_rpm;
  Easy_Menu_Ui_Data.right_target = target_rpm;
  Easy_Menu_Display_Refresh();

  Uart_Printf(wireless_UART,
              "RX:%s\r\nRPM OK: basic target=%d\r\n",
              line, target_rpm);
}

static void Uart5_Handle_SetMotorPwm(const char *line,
                                     const char *arguments,
                                     const void *context)
{
  char *tail;
  int left_pwm;
  int right_pwm;
  int left_pwm_limit;
  int right_pwm_limit;
  int consumed = 0;
  int parsed_count;

  (void)context;

  parsed_count = sscanf(arguments, "%d,%d)%n", &left_pwm, &right_pwm, &consumed);
  if (parsed_count != 2 || consumed <= 0)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPWM ERROR: format, expected 2 integers\r\n",
                line);
    return;
  }

  tail = (char *)arguments + consumed;
  while (*tail == ' ' || *tail == '\t') tail++;
  if (*tail == ';')
  {
    tail++;
    while (*tail == ' ' || *tail == '\t') tail++;
  }
  if (*tail != '\0')
  {
    Uart_Printf(wireless_UART, "RX:%s\r\nPWM ERROR: trailing characters\r\n", line);
    return;
  }

  left_pwm_limit = (int)left_motor.config.in1.htim->Init.Period;
  right_pwm_limit = (int)right_motor.config.in1.htim->Init.Period;
  if (left_pwm < -left_pwm_limit || left_pwm > left_pwm_limit ||
      right_pwm < -right_pwm_limit || right_pwm > right_pwm_limit)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPWM ERROR: left range=%d..%d, right range=%d..%d\r\n",
                line,
                -left_pwm_limit, left_pwm_limit,
                -right_pwm_limit, right_pwm_limit);
    return;
  }

  if (uart5_pwm_measure.state != UART5_PWM_MEASURE_IDLE)
  {
    uart5_pwm_measure.state = UART5_PWM_MEASURE_IDLE;
    Uart_Printf(wireless_UART, "PWM MEASURE CANCELED: manual PWM override\r\n");
  }

  /* 手动 PWM 测试必须先关闭闭环，防止 10 ms PID 任务覆盖电机输出。 */
  pid_running = 0U;
  Motor_Set_Speed(&left_motor, left_pwm);
  Motor_Set_Speed(&right_motor, right_pwm);

  Uart_Printf(wireless_UART,
              "RX:%s\r\nPWM OK: requested_left=%d requested_right=%d applied_left=%d applied_right=%d\r\n",
              line, left_pwm, right_pwm, left_motor.speed, right_motor.speed);
}

static void Uart5_Handle_MeasureMotorPwm(const char *line,
                                         const char *arguments,
                                         const void *context)
{
  char *tail;
  int left_pwm;
  int right_pwm;
  int left_pwm_limit;
  int right_pwm_limit;
  int consumed = 0;
  int parsed_count;

  (void)context;

  parsed_count = sscanf(arguments, "%d,%d)%n", &left_pwm, &right_pwm, &consumed);
  if (parsed_count != 2 || consumed <= 0)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPWM MEASURE ERROR: format, expected 2 integers\r\n",
                line);
    return;
  }

  tail = (char *)arguments + consumed;
  while (*tail == ' ' || *tail == '\t') tail++;
  if (*tail == ';')
  {
    tail++;
    while (*tail == ' ' || *tail == '\t') tail++;
  }
  if (*tail != '\0')
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPWM MEASURE ERROR: trailing characters\r\n",
                line);
    return;
  }

  left_pwm_limit = (int)left_motor.config.in1.htim->Init.Period;
  right_pwm_limit = (int)right_motor.config.in1.htim->Init.Period;
  if (left_pwm < -left_pwm_limit || left_pwm > left_pwm_limit ||
      right_pwm < -right_pwm_limit || right_pwm > right_pwm_limit)
  {
    Uart_Printf(wireless_UART,
                "RX:%s\r\nPWM MEASURE ERROR: left range=%d..%d, right range=%d..%d\r\n",
                line,
                -left_pwm_limit, left_pwm_limit,
                -right_pwm_limit, right_pwm_limit);
    return;
  }

  if (uart5_pwm_measure.state != UART5_PWM_MEASURE_IDLE)
  {
    Uart_Printf(wireless_UART, "PWM MEASURE ERROR: busy\r\n");
    return;
  }

  pid_running = 0U;
  Motor_Set_Speed(&left_motor, left_pwm);
  Motor_Set_Speed(&right_motor, right_pwm);

  uart5_pwm_measure.requested_left_pwm = left_pwm;
  uart5_pwm_measure.requested_right_pwm = right_pwm;
  uart5_pwm_measure.applied_left_pwm = left_motor.speed;
  uart5_pwm_measure.applied_right_pwm = right_motor.speed;
  uart5_pwm_measure.phase_started_ms = HAL_GetTick();
  uart5_pwm_measure.state = UART5_PWM_MEASURE_SETTLING;

  Uart_Printf(wireless_UART,
              "RX:%s\r\nPWM MEASURE START: requested_left=%d applied_left=%d "
              "requested_right=%d applied_right=%d settle_ms=%lu sample_ms=%lu\r\n",
              line,
              uart5_pwm_measure.requested_left_pwm,
              uart5_pwm_measure.applied_left_pwm,
              uart5_pwm_measure.requested_right_pwm,
              uart5_pwm_measure.applied_right_pwm,
              (unsigned long)UART5_PWM_MEASURE_SETTLE_MS,
              (unsigned long)UART5_PWM_MEASURE_SAMPLE_MS);
}

static const Uart5_CommandEntry_t uart5_command_table[] =
{
  {"set_pid_speed_left", Uart5_Handle_SetPid, &uart5_pid_left_context},
  {"set_pid_speed_right", Uart5_Handle_SetPid, &uart5_pid_right_context},
  {"set_pid_angle", Uart5_Handle_SetPid, &uart5_pid_angle_context},
  {"set_pid_line", Uart5_Handle_SetPid, &uart5_pid_line_context},
  {"set_target_rpm_left", Uart5_Handle_SetTargetRpm, &uart5_pid_left_context},
  {"set_target_rpm_right", Uart5_Handle_SetTargetRpm, &uart5_pid_right_context},
  {"set_basic_rpm", Uart5_Handle_SetBasicRpm, NULL},
  {"set_motor_pwm", Uart5_Handle_SetMotorPwm, NULL},
  {"measure_motor_pwm", Uart5_Handle_MeasureMotorPwm, NULL},
};

static void Uart5_Dispatch_Command(char *line)
{
  const char *command_end;
  size_t command_length;
  size_t i;

  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  command_end = strchr(line, '(');
  if (command_end == NULL)
  {
    Uart_Printf(wireless_UART, "RX:%s\r\nERROR: invalid command\r\n", line);
    return;
  }

  command_length = (size_t)(command_end - line);
  while (command_length > 0U &&
         (line[command_length - 1U] == ' ' || line[command_length - 1U] == '\t'))
  {
    command_length--;
  }

  for (i = 0U; i < sizeof(uart5_command_table) / sizeof(uart5_command_table[0]); i++)
  {
    if (strlen(uart5_command_table[i].name) == command_length &&
        memcmp(line, uart5_command_table[i].name, command_length) == 0)
    {
      uart5_command_table[i].handler(line, command_end + 1, uart5_command_table[i].context);
      return;
    }
  }

  Uart_Printf(wireless_UART, "RX:%s\r\nERROR: unknown command\r\n", line);
}

static void Uart5_Process_Line(void)
{
  if (uart5_command_line_overflow != 0U)
  {
    Uart_Printf(wireless_UART, "ERROR: command line too long\r\n");
  }
  else if (uart5_command_line_length > 0U)
  {
    uart5_command_line[uart5_command_line_length] = '\0';
    Uart5_Dispatch_Command(uart5_command_line);
  }

  uart5_command_line_length = 0U;
  uart5_command_line_overflow = 0U;
}

static uint8_t Uart6_Parse_Uint32(const char *text, uint32_t *value_out)
{
  uint32_t value = 0U;
  uint32_t digit;

  if (text[0] == '\0') return 0U;

  while (*text != '\0')
  {
    if (*text < '0' || *text > '9') return 0U;

    digit = (uint32_t)(*text - '0');
    if (value > (0xFFFFFFFFUL - digit) / 10U) return 0U;

    value = value * 10U + digit;
    text++;
  }

  *value_out = value;
  return 1U;
}

static uint8_t Uart6_Parse_Position(const char *text, int16_t *position_out)
{
  uint32_t integer_part = 0U;
  uint32_t scaled_position;
  uint8_t integer_digits = 0U;
  int32_t sign = 1;

  if (*text == '-' || *text == '+')
  {
    if (*text == '-') sign = -1;
    text++;
  }

  while (*text >= '0' && *text <= '9')
  {
    integer_part = integer_part * 10U + (uint32_t)(*text - '0');
    if (integer_part > UART6_POSITION_LIMIT_CENTI_CM / 100U) return 0U;
    integer_digits++;
    text++;
  }

  if (integer_digits == 0U || *text != '.') return 0U;
  text++;

  if (strlen(text) != 2U ||
      text[0] < '0' || text[0] > '9' ||
      text[1] < '0' || text[1] > '9' ||
      text[2] != '\0')
  {
    return 0U;
  }

  scaled_position = integer_part * 100U +
                    (uint32_t)(text[0] - '0') * 10U +
                    (uint32_t)(text[1] - '0');
  if (scaled_position > UART6_POSITION_LIMIT_CENTI_CM) return 0U;

  *position_out = (int16_t)(sign * (int32_t)scaled_position);
  return 1U;
}

static uint8_t Uart6_Parse_Confidence(const char *text,
                                      uint16_t *confidence_out)
{
  uint32_t scaled_confidence;

  if (strlen(text) != 5U ||
      (text[0] != '0' && text[0] != '1') || text[1] != '.' ||
      text[2] < '0' || text[2] > '9' ||
      text[3] < '0' || text[3] > '9' ||
      text[4] < '0' || text[4] > '9' || text[5] != '\0')
  {
    return 0U;
  }

  scaled_confidence = (uint32_t)(text[0] - '0') * 1000U +
                      (uint32_t)(text[2] - '0') * 100U +
                      (uint32_t)(text[3] - '0') * 10U +
                      (uint32_t)(text[4] - '0');
  if (scaled_confidence > UART6_CONFIDENCE_MAX_MILLI) return 0U;

  *confidence_out = (uint16_t)scaled_confidence;
  return 1U;
}

static uint8_t Uart6_Hex_Nibble(char value)
{
  if (value >= '0' && value <= '9') return (uint8_t)(value - '0');
  if (value >= 'A' && value <= 'F') return (uint8_t)(value - 'A' + 10);
  return 0xFFU;
}

static Uart6_FrameResult_t Uart6_Parse_Frame(char *frame_data,
                                             uint16_t frame_length,
                                             Uart6_VisionFrame_t *vision_frame)
{
  char *fields[5];
  uint16_t checksum_separator = 0xFFFFU;
  uint16_t i;
  uint8_t comma_count = 0U;
  uint8_t field_count = 1U;
  uint8_t checksum = 0U;
  uint8_t checksum_high;
  uint8_t checksum_low;
  uint8_t received_checksum;

  for (i = 0U; i < frame_length; i++)
  {
    if (frame_data[i] == ',')
    {
      comma_count++;
      checksum_separator = i;
    }
  }

  if (comma_count != 5U || checksum_separator == 0xFFFFU ||
      checksum_separator + 3U != frame_length)
  {
    return UART6_FRAME_FORMAT_ERROR;
  }

  checksum_high = Uart6_Hex_Nibble(frame_data[checksum_separator + 1U]);
  checksum_low = Uart6_Hex_Nibble(frame_data[checksum_separator + 2U]);
  if (checksum_high == 0xFFU || checksum_low == 0xFFU)
  {
    return UART6_FRAME_FORMAT_ERROR;
  }
  received_checksum = (uint8_t)((checksum_high << 4U) | checksum_low);

  for (i = 0U; i < checksum_separator; i++)
  {
    checksum ^= (uint8_t)frame_data[i];
  }
  if (checksum != received_checksum) return UART6_FRAME_CHECKSUM_ERROR;

  fields[0] = frame_data;
  frame_data[checksum_separator] = '\0';
  for (i = 0U; i < checksum_separator; i++)
  {
    if (frame_data[i] == ',')
    {
      frame_data[i] = '\0';
      if (field_count >= 5U) return UART6_FRAME_FORMAT_ERROR;
      fields[field_count++] = &frame_data[i + 1U];
    }
  }

  if (field_count != 5U || strcmp(fields[0], "BALL") != 0)
  {
    return UART6_FRAME_FORMAT_ERROR;
  }

  if (Uart6_Parse_Position(fields[1], &vision_frame->position_centi_cm) == 0U ||
      Uart6_Parse_Uint32(fields[2], &vision_frame->timestamp_ms) == 0U ||
      Uart6_Parse_Confidence(fields[4], &vision_frame->confidence_milli) == 0U)
  {
    return UART6_FRAME_FORMAT_ERROR;
  }

  if (fields[3][0] == '0' && fields[3][1] == '\0')
  {
    vision_frame->detected = 0U;
  }
  else if (fields[3][0] == '1' && fields[3][1] == '\0')
  {
    vision_frame->detected = 1U;
  }
  else
  {
    return UART6_FRAME_FORMAT_ERROR;
  }

  if (vision_frame->detected == 0U &&
      (vision_frame->position_centi_cm != 0 ||
       vision_frame->confidence_milli != 0U))
  {
    return UART6_FRAME_FORMAT_ERROR;
  }

  return UART6_FRAME_VALID;
}

static void Uart6_Reset_Frame_Parser(void)
{
  uart6_frame_state = UART6_WAIT_FIRST_HASH;
  uart6_frame_data_length = 0U;
}

static void Uart6_Start_Frame(void)
{
  uart6_frame_state = UART6_READ_FRAME_DATA;
  uart6_frame_data_length = 0U;
}

static void Uart6_Handle_Complete_Frame(void)
{
  Uart6_VisionFrame_t vision_frame;
  Uart6_FrameResult_t result;
#if UART6_FORWARD_TO_WIRELESS
  uint32_t absolute_position;
  const char *position_sign;
#endif

  uart6_frame_data[uart6_frame_data_length] = '\0';
  result = Uart6_Parse_Frame(uart6_frame_data,
                             uart6_frame_data_length,
                             &vision_frame);
  if (result == UART6_FRAME_CHECKSUM_ERROR)
  {
    uart6_parser_stats.checksum_errors++;
    return;
  }
  if (result != UART6_FRAME_VALID)
  {
    uart6_parser_stats.content_errors++;
    return;
  }

#if UART6_FORWARD_TO_WIRELESS
  if (vision_frame.position_centi_cm < 0)
  {
    position_sign = "-";
    absolute_position = (uint32_t)(-(int32_t)vision_frame.position_centi_cm);
  }
  else
  {
    position_sign = "";
    absolute_position = (uint32_t)vision_frame.position_centi_cm;
  }

  Uart_Printf(DEBUG_UART,
              "CV,%s%lu.%02lu,%lu,%u,%lu.%03lu\r\n",
              position_sign,
              (unsigned long)(absolute_position / 100U),
              (unsigned long)(absolute_position % 100U),
              (unsigned long)vision_frame.timestamp_ms,
              (unsigned int)vision_frame.detected,
              (unsigned long)(vision_frame.confidence_milli / 1000U),
              (unsigned long)(vision_frame.confidence_milli % 1000U));
#endif
}

static void Uart6_Process_Byte(uint8_t byte)
{
  switch (uart6_frame_state)
  {
    case UART6_WAIT_FIRST_HASH:
      if (byte == '#') uart6_frame_state = UART6_WAIT_SECOND_HASH;
      break;

    case UART6_WAIT_SECOND_HASH:
      if (byte == '#')
      {
        Uart6_Start_Frame();
      }
      else
      {
        uart6_frame_state = UART6_WAIT_FIRST_HASH;
      }
      break;

    case UART6_READ_FRAME_DATA:
      if (byte == '$')
      {
        uart6_frame_state = UART6_WAIT_SECOND_DOLLAR;
      }
      else if (byte == '#')
      {
        uart6_frame_state = UART6_WAIT_RESTART_HASH;
      }
      else if (byte < 0x20U || byte > 0x7EU)
      {
        uart6_parser_stats.framing_errors++;
        Uart6_Reset_Frame_Parser();
      }
      else if (uart6_frame_data_length >= UART6_FRAME_DATA_MAX_LENGTH)
      {
        uart6_parser_stats.overflow_errors++;
        Uart6_Reset_Frame_Parser();
      }
      else
      {
        uart6_frame_data[uart6_frame_data_length++] = (char)byte;
      }
      break;

    case UART6_WAIT_SECOND_DOLLAR:
      if (byte == '$')
      {
        Uart6_Handle_Complete_Frame();
        Uart6_Reset_Frame_Parser();
      }
      else
      {
        uart6_parser_stats.framing_errors++;
        uart6_frame_data_length = 0U;
        uart6_frame_state = (byte == '#') ?
                            UART6_WAIT_SECOND_HASH : UART6_WAIT_FIRST_HASH;
      }
      break;

    case UART6_WAIT_RESTART_HASH:
      uart6_parser_stats.framing_errors++;
      if (byte == '#')
      {
        Uart6_Start_Frame();
      }
      else
      {
        Uart6_Reset_Frame_Parser();
      }
      break;

    default:
      Uart6_Reset_Frame_Parser();
      break;
  }
}

static void Uart6_Report_Parser_Errors(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (uart6_parser_stats.framing_errors == uart6_reported_stats.framing_errors &&
      uart6_parser_stats.overflow_errors == uart6_reported_stats.overflow_errors &&
      uart6_parser_stats.checksum_errors == uart6_reported_stats.checksum_errors &&
      uart6_parser_stats.content_errors == uart6_reported_stats.content_errors)
  {
    return;
  }

  if ((uint32_t)(now_ms - uart6_last_error_report_ms) <
      UART6_ERROR_REPORT_INTERVAL_MS)
  {
    return;
  }

  Uart_Printf(wireless_UART,
              "CV_ERR,framing=%lu,overflow=%lu,checksum=%lu,content=%lu\r\n",
              (unsigned long)uart6_parser_stats.framing_errors,
              (unsigned long)uart6_parser_stats.overflow_errors,
              (unsigned long)uart6_parser_stats.checksum_errors,
              (unsigned long)uart6_parser_stats.content_errors);
  uart6_reported_stats = uart6_parser_stats;
  uart6_last_error_report_ms = now_ms;
}

static void Uart6_Protocol_Init(void)
{
  memset(uart6_frame_data, 0, sizeof(uart6_frame_data));
  memset(&uart6_parser_stats, 0, sizeof(uart6_parser_stats));
  memset(&uart6_reported_stats, 0, sizeof(uart6_reported_stats));
  uart6_last_error_report_ms = HAL_GetTick();
  Uart6_Reset_Frame_Parser();
}

void Uart_Init(void)
{
  Uart_Printf(DEBUG_UART, "Uart_Init ......\r\n");
  
  /* 串口 1 */
  rt_ringbuffer_init(&uart1_ring_buffer, uart1_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 2 */
  rt_ringbuffer_init(&uart2_ring_buffer, uart2_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 4 */
  rt_ringbuffer_init(&uart4_ring_buffer, uart4_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 5 */
  rt_ringbuffer_init(&uart5_ring_buffer, uart5_ring_buffer_input, BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
  /* 串口 6 */
  Uart6_Protocol_Init();
  rt_ringbuffer_init(&uart6_ring_buffer,
                     uart6_ring_buffer_input,
                     UART6_RING_BUFFER_SIZE);
  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_dma_buffer, BUFFER_SIZE); // 启动读取中断
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); // 关闭 DMA 的"半满中断"功能
  
}

/* 串口 1 */
void Uart1_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart1_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart1_ring_buffer, uart1_data_buffer, uart_data_len);
    uart1_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART1 Ringbuffer:%s\r\n", uart1_data_buffer);
    Uart_Printf(&huart1, "UART1 Ringbuffer:%s\r\n", uart1_data_buffer);
    
    memset(uart1_data_buffer, 0, uart_data_len);
  }
}

/* 串口 2 */
void Uart2_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart2_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart2_ring_buffer, uart2_data_buffer, uart_data_len);
    uart2_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART2 Ringbuffer:%s\r\n", uart2_data_buffer);
    Uart_Printf(&huart2, "UART2 Ringbuffer:%s\r\n", uart2_data_buffer);
    
    memset(uart2_data_buffer, 0, uart_data_len);
  }
}

/* 串口 4 */
void Uart4_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart4_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart4_ring_buffer, uart4_data_buffer, uart_data_len);
    uart4_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART4 Ringbuffer:%s\r\n", uart4_data_buffer);
    Uart_Printf(&huart4, "UART4 Ringbuffer:%s\r\n", uart4_data_buffer);
    
    memset(uart4_data_buffer, 0, uart_data_len);
  }
}

/* 串口 5 */
void Uart5_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart5_ring_buffer);
  uint16_t i;
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart5_ring_buffer, uart5_data_buffer, uart_data_len);
    for (i = 0U; i < uart_data_len; i++)
    {
      if (uart5_data_buffer[i] == '\r' || uart5_data_buffer[i] == '\n' ||
          uart5_data_buffer[i] == '\0')
      {
        Uart5_Process_Line();
      }
      else if (uart5_command_line_overflow == 0U)
      {
        if (uart5_command_line_length < BUFFER_SIZE - 1U)
        {
          uart5_command_line[uart5_command_line_length++] =
              (char)uart5_data_buffer[i];
        }
        else
        {
          uart5_command_line_overflow = 1U;
        }
      }
    }

    memset(uart5_data_buffer, 0, uart_data_len);
  }

  Uart5_Pwm_Measure_Task();

  if(pid_running != 0U)
  {
    /* 仅输出当前参与控制的速度环和循迹环，供上位机按 6 列 CSV 绘图。 */
    Uart_Printf(wireless_UART,
                "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                pid_speed_left.target,
                left_encoder.rpm,
                pid_speed_right.target,
                right_encoder.rpm,
                pid_line.target,
                g_line_position_error);
  }
}

/* 串口 6 */
void Uart6_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart6_ring_buffer);
  uint16_t bytes_read;
  uint16_t i;

  /* 10 ms内最多读256字节，高于115200 8N1的最大输入量，同时保证任务有界。 */
  if (uart_data_len > (uint16_t)sizeof(uart6_data_buffer))
  {
    uart_data_len = (uint16_t)sizeof(uart6_data_buffer);
  }

  bytes_read = (uint16_t)rt_ringbuffer_get(&uart6_ring_buffer,
                                            uart6_data_buffer,
                                            uart_data_len);
  for (i = 0U; i < bytes_read; i++)
  {
    Uart6_Process_Byte(uart6_data_buffer[i]);
  }

  Uart6_Report_Parser_Errors();
}

void System_State_Uart_Print(void)
{
//  Uart_Printf(DEBUG_UART, "\r\n========== System State ==========\r\n");
//  
//  Uart_Printf(DEBUG_UART, "[RTC] 20%02d-%02d-%02d  %02d:%02d:%02d\r\n", RTC_Day.Year, RTC_Day.Month, RTC_Day.Date, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
//  
//  Uart_Printf(DEBUG_UART, "[PID State] %d\r\n", pid_running);
//  
//  Uart_Printf(DEBUG_UART, "[LED] %d-%d-%d-%d\r\n", led_buf[0], led_buf[1], led_buf[2], led_buf[3]);
//  
//  Uart_Printf(DEBUG_UART, "[Grayscale] %d-%d-%d-%d-%d-%d-%d-%d\r\n",(gray_digtal>>0)&0x01,(gray_digtal>>1)&0x01,(gray_digtal>>2)&0x01,(gray_digtal>>3)&0x01,
//                                                                   (gray_digtal>>4)&0x01,(gray_digtal>>5)&0x01,(gray_digtal>>6)&0x01,(gray_digtal>>7)&0x01);

//	Uart_Printf(DEBUG_UART, "[Encoder] left:%.2fcm/s    right:%.2fcm/s\r\n", left_encoder.speed_cm_s, right_encoder.speed_cm_s);
//	
//  Uart_Printf(DEBUG_UART, "[Motor] left:%d    right:%d\r\n", left_motor.speed, right_motor.speed);

//	Uart_Printf(DEBUG_UART, "[Step_Motor] X:%d    Y:%d\r\n", Get_X_Step_Motor_Speed(), Get_Y_Step_Motor_Speed());

//  #if BNO08x_ON == 0
//  Uart_Printf(DEBUG_UART, "[ICM20608] %.2f, %.2f, %.2f\n", icm20608.Roll, icm20608.Pitch, icm20608.Yaw);
//  #else
//  Uart_Printf(DEBUG_UART, "[BNO08x] %.2f, %.2f, %.2f\n", bno08x.Roll, bno08x.Pitch, bno08x.Yaw);
//  #endif
//  Uart_Printf(DEBUG_UART, "[Temperature] %.2f \r\n", Humiture.Temp);

//  Uart_Printf(DEBUG_UART, "[Humidity] %.2f\r\n", Humiture.RH);
//  
//  Uart_Printf(DEBUG_UART, "[Now] %s\r\n", menu_get_current_page_name(navigator));
//  
//  Uart_Printf(DEBUG_UART, "[Restart Count] %d\r\n", start_count);
//  
//  Uart_Printf(DEBUG_UART, "==================================\r\n");
}
