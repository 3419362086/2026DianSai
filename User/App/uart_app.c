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

static char uart5_command_line[BUFFER_SIZE];
static uint16_t uart5_command_line_length;
static uint8_t uart5_command_line_overflow;

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

static void Uart5_Handle_SetPid(const char *line,
                                const char *arguments,
                                const void *context)
{
  const Uart5_PidCommandContext_t *pid_context =
      (const Uart5_PidCommandContext_t *)context;
  float kp, ki, kd, out_min, out_max;
  float symmetric_limit;
  char *tail;
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

  pid_context->params->kp = kp;
  pid_context->params->ki = ki;
  pid_context->params->kd = kd;
  pid_context->params->out_min = out_min;
  pid_context->params->out_max = out_max;
  pid_set_params(pid_context->pid, kp, ki, kd);
  pid_set_limit(pid_context->pid, symmetric_limit);
  *pid_context->ui_kp = kp;
  *pid_context->ui_ki = ki;
  *pid_context->ui_kd = kd;
  Easy_Menu_Display_Refresh();

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
  *pid_context->ui_target = target_rpm;
  Easy_Menu_Display_Refresh();

  Uart_Printf(wireless_UART,
              "RX:%s\r\nRPM OK: %s target=%d\r\n",
              line, pid_context->wheel_name, target_rpm);
}

static const Uart5_CommandEntry_t uart5_command_table[] =
{
  {"set_pid_speed_left", Uart5_Handle_SetPid, &uart5_pid_left_context},
  {"set_pid_speed_right", Uart5_Handle_SetPid, &uart5_pid_right_context},
  {"set_target_rpm_left", Uart5_Handle_SetTargetRpm, &uart5_pid_left_context},
  {"set_target_rpm_right", Uart5_Handle_SetTargetRpm, &uart5_pid_right_context},
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
  rt_ringbuffer_init(&uart6_ring_buffer, uart6_ring_buffer_input, BUFFER_SIZE);
  
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

  if(pid_running != 0U)
  {
    Uart_Printf(wireless_UART,
                "%.2f,%.2f,%.2f,%.2f\r\n",
                pid_speed_left.target,
                left_encoder.rpm,
                pid_speed_right.target,
                right_encoder.rpm);
  }
}

/* 串口 6 */
void Uart6_Task(void)
{
  uint16_t uart_data_len = rt_ringbuffer_data_len(&uart6_ring_buffer);
  if(uart_data_len > 0)
  {
    rt_ringbuffer_get(&uart6_ring_buffer, uart6_data_buffer, uart_data_len);
    uart6_data_buffer[uart_data_len] = '\0';
    /* 数据解析 */
    Uart_Printf(DEBUG_UART, "UART6 Ringbuffer:%s\r\n", uart6_data_buffer);
    Uart_Printf(&huart6, "UART6 Ringbuffer:%s\r\n", uart6_data_buffer);
    
    memset(uart6_data_buffer, 0, uart_data_len);
  }
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
