#ifndef __BALL_CONTROL_H__
#define __BALL_CONTROL_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * 小球控制模块的上层集成接口。
 *
 * 数据所有权：USART6 解析器构造视觉样本并通过 Push 接口提交；本模块复制
 * 样本后独占观测器、摆杆角度估计和控制状态，调用者不得直接修改内部状态。
 * 坐标约定：摆杆中心为 0 cm，车头侧为正，车尾侧为负。
 */
typedef struct
{
  uint32_t timestamp_ms;       /* MaixCAM 本帧采样时间戳，单位 ms。 */
  int16_t position_centi_cm;   /* 小球位置，定点单位为 0.01 cm。 */
  uint16_t confidence_milli;   /* 协议保留字段，取值 0~1000，不参与闭环计算。 */
  uint8_t detected;            /* 1 表示捕捉到小球，0 表示未捕捉到。 */
} Ball_Vision_Sample_t;

typedef enum
{
  BALL_CONTROL_STOPPED = 0,        /* 控制输出已停止。 */
  BALL_CONTROL_WAITING_FOR_VISION, /* 已请求启动，正在等待零位和可用视觉数据。 */
  BALL_CONTROL_RUNNING,            /* 小球闭环控制正在运行。 */
  BALL_CONTROL_FAULT               /* 检测到故障，控制输出已停止。 */
} Ball_Control_State_t;

/* 初始化模块私有状态；本函数不会驱动电机，也不会触发回零。 */
void Ball_Control_Init(void);

/* 10 ms 协作式任务；后续阻塞式电机命令只能由该主循环任务调度发送。 */
void Ball_Control_Task(void);

/* 请求启动闭环；收到满足条件的视觉数据前不允许产生控制输出。 */
void Ball_Control_Start(void);

/* 请求停止闭环；电机安全停止动作由主循环任务统一处理。 */
void Ball_Control_Stop(void);

/* 复位软件状态；调用方触发回零，用户确认杆停止后再调用 Start。 */
void Ball_Control_Reset(void);

/* 设置目标位置，单位 cm；超出摄像头量程时返回 false 且不更新目标。 */
bool Ball_Control_Set_Target(float target_cm);

/* 复制一帧已解析的视觉数据；模块不会保存调用者传入的指针。 */
void Ball_Control_Push_Vision(const Ball_Vision_Sample_t *sample);

/* 获取当前对外可见的控制状态。 */
Ball_Control_State_t Ball_Control_Get_State(void);

#endif
