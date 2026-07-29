#ifndef __GRAY_APP_H
#define __GRAY_APP_H

#include "MyDefine.h"

#define GRAY_CHANNEL_COUNT 8U

void Gray_Init(void);
void Gray_Task(void);

extern uint8_t gray_digtal[GRAY_CHANNEL_COUNT]; // 灰度传感器开关量
extern volatile float g_line_position_error; // 循迹误差，丢线时保持上一次有效值

#endif
