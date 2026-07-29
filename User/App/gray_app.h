#ifndef __GRAY_APP_H
#define __GRAY_APP_H

#include "MyDefine.h"

#define GRAY_CHANNEL_COUNT 8U

void Gray_Init(void);
void Gray_Task(void);

extern uint8_t gray_digtal[GRAY_CHANNEL_COUNT]; // 灰度传感器开关量

#endif
