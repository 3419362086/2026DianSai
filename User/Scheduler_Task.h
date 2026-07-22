#ifndef __SCHEDULER_TASK_H__
#define __SCHEDULER_TASK_H__

#include "MyDefine.h"

void System_Init(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void Delay_Us(unsigned int us);

#endif
