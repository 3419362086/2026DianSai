#ifndef __RTC_APP_H__
#define __RTC_APP_H__

#include "MyDefine.h"

void RTC_Init(void);
void RTC_Task(void);

extern RTC_DateTypeDef RTC_Day;
extern RTC_TimeTypeDef RTC_Time;


#endif
