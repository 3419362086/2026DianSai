#include "rtc_app.h"

RTC_TimeTypeDef RTC_Time;
RTC_DateTypeDef RTC_Day;

void RTC_Init(void)
{
  Uart_Printf(DEBUG_UART, "RTC_Init ......\r\n");
  
  HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN); 
  
  HAL_RTC_GetDate(&hrtc, &RTC_Day, RTC_FORMAT_BIN);
}

void RTC_Task(void)
{
  HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN); // ?????? tim ????? date
  
  HAL_RTC_GetDate(&hrtc, &RTC_Day, RTC_FORMAT_BIN);
}
