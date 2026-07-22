#ifndef __MYDEFINE_H__
#define __MYDEFINE_H__

/* ========== HAL 库头文件 ========== */
#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* ========== C 语言头文件 ========== */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* ========== 组件库头文件 ========== */
#include "ebtn.h"

#include "ringbuffer.h"

#include "oled.h"

#include "hardware_iic.h"

#include "icm20608.h"

#include "pid.h"

#include "Emm_V5.h"

#include "bno08x_hal.h"

#include "AHT20.h"

#include "gd25qxx.h"

#include "Easy_Menu.h"

#include "pstwo.h"

/* ========== 驱动库头文件 ========== */
#include "led_driver.h"
#include "key_driver.h"
#include "uart_driver.h"
#include "oled_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "gyroscope_driver.h"
#include "zdt_motor_driver.h"

/* ========== 应用层头文件 ========== */
#include "led_app.h"
#include "key_app.h"
#include "uart_app.h"
#include "oled_app.h"
#include "gray_app.h"
#include "motor_app.h"
#include "encoder_app.h"
#include "gyroscope_app.h"
#include "pid_app.h"
#include "zdt_motor_app.h"
#include "aht20_app.h"
#include "flash_app.h"
#include "rtc_app.h"

/* ========== 核心调度器头文件 ========== */
#include "Scheduler.h"
#include "Scheduler_Task.h"

#endif
