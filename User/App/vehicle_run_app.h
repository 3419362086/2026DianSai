#ifndef __VEHICLE_RUN_APP_H__
#define __VEHICLE_RUN_APP_H__

#include "MyDefine.h"

/*
 * 题二车辆运行管理接口。
 *
 * 本模块负责启动、复位、累计航向、灰度停车判定和计时锁存；OLED 页面
 * 只能通过查询接口读取状态和时间，不参与自动停车判定。
 */

/*
 * 启动一轮运行：清理控制器历史状态并从 0 开始计时。
 * 已在运行时重复调用不会重新计时；触发停车后可以直接再次启动。
 */
void Vehicle_Run_Start(void);

/* 停车并复位本轮状态和计时，使车辆可以通过下一次 Start 开始新一轮。 */
void Vehicle_Run_Reset(void);

/*
 * 灰度采集完成后的停车判定，在主循环中紧跟 Gray_Task 调用。
 * 累计净转角绝对值达到 300 度后，至少 3 路灰度为 1 时执行刹车。
 */
void Vehicle_Run_Task(void);

/* 返回 1 表示正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Get_State(void);

/*
 * 返回本轮运行时间，单位为 ms。运行时返回实时差值，暂停时返回锁存值；
 * 因此自动停车后时间保持不变，只有 Reset 才会清零。
 */
uint32_t Vehicle_Run_Get_Elapsed_Ms(void);

#endif
