#ifndef __VEHICLE_RUN_APP_H__
#define __VEHICLE_RUN_APP_H__

#include "MyDefine.h"

/*
 * 题二、题四车辆运行管理接口。
 *
 * 题二负责累计航向和灰度停车判定，题四负责 7 秒停车判定。两题分别保存
 * 状态和计时，并互斥使用电机及 PID；OLED 页面只查询状态和时间。
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
 * 累计净转角绝对值达到 250 度后，至少 3 路灰度为 1 时执行刹车。
 */
void Vehicle_Run_Task(void);

/* 返回 1 表示正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Get_State(void);

/*
 * 返回本轮运行时间，单位为 ms。运行时返回实时差值，暂停时返回锁存值；
 * 因此自动停车后时间保持不变，只有 Reset 才会清零。
 */
uint32_t Vehicle_Run_Get_Elapsed_Ms(void);

/* 题四启动：应用 75 rpm 基础目标值，并从 0 开始计时。 */
void Vehicle_Run_Question4_Start(void);

/* 题四复位：停车、清零计时，并重新设置 75 rpm 基础目标值。 */
void Vehicle_Run_Question4_Reset(void);

/* 返回 1 表示题四正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Question4_Get_State(void);

/* 返回题四本轮运行时间，单位为 ms；停车后返回锁存值。 */
uint32_t Vehicle_Run_Question4_Get_Elapsed_Ms(void);

#endif
