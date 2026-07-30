#ifndef __VEHICLE_RUN_APP_H__
#define __VEHICLE_RUN_APP_H__

#include "MyDefine.h"

/*
 * 题二车辆运行管理接口。
 *
 * 本模块负责启动、复位、一圈航向累计、自动停车和计时锁存；OLED 页面
 * 只能通过查询接口读取状态和时间，不参与自动停车判定。这样即使退出菜单
 * 页面，或 OLED 的 I2C 刷新暂时变慢，10 ms 实时任务仍会独立完成停车。
 */

/*
 * 启动一轮运行：清理控制器历史状态、记录发车航向并从 0 开始计时。
 * 已在运行时重复调用不会重新计时；完成一圈后必须先调用 Reset 才能再启动。
 */
void Vehicle_Run_Start(void);

/* 停车并复位本轮状态和计时，使车辆可以通过下一次 Start 开始新一轮。 */
void Vehicle_Run_Reset(void);

/*
 * 10 ms 实时任务，由 TIM2 周期回调在 Gyroscope_Task 之后、PID_Task 之前调用。
 * 函数内部不阻塞、不打印，也不操作 OLED。
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
