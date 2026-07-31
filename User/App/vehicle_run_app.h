#ifndef __VEHICLE_RUN_APP_H__
#define __VEHICLE_RUN_APP_H__

#include "MyDefine.h"

/*
 * 题二至题六运行管理接口。
 *
 * 题二负责累计航向和灰度停车判定，题三只计时且保持车辆停止，题四负责
 * 7 秒停车判定，题五和题六分别负责 28 秒停车判定。五道题分别保存状态和
 * 累计运行时间，并互斥使用车辆控制资源；OLED 页面只发出命令并查询状态。
 * Pause 会停止车辆和 Y 轴但保留计时；Reset 还会清零计时并触发 Y 轴回零。
 * Y 轴命令使用阻塞 UART，相关接口只能从任务上下文调用，不能从中断调用。
 */

/*
 * 启动或继续运行：清理控制器历史状态并从已锁存时间继续计时。
 * 已在运行时重复调用不会重新计时；Reset 后首次启动从 0 开始。
 */
void Vehicle_Run_Start(void);

/* 暂停题二并锁存时间，不清零计时。 */
void Vehicle_Run_Pause(void);

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

/* 题三启动/继续：车辆保持停止，仅启动本题计时。 */
void Vehicle_Run_Question3_Start(void);

/* 题三暂停：锁存时间并停止 Y 轴，不触发回零。 */
void Vehicle_Run_Question3_Pause(void);

/* 题三复位：停车、清零计时并触发 Y 轴回零。 */
void Vehicle_Run_Question3_Reset(void);

/* 返回 1 表示题三正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Question3_Get_State(void);

/* 返回题三累计运行时间，单位为 ms。 */
uint32_t Vehicle_Run_Question3_Get_Elapsed_Ms(void);

/* 题四启动/继续：应用 85 rpm 基础目标值并从已锁存时间继续计时。 */
void Vehicle_Run_Question4_Start(void);

/* 题四暂停并锁存时间，不清零计时。 */
void Vehicle_Run_Question4_Pause(void);

/* 题四复位：停车、清零计时，并重新设置 85 rpm 基础目标值。 */
void Vehicle_Run_Question4_Reset(void);

/* 返回 1 表示题四正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Question4_Get_State(void);

/* 返回题四本轮运行时间，单位为 ms；停车后返回锁存值。 */
uint32_t Vehicle_Run_Question4_Get_Elapsed_Ms(void);

/* 题五启动/继续：应用 75 rpm 基础目标值并从已锁存时间继续计时。 */
void Vehicle_Run_Question5_Start(void);

/* 题五暂停并锁存时间，不清零计时。 */
void Vehicle_Run_Question5_Pause(void);

/* 题五复位：停车、清零计时，并重新设置 75 rpm 基础目标值。 */
void Vehicle_Run_Question5_Reset(void);

/* 返回 1 表示题五正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Question5_Get_State(void);

/* 返回题五本轮运行时间，单位为 ms；停车后返回锁存值。 */
uint32_t Vehicle_Run_Question5_Get_Elapsed_Ms(void);

/* 题六启动/继续：应用 75 rpm 基础目标值并从已锁存时间继续计时。 */
void Vehicle_Run_Question6_Start(void);

/* 题六暂停并锁存时间，不清零计时。 */
void Vehicle_Run_Question6_Pause(void);

/* 题六复位：停车、清零计时，并重新设置 75 rpm 基础目标值。 */
void Vehicle_Run_Question6_Reset(void);

/* 返回 1 表示题六正在运行，返回 0 表示暂停。 */
unsigned char Vehicle_Run_Question6_Get_State(void);

/* 返回题六本轮运行时间，单位为 ms；停车后返回锁存值。 */
uint32_t Vehicle_Run_Question6_Get_Elapsed_Ms(void);

#endif
