#include "zdt_motor_app.h"

Emm_V5_Response_t x_motor;
Emm_V5_Response_t y_motor;

void ZDT_Motor_Init(void)
{
    /* 本次上电使用闭环模式，不写入驱动器存储器。 */
    Emm_V5_Modify_Ctrl_Mode(MOTOR_X_UART, MOTOR_X_ADDR, true, 2);
    Emm_V5_Modify_Ctrl_Mode(MOTOR_Y_UART, MOTOR_Y_ADDR, true, 2);

    /* 一次性零点标定：完成标定后请注释以下两行。 */
    // Emm_V5_Origin_Set_O(MOTOR_X_UART, MOTOR_X_ADDR, true);
    // Emm_V5_Origin_Set_O(MOTOR_Y_UART, MOTOR_Y_ADDR, true);

    /* 一次性保存单圈就近回零参数并开启上电自动回零，配置完成后请注释以下调用。 */
    // Emm_V5_Origin_Modify_Params(MOTOR_X_UART, MOTOR_X_ADDR, true,
    //                             0, 0, 50, 10000, 0, 0, 0, true);
    // Emm_V5_Origin_Modify_Params(MOTOR_Y_UART, MOTOR_Y_ADDR, true,
    //                             0, 0, 50, 10000, 0, 0, 0, true);

    /* 使能X轴电机 */
    Emm_V5_En_Control(MOTOR_X_UART, MOTOR_X_ADDR, true, MOTOR_SYNC_FLAG);

    /* X轴使能后主动触发单圈就近回零。 */
    Emm_V5_Origin_Trigger_Return(MOTOR_X_UART, MOTOR_X_ADDR, 0, false);

    /* 使能Y轴电机 */
    Emm_V5_En_Control(MOTOR_Y_UART, MOTOR_Y_ADDR, true, MOTOR_SYNC_FLAG);

    /* Y轴使能后主动触发单圈就近回零。 */
    Emm_V5_Origin_Trigger_Return(MOTOR_Y_UART, MOTOR_Y_ADDR, 0, false);

    /* 不在初始化时停止X轴，避免中断上电自动回零。 */
//    Emm_V5_Stop_Now(MOTOR_X_UART, MOTOR_X_ADDR, MOTOR_SYNC_FLAG);

    /* 不在初始化时停止Y轴，避免中断上电自动回零。 */
//    Emm_V5_Stop_Now(MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
}


//Y轴
void ZDT_Motor_Task(void)
{
    Motor_Vel_Synchronous_Control(0, 50);
}
