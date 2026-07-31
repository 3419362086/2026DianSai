#include "zdt_motor_app.h"

Emm_V5_Response_t x_motor;
Emm_V5_Response_t y_motor;

void ZDT_Motor_Init(void)
{
    /* X42S V1.0 的闭环模式由驱动器面板配置。 */
    Emm_V5_En_Control(MOTOR_X_UART, MOTOR_X_ADDR, true, MOTOR_SYNC_FLAG);
    Emm_V5_En_Control(MOTOR_Y_UART, MOTOR_Y_ADDR, true, MOTOR_SYNC_FLAG);
    // Emm_V5_Origin_Set_O(MOTOR_X_UART, MOTOR_X_ADDR, true);
    // Emm_V5_Origin_Set_O(MOTOR_Y_UART, MOTOR_Y_ADDR, true);
    // Emm_V5_Origin_Modify_Params(MOTOR_X_UART, MOTOR_X_ADDR, true, 0, 0, 10, 10000, 0, 0, 0, true);
    // Emm_V5_Origin_Modify_Params(MOTOR_Y_UART, MOTOR_Y_ADDR, true, 0, 0, 10, 10000, 0, 0, 0, true);
    Emm_V5_Origin_Trigger_Return(MOTOR_X_UART, MOTOR_X_ADDR, 0, MOTOR_SYNC_FLAG);
    Emm_V5_Origin_Trigger_Return(MOTOR_Y_UART, MOTOR_Y_ADDR, 0, MOTOR_SYNC_FLAG);
    // Emm_V5_Stop_Now(MOTOR_X_UART, MOTOR_X_ADDR, MOTOR_SYNC_FLAG);
    // Emm_V5_Stop_Now(MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
}


//Y轴
void ZDT_Motor_Task(void)
{
    // Motor_Vel_Synchronous_Control(0, 50);
}
