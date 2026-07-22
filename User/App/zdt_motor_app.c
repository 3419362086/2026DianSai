#include "zdt_motor_app.h"

Emm_V5_Response_t x_motor;
Emm_V5_Response_t y_motor;

void ZDT_Motor_Init(void)
{
    /* 使能X轴电机 */
    Emm_V5_En_Control(MOTOR_X_UART, MOTOR_X_ADDR, true, MOTOR_SYNC_FLAG);

    /* 使能Y轴电机 */
    Emm_V5_En_Control(MOTOR_Y_UART, MOTOR_Y_ADDR, true, MOTOR_SYNC_FLAG);

    /* 停止X轴电机 */
    Emm_V5_Stop_Now(MOTOR_X_UART, MOTOR_X_ADDR, MOTOR_SYNC_FLAG);

    /* 停止Y轴电机 */
    Emm_V5_Stop_Now(MOTOR_Y_UART, MOTOR_Y_ADDR, MOTOR_SYNC_FLAG);
}



void ZDT_Motor_Task(void)
{
  
}


