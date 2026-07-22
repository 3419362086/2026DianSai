#include "oled_app.h"

void Oled_Init(void)
{
    Uart_Printf(DEBUG_UART, "Oled_Init ......\r\n");
    OLED_Init();

    Easy_Menu_Init(NULL, Menu_Display_Char, NULL, Menu_Display_Chinese_Char_Line);
}

void Oled_Task(void)
{
    Easy_Menu_Display(HAL_GetTick());
}
