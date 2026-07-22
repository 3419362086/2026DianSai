#include "key_app.h"

void Key_Init()
{
    Uart_Printf(DEBUG_UART, "Key_Init ......\r\n");
    Ebtn_Init();
}

void Key_Task()
{
    ebtn_process(HAL_GetTick());
}

extern MOTOR left_motor;
extern MOTOR right_motor;
#include "Easy_Menu_User.h"
/* 处理按键事件的回调函数 */
void my_handle_key_event(struct ebtn_btn *btn, ebtn_evt_t evt) {
    uint16_t key_id = btn->key_id;                 // 获取触发事件的按键 ID
    // uint16_t click_cnt = ebtn_click_get_count(btn); // 获取连击次数 (仅在 ONCLICK 事件时有意义)
    // uint16_t kalive_cnt = ebtn_keepalive_get_count(btn); // 获取长按计数 (仅在 KEEPALIVE 事件时有意义)

    // 根据事件类型进行处理
    switch (evt) {
        case EBTN_EVT_ONPRESS: // 按下事件 (消抖成功后触发一次)
            switch(key_id)
            {
              case 1:
                    Easy_Menu_Input(EASY_MENU_UP);
                    break;
                case 2:
                    Easy_Menu_Input(EASY_MENU_DOWN);
                    break;
                case 3:
                    Easy_Menu_Input(EASY_MENU_LEFT);
                    break;
                case 4:
                    Easy_Menu_Input(EASY_MENU_RIGHT);
                    break;
              case USER_PS2_UP:

                break;
              case USER_PS2_DOWN:

                break;
              case USER_PS2_LEFT:

                break;
              case USER_PS2_RIGHT:

                break;
            }
            break;
        case EBTN_EVT_ONRELEASE: // 释放事件 (消抖成功后触发一次)
            break;

        case EBTN_EVT_ONCLICK: // 单击/连击事件 (在释放后，或达到最大连击数，或超时后触发)

            break;
        case EBTN_EVT_KEEPALIVE: // 保持活动/长按事件 (按下持续时间超过阈值后，按周期触发)
            switch(key_id)
            {
              case 1:
                Easy_Menu_Input(EASY_MENU_UP);
                break;
            case 2:
                Easy_Menu_Input(EASY_MENU_DOWN);
                break;
            case 3:
                Easy_Menu_Input(EASY_MENU_LEFT);
                break;
            case 4:
                Easy_Menu_Input(EASY_MENU_RIGHT);
                break;
              case USER_PS2_UP:

                break;
              case USER_PS2_DOWN:

                break;
              case USER_PS2_LEFT:

                break;
              case USER_PS2_RIGHT:

                break;
            }
            break;

    }
}



