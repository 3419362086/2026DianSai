#ifndef __KEY_DRIVER_H__
#define __KEY_DRIVER_H__

#include "main.h"
#include "ebtn.h"
#include "pstwo.h"

typedef enum
{
    USER_BUTTON_1 = 1,
    USER_BUTTON_2 = 2,
    USER_BUTTON_3 = 3,
    USER_BUTTON_4 = 4,
  
    USER_PS2_UP,
    USER_PS2_DOWN,
    USER_PS2_LEFT,
    USER_PS2_RIGHT,
  
    USER_BUTTON_MAX,

    USER_BUTTON_COMBO_MAX,
} user_button_t;

unsigned char Key_Read(void);
int Ebtn_Init(void);
void my_handle_key_event(struct ebtn_btn *btn, ebtn_evt_t evt);

void Blue_Key_Parse(unsigned char* buffer);
void Reset_Blue_Key(void);
unsigned char Get_Blue_Key(void);

#endif
