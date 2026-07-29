#ifndef __EASY_MENU_USER_H__
#define __EASY_MENU_USER_H__

#include "Easy_Menu_Core.h"

typedef struct
{
    unsigned char led1;
    unsigned char led2;
    unsigned char led3;
    unsigned char led4;
    float roll;
    float pitch;
    float yaw;
    signed int left_pwm;
    signed int right_pwm;
    float left_rpm;
    float right_rpm;
    signed int left_target;
    float left_kp;
    float left_ki;
    float left_kd;
    signed int right_target;
    float right_kp;
    float right_ki;
    float right_kd;
    signed int left_step_speed;
    signed int right_step_speed;
    float temp;
    float rh;
    unsigned char reset_count;
} Easy_Menu_Ui_Data_Layout_t;

extern Easy_Menu_Ui_Data_Layout_t Easy_Menu_Ui_Data;

void Easy_Menu_Info_Send(void);
void Ordinary_Page_7_3_Item_Callback(char *str);
#endif 
