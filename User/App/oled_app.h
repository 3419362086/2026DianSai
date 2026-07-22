#ifndef __OLED_APP_H__
#define __OLED_APP_H__

#include "MyDefine.h"

void Oled_Init(void);
void Oled_Task(void);

extern void* navigator;
extern unsigned char oled_clear_flag;
extern unsigned char statr_flag;

#endif
