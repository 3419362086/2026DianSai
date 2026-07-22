#ifndef __MOTOR_APP_H__
#define __MOTOR_APP_H__

#include "MyDefine.h"

void Motor_Init(void);
void Motor_Task(void);

extern MOTOR left_motor;
extern MOTOR right_motor;

#endif
