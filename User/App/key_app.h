#ifndef __KEY_APP_H__
#define __KEY_APP_H__

#include "MyDefine.h"

void Key_Init(void);
void Key_Task(void);
void my_handle_key_event(struct ebtn_btn *btn, ebtn_evt_t evt);

#endif
