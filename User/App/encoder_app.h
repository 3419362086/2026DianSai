#ifndef __ENCODER_APP_H__
#define __ENCODER_APP_H__

#include "MyDefine.h"

void Encoder_Init(void);
void Encoder_Task(void);

extern Encoder left_encoder;
extern Encoder right_encoder;

#endif
