#ifndef __GRAY_DRIVER_H_
#define __GRAY_DRIVER_H_


#include "MyDefine.h"
#define u8 uint8_t

#define IR_ADDRESS 0x12 ַ

void deal_IRdata(u8 *x1,u8 *x2,u8 *x3,u8 *x4,u8 *x5,u8 *x6,u8 *x7,u8 *x8);
void set_adjust_mode(u8 mode);

#endif

