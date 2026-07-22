#ifndef __FLASH_APP_H__
#define __FLASH_APP_H__

#include "MyDefine.h"

void Flash_Init(void);
void Flash_Task(void);
void Flash_SPI_Test(void);


extern unsigned char start_count;

#endif
