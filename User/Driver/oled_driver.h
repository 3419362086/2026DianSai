#ifndef __OLED_DRIVER_H__
#define __OLED_DRIVER_H__

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "oled.h"



void Menu_Display_Char(unsigned short int x, unsigned char line, char ch, unsigned char reverse_flag);

extern uint8_t bmp_pic[88][512];

// 声明DMA发送完成标志
extern volatile uint8_t oled_dma_tx_complete;

extern uint8_t BufFinshFlag; 
void Oled_Show_Chinese_Char2(uint8_t x, uint8_t y, uint8_t font_size, uint8_t target_char[2], uint8_t reverse_flag);
void OLED_Show_Pic(uint8_t BMP[], unsigned char reverse_flag);
void Menu_Display_Chinese_Char_Line(unsigned short int x, unsigned char line, char* ch, unsigned char reverse_flag);
void Safe_Abort_OLED_DMA(void);
int Oled_Printf(uint8_t x, uint8_t y, uint8_t fontsize, const char *format, ...);

#endif
