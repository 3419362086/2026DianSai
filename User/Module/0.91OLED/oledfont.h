#ifndef OLEDFONT_H
#define OLEDFONT_H

typedef struct 
{
	unsigned char data[32];  // 字模（8 个字节为一行，16x16 一行用 2 个字节表示，所以是 2 * 16 = 32）
	unsigned char index[2];  // 字模索引
} Chinese_Font_16; // 16 * 16 中文字库结构体(GB2312)

extern const Chinese_Font_16 CHINESE_16x16[100];


extern const unsigned char oled_F6X8[][6];
extern const unsigned char oled_F8X16[];
extern const unsigned char oled_Hzk[][32];
extern const unsigned char oled_Hzb[][128];

#endif // OLEDFONT_H
