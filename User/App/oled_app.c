#include "oled_app.h"

/*
 * 主菜单标题“题二”“题四”“题五”“题六”使用 GBK 编码。原
 * CHINESE_16x16 字库没有“题”“二”“四”“五”“六”，因此菜单查找不到
 * 字模时会保留屏幕旧内容，
 * 视觉上表现为标题乱码或显示成上一项内容。
 *
 * 下面两个字模与原字库保持相同格式：16x16 宋体、按 SSD1306 页布局逐列存储，
 * 前 16 字节对应上方 8 行，后 16 字节对应下方 8 行。字模位于 Flash，
 * 不占用运行时 RAM；这里只补菜单实际使用的五个字，避免扩大通用字库的修改范围。
 */
static const uint8_t oled_menu_question_glyphs[5][32] =
{
    /* “题”：GBK 0xCC 0xE2，Unicode U+9898。 */
    {
        0x80,0x80,0xBE,0xAA,0xAA,0xAA,0xBE,0x80,
        0x02,0xF2,0x1A,0xD6,0x12,0xF2,0x02,0x00,
        0x80,0x60,0x1C,0x20,0x7F,0x44,0x44,0x44,
        0x50,0x4B,0x44,0x43,0x44,0x4B,0x50,0x00
    },
    /* “二”：GBK 0xB6 0xFE，Unicode U+4E8C。 */
    {
        0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,
        0x08,0x08,0x08,0x08,0x08,0x00,0x00,0x00,
        0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
        0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00
    },
    /* “四”：GBK 0xCB 0xC4，Unicode U+56DB。 */
    {
        0x00,0xFC,0x04,0x04,0x04,0xFC,0x04,0x04,
        0x04,0xFC,0x04,0x04,0x04,0xFC,0x00,0x00,
        0x00,0x7F,0x28,0x24,0x23,0x20,0x20,0x20,
        0x20,0x21,0x22,0x22,0x22,0x7F,0x00,0x00
    },
    /* “五”：GBK 0xCE 0xE5，Unicode U+4E94。 */
    {
        0x00,0x02,0x42,0x42,0x42,0xC2,0x7E,0x42,
        0x42,0x42,0x42,0xC2,0x02,0x02,0x00,0x00,
        0x40,0x40,0x40,0x40,0x78,0x47,0x40,0x40,
        0x40,0x40,0x40,0x7F,0x40,0x40,0x40,0x00
    },
    /* “六”：GBK 0xC1 0xF9，Unicode U+516D。 */
    {
        0x20,0x20,0x20,0x20,0x20,0x20,0x21,0x22,
        0x2C,0x20,0x20,0x20,0x20,0x20,0x20,0x00,
        0x00,0x40,0x20,0x10,0x0C,0x03,0x00,0x00,
        0x00,0x01,0x02,0x04,0x18,0x60,0x00,0x00
    }
};

/**
 * @brief  将一个补充的 16x16 字模写入 OLED 指定位置。
 * @param  x             字模左边界的像素 X 坐标。
 * @param  page          字模上半部分所在的 SSD1306 页号。
 * @param  glyph         32 字节字模；前后各 16 字节对应上下两页。
 * @param  reverse_flag  0 为正常显示，非 0 为反色显示。
 *
 * @note   每次只写这个汉字占用的两页、每页 16 字节，不清屏，也不刷新其他区域。
 *         反色时使用栈上的 16 字节缓冲，避免修改 Flash 中的只读字模。
 */
static void Oled_Menu_Show_16x16_Glyph(uint8_t x,
                                      uint8_t page,
                                      const uint8_t glyph[32],
                                      uint8_t reverse_flag)
{
    /* 反色发送缓冲：一次只处理字模的一页，因此只需要 16 字节。 */
    uint8_t reversed_page_data[16];
    /* 当前处理的半页序号：0 为上半部分，1 为下半部分。 */
    uint8_t glyph_page_index;
    /* 当前半页内的列索引。 */
    uint8_t column_index;

    for(glyph_page_index = 0U; glyph_page_index < 2U; glyph_page_index++)
    {
        /* 当前半页在 32 字节字模中的起始地址。 */
        const uint8_t *page_data = &glyph[glyph_page_index * 16U];

        OLED_Set_Position(x, (uint8_t)(page + glyph_page_index));

        if(reverse_flag == 0U)
        {
            HAL_I2C_Mem_Write(&hi2c2,
                              0x78,
                              0x40,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)page_data,
                              16U,
                              100U);
        }
        else
        {
            for(column_index = 0U; column_index < 16U; column_index++)
            {
                reversed_page_data[column_index] = (uint8_t)(~page_data[column_index]);
            }

            HAL_I2C_Mem_Write(&hi2c2,
                              0x78,
                              0x40,
                              I2C_MEMADD_SIZE_8BIT,
                              reversed_page_data,
                              16U,
                              100U);
        }
    }
}

/**
 * @brief  Easy Menu 的中文行显示适配函数，补充题二、题四、题五、题六字模。
 * @param  x             字符左边界的像素 X 坐标。
 * @param  line          菜单行号；0、1 分别映射到 OLED 第 0、2 页。
 * @param  chinese_char  指向一个 GBK 双字节汉字编码。
 * @param  reverse_flag  菜单选中项的反色标志。
 *
 * @note   只截获“题”“二”“四”“五”“六”。其他汉字继续调用原显示函数。
 */
static void Oled_Menu_Display_Chinese_Char_Line(unsigned short int x,
                                                unsigned char line,
                                                char *chinese_char,
                                                unsigned char reverse_flag)
{
    /* 以无符号字节比较 GBK，避免 char 默认有符号时出现负值比较问题。 */
    const uint8_t *gbk_code = (const uint8_t *)chinese_char;
    /* 字模编号：0“题”、1“二”、2“四”、3“五”、4“六”；5 表示未匹配。 */
    uint8_t glyph_index = 5U;

    if((gbk_code[0] == 0xCCU) && (gbk_code[1] == 0xE2U))
    {
        glyph_index = 0U;
    }
    else if((gbk_code[0] == 0xB6U) && (gbk_code[1] == 0xFEU))
    {
        glyph_index = 1U;
    }
    else if((gbk_code[0] == 0xCBU) && (gbk_code[1] == 0xC4U))
    {
        glyph_index = 2U;
    }
    else if((gbk_code[0] == 0xCEU) && (gbk_code[1] == 0xE5U))
    {
        glyph_index = 3U;
    }
    else if((gbk_code[0] == 0xC1U) && (gbk_code[1] == 0xF9U))
    {
        glyph_index = 4U;
    }

    if((glyph_index < 5U) && (line < 2U))
    {
        Oled_Menu_Show_16x16_Glyph((uint8_t)x,
                                  (uint8_t)(line * 2U),
                                  oled_menu_question_glyphs[glyph_index],
                                  reverse_flag);
        return;
    }

    Menu_Display_Chinese_Char_Line(x, line, chinese_char, reverse_flag);
}

void Oled_Init(void)
{
    Uart_Printf(DEBUG_UART, "Oled_Init ......\r\n");
    OLED_Init();

    Easy_Menu_Init(NULL, Menu_Display_Char, NULL, Oled_Menu_Display_Chinese_Char_Line);
}

void Oled_Task(void)
{
    Easy_Menu_Display(HAL_GetTick());
}
