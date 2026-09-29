#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"

#include <stdint.h>


/* SSD1306常见I2C地址：7位地址0x3C左移1位 */
#define OLED_ADDR              0x78

/* I2C超时时间 */
#define OLED_I2C_TIMEOUT       20

/* OLED尺寸 */
#define OLED_WIDTH             128
#define OLED_PAGE_NUM          8

/* 8x16字体 */
#define OLED_CHAR_WIDTH        8
#define OLED_CHAR_HEIGHT       16

/* 一行最多16个ASCII字符 */
#define OLED_MAX_CHAR_PER_LINE 16


/*
 * 写一组OLED命令
 *
 * buffer[0] = 0x00
 * 后面全部是命令
 */
static HAL_StatusTypeDef OLED_WriteCommands(
    const uint8_t *commands,
    uint16_t length)
{
    uint8_t buffer[32];
    uint16_t i;

    if (length == 0 || length > 31)
    {
        return HAL_ERROR;
    }

    buffer[0] = 0x00;

    for (i = 0; i < length; i++)
    {
        buffer[i + 1] = commands[i];
    }

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        buffer,
        length + 1,
        OLED_I2C_TIMEOUT
    );
}


/*
 * 写一组OLED显示数据
 *
 * buffer[0] = 0x40
 * 后面全部是显示数据
 */
static HAL_StatusTypeDef OLED_WriteData(
    const uint8_t *data,
    uint16_t length)
{
    uint8_t buffer[129];
    uint16_t i;

    if (length == 0 || length > 128)
    {
        return HAL_ERROR;
    }

    buffer[0] = 0x40;

    for (i = 0; i < length; i++)
    {
        buffer[i + 1] = data[i];
    }

    return HAL_I2C_Master_Transmit(
        &hi2c1,
        OLED_ADDR,
        buffer,
        length + 1,
        OLED_I2C_TIMEOUT
    );
}


/*
 * 设置OLED显存位置
 *
 * Page: 0~7
 * X:    0~127
 */
static void OLED_SetCursor(uint8_t Page,
                           uint8_t X)
{
    uint8_t commands[3];

    if (Page >= OLED_PAGE_NUM)
    {
        return;
    }

    if (X >= OLED_WIDTH)
    {
        return;
    }

    commands[0] = 0xB0 | Page;

    commands[1] =
        0x10 | ((X >> 4) & 0x0F);

    commands[2] =
        0x00 | (X & 0x0F);

    OLED_WriteCommands(commands, 3);
}


/*
 * 清屏
 */
void OLED_Clear(void)
{
    uint8_t page;
    uint8_t clear_buffer[128];
    uint16_t i;

    for (i = 0; i < 128; i++)
    {
        clear_buffer[i] = 0x00;
    }

    for (page = 0;
         page < OLED_PAGE_NUM;
         page++)
    {
        OLED_SetCursor(page, 0);

        OLED_WriteData(
            clear_buffer,
            128
        );
    }
}


/*
 * 显示一个8x16 ASCII字符
 *
 * Line:   1~4
 * Column: 1~16
 */
void OLED_ShowChar(uint8_t Line,
                   uint8_t Column,
                   char Char)
{
    uint8_t page;
    uint8_t x;

    if (Line < 1 || Line > 4)
    {
        return;
    }

    if (Column < 1 ||
        Column > OLED_MAX_CHAR_PER_LINE)
    {
        return;
    }

    if (Char < ' ' || Char > '~')
    {
        Char = ' ';
    }

    page = (Line - 1) * 2;
    x = (Column - 1) * OLED_CHAR_WIDTH;


    /* 字符上半部分 */
    OLED_SetCursor(page, x);

    OLED_WriteData(
        &OLED_F8x16[Char - ' '][0],
        8
    );


    /* 字符下半部分 */
    OLED_SetCursor(page + 1, x);

    OLED_WriteData(
        &OLED_F8x16[Char - ' '][8],
        8
    );
}


/*
 * 批量显示字符串
 *
 * 这是新版驱动最重要的优化：
 * 不再一个字符一个字符启动I2C。
 */
void OLED_ShowString(uint8_t Line,
                     uint8_t Column,
                     const char *String)
{
    uint8_t top_buffer[128];
    uint8_t bottom_buffer[128];

    uint8_t char_count = 0;
    uint8_t i;
    uint8_t j;

    uint8_t page;
    uint8_t x;

    char c;


    if (String == 0)
    {
        return;
    }

    if (Line < 1 || Line > 4)
    {
        return;
    }

    if (Column < 1 ||
        Column > OLED_MAX_CHAR_PER_LINE)
    {
        return;
    }


    /*
     * 最多显示到本行末尾
     */
    while (String[char_count] != '\0' &&
           char_count <
           (OLED_MAX_CHAR_PER_LINE
            - Column + 1))
    {
        char_count++;
    }

    if (char_count == 0)
    {
        return;
    }


    /*
     * 先把所有字符对应的字模
     * 拼进两个连续buffer
     */
    for (i = 0; i < char_count; i++)
    {
        c = String[i];

        if (c < ' ' || c > '~')
        {
            c = ' ';
        }

        for (j = 0; j < 8; j++)
        {
            top_buffer[i * 8 + j] =
                OLED_F8x16[c - ' '][j];

            bottom_buffer[i * 8 + j] =
                OLED_F8x16[c - ' '][j + 8];
        }
    }


    page = (Line - 1) * 2;

    x = (Column - 1)
        * OLED_CHAR_WIDTH;


    /*
     * 整个字符串上半部分
     * 一次I2C发送
     */
    OLED_SetCursor(page, x);

    OLED_WriteData(
        top_buffer,
        char_count * 8
    );


    /*
     * 整个字符串下半部分
     * 一次I2C发送
     */
    OLED_SetCursor(page + 1, x);

    OLED_WriteData(
        bottom_buffer,
        char_count * 8
    );
}


/*
 * 显示固定长度无符号十进制数
 *
 * 例如：
 * Number = 123
 * Length = 6
 *
 * 显示：
 * 000123
 */
void OLED_ShowNum(uint8_t Line,
                  uint8_t Column,
                  uint32_t Number,
                  uint8_t Length)
{
    char string[11];
    int8_t i;

    if (Length == 0)
    {
        return;
    }

    if (Length > 10)
    {
        Length = 10;
    }

    string[Length] = '\0';

    for (i = (int8_t)Length - 1;
         i >= 0;
         i--)
    {
        string[i] =
            (Number % 10) + '0';

        Number /= 10;
    }

    OLED_ShowString(
        Line,
        Column,
        string
    );
}


/*
 * 显示带符号十进制整数
 *
 * Length只代表数字位数
 *
 * 例如：
 * Number = -123
 * Length = 6
 *
 * 显示：
 * -000123
 */
void OLED_ShowSignedNum(
    uint8_t Line,
    uint8_t Column,
    int32_t Number,
    uint8_t Length)
{
    char string[12];
    uint32_t magnitude;
    int8_t i;

    if (Length == 0)
    {
        return;
    }

    if (Length > 10)
    {
        Length = 10;
    }


    if (Number >= 0)
    {
        string[0] = '+';

        magnitude =
            (uint32_t)Number;
    }
    else
    {
        string[0] = '-';

        /*
         * 先扩大到int64_t再取负，
         * 避免INT32_MIN直接取负溢出
         */
        magnitude =
            (uint32_t)(-(int64_t)Number);
    }


    string[Length + 1] = '\0';


    for (i = (int8_t)Length;
         i >= 1;
         i--)
    {
        string[i] =
            (magnitude % 10) + '0';

        magnitude /= 10;
    }


    OLED_ShowString(
        Line,
        Column,
        string
    );
}


/*
 * 显示十六进制数
 */
void OLED_ShowHexNum(
    uint8_t Line,
    uint8_t Column,
    uint32_t Number,
    uint8_t Length)
{
    char string[9];
    uint8_t value;
    int8_t i;

    if (Length == 0)
    {
        return;
    }

    if (Length > 8)
    {
        Length = 8;
    }


    string[Length] = '\0';


    for (i = (int8_t)Length - 1;
         i >= 0;
         i--)
    {
        value =
            Number & 0x0F;

        if (value < 10)
        {
            string[i] =
                value + '0';
        }
        else
        {
            string[i] =
                value - 10 + 'A';
        }

        Number >>= 4;
    }


    OLED_ShowString(
        Line,
        Column,
        string
    );
}


/*
 * 显示二进制数
 */
void OLED_ShowBinNum(
    uint8_t Line,
    uint8_t Column,
    uint32_t Number,
    uint8_t Length)
{
    char string[17];
    int8_t i;

    /*
     * OLED一行只有16个8x16字符，
     * 所以这里最多显示16位
     */
    if (Length == 0)
    {
        return;
    }

    if (Length > 16)
    {
        Length = 16;
    }


    string[Length] = '\0';


    for (i = (int8_t)Length - 1;
         i >= 0;
         i--)
    {
        string[i] =
            (Number & 0x01)
            ? '1'
            : '0';

        Number >>= 1;
    }


    OLED_ShowString(
        Line,
        Column,
        string
    );
}


/*
 * OLED初始化
 */
void OLED_Init(void)
{
    /*
     * SSD1306上电后需要等待稳定
     */
    HAL_Delay(100);


    /*
     * SSD1306初始化命令
     *
     * 一次批量发送，
     * 不再每条命令单独启动I2C
     */
    {
        const uint8_t init_commands[] =
        {
            0xAE,

            0xD5,
            0x80,

            0xA8,
            0x3F,

            0xD3,
            0x00,

            0x40,

            0xA1,

            0xC8,

            0xDA,
            0x12,

            0x81,
            0xCF,

            0xD9,
            0xF1,

            0xDB,
            0x30,

            0xA4,

            0xA6,

            0x8D,
            0x14,

            0xAF
        };

        OLED_WriteCommands(
            init_commands,
            sizeof(init_commands)
        );
    }


    OLED_Clear();
}
