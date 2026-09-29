#ifndef __OLED_H
#define __OLED_H

#include "main.h"


/* OLED基本操作 */
void OLED_Init(void);
void OLED_Clear(void);


/* 字符与字符串 */
void OLED_ShowChar(uint8_t Line,
                   uint8_t Column,
                   char Char);

void OLED_ShowString(uint8_t Line,
                     uint8_t Column,
                     const char *String);


/* 数字显示 */
void OLED_ShowNum(uint8_t Line,
                  uint8_t Column,
                  uint32_t Number,
                  uint8_t Length);

void OLED_ShowSignedNum(uint8_t Line,
                        uint8_t Column,
                        int32_t Number,
                        uint8_t Length);

void OLED_ShowHexNum(uint8_t Line,
                     uint8_t Column,
                     uint32_t Number,
                     uint8_t Length);

void OLED_ShowBinNum(uint8_t Line,
                     uint8_t Column,
                     uint32_t Number,
                     uint8_t Length);


#endif
