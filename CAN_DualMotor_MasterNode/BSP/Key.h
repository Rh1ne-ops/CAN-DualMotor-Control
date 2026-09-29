#ifndef __KEY_H
#define __KEY_H

#include "main.h"


typedef enum
{
    KEY_UP = 0,
    KEY_DOWN,
    KEY_OK,
    KEY_BACK,
    KEY_STOP,
    KEY_PAGE,

    KEY_NUM

} Key_t;


/* 初始化按键模块 */
void Key_Init(void);

/* 固定周期调用，推荐10ms */
void Key_Update(void);

/* 查询按键当前是否处于按下状态 */
uint8_t Key_IsDown(Key_t key);

/* 查询是否刚刚产生一次按下事件 */
uint8_t Key_WasPressed(Key_t key);


#endif
