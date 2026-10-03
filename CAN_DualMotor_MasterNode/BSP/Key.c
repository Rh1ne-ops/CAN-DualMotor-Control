#include "Key.h"


/* 消抖次数
 * Key_Update()每10ms调用一次
 * 连续2次状态相同才确认
 * 即约20ms消抖
 */
#define KEY_DEBOUNCE_COUNT    2


typedef struct
{
    uint8_t stable_state;      /* 消抖后的稳定状态 */
    uint8_t last_raw_state;    /* 上一次读取到的原始状态 */
    uint8_t debounce_count;    /* 连续相同次数 */
    uint8_t press_event;       /* 是否产生了新的按下事件 */

} Key_State_t;


static Key_State_t key_state[KEY_NUM];


/* --------------------------------------------------
 * 读取某个按键的原始状态
 *
 * 返回：
 * 1 = 按下
 * 0 = 松开
 * -------------------------------------------------- */
static uint8_t Key_ReadRaw(Key_t key)
{
    GPIO_PinState state;

    switch (key)
    {
        case KEY_UP:
            state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
            break;

        case KEY_DOWN:
            state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
            break;

        case KEY_OK:
            state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
            break;

        case KEY_BACK:
            state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
            break;

        case KEY_STOP:
            state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
            break;

        case KEY_PAGE:
            state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
            break;

        default:
            return 0;
    }

    /*
     * 当前按键采用上拉输入：
     *
     * 松开 = HIGH
     * 按下 = LOW
     */
    if (state == GPIO_PIN_RESET)
    {
        return 1;
    }

    return 0;
}


/* 初始化 */
void Key_Init(void)
{
    uint8_t i;

    for (i = 0; i < KEY_NUM; i++)
    {
        key_state[i].stable_state = 0;
        key_state[i].last_raw_state = 0;
        key_state[i].debounce_count = 0;
        key_state[i].press_event = 0;
    }
}


/* --------------------------------------------------
 * 按键扫描
 *
 * 推荐固定每10ms调用一次
 * -------------------------------------------------- */
void Key_Update(void)
{
    uint8_t i;
    uint8_t raw_state;

    for (i = 0; i < KEY_NUM; i++)
    {
        raw_state = Key_ReadRaw((Key_t)i);

        /*
         * 本次读取和上次原始读取相同
         */
        if (raw_state == key_state[i].last_raw_state)
        {
            if (key_state[i].debounce_count < KEY_DEBOUNCE_COUNT)
            {
                key_state[i].debounce_count++;
            }
        }
        else
        {
            /*
             * 状态发生变化
             * 重新开始消抖计数
             */
            key_state[i].last_raw_state = raw_state;
            key_state[i].debounce_count = 0;
        }


        /*
         * 连续稳定达到规定次数
         */
        if (key_state[i].debounce_count >= KEY_DEBOUNCE_COUNT)
        {
            /*
             * 只有稳定状态真的发生变化才处理
             */
            if (key_state[i].stable_state != raw_state)
            {
                key_state[i].stable_state = raw_state;

                /*
                 * 从松开 → 按下
                 * 产生一次按下事件
                 */
                if (raw_state == 1)
                {
                    key_state[i].press_event = 1;
                }
            }
        }
    }
}


/* 当前是不是处于按下状态 */
uint8_t Key_IsDown(Key_t key)
{
    if (key >= KEY_NUM)
    {
        return 0;
    }

    return key_state[key].stable_state;
}


/* --------------------------------------------------
 * 获取一次按下事件
 *
 * 读取以后自动清除
 * -------------------------------------------------- */
uint8_t Key_WasPressed(Key_t key)
{
    if (key >= KEY_NUM)
    {
        return 0;
    }

    if (key_state[key].press_event)
    {
        key_state[key].press_event = 0;

        return 1;
    }

    return 0;
}
