#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"

typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];

} Can_Frame_t;


void Can_Init(void);

uint8_t Can_Send(uint32_t id,
                 uint8_t *data,
                 uint8_t len);

uint8_t Can_GetRxFrame(Can_Frame_t *frame);

#endif
