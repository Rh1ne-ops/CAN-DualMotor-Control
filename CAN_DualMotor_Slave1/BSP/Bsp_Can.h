#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"


typedef struct
{
    uint32_t id;
    uint8_t len;
    uint8_t data[8];

} Can_Frame_t;


/* CAN接收软件队列长度 */
#define CAN_RX_QUEUE_SIZE  16


void Can_Init(void);

uint8_t Can_Send(uint32_t id,
                 uint8_t *data,
                 uint8_t len);

uint8_t Can_GetRxFrame(Can_Frame_t *frame);


/* 调试用 */
uint16_t Can_GetRxOverflowCount(void);


#endif
