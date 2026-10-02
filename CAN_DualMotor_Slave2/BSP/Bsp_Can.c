#include "Bsp_Can.h"
#include "can.h"


/* =========================================================
   CAN RX 环形队列
   ========================================================= */

static Can_Frame_t rx_queue[CAN_RX_QUEUE_SIZE];

static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static volatile uint16_t rx_overflow_count = 0;


/* =========================================================
   CAN初始化
   ========================================================= */

void Can_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /*
     * ID = 0
     * MASK = 0
     *
     * 全部接收
     */
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;

    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;


    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_CAN_Start(&hcan) != HAL_OK)
    {
        Error_Handler();
    }


    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * 清软件队列
     */
    rx_head = 0;
    rx_tail = 0;
    rx_overflow_count = 0;
}


/* =========================================================
   发送CAN
   ========================================================= */

uint8_t Can_Send(uint32_t id,
                 uint8_t *data,
                 uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;


    if (len > 8)
    {
        return 0;
    }


    tx_header.StdId = id;
    tx_header.ExtId = 0;

    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;

    tx_header.DLC = len;

    tx_header.TransmitGlobalTime = DISABLE;


    if (HAL_CAN_AddTxMessage(&hcan,
                             &tx_header,
                             data,
                             &tx_mailbox) != HAL_OK)
    {
        return 0;
    }


    return 1;
}


/* =========================================================
   主循环从队列取一帧
   ========================================================= */

uint8_t Can_GetRxFrame(Can_Frame_t *frame)
{
    if (frame == 0)
    {
        return 0;
    }


    /*
     * head == tail
     * 说明队列为空
     */
    if (rx_head == rx_tail)
    {
        return 0;
    }


    /*
     * 取出最早的一帧
     */
    *frame = rx_queue[rx_tail];


    /*
     * tail向后移动
     */
    rx_tail++;

    if (rx_tail >= CAN_RX_QUEUE_SIZE)
    {
        rx_tail = 0;
    }


    return 1;
}


/* =========================================================
   CAN FIFO0接收中断
   ========================================================= */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rx_header;

    uint8_t data[8];

    uint8_t next_head;
    uint8_t i;


    if (hcan_ptr->Instance != CAN1)
    {
        return;
    }


    if (HAL_CAN_GetRxMessage(hcan_ptr,
                             CAN_RX_FIFO0,
                             &rx_header,
                             data) != HAL_OK)
    {
        return;
    }


    /*
     * 计算下一格
     */
    next_head = rx_head + 1;

    if (next_head >= CAN_RX_QUEUE_SIZE)
    {
        next_head = 0;
    }


    /*
     * next_head == tail
     *
     * 表示队列已经满了。
     *
     * 当前策略：
     * 丢弃新帧，不覆盖旧帧。
     */
    if (next_head == rx_tail)
    {
        rx_overflow_count++;

        return;
    }


    /*
     * 写入当前head位置
     */
    rx_queue[rx_head].id = rx_header.StdId;
    rx_queue[rx_head].len = rx_header.DLC;


    for (i = 0; i < rx_header.DLC; i++)
    {
        rx_queue[rx_head].data[i] = data[i];
    }


    /*
     * 数据全部写完后，
     * 最后再更新head。
     */
    rx_head = next_head;
}


/* =========================================================
   获取队列溢出次数
   ========================================================= */

uint16_t Can_GetRxOverflowCount(void)
{
    return rx_overflow_count;
}
