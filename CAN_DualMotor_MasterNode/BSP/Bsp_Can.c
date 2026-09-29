#include "Bsp_Can.h"
#include "can.h"


static Can_Frame_t rx_frame;

static volatile  uint8_t rx_flag = 0;


/* CAN初始化 */
//void Can_Init(void)
//{
//    CAN_FilterTypeDef filter = {0};

//    /* 过滤器编号 */
//    filter.FilterBank = 0;

//    /* 标准ID掩码模式 */
//    filter.FilterMode = CAN_FILTERMODE_IDMASK;

//    /* 32位过滤器 */
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;

//    /*
//     * 暂时全部接收
//     * ID = 0
//     * Mask = 0
//     */
//    filter.FilterIdHigh = 0x0000;
//    filter.FilterIdLow = 0x0000;

//    filter.FilterMaskIdHigh = 0x0000;
//    filter.FilterMaskIdLow = 0x0000;

//    /* 收到的数据进入FIFO0 */
//    filter.FilterFIFOAssignment = CAN_RX_FIFO0;

//    /* 开启过滤器 */
//    filter.FilterActivation = ENABLE;

//    HAL_CAN_ConfigFilter(&hcan, &filter);

//    /* 启动CAN */
//    HAL_CAN_Start(&hcan);

//    /* 开启FIFO0接收中断 */
//    HAL_CAN_ActivateNotification(
//        &hcan,
//        CAN_IT_RX_FIFO0_MSG_PENDING
//    );
//}
void Can_Init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

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

    if (HAL_CAN_ActivateNotification(
            &hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}


/* 发送一帧CAN */
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

    /* 标准帧 */
    tx_header.IDE = CAN_ID_STD;

    /* 数据帧 */
    tx_header.RTR = CAN_RTR_DATA;

    /* 数据长度 */
    tx_header.DLC = len;

    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(
            &hcan,
            &tx_header,
            data,
            &tx_mailbox
        ) != HAL_OK)
    {
        return 0;
    }

    return 1;
}


/* 上层读取最新收到的一帧 */
uint8_t Can_GetRxFrame(Can_Frame_t *frame)
{
    if (rx_flag == 0)
    {
        return 0;
    }

    *frame = rx_frame;

    rx_flag = 0;

    return 1;
}


/* CAN FIFO0接收中断回调 */
void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan_ptr)
{
    CAN_RxHeaderTypeDef rx_header;

    uint8_t data[8];

    if (hcan_ptr->Instance != CAN1)
    {
        return;
    }

    if (HAL_CAN_GetRxMessage(
            hcan_ptr,
            CAN_RX_FIFO0,
            &rx_header,
            data
        ) != HAL_OK)
    {
        return;
    }

    rx_frame.id = rx_header.StdId;
    rx_frame.len = rx_header.DLC;

    for (uint8_t i = 0; i < rx_header.DLC; i++)
    {
        rx_frame.data[i] = data[i];
    }

    rx_flag = 1;
}
