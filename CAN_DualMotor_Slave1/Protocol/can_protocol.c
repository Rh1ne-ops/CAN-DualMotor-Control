#include "can_protocol.h"
#include "Bsp_Can.h"


static Can_MotorCommand_t received_cmd;
static uint8_t new_cmd_flag = 0;


void CanProtocol_Process(void)
{
    Can_Frame_t frame;

    uint32_t temp_count;
    uint16_t temp_speed;

    if (!Can_GetRxFrame(&frame))
    {
        return;
    }


#if THIS_SLAVE_ID == 1

    if (frame.id != CAN_ID_CMD_SLAVE1)
    {
        return;
    }

#elif THIS_SLAVE_ID == 2

    if (frame.id != CAN_ID_CMD_SLAVE2)
    {
        return;
    }

#else

#error "THIS_SLAVE_ID must be 1 or 2"

#endif


    if (frame.len != 8)
    {
        return;
    }


    received_cmd.command =
        frame.data[0];

    received_cmd.direction =
        (int8_t)frame.data[1];


    temp_count =
        ((uint32_t)frame.data[2] << 24) |
        ((uint32_t)frame.data[3] << 16) |
        ((uint32_t)frame.data[4] << 8)  |
        ((uint32_t)frame.data[5]);

    received_cmd.target_count =
        (int32_t)temp_count;


    temp_speed =
        ((uint16_t)frame.data[6] << 8) |
        ((uint16_t)frame.data[7]);

    received_cmd.target_speed =
        (int16_t)temp_speed;


    new_cmd_flag = 1;
}


uint8_t CanProtocol_GetCommand(
    Can_MotorCommand_t *cmd)
{
    if (new_cmd_flag == 0)
    {
        return 0;
    }

    *cmd = received_cmd;

    new_cmd_flag = 0;

    return 1;
}


uint8_t CanProtocol_SendMotorStatus(
    Can_MotorStatus_t *status)
{
    uint8_t data[8];

    uint32_t can_id;
    uint32_t count;
    uint16_t delta;


#if THIS_SLAVE_ID == 1

    can_id = CAN_ID_STATUS_SLAVE1;

#elif THIS_SLAVE_ID == 2

    can_id = CAN_ID_STATUS_SLAVE2;

#else

#error "THIS_SLAVE_ID must be 1 or 2"

#endif


    count = (uint32_t)status->total_count;

    data[0] = (uint8_t)(count >> 24);
    data[1] = (uint8_t)(count >> 16);
    data[2] = (uint8_t)(count >> 8);
    data[3] = (uint8_t)(count);


    delta = (uint16_t)status->delta_count;

    data[4] = (uint8_t)(delta >> 8);
    data[5] = (uint8_t)(delta);


    data[6] = status->state;

    data[7] = 0;


    return Can_Send(can_id, data, 8);
}


uint8_t CanProtocol_SendHeartbeat(void)
{
    uint8_t data[1];
    uint32_t can_id;


#if THIS_SLAVE_ID == 1

    can_id = CAN_ID_HEARTBEAT_S1;

#elif THIS_SLAVE_ID == 2

    can_id = CAN_ID_HEARTBEAT_S2;

#else

#error "THIS_SLAVE_ID must be 1 or 2"

#endif


    data[0] = 0x01;

    return Can_Send(can_id, data, 1);
}
