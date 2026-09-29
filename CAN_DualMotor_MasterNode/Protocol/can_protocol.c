#include "can_protocol.h"
#include "Bsp_Can.h"

static Can_MotorStatus_t slave1_status;
static Can_MotorStatus_t slave2_status;

static uint8_t slave1_status_flag = 0;
static uint8_t slave2_status_flag = 0;

uint8_t CanProtocol_SendMotorCommand(
    uint8_t slave_id,
    Can_MotorCommand_t *cmd)
{
    uint8_t data[8];

    uint32_t can_id;


    if (slave_id == 1)
    {
        can_id = CAN_ID_CMD_SLAVE1;
    }
    else if (slave_id == 2)
    {
        can_id = CAN_ID_CMD_SLAVE2;
    }
    else
    {
        return 0;
    }


    data[0] = cmd->command;

    data[1] = (uint8_t)cmd->direction;


    data[2] = (uint8_t)(cmd->target_count >> 24);
    data[3] = (uint8_t)(cmd->target_count >> 16);
    data[4] = (uint8_t)(cmd->target_count >> 8);
    data[5] = (uint8_t)(cmd->target_count);


    data[6] = (uint8_t)(cmd->target_speed >> 8);
    data[7] = (uint8_t)(cmd->target_speed);


    return Can_Send(can_id, data, 8);
}
void CanProtocol_Process(void)
{
    Can_Frame_t frame;

    uint32_t temp_count;
    uint16_t temp_delta;

    if (!Can_GetRxFrame(&frame))
    {
        return;
    }

    switch (frame.id)
    {
        case CAN_ID_STATUS_SLAVE1:
        {
            if (frame.len != 8)
            {
                break;
            }

            temp_count =
                ((uint32_t)frame.data[0] << 24) |
                ((uint32_t)frame.data[1] << 16) |
                ((uint32_t)frame.data[2] << 8)  |
                ((uint32_t)frame.data[3]);

            slave1_status.total_count =
                (int32_t)temp_count;

            temp_delta =
                ((uint16_t)frame.data[4] << 8) |
                ((uint16_t)frame.data[5]);

            slave1_status.delta_count =
                (int16_t)temp_delta;

            slave1_status.state =
                frame.data[6];

            slave1_status_flag = 1;

            break;
        }

        case CAN_ID_STATUS_SLAVE2:
        {
            if (frame.len != 8)
            {
                break;
            }

            temp_count =
                ((uint32_t)frame.data[0] << 24) |
                ((uint32_t)frame.data[1] << 16) |
                ((uint32_t)frame.data[2] << 8)  |
                ((uint32_t)frame.data[3]);

            slave2_status.total_count =
                (int32_t)temp_count;

            temp_delta =
                ((uint16_t)frame.data[4] << 8) |
                ((uint16_t)frame.data[5]);

            slave2_status.delta_count =
                (int16_t)temp_delta;

            slave2_status.state =
                frame.data[6];

            slave2_status_flag = 1;

            break;
        }

        case CAN_ID_HEARTBEAT_S1:
        {
            /* 后面更新时间戳 */
            break;
        }

        case CAN_ID_HEARTBEAT_S2:
        {
            /* 后面更新时间戳 */
            break;
        }

        default:
            break;
    }
}
uint8_t CanProtocol_GetSlave1Status(
    Can_MotorStatus_t *status)
{
    if (slave1_status_flag == 0)
    {
        return 0;
    }

    *status = slave1_status;

    slave1_status_flag = 0;

    return 1;
}


uint8_t CanProtocol_GetSlave2Status(
    Can_MotorStatus_t *status)
{
    if (slave2_status_flag == 0)
    {
        return 0;
    }

    *status = slave2_status;

    slave2_status_flag = 0;

    return 1;
}
