#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "main.h"

/* CAN ID */
#define CAN_ID_CMD_SLAVE1      0x101
#define CAN_ID_CMD_SLAVE2      0x102

#define CAN_ID_STATUS_SLAVE1   0x201
#define CAN_ID_STATUS_SLAVE2   0x202

#define CAN_ID_HEARTBEAT_S1    0x301
#define CAN_ID_HEARTBEAT_S2    0x302


/* 电机命令 */
typedef enum
{
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_RUN  = 1

} Motor_Command_t;
typedef enum
{
    CAN_MOTOR_STATE_IDLE = 0,
    CAN_MOTOR_STATE_RUNNING,
    CAN_MOTOR_STATE_DONE,
    CAN_MOTOR_STATE_FAULT
} Can_MotorState_t;

/* 主机发给从机的命令 */
typedef struct
{
    uint8_t command;
    int8_t direction;

    int32_t target_count;

    int16_t target_speed;

} Can_MotorCommand_t;


/* 从机发给主机的状态 */
typedef struct
{
    int32_t total_count;

    int16_t delta_count;

    uint8_t state;

} Can_MotorStatus_t;


/* 发送 */
uint8_t CanProtocol_SendMotorCommand(
    uint8_t slave_id,
    Can_MotorCommand_t *cmd);



/* 接收处理 */
void CanProtocol_Process(void);

uint8_t CanProtocol_GetSlave1Status(
    Can_MotorStatus_t *status);

uint8_t CanProtocol_GetSlave2Status(
    Can_MotorStatus_t *status);


uint8_t CanProtocol_GetHeartbeat(uint8_t slave_id);
#endif
