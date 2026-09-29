#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "main.h"


/* 当前从机编号
 * Slave1填1
 * Slave2填2
 */
#define THIS_SLAVE_ID  1


/* CAN ID */
#define CAN_ID_CMD_SLAVE1       0x101
#define CAN_ID_CMD_SLAVE2       0x102

#define CAN_ID_STATUS_SLAVE1    0x201
#define CAN_ID_STATUS_SLAVE2    0x202

#define CAN_ID_HEARTBEAT_S1     0x301
#define CAN_ID_HEARTBEAT_S2     0x302


typedef enum
{
    MOTOR_CMD_STOP = 0,
    MOTOR_CMD_RUN  = 1

} Motor_Command_t;


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


/* 处理接收到的CAN帧 */
void CanProtocol_Process(void);


/* Application层读取新命令 */
uint8_t CanProtocol_GetCommand(
    Can_MotorCommand_t *cmd);


/* 向主机发送当前电机状态 */
uint8_t CanProtocol_SendMotorStatus(
    Can_MotorStatus_t *status);


/* 向主机发送心跳 */
uint8_t CanProtocol_SendHeartbeat(void);


#endif
