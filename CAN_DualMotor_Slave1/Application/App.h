#ifndef __APP_H
#define __APP_H

#include "main.h"
#include "can_protocol.h"


/* =========================
   从机APP状态
   ========================= */

typedef enum
{
    APP_SLAVE_STATE_IDLE = 0,
    APP_SLAVE_STATE_RUNNING,
    APP_SLAVE_STATE_DONE,
    APP_SLAVE_STATE_FAULT
} App_SlaveState_t;


/* =========================
   从机故障类型
   ========================= */

typedef enum
{
    APP_SLAVE_FAULT_NONE = 0,

    /* Master运行过程中失联 */
    APP_SLAVE_FAULT_MASTER_TIMEOUT,

    /* 收到非法RUN参数 */
    APP_SLAVE_FAULT_INVALID_COMMAND

} App_SlaveFault_t;


/* =========================
   调试 / 状态查看
   ========================= */

typedef struct
{
    App_SlaveState_t state;

    App_SlaveFault_t fault;

    uint8_t master_online;

    int32_t total_count;

    int16_t delta_count;

    Can_MotorCommand_t current_command;

} App_SlaveRuntime_t;


/* =========================
   APP接口
   ========================= */

void App_Init(void);

void App_Task(void);


/* 调试 / OLED / 串口以后可以读取 */
void App_GetRuntime(App_SlaveRuntime_t *runtime);

App_SlaveState_t App_GetState(void);

App_SlaveFault_t App_GetFault(void);


#endif
