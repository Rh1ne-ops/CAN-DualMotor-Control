#ifndef __APP_H
#define __APP_H

#include "main.h"
#include "can_protocol.h"


/* =========================
   系统运行模式
   ========================= */
typedef enum
{
    APP_MODE_S1_ONLY = 0,   /* 只运行 Slave1 */
    APP_MODE_S2_ONLY,       /* 只运行 Slave2 */
    APP_MODE_SYNC,          /* 两机同时启动；同步任务时使用相同目标 */
    APP_MODE_ASYNC,         /* 两机同时启动，但圈数/速度可分别设置 */
    APP_MODE_DELAY,         /* Slave1先启动，Slave2延时启动 */
    APP_MODE_COMBO,         /* 第一段运行 -> 等待 -> 反向第二段运行 */

    APP_MODE_COUNT
} App_Mode_t;


/* =========================
   组合动作阶段
   ========================= */
typedef enum
{
    APP_PHASE_NONE = 0,
    APP_PHASE_PRIMARY,      /* 第一段动作 */
    APP_PHASE_WAIT,         /* 中间等待 */
    APP_PHASE_SECONDARY     /* 第二段反向动作 */
} App_Phase_t;


/* =========================
   主机系统状态
   ========================= */
typedef enum
{
    APP_STATE_IDLE = 0,
    APP_STATE_RUNNING,
    APP_STATE_STOPPING,
    APP_STATE_FINISHED,
    APP_STATE_FAULT
} App_State_t;


/* =========================
   故障类型
   ========================= */
typedef enum
{
    APP_FAULT_NONE = 0,

    APP_FAULT_SLAVE1_OFFLINE,
    APP_FAULT_SLAVE2_OFFLINE,

    APP_FAULT_SLAVE1_NO_ACK,
    APP_FAULT_SLAVE2_NO_ACK,

    APP_FAULT_SLAVE1,
    APP_FAULT_SLAVE2,

    APP_FAULT_CAN_TX
} App_Fault_t;


/* =========================
   启动结果
   ========================= */
typedef enum
{
    APP_START_OK = 0,

    APP_START_ERR_BUSY,
    APP_START_ERR_PARAM,

    APP_START_ERR_SLAVE1_OFFLINE,
    APP_START_ERR_SLAVE2_OFFLINE
} App_StartResult_t;


/* =========================
   单个电机目标参数
   ========================= */
typedef struct
{
    int32_t target_count;       /* 相对目标编码器计数，保存绝对值 */
    int16_t target_speed;       /* 速度上限，单位仍为 count / 10ms */
    int8_t  direction;          /* +1 / -1 */
} App_MotorTarget_t;


/* =========================
   整个系统配置
   ========================= */
typedef struct
{
    App_Mode_t mode;

    App_MotorTarget_t slave1;
    App_MotorTarget_t slave2;

    /*
     * DELAY模式：Slave2相对Slave1的启动延时。
     * COMBO模式：第一段与第二段之间的等待时间。
     */
    uint16_t async_delay_ms;
} App_Config_t;


/* =========================
   APP运行信息

   OLED / 状态机只读取这里
   ========================= */
typedef struct
{
    App_State_t state;
    App_Fault_t fault;
    App_Mode_t  mode;
    App_Phase_t phase;

    uint8_t slave1_online;
    uint8_t slave2_online;

    uint8_t slave1_started;
    uint8_t slave2_started;

    uint8_t slave1_finished;
    uint8_t slave2_finished;

    Can_MotorStatus_t slave1_status;
    Can_MotorStatus_t slave2_status;

} App_Runtime_t;


/* =========================
   APP API
   ========================= */

void App_Init(void);
void App_Task(void);


/* 参数配置 */
uint8_t App_SetConfig(const App_Config_t *config);
void App_GetConfig(App_Config_t *config);


/* 系统控制 */
App_StartResult_t App_RequestStart(void);
void App_RequestStop(void);
void App_AcknowledgeFinished(void);
void App_ClearFault(void);


/* 状态读取 */
void App_GetRuntime(App_Runtime_t *runtime);
App_State_t App_GetState(void);
App_Fault_t App_GetFault(void);


#endif
