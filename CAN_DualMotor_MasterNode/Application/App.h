#ifndef __APP_H
#define __APP_H

#include "main.h"
#include "can_protocol.h"


/* =========================
   系统运行模式
   ========================= */
typedef enum
{
    APP_MODE_SYNC = 0,      // 两个从机同时启动
    APP_MODE_ASYNC          // Slave1先启动，Slave2延时启动
} App_Mode_t;


/* =========================
   主机系统状态
   ========================= */
typedef enum
{
    APP_STATE_IDLE = 0,     // 待机
    APP_STATE_RUNNING,      // 运行
    APP_STATE_STOPPING,     // 正在停止
    APP_STATE_FINISHED,     // 正常完成
    APP_STATE_FAULT         // 故障
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
    int32_t target_count;       // 相对目标编码器计数
    int16_t target_speed;       // 目标速度
    int8_t  direction;          // 1 / -1
} App_MotorTarget_t;


/* =========================
   整个系统配置
   ========================= */
typedef struct
{
    App_Mode_t mode;

    App_MotorTarget_t slave1;
    App_MotorTarget_t slave2;

    uint16_t async_delay_ms;    // 异步模式下 Slave2 延迟启动
} App_Config_t;


/* =========================
   APP运行信息

   OLED / 状态机以后只读这里
   ========================= */
typedef struct
{
    App_State_t state;
    App_Fault_t fault;
    App_Mode_t  mode;

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
