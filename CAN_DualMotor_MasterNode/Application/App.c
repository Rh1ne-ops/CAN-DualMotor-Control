#include "App.h"
#include "can_protocol.h"


/* =========================================================
   时间参数
   ========================================================= */

/* RUN 指令周期发送
   不能只发送一次。
   周期发送同时可以作为 Slave 的命令看门狗来源。
*/
#define APP_COMMAND_PERIOD_MS          50U


/* STOP 指令重复发送周期 */
#define APP_STOP_PERIOD_MS             100U


/* 超过这个时间没有收到从机任何消息 -> 掉线 */
#define APP_SLAVE_TIMEOUT_MS           500U


/* 发送 RUN 后，这么久还没看到 RUNNING -> 启动失败 */
#define APP_RUN_ACK_TIMEOUT_MS         500U


/* 主机主动 STOP 后维持 STOPPING 一段时间 */
#define APP_STOP_HOLD_MS               300U


/* 连续 CAN 发送失败次数 */
#define APP_CAN_TX_FAIL_LIMIT          5U



/* =========================================================
   APP内部数据
   ========================================================= */

static App_Config_t app_config;
static App_Runtime_t app_runtime;


/* 最近一次收到从机消息的时间 */
static uint32_t slave1_last_rx_tick;
static uint32_t slave2_last_rx_tick;


/* 是否至少收到过一次对应从机的数据 */
static uint8_t slave1_seen;
static uint8_t slave2_seen;


/* 本次任务启动时间 */
static uint32_t app_run_start_tick;


/* 每个从机真正开始执行的时间 */
static uint32_t slave1_start_tick;
static uint32_t slave2_start_tick;


/* 周期发送时间 */
static uint32_t last_command_tick;
static uint32_t last_stop_tick;


/* 是否已经看到从机进入 RUNNING */
static uint8_t slave1_run_ack;
static uint8_t slave2_run_ack;


/* CAN发送连续失败次数 */
static uint8_t can_tx_fail_count;



/* =========================================================
   内部函数声明
   ========================================================= */

static void App_ProcessCanRx(uint32_t now);

static void App_UpdateOnlineState(uint32_t now);

static uint8_t App_CheckConfig(void);

static void App_RunTask(uint32_t now);

static void App_CheckRunFeedback(uint32_t now);

static uint8_t App_SendRunCommand(uint8_t slave_id);

static void App_SendStopToSlave(uint8_t slave_id);

static void App_SendStopAll(void);

static void App_EnterFault(App_Fault_t fault, uint32_t now);

static void App_ResetRunFlags(void);



/* =========================================================
   初始化
   ========================================================= */

void App_Init(void)
{
    /* 默认系统模式 */
    app_config.mode = APP_MODE_SYNC;


    /* Slave1 默认参数 */
    app_config.slave1.target_count = 0;
    app_config.slave1.target_speed = 0;
    app_config.slave1.direction = 1;


    /* Slave2 默认参数 */
    app_config.slave2.target_count = 0;
    app_config.slave2.target_speed = 0;
    app_config.slave2.direction = 1;


    /* 异步模式默认间隔 */
    app_config.async_delay_ms = 1000;


    /* 系统状态 */
    app_runtime.state = APP_STATE_IDLE;
    app_runtime.fault = APP_FAULT_NONE;
    app_runtime.mode = app_config.mode;


    app_runtime.slave1_online = 0;
    app_runtime.slave2_online = 0;

    app_runtime.slave1_started = 0;
    app_runtime.slave2_started = 0;

    app_runtime.slave1_finished = 0;
    app_runtime.slave2_finished = 0;


    app_runtime.slave1_status.total_count = 0;
    app_runtime.slave1_status.delta_count = 0;
    app_runtime.slave1_status.state = CAN_MOTOR_STATE_IDLE;

    app_runtime.slave2_status.total_count = 0;
    app_runtime.slave2_status.delta_count = 0;
    app_runtime.slave2_status.state = CAN_MOTOR_STATE_IDLE;


    slave1_last_rx_tick = 0;
    slave2_last_rx_tick = 0;

    slave1_seen = 0;
    slave2_seen = 0;


    app_run_start_tick = 0;

    slave1_start_tick = 0;
    slave2_start_tick = 0;


    last_command_tick = HAL_GetTick();
    last_stop_tick = HAL_GetTick();


    slave1_run_ack = 0;
    slave2_run_ack = 0;


    can_tx_fail_count = 0;
}



/* =========================================================
   APP主任务

   main while(1)中不断调用
   ========================================================= */

void App_Task(void)
{
    uint32_t now;


    now = HAL_GetTick();


    /* 1. 处理CAN接收 */
    CanProtocol_Process();


    /* 2. 获取协议层解析后的数据 */
    App_ProcessCanRx(now);


    /* 3. 判断从机是否在线 */
    App_UpdateOnlineState(now);


    /* 4. 根据当前系统状态运行 */
    switch (app_runtime.state)
    {
        case APP_STATE_IDLE:

            /*
             * IDLE状态不主动启动电机。
             *
             * 如果从机在线，则偶尔补发STOP，
             * 防止某个从机复位后处于异常状态。
             */
            if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
            {
                last_stop_tick = now;

                App_SendStopAll();
            }

            break;



        case APP_STATE_RUNNING:

            App_RunTask(now);

            break;



        case APP_STATE_STOPPING:

            if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
            {
                last_stop_tick = now;

                App_SendStopAll();
            }


            if ((uint32_t)(now - app_run_start_tick) >= APP_STOP_HOLD_MS)
            {
                app_runtime.state = APP_STATE_IDLE;

                App_ResetRunFlags();
            }

            break;



        case APP_STATE_FINISHED:

            /*
             * FINISHED后保持STOP。
             *
             * OLED可以显示 COMPLETE，
             * 用户按BACK/OK后再调用
             * App_AcknowledgeFinished()
             */
            if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
            {
                last_stop_tick = now;

                App_SendStopAll();
            }

            break;



        case APP_STATE_FAULT:

            /*
             * FAULT状态始终尝试让仍然在线的从机停止。
             */
            if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
            {
                last_stop_tick = now;

                App_SendStopAll();
            }

            break;



        default:

            app_runtime.state = APP_STATE_IDLE;

            break;
    }
}



/* =========================================================
   CAN接收处理
   ========================================================= */

static void App_ProcessCanRx(uint32_t now)
{
    Can_MotorStatus_t status;


    /* ---------- Slave1 状态 ---------- */

    if (CanProtocol_GetSlave1Status(&status))
    {
        app_runtime.slave1_status = status;

        slave1_last_rx_tick = now;
        slave1_seen = 1;
    }


    /* ---------- Slave2 状态 ---------- */

    if (CanProtocol_GetSlave2Status(&status))
    {
        app_runtime.slave2_status = status;

        slave2_last_rx_tick = now;
        slave2_seen = 1;
    }


    /* ---------- Slave1 心跳 ---------- */

    if (CanProtocol_GetHeartbeat(1))
    {
        slave1_last_rx_tick = now;
        slave1_seen = 1;
    }


    /* ---------- Slave2 心跳 ---------- */

    if (CanProtocol_GetHeartbeat(2))
    {
        slave2_last_rx_tick = now;
        slave2_seen = 1;
    }
}



/* =========================================================
   在线检测
   ========================================================= */

static void App_UpdateOnlineState(uint32_t now)
{
    if ((slave1_seen != 0) &&
        ((uint32_t)(now - slave1_last_rx_tick) <= APP_SLAVE_TIMEOUT_MS))
    {
        app_runtime.slave1_online = 1;
    }
    else
    {
        app_runtime.slave1_online = 0;
    }


    if ((slave2_seen != 0) &&
        ((uint32_t)(now - slave2_last_rx_tick) <= APP_SLAVE_TIMEOUT_MS))
    {
        app_runtime.slave2_online = 1;
    }
    else
    {
        app_runtime.slave2_online = 0;
    }
}



/* =========================================================
   参数检查
   ========================================================= */

static uint8_t App_CheckConfig(void)
{
    if ((app_config.mode != APP_MODE_SYNC) &&
        (app_config.mode != APP_MODE_ASYNC))
    {
        return 0;
    }


    /* Slave1 */

    if (app_config.slave1.target_count <= 0)
        return 0;

    if (app_config.slave1.target_speed <= 0)
        return 0;

    if ((app_config.slave1.direction != 1) &&
        (app_config.slave1.direction != -1))
    {
        return 0;
    }


    /* Slave2 */

    if (app_config.slave2.target_count <= 0)
        return 0;

    if (app_config.slave2.target_speed <= 0)
        return 0;

    if ((app_config.slave2.direction != 1) &&
        (app_config.slave2.direction != -1))
    {
        return 0;
    }


    return 1;
}



/* =========================================================
   RUN状态核心
   ========================================================= */

static void App_RunTask(uint32_t now)
{
    uint8_t tx_ok;


    /* ---------- 先检查从机掉线 ---------- */

    if (app_runtime.slave1_online == 0)
    {
        App_EnterFault(APP_FAULT_SLAVE1_OFFLINE, now);
        return;
    }


    if (app_runtime.slave2_online == 0)
    {
        App_EnterFault(APP_FAULT_SLAVE2_OFFLINE, now);
        return;
    }


    /* ---------- 从机主动报告故障 ---------- */

    if (app_runtime.slave1_status.state == CAN_MOTOR_STATE_FAULT)
    {
        App_EnterFault(APP_FAULT_SLAVE1, now);
        return;
    }


    if (app_runtime.slave2_status.state == CAN_MOTOR_STATE_FAULT)
    {
        App_EnterFault(APP_FAULT_SLAVE2, now);
        return;
    }



    /* =====================================================
       ASYNC模式 Slave2延迟启动
       ===================================================== */

    if (app_config.mode == APP_MODE_ASYNC)
    {
        if (app_runtime.slave2_started == 0)
        {
            if ((uint32_t)(now - app_run_start_tick) >=
                app_config.async_delay_ms)
            {
                app_runtime.slave2_started = 1;

                slave2_start_tick = now;

                /*
                 * 强制下一轮马上发一次RUN
                 */
                last_command_tick =
                    now - APP_COMMAND_PERIOD_MS;
            }
        }
    }



    /* ---------- 检查运行反馈 ---------- */

    App_CheckRunFeedback(now);


    if (app_runtime.state != APP_STATE_RUNNING)
    {
        return;
    }



    /* =====================================================
       周期发送RUN
       ===================================================== */

    if ((uint32_t)(now - last_command_tick) >=
        APP_COMMAND_PERIOD_MS)
    {
        last_command_tick = now;

        tx_ok = 1;


        /* ---------- Slave1 ---------- */

        if ((app_runtime.slave1_started != 0) &&
            (app_runtime.slave1_finished == 0))
        {
            if (!App_SendRunCommand(1))
            {
                tx_ok = 0;
            }
        }
        else
        {
            App_SendStopToSlave(1);
        }



        /* ---------- Slave2 ---------- */

        if ((app_runtime.slave2_started != 0) &&
            (app_runtime.slave2_finished == 0))
        {
            if (!App_SendRunCommand(2))
            {
                tx_ok = 0;
            }
        }
        else
        {
            App_SendStopToSlave(2);
        }



        /* ---------- CAN发送错误累计 ---------- */

        if (tx_ok)
        {
            can_tx_fail_count = 0;
        }
        else
        {
            if (can_tx_fail_count < 255)
            {
                can_tx_fail_count++;
            }


            if (can_tx_fail_count >= APP_CAN_TX_FAIL_LIMIT)
            {
                App_EnterFault(APP_FAULT_CAN_TX, now);

                return;
            }
        }
    }
}



/* =========================================================
   检查 Slave RUN应答 / 完成状态
   ========================================================= */

static void App_CheckRunFeedback(uint32_t now)
{
    /* =====================================================
       Slave1
       ===================================================== */

    if (app_runtime.slave1_started)
    {
        if (app_runtime.slave1_status.state ==
            CAN_MOTOR_STATE_RUNNING)
        {
            slave1_run_ack = 1;
        }


        /*
         * 必须先看到RUNNING，
         * 再接受DONE。
         *
         * 防止上一次任务留下的DONE状态
         * 被误认为本次已经完成。
         */
        if ((slave1_run_ack != 0) &&
            (app_runtime.slave1_status.state ==
             CAN_MOTOR_STATE_DONE))
        {
            app_runtime.slave1_finished = 1;
        }


        /*
         * RUN发出后长时间没有进入RUNNING
         */
        if ((slave1_run_ack == 0) &&
            ((uint32_t)(now - slave1_start_tick) >
             APP_RUN_ACK_TIMEOUT_MS))
        {
            App_EnterFault(APP_FAULT_SLAVE1_NO_ACK, now);

            return;
        }
    }



    /* =====================================================
       Slave2
       ===================================================== */

    if (app_runtime.slave2_started)
    {
        if (app_runtime.slave2_status.state ==
            CAN_MOTOR_STATE_RUNNING)
        {
            slave2_run_ack = 1;
        }


        if ((slave2_run_ack != 0) &&
            (app_runtime.slave2_status.state ==
             CAN_MOTOR_STATE_DONE))
        {
            app_runtime.slave2_finished = 1;
        }


        if ((slave2_run_ack == 0) &&
            ((uint32_t)(now - slave2_start_tick) >
             APP_RUN_ACK_TIMEOUT_MS))
        {
            App_EnterFault(APP_FAULT_SLAVE2_NO_ACK, now);

            return;
        }
    }



    /* =====================================================
       两个从机都完成
       ===================================================== */

    if ((app_runtime.slave1_finished != 0) &&
        (app_runtime.slave2_finished != 0))
    {
        App_SendStopAll();

        app_runtime.state = APP_STATE_FINISHED;

        last_stop_tick = now;
    }
}



/* =========================================================
   发送RUN
   ========================================================= */

static uint8_t App_SendRunCommand(uint8_t slave_id)
{
    Can_MotorCommand_t cmd;


    cmd.command = MOTOR_CMD_RUN;


    if (slave_id == 1)
    {
        cmd.direction =
            app_config.slave1.direction;

        cmd.target_count =
            app_config.slave1.target_count;

        cmd.target_speed =
            app_config.slave1.target_speed;
    }
    else if (slave_id == 2)
    {
        cmd.direction =
            app_config.slave2.direction;

        cmd.target_count =
            app_config.slave2.target_count;

        cmd.target_speed =
            app_config.slave2.target_speed;
    }
    else
    {
        return 0;
    }


    return CanProtocol_SendMotorCommand(
        slave_id,
        &cmd
    );
}



/* =========================================================
   发送STOP
   ========================================================= */

static void App_SendStopToSlave(uint8_t slave_id)
{
    Can_MotorCommand_t cmd;


    /*
     * 对已经确认离线的节点不继续疯狂发CAN，
     * 避免总线上完全没有ACK时填满发送邮箱。
     */

    if (slave_id == 1)
    {
        if (app_runtime.slave1_online == 0)
            return;
    }
    else if (slave_id == 2)
    {
        if (app_runtime.slave2_online == 0)
            return;
    }
    else
    {
        return;
    }


    cmd.command = MOTOR_CMD_STOP;

    cmd.direction = 1;

    cmd.target_count = 0;
    cmd.target_speed = 0;


    CanProtocol_SendMotorCommand(
        slave_id,
        &cmd
    );
}



static void App_SendStopAll(void)
{
    App_SendStopToSlave(1);

    App_SendStopToSlave(2);
}



/* =========================================================
   故障入口
   ========================================================= */

static void App_EnterFault(App_Fault_t fault,
                           uint32_t now)
{
    app_runtime.fault = fault;

    app_runtime.state = APP_STATE_FAULT;


    App_SendStopAll();


    last_stop_tick = now;
}



/* =========================================================
   清除一次运行的数据
   ========================================================= */

static void App_ResetRunFlags(void)
{
    app_runtime.slave1_started = 0;
    app_runtime.slave2_started = 0;

    app_runtime.slave1_finished = 0;
    app_runtime.slave2_finished = 0;


    slave1_run_ack = 0;
    slave2_run_ack = 0;


    can_tx_fail_count = 0;
}



/* =========================================================
   写入配置
   ========================================================= */

uint8_t App_SetConfig(const App_Config_t *config)
{
    if (config == 0)
    {
        return 0;
    }


    /*
     * 正在运行或者正在停止时，
     * 不允许修改目标参数。
     */
    if ((app_runtime.state == APP_STATE_RUNNING) ||
        (app_runtime.state == APP_STATE_STOPPING))
    {
        return 0;
    }


    app_config = *config;

    app_runtime.mode = app_config.mode;


    return 1;
}



/* =========================================================
   获取配置
   ========================================================= */

void App_GetConfig(App_Config_t *config)
{
    if (config == 0)
    {
        return;
    }


    *config = app_config;
}



/* =========================================================
   请求启动
   ========================================================= */

App_StartResult_t App_RequestStart(void)
{
    uint32_t now;


    now = HAL_GetTick();


    /* ---------- 当前状态不允许启动 ---------- */

    if ((app_runtime.state == APP_STATE_RUNNING) ||
        (app_runtime.state == APP_STATE_STOPPING))
    {
        return APP_START_ERR_BUSY;
    }


    /* ---------- 参数检查 ---------- */

    if (!App_CheckConfig())
    {
        return APP_START_ERR_PARAM;
    }


    /* ---------- 从机必须在线 ---------- */

    if (app_runtime.slave1_online == 0)
    {
        return APP_START_ERR_SLAVE1_OFFLINE;
    }


    if (app_runtime.slave2_online == 0)
    {
        return APP_START_ERR_SLAVE2_OFFLINE;
    }



    /* =====================================================
       初始化本轮任务
       ===================================================== */

    App_ResetRunFlags();


    app_runtime.fault = APP_FAULT_NONE;

    app_runtime.state = APP_STATE_RUNNING;

    app_runtime.mode = app_config.mode;


    app_run_start_tick = now;


    /* Slave1始终立即启动 */
    app_runtime.slave1_started = 1;

    slave1_start_tick = now;



    /* =====================================================
       SYNC模式
       ===================================================== */

    if (app_config.mode == APP_MODE_SYNC)
    {
        app_runtime.slave2_started = 1;

        slave2_start_tick = now;
    }


    /* =====================================================
       ASYNC模式

       Slave2之后由 App_RunTask 延迟启动
       ===================================================== */

    else
    {
        app_runtime.slave2_started = 0;

        slave2_start_tick = 0;
    }



    /*
     * 保证下一次App_Task立即发送RUN
     */
    last_command_tick =
        now - APP_COMMAND_PERIOD_MS;


    return APP_START_OK;
}



/* =========================================================
   用户请求停止
   ========================================================= */

void App_RequestStop(void)
{
    uint32_t now;


    now = HAL_GetTick();


    if (app_runtime.state == APP_STATE_IDLE)
    {
        return;
    }


    App_SendStopAll();


    /*
     * 这里重新使用 app_run_start_tick
     * 作为STOPPING计时起点。
     */
    app_run_start_tick = now;

    last_stop_tick = now;


    app_runtime.state = APP_STATE_STOPPING;
}



/* =========================================================
   完成界面确认
   ========================================================= */

void App_AcknowledgeFinished(void)
{
    if (app_runtime.state != APP_STATE_FINISHED)
    {
        return;
    }


    App_ResetRunFlags();

    app_runtime.state = APP_STATE_IDLE;
}



/* =========================================================
   清除故障
   ========================================================= */

void App_ClearFault(void)
{
    if (app_runtime.state != APP_STATE_FAULT)
    {
        return;
    }


    app_runtime.fault = APP_FAULT_NONE;

    App_ResetRunFlags();

    app_runtime.state = APP_STATE_IDLE;
}



/* =========================================================
   获取运行状态
   ========================================================= */

void App_GetRuntime(App_Runtime_t *runtime)
{
    if (runtime == 0)
    {
        return;
    }


    *runtime = app_runtime;
}



App_State_t App_GetState(void)
{
    return app_runtime.state;
}



App_Fault_t App_GetFault(void)
{
    return app_runtime.fault;

}
