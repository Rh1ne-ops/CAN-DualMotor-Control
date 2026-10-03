#include "App.h"
#include "can_protocol.h"


/* =========================================================
   时间参数
   ========================================================= */

#define APP_COMMAND_PERIOD_MS          50U
#define APP_STOP_PERIOD_MS             100U
#define APP_SLAVE_TIMEOUT_MS           500U
#define APP_RUN_ACK_TIMEOUT_MS         500U
#define APP_STOP_HOLD_MS               300U
#define APP_CAN_TX_FAIL_LIMIT          5U


/* =========================================================
   APP内部数据
   ========================================================= */

static App_Config_t app_config;
static App_Runtime_t app_runtime;

/* 当前真正下发给两个从机的目标。
 * 普通模式下直接复制 app_config；
 * COMBO 第二段时只把方向取反，不改用户配置。
 */
static App_MotorTarget_t run_target1;
static App_MotorTarget_t run_target2;

/* 最近一次收到从机消息的时间 */
static uint32_t slave1_last_rx_tick;
static uint32_t slave2_last_rx_tick;

/* 是否至少收到过一次对应从机的数据 */
static uint8_t slave1_seen;
static uint8_t slave2_seen;

/* 本次任务启动时间 */
static uint32_t app_run_start_tick;

/* STOPPING计时起点 */
static uint32_t app_stop_start_tick;

/* 每个从机真正开始执行的时间 */
static uint32_t slave1_start_tick;
static uint32_t slave2_start_tick;

/* COMBO中间等待起点 */
static uint32_t combo_wait_start_tick;

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
static void App_LoadRunTargets(void);
static void App_PrepareAsyncTargets(void);
static void App_StartSlave(uint8_t slave_id, uint32_t now);
static uint8_t App_CurrentMotionFinished(void);
static void App_HandleMotionFinished(uint32_t now);
static void App_StartComboSecondPhase(uint32_t now);


/* =========================================================
   初始化
   ========================================================= */

void App_Init(void)
{
    uint32_t now;

    now = HAL_GetTick();

    /* 默认先给出一组安全、可被菜单覆盖的参数 */
    app_config.mode = APP_MODE_SYNC;

    app_config.slave1.target_count = 0;
    app_config.slave1.target_speed = 0;
    app_config.slave1.direction = 1;

    app_config.slave2.target_count = 0;
    app_config.slave2.target_speed = 0;
    app_config.slave2.direction = 1;

    app_config.async_delay_ms = 2000;

    app_runtime.state = APP_STATE_IDLE;
    app_runtime.fault = APP_FAULT_NONE;
    app_runtime.mode = app_config.mode;
    app_runtime.phase = APP_PHASE_NONE;

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

    run_target1 = app_config.slave1;
    run_target2 = app_config.slave2;

    slave1_last_rx_tick = 0;
    slave2_last_rx_tick = 0;

    slave1_seen = 0;
    slave2_seen = 0;

    app_run_start_tick = 0;
    app_stop_start_tick = 0;

    slave1_start_tick = 0;
    slave2_start_tick = 0;

    combo_wait_start_tick = 0;

    last_command_tick = now;
    last_stop_tick = now;

    slave1_run_ack = 0;
    slave2_run_ack = 0;

    can_tx_fail_count = 0;
}


/* =========================================================
   APP主任务
   ========================================================= */

void App_Task(void)
{
    uint32_t now;

    now = HAL_GetTick();

    /* 1. 协议层把当前队列里的CAN帧解析完 */
    CanProtocol_Process();

    /* 2. 取出最新状态 / 心跳 */
    App_ProcessCanRx(now);

    /* 3. 更新在线状态 */
    App_UpdateOnlineState(now);

    /* 4. 根据系统状态执行 */
    switch (app_runtime.state)
    {
        case APP_STATE_IDLE:

            /*
             * IDLE持续补发STOP。
             * 从机即使刚复位，也不会因为旧状态自行启动。
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

            if ((uint32_t)(now - app_stop_start_tick) >= APP_STOP_HOLD_MS)
            {
                app_runtime.state = APP_STATE_IDLE;
                app_runtime.phase = APP_PHASE_NONE;
                App_ResetRunFlags();
            }

            break;


        case APP_STATE_FINISHED:

            /* 完成后持续保持STOP，直到用户确认结果。 */
            if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
            {
                last_stop_tick = now;
                App_SendStopAll();
            }

            break;


        case APP_STATE_FAULT:

            /* 故障状态始终尝试让仍在线的从机停止。 */
            if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
            {
                last_stop_tick = now;
                App_SendStopAll();
            }

            break;


        default:

            app_runtime.state = APP_STATE_IDLE;
            app_runtime.phase = APP_PHASE_NONE;

            break;
    }
}


/* =========================================================
   CAN接收处理
   ========================================================= */

static void App_ProcessCanRx(uint32_t now)
{
    Can_MotorStatus_t status;

    if (CanProtocol_GetSlave1Status(&status))
    {
        app_runtime.slave1_status = status;
        slave1_last_rx_tick = now;
        slave1_seen = 1;
    }

    if (CanProtocol_GetSlave2Status(&status))
    {
        app_runtime.slave2_status = status;
        slave2_last_rx_tick = now;
        slave2_seen = 1;
    }

    if (CanProtocol_GetHeartbeat(1))
    {
        slave1_last_rx_tick = now;
        slave1_seen = 1;
    }

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
    if (app_config.mode >= APP_MODE_COUNT)
    {
        return 0;
    }

    /* S1会参与的模式 */
    if ((app_config.mode != APP_MODE_S2_ONLY))
    {
        if (app_config.slave1.target_count <= 0)
            return 0;

        if (app_config.slave1.target_speed <= 0)
            return 0;

        if ((app_config.slave1.direction != 1) &&
            (app_config.slave1.direction != -1))
        {
            return 0;
        }
    }

    /* S2会参与的模式 */
    if ((app_config.mode != APP_MODE_S1_ONLY))
    {
        if (app_config.slave2.target_count <= 0)
            return 0;

        if (app_config.slave2.target_speed <= 0)
            return 0;

        if ((app_config.slave2.direction != 1) &&
            (app_config.slave2.direction != -1))
        {
            return 0;
        }
    }

    return 1;
}


/* =========================================================
   RUN状态核心
   ========================================================= */

static void App_RunTask(uint32_t now)
{
    uint8_t tx_ok;

    /* 系统级运行时要求两个节点都在线。
     * 即使当前是单电机任务，也不允许另一个节点掉线后继续跑。
     */
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
       DELAY模式：Slave2延时启动
       ===================================================== */

    if (app_config.mode == APP_MODE_DELAY)
    {
        if (app_runtime.slave2_started == 0)
        {
            if ((uint32_t)(now - app_run_start_tick) >= app_config.async_delay_ms)
            {
                App_StartSlave(2, now);
                last_command_tick = now - APP_COMMAND_PERIOD_MS;
            }
        }
    }


    /* =====================================================
       COMBO中间等待

       第一段两机都DONE后：
       STOP -> 等待 -> 两机反向重新RUN
       ===================================================== */

    if ((app_config.mode == APP_MODE_COMBO) &&
        (app_runtime.phase == APP_PHASE_WAIT))
    {
        if ((uint32_t)(now - last_stop_tick) >= APP_STOP_PERIOD_MS)
        {
            last_stop_tick = now;
            App_SendStopAll();
        }

        /*
         * 两个从机已经收到STOP进入IDLE，
         * 并且等待时间也到了，才启动第二段。
         */
        if (((uint32_t)(now - combo_wait_start_tick) >= app_config.async_delay_ms) &&
            (app_runtime.slave1_status.state == CAN_MOTOR_STATE_IDLE) &&
            (app_runtime.slave2_status.state == CAN_MOTOR_STATE_IDLE))
        {
            App_StartComboSecondPhase(now);
        }

        return;
    }


    /* 检查RUNNING / DONE / ACK超时 */
    App_CheckRunFeedback(now);

    if (app_runtime.state != APP_STATE_RUNNING)
    {
        return;
    }


    /* 当前动作段完成 */
    if (App_CurrentMotionFinished())
    {
        App_HandleMotionFinished(now);
        return;
    }


    /* =====================================================
       周期发送RUN / STOP
       ===================================================== */

    if ((uint32_t)(now - last_command_tick) >= APP_COMMAND_PERIOD_MS)
    {
        last_command_tick = now;
        tx_ok = 1;

        /* Slave1 */
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

        /* Slave2 */
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
    /* ---------- Slave1 ---------- */
    if ((app_runtime.slave1_started != 0) &&
        (app_runtime.slave1_finished == 0))
    {
        if (app_runtime.slave1_status.state == CAN_MOTOR_STATE_RUNNING)
        {
            slave1_run_ack = 1;
        }

        if ((slave1_run_ack != 0) &&
            (app_runtime.slave1_status.state == CAN_MOTOR_STATE_DONE))
        {
            app_runtime.slave1_finished = 1;
        }

        if ((slave1_run_ack == 0) &&
            ((uint32_t)(now - slave1_start_tick) > APP_RUN_ACK_TIMEOUT_MS))
        {
            App_EnterFault(APP_FAULT_SLAVE1_NO_ACK, now);
            return;
        }
    }


    /* ---------- Slave2 ---------- */
    if ((app_runtime.slave2_started != 0) &&
        (app_runtime.slave2_finished == 0))
    {
        if (app_runtime.slave2_status.state == CAN_MOTOR_STATE_RUNNING)
        {
            slave2_run_ack = 1;
        }

        if ((slave2_run_ack != 0) &&
            (app_runtime.slave2_status.state == CAN_MOTOR_STATE_DONE))
        {
            app_runtime.slave2_finished = 1;
        }

        if ((slave2_run_ack == 0) &&
            ((uint32_t)(now - slave2_start_tick) > APP_RUN_ACK_TIMEOUT_MS))
        {
            App_EnterFault(APP_FAULT_SLAVE2_NO_ACK, now);
            return;
        }
    }
}


/* =========================================================
   当前动作段是否完成
   ========================================================= */

static uint8_t App_CurrentMotionFinished(void)
{
    if (app_config.mode == APP_MODE_S1_ONLY)
    {
        return app_runtime.slave1_finished;
    }

    if (app_config.mode == APP_MODE_S2_ONLY)
    {
        return app_runtime.slave2_finished;
    }

    /* 其余模式当前动作段都要求两机完成 */
    if ((app_runtime.slave1_finished != 0) &&
        (app_runtime.slave2_finished != 0))
    {
        return 1;
    }

    return 0;
}


/* =========================================================
   当前动作段完成后的处理
   ========================================================= */

static void App_HandleMotionFinished(uint32_t now)
{
    App_SendStopAll();
    last_stop_tick = now;

    /*
     * COMBO第一段完成后不结束整个任务，
     * 而是进入等待阶段。
     */
    if ((app_config.mode == APP_MODE_COMBO) &&
        (app_runtime.phase == APP_PHASE_PRIMARY))
    {
        app_runtime.phase = APP_PHASE_WAIT;

        app_runtime.slave1_started = 0;
        app_runtime.slave2_started = 0;

        combo_wait_start_tick = now;

        return;
    }

    /* 普通模式，或COMBO第二段完成 */
    app_runtime.state = APP_STATE_FINISHED;
}


/* =========================================================
   COMBO第二段
   ========================================================= */

static void App_StartComboSecondPhase(uint32_t now)
{
    /* 第二段目标圈数、速度保持不变，只把方向反过来。 */
    run_target1 = app_config.slave1;
    run_target2 = app_config.slave2;

    run_target1.direction = -run_target1.direction;
    run_target2.direction = -run_target2.direction;

    App_ResetRunFlags();

    app_runtime.phase = APP_PHASE_SECONDARY;

    App_StartSlave(1, now);
    App_StartSlave(2, now);

    last_command_tick = now - APP_COMMAND_PERIOD_MS;
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
        cmd.direction = run_target1.direction;
        cmd.target_count = run_target1.target_count;
        cmd.target_speed = run_target1.target_speed;
    }
    else if (slave_id == 2)
    {
        cmd.direction = run_target2.direction;
        cmd.target_count = run_target2.target_count;
        cmd.target_speed = run_target2.target_speed;
    }
    else
    {
        return 0;
    }

    return CanProtocol_SendMotorCommand(slave_id, &cmd);
}


/* =========================================================
   发送STOP
   ========================================================= */

static void App_SendStopToSlave(uint8_t slave_id)
{
    Can_MotorCommand_t cmd;

    /* 对已经确认离线的节点不继续填发送邮箱。 */
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

    CanProtocol_SendMotorCommand(slave_id, &cmd);
}


static void App_SendStopAll(void)
{
    App_SendStopToSlave(1);
    App_SendStopToSlave(2);
}


/* =========================================================
   启动一个从机
   ========================================================= */

static void App_StartSlave(uint8_t slave_id, uint32_t now)
{
    if (slave_id == 1)
    {
        app_runtime.slave1_started = 1;
        app_runtime.slave1_finished = 0;
        slave1_run_ack = 0;
        slave1_start_tick = now;
    }
    else if (slave_id == 2)
    {
        app_runtime.slave2_started = 1;
        app_runtime.slave2_finished = 0;
        slave2_run_ack = 0;
        slave2_start_tick = now;
    }
}


/* =========================================================
   载入本轮目标
   ========================================================= */

static void App_LoadRunTargets(void)
{
    run_target1 = app_config.slave1;
    run_target2 = app_config.slave2;

    /*
     * ASYNC不是延迟启动。
     *
     * 两台同时启动，但目标圈数不同。
     * 根据“路程 / 速度 = 时间”，自动规划两台速度，
     * 让理论完成时间一致。
     */
    if (app_config.mode == APP_MODE_ASYNC)
    {
        App_PrepareAsyncTargets();
    }
}
static void App_PrepareAsyncTargets(void)
{
    uint32_t count1;
    uint32_t count2;

    uint32_t max_speed1;
    uint32_t max_speed2;

    uint32_t speed1;
    uint32_t speed2;

    count1 = (uint32_t)run_target1.target_count;
    count2 = (uint32_t)run_target2.target_count;

    max_speed1 = (uint32_t)run_target1.target_speed;
    max_speed2 = (uint32_t)run_target2.target_speed;

    if ((count1 == 0U) ||
        (count2 == 0U) ||
        (max_speed1 == 0U) ||
        (max_speed2 == 0U))
    {
        return;
    }

    /*
     * 比较：
     *
     * count1 / max_speed1
     * count2 / max_speed2
     *
     * 不用浮点数，交叉相乘即可。
     *
     * 谁需要的时间更长，
     * 就让谁保持自己的最大速度，
     * 另一台按比例降低速度。
     */
    if ((count1 * max_speed2) >=
        (count2 * max_speed1))
    {
        /*
         * S1是限制整个任务时间的一方。
         */
        speed1 = max_speed1;

        speed2 =
            (count2 * speed1 + count1 / 2U) /
            count1;

        if (speed2 < 1U)
        {
            speed2 = 1U;
        }
    }
    else
    {
        /*
         * S2是限制整个任务时间的一方。
         */
        speed2 = max_speed2;

        speed1 =
            (count1 * speed2 + count2 / 2U) /
            count2;

        if (speed1 < 1U)
        {
            speed1 = 1U;
        }
    }

    run_target1.target_speed = (int16_t)speed1;
    run_target2.target_speed = (int16_t)speed2;
}

/* =========================================================
   故障入口
   ========================================================= */

static void App_EnterFault(App_Fault_t fault, uint32_t now)
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

    slave1_start_tick = 0;
    slave2_start_tick = 0;

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

    /* 运行 / 正在停止时不允许改任务。 */
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

    /* 只有IDLE允许开始新任务。 */
    if (app_runtime.state != APP_STATE_IDLE)
    {
        return APP_START_ERR_BUSY;
    }

    if (!App_CheckConfig())
    {
        return APP_START_ERR_PARAM;
    }

    /* 作为完整系统运行，启动前两个从机都必须在线。 */
    if (app_runtime.slave1_online == 0)
    {
        return APP_START_ERR_SLAVE1_OFFLINE;
    }

    if (app_runtime.slave2_online == 0)
    {
        return APP_START_ERR_SLAVE2_OFFLINE;
    }

    App_ResetRunFlags();
    App_LoadRunTargets();

    app_runtime.fault = APP_FAULT_NONE;
    app_runtime.state = APP_STATE_RUNNING;
    app_runtime.mode = app_config.mode;
    app_runtime.phase = APP_PHASE_PRIMARY;

    app_run_start_tick = now;
    combo_wait_start_tick = 0;


    /* =====================================================
       根据模式决定谁先启动
       ===================================================== */

    switch (app_config.mode)
    {
        case APP_MODE_S1_ONLY:

            App_StartSlave(1, now);
            break;


        case APP_MODE_S2_ONLY:

            App_StartSlave(2, now);
            break;


        case APP_MODE_SYNC:
        case APP_MODE_ASYNC:
        case APP_MODE_COMBO:

            App_StartSlave(1, now);
            App_StartSlave(2, now);
            break;


        case APP_MODE_DELAY:

            App_StartSlave(1, now);
            /* Slave2等 App_RunTask 的延时条件满足后再启动。 */
            break;


        default:

            app_runtime.state = APP_STATE_IDLE;
            app_runtime.phase = APP_PHASE_NONE;
            return APP_START_ERR_PARAM;
    }

    /* 下一次App_Task立即下发RUN。 */
    last_command_tick = now - APP_COMMAND_PERIOD_MS;

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

    app_stop_start_tick = now;
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
    app_runtime.phase = APP_PHASE_NONE;
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

    app_runtime.phase = APP_PHASE_NONE;
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
