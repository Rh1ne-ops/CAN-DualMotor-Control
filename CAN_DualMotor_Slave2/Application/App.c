#include "App.h"

#include "can_protocol.h"

#include "Motor.h"
#include "Encoder.h"
#include "motor_control.h"

#include "tim.h"


/* =========================================================
   控制参数
   ========================================================= */

/*
 * Master运行时每50ms发送一次命令。
 *
 * 300ms没有收到任何有效Master命令，
 * 认为通信失联。
 */
#define APP_MASTER_TIMEOUT_MS          300U


/* 状态上传周期 */
#define APP_STATUS_PERIOD_MS           50U


/* 心跳周期 */
#define APP_HEARTBEAT_PERIOD_MS        100U


/* =========================================================
   当前控制参数

   先沿用我们前面测试通过的参数。
   后面调参只改这里即可。
   ========================================================= */

#define APP_POSITION_KP                0.02f

#define APP_SPEED_KP                   80.0f

#define APP_SPEED_KI                   130.0f



/* =========================================================
   APP内部变量
   ========================================================= */

static App_SlaveRuntime_t app_runtime;


/*
 * 最近一次收到Master有效命令的时间
 */
static uint32_t master_last_cmd_tick;


/*
 * 是否至少收到过Master一次命令
 */
static uint8_t master_seen;


/*
 * 周期发送时间
 */
static uint32_t last_status_tick;

static uint32_t last_heartbeat_tick;


/*
 * 状态发生变化时，
 * 不等待50ms周期，立即发送一次状态。
 */
static uint8_t status_dirty;



/* =========================================================
   内部函数声明
   ========================================================= */

static void App_ProcessCommand(uint32_t now);

static uint8_t App_CheckRunCommand(
    const Can_MotorCommand_t *cmd
);

static void App_StartRun(
    const Can_MotorCommand_t *cmd
);

static void App_StopRun(void);

static void App_CheckMotorFinished(void);

static void App_CheckMasterTimeout(uint32_t now);

static void App_SendFeedback(uint32_t now);

static void App_EnterFault(
    App_SlaveFault_t fault
);

static void App_SetState(
    App_SlaveState_t state
);



/* =========================================================
   APP初始化
   ========================================================= */

void App_Init(void)
{
    uint32_t now;


    now = HAL_GetTick();


   

    /*
     * 整个控制层初始化
     *
     * position Kp
     * speed Kp
     * speed Ki
     */
    MotorControl_Init(
        APP_POSITION_KP,
        APP_SPEED_KP,
        APP_SPEED_KI
    );


    /*
     * 初始位置归零
     */
    Encoder_Reset();


    /*
     * 上电必须先确保电机停止
     */
    MotorControl_Stop();


    /*
     * TIM4 每10ms驱动 MotorControl_Update()
     */
    HAL_TIM_Base_Start_IT(&htim4);



    /* ---------- APP状态 ---------- */

    app_runtime.state =
        APP_SLAVE_STATE_IDLE;

    app_runtime.fault =
        APP_SLAVE_FAULT_NONE;

    app_runtime.master_online = 0;


    app_runtime.total_count = 0;

    app_runtime.delta_count = 0;



    /* ---------- 当前命令 ---------- */

    app_runtime.current_command.command =
        MOTOR_CMD_STOP;

    app_runtime.current_command.direction = 1;

    app_runtime.current_command.target_count = 0;

    app_runtime.current_command.target_speed = 0;



    /* ---------- 通信 ---------- */

    master_last_cmd_tick = now;

    master_seen = 0;


    last_status_tick = now;

    last_heartbeat_tick = now;


    /*
     * 上电后立即发一次状态
     */
    status_dirty = 1;
}



/* =========================================================
   APP主循环
   ========================================================= */

void App_Task(void)
{
    uint32_t now;


    now = HAL_GetTick();


    /*
     * 1. 协议层读取CAN并解析
     */
    CanProtocol_Process();


    /*
     * 2. 检查是否收到Master新命令
     */
    App_ProcessCommand(now);


    /*
     * 3. 检查MotorControl是否已经到达目标
     */
    App_CheckMotorFinished();


    /*
     * 4. 检查Master通信超时
     */
    App_CheckMasterTimeout(now);


    /*
     * 5. 更新编码器信息
     *
     * Encoder_Update已经属于你的10ms控制链的话，
     * 这里只读结果，绝对不要再Update一次。
     */
    app_runtime.total_count =
        Encoder_GetTotalCount();

    app_runtime.delta_count =
        Encoder_GetDeltaCount();


    /*
     * 6. 周期发送Status / Heartbeat
     */
    App_SendFeedback(now);
}



/* =========================================================
   读取Master命令
   ========================================================= */

static void App_ProcessCommand(uint32_t now)
{
    Can_MotorCommand_t cmd;


    if (!CanProtocol_GetCommand(&cmd))
    {
        return;
    }


    /*
     * 能走到这里说明：
     *
     * 收到了一条属于本从机的有效CAN命令帧。
     *
     * 无论RUN还是STOP，都刷新Master看门狗。
     */
    master_last_cmd_tick = now;

    master_seen = 1;

    app_runtime.master_online = 1;



    /* =====================================================
       STOP
       ===================================================== */

    if (cmd.command == MOTOR_CMD_STOP)
    {
        /*
         * STOP永远拥有最高优先级。
         */
        App_StopRun();

        return;
    }



    /* =====================================================
       RUN
       ===================================================== */

    if (cmd.command == MOTOR_CMD_RUN)
    {
        /*
         * 先检查参数是否合法
         */
        if (!App_CheckRunCommand(&cmd))
        {
            App_EnterFault(
                APP_SLAVE_FAULT_INVALID_COMMAND
            );

            return;
        }



        /*
         * -----------------------------
         * IDLE -> RUNNING
         * -----------------------------
         *
         * 只有IDLE状态允许真正启动一个新任务。
         */
        if (app_runtime.state ==
            APP_SLAVE_STATE_IDLE)
        {
            App_StartRun(&cmd);

            return;
        }



        /*
         * -----------------------------
         * RUNNING + RUN
         * -----------------------------
         *
         * Master会每50ms重复发RUN。
         *
         * 此时绝对不能：
         *
         * Encoder_Reset()
         * MotorControl_SetTarget()
         *
         * 否则任务每50ms就会重新开始。
         *
         * 所以这里什么都不做。
         *
         * 上面的master_last_cmd_tick已经刷新，
         * 重复RUN实际上充当通信看门狗。
         */
        if (app_runtime.state ==
            APP_SLAVE_STATE_RUNNING)
        {
            return;
        }



        /*
         * -----------------------------
         * DONE + RUN
         * -----------------------------
         *
         * 完成状态必须锁存。
         *
         * 即使Master还有几帧RUN在路上，
         * 也不能重新启动。
         *
         * 必须：
         *
         * DONE
         *  ↓
         * STOP
         *  ↓
         * IDLE
         *  ↓
         * 下一次RUN
         */
        if (app_runtime.state ==
            APP_SLAVE_STATE_DONE)
        {
            return;
        }



        /*
         * -----------------------------
         * FAULT + RUN
         * -----------------------------
         *
         * 故障状态不接受RUN。
         *
         * Master必须先发STOP复位。
         */
        if (app_runtime.state ==
            APP_SLAVE_STATE_FAULT)
        {
            return;
        }
    }
}



/* =========================================================
   检查RUN参数
   ========================================================= */

static uint8_t App_CheckRunCommand(
    const Can_MotorCommand_t *cmd)
{
    if (cmd == 0)
    {
        return 0;
    }


    /*
     * 方向必须是：
     *
     * +1 正转
     * -1 反转
     */
    if ((cmd->direction != 1) &&
        (cmd->direction != -1))
    {
        return 0;
    }


    /*
     * 我们Master最终设计里，
     * target_count统一存绝对值。
     */
    if (cmd->target_count <= 0)
    {
        return 0;
    }


    /*
     * target_speed同样必须有效。
     */
    if (cmd->target_speed <= 0)
    {
        return 0;
    }


    return 1;
}



/* =========================================================
   启动新任务
   ========================================================= */

static void App_StartRun(
    const Can_MotorCommand_t *cmd)
{
    int32_t signed_target;


    /*
     * 保存本轮任务。
     *
     * 后面的重复RUN不会覆盖它。
     */
    app_runtime.current_command = *cmd;



    /*
     * target_count在协议里保存的是绝对值，
     * direction单独表示方向。
     *
     * Control层使用有符号位置目标。
     */
    if (cmd->direction > 0)
    {
        signed_target =
            cmd->target_count;
    }
    else
    {
        signed_target =
            -cmd->target_count;
    }



    /*
     * 新任务开始前：
     *
     * 先停电机
     * 再把当前位置设为0
     */
    MotorControl_Stop();

    Encoder_Reset();



    /*
     * 设置新的相对位置目标。
     *
     * 你当前MotorControl接口就是单参数：
     *
     * MotorControl_SetTarget(28875);
     *
     * 所以这里完全沿用现有接口。
     */
    MotorControl_SetTarget(
    signed_target,
    cmd->target_speed
		);



    /*
     * 注意：
     *
     * cmd->target_speed
     * 当前已经完整保存到current_command。
     *
     * 但你的现有MotorControl还没有公开
     * “设置最大目标速度”的接口。
     *
     * 这个参数下一步只需要在这里接：
     *
     * MotorControl_SetMaxSpeed(
     *     cmd->target_speed
     * );
     *
     * APP其他部分完全不用改。
     */



    app_runtime.fault =
        APP_SLAVE_FAULT_NONE;


    App_SetState(
        APP_SLAVE_STATE_RUNNING
    );
}



/* =========================================================
   STOP处理
   ========================================================= */

static void App_StopRun(void)
{
    MotorControl_Stop();



    /*
     * STOP也是通信故障恢复命令。
     *
     * 比如：
     *
     * Master断线
     *      ↓
     * Slave FAULT
     *      ↓
     * Master重新上线
     *      ↓
     * Master IDLE周期发送STOP
     *      ↓
     * Slave恢复IDLE
     */
    app_runtime.fault =
        APP_SLAVE_FAULT_NONE;


    app_runtime.current_command.command =
        MOTOR_CMD_STOP;


    App_SetState(
        APP_SLAVE_STATE_IDLE
    );
}



/* =========================================================
   判断任务完成
   ========================================================= */

static void App_CheckMotorFinished(void)
{
    /*
     * 只有RUNNING才检查。
     */
    if (app_runtime.state !=
        APP_SLAVE_STATE_RUNNING)
    {
        return;
    }



    /*
     * MotorControl内部已经负责：
     *
     * 位置环
     *   ↓
     * 速度环
     *   ↓
     * PWM
     *   ↓
     * 目标容差判断
     */
    if (MotorControl_IsReached())
    {
        /*
         * 完成后立即停输出。
         */
        MotorControl_Stop();


        /*
         * DONE必须锁存。
         */
        App_SetState(
            APP_SLAVE_STATE_DONE
        );
    }
}



/* =========================================================
   Master通信看门狗
   ========================================================= */

static void App_CheckMasterTimeout(
    uint32_t now)
{
    /*
     * 从来没收到Master命令，
     * 上电IDLE时不算故障。
     */
    if (master_seen == 0)
    {
        app_runtime.master_online = 0;

        return;
    }



    if ((uint32_t)
        (now - master_last_cmd_tick) <=
        APP_MASTER_TIMEOUT_MS)
    {
        app_runtime.master_online = 1;

        return;
    }



    /*
     * 到这里：
     *
     * Master超过300ms没发有效命令。
     */
    app_runtime.master_online = 0;



    /*
     * 如果本来就在运行：
     *
     * 必须本地停车。
     *
     * 这是整个CAN断线保护最重要的一层。
     */
    if (app_runtime.state ==
        APP_SLAVE_STATE_RUNNING)
    {
        App_EnterFault(
            APP_SLAVE_FAULT_MASTER_TIMEOUT
        );
    }
}



/* =========================================================
   状态 / 心跳上传
   ========================================================= */

static void App_SendFeedback(
    uint32_t now)
{
    Can_MotorStatus_t status;



    /* =====================================================
       Status
       ===================================================== */

    if ((status_dirty != 0) ||
        ((uint32_t)(now - last_status_tick) >=
         APP_STATUS_PERIOD_MS))
    {
        status.total_count =
            Encoder_GetTotalCount();

        status.delta_count =
            Encoder_GetDeltaCount();



        switch (app_runtime.state)
        {
            case APP_SLAVE_STATE_IDLE:

                status.state =
                    CAN_MOTOR_STATE_IDLE;

                break;


            case APP_SLAVE_STATE_RUNNING:

                status.state =
                    CAN_MOTOR_STATE_RUNNING;

                break;


            case APP_SLAVE_STATE_DONE:

                status.state =
                    CAN_MOTOR_STATE_DONE;

                break;


            case APP_SLAVE_STATE_FAULT:

                status.state =
                    CAN_MOTOR_STATE_FAULT;

                break;


            default:

                status.state =
                    CAN_MOTOR_STATE_FAULT;

                break;
        }



        if (CanProtocol_SendMotorStatus(
            &status))
        {
            /*
             * 只有真正发送成功，
             * 才清dirty。
             */
            status_dirty = 0;

            last_status_tick = now;
        }
    }



    /* =====================================================
       Heartbeat
       ===================================================== */

    if ((uint32_t)
        (now - last_heartbeat_tick) >=
        APP_HEARTBEAT_PERIOD_MS)
    {
        if (CanProtocol_SendHeartbeat())
        {
            last_heartbeat_tick = now;
        }
    }
}



/* =========================================================
   进入故障
   ========================================================= */

static void App_EnterFault(
    App_SlaveFault_t fault)
{
    /*
     * 第一动作永远是停车。
     */
    MotorControl_Stop();


    app_runtime.fault = fault;


    App_SetState(
        APP_SLAVE_STATE_FAULT
    );
}



/* =========================================================
   状态切换
   ========================================================= */

static void App_SetState(
    App_SlaveState_t state)
{
    /*
     * 状态没变化不重复处理。
     */
    if (app_runtime.state == state)
    {
        return;
    }


    app_runtime.state = state;


    /*
     * 告诉发送模块：
     *
     * 状态变化了，
     * 下一轮立刻发Status，
     * 不等待50ms。
     */
    status_dirty = 1;
}



/* =========================================================
   获取运行信息
   ========================================================= */

void App_GetRuntime(
    App_SlaveRuntime_t *runtime)
{
    if (runtime == 0)
    {
        return;
    }


    app_runtime.total_count =
        Encoder_GetTotalCount();

    app_runtime.delta_count =
        Encoder_GetDeltaCount();


    *runtime = app_runtime;
}



App_SlaveState_t App_GetState(void)
{
    return app_runtime.state;
}



App_SlaveFault_t App_GetFault(void)
{
    return app_runtime.fault;
}
