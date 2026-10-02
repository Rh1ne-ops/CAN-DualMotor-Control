#include "motor_control.h"

#include "Motor.h"
#include "Encoder.h"
#include "speed_pi.h"
#include "position_control.h"


/* =========================================================
   控制状态
   ========================================================= */

static volatile uint8_t position_reached = 0;

static volatile uint8_t control_enabled = 0;


/*
 * 经过限幅之后，
 * 真正送进速度PI的目标速度
 */
static volatile float target_speed_now = 0.0f;


/*
 * 当前任务允许的最大目标速度
 *
 * 单位：
 * count / 10ms
 */
static volatile float target_speed_limit = 0.0f;


/*
 * 当前编码器状态
 */
static volatile int32_t current_position = 0;

static volatile int16_t current_speed = 0;


/*
 * 当前PI输出PWM
 */
static volatile int16_t pwm_command = 0;


/*
 * 当前运动任务的位置目标
 */
static int32_t target_position = 0;


/*
 * 上一次位置误差
 *
 * 用于判断是否跨过目标位置
 */
static int32_t last_position_error = 0;


/*
 * last_position_error是否有效
 */
static uint8_t error_valid = 0;



/* =========================================================
   初始化整个闭环控制系统
   ========================================================= */

void MotorControl_Init(float position_kp,
                       float speed_kp,
                       float speed_ki)
{
    /*
     * BSP初始化
     */
    Motor_Init();

    Encoder_Init();


    /*
     * 控制器初始化
     */
    PositionControl_Init(position_kp);

    SpeedPI_Init(speed_kp, speed_ki);



    /* ---------- 运行数据 ---------- */

    current_position = 0;

    current_speed = 0;

    pwm_command = 0;


    target_speed_now = 0.0f;

    target_speed_limit = 0.0f;



    /* ---------- 位置任务 ---------- */

    target_position = 0;

    last_position_error = 0;

    error_valid = 0;

    position_reached = 0;



    /* ---------- 控制状态 ---------- */

    control_enabled = 0;



    /*
     * 上电默认停止
     */
    Motor_Stop();
}



/* =========================================================
   设置新的运动目标
   ========================================================= */

void MotorControl_SetTarget(int32_t target_count,
                            int16_t speed_limit)
{
    /*
     * 非法速度参数不允许运行。
     *
     * 方向已经由target_count正负决定，
     * 所以speed_limit本身始终要求 > 0。
     */
    if (speed_limit <= 0)
    {
        MotorControl_Stop();

        return;
    }


    /*
     * 修改任务参数过程中先禁止控制。
     *
     * 因为MotorControl_Update运行在TIM4中断中，
     * 避免刚改一半参数就被中断拿去计算。
     */
    control_enabled = 0;



    /* ---------- 位置目标 ---------- */

    PositionControl_SetTarget(
        target_count
    );

    target_position =
        target_count;



    /* ---------- 速度限制 ---------- */

    target_speed_limit =
        (float)speed_limit;



    /*
     * 新任务开始，
     * 必须清除上一轮速度PI积分。
     */
    SpeedPI_Reset();



    /* ---------- 重置任务状态 ---------- */

    position_reached = 0;

    error_valid = 0;

    last_position_error = 0;


    target_speed_now = 0.0f;

    pwm_command = 0;



    /*
     * 所有参数都准备好以后，
     * 最后才允许控制中断真正工作。
     */
    control_enabled = 1;
}



/* =========================================================
   10ms控制任务
   ========================================================= */

void MotorControl_Update(void)
{
    int32_t position_error;



    /* =====================================================
       1. 更新编码器
       ===================================================== */

    Encoder_Update();


    current_position =
        Encoder_GetTotalCount();


    current_speed =
        Encoder_GetDeltaCount();



    /* =====================================================
       2. 控制未启动
       ===================================================== */

    if (!control_enabled)
    {
        target_speed_now = 0.0f;

        pwm_command = 0;

        Motor_Stop();

        return;
    }



    /* =====================================================
       3. 已经完成过本轮目标
       ===================================================== */

    if (position_reached)
    {
        target_speed_now = 0.0f;

        pwm_command = 0;

        Motor_Stop();

        return;
    }



    /* =====================================================
       4. 计算位置误差
       ===================================================== */

    position_error =
        target_position - current_position;



    /*
     * 第一次进入控制时：
     *
     * 只记录误差，
     * 不进行“跨过目标”判断。
     */
    if (!error_valid)
    {
        last_position_error =
            position_error;

        error_valid = 1;
    }



    /* =====================================================
       5. 判断是否完成
       ===================================================== */

    if (PositionControl_IsReached(current_position) ||

        /*
         * 正误差变负：
         * 从目标左边冲到右边
         */
        (last_position_error > 0 &&
         position_error < 0) ||

        /*
         * 负误差变正：
         * 从目标右边冲到左边
         */
        (last_position_error < 0 &&
         position_error > 0))
    {
        /*
         * 锁存完成状态
         */
        position_reached = 1;


        /*
         * 停止闭环
         */
        control_enabled = 0;


        /*
         * 清除PI积分
         */
        SpeedPI_Reset();


        target_speed_now = 0.0f;

        pwm_command = 0;


        Motor_Stop();

        return;
    }



    /*
     * 保存本轮位置误差，
     * 下一周期用于检查过零。
     */
    last_position_error =
        position_error;



    /* =====================================================
       6. 位置环
       ===================================================== */

    target_speed_now =
        PositionControl_Update(
            current_position
        );



    /* =====================================================
       7. 对位置环输出进行速度限制

       这是这次最重要的新增部分。
       ===================================================== */

    if (target_speed_now >
        target_speed_limit)
    {
        target_speed_now =
            target_speed_limit;
    }

    else if (target_speed_now <
             -target_speed_limit)
    {
        target_speed_now =
            -target_speed_limit;
    }



    /* =====================================================
       8. 速度PI
       ===================================================== */

    pwm_command =
        SpeedPI_Update(
            target_speed_now,
            (float)current_speed
        );



    /* =====================================================
       9. 输出给电机
       ===================================================== */

    if (pwm_command > 0)
    {
        Motor_Forward();

        Motor_SetPWM(
            (uint16_t)pwm_command
        );
    }

    else if (pwm_command < 0)
    {
        Motor_Reverse();

        Motor_SetPWM(
            (uint16_t)(-pwm_command)
        );
    }

    else
    {
        Motor_Stop();
    }
}



/* =========================================================
   强制停止
   ========================================================= */

void MotorControl_Stop(void)
{
    /*
     * 先禁止闭环
     */
    control_enabled = 0;


    /*
     * 清PI积分
     */
    SpeedPI_Reset();


    target_speed_now = 0.0f;

    target_speed_limit = 0.0f;

    pwm_command = 0;


    Motor_Stop();
}



/* =========================================================
   是否达到目标
   ========================================================= */

uint8_t MotorControl_IsReached(void)
{
    return position_reached;
}



/* =========================================================
   调试数据
   ========================================================= */

int32_t MotorControl_GetPosition(void)
{
    return current_position;
}



int16_t MotorControl_GetSpeed(void)
{
    return current_speed;
}



int16_t MotorControl_GetPWM(void)
{
    return pwm_command;
}



float MotorControl_GetTargetSpeed(void)
{
    return target_speed_now;
}



float MotorControl_GetSpeedLimit(void)
{
    return target_speed_limit;
}
