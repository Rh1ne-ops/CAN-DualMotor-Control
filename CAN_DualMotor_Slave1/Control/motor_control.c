#include "motor_control.h"

#include "Motor.h"
#include "Encoder.h"
#include "speed_pi.h"
#include "position_control.h"
static volatile uint8_t position_reached = 0;

static volatile float target_speed_now = 0.0f;

static volatile int32_t current_position = 0;

static volatile int16_t current_speed = 0;

static volatile int16_t pwm_command = 0;
// 【新增】
static volatile uint8_t control_enabled = 0;
// [新增] 当前位置任务的目标count
static int32_t target_position = 0;

// [新增] 上一次位置误差，用于检测是否跨过目标
static int32_t last_position_error = 0;

// [新增] 上一次误差是否有效
static uint8_t error_valid = 0;

/* 初始化整个单电机闭环控制链 */
void MotorControl_Init(float position_kp,
                       float speed_kp,
                       float speed_ki)
{
    Motor_Init();

    Encoder_Init();

    PositionControl_Init(position_kp);

    SpeedPI_Init(speed_kp, speed_ki);

    current_position = 0;
    current_speed = 0;
    pwm_command = 0;

		target_position = 0;
    last_position_error = 0;
    error_valid = 0;
    position_reached = 0;
	
		control_enabled = 0;

    Motor_Stop();
}


/* 设置位置目标 */
void MotorControl_SetTarget(int32_t target_count)
{
    PositionControl_SetTarget(target_count);

    target_position = target_count;
    /*
     * 开始新目标时清积分，
     * 防止上一次任务的积分残留
     */
    SpeedPI_Reset();
		position_reached = 0;
		error_valid = 0;

    // [新增]
    target_speed_now = 0.0f;
	
		control_enabled = 1;
}


/*
 * 每10ms调用一次
 */
void MotorControl_Update(void)
{
    int32_t position_error;
		Encoder_Update();

    current_position =
        Encoder_GetTotalCount();

    current_speed =
        Encoder_GetDeltaCount();


		if (!control_enabled)
		{
				target_speed_now = 0.0f;
				pwm_command = 0;

				Motor_Stop();

				return;
		}
			
				/* 已经到达过目标，就保持停止 */
    if (position_reached)
    {
				target_speed_now = 0.0f;
			
        pwm_command = 0;

        Motor_Stop();

        return;
    }
		position_error =
        target_position - current_position;

		   /*
     * [新增]
     * 第一次运行时只记录误差，
     * 不做过零判断。
     */
    if (!error_valid)
    {
        last_position_error =
            position_error;

        error_valid = 1;
    }

		
		
     if (PositionControl_IsReached(current_position) ||

        (last_position_error > 0 &&
         position_error < 0) ||

        (last_position_error < 0 &&
         position_error > 0))
    {
        // [新增] 锁存“已经完成”
        position_reached = 1;

        // [新增] 清积分，防止残余积分再次推电机
        SpeedPI_Reset();
				control_enabled = 0;
				
        target_speed_now = 0.0f;
        pwm_command = 0;

        Motor_Stop();

        return;
    }
		last_position_error =
        position_error;


    target_speed_now =
        PositionControl_Update(current_position);


    pwm_command =
        SpeedPI_Update(
            target_speed_now,
            (float)current_speed
        );


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

float MotorControl_GetTargetSpeed(void)
{
    return target_speed_now;
}

/* 强制停止 */
void MotorControl_Stop(void)
{
    SpeedPI_Reset();
		control_enabled = 0;
    pwm_command = 0;
		target_speed_now = 0.0f;
    Motor_Stop();
}


/* 判断当前位置是否到达目标 */
uint8_t MotorControl_IsReached(void)
{
    return position_reached;
}


/* 当前累计位置 */
int32_t MotorControl_GetPosition(void)
{
    return current_position;
}


/* 最近10ms的count变化 */
int16_t MotorControl_GetSpeed(void)
{
    return current_speed;
}


/* 当前PI输出 */
int16_t MotorControl_GetPWM(void)
{
    return pwm_command;
}
