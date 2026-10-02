#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"


/*
 * 初始化：
 * position_kp : 位置环P参数
 * speed_kp    : 速度环PI的Kp
 * speed_ki    : 速度环PI的Ki
 */
void MotorControl_Init(float position_kp,
                       float speed_kp,
                       float speed_ki);


/*
 * 启动一次新的位置运动任务
 *
 * target_count:
 *     有符号目标位置
 *     > 0 正方向
 *     < 0 反方向
 *
 * speed_limit:
 *     最大目标速度
 *     单位与 Encoder_GetDeltaCount() 一致，
 *     即当前项目中为“每10ms编码器count变化量”
 */
void MotorControl_SetTarget(int32_t target_count,
                            int16_t speed_limit);


/* 每10ms调用一次 */
void MotorControl_Update(void);


/* 强制停止 */
void MotorControl_Stop(void);


/* 是否已经到达目标 */
uint8_t MotorControl_IsReached(void);


/* 调试数据 */
int32_t MotorControl_GetPosition(void);

int16_t MotorControl_GetSpeed(void);

int16_t MotorControl_GetPWM(void);


/*
 * 经过速度限制后的实际速度环目标
 */
float MotorControl_GetTargetSpeed(void);


/*
 * 当前设置的速度上限
 */
float MotorControl_GetSpeedLimit(void);


#endif
