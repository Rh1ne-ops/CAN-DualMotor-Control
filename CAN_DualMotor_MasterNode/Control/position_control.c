#include "position_control.h"

#include <stdlib.h>



/* 最大目标速度
 * 单位：count / 10ms
 */
#define POSITION_SPEED_MAX      34.0f
#define POSITION_SPEED_MIN 			7.0f
/* 到位允许误差
 * 单位：count
 * 后面根据实际编码器每圈count再调整
 */
#define POSITION_TOLERANCE      15
#define POSITION_MIN_ZONE      150

static float position_kp = 0.0f;

static int32_t position_target = 0;


/* 初始化 */
void PositionControl_Init(float kp)
{
    position_kp = kp;

    position_target = 0;
}


/* 设置目标位置 */
void PositionControl_SetTarget(int32_t target_count)
{
    position_target = target_count;
}


/* 复位 */
void PositionControl_Reset(void)
{
    position_target = 0;
}


/* --------------------------------------------------
 * 位置环
 *
 * 输入：
 * current_count = 当前累计位置
 *
 * 输出：
 * target_speed = 给速度环的目标速度
 * 单位：count / 10ms
 * -------------------------------------------------- */
float PositionControl_Update(int32_t current_count)
{
    int32_t error;
    int32_t abs_error;
    float target_speed;

    error = position_target - current_count;
    abs_error = abs(error);

    /* 足够接近：直接停，不再反复纠偏 */
    if (abs_error <= POSITION_TOLERANCE)
    {
        return 0.0f;
    }

    target_speed = position_kp * error;

    /* 最大速度限制 */
    if (target_speed > POSITION_SPEED_MAX)
    {
        target_speed = POSITION_SPEED_MAX;
    }
    else if (target_speed < -POSITION_SPEED_MAX)
    {
        target_speed = -POSITION_SPEED_MAX;
    }

    /*
     * 只有还离目标比较远时，
     * 才强制最低速度。
     *
     * 太靠近目标就不要再强行±7。
     */
    if (abs_error > POSITION_MIN_ZONE)
    {
        if (target_speed > 0.0f &&
            target_speed < POSITION_SPEED_MIN)
        {
            target_speed = POSITION_SPEED_MIN;
        }
        else if (target_speed < 0.0f &&
                 target_speed > -POSITION_SPEED_MIN)
        {
            target_speed = -POSITION_SPEED_MIN;
        }
    }

    return target_speed;
}


/* 判断是否到位 */
uint8_t PositionControl_IsReached(int32_t current_count)
{
    int32_t error;

    error = position_target - current_count;

    if (abs(error) <= POSITION_TOLERANCE)
    {
        return 1;
    }

    return 0;
}

