#include "speed_pi.h"


/* 控制周期：10ms */
#define SPEED_PI_DT            0.01f

/* 输出限制，对应 Motor_SetPWM() 的 0~1000 */
#define SPEED_PI_OUTPUT_MAX    1000.0f

/* 积分项限制，防止积分无限累积 */
#define SPEED_PI_INTEGRAL_MAX  5.0f
/*
 * 速度前馈系数
 *
 * 根据实测：
 * PWM800  -> SPD30
 * PWM900  -> SPD33
 * PWM1000 -> SPD37
 *
 * 暂定 27 PWM / (count/10ms)
 */
#define SPEED_FF_K           27.0f
#define SPEED_FF_START_SPEED   8.0f
#define SPEED_FF_FULL_SPEED   18.0f
static float speed_kp = 0.0f;
static float speed_ki = 0.0f;

static float speed_integral = 0.0f;


/* 初始化PI参数 */
void SpeedPI_Init(float kp, float ki)
{
    speed_kp = kp;
    speed_ki = ki;

    speed_integral = 0.0f;
}


/* 清除积分 */
void SpeedPI_Reset(void)
{
    speed_integral = 0.0f;
}


/* 速度PI计算 */
int16_t SpeedPI_Update(float target_speed,
                       float actual_speed)
{
    float error;
    float feedforward;
    float p_output;
    float i_output;

    float new_integral;
    float output;
	
		/*防止末端抽动的*/
		float abs_target;
    float ff_ratio;
	
		if (target_speed == 0.0f)
    {
        speed_integral = 0.0f;

        return 0;
    }

    /* 速度误差 */
    error = target_speed - actual_speed;

		if (target_speed >= 0.0f)
    {
        abs_target = target_speed;
    }
    else
    {
        abs_target = -target_speed;
    }

    if (abs_target <= SPEED_FF_START_SPEED)
    {
        ff_ratio = 0.0f;
    }
    else if (abs_target >= SPEED_FF_FULL_SPEED)
    {
        ff_ratio = 1.0f;
    }
    else
    {
        ff_ratio =
            (abs_target - SPEED_FF_START_SPEED) /
            (SPEED_FF_FULL_SPEED - SPEED_FF_START_SPEED);
    }

    feedforward =
        SPEED_FF_K *
        target_speed *
        ff_ratio;

		p_output =
        speed_kp * error;
    /* 积分 */
		new_integral =
        speed_integral
        + error * SPEED_PI_DT;
    /*
     * 积分状态限幅
     */
    if (new_integral >
        SPEED_PI_INTEGRAL_MAX)
    {
        new_integral =
            SPEED_PI_INTEGRAL_MAX;
    }
    else if (new_integral <
             -SPEED_PI_INTEGRAL_MAX)
    {
        new_integral =
            -SPEED_PI_INTEGRAL_MAX;
    }


    /*
     * 暂时使用新积分计算输出
     */
    i_output =
        speed_ki * new_integral;

    output =
        feedforward
        + p_output
        + i_output;


    /*
     * 抗积分饱和
     *
     * 如果输出已经超过+1000，
     * 而error还是正的，
     * 就不要继续增加正积分。
     *
     * 负方向同理。
     */
    if ((output > SPEED_PI_OUTPUT_MAX &&
         error > 0.0f) ||

        (output < -SPEED_PI_OUTPUT_MAX &&
         error < 0.0f))
    {
        /* 不接受本次积分 */
    }
    else
    {
        speed_integral =
            new_integral;
    }


    /*
     * 用真正保存下来的积分
     * 再计算最终输出
     */
    i_output =
        speed_ki * speed_integral;

    output =
        feedforward
        + p_output
        + i_output;


    /*
     * PWM最终限幅
     */
    if (output >
        SPEED_PI_OUTPUT_MAX)
    {
        output =
            SPEED_PI_OUTPUT_MAX;
    }
    else if (output <
             -SPEED_PI_OUTPUT_MAX)
    {
        output =
            -SPEED_PI_OUTPUT_MAX;
    }


    return (int16_t)output;
}
