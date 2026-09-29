#include "Encoder.h"
#include "tim.h"

/* 上一次读取的TIM2硬件计数值 */
static uint16_t last_count = 0;

/* 最近一个采样周期内变化的count */
static int16_t delta_count = 0;

/* 软件累计总count */
static int32_t total_count = 0;
void Encoder_Init(void){

	__HAL_TIM_SET_COUNTER(&htim2,0);
	
	 last_count  = 0;
   delta_count = 0;
   total_count = 0;
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);

}
void Encoder_Update(void){
	
	uint16_t current_count;
  current_count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
	delta_count = (int16_t)(uint16_t)(current_count - last_count);
	total_count += delta_count;
	last_count = current_count;
	
}
int32_t Encoder_GetTotalCount(void){
	return total_count;
}
int16_t Encoder_GetDeltaCount(void){
	return delta_count;
}
void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    last_count  = 0;
    delta_count = 0;
    total_count = 0;
}

