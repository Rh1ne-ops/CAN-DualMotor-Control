#include "Motor.h"                 
#include "tim.h"
void Motor_Init(void){
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	
	
	
	Motor_Stop();
}
void Motor_SetPWM(uint16_t pwm){
	
	uint32_t arr;
	uint32_t compare;
	
	if(pwm>MOTOR_PWM_MAX){
		pwm = MOTOR_PWM_MAX;
	}
	arr = __HAL_TIM_GET_AUTORELOAD(&htim3);

	compare = (arr*pwm)/MOTOR_PWM_MAX;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,compare);
}
void Motor_Forward(void)
{
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      GPIO_PIN_RESET);
}


void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      GPIO_PIN_SET);
}
void Motor_Stop(void){
	Motor_SetPWM(0);
	
	HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_0,
                      GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB,
                      GPIO_PIN_1,
                      GPIO_PIN_RESET);

}
