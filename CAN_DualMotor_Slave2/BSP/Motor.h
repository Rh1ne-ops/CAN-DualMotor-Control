#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

#define MOTOR_PWM_MAX  1000

void Motor_Init(void);

void Motor_SetPWM(uint16_t pwm);

void Motor_Forward(void);

void Motor_Reverse(void);

void Motor_Stop(void);

#endif

