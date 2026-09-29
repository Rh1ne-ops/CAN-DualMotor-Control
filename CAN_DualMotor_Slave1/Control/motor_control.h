#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"


void MotorControl_Init(float position_kp,
                       float speed_kp,
                       float speed_ki);

void MotorControl_SetTarget(int32_t target_count);

void MotorControl_Update(void);

void MotorControl_Stop(void);

uint8_t MotorControl_IsReached(void);

int32_t MotorControl_GetPosition(void);

int16_t MotorControl_GetSpeed(void);

int16_t MotorControl_GetPWM(void);

float MotorControl_GetTargetSpeed(void);

#endif
