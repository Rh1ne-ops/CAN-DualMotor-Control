#ifndef __POSITION_CONTROL_H
#define __POSITION_CONTROL_H

#include "main.h"


void PositionControl_Init(float kp);

void PositionControl_SetTarget(int32_t target_count);

void PositionControl_Reset(void);

float PositionControl_Update(int32_t current_count);

uint8_t PositionControl_IsReached(int32_t current_count);


#endif
