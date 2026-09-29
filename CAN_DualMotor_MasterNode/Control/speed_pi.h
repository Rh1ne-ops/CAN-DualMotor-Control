#ifndef __SPEED_PI_H
#define __SPEED_PI_H

#include "main.h"


void SpeedPI_Init(float kp, float ki);

void SpeedPI_Reset(void);

int16_t SpeedPI_Update(float target_speed,
                       float actual_speed);


#endif
