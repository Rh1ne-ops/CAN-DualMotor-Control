#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_Reset(void);

int32_t Encoder_GetTotalCount(void);
int16_t Encoder_GetDeltaCount(void);

#endif

