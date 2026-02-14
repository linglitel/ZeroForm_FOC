//
// Created by linglitel on 2026/1/30.
//

#ifndef ZEROFORM_FOC_APP_ENCODER_H
#define ZEROFORM_FOC_APP_ENCODER_H

#include "main.h"

#define MT6835_RESOLUTION 2097152.0f  // 21bit Resolution (2^21)

void Encoder_Init(void);

void Encoder_Update(void);

// 在速度环中调用，计算速度（1kHz）
void Encoder_UpdateVelocity(void);

#endif //ZEROFORM_FOC_APP_ENCODER_H
