//
// Created by linglitel on 2026/1/30.
//

#ifndef ZEROFORM_FOC_APP_ENCODER_H
#define ZEROFORM_FOC_APP_ENCODER_H

#include "main.h"

#define MT6835_RESOLUTION 2097152.0f  // 21bit Resolution (2^21)

void Encoder_Init(void);

void Encoder_Update(void);


#endif //ZEROFORM_FOC_APP_ENCODER_H
