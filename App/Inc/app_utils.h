//
// Created by linglitel on 2026/1/30.
//

#ifndef ZEROFORM_FOC_APP_UTILS_H
#define ZEROFORM_FOC_APP_UTILS_H

#define _constrain(amt, low, high) ({ \
    typeof(amt) _amt = (amt); \
    _amt < (low) ? (low) : (_amt > (high) ? (high) : _amt); \
})

#define _2PI (6.283185307179586f)

void DWT_Init(void);

float _normalizeAngle(float angle);

#endif //ZEROFORM_FOC_APP_UTILS_H
