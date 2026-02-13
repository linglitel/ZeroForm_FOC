//
// Created by linglitel on 2026/1/30.
//
#include "app_utils.h"

#include <math.h>

#include "app_encoder.h"
static uint32_t cpu_freq_mhz = 0;

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    cpu_freq_mhz = SystemCoreClock / 1000000;
}

void DWT_Delay(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < cycles);
}

float _normalizeAngle(float angle) {
    float a = fmodf(angle, 2.0f * (float) M_PI);
    return (a < 0) ? a + 2.0f * (float) M_PI : a;
}
