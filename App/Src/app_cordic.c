//
// Created by linglitel on 2026/2/12.
// CORDIC hardware accelerated trigonometric functions
//

#include "app_cordic.h"
#include "main.h"

extern CORDIC_HandleTypeDef hcordic;

// CORDIC Q1.31 format conversion constants
#define CORDIC_Q31_SCALE    (2147483648.0f)  // 2^31
#define CORDIC_ANGLE_SCALE  (CORDIC_Q31_SCALE / 3.14159265358979f)  // Scale for [-pi, pi] to Q1.31

/**
 * @brief Initialize CORDIC for sine/cosine calculation
 */
void CORDIC_Init(void) {
    CORDIC_ConfigTypeDef config = {0};

    config.Function = CORDIC_FUNCTION_COSINE;
    config.Scale = CORDIC_SCALE_0;
    config.InSize = CORDIC_INSIZE_32BITS;
    config.OutSize = CORDIC_OUTSIZE_32BITS;
    config.NbWrite = CORDIC_NBWRITE_1;
    config.NbRead = CORDIC_NBREAD_2;
    config.Precision = CORDIC_PRECISION_6CYCLES;

    HAL_CORDIC_Configure(&hcordic, &config);
}

/**
 * @brief Calculate sine and cosine using CORDIC hardware
 * @param angle Input angle in radians [-pi, pi]
 * @param sin_out Pointer to store sine result
 * @param cos_out Pointer to store cosine result
 */
void CORDIC_SinCos(float angle, float *sin_out, float *cos_out) {
    // Normalize angle to [-pi, pi]
    while (angle > 3.14159265358979f) angle -= 6.28318530717959f;
    while (angle < -3.14159265358979f) angle += 6.28318530717959f;

    // Convert to Q1.31 format
    int32_t angle_q31 = (int32_t) (angle * CORDIC_ANGLE_SCALE);

    // Write angle to CORDIC
    CORDIC->WDATA = (uint32_t) angle_q31;

    // Read results (cosine first, then sine)
    int32_t cos_q31 = (int32_t) CORDIC->RDATA;
    int32_t sin_q31 = (int32_t) CORDIC->RDATA;

    // Convert back to float
    *cos_out = (float) cos_q31 / CORDIC_Q31_SCALE;
    *sin_out = (float) sin_q31 / CORDIC_Q31_SCALE;
}

/**
 * @brief Calculate cosine using CORDIC hardware
 * @param angle Input angle in radians
 * @return Cosine value
 */
float CORDIC_Cos(float angle) {
    float sin_val, cos_val;
    CORDIC_SinCos(angle, &sin_val, &cos_val);
    return cos_val;
}

/**
 * @brief Calculate sine using CORDIC hardware
 * @param angle Input angle in radians
 * @return Sine value
 */
float CORDIC_Sin(float angle) {
    float sin_val, cos_val;
    CORDIC_SinCos(angle, &sin_val, &cos_val);
    return sin_val;
}
