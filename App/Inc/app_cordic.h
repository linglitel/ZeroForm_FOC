//
// Created by linglitel on 2026/2/12.
// CORDIC hardware accelerated trigonometric functions
//

#ifndef ZEROFORM_FOC_APP_CORDIC_H
#define ZEROFORM_FOC_APP_CORDIC_H

/**
 * @brief Initialize CORDIC for sine/cosine calculation
 * @note Must be called after HAL_CORDIC_Init() in CubeMX generated code
 */
void CORDIC_Init(void);

/**
 * @brief Calculate sine and cosine using CORDIC hardware
 * @param angle Input angle in radians [-pi, pi]
 * @param sin_out Pointer to store sine result
 * @param cos_out Pointer to store cosine result
 * @note This is the most efficient way to get both sin and cos
 */
void CORDIC_SinCos(float angle, float *sin_out, float *cos_out);

/**
 * @brief Calculate cosine using CORDIC hardware
 * @param angle Input angle in radians
 * @return Cosine value
 */
float CORDIC_Cos(float angle);

/**
 * @brief Calculate sine using CORDIC hardware
 * @param angle Input angle in radians
 * @return Sine value
 */
float CORDIC_Sin(float angle);

#endif //ZEROFORM_FOC_APP_CORDIC_H
