//
// Created by linglitel on 2026/1/31.
//
#include "app_motor.h"
#include "main.h"
#include "app_utils.h"

extern TIM_HandleTypeDef htim1;

void Motor_Set_Duty(float a, float b, float c) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(_constrain(a, 0.00f, 0.95f) * htim1.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(_constrain(b, 0.00f, 0.95f) * htim1.Instance->ARR));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(_constrain(c, 0.00f, 0.95f) * htim1.Instance->ARR));
}
