//
// Created by linglitel on 2026/1/30.
//

#include "app_current.h"

#include "main.h"
#include "app_foc.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;

#define ADC_RESOLUTION      4096.0f
#define VREF                3.3f
#define SENSOR_SENSITIVITY  0.01f
#define AMP_GAIN            20.0f
#define AMPS_PER_COUNT      (VREF / (ADC_RESOLUTION * SENSOR_SENSITIVITY * AMP_GAIN))

static float ia_offset = 0.0f;
static float ic_offset = 0.0f;

void Current_Init(void) {
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    HAL_Delay(500);
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedStart(&hadc1);
    float sum_a = 0;
    float sum_b = 0;
    int calibration_rounds = 1000;
    HAL_TIM_Base_Start(&htim1);
    for (int i = 0; i < calibration_rounds; i++) {
        HAL_Delay(1);
        sum_a += (float) hadc1.Instance->JDR1;
        sum_b += (float) hadc2.Instance->JDR1;
    }
    ia_offset = sum_a / (float) calibration_rounds;
    ic_offset = sum_b / (float) calibration_rounds;
    HAL_TIM_Base_Stop(&htim1);
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);
}

void Current_Update(void) {
    FOC.Ia = ((float) (hadc1.Instance->JDR1) - ia_offset) * AMPS_PER_COUNT;
    FOC.Ic = -((float) (hadc2.Instance->JDR1) - ic_offset) * AMPS_PER_COUNT;
    FOC.Ib = -(FOC.Ia + FOC.Ic);
}
