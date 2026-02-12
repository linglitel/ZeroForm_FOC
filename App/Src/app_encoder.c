//
// Created by linglitel on 2026/1/30.
//

#include "app_encoder.h"
#include "app_foc.h"
#include "app_utils.h"

extern SPI_HandleTypeDef hspi1;


void Encoder_Init(void) {
    uint8_t tx[2] = {0xA0, 0x03};
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
}


void Encoder_Update(void) {
    uint8_t tx[6] = {0xA0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t rx[6] = {0};
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 6, 10);
    HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET); // CS HIGH
    uint32_t raw = ((uint32_t) rx[2] << 13) |
                   ((uint32_t) rx[3] << 5) |
                   ((uint32_t) rx[4] >> 3);
    FOC.mechanical_angle = (float) raw * _2PI / MT6835_RESOLUTION;
    float electrical_angle = (float) FOC.direction * FOC.mechanical_angle * (float) FOC.pairs - FOC.
                             electrical_angle_offset;
    FOC.electrical_angle = _normalizeAngle(electrical_angle);

    int32_t delta_raw = (int32_t) raw - (int32_t) FOC.mechanical_angle_previous;
    if (delta_raw > (int32_t) (MT6835_RESOLUTION / 2)) {
        delta_raw -= (int32_t) MT6835_RESOLUTION;
    } else if (delta_raw < -(int32_t) (MT6835_RESOLUTION / 2)) {
        delta_raw += (int32_t) MT6835_RESOLUTION;
    }
    FOC.mechanical_angle_previous = raw;
    float delta_angle = (float) delta_raw * _2PI / (float) MT6835_RESOLUTION;
    float raw_mechanical_velocity =
            (delta_angle / FOC_CONTROL_PERIOD) * (float) FOC.direction;
    FOC.mechanical_velocity =
            0.2f * raw_mechanical_velocity +
            0.8f * FOC.mechanical_velocity;
    FOC.electronic_velocity =
            FOC.mechanical_velocity * (float) FOC.pairs;
}
