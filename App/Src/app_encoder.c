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


// 用于速度计算的累积变量
static int32_t velocity_delta_accumulator = 0;
static uint8_t velocity_sample_count = 0;

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

    // 计算角度增量并累积（用于速度环）
    int32_t delta_raw = (int32_t) raw - (int32_t) FOC.mechanical_angle_previous;
    if (delta_raw > (int32_t) (MT6835_RESOLUTION / 2)) {
        delta_raw -= (int32_t) MT6835_RESOLUTION;
    } else if (delta_raw < -(int32_t) (MT6835_RESOLUTION / 2)) {
        delta_raw += (int32_t) MT6835_RESOLUTION;
    }
    FOC.mechanical_angle_previous = raw;
    
    // 累积角度增量，在速度环中计算速度
    velocity_delta_accumulator += delta_raw;
    velocity_sample_count++;
}

// 在速度环中调用，计算速度（1kHz）
void Encoder_UpdateVelocity(void) {
    // 使用累积的角度增量计算速度
    // 速度环周期 = 1ms = 0.001s
    float delta_angle = (float) velocity_delta_accumulator * _2PI / (float) MT6835_RESOLUTION;
    float raw_velocity = (delta_angle / 0.001f) * (float) FOC.direction;  // 1kHz = 1ms周期
    
    // 速度滤波
    FOC.mechanical_velocity = 0.5f * raw_velocity + 0.5f * FOC.mechanical_velocity;
    FOC.electronic_velocity = FOC.mechanical_velocity * (float) FOC.pairs;
    
    // 重置累积器
    velocity_delta_accumulator = 0;
    velocity_sample_count = 0;
}
