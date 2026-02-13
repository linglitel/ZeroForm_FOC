//
// Created by linglitel on 2026/1/30.
//

#include "app_main.h"
#include "app_can.h"
#include "app_foc.h"
#include "app_cmd.h"
#include "app_cordic.h"
#include "stdio.h"
#include "app_current.h"
#include "app_encoder.h"
#include "app_utils.h"
#include "app_flash.h"

const char *VERSION = "1.0.2";

#define ADC_RESOLUTION      4095.0f
#define ADC_VREF            3.3f
#define VIN_R_UP            75000.0f
#define VIN_R_DOWN          10000.0f
#define VIN_DIV_RATIO       ((VIN_R_UP + VIN_R_DOWN) / VIN_R_DOWN)
#define ADC_TO_VOLT(adc)    ((adc) * ADC_VREF / ADC_RESOLUTION)
#define ADC_TO_VIN(adc)     (ADC_TO_VOLT(adc) * VIN_DIV_RATIO)

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern SPI_HandleTypeDef hspi1;
extern FDCAN_HandleTypeDef hfdcan1;

int _write(int file, const char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

void App_Init(void) {
    printf("ZeroForm FOC \r\n");
    printf("Author: ZeroForm Studio && 芯辰科技 \r\n");
    printf("Version: %s \r\n", VERSION);
    printf("System Initializing\r\n");

    HAL_GPIO_WritePin(BLDC_RESET_GPIO_Port, BLDC_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

    Current_Init();
    Encoder_Init();

    FOC.direction = -1;
    FOC.pairs = 11;
    FOC.Vbus = 12;

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_MOE_ENABLE(&htim1);

    ADC_Enable(&hadc2);
    ADC_Enable(&hadc1);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    float vbus = ADC_TO_VIN(raw);
    if (vbus < 11.0f) {
        printf("VBUS undervoltage fault, Please supply power in 10s\r\n");
        HAL_Delay(10000);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        uint16_t m_raw = HAL_ADC_GetValue(&hadc1);
        float m_vbus = ADC_TO_VIN(m_raw);
        if (m_vbus < 11.0f) {
            printf("VBUS undervoltage fault ERROR_HANDLE\r\n");
            Error_Handler();
        }
    }
    HAL_ADC_Stop(&hadc1);

    DWT_Init();
    CORDIC_Init();

    PID_Init(&FOC.pid_id, 0, 0, 0, 1.0f / 10000.0f, 0, 0);
    FOC.Id = 0;
    PID_Init(&FOC.pid_iq, 2.5f, 0.45f, 0.0f, 1.0f / 10000.0f, 16.0f, 6.0f);
    PID_Init(&FOC.pid_velocity, 5.0f, 0.0f, 0.0f, 1.0f / 5000.0f, 30.0f, 10.0f);
    PID_Init(&FOC.pid_position, 5.0f, 0.0f, 0.0f, 1.0f / 5000.0f, 50.0f, 20.0f);
    FOC.mode = IDLE;

    Encoder_Update();
    FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);

    // 检查Flash中是否有有效的电零角校准数据
    bool flash_valid = Flash_Init();

    if (flash_valid && Flash_IsCalibrationValid()) {
        // 有有效校准数据，直接加载
        FOC.electrical_angle_offset = Flash_GetElectricalOffset();
        printf("Loaded calibration from Flash: offset = %.4f rad\r\n", FOC.electrical_angle_offset);
    } else {
        // 首次上电或数据无效，执行电零角校准
        printf("No valid calibration found, starting calibration...\r\n");
        FOC_AlignSensor(5.0f);
        HAL_Delay(1000);
        printf("Encoder aligned: offset = %.4f rad\r\n", FOC.electrical_angle_offset);

        // 保存到Flash
        if (Flash_SaveElectricalOffset(FOC.electrical_angle_offset) == HAL_OK) {
            printf("Calibration saved to Flash!\r\n");
        } else {
            printf("Failed to save calibration!\r\n");
        }
    }

    CAN_Init(&hfdcan1);
    CAN_Start();
    printf("System Initialized\r\n");

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    CMD_Init(&huart1);

    while (1) {
        CMD_Process();
        CAN_Process();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    CMD_UART_RxCallback(huart);
}
