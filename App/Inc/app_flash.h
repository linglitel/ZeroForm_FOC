/**
 * @file    app_flash.h
 * @brief   ZeroForm FOC Flash存储模块 (精简版)
 * @author  linglitel
 * @date    2026/2/13
 * 
 * @details 仅存储电零角偏移到Flash，实现掉电保存功能。
 *          使用Flash最后一页(Page 63)作为存储区域。
 */

#ifndef ZEROFORM_FOC_APP_FLASH_H
#define ZEROFORM_FOC_APP_FLASH_H

#include "main.h"
#include <stdbool.h>

//==============================================================================
// Flash配置
//==============================================================================
#define APP_FLASH_STORAGE_PAGE      63
#define APP_FLASH_STORAGE_ADDRESS   (0x08000000 + (APP_FLASH_STORAGE_PAGE * 2048))
#define APP_FLASH_MAGIC_NUMBER      0x5A5A5A5A

//==============================================================================
// Flash存储数据结构 (16字节，对齐到8字节)
//==============================================================================
typedef struct {
    uint32_t magic;                     // 魔数校验
    float electrical_angle_offset;      // 电零角偏移
    float logical_zero_offset;          // 逻辑零点偏移
    uint8_t calibration_valid;          // 校准有效标志
    uint8_t zero_point_valid;           // 逻辑零点有效标志
    uint8_t reserved[2];               // 对齐填充
} Flash_Data_t;

//==============================================================================
// 函数声明
//==============================================================================

/**
 * @brief 初始化Flash模块，检查是否有有效校准数据
 * @return true: 有有效数据, false: 无有效数据
 */
bool Flash_Init(void);

/**
 * @brief 检查是否有有效的电零角校准
 * @return true: 有效, false: 无效
 */
bool Flash_IsCalibrationValid(void);

/**
 * @brief 获取电零角偏移
 * @return 电零角偏移值 (rad)
 */
float Flash_GetElectricalOffset(void);

/**
 * @brief 保存电零角偏移到Flash
 * @param offset 电零角偏移值 (rad)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_SaveElectricalOffset(float offset);

/**
 * @brief 检查是否有有效的逻辑零点
 * @return true: 有效, false: 无效
 */
bool Flash_IsZeroPointValid(void);

/**
 * @brief 获取逻辑零点偏移
 * @return 逻辑零点偏移值 (rad)
 */
float Flash_GetLogicalZero(void);

/**
 * @brief 保存逻辑零点偏移到Flash
 * @param offset 逻辑零点偏移值 (rad)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_SaveLogicalZero(float offset);

/**
 * @brief 擦除Flash存储区
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_Erase(void);

#endif //ZEROFORM_FOC_APP_FLASH_H
