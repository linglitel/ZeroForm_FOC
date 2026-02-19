/**
 * @file    app_flash.c
 * @brief   ZeroForm FOC Flash存储模块实现 (精简版)
 * @author  linglitel
 * @date    2026/2/13
 */

#include "app_flash.h"
#include <string.h>

//==============================================================================
// 私有变量
//==============================================================================
static Flash_Data_t flash_data;
static bool data_loaded = false;

//==============================================================================
// 公共函数实现
//==============================================================================

bool Flash_Init(void) {
    const Flash_Data_t *flash_ptr = (const Flash_Data_t *)APP_FLASH_STORAGE_ADDRESS;
    
    // 检查魔数是否有效
    if (flash_ptr->magic == APP_FLASH_MAGIC_NUMBER && flash_ptr->calibration_valid) {
        memcpy(&flash_data, flash_ptr, sizeof(Flash_Data_t));
        data_loaded = true;
        return true;
    }
    
    // 数据无效，使用默认值
    flash_data.magic = APP_FLASH_MAGIC_NUMBER;
    flash_data.electrical_angle_offset = 0.0f;
    flash_data.logical_zero_offset = 0.0f;
    flash_data.calibration_valid = 0;
    flash_data.zero_point_valid = 0;
    data_loaded = true;
    return false;
}

bool Flash_IsCalibrationValid(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return (flash_data.calibration_valid != 0);
}

float Flash_GetElectricalOffset(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return flash_data.electrical_angle_offset;
}

HAL_StatusTypeDef Flash_SaveElectricalOffset(float offset) {
    HAL_StatusTypeDef status;
    
    // 更新数据
    flash_data.magic = APP_FLASH_MAGIC_NUMBER;
    flash_data.electrical_angle_offset = offset;
    flash_data.calibration_valid = 1;
    
    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }
    
    // 擦除页面
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = APP_FLASH_STORAGE_PAGE;
    erase_init.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }
    
    // 写入数据 (每次写入8字节)
    uint64_t *src = (uint64_t *)&flash_data;
    uint32_t dest_addr = APP_FLASH_STORAGE_ADDRESS;
    uint32_t num_doublewords = (sizeof(Flash_Data_t) + 7) / 8;
    
    for (uint32_t i = 0; i < num_doublewords; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest_addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        dest_addr += 8;
    }
    
    HAL_FLASH_Lock();
    return HAL_OK;
}

bool Flash_IsZeroPointValid(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return (flash_data.zero_point_valid != 0);
}

float Flash_GetLogicalZero(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return flash_data.logical_zero_offset;
}

HAL_StatusTypeDef Flash_SaveLogicalZero(float offset) {
    HAL_StatusTypeDef status;
    
    // 更新数据（保留其他字段不变）
    flash_data.magic = APP_FLASH_MAGIC_NUMBER;
    flash_data.logical_zero_offset = offset;
    flash_data.zero_point_valid = 1;
    
    // 解锁Flash
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }
    
    // 擦除页面
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = APP_FLASH_STORAGE_PAGE;
    erase_init.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }
    
    // 写入全部数据 (每次写入8字节)
    uint64_t *src = (uint64_t *)&flash_data;
    uint32_t dest_addr = APP_FLASH_STORAGE_ADDRESS;
    uint32_t num_doublewords = (sizeof(Flash_Data_t) + 7) / 8;
    
    for (uint32_t i = 0; i < num_doublewords; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest_addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        dest_addr += 8;
    }
    
    HAL_FLASH_Lock();
    return HAL_OK;
}

HAL_StatusTypeDef Flash_Erase(void) {
    HAL_StatusTypeDef status;
    
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }
    
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = APP_FLASH_STORAGE_PAGE;
    erase_init.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
    
    // 重置数据
    flash_data.magic = APP_FLASH_MAGIC_NUMBER;
    flash_data.electrical_angle_offset = 0.0f;
    flash_data.logical_zero_offset = 0.0f;
    flash_data.calibration_valid = 0;
    flash_data.zero_point_valid = 0;
    
    return status;
}
