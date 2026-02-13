/**
 * @file    app_flash.c
 * @brief   ZeroForm FOC Flash存储模块实现
 * @author  linglitel
 * @date    2026/2/13
 * 
 * @details 实现Flash读写操作，包括CRC校验、数据验证等功能。
 */

#include "app_flash.h"
#include "app_foc.h"
#include <string.h>
#include <stddef.h>

//==============================================================================
// 私有变量
//==============================================================================

/** @brief RAM中的数据副本 */
static Flash_StoredData_t flash_data;

/** @brief 数据是否已从Flash加载 */
static bool data_loaded = false;

//==============================================================================
// 私有函数声明
//==============================================================================
static uint32_t Flash_CalculateCRC(const Flash_StoredData_t *data);
static bool Flash_ValidateData(const Flash_StoredData_t *data);
static void Flash_SetDefaults(void);

//==============================================================================
// CRC32计算
//==============================================================================

/**
 * @brief   计算数据的CRC32校验值
 * @param   data 待校验的数据结构指针
 * @return  CRC32校验值
 * @note    使用标准CRC32多项式 0xEDB88320
 */
static uint32_t Flash_CalculateCRC(const Flash_StoredData_t *data) {
    /* 跳过magic, version, crc32字段本身，从electrical_angle_offset开始计算 */
    const uint8_t *ptr = (const uint8_t *)&data->electrical_angle_offset;
    uint32_t size = sizeof(Flash_StoredData_t) - offsetof(Flash_StoredData_t, electrical_angle_offset);

    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < size; i++) {
        crc ^= ptr[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFF;
}

//==============================================================================
// 验证Flash数据有效性
//==============================================================================

/**
 * @brief   验证Flash数据的有效性
 * @param   data 待验证的数据结构指针
 * @return  true: 数据有效, false: 数据无效
 */
static bool Flash_ValidateData(const Flash_StoredData_t *data) {
    /* 检查魔数 */
    if (data->magic != APP_FLASH_MAGIC_NUMBER) {
        return false;
    }

    /* 检查版本 */
    if (data->version != APP_FLASH_DATA_VERSION) {
        return false;
    }

    /* 检查CRC */
    uint32_t calculated_crc = Flash_CalculateCRC(data);
    if (data->crc32 != calculated_crc) {
        return false;
    }

    return true;
}

//==============================================================================
// 设置默认值
//==============================================================================

/**
 * @brief   将flash_data设置为默认值
 */
static void Flash_SetDefaults(void) {
    memset(&flash_data, 0, sizeof(Flash_StoredData_t));

    flash_data.magic = APP_FLASH_MAGIC_NUMBER;
    flash_data.version = APP_FLASH_DATA_VERSION;

    // 默认校准无效
    flash_data.calibration_valid = 0;
    flash_data.electrical_angle_offset = 0.0f;

    // 默认CAN配置
    flash_data.node_id = 0x01;
    flash_data.heartbeat_period = 100;

    // 默认PID参数 (Iq)
    flash_data.pid_iq.Kp = 2.5f;
    flash_data.pid_iq.Ki = 0.45f;
    flash_data.pid_iq.Kd = 0.0f;
    flash_data.pid_iq.output_limit = 16.0f;
    flash_data.pid_iq.integral_limit = 6.0f;

    // 默认PID参数 (Id)
    flash_data.pid_id.Kp = 0.0f;
    flash_data.pid_id.Ki = 0.0f;
    flash_data.pid_id.Kd = 0.0f;
    flash_data.pid_id.output_limit = 0.0f;
    flash_data.pid_id.integral_limit = 0.0f;

    // 默认PID参数 (Velocity)
    flash_data.pid_velocity.Kp = 0.1f;
    flash_data.pid_velocity.Ki = 0.01f;
    flash_data.pid_velocity.Kd = 0.0f;
    flash_data.pid_velocity.output_limit = 30.0f;
    flash_data.pid_velocity.integral_limit = 10.0f;

    // 默认PID参数 (Position)
    flash_data.pid_position.Kp = 5.0f;
    flash_data.pid_position.Ki = 0.0f;
    flash_data.pid_position.Kd = 0.1f;
    flash_data.pid_position.output_limit = 50.0f;
    flash_data.pid_position.integral_limit = 20.0f;

    // 齿槽补偿未校准
    flash_data.cogging_calibrated = 0;
    flash_data.cogging_enabled = 0;
    memset(flash_data.cogging_table, 0, sizeof(flash_data.cogging_table));
}

//==============================================================================
// 公共函数实现
//==============================================================================

/**
 * @brief 初始化Flash存储模块
 */
bool Flash_Init(void) {
    // 从Flash读取数据
    const Flash_StoredData_t *flash_ptr = (const Flash_StoredData_t *)APP_FLASH_STORAGE_ADDRESS;

    // 验证数据有效性
    if (Flash_ValidateData(flash_ptr)) {
        // 数据有效，复制到RAM
        memcpy(&flash_data, flash_ptr, sizeof(Flash_StoredData_t));
        data_loaded = true;
        return true;
    } else {
        // 数据无效，使用默认值
        Flash_SetDefaults();
        data_loaded = true;
        return false;
    }
}

/**
 * @brief 检查Flash中是否有有效的校准数据
 */
bool Flash_IsCalibrationValid(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return (flash_data.calibration_valid != 0);
}

/**
 * @brief 检查Flash中是否有有效的齿槽补偿数据
 */
bool Flash_IsCoggingValid(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return (flash_data.cogging_calibrated != 0);
}

/**
 * @brief 从Flash加载所有配置
 */
HAL_StatusTypeDef Flash_LoadAll(void) {
    const Flash_StoredData_t *flash_ptr = (const Flash_StoredData_t *)APP_FLASH_STORAGE_ADDRESS;

    if (Flash_ValidateData(flash_ptr)) {
        memcpy(&flash_data, flash_ptr, sizeof(Flash_StoredData_t));
        data_loaded = true;
        return HAL_OK;
    }

    return HAL_ERROR;
}

/**
 * @brief 保存所有配置到Flash
 */
HAL_StatusTypeDef Flash_SaveAll(void) {
    HAL_StatusTypeDef status;

    // 更新CRC
    flash_data.magic = APP_FLASH_MAGIC_NUMBER;
    flash_data.version = APP_FLASH_DATA_VERSION;
    flash_data.crc32 = Flash_CalculateCRC(&flash_data);

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

    // 写入数据 (每次写入8字节，即双字)
    uint64_t *src = (uint64_t *)&flash_data;
    uint32_t dest_addr = APP_FLASH_STORAGE_ADDRESS;
    uint32_t num_doublewords = (sizeof(Flash_StoredData_t) + 7) / 8;

    for (uint32_t i = 0; i < num_doublewords; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest_addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        dest_addr += 8;
    }

    // 锁定Flash
    HAL_FLASH_Lock();

    return HAL_OK;
}

/**
 * @brief 获取存储的电零角偏移
 */
float Flash_GetElectricalOffset(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return flash_data.electrical_angle_offset;
}

/**
 * @brief 设置电零角偏移
 */
void Flash_SetElectricalOffset(float offset) {
    flash_data.electrical_angle_offset = offset;
    flash_data.calibration_valid = 1;
}

/**
 * @brief 获取齿槽补偿表指针
 */
const float* Flash_GetCoggingTable(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return flash_data.cogging_table;
}

/**
 * @brief 设置齿槽补偿表
 */
void Flash_SetCoggingTable(const float *table, uint32_t size) {
    if (size > APP_COGGING_TABLE_SIZE) {
        size = APP_COGGING_TABLE_SIZE;
    }
    memcpy(flash_data.cogging_table, table, size * sizeof(float));
}

/**
 * @brief 标记齿槽校准完成
 */
void Flash_SetCoggingCalibrated(bool calibrated) {
    flash_data.cogging_calibrated = calibrated ? 1 : 0;
}

/**
 * @brief 获取CAN节点ID
 */
uint8_t Flash_GetNodeId(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return flash_data.node_id;
}

/**
 * @brief 设置CAN节点ID
 */
void Flash_SetNodeId(uint8_t node_id) {
    flash_data.node_id = node_id;
}

/**
 * @brief 获取心跳周期
 */
uint32_t Flash_GetHeartbeatPeriod(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return flash_data.heartbeat_period;
}

/**
 * @brief 设置心跳周期
 */
void Flash_SetHeartbeatPeriod(uint32_t period) {
    flash_data.heartbeat_period = period;
}

/**
 * @brief 获取PID参数
 */
const PID_Params_t* Flash_GetPidParams(uint8_t pid_type) {
    if (!data_loaded) {
        Flash_Init();
    }

    switch (pid_type) {
        case 0: return &flash_data.pid_iq;
        case 1: return &flash_data.pid_id;
        case 2: return &flash_data.pid_velocity;
        case 3: return &flash_data.pid_position;
        default: return &flash_data.pid_iq;
    }
}

/**
 * @brief 设置PID参数
 */
void Flash_SetPidParams(uint8_t pid_type, const PID_Params_t *params) {
    PID_Params_t *target = NULL;

    switch (pid_type) {
        case 0: target = &flash_data.pid_iq; break;
        case 1: target = &flash_data.pid_id; break;
        case 2: target = &flash_data.pid_velocity; break;
        case 3: target = &flash_data.pid_position; break;
        default: return;
    }

    memcpy(target, params, sizeof(PID_Params_t));
}

/**
 * @brief 擦除Flash存储区
 */
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

    // 重置为默认值
    Flash_SetDefaults();

    return status;
}

/**
 * @brief 获取当前RAM中的存储数据指针
 */
const Flash_StoredData_t* Flash_GetData(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return &flash_data;
}

/**
 * @brief 从FOC结构体同步PID参数到Flash数据结构
 */
void Flash_SyncPidFromFOC(void) {
    /* 同步Iq电流环PID */
    flash_data.pid_iq.Kp = FOC.pid_iq.Kp;
    flash_data.pid_iq.Ki = FOC.pid_iq.Ki;
    flash_data.pid_iq.Kd = FOC.pid_iq.Kd;
    flash_data.pid_iq.output_limit = FOC.pid_iq.output_limit;
    flash_data.pid_iq.integral_limit = FOC.pid_iq.integral_limit;
    
    /* 同步Id电流环PID */
    flash_data.pid_id.Kp = FOC.pid_id.Kp;
    flash_data.pid_id.Ki = FOC.pid_id.Ki;
    flash_data.pid_id.Kd = FOC.pid_id.Kd;
    flash_data.pid_id.output_limit = FOC.pid_id.output_limit;
    flash_data.pid_id.integral_limit = FOC.pid_id.integral_limit;
    
    /* 同步速度环PID */
    flash_data.pid_velocity.Kp = FOC.pid_velocity.Kp;
    flash_data.pid_velocity.Ki = FOC.pid_velocity.Ki;
    flash_data.pid_velocity.Kd = FOC.pid_velocity.Kd;
    flash_data.pid_velocity.output_limit = FOC.pid_velocity.output_limit;
    flash_data.pid_velocity.integral_limit = FOC.pid_velocity.integral_limit;
    
    /* 同步位置环PID */
    flash_data.pid_position.Kp = FOC.pid_position.Kp;
    flash_data.pid_position.Ki = FOC.pid_position.Ki;
    flash_data.pid_position.Kd = FOC.pid_position.Kd;
    flash_data.pid_position.output_limit = FOC.pid_position.output_limit;
    flash_data.pid_position.integral_limit = FOC.pid_position.integral_limit;
}

/**
 * @brief 从Flash数据结构加载PID参数到FOC结构体
 */
void Flash_LoadPidToFOC(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    
    /* 加载Iq电流环PID */
    FOC.pid_iq.Kp = flash_data.pid_iq.Kp;
    FOC.pid_iq.Ki = flash_data.pid_iq.Ki;
    FOC.pid_iq.Kd = flash_data.pid_iq.Kd;
    FOC.pid_iq.output_limit = flash_data.pid_iq.output_limit;
    FOC.pid_iq.integral_limit = flash_data.pid_iq.integral_limit;
    
    /* 加载Id电流环PID */
    FOC.pid_id.Kp = flash_data.pid_id.Kp;
    FOC.pid_id.Ki = flash_data.pid_id.Ki;
    FOC.pid_id.Kd = flash_data.pid_id.Kd;
    FOC.pid_id.output_limit = flash_data.pid_id.output_limit;
    FOC.pid_id.integral_limit = flash_data.pid_id.integral_limit;
    
    /* 加载速度环PID */
    FOC.pid_velocity.Kp = flash_data.pid_velocity.Kp;
    FOC.pid_velocity.Ki = flash_data.pid_velocity.Ki;
    FOC.pid_velocity.Kd = flash_data.pid_velocity.Kd;
    FOC.pid_velocity.output_limit = flash_data.pid_velocity.output_limit;
    FOC.pid_velocity.integral_limit = flash_data.pid_velocity.integral_limit;
    
    /* 加载位置环PID */
    FOC.pid_position.Kp = flash_data.pid_position.Kp;
    FOC.pid_position.Ki = flash_data.pid_position.Ki;
    FOC.pid_position.Kd = flash_data.pid_position.Kd;
    FOC.pid_position.output_limit = flash_data.pid_position.output_limit;
    FOC.pid_position.integral_limit = flash_data.pid_position.integral_limit;
}

/**
 * @brief 获取齿槽补偿启用状态
 */
bool Flash_GetCoggingEnabled(void) {
    if (!data_loaded) {
        Flash_Init();
    }
    return (flash_data.cogging_enabled != 0);
}

/**
 * @brief 设置齿槽补偿启用状态
 */
void Flash_SetCoggingEnabled(bool enabled) {
    flash_data.cogging_enabled = enabled ? 1 : 0;
}
