/**
 * @file    app_flash.h
 * @brief   ZeroForm FOC Flash存储模块
 * @author  linglitel
 * @date    2026/2/13
 * 
 * @details 本模块负责将校准数据和配置参数存储到STM32G431内部Flash中，
 *          实现掉电保存功能。使用Flash最后一页(Page 63)作为存储区域。
 * 
 * @note    STM32G431CB: 128KB Flash, Page Size = 2KB
 */

#ifndef ZEROFORM_FOC_APP_FLASH_H
#define ZEROFORM_FOC_APP_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"

//==============================================================================
// Flash配置宏定义
//==============================================================================

/**
 * @brief Flash存储页号 (使用最后一页)
 * @note  STM32G431CB共64页，Page 63为最后一页
 */
#define APP_FLASH_STORAGE_PAGE          63

/**
 * @brief Flash存储起始地址
 * @note  计算公式: 0x08000000 + (Page * 2048)
 */
#define APP_FLASH_STORAGE_ADDRESS       (0x08000000 + (APP_FLASH_STORAGE_PAGE * 2048))  // 0x0801F800

/**
 * @brief Flash页大小 (字节)
 */
#define APP_FLASH_PAGE_SIZE             2048

/**
 * @brief 数据魔数，用于验证Flash数据有效性
 */
#define APP_FLASH_MAGIC_NUMBER          0x5A5A5A5A

/**
 * @brief 数据版本号，用于兼容性检查
 */
#define APP_FLASH_DATA_VERSION          1

/**
 * @brief 齿槽补偿表大小 (采样点数)
 * @note  128个点覆盖一个机械周期 (0-2π)
 */
#define APP_COGGING_TABLE_SIZE          128

//==============================================================================
// PID参数结构体 (用于存储)
//==============================================================================
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float output_limit;
    float integral_limit;
} PID_Params_t;

//==============================================================================
// Flash存储数据结构体
//==============================================================================
typedef struct {
    // 头部信息 (12字节)
    uint32_t magic;                         // 魔数校验
    uint32_t version;                       // 数据版本
    uint32_t crc32;                         // CRC校验
    
    // 电机校准数据 (8字节)
    float electrical_angle_offset;          // 电零角偏移
    uint8_t calibration_valid;              // 校准有效标志
    uint8_t reserved1[3];                   // 对齐填充
    
    // CAN配置 (8字节)
    uint8_t node_id;                        // CAN节点ID
    uint8_t reserved2[3];                   // 对齐填充
    uint32_t heartbeat_period;              // 心跳周期 (ms)
    
    // PID参数 (80字节 = 4 * 20字节)
    PID_Params_t pid_iq;                    // Iq电流环PID
    PID_Params_t pid_id;                    // Id电流环PID
    PID_Params_t pid_velocity;              // 速度环PID
    PID_Params_t pid_position;              // 位置环PID
    
    // 齿槽补偿数据 (520字节)
    uint8_t cogging_calibrated;             // 齿槽校准有效标志
    uint8_t cogging_enabled;                // 齿槽补偿启用标志
    uint8_t reserved3[2];                   // 对齐填充
    float cogging_table[APP_COGGING_TABLE_SIZE]; // 齿槽补偿表 (512字节)
    
} __attribute__((aligned(8))) Flash_StoredData_t;

//==============================================================================
// 函数声明
//==============================================================================

/**
 * @brief 初始化Flash存储模块
 * @return true: Flash中有有效数据, false: 无有效数据
 */
bool Flash_Init(void);

/**
 * @brief 检查Flash中是否有有效的校准数据
 * @return true: 有效, false: 无效
 */
bool Flash_IsCalibrationValid(void);

/**
 * @brief 检查Flash中是否有有效的齿槽补偿数据
 * @return true: 有效, false: 无效
 */
bool Flash_IsCoggingValid(void);

/**
 * @brief 从Flash加载所有配置
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_LoadAll(void);

/**
 * @brief 保存所有配置到Flash
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_SaveAll(void);

/**
 * @brief 获取存储的电零角偏移
 * @return 电零角偏移值
 */
float Flash_GetElectricalOffset(void);

/**
 * @brief 设置电零角偏移 (仅在RAM中，需调用Flash_SaveAll保存)
 * @param offset 电零角偏移值
 */
void Flash_SetElectricalOffset(float offset);

/**
 * @brief 获取齿槽补偿表指针
 * @return 齿槽补偿表指针
 */
const float* Flash_GetCoggingTable(void);

/**
 * @brief 设置齿槽补偿表 (仅在RAM中，需调用Flash_SaveAll保存)
 * @param table 齿槽补偿表数据
 * @param size 数据大小
 */
void Flash_SetCoggingTable(const float *table, uint32_t size);

/**
 * @brief 标记齿槽校准完成
 */
void Flash_SetCoggingCalibrated(bool calibrated);

/**
 * @brief 获取CAN节点ID
 * @return 节点ID
 */
uint8_t Flash_GetNodeId(void);

/**
 * @brief 设置CAN节点ID (仅在RAM中)
 * @param node_id 节点ID
 */
void Flash_SetNodeId(uint8_t node_id);

/**
 * @brief 获取心跳周期
 * @return 心跳周期 (ms)
 */
uint32_t Flash_GetHeartbeatPeriod(void);

/**
 * @brief 设置心跳周期 (仅在RAM中)
 * @param period 心跳周期 (ms)
 */
void Flash_SetHeartbeatPeriod(uint32_t period);

/**
 * @brief 获取PID参数
 * @param pid_type PID类型 (0=iq, 1=id, 2=velocity, 3=position)
 * @return PID参数指针
 */
const PID_Params_t* Flash_GetPidParams(uint8_t pid_type);

/**
 * @brief 设置PID参数 (仅在RAM中)
 * @param pid_type PID类型
 * @param params PID参数
 */
void Flash_SetPidParams(uint8_t pid_type, const PID_Params_t *params);

/**
 * @brief 擦除Flash存储区
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Flash_Erase(void);

/**
 * @brief 获取当前RAM中的存储数据指针 (用于调试)
 * @return 存储数据指针
 */
const Flash_StoredData_t* Flash_GetData(void);

/**
 * @brief 从FOC结构体同步PID参数到Flash数据结构
 * @note  用于保存当前运行时的PID参数
 */
void Flash_SyncPidFromFOC(void);

/**
 * @brief 从Flash数据结构加载PID参数到FOC结构体
 * @note  用于恢复保存的PID参数
 */
void Flash_LoadPidToFOC(void);

/**
 * @brief 获取齿槽补偿启用状态
 * @return true: 启用, false: 禁用
 */
bool Flash_GetCoggingEnabled(void);

/**
 * @brief 设置齿槽补偿启用状态 (仅在RAM中)
 * @param enabled 启用状态
 */
void Flash_SetCoggingEnabled(bool enabled);

#endif //ZEROFORM_FOC_APP_FLASH_H
