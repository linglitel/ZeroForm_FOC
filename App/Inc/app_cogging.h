/**
 * @file    app_cogging.h
 * @brief   ZeroForm FOC 齿槽补偿模块
 * @author  linglitel
 * @date    2026/2/13
 * 
 * @details 本模块实现齿槽力矩的校准和补偿功能。
 *          通过低速旋转电机采集齿槽力矩数据，生成补偿表，
 *          在运行时根据机械角度查表进行前馈补偿。
 * 
 * @note    校准过程需要约30秒，期间电机会正反转各一圈
 */

#ifndef ZEROFORM_FOC_APP_COGGING_H
#define ZEROFORM_FOC_APP_COGGING_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "app_flash.h"

//==============================================================================
// 齿槽校准配置宏定义
//==============================================================================

/**
 * @brief 校准速度 (rad/s)
 * @note  低速旋转以获取准确的齿槽力矩数据
 */
#define COGGING_CALIB_VELOCITY      5.0f

/**
 * @brief 校准电流限制 (A)
 */
#define COGGING_CALIB_CURRENT_LIMIT 2.0f

/**
 * @brief 校准超时时间 (ms)
 */
#define COGGING_CALIB_TIMEOUT       60000

/**
 * @brief 补偿表大小 (与Flash中定义一致)
 */
#define COGGING_TABLE_SIZE_LOCAL    APP_COGGING_TABLE_SIZE

//==============================================================================
// 齿槽校准状态
//==============================================================================
typedef enum {
    COGGING_STATE_IDLE = 0,         // 空闲
    COGGING_STATE_FORWARD,          // 正向旋转采集
    COGGING_STATE_BACKWARD,         // 反向旋转采集
    COGGING_STATE_PROCESSING,       // 数据处理中
    COGGING_STATE_COMPLETE,         // 校准完成
    COGGING_STATE_ERROR             // 校准错误
} Cogging_State_t;

//==============================================================================
// 齿槽校准数据结构
//==============================================================================
typedef struct {
    Cogging_State_t state;                          // 当前状态
    
    float forward_data[COGGING_TABLE_SIZE_LOCAL];   // 正向采集数据
    float backward_data[COGGING_TABLE_SIZE_LOCAL];  // 反向采集数据
    float compensation_table[COGGING_TABLE_SIZE_LOCAL]; // 最终补偿表
    
    uint32_t sample_count;                          // 采样计数
    float start_angle;                              // 起始角度
    float current_angle;                            // 当前角度
    float total_rotation;                           // 总旋转角度
    
    uint32_t start_time;                            // 开始时间
    uint8_t calibrated;                             // 是否已校准
    
    // 校准过程中的临时变量
    float angle_accumulator[COGGING_TABLE_SIZE_LOCAL];  // 角度累加器
    uint32_t sample_per_slot[COGGING_TABLE_SIZE_LOCAL]; // 每个槽的采样数
} Cogging_Calibration_t;

//==============================================================================
// 全局变量声明
//==============================================================================
extern Cogging_Calibration_t Cogging;

//==============================================================================
// 函数声明
//==============================================================================

/**
 * @brief 初始化齿槽补偿模块
 */
void Cogging_Init(void);

/**
 * @brief 开始齿槽校准
 * @return true: 成功开始, false: 无法开始 (可能正在校准中)
 */
bool Cogging_StartCalibration(void);

/**
 * @brief 停止齿槽校准
 */
void Cogging_StopCalibration(void);

/**
 * @brief 齿槽校准处理 (在主循环中调用)
 * @note 校准过程中会自动控制电机
 */
void Cogging_Process(void);

/**
 * @brief 获取当前校准状态
 * @return 校准状态
 */
Cogging_State_t Cogging_GetState(void);

/**
 * @brief 检查齿槽补偿是否已校准
 * @return true: 已校准, false: 未校准
 */
bool Cogging_IsCalibrated(void);

/**
 * @brief 获取齿槽补偿值
 * @param mechanical_angle 机械角度 (0 ~ 2π)
 * @return 补偿电流值 (A)
 */
float Cogging_GetCompensation(float mechanical_angle);

/**
 * @brief 启用/禁用齿槽补偿
 * @param enable true: 启用, false: 禁用
 */
void Cogging_SetEnabled(bool enable);

/**
 * @brief 检查齿槽补偿是否启用
 * @return true: 启用, false: 禁用
 */
bool Cogging_IsEnabled(void);

/**
 * @brief 从Flash加载齿槽补偿表
 * @return true: 加载成功, false: 加载失败
 */
bool Cogging_LoadFromFlash(void);

/**
 * @brief 保存齿槽补偿表到Flash (通过Flash模块)
 * @note 实际保存需要调用 Flash_SaveAll()
 */
void Cogging_PrepareForSave(void);

/**
 * @brief 获取校准进度 (0-100%)
 * @return 进度百分比
 */
uint8_t Cogging_GetProgress(void);

#endif //ZEROFORM_FOC_APP_COGGING_H
