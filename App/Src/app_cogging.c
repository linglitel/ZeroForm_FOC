//
// Created by linglitel on 2026/2/13.
// ZeroForm FOC 齿槽补偿模块实现
//

#include "app_cogging.h"
#include "app_foc.h"
#include "app_utils.h"
#include <string.h>
#include <math.h>

//==============================================================================
// 全局变量
//==============================================================================
Cogging_Calibration_t Cogging;

// 齿槽补偿启用标志
static bool cogging_enabled = false;


//==============================================================================
// 私有函数声明
//==============================================================================
static void Cogging_ProcessForward(void);

static void Cogging_ProcessBackward(void);

static void Cogging_ProcessData(void);

static uint32_t Cogging_AngleToIndex(float angle);

//==============================================================================
// 公共函数实现
//==============================================================================

/**
 * @brief 初始化齿槽补偿模块
 */
void Cogging_Init(void) {
    memset(&Cogging, 0, sizeof(Cogging_Calibration_t));
    Cogging.state = COGGING_STATE_IDLE;
    Cogging.calibrated = 0;
    cogging_enabled = false;
}

/**
 * @brief 开始齿槽校准
 */
bool Cogging_StartCalibration(void) {
    // 检查是否已在校准中
    if (Cogging.state != COGGING_STATE_IDLE &&
        Cogging.state != COGGING_STATE_COMPLETE &&
        Cogging.state != COGGING_STATE_ERROR) {
        return false;
    }

    // 重置校准数据
    memset(Cogging.forward_data, 0, sizeof(Cogging.forward_data));
    memset(Cogging.backward_data, 0, sizeof(Cogging.backward_data));
    memset(Cogging.compensation_table, 0, sizeof(Cogging.compensation_table));
    memset(Cogging.angle_accumulator, 0, sizeof(Cogging.angle_accumulator));
    memset(Cogging.sample_per_slot, 0, sizeof(Cogging.sample_per_slot));

    Cogging.sample_count = 0;
    Cogging.total_rotation = 0.0f;
    Cogging.start_time = HAL_GetTick();

    // 记录起始角度
    Cogging.start_angle = FOC.mechanical_angle;
    Cogging.current_angle = Cogging.start_angle;

    // 切换到速度模式，设置低速正向旋转
    FOC.mode = Velocity;
    FOC.velocity_target = COGGING_CALIB_VELOCITY;

    Cogging.state = COGGING_STATE_FORWARD;

    return true;
}

/**
 * @brief 停止齿槽校准
 */
void Cogging_StopCalibration(void) {
    FOC.mode = IDLE;
    FOC.velocity_target = 0.0f;
    FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);

    if (Cogging.state != COGGING_STATE_COMPLETE) {
        Cogging.state = COGGING_STATE_ERROR;
    }
}

/**
 * @brief 齿槽校准处理
 */
void Cogging_Process(void) {
    // 检查超时
    if (Cogging.state != COGGING_STATE_IDLE &&
        Cogging.state != COGGING_STATE_COMPLETE &&
        Cogging.state != COGGING_STATE_ERROR) {
        if (HAL_GetTick() - Cogging.start_time > COGGING_CALIB_TIMEOUT) {
            Cogging_StopCalibration();
            Cogging.state = COGGING_STATE_ERROR;
            return;
        }
    }

    switch (Cogging.state) {
        case COGGING_STATE_FORWARD:
            Cogging_ProcessForward();
            break;

        case COGGING_STATE_BACKWARD:
            Cogging_ProcessBackward();
            break;

        case COGGING_STATE_PROCESSING:
            Cogging_ProcessData();
            break;

        default:
            break;
    }
}

/**
 * @brief 获取当前校准状态
 */
Cogging_State_t Cogging_GetState(void) {
    return Cogging.state;
}

/**
 * @brief 检查齿槽补偿是否已校准
 */
bool Cogging_IsCalibrated(void) {
    return (Cogging.calibrated != 0);
}

/**
 * @brief 获取齿槽补偿值
 */
float Cogging_GetCompensation(float mechanical_angle) {
    if (!cogging_enabled || !Cogging.calibrated) {
        return 0.0f;
    }

    // 将角度归一化到 [0, 2π)
    float normalized_angle = _normalizeAngle(mechanical_angle);
    if (normalized_angle < 0) {
        normalized_angle += _2PI;
    }

    // 计算索引
    uint32_t index = Cogging_AngleToIndex(normalized_angle);

    // 线性插值
    uint32_t next_index = (index + 1) % COGGING_TABLE_SIZE_LOCAL;
    float angle_per_slot = _2PI / (float) COGGING_TABLE_SIZE_LOCAL;
    float slot_start_angle = (float) index * angle_per_slot;
    float fraction = (normalized_angle - slot_start_angle) / angle_per_slot;

    // 限制fraction范围
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    float compensation = Cogging.compensation_table[index] * (1.0f - fraction) +
                         Cogging.compensation_table[next_index] * fraction;

    return compensation;
}

/**
 * @brief 启用/禁用齿槽补偿
 */
void Cogging_SetEnabled(bool enable) {
    cogging_enabled = enable && Cogging.calibrated;
}

/**
 * @brief 检查齿槽补偿是否启用
 */
bool Cogging_IsEnabled(void) {
    return cogging_enabled;
}

/**
 * @brief 从Flash加载齿槽补偿表
 */
bool Cogging_LoadFromFlash(void) {
    if (!Flash_IsCoggingValid()) {
        return false;
    }

    const float *table = Flash_GetCoggingTable();
    memcpy(Cogging.compensation_table, table, sizeof(Cogging.compensation_table));
    Cogging.calibrated = 1;

    return true;
}

/**
 * @brief 保存齿槽补偿表到Flash
 */
void Cogging_PrepareForSave(void) {
    if (Cogging.calibrated) {
        Flash_SetCoggingTable(Cogging.compensation_table, COGGING_TABLE_SIZE_LOCAL);
        Flash_SetCoggingCalibrated(true);
    }
}

/**
 * @brief 获取校准进度
 */
uint8_t Cogging_GetProgress(void) {
    switch (Cogging.state) {
        case COGGING_STATE_IDLE:
            return 0;

        case COGGING_STATE_FORWARD:
            // 正向旋转占50%
            return (uint8_t) (Cogging.total_rotation / _2PI * 50.0f);

        case COGGING_STATE_BACKWARD:
            // 反向旋转占50%
            return 50 + (uint8_t) (Cogging.total_rotation / _2PI * 50.0f);

        case COGGING_STATE_PROCESSING:
            return 95;

        case COGGING_STATE_COMPLETE:
            return 100;

        case COGGING_STATE_ERROR:
            return 0;

        default:
            return 0;
    }
}

//==============================================================================
// 私有函数实现
//==============================================================================

/**
 * @brief 角度转换为索引
 */
static uint32_t Cogging_AngleToIndex(float angle) {
    // 将角度归一化到 [0, 2π)
    while (angle < 0) angle += _2PI;
    while (angle >= _2PI) angle -= _2PI;

    // 计算索引
    uint32_t index = (uint32_t) (angle / _2PI * (float) COGGING_TABLE_SIZE_LOCAL);
    if (index >= COGGING_TABLE_SIZE_LOCAL) {
        index = COGGING_TABLE_SIZE_LOCAL - 1;
    }

    return index;
}

/**
 * @brief 处理正向旋转采集
 */
static void Cogging_ProcessForward(void) {
    // 获取当前角度和电流
    float current_angle = FOC.mechanical_angle;
    float iq_current = FOC.Iq;

    // 计算角度变化
    float delta_angle = current_angle - Cogging.current_angle;

    // 处理角度跨越
    if (delta_angle > 3.14159f) {
        delta_angle -= _2PI;
    } else if (delta_angle < -3.14159f) {
        delta_angle += _2PI;
    }

    Cogging.total_rotation += fabsf(delta_angle);
    Cogging.current_angle = current_angle;

    // 记录数据
    uint32_t index = Cogging_AngleToIndex(current_angle);
    Cogging.forward_data[index] += iq_current;
    Cogging.sample_per_slot[index]++;
    Cogging.sample_count++;

    // 检查是否完成一圈
    if (Cogging.total_rotation >= _2PI) {
        // 计算平均值
        for (uint32_t i = 0; i < COGGING_TABLE_SIZE_LOCAL; i++) {
            if (Cogging.sample_per_slot[i] > 0) {
                Cogging.forward_data[i] /= (float) Cogging.sample_per_slot[i];
            }
        }

        // 重置采样计数，准备反向采集
        memset(Cogging.sample_per_slot, 0, sizeof(Cogging.sample_per_slot));
        Cogging.total_rotation = 0.0f;
        Cogging.sample_count = 0;

        // 切换到反向旋转
        FOC.velocity_target = -COGGING_CALIB_VELOCITY;
        Cogging.state = COGGING_STATE_BACKWARD;
    }
}

/**
 * @brief 处理反向旋转采集
 */
static void Cogging_ProcessBackward(void) {
    // 获取当前角度和电流
    float current_angle = FOC.mechanical_angle;
    float iq_current = FOC.Iq;

    // 计算角度变化
    float delta_angle = current_angle - Cogging.current_angle;

    // 处理角度跨越
    if (delta_angle > 3.14159f) {
        delta_angle -= _2PI;
    } else if (delta_angle < -3.14159f) {
        delta_angle += _2PI;
    }

    Cogging.total_rotation += fabsf(delta_angle);
    Cogging.current_angle = current_angle;

    // 记录数据
    uint32_t index = Cogging_AngleToIndex(current_angle);
    Cogging.backward_data[index] += iq_current;
    Cogging.sample_per_slot[index]++;
    Cogging.sample_count++;

    // 检查是否完成一圈
    if (Cogging.total_rotation >= _2PI) {
        // 计算平均值
        for (uint32_t i = 0; i < COGGING_TABLE_SIZE_LOCAL; i++) {
            if (Cogging.sample_per_slot[i] > 0) {
                Cogging.backward_data[i] /= (float) Cogging.sample_per_slot[i];
            }
        }

        // 停止电机
        FOC.mode = IDLE;
        FOC.velocity_target = 0.0f;
        FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);

        // 进入数据处理阶段
        Cogging.state = COGGING_STATE_PROCESSING;
    }
}

/**
 * @brief 处理采集数据，生成补偿表
 */
static void Cogging_ProcessData(void) {
    // 计算补偿表：取正向和反向的平均值，然后取反
    // 齿槽力矩 = (正向电流 + 反向电流) / 2
    // 补偿电流 = -齿槽力矩

    float sum = 0.0f;

    for (uint32_t i = 0; i < COGGING_TABLE_SIZE_LOCAL; i++) {
        float cogging_torque = (Cogging.forward_data[i] + Cogging.backward_data[i]) / 2.0f;
        Cogging.compensation_table[i] = -cogging_torque;
        sum += Cogging.compensation_table[i];
    }

    // 去除直流偏置
    float dc_offset = sum / (float) COGGING_TABLE_SIZE_LOCAL;
    for (uint32_t i = 0; i < COGGING_TABLE_SIZE_LOCAL; i++) {
        Cogging.compensation_table[i] -= dc_offset;
    }

    // 标记校准完成
    Cogging.calibrated = 1;
    Cogging.state = COGGING_STATE_COMPLETE;
    
    // 自动保存到Flash
    Cogging_PrepareForSave();
    Flash_SaveAll();
}
