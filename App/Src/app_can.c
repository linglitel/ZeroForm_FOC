//
// Created by linglitel on 2026/2/12.
// ZeroForm FOC CAN通信模块实现
//

#include "app_can.h"

#include <stdbool.h>
#include <string.h>
#include "app_flash.h"
#include "app_cogging.h"

//==============================================================================
// 设备信息
//==============================================================================
#define DEVICE_NAME         "ZeroForm_FOC"
#define BUILD_DATE          __DATE__

//==============================================================================
// 默认配置
//==============================================================================
#define DEFAULT_NODE_ID         0x01
#define DEFAULT_HEARTBEAT_MS    100

//==============================================================================
// 全局变量
//==============================================================================
CAN_Config_t CAN_Config = {
    .node_id = DEFAULT_NODE_ID,
    .motor_pairs = 11,
    .motor_direction = -1,
    .vbus_nominal = 12.0f,
    .heartbeat_period = DEFAULT_HEARTBEAT_MS,
};

CAN_Status_t CAN_Status = {
    .initialized = false,
    .enabled = false,
    .error_code = CAN_ERR_OK,
    .status_flags = 0,
    .last_heartbeat = 0,
    .rx_count = 0,
    .tx_count = 0,
    .error_count = 0,
};

//==============================================================================
// 私有变量
//==============================================================================
static FDCAN_HandleTypeDef *hfdcan_ptr = NULL;
static FDCAN_TxHeaderTypeDef TxHeader;
static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[64];
static uint8_t RxData[64];

//==============================================================================
// 私有函数声明
//==============================================================================
static HAL_StatusTypeDef CAN_ConfigFilter(void);
static void CAN_ProcessRxMessage(uint32_t can_id, uint8_t *data, uint32_t dlc);
static void CAN_HandleSetMode(uint8_t *data, uint32_t dlc);
static void CAN_HandleSetTarget(uint8_t *data, uint32_t dlc);
static void CAN_HandleSetPid(uint8_t *data, uint32_t dlc);
static void CAN_HandleGetStatus(uint8_t *data, uint32_t dlc);
static void CAN_HandleEmergency(uint8_t *data, uint32_t dlc);
static void CAN_HandleConfig(uint8_t *data, uint32_t dlc);
static void CAN_HandleCalibrate(uint8_t *data, uint32_t dlc);
static void CAN_HandleBroadcast(uint8_t *data, uint32_t dlc);
static uint32_t CAN_DlcToBytes(uint32_t dlc);
static uint32_t CAN_BytesToDlc(uint32_t bytes);
static HAL_StatusTypeDef CAN_Transmit(uint32_t can_id, uint8_t *data, uint32_t len);
static uint8_t CAN_GetStatusFlags(void);

//==============================================================================
// 公共函数实现
//==============================================================================

/**
 * @brief 初始化CAN模块
 */
HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan == NULL) {
        return HAL_ERROR;
    }
    
    hfdcan_ptr = hfdcan;
    
    // 配置滤波器
    if (CAN_ConfigFilter() != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 配置发送头
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;  // 启用BRS
    TxHeader.FDFormat = FDCAN_FD_CAN;       // 使用CAN FD格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    CAN_Status.initialized = true;
    
    return HAL_OK;
}

/**
 * @brief 启动CAN通信
 */
HAL_StatusTypeDef CAN_Start(void) {
    if (!CAN_Status.initialized || hfdcan_ptr == NULL) {
        return HAL_ERROR;
    }
    
    // 启动FDCAN
    if (HAL_FDCAN_Start(hfdcan_ptr) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 启用接收中断
    if (HAL_FDCAN_ActivateNotification(hfdcan_ptr, 
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        return HAL_ERROR;
    }
    
    CAN_Status.enabled = true;
    CAN_Status.last_heartbeat = HAL_GetTick();
    
    return HAL_OK;
}

/**
 * @brief 停止CAN通信
 */
HAL_StatusTypeDef CAN_Stop(void) {
    if (hfdcan_ptr == NULL) {
        return HAL_ERROR;
    }
    
    HAL_FDCAN_DeactivateNotification(hfdcan_ptr, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    HAL_FDCAN_Stop(hfdcan_ptr);
    
    CAN_Status.enabled = false;
    
    return HAL_OK;
}

/**
 * @brief CAN主循环处理
 */
void CAN_Process(void) {
    if (!CAN_Status.enabled) {
        return;
    }
    
    uint32_t current_tick = HAL_GetTick();
    
    // 心跳包发送
    if (current_tick - CAN_Status.last_heartbeat >= CAN_Config.heartbeat_period) {
        CAN_SendHeartbeat();
        CAN_Status.last_heartbeat = current_tick;
    }
}

/**
 * @brief 发送心跳包
 */
HAL_StatusTypeDef CAN_SendHeartbeat(void) {
    CAN_HeartbeatData_t heartbeat;
    
    // 填充心跳数据
    heartbeat.mode = (uint8_t)FOC.mode;
    heartbeat.status_flags = CAN_GetStatusFlags();
    heartbeat.error_code = CAN_Status.error_code;
    heartbeat.reserved = 0;
    heartbeat.position = FOC.mechanical_angle * (float)FOC.direction;
    heartbeat.velocity = FOC.mechanical_velocity;
    heartbeat.iq_current = FOC.Iq;
    heartbeat.id_current = FOC.Id;
    heartbeat.vbus = FOC.Vbus;
    heartbeat.temperature = 0.0f;  // 预留
    heartbeat.timestamp = HAL_GetTick();
    
    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_HEARTBEAT, CAN_Config.node_id);
    
    return CAN_Transmit(can_id, (uint8_t*)&heartbeat, sizeof(CAN_HeartbeatData_t));
}

/**
 * @brief 发送ACK响应
 */
HAL_StatusTypeDef CAN_SendAck(uint8_t cmd_code, uint8_t result, uint16_t error_detail, uint32_t data) {
    CAN_AckData_t ack;
    
    ack.cmd_code = cmd_code;
    ack.result = result;
    ack.error_detail = error_detail;
    ack.data = data;
    
    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_ACK, CAN_Config.node_id);
    
    return CAN_Transmit(can_id, (uint8_t*)&ack, sizeof(CAN_AckData_t));
}

/**
 * @brief 发送状态响应
 */
HAL_StatusTypeDef CAN_SendStatusResponse(uint8_t request_type) {
    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_STATUS_RESP, CAN_Config.node_id);
    
    switch (request_type) {
        case CAN_STATUS_BASIC: {
            // 发送基本状态 (与心跳包相同)
            CAN_HeartbeatData_t status;
            status.mode = (uint8_t)FOC.mode;
            status.status_flags = CAN_GetStatusFlags();
            status.error_code = CAN_Status.error_code;
            status.reserved = 0;
            status.position = FOC.mechanical_angle * (float)FOC.direction;
            status.velocity = FOC.mechanical_velocity;
            status.iq_current = FOC.Iq;
            status.id_current = FOC.Id;
            status.vbus = FOC.Vbus;
            status.temperature = 0.0f;
            status.timestamp = HAL_GetTick();
            return CAN_Transmit(can_id, (uint8_t*)&status, sizeof(CAN_HeartbeatData_t));
        }
        
        case CAN_STATUS_PID_CURRENT: {
            CAN_PidResponseData_t pid;
            pid.response_type = CAN_STATUS_PID_CURRENT;
            memset(pid.reserved, 0, sizeof(pid.reserved));
            pid.kp = FOC.pid_iq.Kp;
            pid.ki = FOC.pid_iq.Ki;
            pid.kd = FOC.pid_iq.Kd;
            pid.output_limit = FOC.pid_iq.output_limit;
            pid.integral_limit = FOC.pid_iq.integral_limit;
            return CAN_Transmit(can_id, (uint8_t*)&pid, sizeof(CAN_PidResponseData_t));
        }
        
        case CAN_STATUS_PID_VELOCITY: {
            CAN_PidResponseData_t pid;
            pid.response_type = CAN_STATUS_PID_VELOCITY;
            memset(pid.reserved, 0, sizeof(pid.reserved));
            pid.kp = FOC.pid_velocity.Kp;
            pid.ki = FOC.pid_velocity.Ki;
            pid.kd = FOC.pid_velocity.Kd;
            pid.output_limit = FOC.pid_velocity.output_limit;
            pid.integral_limit = FOC.pid_velocity.integral_limit;
            return CAN_Transmit(can_id, (uint8_t*)&pid, sizeof(CAN_PidResponseData_t));
        }
        
        case CAN_STATUS_PID_POSITION: {
            CAN_PidResponseData_t pid;
            pid.response_type = CAN_STATUS_PID_POSITION;
            memset(pid.reserved, 0, sizeof(pid.reserved));
            pid.kp = FOC.pid_position.Kp;
            pid.ki = FOC.pid_position.Ki;
            pid.kd = FOC.pid_position.Kd;
            pid.output_limit = FOC.pid_position.output_limit;
            pid.integral_limit = FOC.pid_position.integral_limit;
            return CAN_Transmit(can_id, (uint8_t*)&pid, sizeof(CAN_PidResponseData_t));
        }
        
        case CAN_STATUS_CONFIG: {
            CAN_ConfigResponseData_t config;
            config.response_type = CAN_STATUS_CONFIG;
            config.node_id = CAN_Config.node_id;
            config.motor_pairs = CAN_Config.motor_pairs;
            config.motor_direction = CAN_Config.motor_direction;
            config.vbus_nominal = CAN_Config.vbus_nominal;
            config.heartbeat_period = CAN_Config.heartbeat_period;
            config.encoder_offset = FOC.electrical_angle_offset;
            return CAN_Transmit(can_id, (uint8_t*)&config, sizeof(CAN_ConfigResponseData_t));
        }
        
        case CAN_STATUS_VERSION: {
            CAN_VersionResponseData_t version;
            version.response_type = CAN_STATUS_VERSION;
            version.version_major = CAN_PROTOCOL_VERSION_MAJOR;
            version.version_minor = CAN_PROTOCOL_VERSION_MINOR;
            version.version_patch = CAN_PROTOCOL_VERSION_PATCH;
            strncpy(version.build_date, BUILD_DATE, sizeof(version.build_date) - 1);
            version.build_date[sizeof(version.build_date) - 1] = '\0';
            strncpy(version.device_name, DEVICE_NAME, sizeof(version.device_name) - 1);
            version.device_name[sizeof(version.device_name) - 1] = '\0';
            return CAN_Transmit(can_id, (uint8_t*)&version, sizeof(CAN_VersionResponseData_t));
        }
        
        default:
            return HAL_ERROR;
    }
}

/**
 * @brief 发送紧急停止
 */
HAL_StatusTypeDef CAN_SendEmergency(uint8_t action, uint8_t reason) {
    CAN_EmergencyData_t emergency;
    
    emergency.action = action;
    emergency.reason = reason;
    memset(emergency.reserved, 0, sizeof(emergency.reserved));
    
    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_EMERGENCY, CAN_Config.node_id);
    
    return CAN_Transmit(can_id, (uint8_t*)&emergency, sizeof(CAN_EmergencyData_t));
}

/**
 * @brief FDCAN接收回调
 */
void CAN_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan != hfdcan_ptr) {
        return;
    }
    
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        // 从FIFO0读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            CAN_Status.rx_count++;
            uint32_t dlc_bytes = CAN_DlcToBytes(RxHeader.DataLength);
            CAN_ProcessRxMessage(RxHeader.Identifier, RxData, dlc_bytes);
        }
    }
}

/**
 * @brief 设置节点ID
 */
CAN_ErrorCode_t CAN_SetNodeId(uint8_t node_id) {
    if (node_id < 1 || node_id > 127) {
        return CAN_ERR_INVALID_PARAM;
    }
    
    CAN_Config.node_id = node_id;
    
    // 重新配置滤波器
    if (CAN_Status.enabled) {
        CAN_Stop();
        CAN_ConfigFilter();
        CAN_Start();
    } else {
        CAN_ConfigFilter();
    }
    
    return CAN_ERR_OK;
}

/**
 * @brief 获取当前节点ID
 */
uint8_t CAN_GetNodeId(void) {
    return CAN_Config.node_id;
}

/**
 * @brief 保存配置到Flash (预留)
 */
CAN_ErrorCode_t CAN_SaveConfig(void) {
    // TODO: 实现Flash存储
    return CAN_ERR_OK;
}

/**
 * @brief 从Flash加载配置 (预留)
 */
CAN_ErrorCode_t CAN_LoadConfig(void) {
    // TODO: 实现Flash读取
    return CAN_ERR_OK;
}

/**
 * @brief 恢复默认配置
 */
void CAN_ResetConfig(void) {
    CAN_Config.node_id = DEFAULT_NODE_ID;
    CAN_Config.motor_pairs = 11;
    CAN_Config.motor_direction = -1;
    CAN_Config.vbus_nominal = 12.0f;
    CAN_Config.heartbeat_period = DEFAULT_HEARTBEAT_MS;
}

//==============================================================================
// 私有函数实现
//==============================================================================

/**
 * @brief 配置CAN滤波器
 */
static HAL_StatusTypeDef CAN_ConfigFilter(void) {
    FDCAN_FilterTypeDef filter;
    
    // 滤波器1: 接收本节点的所有命令 (CMD 0x1-0x8)
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_Config.node_id;      // 匹配节点ID
    filter.FilterID2 = 0x07F;                    // 掩码: 只检查低7位 (节点ID)
    
    if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 滤波器2: 接收广播命令 (ID = 0x7FF)
    filter.FilterIndex = 1;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterID1 = CAN_BROADCAST_ID;
    filter.FilterID2 = CAN_BROADCAST_ID;
    
    if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 配置全局滤波器: 拒绝不匹配的帧
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan_ptr, 
            FDCAN_REJECT, FDCAN_REJECT, 
            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief 处理接收到的CAN消息
 */
static void CAN_ProcessRxMessage(uint32_t can_id, uint8_t *data, uint32_t dlc) {
    // 检查是否是广播消息
    if (can_id == CAN_BROADCAST_ID) {
        CAN_HandleBroadcast(data, dlc);
        return;
    }
    
    // 提取命令码和节点ID
    uint8_t cmd = CAN_GET_CMD(can_id);
    uint8_t node_id = CAN_GET_NODE_ID(can_id);
    
    // 检查节点ID是否匹配
    if (node_id != CAN_Config.node_id) {
        return;
    }
    
    // 根据命令码处理
    switch (cmd) {
        case CAN_CMD_SET_MODE:
            CAN_HandleSetMode(data, dlc);
            break;
            
        case CAN_CMD_SET_TARGET:
            CAN_HandleSetTarget(data, dlc);
            break;
            
        case CAN_CMD_SET_PID:
            CAN_HandleSetPid(data, dlc);
            break;
            
        case CAN_CMD_GET_STATUS:
            CAN_HandleGetStatus(data, dlc);
            break;
            
        case CAN_CMD_EMERGENCY:
            CAN_HandleEmergency(data, dlc);
            break;
            
        case CAN_CMD_CONFIG:
            CAN_HandleConfig(data, dlc);
            break;
            
        case CAN_CMD_CALIBRATE:
            CAN_HandleCalibrate(data, dlc);
            break;
            
        default:
            CAN_SendAck(cmd, CAN_ERR_UNKNOWN_CMD, 0, 0);
            break;
    }
}

/**
 * @brief 处理设置模式命令
 */
static void CAN_HandleSetMode(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_SetModeData_t)) {
        CAN_SendAck(CAN_CMD_SET_MODE, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }
    
    CAN_SetModeData_t *mode_data = (CAN_SetModeData_t*)data;
    
    switch (mode_data->mode) {
        case CAN_MODE_IDLE:
            FOC.mode = IDLE;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            break;
            
        case CAN_MODE_CURRENT:
            FOC.mode = Current;
            break;
            
        case CAN_MODE_VELOCITY:
            FOC.mode = Velocity;
            FOC.velocity_target = 0.0f;
            break;
            
        case CAN_MODE_POSITION:
            FOC.mode = Position;
            FOC.position_target = FOC.mechanical_angle * (float)FOC.direction;
            break;
            
        default:
            CAN_SendAck(CAN_CMD_SET_MODE, CAN_ERR_INVALID_PARAM, mode_data->mode, 0);
            return;
    }
    
    CAN_SendAck(CAN_CMD_SET_MODE, CAN_ERR_OK, 0, mode_data->mode);
}

/**
 * @brief 处理设置目标值命令
 */
static void CAN_HandleSetTarget(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_SetTargetData_t)) {
        CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }
    
    CAN_SetTargetData_t *target_data = (CAN_SetTargetData_t*)data;
    
    switch (target_data->target_type) {
        case CAN_TARGET_VELOCITY:
            if (FOC.mode != Velocity) {
                CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_WRONG_MODE, FOC.mode, 0);
                return;
            }
            FOC.velocity_target = target_data->target_value;
            break;
            
        case CAN_TARGET_POSITION_ABS:
            if (FOC.mode != Position) {
                CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_WRONG_MODE, FOC.mode, 0);
                return;
            }
            FOC.position_target = target_data->target_value;
            break;
            
        case CAN_TARGET_POSITION_REL:
            if (FOC.mode != Position) {
                CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_WRONG_MODE, FOC.mode, 0);
                return;
            }
            FOC.position_target = FOC.mechanical_angle * (float)FOC.direction + target_data->target_value;
            break;
            
        case CAN_TARGET_CURRENT:
            if (FOC.mode != Current) {
                CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_WRONG_MODE, FOC.mode, 0);
                return;
            }
            FOC.Iq_target = target_data->target_value;
            break;
            
        default:
            CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_INVALID_PARAM, target_data->target_type, 0);
            return;
    }
    
    CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_OK, 0, 0);
}

/**
 * @brief 处理设置PID参数命令
 */
static void CAN_HandleSetPid(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_SetPidData_t)) {
        CAN_SendAck(CAN_CMD_SET_PID, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }
    
    CAN_SetPidData_t *pid_data = (CAN_SetPidData_t*)data;
    PID_Controller *pid = NULL;
    
    switch (pid_data->pid_type) {
        case CAN_PID_CURRENT_IQ:
            pid = &FOC.pid_iq;
            break;
            
        case CAN_PID_CURRENT_ID:
            pid = &FOC.pid_id;
            break;
            
        case CAN_PID_VELOCITY:
            pid = &FOC.pid_velocity;
            break;
            
        case CAN_PID_POSITION:
            pid = &FOC.pid_position;
            break;
            
        default:
            CAN_SendAck(CAN_CMD_SET_PID, CAN_ERR_INVALID_PARAM, pid_data->pid_type, 0);
            return;
    }
    
    // 更新PID参数
    pid->Kp = pid_data->kp;
    pid->Ki = pid_data->ki;
    pid->Kd = pid_data->kd;
    pid->output_limit = pid_data->output_limit;
    pid->integral_limit = pid_data->integral_limit;
    
    // 重置积分项
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    
    CAN_SendAck(CAN_CMD_SET_PID, CAN_ERR_OK, 0, pid_data->pid_type);
}

/**
 * @brief 处理获取状态命令
 */
static void CAN_HandleGetStatus(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_GetStatusData_t)) {
        CAN_SendAck(CAN_CMD_GET_STATUS, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }
    
    CAN_GetStatusData_t *status_req = (CAN_GetStatusData_t*)data;
    
    if (CAN_SendStatusResponse(status_req->request_type) != HAL_OK) {
        CAN_SendAck(CAN_CMD_GET_STATUS, CAN_ERR_INVALID_PARAM, status_req->request_type, 0);
    }
}

/**
 * @brief 处理紧急停止命令
 */
static void CAN_HandleEmergency(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_EmergencyData_t)) {
        return;  // 紧急停止不发送ACK
    }
    
    CAN_EmergencyData_t *emergency = (CAN_EmergencyData_t*)data;
    
    switch (emergency->action) {
        case CAN_EMERGENCY_STOP:
            FOC.mode = IDLE;
            FOC.Iq_target = 0.0f;
            FOC.velocity_target = 0.0f;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            CAN_Status.status_flags |= CAN_STATUS_FLAG_FAULT;
            break;
            
        case CAN_EMERGENCY_RECOVER:
            CAN_Status.status_flags &= ~CAN_STATUS_FLAG_FAULT;
            CAN_Status.error_code = CAN_ERR_OK;
            break;
            
        case CAN_EMERGENCY_RESET:
            NVIC_SystemReset();
            break;
            
        default:
            break;
    }
    
    // 紧急停止命令发送ACK
    CAN_SendAck(CAN_CMD_EMERGENCY, CAN_ERR_OK, 0, emergency->action);
}

/**
 * @brief 处理配置命令
 */
static void CAN_HandleConfig(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_ConfigData_t)) {
        CAN_SendAck(CAN_CMD_CONFIG, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }
    
    CAN_ConfigData_t *config = (CAN_ConfigData_t*)data;
    CAN_ErrorCode_t result = CAN_ERR_OK;
    
    switch (config->config_cmd) {
        case CAN_CONFIG_SET_NODE_ID:
            result = CAN_SetNodeId((uint8_t)config->value_int);
            break;
            
        case CAN_CONFIG_SET_MOTOR_PAIRS:
            if (config->value_int > 0 && config->value_int < 256) {
                CAN_Config.motor_pairs = (uint8_t)config->value_int;
                FOC.pairs = CAN_Config.motor_pairs;
            } else {
                result = CAN_ERR_INVALID_PARAM;
            }
            break;
            
        case CAN_CONFIG_SET_MOTOR_DIR:
            if (config->value_int == 1 || config->value_int == (uint32_t)-1) {
                CAN_Config.motor_direction = (int8_t)config->value_int;
                FOC.direction = CAN_Config.motor_direction;
            } else {
                result = CAN_ERR_INVALID_PARAM;
            }
            break;
            
        case CAN_CONFIG_SET_VBUS:
            if (config->value_float > 0.0f && config->value_float < 100.0f) {
                CAN_Config.vbus_nominal = config->value_float;
                FOC.Vbus = CAN_Config.vbus_nominal;
            } else {
                result = CAN_ERR_INVALID_PARAM;
            }
            break;
            
        case CAN_CONFIG_SET_HEARTBEAT:
            if (config->value_int >= 10 && config->value_int <= 10000) {
                CAN_Config.heartbeat_period = config->value_int;
            } else {
                result = CAN_ERR_INVALID_PARAM;
            }
            break;
            
        case CAN_CONFIG_SAVE:
            result = CAN_SaveConfig();
            break;
            
        case CAN_CONFIG_LOAD:
            result = CAN_LoadConfig();
            break;
            
        case CAN_CONFIG_RESET:
            CAN_ResetConfig();
            break;
            
        default:
            result = CAN_ERR_UNKNOWN_CMD;
            break;
    }
    
    CAN_SendAck(CAN_CMD_CONFIG, result, 0, config->config_cmd);
}

/**
 * @brief 处理校准命令
 */
static void CAN_HandleCalibrate(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_CalibrateData_t)) {
        CAN_SendAck(CAN_CMD_CALIBRATE, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }
    
    CAN_CalibrateData_t *calib = (CAN_CalibrateData_t*)data;
    CAN_ErrorCode_t result = CAN_ERR_OK;
    
    switch (calib->calib_type) {
        case CAN_CALIB_ENCODER_ALIGN:
            // 执行编码器对齐校准
            FOC.mode = IDLE;
            FOC_AlignSensor(calib->param1 > 0 ? calib->param1 : 5.0f);
            Flash_SetElectricalOffset(FOC.electrical_angle_offset);
            CAN_Status.status_flags |= CAN_STATUS_FLAG_CALIBRATED;
            break;
            
        case CAN_CALIB_CURRENT_OFFSET:
            // 电流偏置校准 (预留)
            break;
            
        case CAN_CALIB_SET_ZERO:
            // 设置当前位置为零点 (预留)
            break;
            
        case CAN_CALIB_COGGING:  // 齿槽校准
            if (Cogging_GetState() != COGGING_STATE_IDLE && 
                Cogging_GetState() != COGGING_STATE_COMPLETE &&
                Cogging_GetState() != COGGING_STATE_ERROR) {
                result = CAN_ERR_BUSY;
            } else {
                if (!Cogging_StartCalibration()) {
                    result = CAN_ERR_BUSY;
                }
            }
            break;
            
        case CAN_CALIB_SAVE:  // 保存校准数据到Flash
            Cogging_PrepareForSave();
            if (Flash_SaveAll() != HAL_OK) {
                result = CAN_ERR_FLASH;
            }
            break;
            
        default:
            result = CAN_ERR_INVALID_PARAM;
            break;
    }
    
    CAN_SendAck(CAN_CMD_CALIBRATE, result, 0, calib->calib_type);
}

/**
 * @brief 处理广播命令
 */
static void CAN_HandleBroadcast(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_BroadcastData_t)) {
        return;  // 广播命令不发送ACK
    }
    
    CAN_BroadcastData_t *broadcast = (CAN_BroadcastData_t*)data;
    
    switch (broadcast->broadcast_cmd) {
        case CAN_BROADCAST_EMERGENCY_STOP:
            FOC.mode = IDLE;
            FOC.Iq_target = 0.0f;
            FOC.velocity_target = 0.0f;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            CAN_Status.status_flags |= CAN_STATUS_FLAG_FAULT;
            break;
            
        case CAN_BROADCAST_SYNC_TIME:
            // 时间同步 (预留)
            break;
            
        case CAN_BROADCAST_ALL_IDLE:
            FOC.mode = IDLE;
            FOC.Iq_target = 0.0f;
            FOC.velocity_target = 0.0f;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            break;
            
        default:
            break;
    }
    // 广播命令不发送ACK
}

/**
 * @brief DLC转换为字节数 (CAN FD)
 */
static uint32_t CAN_DlcToBytes(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}

/**
 * @brief 字节数转换为DLC (CAN FD)
 */
static uint32_t CAN_BytesToDlc(uint32_t bytes) {
    if (bytes <= 8) {
        return bytes << 16;  // FDCAN_DLC_BYTES_x = x << 16 for 0-8
    } else if (bytes <= 12) {
        return FDCAN_DLC_BYTES_12;
    } else if (bytes <= 16) {
        return FDCAN_DLC_BYTES_16;
    } else if (bytes <= 20) {
        return FDCAN_DLC_BYTES_20;
    } else if (bytes <= 24) {
        return FDCAN_DLC_BYTES_24;
    } else if (bytes <= 32) {
        return FDCAN_DLC_BYTES_32;
    } else if (bytes <= 48) {
        return FDCAN_DLC_BYTES_48;
    } else {
        return FDCAN_DLC_BYTES_64;
    }
}

/**
 * @brief 发送CAN消息
 */
static HAL_StatusTypeDef CAN_Transmit(uint32_t can_id, uint8_t *data, uint32_t len) {
    if (hfdcan_ptr == NULL || !CAN_Status.enabled) {
        return HAL_ERROR;
    }
    
    // 设置发送头
    TxHeader.Identifier = can_id;
    TxHeader.DataLength = CAN_BytesToDlc(len);
    
    // 复制数据
    memcpy(TxData, data, len);
    
    // 发送消息
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_ptr, &TxHeader, TxData);
    
    if (status == HAL_OK) {
        CAN_Status.tx_count++;
    } else {
        CAN_Status.error_count++;
    }
    
    return status;
}

/**
 * @brief 获取状态标志
 */
static uint8_t CAN_GetStatusFlags(void) {
    uint8_t flags = 0;
    
    // 检查是否运行中
    if (FOC.mode != IDLE) {
        flags |= CAN_STATUS_FLAG_RUNNING;
    }
    
    // 检查是否有故障
    if (CAN_Status.error_code != CAN_ERR_OK) {
        flags |= CAN_STATUS_FLAG_FAULT;
    }
    
    // 检查是否已校准
    if (FOC.electrical_angle_offset != 0.0f) {
        flags |= CAN_STATUS_FLAG_CALIBRATED;
    }
    
    return flags;
}

/**
 * @brief FDCAN RxFifo0回调 (需要在stm32g4xx_it.c中调用)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    CAN_RxCallback(hfdcan, RxFifo0ITs);
}
