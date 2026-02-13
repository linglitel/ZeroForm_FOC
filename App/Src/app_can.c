//
// Created by linglitel on 2026/2/12.
// ZeroForm FOC CAN通信模块实现 (精简版)
//

#include "app_can.h"
#include "app_flash.h"
#include <stdbool.h>
#include <string.h>

#define DEFAULT_NODE_ID         0x01
#define DEFAULT_HEARTBEAT_MS    100

CAN_Config_t CAN_Config = {
    .node_id = DEFAULT_NODE_ID,
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

static FDCAN_HandleTypeDef *hfdcan_ptr = NULL;
static FDCAN_TxHeaderTypeDef TxHeader;
static FDCAN_RxHeaderTypeDef RxHeader;
static uint8_t TxData[64];
static uint8_t RxData[64];

static HAL_StatusTypeDef CAN_ConfigFilter(void);

static void CAN_ProcessRxMessage(uint32_t can_id, uint8_t *data, uint32_t dlc);

static void CAN_HandleSetMode(uint8_t *data, uint32_t dlc);

static void CAN_HandleSetTarget(uint8_t *data, uint32_t dlc);


static void CAN_HandleGetStatus(uint8_t *data, uint32_t dlc);

static void CAN_HandleEmergency(uint8_t *data, uint32_t dlc);

static void CAN_HandleConfig(uint8_t *data, uint32_t dlc);

static void CAN_HandleBroadcast(uint8_t *data, uint32_t dlc);

static uint32_t CAN_DlcToBytes(uint32_t dlc);

static uint32_t CAN_BytesToDlc(uint32_t bytes);

static HAL_StatusTypeDef CAN_Transmit(uint32_t can_id, uint8_t *data, uint32_t len);

static uint8_t CAN_GetStatusFlags(void);

HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *hfdcan) {
    if (hfdcan == NULL) return HAL_ERROR;

    hfdcan_ptr = hfdcan;
    if (CAN_ConfigFilter() != HAL_OK) return HAL_ERROR;

    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    CAN_Status.initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Start(void) {
    if (!CAN_Status.initialized || hfdcan_ptr == NULL) return HAL_ERROR;

    if (HAL_FDCAN_Start(hfdcan_ptr) != HAL_OK) return HAL_ERROR;
    if (HAL_FDCAN_ActivateNotification(hfdcan_ptr,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        return HAL_ERROR;
    }

    CAN_Status.enabled = true;
    CAN_Status.last_heartbeat = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef CAN_Stop(void) {
    if (hfdcan_ptr == NULL) return HAL_ERROR;

    HAL_FDCAN_DeactivateNotification(hfdcan_ptr,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
    HAL_FDCAN_Stop(hfdcan_ptr);
    CAN_Status.enabled = false;
    return HAL_OK;
}

void CAN_Process(void) {
    if (!CAN_Status.enabled) return;

    uint32_t current_tick = HAL_GetTick();
    if (current_tick - CAN_Status.last_heartbeat >= CAN_Config.heartbeat_period) {
        CAN_SendHeartbeat();
        CAN_Status.last_heartbeat = current_tick;
    }
}

HAL_StatusTypeDef CAN_SendHeartbeat(void) {
    CAN_HeartbeatData_t heartbeat;

    heartbeat.mode = (uint8_t) FOC.mode;
    heartbeat.status_flags = CAN_GetStatusFlags();
    heartbeat.error_code = CAN_Status.error_code;
    heartbeat.reserved = 0;
    heartbeat.position = FOC.mechanical_angle * (float) FOC.direction;
    heartbeat.velocity = FOC.mechanical_velocity;
    heartbeat.iq_current = FOC.Iq;
    heartbeat.id_current = FOC.Id;
    heartbeat.vbus = FOC.Vbus;
    heartbeat.temperature = 0.0f;
    heartbeat.timestamp = HAL_GetTick();

    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_HEARTBEAT, CAN_Config.node_id);
    return CAN_Transmit(can_id, (uint8_t *) &heartbeat, sizeof(CAN_HeartbeatData_t));
}

HAL_StatusTypeDef CAN_SendAck(uint8_t cmd_code, uint8_t result, uint16_t error_detail, uint32_t data) {
    CAN_AckData_t ack;
    ack.cmd_code = cmd_code;
    ack.result = result;
    ack.error_detail = error_detail;
    ack.data = data;

    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_ACK, CAN_Config.node_id);
    return CAN_Transmit(can_id, (uint8_t *) &ack, sizeof(CAN_AckData_t));
}

HAL_StatusTypeDef CAN_SendStatusResponse(uint8_t request_type) {
    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_STATUS_RESP, CAN_Config.node_id);

    switch (request_type) {
        case CAN_STATUS_BASIC: {
            CAN_HeartbeatData_t status;
            status.mode = (uint8_t) FOC.mode;
            status.status_flags = CAN_GetStatusFlags();
            status.error_code = CAN_Status.error_code;
            status.reserved = 0;
            status.position = FOC.mechanical_angle * (float) FOC.direction;
            status.velocity = FOC.mechanical_velocity;
            status.iq_current = FOC.Iq;
            status.id_current = FOC.Id;
            status.vbus = FOC.Vbus;
            status.temperature = 0.0f;
            status.timestamp = HAL_GetTick();
            return CAN_Transmit(can_id, (uint8_t *) &status, sizeof(CAN_HeartbeatData_t));
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
            return CAN_Transmit(can_id, (uint8_t *) &pid, sizeof(CAN_PidResponseData_t));
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
            return CAN_Transmit(can_id, (uint8_t *) &pid, sizeof(CAN_PidResponseData_t));
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
            return CAN_Transmit(can_id, (uint8_t *) &pid, sizeof(CAN_PidResponseData_t));
        }
        case CAN_STATUS_CONFIG: {
            CAN_ConfigResponseData_t config;
            config.response_type = CAN_STATUS_CONFIG;
            config.node_id = CAN_Config.node_id;
            config.heartbeat_period = CAN_Config.heartbeat_period;
            config.encoder_offset = FOC.electrical_angle_offset;
            return CAN_Transmit(can_id, (uint8_t *) &config, sizeof(CAN_ConfigResponseData_t));
        }
        default:
            return HAL_ERROR;
    }
}

HAL_StatusTypeDef CAN_SendEmergency(uint8_t action, uint8_t reason) {
    CAN_EmergencyData_t emergency;
    emergency.action = action;
    emergency.reason = reason;
    memset(emergency.reserved, 0, sizeof(emergency.reserved));

    uint32_t can_id = CAN_MAKE_ID(CAN_CMD_EMERGENCY, CAN_Config.node_id);
    return CAN_Transmit(can_id, (uint8_t *) &emergency, sizeof(CAN_EmergencyData_t));
}

void CAN_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (hfdcan != hfdcan_ptr) return;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            CAN_Status.rx_count++;
            uint32_t dlc_bytes = CAN_DlcToBytes(RxHeader.DataLength);
            CAN_ProcessRxMessage(RxHeader.Identifier, RxData, dlc_bytes);
        }
    }
}

CAN_ErrorCode_t CAN_SetNodeId(uint8_t node_id) {
    if (node_id < 1 || node_id > 127) return CAN_ERR_INVALID_PARAM;

    CAN_Config.node_id = node_id;
    if (CAN_Status.enabled) {
        CAN_Stop();
        CAN_ConfigFilter();
        CAN_Start();
    } else {
        CAN_ConfigFilter();
    }
    return CAN_ERR_OK;
}


uint8_t CAN_GetNodeId(void) {
    return CAN_Config.node_id;
}

void CAN_ResetConfig(void) {
    CAN_Config.node_id = DEFAULT_NODE_ID;
    CAN_Config.motor_pairs = 11;
    CAN_Config.motor_direction = -1;
    CAN_Config.vbus_nominal = 12.0f;
    CAN_Config.heartbeat_period = DEFAULT_HEARTBEAT_MS;
}

static HAL_StatusTypeDef CAN_ConfigFilter(void) {
    FDCAN_FilterTypeDef filter;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_Config.node_id;
    filter.FilterID2 = 0x07F;

    if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter) != HAL_OK) return HAL_ERROR;

    filter.FilterIndex = 1;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterID1 = CAN_BROADCAST_ID;
    filter.FilterID2 = CAN_BROADCAST_ID;

    if (HAL_FDCAN_ConfigFilter(hfdcan_ptr, &filter) != HAL_OK) return HAL_ERROR;

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan_ptr,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

static void CAN_ProcessRxMessage(uint32_t can_id, uint8_t *data, uint32_t dlc) {
    if (can_id == CAN_BROADCAST_ID) {
        CAN_HandleBroadcast(data, dlc);
        return;
    }

    uint8_t cmd = CAN_GET_CMD(can_id);
    uint8_t node_id = CAN_GET_NODE_ID(can_id);

    if (node_id != CAN_Config.node_id) return;

    switch (cmd) {
        case CAN_CMD_SET_MODE: CAN_HandleSetMode(data, dlc);
            break;
        case CAN_CMD_SET_TARGET: CAN_HandleSetTarget(data, dlc);
            break;
        case CAN_CMD_GET_STATUS: CAN_HandleGetStatus(data, dlc);
            break;
        case CAN_CMD_EMERGENCY: CAN_HandleEmergency(data, dlc);
            break;
        case CAN_CMD_CONFIG: CAN_HandleConfig(data, dlc);
            break;
        default: CAN_SendAck(cmd, CAN_ERR_UNKNOWN_CMD, 0, 0);
            break;
    }
}

static void CAN_HandleSetMode(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_SetModeData_t)) {
        CAN_SendAck(CAN_CMD_SET_MODE, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }

    CAN_SetModeData_t *mode_data = (CAN_SetModeData_t *) data;

    switch (mode_data->mode) {
        case CAN_MODE_IDLE:
            FOC.mode = IDLE;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            break;
        case CAN_MODE_CURRENT:
            // 重置电流环PID状态
            FOC.pid_iq.integral = 0.0f;
            FOC.Iq_target = 0.0f;
            FOC.mode = Current;
            break;
        case CAN_MODE_VELOCITY:
            // 重置速度环和电流环PID状态
            FOC.pid_velocity.integral = 0.0f;
            FOC.pid_velocity.last_error = 0.0f;
            FOC.pid_iq.integral = 0.0f;
            FOC.velocity_target = 0.0f;
            FOC.Iq_target = 0.0f;
            FOC.mode = Velocity;
            break;
        case CAN_MODE_POSITION:
            // 重置位置环和电流环PID状态
            FOC.pid_position.integral = 0.0f;
            FOC.pid_position.last_error = 0.0f;
            FOC.pid_iq.integral = 0.0f;
            FOC.position_target = FOC.mechanical_angle * (float) FOC.direction;
            FOC.Iq_target = 0.0f;
            FOC.mode = Position;
            break;
        default:
            CAN_SendAck(CAN_CMD_SET_MODE, CAN_ERR_INVALID_PARAM, mode_data->mode, 0);
            return;
    }
    CAN_SendAck(CAN_CMD_SET_MODE, CAN_ERR_OK, 0, mode_data->mode);
}

static void CAN_HandleSetTarget(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_SetTargetData_t)) {
        CAN_SendAck(CAN_CMD_SET_TARGET, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }

    CAN_SetTargetData_t *target_data = (CAN_SetTargetData_t *) data;

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
            FOC.position_target = FOC.mechanical_angle * (float) FOC.direction + target_data->target_value;
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


static void CAN_HandleGetStatus(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_GetStatusData_t)) {
        CAN_SendAck(CAN_CMD_GET_STATUS, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }

    CAN_GetStatusData_t *status_req = (CAN_GetStatusData_t *) data;
    if (CAN_SendStatusResponse(status_req->request_type) != HAL_OK) {
        CAN_SendAck(CAN_CMD_GET_STATUS, CAN_ERR_INVALID_PARAM, status_req->request_type, 0);
    }
}

static void CAN_HandleEmergency(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_EmergencyData_t)) return;

    CAN_EmergencyData_t *emergency = (CAN_EmergencyData_t *) data;

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
    CAN_SendAck(CAN_CMD_EMERGENCY, CAN_ERR_OK, 0, emergency->action);
}

static void CAN_HandleConfig(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_ConfigData_t)) {
        CAN_SendAck(CAN_CMD_CONFIG, CAN_ERR_INVALID_PARAM, 0, 0);
        return;
    }

    CAN_ConfigData_t *config = (CAN_ConfigData_t *) data;
    CAN_ErrorCode_t result = CAN_ERR_OK;

    switch (config->config_cmd) {
        case CAN_CONFIG_SET_NODE_ID:
            result = CAN_SetNodeId((uint8_t) config->value_int);
            break;
        case CAN_CONFIG_SET_HEARTBEAT:
            if (config->value_int >= 10 && config->value_int <= 10000) {
                CAN_Config.heartbeat_period = config->value_int;
            } else {
                result = CAN_ERR_INVALID_PARAM;
            }
            break;
        case CAN_CONFIG_SET_CALIBRATION: {
            // 校准电压: value_float, 如果为0或未设置则使用默认3V
            float calib_voltage = config->value_float;
            if (calib_voltage <= 0.0f || calib_voltage > 10.0f) {
                calib_voltage = 3.0f;
            }
            FOC.mode = IDLE;
            FOC_AlignSensor(calib_voltage);
            // 保存到Flash
            if (Flash_SaveElectricalOffset(FOC.electrical_angle_offset) == HAL_OK) {
                CAN_Status.status_flags |= CAN_STATUS_FLAG_CALIBRATED;
            } else {
                result = CAN_ERR_EXEC_FAIL; // Flash写入失败
            }
            break;
        }
        case CAN_CONFIG_RESET:
            CAN_ResetConfig();
            break;
        default:
            result = CAN_ERR_UNKNOWN_CMD;
            break;
    }
    CAN_SendAck(CAN_CMD_CONFIG, result, 0, config->config_cmd);
}

static void CAN_HandleBroadcast(uint8_t *data, uint32_t dlc) {
    if (dlc < sizeof(CAN_BroadcastData_t)) return;

    CAN_BroadcastData_t *broadcast = (CAN_BroadcastData_t *) data;

    switch (broadcast->broadcast_cmd) {
        case CAN_BROADCAST_EMERGENCY_STOP:
        case CAN_BROADCAST_ALL_IDLE:
            FOC.mode = IDLE;
            FOC.Iq_target = 0.0f;
            FOC.velocity_target = 0.0f;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            if (broadcast->broadcast_cmd == CAN_BROADCAST_EMERGENCY_STOP) {
                CAN_Status.status_flags |= CAN_STATUS_FLAG_FAULT;
            }
            break;
        default:
            break;
    }
}

static uint32_t CAN_DlcToBytes(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
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

static uint32_t CAN_BytesToDlc(uint32_t bytes) {
    if (bytes <= 8) return bytes << 16;
    else if (bytes <= 12) return FDCAN_DLC_BYTES_12;
    else if (bytes <= 16) return FDCAN_DLC_BYTES_16;
    else if (bytes <= 20) return FDCAN_DLC_BYTES_20;
    else if (bytes <= 24) return FDCAN_DLC_BYTES_24;
    else if (bytes <= 32) return FDCAN_DLC_BYTES_32;
    else if (bytes <= 48) return FDCAN_DLC_BYTES_48;
    else return FDCAN_DLC_BYTES_64;
}

static HAL_StatusTypeDef CAN_Transmit(uint32_t can_id, uint8_t *data, uint32_t len) {
    if (hfdcan_ptr == NULL || !CAN_Status.enabled) return HAL_ERROR;

    TxHeader.Identifier = can_id;
    TxHeader.DataLength = CAN_BytesToDlc(len);
    memcpy(TxData, data, len);

    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_ptr, &TxHeader, TxData);
    if (status == HAL_OK) {
        CAN_Status.tx_count++;
    } else {
        CAN_Status.error_count++;
    }
    return status;
}

static uint8_t CAN_GetStatusFlags(void) {
    uint8_t flags = 0;
    if (FOC.mode != IDLE) flags |= CAN_STATUS_FLAG_RUNNING;
    if (CAN_Status.error_code != CAN_ERR_OK) flags |= CAN_STATUS_FLAG_FAULT;
    if (FOC.electrical_angle_offset != 0.0f) flags |= CAN_STATUS_FLAG_CALIBRATED;
    return flags;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    CAN_RxCallback(hfdcan, RxFifo0ITs);
}
