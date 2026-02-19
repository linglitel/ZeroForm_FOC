//
// Created by linglitel on 2026/2/12.
// ZeroForm FOC CAN通信模块 (精简版)
//

#ifndef ZEROFORM_FOC_APP_CAN_H
#define ZEROFORM_FOC_APP_CAN_H

#include "main.h"
#include "app_foc.h"

//==============================================================================
// CAN ID 宏定义
//==============================================================================
#define CAN_MAKE_ID(cmd, node_id)    (((cmd) << 7) | ((node_id) & 0x7F))
#define CAN_GET_CMD(can_id)          (((can_id) >> 7) & 0x0F)
#define CAN_GET_NODE_ID(can_id)      ((can_id) & 0x7F)
#define CAN_BROADCAST_ID             0x7FF

//==============================================================================
// 功能码定义
//==============================================================================
typedef enum {
    CAN_CMD_HEARTBEAT = 0x0,
    CAN_CMD_SET_MODE = 0x1,
    CAN_CMD_SET_TARGET = 0x2,
    CAN_CMD_SET_PID = 0x3,
    CAN_CMD_GET_STATUS = 0x4,
    CAN_CMD_STATUS_RESP = 0x5,
    CAN_CMD_EMERGENCY = 0x6,
    CAN_CMD_CONFIG = 0x7,
    CAN_CMD_ACK = 0x9,
    CAN_CMD_BROADCAST = 0xF
} CAN_CmdCode_t;

//==============================================================================
// 控制模式定义
//==============================================================================
typedef enum {
    CAN_MODE_IDLE = 0,
    CAN_MODE_CURRENT = 1,
    CAN_MODE_VELOCITY = 2,
    CAN_MODE_POSITION = 3
} CAN_ControlMode_t;

//==============================================================================
// 目标类型定义
//==============================================================================
typedef enum {
    CAN_TARGET_VELOCITY = 0,
    CAN_TARGET_POSITION_ABS = 1,
    CAN_TARGET_POSITION_REL = 2,
    CAN_TARGET_CURRENT = 3
} CAN_TargetType_t;

//==============================================================================
// PID类型定义
//==============================================================================
typedef enum {
    CAN_PID_CURRENT_IQ = 0,
    CAN_PID_CURRENT_ID = 1,
    CAN_PID_VELOCITY = 2,
    CAN_PID_POSITION = 3
} CAN_PidType_t;

//==============================================================================
// 状态请求类型定义
//==============================================================================
typedef enum {
    CAN_STATUS_BASIC = 0,
    CAN_STATUS_PID_CURRENT = 1,
    CAN_STATUS_PID_VELOCITY = 2,
    CAN_STATUS_PID_POSITION = 3,
    CAN_STATUS_CONFIG = 4
} CAN_StatusRequest_t;

//==============================================================================
// 紧急停止动作定义
//==============================================================================
typedef enum {
    CAN_EMERGENCY_STOP = 0,
    CAN_EMERGENCY_RECOVER = 1,
    CAN_EMERGENCY_RESET = 2
} CAN_EmergencyAction_t;

//==============================================================================
// 配置命令定义
//==============================================================================
typedef enum {
    CAN_CONFIG_SET_NODE_ID = 0x00,
    CAN_CONFIG_SET_HEARTBEAT = 0x01,
    CAN_CONFIG_SET_CALIBRATION = 0x02,
    CAN_CONFIG_SET_ZERO = 0x03,
    CAN_CONFIG_RESET = 0x12
} CAN_ConfigCmd_t;

//==============================================================================
// 广播命令定义
//==============================================================================
typedef enum {
    CAN_BROADCAST_EMERGENCY_STOP = 0,
    CAN_BROADCAST_ALL_IDLE = 2
} CAN_BroadcastCmd_t;

//==============================================================================
// 错误码定义
//==============================================================================
typedef enum {
    CAN_ERR_OK = 0x00,
    CAN_ERR_UNKNOWN_CMD = 0x01,
    CAN_ERR_INVALID_PARAM = 0x02,
    CAN_ERR_WRONG_MODE = 0x03,
    CAN_ERR_EXEC_FAIL = 0x04,
} CAN_ErrorCode_t;

//==============================================================================
// 状态标志位定义
//==============================================================================
#define CAN_STATUS_FLAG_RUNNING         (1 << 0)
#define CAN_STATUS_FLAG_FAULT           (1 << 1)
#define CAN_STATUS_FLAG_CALIBRATED      (1 << 2)

//==============================================================================
// 数据帧结构体定义
//==============================================================================
#pragma pack(push, 1)

typedef struct {
    uint8_t mode;
    uint8_t status_flags;
    uint8_t error_code;
    uint8_t reserved;
    float position;
    float velocity;
    float iq_current;
    float id_current;
    float vbus;
    float temperature;
    uint32_t timestamp;
} CAN_HeartbeatData_t;

typedef struct {
    uint8_t mode;
    uint8_t reserved[7];
} CAN_SetModeData_t;

typedef struct {
    uint8_t target_type;
    uint8_t reserved[3];
    float target_value;
    float velocity_ff;
    float current_ff;
    float max_velocity;
    float max_current;
} CAN_SetTargetData_t;

typedef struct {
    uint8_t pid_type;
    uint8_t reserved[3];
    float kp;
    float ki;
    float kd;
    float output_limit;
    float integral_limit;
    uint8_t reserved2[8];
} CAN_SetPidData_t;

typedef struct {
    uint8_t request_type;
    uint8_t reserved[7];
} CAN_GetStatusData_t;

typedef struct {
    uint8_t response_type;
    uint8_t reserved[3];
    float kp;
    float ki;
    float kd;
    float output_limit;
    float integral_limit;
} CAN_PidResponseData_t;

typedef struct {
    uint8_t response_type;
    uint8_t node_id;
    uint32_t heartbeat_period;
    float encoder_offset;
} CAN_ConfigResponseData_t;

typedef struct {
    uint8_t action;
    uint8_t reason;
    uint8_t reserved[6];
} CAN_EmergencyData_t;

typedef struct {
    uint8_t config_cmd;
    uint8_t reserved[3];
    uint32_t value_int;
    float value_float;
    uint8_t reserved2[4];
} CAN_ConfigData_t;

typedef struct {
    uint8_t cmd_code;
    uint8_t result;
    uint16_t error_detail;
    uint32_t data;
} CAN_AckData_t;

typedef struct {
    uint8_t broadcast_cmd;
    uint8_t params[7];
} CAN_BroadcastData_t;

#pragma pack(pop)

//==============================================================================
// CAN配置结构体
//==============================================================================
typedef struct {
    uint8_t node_id;
    uint8_t motor_pairs;
    int8_t motor_direction;
    float vbus_nominal;
    uint32_t heartbeat_period;
} CAN_Config_t;

//==============================================================================
// CAN状态结构体
//==============================================================================
typedef struct {
    uint8_t initialized;
    uint8_t enabled;
    uint8_t error_code;
    uint8_t status_flags;
    uint32_t last_heartbeat;
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t error_count;
} CAN_Status_t;

//==============================================================================
// 全局变量声明
//==============================================================================
extern CAN_Config_t CAN_Config;
extern CAN_Status_t CAN_Status;

//==============================================================================
// 函数声明
//==============================================================================
HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *hfdcan);

HAL_StatusTypeDef CAN_Start(void);

HAL_StatusTypeDef CAN_Stop(void);

void CAN_Process(void);

HAL_StatusTypeDef CAN_SendHeartbeat(void);

HAL_StatusTypeDef CAN_SendAck(uint8_t cmd_code, uint8_t result, uint16_t error_detail, uint32_t data);

HAL_StatusTypeDef CAN_SendStatusResponse(uint8_t request_type);

HAL_StatusTypeDef CAN_SendEmergency(uint8_t action, uint8_t reason);

void CAN_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

CAN_ErrorCode_t CAN_SetNodeId(uint8_t node_id);

uint8_t CAN_GetNodeId(void);

void CAN_ResetConfig(void);

#endif //ZEROFORM_FOC_APP_CAN_H
