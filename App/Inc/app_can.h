//
// Created by linglitel on 2026/2/12.
// ZeroForm FOC CAN通信模块
//

#ifndef ZEROFORM_FOC_APP_CAN_H
#define ZEROFORM_FOC_APP_CAN_H

#include "main.h"
#include "app_foc.h"

//==============================================================================
// CAN协议版本
//==============================================================================
#define CAN_PROTOCOL_VERSION_MAJOR  1
#define CAN_PROTOCOL_VERSION_MINOR  0
#define CAN_PROTOCOL_VERSION_PATCH  0

//==============================================================================
// CAN ID 宏定义
//==============================================================================
#define CAN_MAKE_ID(cmd, node_id)    (((cmd) << 7) | ((node_id) & 0x7F))
#define CAN_GET_CMD(can_id)          (((can_id) >> 7) & 0x0F)
#define CAN_GET_NODE_ID(can_id)      ((can_id) & 0x7F)

// 广播ID
#define CAN_BROADCAST_ID             0x7FF

//==============================================================================
// 功能码定义 (CMD)
//==============================================================================
typedef enum {
    CAN_CMD_HEARTBEAT    = 0x0,   // 心跳包/状态广播
    CAN_CMD_SET_MODE     = 0x1,   // 设置控制模式
    CAN_CMD_SET_TARGET   = 0x2,   // 设置目标值
    CAN_CMD_SET_PID      = 0x3,   // 设置PID参数
    CAN_CMD_GET_STATUS   = 0x4,   // 请求状态
    CAN_CMD_STATUS_RESP  = 0x5,   // 状态响应
    CAN_CMD_EMERGENCY    = 0x6,   // 紧急停止
    CAN_CMD_CONFIG       = 0x7,   // 配置参数
    CAN_CMD_CALIBRATE    = 0x8,   // 校准命令
    CAN_CMD_ACK          = 0x9,   // 命令确认
    CAN_CMD_BROADCAST    = 0xF    // 广播命令
} CAN_CmdCode_t;

//==============================================================================
// 控制模式定义
//==============================================================================
typedef enum {
    CAN_MODE_IDLE       = 0,    // 空闲模式
    CAN_MODE_CURRENT    = 1,    // 电流/力矩模式
    CAN_MODE_VELOCITY   = 2,    // 速度模式
    CAN_MODE_POSITION   = 3     // 位置模式
} CAN_ControlMode_t;

//==============================================================================
// 目标类型定义
//==============================================================================
typedef enum {
    CAN_TARGET_VELOCITY     = 0,    // 目标速度
    CAN_TARGET_POSITION_ABS = 1,    // 绝对位置
    CAN_TARGET_POSITION_REL = 2,    // 相对位置
    CAN_TARGET_CURRENT      = 3     // 目标电流
} CAN_TargetType_t;

//==============================================================================
// PID类型定义
//==============================================================================
typedef enum {
    CAN_PID_CURRENT_IQ  = 0,    // Iq电流环
    CAN_PID_CURRENT_ID  = 1,    // Id电流环
    CAN_PID_VELOCITY    = 2,    // 速度环
    CAN_PID_POSITION    = 3     // 位置环
} CAN_PidType_t;

//==============================================================================
// 状态请求类型定义
//==============================================================================
typedef enum {
    CAN_STATUS_BASIC        = 0,    // 基本状态
    CAN_STATUS_PID_CURRENT  = 1,    // 电流环PID参数
    CAN_STATUS_PID_VELOCITY = 2,    // 速度环PID参数
    CAN_STATUS_PID_POSITION = 3,    // 位置环PID参数
    CAN_STATUS_CONFIG       = 4,    // 配置参数
    CAN_STATUS_VERSION      = 5     // 版本信息
} CAN_StatusRequest_t;

//==============================================================================
// 紧急停止动作定义
//==============================================================================
typedef enum {
    CAN_EMERGENCY_STOP      = 0,    // 紧急停止
    CAN_EMERGENCY_RECOVER   = 1,    // 恢复运行
    CAN_EMERGENCY_RESET     = 2     // 复位控制器
} CAN_EmergencyAction_t;

//==============================================================================
// 紧急停止原因定义
//==============================================================================
typedef enum {
    CAN_REASON_USER         = 0,    // 用户触发
    CAN_REASON_OVERCURRENT  = 1,    // 过流
    CAN_REASON_OVERVOLTAGE  = 2,    // 过压
    CAN_REASON_UNDERVOLTAGE = 3,    // 欠压
    CAN_REASON_OVERTEMP     = 4,    // 过温
    CAN_REASON_ENCODER      = 5,    // 编码器故障
    CAN_REASON_COMM_TIMEOUT = 6     // 通信超时
} CAN_EmergencyReason_t;

//==============================================================================
// 配置命令定义
//==============================================================================
typedef enum {
    CAN_CONFIG_SET_NODE_ID      = 0x00,   // 设置节点ID
    CAN_CONFIG_SET_MOTOR_PAIRS  = 0x01,   // 设置电机极对数
    CAN_CONFIG_SET_MOTOR_DIR    = 0x02,   // 设置电机方向
    CAN_CONFIG_SET_VBUS         = 0x03,   // 设置标称母线电压
    CAN_CONFIG_SET_HEARTBEAT    = 0x04,   // 设置心跳周期
    CAN_CONFIG_SAVE             = 0x10,   // 保存配置到Flash
    CAN_CONFIG_LOAD             = 0x11,   // 从Flash加载配置
    CAN_CONFIG_RESET            = 0x12    // 恢复默认配置
} CAN_ConfigCmd_t;

//==============================================================================
// 校准类型定义
//==============================================================================
typedef enum {
    CAN_CALIB_ENCODER_ALIGN     = 0,    // 编码器对齐校准
    CAN_CALIB_CURRENT_OFFSET    = 1,    // 电流偏置校准
    CAN_CALIB_SET_ZERO          = 2,    // 设置当前位置为零点
    CAN_CALIB_COGGING           = 3,    // 齿槽校准
    CAN_CALIB_SAVE              = 4     // 保存校准数据到Flash
} CAN_CalibType_t;

//==============================================================================
// 广播命令定义
//==============================================================================
typedef enum {
    CAN_BROADCAST_EMERGENCY_STOP = 0,   // 所有节点紧急停止
    CAN_BROADCAST_SYNC_TIME      = 1,   // 时间同步
    CAN_BROADCAST_ALL_IDLE       = 2    // 所有节点进入空闲
} CAN_BroadcastCmd_t;

//==============================================================================
// 错误码定义
//==============================================================================
typedef enum {
    CAN_ERR_OK              = 0x00,   // 成功
    CAN_ERR_UNKNOWN_CMD     = 0x01,   // 未知命令
    CAN_ERR_INVALID_PARAM   = 0x02,   // 参数无效
    CAN_ERR_WRONG_MODE      = 0x03,   // 模式错误
    CAN_ERR_NOT_CALIBRATED  = 0x04,   // 未校准
    CAN_ERR_OVERCURRENT     = 0x05,   // 过流保护
    CAN_ERR_UNDERVOLTAGE    = 0x06,   // 欠压保护
    CAN_ERR_OVERVOLTAGE     = 0x07,   // 过压保护
    CAN_ERR_OVERTEMP        = 0x08,   // 过温保护
    CAN_ERR_ENCODER         = 0x09,   // 编码器故障
    CAN_ERR_COMM_TIMEOUT    = 0x0A,   // 通信超时
    CAN_ERR_BUSY            = 0x0B,   // 设备忙
    CAN_ERR_FLASH           = 0x0C    // Flash操作失败
} CAN_ErrorCode_t;

//==============================================================================
// 状态标志位定义
//==============================================================================
#define CAN_STATUS_FLAG_RUNNING         (1 << 0)    // 电机运行中
#define CAN_STATUS_FLAG_FAULT           (1 << 1)    // 存在故障
#define CAN_STATUS_FLAG_CALIBRATED      (1 << 2)    // 已校准
#define CAN_STATUS_FLAG_TARGET_REACHED  (1 << 3)    // 目标已到达
#define CAN_STATUS_FLAG_OVERTEMP        (1 << 4)    // 过温警告
#define CAN_STATUS_FLAG_OVERCURRENT     (1 << 5)    // 过流警告

//==============================================================================
// 数据帧结构体定义
//==============================================================================

#pragma pack(push, 1)

// 心跳包数据 (32字节)
typedef struct {
    uint8_t mode;               // 当前模式
    uint8_t status_flags;       // 状态标志位
    uint8_t error_code;         // 错误码
    uint8_t reserved;           // 保留
    float position;             // 当前机械角度 (rad)
    float velocity;             // 当前机械速度 (rad/s)
    float iq_current;           // 当前Iq电流 (A)
    float id_current;           // 当前Id电流 (A)
    float vbus;                 // 母线电压 (V)
    float temperature;          // 温度 (°C)
    uint32_t timestamp;         // 系统时间戳 (ms)
} CAN_HeartbeatData_t;

// 设置模式数据 (8字节)
typedef struct {
    uint8_t mode;               // 目标模式
    uint8_t reserved[7];        // 保留
} CAN_SetModeData_t;

// 设置目标值数据 (24字节)
typedef struct {
    uint8_t target_type;        // 目标类型
    uint8_t reserved[3];        // 保留
    float target_value;         // 主目标值
    float velocity_ff;          // 速度前馈 (位置模式)
    float current_ff;           // 电流前馈
    float max_velocity;         // 最大速度限制
    float max_current;          // 最大电流限制
} CAN_SetTargetData_t;

// 设置PID参数数据 (32字节)
typedef struct {
    uint8_t pid_type;           // PID环类型
    uint8_t reserved[3];        // 保留
    float kp;                   // 比例增益
    float ki;                   // 积分增益
    float kd;                   // 微分增益
    float output_limit;         // 输出限幅
    float integral_limit;       // 积分限幅
    uint8_t reserved2[8];       // 保留
} CAN_SetPidData_t;

// 请求状态数据 (8字节)
typedef struct {
    uint8_t request_type;       // 请求类型
    uint8_t reserved[7];        // 保留
} CAN_GetStatusData_t;

// PID参数响应数据 (24字节)
typedef struct {
    uint8_t response_type;      // 响应类型
    uint8_t reserved[3];        // 保留
    float kp;                   // 比例增益
    float ki;                   // 积分增益
    float kd;                   // 微分增益
    float output_limit;         // 输出限幅
    float integral_limit;       // 积分限幅
} CAN_PidResponseData_t;

// 配置参数响应数据 (16字节)
typedef struct {
    uint8_t response_type;      // 响应类型 = 4
    uint8_t node_id;            // 当前节点ID
    uint8_t motor_pairs;        // 电机极对数
    int8_t motor_direction;     // 电机方向 (+1/-1)
    float vbus_nominal;         // 标称母线电压
    uint32_t heartbeat_period;  // 心跳周期 (ms)
    float encoder_offset;       // 编码器偏移
} CAN_ConfigResponseData_t;

// 版本信息响应数据 (36字节)
typedef struct {
    uint8_t response_type;      // 响应类型 = 5
    uint8_t version_major;      // 主版本号
    uint8_t version_minor;      // 次版本号
    uint8_t version_patch;      // 补丁版本号
    char build_date[16];        // 编译日期
    char device_name[16];       // 设备名称
} CAN_VersionResponseData_t;

// 紧急停止数据 (8字节)
typedef struct {
    uint8_t action;             // 动作类型
    uint8_t reason;             // 原因码
    uint8_t reserved[6];        // 保留
} CAN_EmergencyData_t;

// 配置参数数据 (16字节)
typedef struct {
    uint8_t config_cmd;         // 配置命令
    uint8_t reserved[3];        // 保留
    uint32_t value_int;         // 整数值
    float value_float;          // 浮点值
    uint8_t reserved2[4];       // 保留
} CAN_ConfigData_t;

// 校准命令数据 (12字节)
typedef struct {
    uint8_t calib_type;         // 校准类型
    uint8_t reserved[3];        // 保留
    float param1;               // 参数1
    float param2;               // 参数2
} CAN_CalibrateData_t;

// 命令确认数据 (8字节)
typedef struct {
    uint8_t cmd_code;           // 原命令功能码
    uint8_t result;             // 执行结果
    uint16_t error_detail;      // 错误详情
    uint32_t data;              // 附加数据
} CAN_AckData_t;

// 广播命令数据 (8字节)
typedef struct {
    uint8_t broadcast_cmd;      // 广播命令类型
    uint8_t params[7];          // 命令参数
} CAN_BroadcastData_t;

#pragma pack(pop)

//==============================================================================
// CAN配置结构体
//==============================================================================
typedef struct {
    uint8_t node_id;            // 节点ID (1-127)
    uint8_t motor_pairs;        // 电机极对数
    int8_t motor_direction;     // 电机方向 (+1/-1)
    float vbus_nominal;         // 标称母线电压
    uint32_t heartbeat_period;  // 心跳周期 (ms)
} CAN_Config_t;

//==============================================================================
// CAN状态结构体
//==============================================================================
typedef struct {
    uint8_t initialized;        // 是否已初始化
    uint8_t enabled;            // 是否启用
    uint8_t error_code;         // 当前错误码
    uint8_t status_flags;       // 状态标志
    uint32_t last_heartbeat;    // 上次心跳时间
    uint32_t rx_count;          // 接收计数
    uint32_t tx_count;          // 发送计数
    uint32_t error_count;       // 错误计数
} CAN_Status_t;

//==============================================================================
// 全局变量声明
//==============================================================================
extern CAN_Config_t CAN_Config;
extern CAN_Status_t CAN_Status;

//==============================================================================
// 函数声明
//==============================================================================

/**
 * @brief 初始化CAN模块
 * @param hfdcan FDCAN句柄
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief 启动CAN通信
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Start(void);

/**
 * @brief 停止CAN通信
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_Stop(void);

/**
 * @brief CAN主循环处理 (在主循环中调用)
 */
void CAN_Process(void);

/**
 * @brief 发送心跳包
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_SendHeartbeat(void);

/**
 * @brief 发送ACK响应
 * @param cmd_code 原命令功能码
 * @param result 执行结果
 * @param error_detail 错误详情
 * @param data 附加数据
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_SendAck(uint8_t cmd_code, uint8_t result, uint16_t error_detail, uint32_t data);

/**
 * @brief 发送状态响应
 * @param request_type 请求类型
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_SendStatusResponse(uint8_t request_type);

/**
 * @brief 发送紧急停止
 * @param action 动作类型
 * @param reason 原因码
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_SendEmergency(uint8_t action, uint8_t reason);

/**
 * @brief FDCAN接收回调 (在中断中调用)
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs 中断标志
 */
void CAN_RxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief 设置节点ID
 * @param node_id 新的节点ID (1-127)
 * @return CAN_ErrorCode_t
 */
CAN_ErrorCode_t CAN_SetNodeId(uint8_t node_id);

/**
 * @brief 获取当前节点ID
 * @return 当前节点ID
 */
uint8_t CAN_GetNodeId(void);

/**
 * @brief 保存配置到Flash
 * @return CAN_ErrorCode_t
 */
CAN_ErrorCode_t CAN_SaveConfig(void);

/**
 * @brief 从Flash加载配置
 * @return CAN_ErrorCode_t
 */
CAN_ErrorCode_t CAN_LoadConfig(void);

/**
 * @brief 恢复默认配置
 */
void CAN_ResetConfig(void);

#endif //ZEROFORM_FOC_APP_CAN_H
