# ZeroForm FOC CAN通信协议规范 V1.0

## 1. 概述

本协议专为ZeroForm FOC电机控制器设计，基于FDCAN实现高速、可靠的多节点电机控制通信。

### 1.1 基本参数

| 参数 | 值 | 说明 |
|------|-----|------|
| CAN类型 | FDCAN (CAN FD) | 仅支持CAN FD模式 |
| 标称波特率 | 1 Mbps | 仲裁段 |
| 数据波特率 | 5 Mbps | 数据段 (BRS启用) |
| 最大数据长度 | 64字节 | 充分利用FDCAN特性 |
| 节点ID范围 | 0x01 - 0x7F | 支持127个节点 |
| 默认节点ID | 0x01 | 可通过配置命令修改 |

### 1.2 帧格式

- 使用 **11位标准ID** 格式
- 启用 **BRS (Bit Rate Switch)** 数据段加速
- 启用 **FDF (FD Format)** 标识

---

## 2. CAN ID 分配方案

### 2.1 ID结构

```
| Bit 10-7 (4bit) | Bit 6-0 (7bit) |
|-----------------|----------------|
|   功能码 (CMD)   |   节点ID (ID)   |
```

### 2.2 ID计算公式

```c
CAN_ID = (CMD << 7) | NODE_ID

// 宏定义
#define CAN_MAKE_ID(cmd, node_id)    (((cmd) << 7) | ((node_id) & 0x7F))
#define CAN_GET_CMD(can_id)          (((can_id) >> 7) & 0x0F)
#define CAN_GET_NODE_ID(can_id)      ((can_id) & 0x7F)
```

### 2.3 功能码定义

| 功能码 | 值 (Hex) | CAN ID范围 | 方向 | 描述 |
|--------|----------|------------|------|------|
| CMD_HEARTBEAT | 0x0 | 0x001-0x07F | 节点→主机 | 心跳包/状态广播 |
| CMD_SET_MODE | 0x1 | 0x080-0x0FF | 主机→节点 | 设置控制模式 |
| CMD_SET_TARGET | 0x2 | 0x100-0x17F | 主机→节点 | 设置目标值 |
| CMD_SET_PID | 0x3 | 0x180-0x1FF | 主机→节点 | 设置PID参数 |
| CMD_GET_STATUS | 0x4 | 0x200-0x27F | 主机→节点 | 请求状态 |
| CMD_STATUS_RESP | 0x5 | 0x280-0x2FF | 节点→主机 | 状态响应 |
| CMD_EMERGENCY | 0x6 | 0x300-0x37F | 双向 | 紧急停止 |
| CMD_CONFIG | 0x7 | 0x380-0x3FF | 主机→节点 | 配置参数 |
| CMD_CALIBRATE | 0x8 | 0x400-0x47F | 主机→节点 | 校准命令 |
| CMD_ACK | 0x9 | 0x480-0x4FF | 节点→主机 | 命令确认/错误响应 |
| CMD_BROADCAST | 0xF | 0x780-0x7FF | 主机→所有 | 广播命令 |

### 2.4 特殊ID

| ID | 描述 |
|----|------|
| 0x000 | 保留 (不使用) |
| 0x7FF | 广播到所有节点 |

---

## 3. 数据帧格式定义

### 3.1 心跳包 (CMD_HEARTBEAT = 0x0)

**方向**: 节点 → 主机  
**周期**: 100ms  
**数据长度**: 32字节  
**CAN ID**: 0x001 ~ 0x07F (取决于节点ID)

| Byte | 字段名 | 类型 | 单位 | 描述 |
|------|--------|------|------|------|
| 0 | mode | uint8 | - | 当前模式 |
| 1 | status_flags | uint8 | - | 状态标志位 |
| 2 | error_code | uint8 | - | 错误码 |
| 3 | reserved | uint8 | - | 保留 |
| 4-7 | position | float32 | rad | 当前机械角度 |
| 8-11 | velocity | float32 | rad/s | 当前机械速度 |
| 12-15 | iq_current | float32 | A | 当前Iq电流 |
| 16-19 | id_current | float32 | A | 当前Id电流 |
| 20-23 | vbus | float32 | V | 母线电压 |
| 24-27 | temperature | float32 | °C | 温度 (预留) |
| 28-31 | timestamp | uint32 | ms | 系统时间戳 |

**status_flags 位定义:**
| Bit | 名称 | 描述 |
|-----|------|------|
| 0 | RUNNING | 电机运行中 |
| 1 | FAULT | 存在故障 |
| 2 | CALIBRATED | 已校准 |
| 3 | TARGET_REACHED | 目标已到达 |
| 4 | OVERTEMP | 过温警告 |
| 5 | OVERCURRENT | 过流警告 |
| 6-7 | reserved | 保留 |

---

### 3.2 设置模式 (CMD_SET_MODE = 0x1)

**方向**: 主机 → 节点  
**数据长度**: 8字节  
**CAN ID**: 0x080 + NODE_ID

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | mode | uint8 | 目标模式 |
| 1-7 | reserved | - | 保留 |

**模式定义:**
| 值 | 模式名 | 描述 |
|----|--------|------|
| 0 | IDLE | 空闲模式 (电机自由) |
| 1 | CURRENT | 电流/力矩模式 |
| 2 | VELOCITY | 速度模式 |
| 3 | POSITION | 位置模式 |

---

### 3.3 设置目标值 (CMD_SET_TARGET = 0x2)

**方向**: 主机 → 节点  
**数据长度**: 24字节  
**CAN ID**: 0x100 + NODE_ID

| Byte | 字段名 | 类型 | 单位 | 描述 |
|------|--------|------|------|------|
| 0 | target_type | uint8 | - | 目标类型 |
| 1-3 | reserved | - | - | 保留 |
| 4-7 | target_value | float32 | 见下表 | 主目标值 |
| 8-11 | velocity_ff | float32 | rad/s | 速度前馈 (位置模式) |
| 12-15 | current_ff | float32 | A | 电流前馈 |
| 16-19 | max_velocity | float32 | rad/s | 最大速度限制 |
| 20-23 | max_current | float32 | A | 最大电流限制 |

**target_type 定义:**
| 值 | 类型 | target_value单位 | 描述 |
|----|------|------------------|------|
| 0 | VELOCITY | rad/s | 目标速度 |
| 1 | POSITION_ABS | rad | 绝对位置 |
| 2 | POSITION_REL | rad | 相对位置 |
| 3 | CURRENT | A | 目标电流 (Iq) |

---

### 3.4 设置PID参数 (CMD_SET_PID = 0x3)

**方向**: 主机 → 节点  
**数据长度**: 32字节  
**CAN ID**: 0x180 + NODE_ID

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | pid_type | uint8 | PID环类型 |
| 1-3 | reserved | - | 保留 |
| 4-7 | kp | float32 | 比例增益 |
| 8-11 | ki | float32 | 积分增益 |
| 12-15 | kd | float32 | 微分增益 |
| 16-19 | output_limit | float32 | 输出限幅 |
| 20-23 | integral_limit | float32 | 积分限幅 |
| 24-31 | reserved | - | 保留 |

**pid_type 定义:**
| 值 | 类型 | 描述 |
|----|------|------|
| 0 | CURRENT_IQ | Iq电流环 |
| 1 | CURRENT_ID | Id电流环 |
| 2 | VELOCITY | 速度环 |
| 3 | POSITION | 位置环 |

---

### 3.5 请求状态 (CMD_GET_STATUS = 0x4)

**方向**: 主机 → 节点  
**数据长度**: 8字节  
**CAN ID**: 0x200 + NODE_ID

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | request_type | uint8 | 请求类型 |
| 1-7 | reserved | - | 保留 |

**request_type 定义:**
| 值 | 类型 | 描述 |
|----|------|------|
| 0 | BASIC | 基本状态 (同心跳包) |
| 1 | PID_CURRENT | 电流环PID参数 |
| 2 | PID_VELOCITY | 速度环PID参数 |
| 3 | PID_POSITION | 位置环PID参数 |
| 4 | CONFIG | 配置参数 |
| 5 | VERSION | 固件版本信息 |

---

### 3.6 状态响应 (CMD_STATUS_RESP = 0x5)

**方向**: 节点 → 主机  
**数据长度**: 根据请求类型变化  
**CAN ID**: 0x280 + NODE_ID

#### 3.6.1 基本状态响应 (request_type=0)
与心跳包格式相同，32字节

#### 3.6.2 PID参数响应 (request_type=1/2/3)
| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | response_type | uint8 | 响应类型 (1/2/3) |
| 1-3 | reserved | - | 保留 |
| 4-7 | kp | float32 | 比例增益 |
| 8-11 | ki | float32 | 积分增益 |
| 12-15 | kd | float32 | 微分增益 |
| 16-19 | output_limit | float32 | 输出限幅 |
| 20-23 | integral_limit | float32 | 积分限幅 |

#### 3.6.3 配置参数响应 (request_type=4)
| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | response_type | uint8 | 响应类型 = 4 |
| 1 | node_id | uint8 | 当前节点ID |
| 2 | motor_pairs | uint8 | 电机极对数 |
| 3 | motor_direction | int8 | 电机方向 (+1/-1) |
| 4-7 | vbus_nominal | float32 | 标称母线电压 |
| 8-11 | heartbeat_period | uint32 | 心跳周期 (ms) |
| 12-15 | encoder_offset | float32 | 编码器偏移 |

#### 3.6.4 版本信息响应 (request_type=5)
| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | response_type | uint8 | 响应类型 = 5 |
| 1 | version_major | uint8 | 主版本号 |
| 2 | version_minor | uint8 | 次版本号 |
| 3 | version_patch | uint8 | 补丁版本号 |
| 4-19 | build_date | char[16] | 编译日期 |
| 20-35 | device_name | char[16] | 设备名称 |

---

### 3.7 紧急停止 (CMD_EMERGENCY = 0x6)

**方向**: 双向  
**数据长度**: 8字节  
**CAN ID**: 0x300 + NODE_ID (或 0x37F 广播)

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | action | uint8 | 动作类型 |
| 1 | reason | uint8 | 原因码 |
| 2-7 | reserved | - | 保留 |

**action 定义:**
| 值 | 动作 | 描述 |
|----|------|------|
| 0 | STOP | 紧急停止 |
| 1 | RECOVER | 恢复运行 |
| 2 | RESET | 复位控制器 |

**reason 定义:**
| 值 | 原因 | 描述 |
|----|------|------|
| 0 | USER | 用户触发 |
| 1 | OVERCURRENT | 过流 |
| 2 | OVERVOLTAGE | 过压 |
| 3 | UNDERVOLTAGE | 欠压 |
| 4 | OVERTEMP | 过温 |
| 5 | ENCODER_FAULT | 编码器故障 |
| 6 | COMM_TIMEOUT | 通信超时 |

---

### 3.8 配置参数 (CMD_CONFIG = 0x7)

**方向**: 主机 → 节点  
**数据长度**: 16字节  
**CAN ID**: 0x380 + NODE_ID

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | config_cmd | uint8 | 配置命令 |
| 1-3 | reserved | - | 保留 |
| 4-7 | value_int | uint32 | 整数值 |
| 8-11 | value_float | float32 | 浮点值 |
| 12-15 | reserved | - | 保留 |

**config_cmd 定义:**
| 值 | 命令 | 使用字段 | 描述 |
|----|------|----------|------|
| 0x00 | SET_NODE_ID | value_int | 设置节点ID (1-127) |
| 0x01 | SET_MOTOR_PAIRS | value_int | 设置电机极对数 |
| 0x02 | SET_MOTOR_DIR | value_int | 设置电机方向 (+1/-1) |
| 0x03 | SET_VBUS | value_float | 设置标称母线电压 |
| 0x04 | SET_HEARTBEAT | value_int | 设置心跳周期 (ms) |
| 0x10 | SAVE_CONFIG | - | 保存配置到Flash |
| 0x11 | LOAD_CONFIG | - | 从Flash加载配置 |
| 0x12 | RESET_CONFIG | - | 恢复默认配置 |

---

### 3.9 校准命令 (CMD_CALIBRATE = 0x8)

**方向**: 主机 → 节点  
**数据长度**: 12字节  
**CAN ID**: 0x400 + NODE_ID

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | calib_type | uint8 | 校准类型 |
| 1-3 | reserved | - | 保留 |
| 4-7 | param1 | float32 | 参数1 |
| 8-11 | param2 | float32 | 参数2 |

**calib_type 定义:**
| 值 | 类型 | param1 | param2 | 描述 |
|----|------|--------|--------|------|
| 0 | ENCODER_ALIGN | 对齐电压(V) | - | 编码器对齐校准 |
| 1 | CURRENT_OFFSET | - | - | 电流偏置校准 |
| 2 | SET_ZERO | - | - | 设置当前位置为零点 |

---

### 3.10 命令确认 (CMD_ACK = 0x9)

**方向**: 节点 → 主机  
**数据长度**: 8字节  
**CAN ID**: 0x480 + NODE_ID

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | cmd_code | uint8 | 原命令功能码 |
| 1 | result | uint8 | 执行结果 |
| 2-3 | error_detail | uint16 | 错误详情 |
| 4-7 | data | uint32 | 附加数据 |

---

### 3.11 广播命令 (CMD_BROADCAST = 0xF)

**方向**: 主机 → 所有节点  
**数据长度**: 8字节  
**CAN ID**: 0x7FF (固定)

| Byte | 字段名 | 类型 | 描述 |
|------|--------|------|------|
| 0 | broadcast_cmd | uint8 | 广播命令类型 |
| 1-7 | params | - | 命令参数 |

**broadcast_cmd 定义:**
| 值 | 命令 | 描述 |
|----|------|------|
| 0 | EMERGENCY_STOP | 所有节点紧急停止 |
| 1 | SYNC_TIME | 时间同步 |
| 2 | ALL_IDLE | 所有节点进入空闲 |

---

## 4. 错误码定义

| 错误码 | 名称 | 描述 |
|--------|------|------|
| 0x00 | OK | 成功 |
| 0x01 | ERR_UNKNOWN_CMD | 未知命令 |
| 0x02 | ERR_INVALID_PARAM | 参数无效 |
| 0x03 | ERR_WRONG_MODE | 模式错误 |
| 0x04 | ERR_NOT_CALIBRATED | 未校准 |
| 0x05 | ERR_OVERCURRENT | 过流保护 |
| 0x06 | ERR_UNDERVOLTAGE | 欠压保护 |
| 0x07 | ERR_OVERVOLTAGE | 过压保护 |
| 0x08 | ERR_OVERTEMP | 过温保护 |
| 0x09 | ERR_ENCODER | 编码器故障 |
| 0x0A | ERR_COMM_TIMEOUT | 通信超时 |
| 0x0B | ERR_BUSY | 设备忙 |
| 0x0C | ERR_FLASH | Flash操作失败 |

---

## 5. 通信时序

### 5.1 心跳机制

- 节点每100ms发送一次心跳包
- 主机可通过心跳包监控所有节点状态
- 如果主机超过500ms未收到某节点心跳，应视为该节点离线

### 5.2 命令响应

- 所有命令帧发送后，节点应在10ms内回复ACK
- 如果主机50ms内未收到ACK，应重发命令
- 最多重试3次

### 5.3 多节点同步

- 使用广播命令可同时控制所有节点
- 时间同步命令可用于多轴协调运动

---

## 6. 滤波器配置

节点应配置FDCAN滤波器接收以下ID:

1. **本节点命令**: (CMD << 7) | NODE_ID，其中CMD = 0x1~0x8
2. **广播命令**: 0x7FF
3. **紧急停止广播**: 0x37F

---

## 7. 使用示例

### 7.1 启动速度控制

```
1. 主机 → 节点1: [ID=0x081] mode=2 (Velocity)
2. 节点1 → 主机: [ID=0x481] cmd=0x1, result=0 (OK)
3. 主机 → 节点1: [ID=0x101] type=0, value=10.0 rad/s
4. 节点1 → 主机: [ID=0x481] cmd=0x2, result=0 (OK)
```

### 7.2 位置控制

```
1. 主机 → 节点1: [ID=0x081] mode=3 (Position)
2. 节点1 → 主机: [ID=0x481] cmd=0x1, result=0 (OK)
3. 主机 → 节点1: [ID=0x101] type=1, value=3.14159 rad
4. 节点1 → 主机: [ID=0x481] cmd=0x2, result=0 (OK)
```

### 7.3 修改节点ID

```
1. 主机 → 节点1: [ID=0x381] config_cmd=0x00, value_int=2
2. 节点1 → 主机: [ID=0x481] cmd=0x7, result=0 (OK)
3. 主机 → 节点2: [ID=0x382] config_cmd=0x10 (保存配置)
4. 节点2 → 主机: [ID=0x482] cmd=0x7, result=0 (OK)
```

### 7.4 紧急停止所有节点

```
1. 主机 → 广播: [ID=0x7FF] broadcast_cmd=0 (EMERGENCY_STOP)
   (所有节点立即停止，无需ACK)
```

---

## 8. 版本历史

| 版本 | 日期 | 描述 |
|------|------|------|
| V1.0 | 2026-02-12 | 初始版本 |

---

## 9. 附录: 数据类型说明

| 类型 | 大小 | 字节序 | 描述 |
|------|------|--------|------|
| uint8 | 1字节 | - | 无符号8位整数 |
| int8 | 1字节 | - | 有符号8位整数 |
| uint16 | 2字节 | Little-Endian | 无符号16位整数 |
| int16 | 2字节 | Little-Endian | 有符号16位整数 |
| uint32 | 4字节 | Little-Endian | 无符号32位整数 |
| int32 | 4字节 | Little-Endian | 有符号32位整数 |
| float32 | 4字节 | Little-Endian (IEEE 754) | 单精度浮点数 |
