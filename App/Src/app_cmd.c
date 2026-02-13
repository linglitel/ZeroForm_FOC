//
// Created by linglitel on 2026/2/12.
// 串口命令解析模块 - 中断接收版本
//

#include "app_cmd.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_flash.h"
#include "app_cogging.h"
// 全局命令处理器
CMD_Handler_t CMD = {0};

// UART句柄指针
static UART_HandleTypeDef *cmd_huart = NULL;

// 角度转弧度
#define DEG_TO_RAD(deg) ((deg) * 3.14159265358979f / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / 3.14159265358979f)

// 内部函数声明
static void CMD_ParseAndExecute(char *cmd_str);

static void CMD_SendResponse(const char *response);

static float CMD_ParseAngle(const char *str);

/**
 * @brief 初始化命令处理模块（使用中断接收）
 */
void CMD_Init(UART_HandleTypeDef *huart) {
    cmd_huart = huart;
    CMD.rx_index = 0;
    CMD.cmd_ready = 0;
    memset(CMD.rx_buffer, 0, CMD_BUFFER_SIZE);

    // 启动UART接收中断（单字节模式）
    HAL_UART_Receive_IT(cmd_huart, &CMD.rx_byte, 1);

    CMD_SendResponse("FOC Ready!\r\n");
    CMD_SendResponse("Commands: MODE, VEL, POS, ZERO, STATUS, STOP\r\n");
}

void CMD_UART_RxCallback(UART_HandleTypeDef *huart) {
    if (huart != cmd_huart) return;

    uint8_t ch = CMD.rx_byte;

    if (ch == '\n' || ch == '\r') {
        if (CMD.rx_index > 0) {
            CMD.rx_buffer[CMD.rx_index] = '\0';
            CMD.cmd_ready = 1;
            CMD.rx_index = 0;
        }
    } else {
        // 存储字符
        if (CMD.rx_index < CMD_BUFFER_SIZE - 1) {
            CMD.rx_buffer[CMD.rx_index++] = ch;
        } else {
            // 缓冲区溢出，重置
            CMD.rx_index = 0;
        }
    }

    // 继续接收下一个字节
    HAL_UART_Receive_IT(cmd_huart, &CMD.rx_byte, 1);
}

/**
 * @brief 处理接收到的命令（在主循环中调用）
 */
void CMD_Process(void) {
    if (!CMD.cmd_ready) return;

    // 复制命令字符串
    char cmd_str[CMD_BUFFER_SIZE];
    strncpy(cmd_str, (char *) CMD.rx_buffer, CMD_BUFFER_SIZE);

    // 清空缓冲区
    CMD.cmd_ready = 0;
    memset(CMD.rx_buffer, 0, CMD_BUFFER_SIZE);

    // 解析并执行命令
    CMD_ParseAndExecute(cmd_str);
}

/**
 * @brief 发送响应
 */
static void CMD_SendResponse(const char *response) {
    if (cmd_huart == NULL) return;
    HAL_UART_Transmit(cmd_huart, (uint8_t *) response, strlen(response), 100);
}

/**
 * @brief 解析角度值（支持度和弧度）
 */
static float CMD_ParseAngle(const char *str) {
    if (str == NULL) return 0.0f;

    char buf[32];
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    size_t len = strlen(buf);

    // 检查是否以 'R' 或 'r' 结尾（表示弧度）
    if (len > 0 && (buf[len - 1] == 'R' || buf[len - 1] == 'r')) {
        buf[len - 1] = '\0';
        return strtof(buf, NULL);
    }

    // 默认为度，转换为弧度
    return DEG_TO_RAD(strtof(buf, NULL));
}

/**
 * @brief 解析并执行命令
 */
static void CMD_ParseAndExecute(char *cmd_str) {
    char response[128];

    // 转换为大写便于比较
    for (int i = 0; cmd_str[i]; i++) {
        if (cmd_str[i] >= 'a' && cmd_str[i] <= 'z') {
            cmd_str[i] -= 32;
        }
    }

    // 跳过前导空格
    while (*cmd_str == ' ') cmd_str++;

    // ========== MODE 命令 ==========
    if (strncmp(cmd_str, "MODE", 4) == 0) {
        char *param = cmd_str + 4;
        while (*param == ' ') param++;

        if (*param == 'V') {
            FOC.mode = Velocity;
            FOC.velocity_target = 0.0f;
            CMD_SendResponse("OK: Mode = Velocity\r\n");
        } else if (*param == 'P') {
            FOC.mode = Position;
            FOC.position_target = FOC.mechanical_angle * (float) FOC.direction;
            CMD_SendResponse("OK: Mode = Position\r\n");
        } else if (*param == 'I') {
            FOC.mode = IDLE;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            CMD_SendResponse("OK: Mode = IDLE\r\n");
        } else {
            CMD_SendResponse("ERR: Invalid mode. Use V/P/I\r\n");
        }
        return;
    }

    // ========== VEL 命令 ==========
    if (strncmp(cmd_str, "VEL", 3) == 0) {
        if (FOC.mode != Velocity) {
            CMD_SendResponse("ERR: Not in Velocity mode\r\n");
            return;
        }

        char *param = cmd_str + 3;
        while (*param == ' ') param++;

        float vel = strtof(param, NULL);
        FOC.velocity_target = vel;

        snprintf(response, sizeof(response), "OK: Velocity = %.2f rad/s\r\n", vel);
        CMD_SendResponse(response);
        return;
    }

    // ========== POS 命令 ==========
    if (strncmp(cmd_str, "POS", 3) == 0) {
        if (FOC.mode != Position) {
            CMD_SendResponse("ERR: Not in Position mode\r\n");
            return;
        }

        char *param = cmd_str + 3;
        while (*param == ' ') param++;

        char pos_type = *param;
        param++;
        while (*param == ' ') param++;

        float angle_rad = CMD_ParseAngle(param);

        if (pos_type == 'A') {
            FOC.position_target = angle_rad;
            snprintf(response, sizeof(response),
                     "OK: Absolute Position = %.2f deg (%.4f rad)\r\n",
                     RAD_TO_DEG(angle_rad), angle_rad);
        } else if (pos_type == 'R') {
            FOC.position_target = FOC.mechanical_angle * FOC.direction + angle_rad;
            snprintf(response, sizeof(response),
                     "OK: Current Position = %.4f deg, Relative Position += %.2f deg, Target = %.4f rad\r\n",
                     RAD_TO_DEG(FOC.mechanical_angle * (float) FOC.direction),RAD_TO_DEG(angle_rad),
                     FOC.position_target);
        } else {
            CMD_SendResponse("ERR: Use POS A <angle> or POS R <angle>\r\n");
            return;
        }

        CMD_SendResponse(response);
        return;
    }
    // ========== STATUS 命令 ==========
    if (strncmp(cmd_str, "STATUS", 6) == 0) {
        const char *mode_str;
        switch (FOC.mode) {
            case IDLE: mode_str = "IDLE";
                break;
            case Velocity: mode_str = "Velocity";
                break;
            case Position: mode_str = "Position";
                break;
            default: mode_str = "Unknown";
                break;
        }

        snprintf(response, sizeof(response),
                 "Mode: %s\r\n"
                 "Angle: %.2f deg (%.4f rad)\r\n"
                 "Velocity: %.2f rad/s\r\n"
                 "Iq: %.3f A\r\n"
                 "Vbus: %.3f V\r\n",
                 mode_str,
                 RAD_TO_DEG(FOC.mechanical_angle*FOC.direction), FOC.mechanical_angle * FOC.direction,
                 FOC.mechanical_velocity,
                 FOC.Iq,
                 FOC.Vbus);
        CMD_SendResponse(response);

        if (FOC.mode == Velocity) {
            snprintf(response, sizeof(response),
                     "Target Vel: %.2f rad/s\r\n", FOC.velocity_target);
            CMD_SendResponse(response);
        } else if (FOC.mode == Position) {
            snprintf(response, sizeof(response),
                     "Target Pos: %.2f deg (%.4f rad)\r\n",
                     RAD_TO_DEG(FOC.position_target), FOC.position_target);
            CMD_SendResponse(response);
        }
        return;
    }

    // ========== STOP 命令 ==========
    if (strncmp(cmd_str, "STOP", 4) == 0) {
        FOC.mode = IDLE;
        FOC.Iq_target = 0.0f;
        FOC.velocity_target = 0.0f;
        CMD_SendResponse("OK: Motor stopped\r\n");
        return;
    }

    // ========== CALIB 命令 ==========
    if (strncmp(cmd_str, "CALIB", 5) == 0) {
        char *param = cmd_str + 5;
        while (*param == ' ') param++;
        
        if (strncmp(param, "ALIGN", 5) == 0) {
            // 电零角校准
            param += 5;
            while (*param == ' ') param++;
            float voltage = 5.0f;
            if (*param != '\0') {
                voltage = strtof(param, NULL);
                if (voltage < 1.0f) voltage = 1.0f;
                if (voltage > 10.0f) voltage = 10.0f;
            }
            CMD_SendResponse("Calibrating encoder alignment...\r\n");
            FOC.mode = IDLE;
            FOC_AlignSensor(voltage);
            // 更新Flash中的电零角
            Flash_SetElectricalOffset(FOC.electrical_angle_offset);
            snprintf(response, sizeof(response), 
                     "OK: Electrical offset = %.4f rad\r\n"
                     "Use SAVE to store to Flash\r\n", 
                     FOC.electrical_angle_offset);
            CMD_SendResponse(response);
        } else if (strncmp(param, "COG", 3) == 0) {
            // 齿槽校准
            if (Cogging_GetState() != COGGING_STATE_IDLE && 
                Cogging_GetState() != COGGING_STATE_COMPLETE &&
                Cogging_GetState() != COGGING_STATE_ERROR) {
                CMD_SendResponse("ERR: Cogging calibration already in progress\r\n");
                return;
            }
            CMD_SendResponse("Starting cogging calibration...\r\n");
            CMD_SendResponse("Motor will rotate slowly. Please wait ~30s\r\n");
            if (Cogging_StartCalibration()) {
                CMD_SendResponse("OK: Calibration started\r\n");
            } else {
                CMD_SendResponse("ERR: Failed to start calibration\r\n");
            }
        } else if (strncmp(param, "STATUS", 6) == 0) {
            // 校准状态
            snprintf(response, sizeof(response),
                     "Calibration Status:\r\n"
                     "  Electrical offset: %.4f rad (valid: %s)\r\n"
                     "  Cogging: %s (progress: %d%%)\r\n",
                     FOC.electrical_angle_offset,
                     Flash_IsCalibrationValid() ? "Yes" : "No",
                     Cogging_IsCalibrated() ? "Calibrated" : "Not calibrated",
                     Cogging_GetProgress());
            CMD_SendResponse(response);
        } else {
            CMD_SendResponse("CALIB commands:\r\n");
            CMD_SendResponse("  CALIB ALIGN [V]  - Encoder alignment (default 5V)\r\n");
            CMD_SendResponse("  CALIB COG        - Cogging torque calibration\r\n");
            CMD_SendResponse("  CALIB STATUS     - Show calibration status\r\n");
        }
        return;
    }
    
    // ========== SAVE 命令 ==========
    if (strncmp(cmd_str, "SAVE", 4) == 0) {
        CMD_SendResponse("Saving configuration to Flash...\r\n");
        // 准备齿槽数据
        Cogging_PrepareForSave();
        // 保存到Flash
        if (Flash_SaveAll() == HAL_OK) {
            CMD_SendResponse("OK: Configuration saved to Flash\r\n");
        } else {
            CMD_SendResponse("ERR: Failed to save to Flash\r\n");
        }
        return;
    }
    
    // ========== COGGING 命令 ==========
    if (strncmp(cmd_str, "COGGING", 7) == 0) {
        char *param = cmd_str + 7;
        while (*param == ' ') param++;
        
        if (strncmp(param, "ON", 2) == 0) {
            if (Cogging_IsCalibrated()) {
                Cogging_SetEnabled(true);
                Flash_SetCoggingEnabled(true);  // 同步到Flash数据
                CMD_SendResponse("OK: Cogging compensation enabled\r\n");
                CMD_SendResponse("Use SAVE to persist this setting\r\n");
            } else {
                CMD_SendResponse("ERR: Cogging not calibrated. Run CALIB COG first\r\n");
            }
        } else if (strncmp(param, "OFF", 3) == 0) {
            Cogging_SetEnabled(false);
            Flash_SetCoggingEnabled(false);  // 同步到Flash数据
            CMD_SendResponse("OK: Cogging compensation disabled\r\n");
            CMD_SendResponse("Use SAVE to persist this setting\r\n");
        } else {
            snprintf(response, sizeof(response),
                     "Cogging compensation: %s\r\n"
                     "  COGGING ON   - Enable compensation\r\n"
                     "  COGGING OFF  - Disable compensation\r\n",
                     Cogging_IsEnabled() ? "Enabled" : "Disabled");
            CMD_SendResponse(response);
        }
        return;
    }

    // ========== HELP 命令 ==========
    if (strncmp(cmd_str, "HELP", 4) == 0) {
        CMD_SendResponse("Commands:\r\n");
        CMD_SendResponse("  MODE V/P/I    - Set mode (Velocity/Position/Idle)\r\n");
        CMD_SendResponse("  VEL <rad/s>   - Set velocity target\r\n");
        CMD_SendResponse("  POS A <deg>   - Absolute position (degrees)\r\n");
        CMD_SendResponse("  POS R <deg>   - Relative position (degrees)\r\n");
        CMD_SendResponse("  STATUS        - Show current status\r\n");
        CMD_SendResponse("  STOP          - Stop motor\r\n");
        CMD_SendResponse("  CALIB ALIGN   - Encoder alignment calibration\r\n");
        CMD_SendResponse("  CALIB COG     - Cogging torque calibration\r\n");
        CMD_SendResponse("  CALIB STATUS  - Show calibration status\r\n");
        CMD_SendResponse("  COGGING ON/OFF- Enable/disable cogging compensation\r\n");
        CMD_SendResponse("  SAVE          - Save config to Flash\r\n");
        return;
    }

    // 未知命令
    snprintf(response, sizeof(response), "ERR: Unknown command '%s'. Type HELP\r\n", cmd_str);
    CMD_SendResponse(response);
}
