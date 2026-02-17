//
// Created by linglitel on 2026/2/12.
// 串口命令解析模块 (精简版)
//

#include "app_cmd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "app_flash.h"

CMD_Handler_t CMD = {0};
static UART_HandleTypeDef *cmd_huart = NULL;

#define DEG_TO_RAD(deg) ((deg) * 3.14159265358979f / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / 3.14159265358979f)

static void CMD_ParseAndExecute(char *cmd_str);

static void CMD_SendResponse(const char *response);

static float CMD_ParseAngle(const char *str);

void CMD_Init(UART_HandleTypeDef *huart) {
    cmd_huart = huart;
    CMD.rx_index = 0;
    CMD.cmd_ready = 0;
    memset(CMD.rx_buffer, 0, CMD_BUFFER_SIZE);
    HAL_UART_Receive_IT(cmd_huart, &CMD.rx_byte, 1);
    CMD_SendResponse("FOC Ready!\r\n");
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
        if (CMD.rx_index < CMD_BUFFER_SIZE - 1) {
            CMD.rx_buffer[CMD.rx_index++] = ch;
        } else {
            CMD.rx_index = 0;
        }
    }
    HAL_UART_Receive_IT(cmd_huart, &CMD.rx_byte, 1);
}

void CMD_Process(void) {
    if (!CMD.cmd_ready) return;

    char cmd_str[CMD_BUFFER_SIZE];
    strncpy(cmd_str, (char *) CMD.rx_buffer, CMD_BUFFER_SIZE);
    CMD.cmd_ready = 0;
    memset(CMD.rx_buffer, 0, CMD_BUFFER_SIZE);
    CMD_ParseAndExecute(cmd_str);
}

static void CMD_SendResponse(const char *response) {
    if (cmd_huart == NULL) return;
    HAL_UART_Transmit(cmd_huart, (uint8_t *) response, strlen(response), 100);
}

static float CMD_ParseAngle(const char *str) {
    if (str == NULL) return 0.0f;

    char buf[32];
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    size_t len = strlen(buf);

    if (len > 0 && (buf[len - 1] == 'R' || buf[len - 1] == 'r')) {
        buf[len - 1] = '\0';
        return strtof(buf, NULL);
    }
    return DEG_TO_RAD(strtof(buf, NULL));
}

static void CMD_ParseAndExecute(char *cmd_str) {
    char response[128];

    for (int i = 0; cmd_str[i]; i++) {
        if (cmd_str[i] >= 'a' && cmd_str[i] <= 'z') {
            cmd_str[i] -= 32;
        }
    }
    while (*cmd_str == ' ') cmd_str++;

    // MODE 命令
    if (strncmp(cmd_str, "MODE", 4) == 0) {
        char *param = cmd_str + 4;
        while (*param == ' ') param++;

        if (*param == 'V') {
            // 切换到速度模式前，重置相关PID状态和速度测量
            FOC.pid_velocity.integral = 0.0f;
            FOC.pid_velocity.last_error = 0.0f;
            FOC.pid_iq.integral = 0.0f;
            FOC.velocity_target = 0.0f;
            FOC.Iq_target = 0.0f;
            FOC.mechanical_velocity = 0.0f; // 重置速度测量
            FOC.mode = Velocity;
            CMD_SendResponse("OK: Mode = Velocity\r\n");
        } else if (*param == 'P') {
            // 切换到位置模式前，重置相关PID状态
            FOC.pid_position.integral = 0.0f;
            FOC.pid_position.last_error = 0.0f;
            FOC.pid_iq.integral = 0.0f;
            FOC.position_target = FOC.mechanical_angle * (float) FOC.direction;
            FOC.Iq_target = 0.0f;
            FOC.mode = Position;
            CMD_SendResponse("OK: Mode = Position\r\n");
        } else if (*param == 'C') {
            FOC.pid_position.integral = 0.0f;
            FOC.pid_position.last_error = 0.0f;
            FOC.pid_iq.integral = 0.0f;
            FOC.position_target = FOC.mechanical_angle * (float) FOC.direction;
            FOC.Iq_target = 0.0f;
            FOC.mode = Current;
            CMD_SendResponse("OK: Mode = Current\r\n");
        } else if (*param == 'I') {
            FOC.mode = IDLE;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            CMD_SendResponse("OK: Mode = IDLE\r\n");
        } else {
            CMD_SendResponse("ERR: Invalid mode. Use V/P/I\r\n");
        }
        return;
    }

    // VEL 命令
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

    // POS 命令
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
            snprintf(response, sizeof(response), "OK: Absolute Position = %.2f deg\r\n", RAD_TO_DEG(angle_rad));
        } else if (pos_type == 'R') {
            FOC.position_target = FOC.mechanical_angle * (float) FOC.direction + angle_rad;
            snprintf(response, sizeof(response), "OK: Relative Position += %.2f deg\r\n", RAD_TO_DEG(angle_rad));
        } else {
            CMD_SendResponse("ERR: Use POS A <angle> or POS R <angle>\r\n");
            return;
        }
        CMD_SendResponse(response);
        return;
    }
    if (strncmp(cmd_str, "CUR", 3) == 0) {
        if (FOC.mode != Current) {
            CMD_SendResponse("ERR: Not in Current mode\r\n");
            return;
        }
        char *param = cmd_str + 3;
        while (*param == ' ') param++;
        float Iq = strtof(param, NULL);
        FOC.Iq_target = Iq;
        snprintf(response, sizeof(response), "OK: Current = %.2f rad/s\r\n", Iq);
        CMD_SendResponse(response);
        return;
    }

    // STATUS 命令
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
                 "Mode: %s\r\nAngle: %.2f deg\r\nVelocity: %.2f rad/s\r\nIq: %.3f A\r\n",
                 mode_str, RAD_TO_DEG(FOC.mechanical_angle*FOC.direction),
                 FOC.mechanical_velocity, FOC.Iq);
        CMD_SendResponse(response);
        return;
    }

    // STOP 命令
    if (strncmp(cmd_str, "STOP", 4) == 0) {
        FOC.mode = IDLE;
        FOC.Iq_target = 0.0f;
        FOC.velocity_target = 0.0f;
        FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
        CMD_SendResponse("OK: Motor stopped\r\n");
        return;
    }

    // CALIB 命令 (仅保留电零角校准)
    if (strncmp(cmd_str, "CALIB", 5) == 0) {
        char *param = cmd_str + 5;
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

        if (Flash_SaveElectricalOffset(FOC.electrical_angle_offset) == HAL_OK) {
            snprintf(response, sizeof(response), "OK: offset = %.4f rad (saved)\r\n", FOC.electrical_angle_offset);
        } else {
            snprintf(response, sizeof(response), "OK: offset = %.4f rad (save failed)\r\n",
                     FOC.electrical_angle_offset);
        }
        CMD_SendResponse(response);
        return;
    }

    // DEBUG 命令 - 显示调试信息
    if (strncmp(cmd_str, "DEBUG", 5) == 0) {
        snprintf(response, sizeof(response),
                 "=== Velocity Loop ===\r\n"
                 "Target: %.2f rad/s\r\n"
                 "Actual: %.2f rad/s\r\n"
                 "Error: %.2f rad/s\r\n"
                 "Output(Iq): %.3f A\r\n",
                 FOC.velocity_target, FOC.mechanical_velocity,
                 FOC_Debug.velocity_error, FOC_Debug.velocity_output);
        CMD_SendResponse(response);

        snprintf(response, sizeof(response),
                 "PID: P=%.3f I=%.3f D=%.3f\r\n"
                 "Integral: %.3f\r\n",
                 FOC.pid_velocity.Kp, FOC.pid_velocity.Ki, FOC.pid_velocity.Kd,
                 FOC.pid_velocity.integral);
        CMD_SendResponse(response);

        snprintf(response, sizeof(response),
                 "=== Current Loop ===\r\n"
                 "Iq_target: %.3f A\r\n"
                 "Iq_actual: %.3f A\r\n"
                 "Iq_error: %.3f A\r\n"
                 "Vq: %.2f V\r\n",
                 FOC.Iq_target, FOC.Iq, FOC.pid_iq.error, FOC.Vq);
        CMD_SendResponse(response);
        return;
    }

    // STREAM 命令 - 开启/关闭数据流输出
    // 格式: STREAM ON 或 STREAM OFF
    // 输出格式(CSV): time,vel_target,vel_actual,vel_error,iq_target,iq_actual,vq,integral
    if (strncmp(cmd_str, "STREAM", 6) == 0) {
        char *param = cmd_str + 6;
        while (*param == ' ') param++;

        if (strncmp(param, "ON", 2) == 0) {
            FOC_Debug.stream_enabled = 1;
            FOC_Debug.debug_counter = 0;
            CMD_SendResponse("OK: Stream ON\r\n");
            CMD_SendResponse("# time,vel_t,vel,err,iq_t,iq,vq,integ\r\n");
        } else if (strncmp(param, "OFF", 3) == 0) {
            FOC_Debug.stream_enabled = 0;
            CMD_SendResponse("OK: Stream OFF\r\n");
        } else {
            CMD_SendResponse("ERR: Use STREAM ON/OFF\r\n");
        }
        return;
    }

    // PID 命令 - 设置速度环PID参数
    if (strncmp(cmd_str, "PID", 3) == 0) {
        char *param = cmd_str + 3;
        while (*param == ' ') param++;

        if (*param == 'V') {
            // 速度环PID: PID V <Kp> <Ki> <Kd>
            param++;
            while (*param == ' ') param++;
            float kp = strtof(param, &param);
            while (*param == ' ') param++;
            float ki = strtof(param, &param);
            while (*param == ' ') param++;
            float kd = strtof(param, NULL);

            FOC.pid_velocity.Kp = kp;
            FOC.pid_velocity.Ki = ki;
            FOC.pid_velocity.Kd = kd;
            FOC.pid_velocity.integral = 0.0f;

            snprintf(response, sizeof(response),
                     "OK: Velocity PID = %.3f, %.3f, %.3f\r\n", kp, ki, kd);
            CMD_SendResponse(response);
        } else if (*param == 'I') {
            // 电流环PID: PID I <Kp> <Ki> <Kd>
            param++;
            while (*param == ' ') param++;
            float kp = strtof(param, &param);
            while (*param == ' ') param++;
            float ki = strtof(param, &param);
            while (*param == ' ') param++;
            float kd = strtof(param, NULL);

            FOC.pid_iq.Kp = kp;
            FOC.pid_iq.Ki = ki;
            FOC.pid_iq.Kd = kd;
            FOC.pid_iq.integral = 0.0f;

            snprintf(response, sizeof(response),
                     "OK: Current PID = %.3f, %.3f, %.3f\r\n", kp, ki, kd);
            CMD_SendResponse(response);
        } else {
            CMD_SendResponse("ERR: Use PID V/I <Kp> <Ki> <Kd>\r\n");
        }
        return;
    }

    // HELP 命令
    if (strncmp(cmd_str, "HELP", 4) == 0) {
        CMD_SendResponse("Commands:\r\n");
        CMD_SendResponse("  MODE V/P/I  - Set mode\r\n");
        CMD_SendResponse("  VEL <rad/s> - Set velocity\r\n");
        CMD_SendResponse("  POS A/R <deg> - Set position\r\n");
        CMD_SendResponse("  STATUS - Show status\r\n");
        CMD_SendResponse("  DEBUG - Show debug info\r\n");
        CMD_SendResponse("  STREAM ON/OFF - Data stream\r\n");
        CMD_SendResponse("  PID V/I <Kp> <Ki> <Kd> - Set PID\r\n");
        CMD_SendResponse("  STOP - Stop motor\r\n");
        CMD_SendResponse("  CALIB [V] - Calibrate encoder\r\n");
        return;
    }

    snprintf(response, sizeof(response), "ERR: Unknown '%s'. Type HELP\r\n", cmd_str);
    CMD_SendResponse(response);
}
