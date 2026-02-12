//
// Created by linglitel on 2026/2/12.
// 串口命令解析模块
//

#ifndef ZEROFORM_FOC_APP_CMD_H
#define ZEROFORM_FOC_APP_CMD_H

#include "main.h"
#include "app_foc.h"

// 命令缓冲区大小
#define CMD_BUFFER_SIZE 64

// 命令结构体
typedef struct {
    uint8_t rx_buffer[CMD_BUFFER_SIZE];  // 接收缓冲区
    uint8_t rx_index;                     // 当前接收位置
    uint8_t rx_byte;                      // 单字节接收缓冲
    uint8_t cmd_ready;                    // 命令就绪标志
} CMD_Handler_t;

extern CMD_Handler_t CMD;

/**
 * @brief 初始化命令处理模块
 * @param huart UART句柄
 */
void CMD_Init(UART_HandleTypeDef *huart);

/**
 * @brief 处理接收到的命令（在主循环中调用）
 */
void CMD_Process(void);

/**
 * @brief UART接收完成回调（需要在stm32g4xx_it.c中调用）
 * @param huart UART句柄
 */
void CMD_UART_RxCallback(UART_HandleTypeDef *huart);

/**
 * 命令格式说明:
 * 
 * 1. 设置模式:
 *    MODE V        - 切换到速度模式
 *    MODE P        - 切换到位置模式
 *    MODE I        - 切换到空闲模式
 * 
 * 2. 设置速度 (速度模式下):
 *    VEL 10.5      - 设置目标速度为 10.5 rad/s
 * 
 * 3. 设置位置 (位置模式下):
 *    POS A 90      - 绝对位置: 转到 90 度 (会转换为弧度)
 *    POS R 45      - 相对位置: 从当前位置转 45 度
 *    POS A 1.57R   - 绝对位置: 转到 1.57 弧度 (带R后缀表示弧度)
 *    POS R 0.5R    - 相对位置: 从当前位置转 0.5 弧度
 * 
 * 4. 设置逻辑零点:
 *    ZERO          - 将当前位置设为逻辑零点
 *    ZERO 45       - 将当前位置偏移 45 度后设为逻辑零点
 * 
 * 5. 查询状态:
 *    STATUS        - 查询当前状态
 * 
 * 6. 停止:
 *    STOP          - 停止电机 (切换到空闲模式)
 * 
 * 所有命令以换行符 '\n' 或回车符 '\r' 结束
 */

#endif //ZEROFORM_FOC_APP_CMD_H
