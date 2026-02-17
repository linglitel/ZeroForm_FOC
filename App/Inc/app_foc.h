/**
 * @file    app_foc.h
 * @brief   ZeroForm FOC 磁场定向控制模块
 * @author  linglitel
 * @date    2026/1/30
 * 
 * @details 本模块实现FOC (Field Oriented Control) 磁场定向控制算法，
 *          包括电流环、速度环、位置环控制，以及SVPWM调制。
 */

#ifndef ZEROFORM_FOC_APP_FOC_H
#define ZEROFORM_FOC_APP_FOC_H

#include <stdint.h>
#include "app_pid.h"

/* 前向声明，避免循环依赖 */
void Encoder_Update(void);

void Current_Update(void);

/**
 * @brief FOC控制周期 (秒)
 * @note  10kHz控制频率
 */
#define FOC_CONTROL_PERIOD (1.0f/10000.0f)

/**
 * @brief FOC控制模式枚举
 */
typedef enum {
    IDLE, /**< 空闲模式，电机自由 */
    Current, /**< 电流/力矩模式 */
    Velocity, /**< 速度模式 */
    Position, /**< 位置模式 */
} FOC_Mode;

/**
 * @brief FOC控制器主结构体
 * @details 包含所有FOC控制所需的状态变量和参数
 */
typedef struct {
    FOC_Mode mode; /**< 当前控制模式 */

    int8_t direction; /**< 电机方向 */
    uint8_t pairs; /**< 电机极对数 */

    float mechanical_angle; /**< 当前机械角度 (0 ~ 2π rad) */
    float electrical_angle; /**< 当前电角度 (0 ~ 2π rad) */
    float electrical_angle_offset; /**< 电零角偏移 (校准值) */

    float mechanical_velocity; /**< 当前机械速度 (rad/s) */
    float electronic_velocity; /**< 当前电速度 (rad/s) */

    uint32_t mechanical_angle_previous; /**< 上一次机械角度 (原始值) */
    float mechanical_velocity_previous; /**< 上一次机械速度 */
    float electronic_velocity_previous; /**< 上一次电速度 */

    /* 控制环内部状态变量 */
    float Iq_prev; /**< 上一次Iq值 (用于低通滤波) */
    uint8_t vel_loop_counter; /**< 速度环分频计数器 */

    float Vbus; /**< 母线电压 (V) */
    float Vq; /**< q轴电压 (V) */
    float Vd; /**< d轴电压 (V) */

    float Ud; /**< d轴调制比 */
    float Uq; /**< q轴调制比 */

    float Id; /**< d轴电流 (A) */
    float Iq; /**< q轴电流 (A) */

    float Ia; /**< A相电流 (A) */
    float Ib; /**< B相电流 (A) */
    float Ic; /**< C相电流 (A) */

    float duty_a;
    float duty_b;
    float duty_c;

    float Iq_target; /**< Iq目标值 (A) */
    float Id_target; /**< Id目标值 (A) */

    float velocity_target; /**< 速度目标值 (rad/s) */
    float position_target; /**< 位置目标值 (rad) */

    /* 前馈控制参数 */
    float velocity_ff; /**< 速度前馈值 (rad/s) */
    float current_ff; /**< 电流前馈值 (A) */
    float Kff_velocity; /**< 速度前馈增益 */

    /* PID控制器 */
    PID_Controller pid_id; /**< Id电流环PID */
    PID_Controller pid_iq; /**< Iq电流环PID */
    PID_Controller pid_velocity; /**< 速度环PID */
    PID_Controller pid_position; /**< 位置环PID */
} FOC_t;

/** @brief FOC控制器全局实例 */
extern FOC_t FOC;

/**
 * @brief FOC调试信息结构体
 */
typedef struct {
    float velocity_error; /**< 速度误差 (rad/s) */
    float velocity_output; /**< 速度环输出 (A) */
    float velocity_measured; /**< 速度环调用时的实际速度 */
    float position_error; /**< 位置误差 (rad) */
    float position_output; /**< 位置环输出 (A) */
    uint32_t debug_counter; /**< 调试计数器 */
    uint8_t stream_enabled; /**< 数据流输出使能 */
} FOC_Debug_t;

/** @brief FOC调试信息全局实例 */
extern FOC_Debug_t FOC_Debug;

//==============================================================================
// 函数声明
//==============================================================================

/**
 * @brief   电零角校准
 * @param   vd 校准电压 (V)
 * @note    校准时电机会锁定在零位，然后读取编码器角度
 */
void FOC_AlignSensor(float vd);

/**
 * @brief   速度环控制
 * @note    在ADC中断中调用，频率为1kHz
 */
void FOC_Velocity_Loop(void);

/**
 * @brief   位置环控制
 * @note    在ADC中断中调用，频率为1kHz
 */
void FOC_Position_Loop(void);

/**
 * @brief   电流环控制
 * @note    在ADC中断中调用，频率为10kHz
 */
void FOC_Current_Loop(void);

/**
 * @brief   设置相电压 (SVPWM调制)
 * @param   vd d轴电压 (V)
 * @param   vq q轴电压 (V)
 * @param   electronic_angle 电角度 (rad)
 */
void FOC_SetPhaseVoltage(float vd, float vq, float electronic_angle);

#endif //ZEROFORM_FOC_APP_FOC_H
