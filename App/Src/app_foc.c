//
// Created by linglitel on 2026/1/30.
//
#include "app_foc.h"
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "app_motor.h"
#include "app_utils.h"
#include "app_cordic.h"
#include "app_encoder.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
extern SPI_HandleTypeDef hspi1;

// 速度/位置环分频系数: 10kHz / 10 = 1kHz
#define VEL_POS_LOOP_DIV    10

FOC_t FOC;
FOC_Debug_t FOC_Debug;

void FOC_SetPhaseVoltage(const float vd, const float vq, const float electronic_angle) {
    if (isnan(vd) || isnan(vq) || isnan(electronic_angle)) {
        Motor_Set_Duty(0, 0, 0);
        return;
    }
    float v_bus = FOC.Vbus;
    float v_base = (2.0f / 3.0f) * v_bus;
    float v_norm_scale = 1.0f / v_base;

    float vd_norm = vd * v_norm_scale;
    float vq_norm = vq * v_norm_scale;

    float v_mag = sqrtf(vd_norm * vd_norm + vq_norm * vq_norm);
    if (v_mag > 0.866f) {
        const float scale = 0.866f / v_mag;
        vd_norm *= scale;
        vq_norm *= scale;
    }

    float cos_theta, sin_theta;
    CORDIC_SinCos(electronic_angle, &sin_theta, &cos_theta);

    float valpha_norm = vd_norm * cos_theta - vq_norm * sin_theta;
    float vbeta_norm = vd_norm * sin_theta + vq_norm * cos_theta;

    float va_raw = valpha_norm;
    float vb_raw = -0.5f * valpha_norm + 0.8660254f * vbeta_norm;
    float vc_raw = -0.5f * valpha_norm - 0.8660254f * vbeta_norm;

    float v_max = fmaxf(va_raw, fmaxf(vb_raw, vc_raw));
    float v_min = fminf(va_raw, fminf(vb_raw, vc_raw));
    float v_neutral = -0.5f * (v_max + v_min);

    float va_final = va_raw + v_neutral;
    float vb_final = vb_raw + v_neutral;
    float vc_final = vc_raw + v_neutral;

    float duty_a = 0.5f * (va_final + 1.0f);
    float duty_b = 0.5f * (vb_final + 1.0f);
    float duty_c = 0.5f * (vc_final + 1.0f);

    Motor_Set_Duty(duty_a, duty_b, duty_c);
}

void FOC_AlignSensor(float vd) {
    __disable_irq();
    float angle_sum = 0.0f;
    uint8_t times = 5;
    
    // 先读取一次编码器，初始化mechanical_angle_previous
    Encoder_Update();
    
    FOC_SetPhaseVoltage(vd, 0, 0);
    DWT_Delay(1000000);  // 1秒，等待电机稳定到零位
    
    for (int i = 0; i < times; i++) {
        Encoder_Update();
        angle_sum += (float) FOC.direction * FOC.mechanical_angle * (float) FOC.pairs;
        DWT_Delay(10000);  // 10ms
    }
    FOC.electrical_angle_offset = _normalizeAngle(angle_sum / (float) times);
    FOC_SetPhaseVoltage(0, 0, 0);
    
    // 校准完成后，初始化mechanical_angle_previous为当前值
    // 这样下次Encoder_Update时速度计算不会出现跳变
    Encoder_Update();
    
    __enable_irq();
}

void FOC_Current_Loop(void) {
    float theta = FOC.electrical_angle;
    float cos_theta, sin_theta;
    CORDIC_SinCos(theta, &sin_theta, &cos_theta);
    float I_alpha = FOC.Ia;
    float I_beta = 0.57735026919f * (FOC.Ia + 2.0f * FOC.Ib);
    FOC.Id = I_alpha * cos_theta + I_beta * sin_theta;
    FOC.Iq = -I_alpha * sin_theta + I_beta * cos_theta;
    // Low-pass filter for Iq
    float alpha = 0.8f;
    FOC.Iq = alpha * FOC.Iq + (1.0f - alpha) * FOC.Iq_prev;
    FOC.Iq_prev = FOC.Iq;
    FOC.Id_target = 0.0f;
    FOC.pid_iq.target = FOC.Iq_target;
    FOC.pid_id.target = FOC.Id_target;
    FOC.Vd = PID_Update(&FOC.pid_id, FOC.Id);
    FOC.Vq = PID_Update(&FOC.pid_iq, FOC.Iq);
    FOC_SetPhaseVoltage(FOC.Vd, FOC.Vq, theta);
}

void FOC_Velocity_Loop(void) {
    // 先更新速度（使用累积的角度增量）
    Encoder_UpdateVelocity();
    
    float vel = FOC.mechanical_velocity;
    
    // 计算速度误差
    float error = FOC.velocity_target - vel;
    
    // 使用统一的PID_Update_Error接口
    float target_current = PID_Update_Error(&FOC.pid_velocity, error);
    
    // 保存调试信息（在中断中保存，确保数据一致性）
    FOC_Debug.velocity_measured = vel;
    FOC_Debug.velocity_error = error;
    FOC_Debug.velocity_output = target_current;
    
    FOC.Iq_target = target_current;
}

void FOC_Position_Loop(void) {
    float pos = FOC.mechanical_angle * (float) FOC.direction;

    // 计算位置误差并归一化到 [-π, π] 实现最短路径
    float error = FOC.position_target - pos;

    // 最短路径: 限制误差到 [-π, π] 范围
    while (error > 3.14159265f) error -= 6.28318530f;
    while (error < -3.14159265f) error += 6.28318530f;

    // 使用统一的PID_Update_Error接口
    float target_current = PID_Update_Error(&FOC.pid_position, error);
    
    // 保存调试信息
    FOC_Debug.position_error = error;
    FOC_Debug.position_output = target_current;
    
    FOC.Iq_target = target_current;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        Encoder_Update();
        Current_Update();
        if (FOC.mode == IDLE) {
            FOC.pid_iq.error = 0.0f;
            FOC.pid_iq.integral = 0.0f;
            FOC.pid_iq.output = 0.0f;
            FOC.pid_velocity.integral = 0.0f;
            FOC.pid_velocity.last_error = 0.0f;
            FOC.pid_position.integral = 0.0f;
            FOC.pid_position.last_error = 0.0f;
            FOC.Iq_target = 0.0f;
            FOC.Iq = 0.0f;
            FOC_SetPhaseVoltage(0, 0, FOC.electrical_angle);
            return;
        }

        if (++FOC.vel_loop_counter >= VEL_POS_LOOP_DIV) {
            FOC.vel_loop_counter = 0;
            if (FOC.mode == Velocity) {
                FOC_Velocity_Loop();
            }
            if (FOC.mode == Position) {
                FOC_Position_Loop();
            }
        }

        FOC_Current_Loop();
    }
}
