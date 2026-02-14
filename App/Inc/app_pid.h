//
// Created by linglitel on 2026/1/30.
//

#ifndef ZEROFORM_FOC_APP_PID_H
#define ZEROFORM_FOC_APP_PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float dt;             // Control loop period (s)
    float output_limit;
    float integral_limit;

    float error;

    float target;
    float integral;
    float last_error;
    float output;
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float dt, float output_limit, float integral_limit);

float PID_Update(PID_Controller *pid, float feedback);

/**
 * @brief   PID更新函数（传入误差）
 * @param   pid PID控制器指针
 * @param   error 误差值 (target - feedback)
 * @return  PID输出值
 */
float PID_Update_Error(PID_Controller *pid, float error);

#endif //ZEROFORM_FOC_APP_PID_H
