//
// Created by linglitel on 2026/1/30.
//

#include "app_pid.h"



void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd,
              float dt, float output_limit, float integral_limit) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->dt = dt;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;

    pid->target = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

float PID_Update(PID_Controller *pid, float feedback) {
    float error = pid->target - feedback;

    pid->error = error;

    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float derivative = (error - pid->last_error) / pid->dt;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    pid->last_error = error;
    pid->output = output;

    return output;
}
