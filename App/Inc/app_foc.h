//
// Created by linglitel on 2026/1/30.
//

#ifndef ZEROFORM_FOC_APP_FOC_H
#define ZEROFORM_FOC_APP_FOC_H

#include <stdint.h>
#include "app_pid.h"

// 前向声明，避免循环依赖
void Encoder_Update(void);
void Current_Update(void);

#define FOC_CONTROL_PERIOD (1.0f/10000.0f)

typedef enum {
    IDLE,
    Current, //not recommend
    Velocity,
    Position,
} FOC_Mode;

typedef struct {
    FOC_Mode mode;

    int8_t direction; //For MT6835 Encoder,it should be -1;
    uint8_t pairs;

    float mechanical_angle; // Current mechanical position (0 to 2PI) (rad)
    float electrical_angle; // Current electrical phase for FOC (0 to 2PI) (rad)
    float electrical_angle_offset; // Calibration offset (0 to 2PI) (rad)

    float mechanical_velocity; // Current mechanical velocity (rad/s)
    float electronic_velocity; // Current electrical velocity (rad/s)

    uint32_t mechanical_angle_previous; // Previous mechanical angle (raw)
    float mechanical_velocity_previous;
    float electronic_velocity_previous; // Previous mechanical velocity (rad/s)

    // Internal state variables for control loops
    float Iq_prev;                // Previous Iq for low-pass filter
    uint8_t vel_loop_counter;     // Velocity loop decimation counter

    float Vbus;
    float Vq; // d-axis voltage (V)
    float Vd; // q-axis voltage (V)

    float Ud; // d-axis modulation
    float Uq; // q-axis modulation

    float Id;
    float Iq;

    float Ia;
    float Ib;
    float Ic;

    float Iq_target; //为什么要加它呢?我不知道
    float Id_target; //

    float velocity_target;
    float position_target;


    PID_Controller pid_id;
    PID_Controller pid_iq;
    PID_Controller pid_velocity;
    PID_Controller pid_position;
} FOC_t;

extern FOC_t FOC;

void FOC_AlignSensor(float vd);

void FOC_Velocity_Loop(void);

void FOC_SetPhaseVoltage(float vd, float vq, float electronic_angle);


#endif //ZEROFORM_FOC_APP_FOC_H
