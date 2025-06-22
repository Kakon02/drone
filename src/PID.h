#pragma once         // Totally fine only with pragma once, but example using include guards for compatibility with older compilers
#ifndef PID_MODULE_H // Unique identifier for the header file to prevent multiple inclusions
#define PID_MODULE_H
#include <Arduino.h>

struct PIDState
{
    float prevError = 0.0f;
    float prevIterm = 0.0f;
};

float pidCompute(float error,
                 float kp, float ki, float kd,
                 float dt,
                 float outMax,
                 PIDState &state);

inline void pidReset(PIDState &s) { s.prevError = s.prevIterm = 0.0f; }

#endif /* PID_MODULE_H */