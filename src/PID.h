#pragma once
#ifndef PID_MODULE_H
#define PID_MODULE_H
#include <Arduino.h>

struct PIDState {
    float prevError  = 0.0f;
    float prevIterm  = 0.0f;
};

float pidCompute(float   error,
                 float   kp, float ki, float kd,
                 float   dt,
                 float   outMax,
                 PIDState &state);

inline void pidReset(PIDState &s) { s.prevError = s.prevIterm = 0.0f; }

#endif /* PID_MODULE_H */