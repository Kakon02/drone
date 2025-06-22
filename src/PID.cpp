#include "pid.h"

float pidCompute(float error,
                 float kp, float ki, float kd,
                 float dt,
                 float outMax,
                 PIDState &state)
{
    // ----- P
    float pTerm = kp * error;

    // ----- I  (trapezoidal integration)
    float iTerm = state.prevIterm + ki * (error + state.prevError) * 0.5f * dt;
    iTerm = constrain(iTerm, -outMax, outMax);

    // ----- D
    float dTerm = kd * (error - state.prevError) / dt;

    // ----- Combine & clamp
    float out = pTerm + iTerm + dTerm;
    out = constrain(out, -outMax, outMax);

    // ----- Save state for next call
    state.prevError = error;
    state.prevIterm = iTerm;
    return out;
}
