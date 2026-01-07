#include "position.h"
#include <math.h>

#define RPM_TO_RAD (2.0f * PI / 60.0f)

PositionEstimator::PositionEstimator(float wheelRadius,
                                     float wheelBase)
{
    R = wheelRadius;
    W = wheelBase;
    reset();
}

void PositionEstimator::reset(float x,
                              float y,
                              float theta)
{
    state.x = x;
    state.y = y;
    state.theta = theta;
}

void PositionEstimator::updateFromRPM(float rpmRight,
                                      float rpmLeft,
                                      float dt,
                                      float yawRad)   // ← viene del MPU
{
    float wR = rpmRight * RPM_TO_RAD;
    float wL = rpmLeft  * RPM_TO_RAD;

    float vR = wR * R;
    float vL = wL * R;

    float v = (vR + vL) * 0.5f;

    state.theta = yawRad;   // ← ORIENTACIÓN REAL

    state.x += v * cos(state.theta) * dt;
    state.y += v * sin(state.theta) * dt;
}


Position PositionEstimator::getPosition() const
{
    return state;
}
