#include "position.h"
#include <math.h>

#define RPM_TO_RAD (2.0f * PI / 60.0f)

PositionEstimator::PositionEstimator(float wheelRadius,
                                     float wheelBase, HiWonderMotors &car, PIDController &pidMotor1, PIDController &pidMotor2)
{
    R = wheelRadius;
    W = wheelBase;
    x_prev = 0.0f;
    y_prev = 0.0f;
    total_distance = 0.0f;
    total_distance_integral = 0.0f;
    segment_distance=0.0f;
    direccion = 0;
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

    state.theta = yawRad;   // ← ORIENTACION REAL

    state.x += v * cos(state.theta) * dt;
    state.y += v * sin(state.theta) * dt;

    total_distance_integral += sqrt( (state.x - x_prev)*(state.x - x_prev) + (state.y - y_prev)*(state.y - y_prev) );

    x_prev = state.x;
    y_prev = state.y;

    total_distance += v * dt;

    segment_distance += v * dt;
}


Position PositionEstimator::getPosition() const
{
    return state;
}


float PositionEstimator::getDistance() {
    return total_distance;
}

float PositionEstimator::getDistanceIntegral() {
    return total_distance_integral;
}

int PositionEstimator::getDireccion() {
    return direccion;
}

void PositionEstimator::setDireccion(int dir) {
    direccion = dir;
}

bool PositionEstimator::move() const {
    return moving;
}

bool PositionEstimator::isFinished() const {
    return finish;
}

void PositionEstimator::setMove(bool moving) {
    this->moving = moving;
}

void PositionEstimator::setFinished(bool finish) {
    this->finish = finish;
}

void PositionEstimator::reset_segment_distance() {
    segment_distance=0.0f;
}


float PositionEstimator::getSegmentDistance() {
    return segment_distance;
}

bool PositionEstimator::getChangeSpeed() {
    return change_speed;
}

void PositionEstimator::setChangeSpeed(bool change) {
    this->change_speed = change;
}

float PositionEstimator::getSpeed() {
    return speed;
}

void PositionEstimator::setSpeed(float speed) {
    this->speed = speed;
}