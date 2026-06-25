#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float p, float i, float d)
    : kp(p), ki(i), kd(d), integral(0), prevError(0), setpoint(0), lastPrintTime(0), dt(0.02), delta_error(0.08), error(0) {}



void PIDController::reset()
{
  integral = 0;
  prevError = 0;
  error = 0;
  real_derivative = 0;
}

float PIDController::compute(float value, float max_integral)
{
  error = setpoint - value;

  if (error > delta_error || error < -delta_error)
  {
    integral += error * dt;
  }
  if (integral > max_integral)
    integral = max_integral; // Anti-windup
  if (integral < -max_integral)
    integral = -max_integral; // Anti-windup

  float derivative = (error - prevError) / dt;
  prevError = error;

  real_derivative = derivative * kd;

  float output = kp * error + ki * integral + real_derivative;

  return output;
}


void PIDController::setSetpoint(float s)
{
  setpoint = s;
  // reset();
}

void PIDController::setKi(float ki)
{
  this->ki = ki;
}

void PIDController::setKd(float kd)
{
  this->kd = kd;
}

void PIDController::setKp(float kp)
{
  this->kp = kp;
}

float PIDController::getKp() const
{
  return kp;
}
float PIDController::getKi() const
{
  return ki;
}
float PIDController::getKd() const
{
  return kd;
}

float PIDController::getSetpoint() const
{
  return setpoint;
}


float PIDController::getError() {
    return error;
}

float PIDController::getRealDerivative() {
    return real_derivative;
}

float PIDController::getIntegral() {
    return integral;
}