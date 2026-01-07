#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float p, float i, float d)
    : kp(p), ki(i), kd(d), integral(0), prevError(0), setpoint(0), lastPrintTime(0), dt(0.02), delta_error(0.08) {}



void PIDController::reset()
{
  integral = 0;
  prevError = 0;
}

float PIDController::compute(float value, float max_integral)
{
  float error = setpoint - value;

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

  float real_derivative = derivative * kd;

  float output = kp * error + ki * integral + real_derivative;


  //unsigned long now = millis();
  /*if (now - lastPrintTime >= 100)
  {
    lastPrintTime = now;
    Serial.print("sensed:");
    Serial.print(value);
    Serial.print(",Setpoint:");
    Serial.print(setpoint);
    Serial.print(",Derivative:");
    Serial.print(real_derivative);
    Serial.print(",Integral:");
    Serial.print(integral);
    Serial.print(",ki:");
    Serial.print(ki);
    Serial.print(",Error:");
    Serial.println(error);
  }*/
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
