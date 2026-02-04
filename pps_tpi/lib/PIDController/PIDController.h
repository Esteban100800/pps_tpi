#pragma once
#include "Motor.h"

class PIDController {
  private:
    
    float setpoint;
    float integral, prevError, derivative;
    unsigned long lastPrintTime;
    float dt;
    float delta_error; // Umbral para el cambio de error
    float error;
    float real_derivative;
    float kp, ki, kd;

  public:
  
    PIDController( float p, float i, float d);
    void setSetpoint(float s);
    float compute(float value, float max_integral);
    void reset();
    float computeAngleError(float current, float target);
    float getKp() const;
    float getKi() const;
    float getKd() const;
    float getSetpoint() const;
    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    float getError();
    float getRealDerivative();
    float getIntegral();
};
