#pragma once

#include "Arduino.h"
#include <ESP32_Servo.h>

class ServoControl
{
public:
  ServoControl(int pin);
  void write(int angle);
  int read();
  void begin();
  void move();

private:
  int _pin;
  int _currentAngle;
  Servo _servo; 
};