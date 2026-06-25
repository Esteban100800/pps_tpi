#include "servo_control.h"



ServoControl::ServoControl(int pin) : _pin(pin), _currentAngle(0) 
{

}

void ServoControl::write(int angle)
{
   _servo.write(angle); 
   _currentAngle = angle; 
}

int ServoControl::read()
{
  return _currentAngle;
}


void ServoControl::begin()
{
   _servo.attach(_pin);
}


void ServoControl::move()
{
  delay(50);
  _servo.write(90);
  delay(500);
  _servo.write(0);
  delay(500);
  _servo.write(90);
  delay(500);
}