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
   int channel = _servo.attach(_pin);
   if (Serial) Serial.println("Canal asignado al servo: " + String(channel));
}


void ServoControl::move()
{
  if (Serial) Serial.println("Servo conectado al pin 33");
  delay(50);
  _servo.write(90);
  if (Serial) Serial.println("Servo en posición 90°");
  delay(500);
  _servo.write(0);
  if (Serial) Serial.println("Servo en posición 0°");
  delay(500);
  _servo.write(90);
  if (Serial) Serial.println("Servo en posición 90°");
  delay(500);
}