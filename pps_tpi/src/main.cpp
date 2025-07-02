#include <Arduino.h>
#include <Wire.h>
#include <magnetometer.h> 
#include <accelerometer.h>

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);    // Pin 2 es el LED incorporado en ESP32
  digitalWrite(2, HIGH); // Encender LED para indicar inicio del setup
  delay(2000);
  config_gyro(); // Configurar el giroscopio
  calibrate_gyro();
  calibrate_mag(); // Calibrar el magnetómetro
  config_magnetometer(); // Configurar el magnetómetro
  // xTaskCreate(calibration, "MiTarea", 2048, NULL, 1, NULL);
}

void loop()
{
  gyro_signals();
  print_status();
  mag_signals();
  // print_mag_status();
  print_heading();
  delay(100);
}
