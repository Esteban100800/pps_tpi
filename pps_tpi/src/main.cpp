#include <Arduino.h>
#include <Wire.h>
#include <magnetometer.h> 
#include <accelerometer.h>
#include "web_server.h"

Accelerometer accel;
Magnetometer mag;
web_server server(accel, mag);  // pasás referencias
void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);    // Pin 2 es el LED incorporado en ESP32
  digitalWrite(2, HIGH); // Encender LED para indicar inicio del setup
  delay(2000);
  accel.begin(); // Configurar el giroscopio
  accel.calibrate();
  mag.begin(); // Configurar el magnetómetro
  mag.calibrate(); // Calibrar el magnetómetro

  // xTaskCreate(calibration, "MiTarea", 2048, NULL, 1, NULL);

  server.begin(); // Iniciar el servidor web
  Serial.println("Setup completo");
}

void loop()
{

  mag.update(); // Actualizar datos del magnetómetro
  accel.update(); // Actualizar datos del giroscopio
  delay(100);
  server.handleClient();
}
