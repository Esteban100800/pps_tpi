#ifndef ESP_WEB_SERVER_H
#define ESP_WEB_SERVER_H

#include <WebServer.h>
#include "PIDController.h"
#include "Motor.h"

class ESPWebServer
{
private:
    Motor &motor;
    PIDController &pid_servo;
    PIDController &pid_motor;
    WebServer server;
    float yaw_value;
    float motor_RPM;
    Motor &motor2;
    PIDController &pid_motor2;
    float motor2_RPM;

    void mountLittleFS(); // Montar LittleFS
    void setupRoutes();   // Configurar rutas
public:
    ESPWebServer(Motor& m, PIDController& pid_servo, PIDController& pid_motor, Motor& m2, PIDController& pid_motor2); // Constructor por referencia
    void begin();                                                            // Inicia WiFi, LittleFS y servidor
    void loop();                                                             // Maneja peticiones entrantes
    void updateSensor(float yaw);                                            // Actualiza valor del sensor
    void updateMotor(float RPM);                                             // Actualiza valor del motor
    void updateMotor2(float RPM);                                            // Actualiza valor del motor2
};

#endif
