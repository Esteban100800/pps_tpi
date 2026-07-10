#ifndef ESP_WEB_SERVER_H
#define ESP_WEB_SERVER_H

#include <WebServer.h>
#include "PIDController.h"
#include "Motor.h"
#include "car_functions.h"
#include "dubin.h"
#include "position.h"
#include "differential.h"


class ESPWebServer
{
private:
    PIDController &pid_servo;
    PIDController &pid_motor;
    WebServer server;
    float yaw_value;
    float motor_RPM;
    PIDController &pid_motor2;
    float motor2_RPM;
    PositionEstimator &odom;
    DubinsPlanner &planner;
    bool newGoalRequested = false;

    volatile bool speedRequest = false;
    volatile float requestedSpeed = 0.0f;

    Pose pendingGoal;

    volatile bool immediateGoalRequested = false;
    Pose immediateGoal;

    String algorithmMode = "dubins"; // "dubins" o "rs"

    void mountLittleFS(); // Montar LittleFS
    void setupRoutes();   // Configurar rutas
public:
    ESPWebServer(PIDController& pid_servo, PIDController& pid_motor, PIDController& pid_motor2,
                PositionEstimator &odom, DubinsPlanner &planner); // Constructor por referencia
    void begin();                                                            // Inicia WiFi, LittleFS y servidor
    void loop();                                                             // Maneja peticiones entrantes
    void updateSensor(float yaw);                                            // Actualiza valor del sensor
    void updateMotor(float RPM);                                             // Actualiza valor del motor
    void updateMotor2(float RPM);                                            // Actualiza valor del motor2
    bool getRequested() const;                                          // Verifica si hay un nuevo objetivo (encolar)
    Pose getPendingGoal();                                                  // Obtiene el objetivo pendiente (encolar)
    void setRequested(); // Marca que se ha atendido la solicitud de nuevo objetivo

    bool getImmediateRequested() const;  // Verifica si hay un objetivo "enviar ahora" (interrumpe)
    Pose getImmediateGoal();             // Obtiene el objetivo "enviar ahora"
    void setImmediateRequested();        // Marca que se ha atendido la solicitud inmediata

    bool isSpeedRequested() ; // Verifica si hay una solicitud de velocidad
    float getRequestedSpeed(); // Obtiene la velocidad solicitada

    bool isDubins()     const { return algorithmMode == "dubins"; }
    bool isReedsShepp() const { return algorithmMode == "rs"; }
};

#endif
