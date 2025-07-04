#pragma once

#include <WebServer.h>
#include "../accelerometer/accelerometer.h"
#include "../magnetometer/magnetometer.h"


class web_server {
public:
    web_server(Accelerometer& accel, Magnetometer& mag);           // Constructor
    void begin();                 // Inicia el servidor
    void handleClient();         // Procesa peticiones entrantes

private:
    WebServer server;
    int output2;
    String output2State;
    Accelerometer& accel;
    Magnetometer& mag;

    void setupRoutes();          // Configura las rutas del servidor
    void handleRoot();
    void handleGPIO2On();
    void handleGPIO2Off();
    void return_data();          // Devuelve datos de los sensores
};


