#ifndef ESP_WEB_SERVER_H
#define ESP_WEB_SERVER_H

#include <WebServer.h>

class ESPWebServer
{
public:
    ESPWebServer();               // Constructor
    void begin();                 // Inicia WiFi, LittleFS y servidor
    void loop();                  // Maneja peticiones entrantes
    void updateSensor(float yaw); // Actualiza valor del sensor

private:
    WebServer server;
    float yaw_value;

    void mountLittleFS();         // Montar LittleFS
    void setupRoutes();           // Configurar rutas
};

#endif
