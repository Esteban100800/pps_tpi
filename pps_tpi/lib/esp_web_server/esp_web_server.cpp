#include "esp_web_server.h"
#include <WiFi.h>
#include <LittleFS.h>

ESPWebServer::ESPWebServer() : server(80), yaw_value(0.0f) {}

void ESPWebServer::begin()
{
    // Configurar pin del LED como salida
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW); // Inicialmente apagado
    
    WiFi.softAP("ESP32-AP", "12345678");

    mountLittleFS();
    setupRoutes();

    server.begin();
    Serial.println("Servidor iniciado");
}

void ESPWebServer::loop()
{
    server.handleClient();
}

void ESPWebServer::updateSensor(float yaw)
{
    yaw_value = yaw;
}

void ESPWebServer::mountLittleFS()
{
    if (!LittleFS.begin())
    {
        Serial.println("Error montando LittleFS");
    }
    else
    {
        Serial.println("LittleFS montado correctamente");
    }
}

void ESPWebServer::setupRoutes()
{
server.on("/", HTTP_GET, [this]() 
{
    File file = LittleFS.open("/index.html", "r");
    server.streamFile(file, "text/html");  
    file.close();
});
server.on("/data", HTTP_GET, [this]() 
{
    String json = "{\"yaw\":" + String(yaw_value, 2) + "}";
    server.send(200, "application/json", json);
});
server.on("/chart.js", HTTP_GET, [this]() {
    File file = LittleFS.open("/chart.js", "r");
    server.streamFile(file, "application/javascript");
    file.close();
});

server.on("/led/on", HTTP_GET, [this]() {
    digitalWrite(2, HIGH);
    Serial.println("LED encendido");
    server.send(200, "text/plain", "LED encendido");
});
server.on("/led/off", HTTP_GET, [this]() {
    digitalWrite(2, LOW);
    Serial.println("LED apagado");
    server.send(200, "text/plain", "LED apagado");
});
}