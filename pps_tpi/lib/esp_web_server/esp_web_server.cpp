#include "esp_web_server.h"
#include <WiFi.h>
#include <LittleFS.h>

ESPWebServer::ESPWebServer(Motor &m, PIDController &pid_servo, PIDController &pid_motor,
                           Motor &m2, PIDController &pid_motor2)
    : motor(m), pid_servo(pid_servo), pid_motor(pid_motor), motor2(m2),
      pid_motor2(pid_motor2), server(80), yaw_value(0.0f), motor_RPM(0.0f), motor2_RPM(0.0f) {}

void ESPWebServer::begin()
{
    // Configurar pin del LED como salida
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW); // Inicialmente apagado

    WiFi.softAP("ESP32-AP", "12345678");

    mountLittleFS();
    setupRoutes();

    server.begin();
   // Serial.println("Servidor iniciado");
}

void ESPWebServer::loop()
{
    server.handleClient();
}

void ESPWebServer::updateSensor(float yaw)
{
    yaw_value = yaw;
}

void ESPWebServer::updateMotor(float RPM)
{
    motor_RPM = RPM;
}

void ESPWebServer::updateMotor2(float RPM)
{
    motor2_RPM = RPM;
}

void ESPWebServer::mountLittleFS()
{
    if (!LittleFS.begin())
    {
        if (Serial) Serial.println("Error montando LittleFS");
    }
    else
    {
        if (Serial) Serial.println("LittleFS montado correctamente");
    }
}

void ESPWebServer::setupRoutes()
{
    server.on("/", HTTP_GET, [this]()
              {
    File file = LittleFS.open("/index.html", "r");
    server.streamFile(file, "text/html");  
    file.close(); });
    server.on("/data", HTTP_GET, [this]()
              {
    String json = "{\"yaw\":" + String(yaw_value, 2) + 
                  ",\"motor_RPM\":" + String(motor_RPM, 2) + 
                  ",\"motor2_RPM\":" + String(motor2_RPM, 2) + 
                  ",\"setpoint_servo\":" + String(pid_servo.getSetpoint(), 2) + 
                  ",\"setpoint_motor\":" + String(pid_motor.getSetpoint(), 2) + 
                  ",\"setpoint_motor2\":" + String(pid_motor2.getSetpoint(), 2) + "}";
    server.send(200, "application/json", json); });
    server.on("/chart.js", HTTP_GET, [this]()
              {
    File file = LittleFS.open("/chart.js", "r");
    server.streamFile(file, "application/javascript");
    file.close(); });

    server.on("/css/styles.css", HTTP_GET, [this]()
              {
    File file = LittleFS.open("/css/styles.css", "r");
    server.streamFile(file, "text/css");
    file.close(); });

    server.on("/js/app.js", HTTP_GET, [this]()
              {
    File file = LittleFS.open("/js/app.js", "r");
    server.streamFile(file, "application/javascript");
    file.close(); });

    server.on("/finisher-header.es5.min.js", HTTP_GET, [this]()
              {
    File file = LittleFS.open("/finisher-header.es5.min.js", "r");
    server.streamFile(file, "application/javascript");
    file.close(); });

    server.on("/led/on", HTTP_GET, [this]()
              {
    digitalWrite(2, HIGH);
    if (Serial) Serial.println("LED encendido");
    server.send(200, "text/plain", "LED encendido"); });
    server.on("/led/off", HTTP_GET, [this]()
              {
    digitalWrite(2, LOW);
    if (Serial) Serial.println("LED apagado");
    server.send(200, "text/plain", "LED apagado"); });
    server.on("/update_pid", HTTP_GET, [this]()
              {
                  if (server.hasArg("kp"))
                      pid_motor.setKp(server.arg("kp").toFloat());
                  if (server.hasArg("ki"))
                      pid_motor.setKi(server.arg("ki").toFloat());
                  if (server.hasArg("kd"))
                      pid_motor.setKd(server.arg("kd").toFloat());

                  pid_motor.reset();

                  /*Serial.printf("PID Motor actualizado -> Kp: %.2f, Ki: %.2f, Kd: %.2f\n",
                                pid_motor.getKp(),
                                pid_motor.getKi(),
                                pid_motor.getKd());*/

                  // server.send(200, "text/plain", "Parámetros PID actualizados");
              });

    server.on("/update_pid_servo", HTTP_GET, [this]()
              {
                  if (server.hasArg("kp"))
                      pid_servo.setKp(server.arg("kp").toFloat());
                  if (server.hasArg("ki"))
                      pid_servo.setKi(server.arg("ki").toFloat());
                  if (server.hasArg("kd"))
                      pid_servo.setKd(server.arg("kd").toFloat());

                  /* Serial.printf("PID Servo actualizado -> Kp: %.2f, Ki: %.2f, Kd: %.2f\n",
                                 pid_servo.getKp(),
                                 pid_servo.getKi(),
                                 pid_servo.getKd());*/

                  // server.send(200, "text/plain", "Parámetros PID Servo actualizados");
              });

    // Endpoints para actualizar setpoints
    server.on("/update_setpoint_servo", HTTP_GET, [this]()
              {
    if (server.hasArg("setpoint")) {
        float setpoint = server.arg("setpoint").toFloat();
        pid_servo.setSetpoint(setpoint);
        
        /*Serial.printf("Setpoint Servo actualizado: %.2f\n", setpoint);*/
        //server.send(200, "text/plain", "Setpoint Servo actualizado");
    } else {
        //server.send(400, "text/plain", "Parámetro setpoint faltante");
    } });

    server.on("/update_setpoint_motor", HTTP_GET, [this]()
              {
    if (server.hasArg("setpoint")) {
        float setpoint = server.arg("setpoint").toFloat();
        pid_motor.setSetpoint(setpoint);
        
        //Serial.printf("Setpoint Motor actualizado: %.2f\n", setpoint);
       // server.send(200, "text/plain", "Setpoint Motor actualizado");
    } else {
       // server.send(400, "text/plain", "Parámetro setpoint faltante");
    } });

    // Endpoints para el motor2
    server.on("/update_pid_motor2", HTTP_GET, [this]()
              {
    if (server.hasArg("kp")) pid_motor2.setKp(server.arg("kp").toFloat());
    if (server.hasArg("ki")) pid_motor2.setKi(server.arg("ki").toFloat());
    if (server.hasArg("kd")) pid_motor2.setKd(server.arg("kd").toFloat());

    pid_motor2.reset();
              });

    server.on("/update_setpoint_motor2", HTTP_GET, [this]()
              {
    if (server.hasArg("setpoint")) {
        float setpoint = server.arg("setpoint").toFloat();
        pid_motor2.setSetpoint(setpoint);
    }
              });
}