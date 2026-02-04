#include "esp_web_server.h"
#include <WiFi.h>
#include <LittleFS.h>

ESPWebServer::ESPWebServer(PIDController &pid_servo, PIDController &pid_motor, PIDController &pid_motor2, PositionEstimator &odom, DubinsPlanner &planner)
    : pid_servo(pid_servo), pid_motor(pid_motor),
      pid_motor2(pid_motor2), odom(odom), planner(planner), server(80), yaw_value(0.0f), motor_RPM(0.0f), motor2_RPM(0.0f) {}

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
        if (Serial)
            Serial.println("Error montando LittleFS");
    }
    else
    {
        if (Serial)
            Serial.println("LittleFS montado correctamente");
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
    Position p = odom.getPosition();

    float err_servo = pid_servo.getError();
    float err_motor = pid_motor.getError();
    float err_motor2 = pid_motor2.getError();
    
    float deriv_servo = pid_servo.getRealDerivative();
    float deriv_motor = pid_motor.getRealDerivative();
    float deriv_motor2 = pid_motor2.getRealDerivative();
    
    float integ_servo = pid_servo.getIntegral();
    float integ_motor = pid_motor.getIntegral();
    float integ_motor2 = pid_motor2.getIntegral();

    String json = "{"
    "\"yaw\":" + String(yaw_value, 2) +
    ",\"motor_RPM\":" + String(motor_RPM, 2) +
    ",\"motor2_RPM\":" + String(motor2_RPM, 2) +
    ",\"x\":" + String(p.x, 3) +
    ",\"y\":" + String(p.y, 3) +
    ",\"setpoint_servo\":" + String(pid_servo.getSetpoint(), 2) +
    ",\"setpoint_motor\":" + String(pid_motor.getSetpoint(), 2) +
    ",\"setpoint_motor2\":" + String(pid_motor2.getSetpoint(), 2) +
    ",\"err_servo\":" + String(err_servo, 3) +
    ",\"err_motor\":" + String(err_motor, 3) +
    ",\"err_motor2\":" + String(err_motor2, 3) +
    ",\"deriv_servo\":" + String(deriv_servo, 3) +
    ",\"deriv_motor\":" + String(deriv_motor, 3) +
    ",\"deriv_motor2\":" + String(deriv_motor2, 3) +
    ",\"integ_servo\":" + String(integ_servo, 3) +
    ",\"integ_motor\":" + String(integ_motor, 3) +
    ",\"integ_motor2\":" + String(integ_motor2, 3) +
    "}";
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

    server.on("/images/robot.png", HTTP_GET, [this]()
              {
    File file = LittleFS.open("/images/robot.png", "r");
    if (file) {
        server.streamFile(file, "image/png");
        file.close();
    } else {
        server.send(404, "text/plain", "Imagen no encontrada");
    } });

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

                  server.send(200, "text/plain", "Parámetros PID actualizados");
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

                   server.send(200, "text/plain", "Parámetros PID Servo actualizados");
              });

    // Endpoints para actualizar setpoints
    server.on("/update_setpoint_servo", HTTP_GET, [this]()
              {
    if (server.hasArg("setpoint")) {
        float setpoint = server.arg("setpoint").toFloat();
        pid_servo.setSetpoint(setpoint);
        
        /*Serial.printf("Setpoint Servo actualizado: %.2f\n", setpoint);*/
        server.send(200, "text/plain", "Setpoint Servo actualizado");
    } else {
        //server.send(400, "text/plain", "Parámetro setpoint faltante");
    } });

    server.on("/update_setpoint_motor", HTTP_GET, [this]()
              {
    if (server.hasArg("setpoint")) {
        float setpoint = server.arg("setpoint").toFloat();
        pid_motor.setSetpoint(setpoint);
        
        //Serial.printf("Setpoint Motor actualizado: %.2f\n", setpoint);
        server.send(200, "text/plain", "Setpoint Motor actualizado");
    } else {
    server.send(400, "text/plain", "Parámetro setpoint faltante");
    } });

    // Endpoints para el motor2
    server.on("/update_pid_motor2", HTTP_GET, [this]()
              {
    if (server.hasArg("kp")) pid_motor2.setKp(server.arg("kp").toFloat());
    if (server.hasArg("ki")) pid_motor2.setKi(server.arg("ki").toFloat());
    if (server.hasArg("kd")) pid_motor2.setKd(server.arg("kd").toFloat());

    pid_motor2.reset();

    server.send(200, "text/plain", "PID Motor2 actualizado"); });

    server.on("/update_setpoint_motor2", HTTP_GET, [this]()
              {
    if (server.hasArg("setpoint")) {
        float setpoint = server.arg("setpoint").toFloat();
        pid_motor2.setSetpoint(setpoint);
        server.send(200, "text/plain", "Setpoint Motor2 actualizado");
    } });

    server.on("/set_goal", HTTP_GET, [this]()
              {
    if (!server.hasArg("x") || !server.hasArg("y") || !server.hasArg("theta")) {
        server.send(400, "text/plain", "Faltan parametros");
        return;
    }

    pendingGoal = {
        server.arg("x").toFloat(),
        server.arg("y").toFloat(),
        server.arg("theta").toFloat() * DEG_TO_RAD
    };

    newGoalRequested = true;

    server.send(200, "text/plain", "Nuevo setpoint aceptado"); });

    server.on("/update_speed", HTTP_GET, [this]() {
    if (!server.hasArg("speed")) {
        server.send(400, "text/plain", "Parametro speed faltante");
        return;
    }

    float speed = server.arg("speed").toFloat();

    if (speed < 0.0f || speed > 1.0f) {
        server.send(400, "text/plain", "Velocidad fuera de rango");
        return;
    }

    requestedSpeed = speed;
    speedRequest = true;

    server.send(200, "text/plain", "Velocidad actualizada");
});

}

bool ESPWebServer::newGoalAvailable() const
{
    return newGoalRequested;
}

Pose ESPWebServer::getPendingGoal()
{
    newGoalRequested = false;
    return pendingGoal;
}

bool ESPWebServer::isSpeedRequested() 
{
    return speedRequest;
}

float ESPWebServer::getRequestedSpeed() 
{
    speedRequest = false;
    return requestedSpeed;
}