#include "web_server.h"
#include <WiFi.h>

const char* ssid = "ESP32-AP";
const char* password = "12345678";

web_server::web_server(Accelerometer& a, Magnetometer& m)
    : server(80), accel(a), mag(m), output2(2), output2State("off") {}

void web_server::begin() {
    pinMode(output2, OUTPUT);
    digitalWrite(output2, LOW);

    WiFi.softAP(ssid, password);
    Serial.print("Access Point IP: ");
    Serial.println(WiFi.softAPIP());

    setupRoutes();
    server.begin();
    Serial.println("HTTP server started");
}

void web_server::handleClient() {
    server.handleClient();
}

void web_server::setupRoutes() {
    server.on("/", [this]() { handleRoot(); });
    server.on("/26/on", [this]() { handleGPIO2On(); });
    server.on("/26/off", [this]() { handleGPIO2Off(); });
    server.on ("/info",[this]() {return_data();} );

}

void web_server::handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Dashboard</title>
  <style>
    body { font-family: Helvetica; text-align: center; }
    .data-label { font-weight: bold; }
    .data-value { color: #4CAF50; font-size: 1.5em; }
    .button { padding: 16px 40px; font-size: 20px; margin: 10px; cursor: pointer; border: none; color: white; }
    .on { background-color: #4CAF50; }
    .off { background-color: #555555; }
  </style>
</head>
<body>
  <h1>ESP32 Dashboard</h1>

  <!-- Sensor Data -->
  <p><span class="data-label">Roll:</span> <span id="roll" class="data-value">--</span></p>
  <p><span class="data-label">Pitch:</span> <span id="pitch" class="data-value">--</span></p>
  <p><span class="data-label">Yaw:</span> <span id="yaw" class="data-value">--</span></p>
  <p><span class="data-label">Heading:</span> <span id="heading" class="data-value">--</span></p>

  <!-- LED Control -->
  <p>LED Estado: <span id="ledState">)" + (output2State == "on" ? "Encendido" : "Apagado") + R"rawliteral(</span></p>
  <button id="btnOn" class="button on">Encender LED</button>
  <button id="btnOff" class="button off">Apagar LED</button>

  <script>
    function fetchData() {
      fetch('/info')
        .then(response => response.json())
        .then(data => {
          document.getElementById('roll').textContent = data.roll.toFixed(2);
          document.getElementById('pitch').textContent = data.pitch.toFixed(2);
          document.getElementById('yaw').textContent = data.yaw.toFixed(2);
          document.getElementById('heading').textContent = data.heading.toFixed(2);
        })
        .catch(console.error);
    }

    // Funciones para encender/apagar LED via fetch
    document.getElementById('btnOn').addEventListener('click', () => {
      fetch('/26/on').then(() => {
        document.getElementById('ledState').textContent = 'Encendido';
      });
    });

    document.getElementById('btnOff').addEventListener('click', () => {
      fetch('/26/off').then(() => {
        document.getElementById('ledState').textContent = 'Apagado';
      });
    });

    setInterval(fetchData, 100);
    fetchData();
  </script>
</body>
</html>
    )rawliteral";

    server.send(200, "text/html", html);
}

void web_server::handleGPIO2On() {
    output2State = "on";
    digitalWrite(output2, HIGH);
    server.send(200, "text/plain", "OK");  // Solo respondemos "OK", sin recargar HTML
}

void web_server::handleGPIO2Off() {
    output2State = "off";
    digitalWrite(output2, LOW);
    server.send(200, "text/plain", "OK");  // Mismo aquí
}



void web_server::return_data() {
    String json = "{";
    json += "\"roll\":" + String(accel.getRoll(), 2) + ",";
    json += "\"pitch\":" + String(accel.getPitch(), 2) + ",";
    json += "\"yaw\":" + String(accel.getYaw(), 2) + ",";
    json += "\"heading\":" + String(mag.getHeadingDegrees(), 2);
    json += "}";
    server.send(200, "application/json", json);
}
