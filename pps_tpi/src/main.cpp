#include "esp_web_server.h"
#include "mpu_dmp.h"

ESPWebServer myServer;
MPUDMP mpu(2); 

float yaw = 123.45;
bool started = false;   

void setup()
{

    Serial.begin(115200);
    while (!Serial) ; 
    
    Serial.println("Presioná una tecla para comenzar...");
    while (!Serial.available()) {
    }
    Serial.read(); 

    mpu.begin();
    myServer.begin();
    started = true;

}

void loop()
{   
    if (started) {
        mpu.update();
    }
    yaw = mpu.getYaw();
    myServer.updateSensor(yaw);
    myServer.loop();
}
