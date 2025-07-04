#include "magnetometer.h"
#include <Arduino.h>
#include <math.h>

Magnetometer::Magnetometer()
    : magX_Gauss(0), magY_Gauss(0), magZ_Gauss(0),
      magMinX(32767), magMinY(32767), magMinZ(32767),
      magMaxX(-32768), magMaxY(-32768), magMaxZ(-32768),
      declination(-0.10f) {}

void Magnetometer::begin() {
    Wire.begin(21, 22, 400000);
    delay(250);

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0B); // Control Register
    Wire.write(0x01);
    Wire.endTransmission();

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x09); // Control Register: 10Hz, 1280 LSB/Gauss, continuous mode
    Wire.write(0x1D);
    Wire.endTransmission();

    Serial.println("Magnetometer setup complete");
}

bool Magnetometer::dataReady() {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x06); // Status register
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 1);

    if (Wire.available()) {
        byte status = Wire.read();
        return status & 0x01; // Bit 0 = DRDY
    }
    return false;
}

void Magnetometer::update() {
    if (!dataReady()) return;

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 6);

    if (Wire.available() == 6) {
        int16_t magX_raw = Wire.read() | (Wire.read() << 8);
        int16_t magY_raw = Wire.read() | (Wire.read() << 8);
        int16_t magZ_raw = Wire.read() | (Wire.read() << 8);

        // Offset y escala
        float offsetX = (magMaxX + magMinX) / 2.0;
        float offsetY = (magMaxY + magMinY) / 2.0;
        float scaleX = (magMaxX - magMinX) / 2.0;
        float scaleY = (magMaxY - magMinY) / 2.0;

        float normX = ((float)magX_raw - offsetX) / scaleX;
        float normY = ((float)magY_raw - offsetY) / scaleY;

        magX_Gauss = normX;
        magY_Gauss = normY;
        magZ_Gauss = (float)magZ_raw; // Opcional: podés escalar si querés
    }
}

void Magnetometer::calibrate() {
    Serial.println("Calibrating magnetometer, rotate in all directions...");
    delay(1000);
    magMinX = magMinY = magMinZ = 32767;
    magMaxX = magMaxY = magMaxZ = -32768;

    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom(MAG_ADDR, 6);

        if (Wire.available() == 6) {
            int16_t mx = Wire.read() | (Wire.read() << 8);
            int16_t my = Wire.read() | (Wire.read() << 8);
            int16_t mz = Wire.read() | (Wire.read() << 8);

            if (mx < magMinX) magMinX = mx;
            if (mx > magMaxX) magMaxX = mx;
            if (my < magMinY) magMinY = my;
            if (my > magMaxY) magMaxY = my;
            if (mz < magMinZ) magMinZ = mz;
            if (mz > magMaxZ) magMaxZ = mz;
        }
        delay(50);
    }

    Serial.println("Magnetometer calibration complete.");
}

float Magnetometer::getX() const {
    return magX_Gauss;
}

float Magnetometer::getY() const {
    return magY_Gauss;
}

float Magnetometer::getZ() const {
    return magZ_Gauss;
}

float Magnetometer::getHeadingDegrees() const {
    float heading = atan2(magY_Gauss, magX_Gauss);
    float headingDegrees = heading * 180.0 / PI;

    if (headingDegrees < 0)
        headingDegrees += 360.0;

    return headingDegrees + declination;
}
