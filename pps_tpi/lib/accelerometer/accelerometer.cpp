#include "Accelerometer.h"

#define MPU_ADDR 0x68

void Accelerometer::begin() {
    Wire.begin(21, 22, 400000);
    delay(250);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void Accelerometer::update() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);

    GyX = (Wire.read() << 8) | Wire.read();
    GyY = (Wire.read() << 8) | Wire.read();
    GyZ = (Wire.read() << 8) | Wire.read();

    rateRoll = (float)GyX / 65.5 - calibRoll;
    ratePitch = (float)GyY / 65.5 - calibPitch;
    rateYaw = (float)GyZ / 65.5 - calibYaw;
}

void Accelerometer::calibrate() {
    printf("Calibrating accelerometer...\n");
    calibRoll = calibPitch = calibYaw = 0;
    for (int i = 0; i < 3000; i++) {
        update();
        calibRoll += rateRoll;
        calibPitch += ratePitch;
        calibYaw += rateYaw;
        delay(1);
    }
    calibRoll /= 3000.0;
    calibPitch /= 3000.0;
    calibYaw /= 3000.0;
}

float Accelerometer::getRoll() const { return rateRoll; }
float Accelerometer::getPitch() const { return ratePitch; }
float Accelerometer::getYaw() const { return rateYaw; }
