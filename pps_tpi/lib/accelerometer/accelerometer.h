#pragma once

#include <Wire.h>

class Accelerometer {
public:
    void begin();
    void update();
    void calibrate();
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;

private:
    int16_t GyX, GyY, GyZ;
    float rateRoll, ratePitch, rateYaw;
    float calibRoll, calibPitch, calibYaw;
};

