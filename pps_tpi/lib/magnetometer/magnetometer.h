#pragma once
#include <Wire.h>

class Magnetometer {
public:
    Magnetometer();
    void begin();
    void update();
    void calibrate();

    float getX() const;
    float getY() const;
    float getZ() const;
    float getHeadingDegrees() const;

private:
    static constexpr uint8_t MAG_ADDR = 0x0D;
    float magX_Gauss, magY_Gauss, magZ_Gauss;
    float magMinX, magMinY, magMinZ;
    float magMaxX, magMaxY, magMaxZ;
    float declination;

    bool dataReady();
};


