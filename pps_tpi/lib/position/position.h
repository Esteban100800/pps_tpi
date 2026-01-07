#ifndef POSITION_H
#define POSITION_H

#include <Arduino.h>

struct Position {
    float x;      // metros
    float y;      // metros
    float theta;  // radianes
};

class PositionEstimator {
public:
    PositionEstimator(float wheelRadius,
                      float wheelBase);

    void reset(float x = 0.0f,
               float y = 0.0f,
               float theta = 0.0f);

    // rpmRight / rpmLeft en RPM
    // dt en segundos
    void updateFromRPM(float rpmRight,
                       float rpmLeft,
                       float dt, float yawRad);   // ← viene del MPU

    Position getPosition() const;

private:
    float R;   // radio rueda [m]
    float W;   // distancia entre ruedas [m]

    Position state;
};

#endif
