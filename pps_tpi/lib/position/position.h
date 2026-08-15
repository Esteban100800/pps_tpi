#ifndef POSITION_H
#define POSITION_H

#include <Arduino.h>
#include <math.h>
#include "car_functions.h"
#include "PIDController.h"

struct Position
{
    float x;     // metros
    float y;     // metros
    float theta; // radianes
};

class PositionEstimator
{
public:
    PositionEstimator(float wheelRadius,
                      float wheelBase, HiWonderMotors &car, PIDController &pidMotor1, PIDController &pidMotor2);

    void reset(float x = 0.0f,
               float y = 0.0f,
               float theta = 0.0f);

    // rpmRight / rpmLeft en RPM
    // dt en segundos
    void updateFromRPM(float rpmRight,
                       float rpmLeft,
                       float dt, float yawRad); //  viene del MPU

    Position getPosition() const;

    float getDistance();
    float getDistanceIntegral();
    int getDireccion();
    void setDireccion(int dir);

    bool move() const;
    bool isFinished() const;

    void setMove(bool moving);
    void setFinished(bool finish);

    void reset_segment_distance();

    float getSegmentDistance();

    bool getChangeSpeed();

    void setChangeSpeed(bool change);

    float getSpeed();

    void setSpeed(float speed);

    float segments_distances[3];

private:
    float R; // radio rueda [m]
    float W; // distancia entre ruedas [m]

    float x_prev;
    float y_prev;

    float total_distance;
    float total_distance_integral;

    float segment_distance;

    int direccion;

    bool moving = false;
    bool finish = true;

    bool change_speed = false;

    float speed = 0.0f;

    Position state;
};

#endif
