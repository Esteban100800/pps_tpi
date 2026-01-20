#pragma once

#include <cmath>

class ElectronicDifferential
{
public:
    ElectronicDifferential(float wheelBase, float maxRPM, float Length, float wheelRadius, float delta_max); //delta max es el angulo maximo del servo

    void computeWheelSpeeds(float v);

    struct differential
    {
        float leftRPM;
        float rightRPM;
    };

    differential getDifferential();

    void setwheelBase(float W);
    void setmaxRPM(float maxRPM);
    void setLength(float Length);
    void setwheelRadius(float wheelRadius);
    void setdelta(float delta_max);
    

private:
    float wheelBase;       // Distancia entre ruedas (wheelbase)
    float maxRPM; // Velocidad máxima de las ruedas
    float Length;  // Longitud del vehículo
    float wheelRadius; // Radio de las ruedas
    float delta_max; // Ángulo máximo del servo
    differential diff;
};

