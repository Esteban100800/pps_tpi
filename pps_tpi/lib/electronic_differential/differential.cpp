#include "differential.h"


ElectronicDifferential::ElectronicDifferential(float wheelBase,
                                                 float maxRPM, float Length, float wheelRadius, float delta_max)
    : wheelBase(wheelBase), maxRPM(maxRPM), Length(Length), wheelRadius(wheelRadius), delta_max(delta_max)
{
}


ElectronicDifferential::differential ElectronicDifferential::getDifferential()
{
    return diff;
}

void ElectronicDifferential::setwheelBase(float W)
{
   wheelBase = W;
}


void ElectronicDifferential::setmaxRPM(float maxRPM)
{
   this->maxRPM = maxRPM;
}

void ElectronicDifferential::setLength(float Length)
{
   this->Length = Length;
}

void ElectronicDifferential::setwheelRadius(float wheelRadius)
{
   this->wheelRadius = wheelRadius;
}

void ElectronicDifferential::setdelta(float delta_max)
{
   this->delta_max = delta_max;
}

void ElectronicDifferential::computeWheelSpeeds(float v)
{
    // velocidades angulares [rad/s]
    float w_R = (v / wheelRadius) * ((2.0f * Length + wheelBase * tanf(delta_max)) / (2.0f * Length));
    float w_L = (v / wheelRadius) * ((2.0f * Length - wheelBase * tanf(delta_max)) / (2.0f * Length));

    // conversión a RPM
    float rightRPM = w_R * 60.0f / (2.0f * M_PI);
    float leftRPM  = w_L * 60.0f / (2.0f * M_PI);

    // Limitación de las RPM al máximo permitido
    if (leftRPM > maxRPM)
        leftRPM = maxRPM;
    else if (leftRPM < -maxRPM)
        leftRPM = -maxRPM;

    if (rightRPM > maxRPM)
        rightRPM = maxRPM;
    else if (rightRPM < -maxRPM)
        rightRPM = -maxRPM;

    // Almacenar en la estructura differential
    diff.leftRPM = leftRPM;
    diff.rightRPM = rightRPM;
}


