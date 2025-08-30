#pragma once
#include <Arduino.h>

#define PPR 1800.0 // Pulsos por revolución
#define MAX_MOTORS 4 // Máximo número de motores

class Motor {
  private:
    int ia1Pin, ia2Pin, pwmPin, pwmChannel;
    int encoderPin;
    volatile long pulseCount;
    float dt;
    float filteredRPM;
    float alpha; // Factor de suavizado (entre 0 y 1)
    int motorIndex; // Índice de este motor en el array
    portMUX_TYPE mux;

  public:
    Motor(int ia1, int ia2, int pwm, int channel, int encoder);
    void begin();
    void setPWM(int pwm);
    float getRPM();
    void resetEncoder();
    void encoderISR();
    void moveForward(int pwm);
    void moveBackward(int pwm);
    void stop();

    // Array estático para manejar múltiples motores
    static Motor* motorInstances[MAX_MOTORS];
    static int motorCount;
    static bool isrServiceInstalled; // Flag para evitar instalar el servicio múltiples veces
    static void IRAM_ATTR globalEncoderISR0();
    static void IRAM_ATTR globalEncoderISR1();
    static void IRAM_ATTR globalEncoderISR2();
    static void IRAM_ATTR globalEncoderISR3();
};
