#ifndef MPU_DMP_H
#define MPU_DMP_H

#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

class MPUDMP {
public:
    MPUDMP(uint8_t interruptPin, uint8_t ledPin = 13);
    void begin();
    void update();

    float getYaw() const;
    float getRawYaw() const;
    float getAccumulatedError() const;

private:
    void initializeDMP();
    void calibrateInitialYaw();
    void read();

    MPU6050 mpu;
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    float yaw = 0.0;
    float yaw_raw_actual = 0.0;
    float yaw_raw_anterior = 0.0;
    float error_acumulado = 0.0;

    bool dmpReady = false;
    bool calibrated = false;

    uint8_t devStatus;
    uint8_t fifoBuffer[64];
    uint16_t packetSize;

    uint8_t interruptPin;
    uint8_t ledPin;
    bool blinkState = false;
};

#endif
