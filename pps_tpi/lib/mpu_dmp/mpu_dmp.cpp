#include "mpu_dmp.h"

MPUDMP::MPUDMP(uint8_t interruptPin, uint8_t ledPin)
    : interruptPin(interruptPin), ledPin(ledPin) {}

void MPUDMP::begin() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    while (!Serial);

    mpu.initialize();
    pinMode(interruptPin, INPUT);
    pinMode(ledPin, OUTPUT);

    if (mpu.testConnection()) {
        Serial.println(F("MPU6050 conectado"));
        initializeDMP();
    } else {
        Serial.println(F("Fallo conexión con MPU6050"));
    }
}

void MPUDMP::initializeDMP() {
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-30);
    mpu.setYGyroOffset(54);
    mpu.setZGyroOffset(63);
    mpu.setZAccelOffset(904);

    if (devStatus == 0) {
        mpu.CalibrateAccel(8);
        mpu.CalibrateGyro(8);
        mpu.PrintActiveOffsets();

        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println(F("DMP listo"));
    } else {
        Serial.print(F("Error en DMP (código "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void MPUDMP::update() {
    if (!dmpReady) return;

    if (!calibrated) {
        calibrateInitialYaw();
        calibrated = true;
    }

    read();

    blinkState = !blinkState;
    digitalWrite(ledPin, blinkState);
}

void MPUDMP::read() {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yaw_raw_actual = ypr[0] * 180 / M_PI;

        float delta = yaw_raw_actual - yaw_raw_anterior;

        if (abs(delta) <= 0.01 && abs(delta) > 0.001) {
            error_acumulado += delta;
        } else if (abs(delta) > 0.012) {
            yaw += delta;
            error_acumulado *= 0.5;
        }

        yaw_raw_anterior = yaw_raw_actual;

       // Serial.print(yaw_raw_actual, 5);
       // Serial.print("\t");
      //  Serial.print(yaw, 5);
      //  Serial.print("\t");
       // Serial.println(error_acumulado, 5);
    }
}

void MPUDMP::calibrateInitialYaw() {
    Serial.println(F("Calibrando yaw inicial..."));
    while (abs(yaw) < 0.03) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180 / M_PI;
            //Serial.println(yaw);
        }
        delay(10);
    }

    Serial.println(F("Estabilizado"));
    yaw_raw_anterior = yaw;
}

float MPUDMP::getYaw() const {
    return yaw;
}

float MPUDMP::getRawYaw() const {
    return yaw_raw_actual;
}

float MPUDMP::getAccumulatedError() const {
    return error_acumulado;
}
