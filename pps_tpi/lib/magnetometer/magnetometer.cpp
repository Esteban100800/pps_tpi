#include "Magnetometer.h"

int16_t magX_raw = 0, magY_raw = 0, magZ_raw = 0;
float magX_Gauss = 0, magY_Gauss = 0, magZ_Gauss = 0;
float magMinX = 0, magMinY = 0, magMinZ = 0;
float magMaxX = 0, magMaxY = 0, magMaxZ = 0;
float declination = -0.10;

void mag_signals()
{
    if (!data_ready())
        return;

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 6);

    if (Wire.available() == 6)
    {
        magX_raw = Wire.read() | (Wire.read() << 8);
        magY_raw = Wire.read() | (Wire.read() << 8);
        magZ_raw = Wire.read() | (Wire.read() << 8);

        // Calcular offset y escala
        float offsetX = (magMaxX + magMinX) / 2.0;
        float offsetY = (magMaxY + magMinY) / 2.0;
        float scaleX = (magMaxX - magMinX) / 2.0;
        float scaleY = (magMaxY - magMinY) / 2.0;

        float normX = ((float)magX_raw - offsetX) / scaleX;
        float normY = ((float)magY_raw - offsetY) / scaleY;

        magX_Gauss = normX;
        magY_Gauss = normY;
    }
}

void print_mag_status()
{
    Serial.print("Magnetometer X: ");
    Serial.print(magX_Gauss);
    Serial.print(" Y: ");
    Serial.print(magY_Gauss);
    Serial.print(" Z: ");
    Serial.println(magZ_Gauss);
}

void calibrate_mag()
{
    Serial.println("Calibrating magnetometer, please rotate it in all directions...");

    magMinX = magMinY = magMinZ = 32767;
    magMaxX = magMaxY = magMaxZ = -32768;

    unsigned long startTime = millis();
    while (millis() - startTime < 10000)
    {
        Wire.beginTransmission(MAG_ADDR);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom(MAG_ADDR, 6);

        if (Wire.available() == 6)
        {
            int16_t mx = Wire.read() | (Wire.read() << 8);
            int16_t my = Wire.read() | (Wire.read() << 8);
            int16_t mz = Wire.read() | (Wire.read() << 8);

            if (mx < magMinX)
                magMinX = mx;
            if (mx > magMaxX)
                magMaxX = mx;
            if (my < magMinY)
                magMinY = my;
            if (my > magMaxY)
                magMaxY = my;
            if (mz < magMinZ)
                magMinZ = mz;
            if (mz > magMaxZ)
                magMaxZ = mz;
        }
        delay(50);
    }

    Serial.println("Mag calibration complete.");
}

bool data_ready()
{
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x06); // Registro de estado del output del sensor
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, 1);

    if (Wire.available())
    {
        byte status = Wire.read();
        return status & 0x01; // Bit 0 = DRDY
    }
    return false;
}

void print_heading()
{
    float heading = atan2(magY_Gauss, magX_Gauss);
    float headingDegrees = heading * 180.0 / PI;

    if (headingDegrees < 0)
    {
        headingDegrees += 360.0;
    }

    headingDegrees += declination; // Ajuste por declinación magnética

    Serial.print("Heading (Yaw): ");
    Serial.print(headingDegrees);
    Serial.println(" deg");
}

void config_magnetometer()
{
    Wire.begin(21, 22, 400000);
    delay(250);

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x0B); // Control Register
    Wire.write(0x01);
    Wire.endTransmission();

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x09); // Control Register
    Wire.write(0x1D); // 10Hz, 1280 LSB/Gauss, Continuous measurement mode
    Wire.endTransmission();


    Serial.println("Setup complete");
}
