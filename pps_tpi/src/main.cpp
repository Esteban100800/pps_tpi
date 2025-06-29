#include <Arduino.h>
#include <Wire.h>


const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t GyX, GyY, GyZ;
float rateRoll, ratePitch, rateYaw, calibRoll, calibPitch, calibYaw;
int calibrationCount = 0; // Number of samples for calibration

void gyro_signals();
void calibration();

void setup()
{
  Serial.begin(115200);
  pinMode(2, OUTPUT);         // Pin 2 es el LED incorporado en ESP32
  digitalWrite(2, HIGH);      // Encender LED para indicar inicio del setup
  Wire.setClock(400000);      // Set I2C clock speed to 400kHz
  Wire.begin(21, 22, 100000); // sda, scl, clock speed
  delay(250);                 // Allow time for the MPU-6050 to initialize
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);
  Serial.println("Setup complete");

  calibration();

 // xTaskCreate(calibration, "MiTarea", 2048, NULL, 1, NULL);
}

void loop()
{
  gyro_signals();
  rateRoll -= calibRoll;   // Subtract calibration values
  ratePitch -= calibPitch; // Subtract calibration values   
  rateYaw -= calibYaw;   // Subtract calibration values
  Serial.print("roll rate= ");
  Serial.print(rateRoll);
  Serial.print(" pitch rate= ");
  Serial.print(ratePitch);
  Serial.print("Yaw rate= ");
  Serial.print(rateYaw);
  Serial.print("\n");
}

void gyro_signals()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(MPU_ADDR, 6);
  GyX = (Wire.read() << 8 | Wire.read());
  GyY = (Wire.read() << 8 | Wire.read());
  GyZ = (Wire.read() << 8 | Wire.read());

  rateRoll = (float)GyX / 65.5;  // Convert to degrees per second
  ratePitch = (float)GyY / 65.5; // Convert to degrees per second
  rateYaw = (float)GyZ / 65.5;   // Convert to degrees per second
}

void calibration()
{
  for (calibrationCount = 0; calibrationCount < 2000; calibrationCount++)
  {
    gyro_signals();
    calibRoll += rateRoll;
    calibPitch += ratePitch;
    calibYaw += rateYaw;
    delay(1);
  }
  calibRoll /= 2000.0; // Average the calibration values
  calibPitch /= 2000.0;
  calibYaw /= 2000.0;

  Serial.println("Calibration complete");
  Serial.print("Calibrated Roll: ");
  Serial.println(calibRoll);
  Serial.print("Calibrated Pitch: ");
  Serial.println(calibPitch);
  Serial.print("Calibrated Yaw: ");
  Serial.println(calibYaw);

  delay(1000);
}