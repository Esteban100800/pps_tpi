# include "accelerometer.h"

int16_t GyX=0, GyY=0, GyZ=0;
float rateRoll=0.0, ratePitch=0.0, rateYaw=0.0, calibRoll=0.0, calibPitch=0.0, calibYaw=0.0;
int calibrationCount=0.0;


void config_gyro()
{
  Wire.setClock(400000);     
  Wire.begin(21, 22, 400000); // sda, scl, clock speed
  delay(250);                 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    // despertamos la IMU-6050
  Wire.endTransmission(true);
  Serial.println("Setup complete");
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

  // FS_SEL_2 = 65.5 LSB/deg/s
  rateRoll = (float)GyX / 65.5;  
  ratePitch = (float)GyY / 65.5;
  rateYaw = (float)GyZ / 65.5;   
}

void calibrate_gyro()
{
  for (calibrationCount = 0; calibrationCount < 2000; calibrationCount++)
  {
    gyro_signals();
    calibRoll += rateRoll;
    calibPitch += ratePitch;
    calibYaw += rateYaw;
    delay(1);
  }

  // Promedio de las lecturas para la correccion del error.
  calibRoll /= 2000.0; 
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


void print_status()
{
  rateRoll -= calibRoll;   
  ratePitch -= calibPitch;  
  rateYaw -= calibYaw;  
  Serial.print("roll rate= ");
  Serial.print(rateRoll);
  Serial.print(" pitch rate= ");
  Serial.print(ratePitch);
  Serial.print("Yaw rate= ");
  Serial.print(rateYaw);
  Serial.print("\n");
}
