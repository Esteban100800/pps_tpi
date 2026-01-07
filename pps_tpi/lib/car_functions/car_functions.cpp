#include "car_functions.h"
#include <Arduino.h>

const uint8_t HiWonderMotors::ENCODER_READ_ADDR[HiWonderMotors::NUM_MOTORS] = {
    0xA0, 0xA4};
const uint8_t HiWonderMotors::ENCODER_WRITE_ADDR[HiWonderMotors::NUM_MOTORS] = {
    0xB0, 0xB4};



HiWonderMotors::HiWonderMotors(const uint8_t i2c_address)
    : _i2c_address(i2c_address) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    _motor_pwm[i] = 0;
  }
}

bool HiWonderMotors::begin() {
  Wire.begin();
  Wire.beginTransmission(_i2c_address);
  if (Wire.endTransmission() == 0) {
    // Conexión exitosa
    // Si responde, configuramos
    uint8_t type = MOTOR_TYPE_JGB37_520_12V_110RPM;
    uint8_t polarity = MOTOR_ENCODER_POLARITY_DEFAULT;
    writeData(MOTOR_TYPE_ADDR, &type, 1);
    delay(5);
    writeData(MOTOR_ENCODER_POLARITY_ADDR, &polarity, 1);
    return true;
  } else {
    // Falló la conexión I2C
    return false;
  }
}

void HiWonderMotors::setMotorPWM(const MotorID motor, const int8_t pwm,
                                 const ControlMode mode) {
  if (motor < 0 || motor > NUM_MOTORS)
    return;
  // HiWonderCar espera el PWM como uint8_t en complemento a dos
  _motor_pwm[motor] = (uint8_t)pwm; // Cast para que -16 → 240

  uint8_t addr =
      (mode == SPEED_MODE) ? MOTOR_FIXED_SPEED_ADDR : MOTOR_FIXED_PWM_ADDR;
  writeData(addr, _motor_pwm, 4);
}

void HiWonderMotors::setAllMotorSpeed(const int8_t speed_values[NUM_MOTORS]) {

  for (int i = 0; i < NUM_MOTORS; ++i) {
    _motor_pwm[i] = static_cast<uint8_t>(speed_values[i]); // Complemento a dos
  }
  writeData(MOTOR_FIXED_SPEED_ADDR, _motor_pwm, NUM_MOTORS);

  // uint8_t addr =
  //    (mode == SPEED_MODE) ? MOTOR_FIXED_SPEED_ADDR : MOTOR_FIXED_PWM_ADDR;
  // writeData(addr, _motor_pwm, NUM_MOTORS);
}

void HiWonderMotors::setAllMotorPWM(const int8_t pwm_values[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    _motor_pwm[i] = static_cast<uint8_t>(pwm_values[i]);
  }

  writeData(MOTOR_FIXED_PWM_ADDR, _motor_pwm, NUM_MOTORS);
}

void HiWonderMotors::stopAll() {
  for (int i = 0; i < NUM_MOTORS; ++i) {
    _motor_pwm[i] = 0;
  }
  writeData(MOTOR_FIXED_SPEED_ADDR, _motor_pwm,
            4); // podés cambiar a PWM_ADDR si querés
}

int32_t HiWonderMotors::GetEncoderCount(const MotorID motor) {
  if (motor < 0 || motor >= NUM_MOTORS)
    return 0;

  uint8_t raw[4];
  uint8_t reg = MOTOR_ENCODER_TOTAL_ADDR + 4 * motor;

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0; // Falló la transmisión
  }

  Wire.requestFrom(_i2c_address, (uint8_t)4);
  if (Wire.available() < 4) {
    return 0; // No se recibieron los datos esperados
  }

  for (int i = 0; i < 4; i++) {
    raw[i] = Wire.read();
  }

  // LSB primero
  return (int32_t)(((uint32_t)raw[0]) | ((uint32_t)raw[1] << 8) |
                   ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24));
}

void HiWonderMotors::SetEncoderCount(const MotorID motor, const int32_t count) {
  if (motor < 0 || motor >= NUM_MOTORS)
    return;

  uint8_t reg =
      MOTOR_ENCODER_TOTAL_ADDR + 4 * motor; // Cada encoder ocupa 4 bytes

  uint32_t ucount =
      static_cast<uint32_t>(count); // Por las dudas, lo casteamos a unsigned
  uint8_t data[4];
  data[0] = ucount & 0xFF; // LSB
  data[1] = (ucount >> 8) & 0xFF;
  data[2] = (ucount >> 16) & 0xFF;
  data[3] = (ucount >> 24) & 0xFF; // MSB

  writeData(reg, data, 4);
}

void HiWonderMotors::setAllEncoderCounts(const int32_t counts[NUM_MOTORS]) {
  uint8_t data[16];

  for (int i = 0; i < NUM_MOTORS; i++) {
    data[i * 4 + 0] = (counts[i] >> 0) & 0xFF; // LSB
    data[i * 4 + 1] = (counts[i] >> 8) & 0xFF;
    data[i * 4 + 2] = (counts[i] >> 16) & 0xFF;
    data[i * 4 + 3] = (counts[i] >> 24) & 0xFF; // MSB
  }

  writeData(MOTOR_ENCODER_TOTAL_ADDR, data, 16);
}

void HiWonderMotors::attachServo(const uint8_t pin, const int minPulse,
                                 const int maxPulse) {
  _servo_pin = pin;
  int resultado = _servo.attach(pin, minPulse, maxPulse);

  _servo_attached = true;
//  Serial.printf(ICON_CONNECTED " Servo conectado a GPIO %d, resultado: %d\n",
 //               pin, resultado);
  setServoAngle(0.0); // Inicializa al centro
}

void HiWonderMotors::setServoAngle(const float angle) {
  if (!_servo_attached) return;

  // Limitar al rango permitido
  float limited_angle = constrain(angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  // Mapeo lineal de -90 → 0 y 90 → 180
  float mapped_angle = (limited_angle + 90.0f) * (180.0f / 180.0f); // opcionalmente sólo: limited_angle + 90

  _servo.write(static_cast<int>(round(mapped_angle)));
}


void HiWonderMotors::testServo() {
  if (_servo_attached) {

    setServoAngle(0);
    delay(500);

    for (int i = 0; i <= MAX_SERVO_ANGLE; i += 10) {
      setServoAngle(i);
//      Serial.println(ICON_ROTATING " Mueve el servo a la posición: " +
  //                   String(i));
      delay(50);
    }
    for (int i = MAX_SERVO_ANGLE; i >= MIN_SERVO_ANGLE; i -= 10) {
      setServoAngle(i);
     // Serial.println(ICON_ROTATING " Mueve el servo a la posición: " +
       //              String(i));
      delay(50);
    }
    for (int i = MIN_SERVO_ANGLE; i <= 0; i += 10) {
      setServoAngle(i);
      //Serial.println(ICON_ROTATING " Mueve el servo a la posición: " +
        //             String(i));
      delay(50);
    }
  }
}

void HiWonderMotors::writeData(const uint8_t reg, const uint8_t *data,
                               const uint8_t len) {
  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

bool HiWonderMotors::getAllEncoderCounts(int32_t encoders[4]) {
  uint8_t raw[16];
  if (wireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR, raw, 16) != 16) {
    return false;
  }

  for (int i = 0; i < 4; i++) {
    encoders[i] = (int32_t)(
        ((uint32_t)raw[i * 4 + 0]) | ((uint32_t)raw[i * 4 + 1] << 8) |
        ((uint32_t)raw[i * 4 + 2] << 16) | ((uint32_t)raw[i * 4 + 3] << 24));
  }

  return true;
}

int HiWonderMotors::wireReadDataArray(uint8_t reg, uint8_t *val,
                                      unsigned int len) {
  uint8_t i = 0;

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return -1;
  }

  Wire.requestFrom(_i2c_address, len);
  while (Wire.available()) {
    if (i >= len) {
      return -1;
    }
    val[i++] = Wire.read();
  }

  return i;
}

void HiWonderMotors::setAngle(const float angle) {
  servo_angle = angle;
}

float HiWonderMotors::getServoAngle() {
  return servo_angle;
}