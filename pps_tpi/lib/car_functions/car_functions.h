#ifndef CAR_FUNCTIONS_H
#define CAR_FUNCTIONS_H

#include <Arduino.h>
#include <ESP32Servo.h> // Agregamos esto
#include <Wire.h>

// *** DRIVER CONSTANTS ***
const uint8_t I2C_ADDRESS = 0x34; // Dirección I2C del HiWonderCar
const int8_t MAX_SPEED = 50;
#define ADC_BAR_ADDR 0x00 // Battery sampling for ADC
#define MOTOR_TYPE_ADDR 0x14 // Encoder motor type setting
#define MOTOR_ENCODER_POLARITY_ADDR                                            \
  0x15 // Set the polarity of the encoder motor
#define MOTOR_FIXED_PWM_ADDR                                                   \
  0x1F // Fixed PWM value for the motor. For open loop control
       // Value ranges -100, 100
#define MOTOR_FIXED_SPEED_ADDR                                                 \
  0x33 // Fixed speed value for the motor. For closed loop control. El
       // rango puede variar entre -50 y +50
#define MOTOR_ENCODER_TOTAL_ADDR 60

// *** MOTOR TYPES
#define MOTOR_TYPE_WITHOUT_ENCODER 0
#define MOTOR_TYPE_TT 1
#define MOTOR_TYPE_N20 2
#define MOTOR_TYPE_JGB37_520_12V_110RPM 3
/* the magnetic ring generates 44
pulses per revolution, combined with a gear reduction ratio of 90
//If the number of pulses per revolution of the motor (U) and the diameter of
the wheel (D) are known, then the distance traveled by each wheel can be
determined through pulse counting.
//For example, if you read the total number of pulses (P) from motor 1, then the
distance traveled can be calculated using the formula: (P/U) * (3.14159 * D)*/

#define MOTOR_ENCODER_POLARITY_DEFAULT 0

// *** CAR CONSTANTS ***

#define R_TURN_MIN 55.0 // Radio de giro mínimo (cm)
#define REDUCCION 30.0 // Reducción del motor
#define PPR 11.0 // Pulsos por revolución del encoder
#define FACTOR_CUADRATURA 4.0 // Factor de cuadratura del encoder
#define V_MAX 80.0 // Velocidad máxima (cm/s)
#define L_CAR 17.5 // Distancia entre ejes (cm)
#define PSI_MAX                                                                \
  30.0 // Ángulo máximo de giro (grados) //TODO: Revisar este valor
#define D 7.0 // Diámetro de la rueda (cm) Originalmente 7.5 cm, pero se ajusta a 7.0 cm

#define REDUCCION 30.0 // Reducción del motor
#define TICKS_POR_VUELTA_DE_RUEDA (PPR * REDUCCION * FACTOR_CUADRATURA)
#define PERIMETRO_RUEDA_M (M_PI * D)
#define CM_POR_TICK (PERIMETRO_RUEDA_M / TICKS_POR_VUELTA_DE_RUEDA)

#define MAX_SERVO_ANGLE 50.0 // Grados
#define MIN_SERVO_ANGLE -50.0 // Grados

enum MotorID { MOTOR_1 = 0, MOTOR_2, MOTOR_3, MOTOR_4 };
enum ControlMode { PWM_MODE = 0, SPEED_MODE = 1 };

struct vehicle_constants {
  float L_car;      // Longitud del vehículo (cm)
  float psi_max;    // Ángulo máximo de giro (grados)
  float v;          // Velocidad lineal constante del vehículo (cm/s)
  float r_turn_min; // Radio de giro mínimo (cm)
};

class HiWonderMotors {
public:
  static constexpr int NUM_MOTORS = 4;
  HiWonderMotors(const uint8_t i2c_address = I2C_ADDRESS);

  bool begin();

  // Motores
  void setMotorPWM(const MotorID motor, const int8_t pwm,
                   const ControlMode mode = SPEED_MODE);
  void setAllMotorSpeed(const int8_t speed_values[NUM_MOTORS]);
  void setAllMotorPWM(const int8_t pwm_values[NUM_MOTORS]);

  void stopAll();

  // Encoders
  int32_t GetEncoderCount(const MotorID motor);
  void SetEncoderCount(const MotorID motor, const int32_t count);
  bool getAllEncoderCounts(int32_t encoders[NUM_MOTORS]); // <-- Nuevo método
  void setAllEncoderCounts(const int32_t counts[NUM_MOTORS]);

  // Servo
  void attachServo(const uint8_t pin, const int minPulse = 500,
                   const int maxPulse = 2500);
  void setServoAngle(const float angle); // Ángulo en grados, de -90 a +90
  void testServo();
  

  float servo_angle; // Ángulo actual del servo

  void setAngle(const float angle);

  float getServoAngle();
  

private:
  void writeData(const uint8_t reg, const uint8_t *data, const uint8_t len);
  int wireReadDataArray(uint8_t reg, uint8_t *val,
                        unsigned int len); // <-- Nuevo método auxiliar

  uint8_t _i2c_address;
  uint8_t _motor_pwm[NUM_MOTORS];

  // Encoders
  static const uint8_t ENCODER_READ_ADDR[NUM_MOTORS];
  static const uint8_t ENCODER_WRITE_ADDR[NUM_MOTORS];

  // Servo
  Servo _servo;
  bool _servo_attached = false;
  uint8_t _servo_pin;
};

inline void show_vehicle_data(const vehicle_constants &vehicle) {
  Serial.printf("L_car: %.2f cm\n", vehicle.L_car);
  Serial.printf("psi_max: %.2f degrees\n", vehicle.psi_max);
  Serial.printf("v: %.2f cm/s\n", vehicle.v);
  Serial.printf("r_turn_min: %.2f cm\n", vehicle.r_turn_min);
  Serial.println("Vehicle data displayed.");
  Serial.println("-------------------------------------------------");
}

inline float encoder_ticks_to_cm(const int32_t ticks) {
  return (float)ticks * CM_POR_TICK;
}

inline int cm_to_encoder_ticks(const float distance_cm) {
  return (int)(distance_cm / CM_POR_TICK);
}

#endif
