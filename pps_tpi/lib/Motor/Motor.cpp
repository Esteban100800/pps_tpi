#include "Motor.h"

// Inicializar variables estáticas
Motor* Motor::motorInstances[MAX_MOTORS] = {nullptr, nullptr, nullptr, nullptr};
int Motor::motorCount = 0;

Motor::Motor(int ia1, int ia2, int pwm, int channel, int encoder) 
    : ia1Pin(ia1), ia2Pin(ia2), pwmPin(pwm), pwmChannel(channel), 
      encoderPin(encoder), pulseCount(0), dt(0.02), filteredRPM(0), alpha(0.005) {
    
    // Asignar un índice a este motor
    if (motorCount < MAX_MOTORS) {
        motorIndex = motorCount;
        motorInstances[motorIndex] = this;
        motorCount++;
        Serial.printf("Motor %d creado en pins IA1=%d, IA2=%d, PWM=%d, ENC=%d\n", 
                     motorIndex, ia1, ia2, pwm, encoder);
    } else {
        Serial.println("ERROR: Demasiados motores creados!");
        motorIndex = -1;
    }
    
    mux = portMUX_INITIALIZER_UNLOCKED; // Inicializar mutex
}
void Motor::begin() {
  pinMode(ia1Pin, OUTPUT);
  pinMode(ia2Pin, OUTPUT);
  digitalWrite(ia1Pin, LOW);
  digitalWrite(ia2Pin, LOW);

  ledcSetup(pwmChannel, 5000, 8);
  delay(300);

  pinMode(encoderPin, INPUT_PULLUP);
  
  // Asignar la función de interrupción correspondiente según el índice
  switch(motorIndex) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(encoderPin), globalEncoderISR0, RISING);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(encoderPin), globalEncoderISR1, RISING);
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(encoderPin), globalEncoderISR2, RISING);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(encoderPin), globalEncoderISR3, RISING);
      break;
    default:
      Serial.println("ERROR: Índice de motor inválido para interrupción");
  }
  
  Serial.printf("Motor %d inicializado con interrupción en pin %d\n", motorIndex, encoderPin);
}

void Motor::setPWM(int pwm) {
    pwm = constrain(pwm, -255, 255);
    
    if (pwm > 0) {
        // Adelante: IA1=PWM, IA2=LOW
        digitalWrite(ia2Pin, LOW);          // Fijar primero LOW
        ledcAttachPin(ia1Pin, pwmChannel);  // Asignar PWM a IA1
        ledcWrite(pwmChannel, pwm);         // Escribir valor PWM
    } 
    else if (pwm < 0) {
        // Atrás: IA2=PWM, IA1=LOW
        digitalWrite(ia1Pin, LOW);          // Fijar primero LOW
        ledcAttachPin(ia2Pin, pwmChannel);  // Asignar PWM a IA2
        ledcWrite(pwmChannel, -pwm);        // Escribir valor absoluto
    } 
    else {
        // Detener
        digitalWrite(ia1Pin, LOW);
        digitalWrite(ia2Pin, LOW);
        ledcWrite(pwmChannel, 0);
    }
}

void Motor::moveForward(int pwm) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(ia1Pin, HIGH);
    digitalWrite(ia2Pin, LOW);
    ledcWrite(pwmChannel, pwm);
}

void Motor::moveBackward(int pwm) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(ia1Pin, LOW);
    digitalWrite(ia2Pin, HIGH);
    ledcWrite(pwmChannel, pwm);
}
void Motor::stop() {
    digitalWrite(ia1Pin, LOW);
    digitalWrite(ia2Pin, LOW);
    ledcWrite(pwmChannel, 0);
}



float Motor::getRPM() {
  float rawRPM = (pulseCount / PPR) / dt * 60.0;
  filteredRPM = (0.854) * filteredRPM + (0.0728) * rawRPM + (0.0728) * filteredRPM; //ralentiza el cambio de RPM
  //filteredRPM = (1 - alpha) * filteredRPM + alpha * rawRPM; // Suavizado exponencial
  return filteredRPM;
}


void Motor::resetEncoder() {
  pulseCount = 0;
}

void Motor::encoderISR() {
  portENTER_CRITICAL_ISR(&mux);
  pulseCount++;
  portEXIT_CRITICAL_ISR(&mux);
}

// Funciones de interrupción individuales para cada motor
void IRAM_ATTR Motor::globalEncoderISR0() {
  if (motorInstances[0]) motorInstances[0]->encoderISR();
}

void IRAM_ATTR Motor::globalEncoderISR1() {
  if (motorInstances[1]) motorInstances[1]->encoderISR();
}

void IRAM_ATTR Motor::globalEncoderISR2() {
  if (motorInstances[2]) motorInstances[2]->encoderISR();
}

void IRAM_ATTR Motor::globalEncoderISR3() {
  if (motorInstances[3]) motorInstances[3]->encoderISR();
}
