#pragma once

// Constantes relacionadas con el automóvil
#define WHEEL_BASE 0.18f  // Distancia entre las ruedas en metros
#define MAX_RPM 100.0f    // RPM máximo de los motores
#define LENGTH 0.175f     // Longitud del automóvil en metros
#define WHEEL_RADIUS 0.035f // Radio de las ruedas en metros
#define DELTA_MAX 0.5236f // Desviación máxima en radianes (30 grados)
#define PATH_RADIUS 0.35f // Radio de la trayectoria en metros



#define ENCODER_PULSES_PER_REV 1320 // Pulsos del encoder por revolución
#define DT_RPM_MEASUREMENT 0.01f   // Intervalo de tiempo para la medición de RPM en segundos

#define ALPHA 0.85f // Coeficiente de filtrado para RPM
#define BETHA 0.0728f // Coeficiente de filtrado para RP


#define MPU_JITTER_THRESHOLD 0.025f // Umbral de jitter para el MPU en radianes
#define MPU_CALIBRATION_SAMPLES 200  // Número de muestras para la calibración del MPU
#define MPU_MOVING_THRESHOLD 0.03f    // Umbral de movimiento para el MPU en radianes


