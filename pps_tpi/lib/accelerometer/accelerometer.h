# pragma once

#include <Arduino.h>
#include <Wire.h>   


const int MPU_ADDR = 0x68; // I2C address of the MPU-6050


extern int16_t GyX, GyY, GyZ;
extern float rateRoll, ratePitch, rateYaw, calibRoll, calibPitch, calibYaw;
extern int calibrationCount;



/*
    * gyro_signals: Obtiene las señales del giroscopio y las normaliza.
    * 
    * Esta función lee los datos del giroscopio a través de I2C, calcula los valores
    * normalizados en grados por segundo y actualiza las variables globales correspondientes.
    * 
    * RETURN: void   
*/
void gyro_signals();

/*
    * calibration: Realiza la calibración del giroscopio.
    * 
    * Esta función lee los datos del giroscopio durante un período de tiempo y
    * calcula el promedio de las lecturas para corregir el error de sesgo.
    * 
    * RETURN: void
*/
void calibrate_gyro();

/*
    * print_status: Imprime el estado del giroscopio en el monitor serie.
    * 
    * Esta función muestra los valores de los ejes X, Y y Z del giroscopio en grados por segundo.
    * 
    * RETURN: void
*/
void print_status();

/*
    * config_gyro: Configura el giroscopio para su uso.
    * 
    * Esta función inicializa el giroscopio configurando sus registros necesarios.
    * 
    * RETURN: void
*/
void config_gyro();
