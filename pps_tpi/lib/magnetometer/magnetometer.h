#pragma once

#include <Arduino.h>
#include <Wire.h>   // Include the Wire library for I2C communication   

// Dirección del magnetómetro
const int MAG_ADDR = 0x0D;

// Declaraciones (NO definiciones)
extern int16_t magX_raw, magY_raw, magZ_raw;
extern float  magX_Gauss, magY_Gauss, magZ_Gauss;
extern float  magMinX ,magMinY , magMinZ;
extern float  magMaxX, magMaxY, magMaxZ;
extern float  declination;

// Funciones

/*
    * mag_signals: Obtiene las señales del magnetómetro y las normaliza.
    * 
    * Esta función lee los datos del magnetómetro a través de I2C, calcula los valores
    * normalizados en Gauss y actualiza las variables globales correspondientes.
    * 
    * RETURN: void   
    
*/
  
void mag_signals();

/*
    * print_mag_status: Imprime el estado del magnetómetro en el monitor serie.
    * 
    * Esta función muestra los valores de los ejes X, Y y Z del magnetómetro en Gauss.
    * 
    * RETURN: void
*/
void print_mag_status();
/*
    * dataReady: Verifica si hay nuevos datos disponibles del magnetómetro.
    * 
    * Esta función consulta el registro de estado del magnetómetro para determinar si
    * hay nuevos datos listos para ser leídos.
    * 
    * RETURN: bool - true si hay datos disponibles, false en caso contrario.
*/
bool data_ready();

/*
    * calibrate_mag: Realiza la calibración del magnetómetro.
    * 
    * Esta función lee los datos del magnetómetro durante un período de tiempo y
    * determina los valores mínimos y máximos para cada eje, que se utilizan para
    * normalizar las lecturas.
    * 
    * RETURN: void
*/
void calibrate_mag();

/*
    * print_heading: Calcula e imprime el rumbo (heading) del magnetómetro.
    * 
    * Esta función calcula el rumbo en grados a partir de las lecturas del magnetómetro
    * y lo ajusta por la declinación magnética antes de imprimirlo en el monitor serie.
    * 
    * RETURN: void
*/
void print_heading();

/*
    * config_magnetometer: Configura el magnetómetro para su uso.
    * 
    * Esta función inicializa la comunicación I2C, configura los registros del
    * magnetómetro y realiza la calibración inicial.
    * 
    * RETURN: void
*/
void config_magnetometer();
