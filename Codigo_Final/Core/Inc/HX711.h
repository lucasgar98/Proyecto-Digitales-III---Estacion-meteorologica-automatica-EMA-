/*
 * HX711.h
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "globals.h"
#include "math.h"
#include "serial.h"

#define P1 0.0
#define P2 500.0 // Peso del objeto utilizado para la calibración del sensor (botella de agua de 500 ml)
#define d 10.6 // Diámetro del recipiente del pluviómetro (en cm)
#define pi 3.141592654 // Número pi

/* Variable estructura que contiene los parámetros de la recta de aproximación lineal del peso */
typedef struct HX711_DATA {
	float m; // Pendiente de la recta de aproximación lineal del peso
	float b; // Ordenada al origen de la recta de aprox. lineal del peso
} HX711_DATA;

void Delay_Microseconds(uint8_t);
uint32_t HX711_Read_Value(_Bool);
uint32_t HX711_Read_Average(uint8_t);
void HX711_Set_Scale(uint8_t);
float Leer_Pluviometro(void);
void HX711_Set_Tare(uint8_t);

#endif /* INC_HX711_H_ */
