/*
 * viento.h
 *
 *  Created on: 28 ene. 2023
 *      Author: User
 */

#ifndef INC_VIENTO_H_
#define INC_VIENTO_H_

#include "globals.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "serial.h"

#define positivo 0 // Signo del ángulo (positivo)
#define negativo 1 // Signo del ángulo (negativo)
#define INT_PULSOS 3.0 // Intervalo de tiempo en el que se cuentan los pulsos del anemómetro (3 segundos)
#define FACTOR_CONV 2.4 // Factor de conversión para obtener la velocidad del viento
#define N1 100 // Número de muestras de la dir. y vel. del viento a tomar en un intervalo de tiempo de 5 min

/* Variable estructura para guardar las muestras de la dirección y velocidad del viento */
typedef struct VIENTO{
	uint8_t angdir[N1]; // Ángulo de la dirección del viento
	_Bool sigdir[N1]; // Signo del ángulo (positivo o negativo)
	float velv[N1]; // Velocidad del viento (en km/h)
} VIENTO;

/* Definimos un tipo de dato enumerado, en el cual a cada dirección del viento se le asigna un ángulo de
 * -180° a +180° */
typedef enum DIR_VIENTO{
	N = 0,
	NE = 45,
	E = 90,
	SE = 135,
	S = -180,
	SW = -135,
	W = -90,
	NW = -45,
} DIR_VIENTO;

float Prom_Muestras_Veleta(uint8_t);
uint8_t Detectar_Angulo(float,_Bool);
float Promedio_Angulo(uint8_t[]);
uint8_t Contar_Signos(_Bool[],_Bool);
DIR_VIENTO Obtener_Dir_Viento(float,uint8_t,uint8_t);
void Enviar_Dir_Viento(DIR_VIENTO);
void Escribir_Dir_Viento(DIR_VIENTO,char[]);
float Promedio_Muestras_Anemometro(float[]);
void Tomar_Muestras_Viento(VIENTO*);

/* Variables externas definidas en el main necesarias para muestrear la dirección y la velocidad del viento */
extern _Bool flag_viento; // Variable bandera que se activa cada 1 ms
extern uint8_t cantmuestviento; // Variable que cuenta las muestras de viento
extern uint16_t contpulsos; // Variable que cuenta los pulsos del anemómetro

#endif /* INC_VIENTO_H_ */
