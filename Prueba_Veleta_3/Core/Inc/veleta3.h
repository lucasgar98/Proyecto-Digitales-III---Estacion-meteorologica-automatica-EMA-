/*
 * veleta3.h
 *
 *  Created on: 9 nov. 2022
 *      Author: User
 */

#ifndef INC_VELETA3_H_
#define INC_VELETA3_H_

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define positivo 0
#define negativo 1
#define M 120

enum DIR{
	N = 0,
	NE = -45,
	E = -90,
	SE = -135,
	S = 180,
	SW = 135,
	W = 90,
	NW = 45,
};

/* Definimos una estructura para guardar como vectores el ángulo de la dirección del viento muestreada junto
 * con el signo del ángulo */
typedef struct ANG_DIRV{
	uint8_t ang[M];
	_Bool sig[M];
} ANG_DIRV;

float Prom_Muestras_Veleta(uint8_t);
ANG_DIRV Detectar_Angulo(float);
float Promedio_Angulo(uint8_t[]);
uint8_t Contar_Signos(_Bool[],_Bool);
int16_t Obtener_Dir_Viento(float,uint8_t,uint8_t);
void Enviar_Dir_Viento(int16_t);

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern _Bool flag_muestras; // Variable bandera que se activa cada 1 ms
extern _Bool flag_conversion; // Variable bandera que se activa cada 1 min

#endif /* INC_VELETA3_H_ */
