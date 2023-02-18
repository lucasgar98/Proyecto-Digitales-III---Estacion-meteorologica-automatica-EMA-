/*
 * anemometro.h
 *
 *  Created on: 9 nov. 2022
 *      Author: User
 */

#ifndef INC_ANEMOMETRO_H_
#define INC_ANEMOMETRO_H_

#include "stm32f1xx_hal.h"

#define INT_PULSOS 5.0 // Intervalo de tiempo en el que se cuentan los pulsos del anemómetro
#define FACTOR_CONV 2.4 // Factor de conversión para obtener la velocidad del viento
#define N 12

float Obtener_Vel_Viento(void);
float Promedio_Muestras(float[]);

extern uint16_t contpulsos;

#endif /* INC_ANEMOMETRO_H_ */
