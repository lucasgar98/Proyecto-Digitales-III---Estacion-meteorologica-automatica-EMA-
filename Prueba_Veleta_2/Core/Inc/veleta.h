/*
 * veleta.h
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#ifndef INC_VELETA_H_
#define INC_VELETA_H_

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"

enum DIR_VIENTO{N,NE,E,SE,S,SW,W,NW};

float Prom_Muestras_Veleta(uint8_t);
uint8_t Detectar_Dir_Viento(float);
void Enviar_Dir_Viento(uint8_t);

extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern _Bool flag_muestreo,flag_conversion;
extern UART_HandleTypeDef huart1;

#endif /* INC_VELETA_H_ */
