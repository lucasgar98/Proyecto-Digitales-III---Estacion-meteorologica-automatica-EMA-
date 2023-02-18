/*
 * veleta.h
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#ifndef INC_VELETA_H_
#define INC_VELETA_H_

#include "globals.h"
#include "serial.h"
#include "stdio.h"
#include "string.h"

enum DIR_VIENTO{N,NE,E,SE,S,SW,W,NW};

float Prom_Muestras_Veleta(uint8_t);
uint8_t Detectar_Dir_Viento(float);
void Enviar_Dir_Viento(uint8_t);

#endif /* INC_VELETA_H_ */
