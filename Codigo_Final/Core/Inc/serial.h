/*
 * serial.h
 *
 *  Created on: Nov 1, 2022
 *      Author: User
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "globals.h"
#include "stdio.h"
#include "string.h"

void Serial_Print(char[],char);
void Serial_Println(char[],char);
void Enviar_Dato_Float(float,char,uint8_t);
void Enviar_Caracter(char[],char);
void Escribir_Dato_Float(float,uint8_t,char[]);
void Escribir_Caracter(char[],char[]);

#endif /* INC_SERIAL_H_ */
