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

void Serial_Println(char[],char);
void Serial_Println_Float(char[],char,float);
void Enviar_Dato_Float(float,char,uint8_t);
void Enviar_Caracter(char[],char);

#endif /* INC_SERIAL_H_ */
