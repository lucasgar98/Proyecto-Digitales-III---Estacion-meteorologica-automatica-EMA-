/*
 * RTC.h
 *
 *  Created on: Oct 26, 2022
 *      Author: User
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "globals.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "serial.h"

/* Variable tipo estructura para guardar la fecha actual leída del RTC en formato de cadena de caracteres */
typedef struct FECHA {
	char dia[3];
	char mes[3];
	char ano[3];
} FECHA;

/* Variable tipo estructura para guardar la hora actual leída del RTC en formato de cadena de caracteres */
typedef struct HORA {
	char horas[3];
	char minutos[3];
	char segundos[3];
} HORA;

FECHA Obtener_Fecha_Actual(void);
HORA Obtener_Hora_Actual(void);
void Enviar_Fecha_Actual(FECHA*,char);
void Enviar_Hora_Actual(HORA*,char);
void Escribir_Fecha_Actual(FECHA*);
void Escribir_Hora_Actual(HORA*);
void Ajustar_Hora_Actual(void);
void Ajustar_Fecha_Actual(void);

#endif /* INC_RTC_H_ */
