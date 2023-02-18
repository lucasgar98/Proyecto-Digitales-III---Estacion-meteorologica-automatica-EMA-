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

typedef struct FECHA {
	char dia[3];
	char mes[3];
	char ano[3];
} FECHA;

typedef struct HORA {
	char horas[3];
	char minutos[3];
	char segundos[3];
} HORA;

FECHA Obtener_Fecha_Actual(void);
HORA Obtener_Hora_Actual(void);
void Enviar_Fecha_Actual(FECHA*,char);
void Enviar_Hora_Actual(HORA*,char);

#endif /* INC_RTC_H_ */
