/*
 * RTC.c
 *
 *  Created on: Oct 26, 2022
 *      Author: User
 */

#include "RTC.h"

FECHA Obtener_Fecha_Actual(void){

	RTC_DateTypeDef fecha1; // Variable tipo estructura para guardar la fecha actual
	FECHA fecha2;

	char dia1[2];
	char dia2[3];
	char mes1[2];
	char mes2[3];
	char ano1[2];
	char ano2[3];

	HAL_RTC_GetDate(&hrtc, &fecha1, RTC_FORMAT_BCD); // Obtenemos la fecha actual (en formato binario)

    utoa((fecha1.Date & 0x0F),dia1,10); // Dígito menos significativo del día
    utoa(((fecha1.Date & 0xF0)>>4),dia2,10); // Dígito más significativo del día
    strcat(dia2,dia1);
    utoa((fecha1.Month & 0x0F),mes1,10); // Dígito menos significativo del mes
    utoa(((fecha1.Month & 0xF0)>>4),mes2,10); // Dígito más significativo del mes
    strcat(mes2,mes1);
    utoa((fecha1.Year & 0x0F),ano1,10); // Dígito menos significativo del año
    utoa(((fecha1.Year & 0xF0)>>4),ano2,10); // Dígito más significativo del año
    strcat(ano2,ano1);

    strcpy(fecha2.dia,dia2);
    strcpy(fecha2.mes,mes2);
    strcpy(fecha2.ano,ano2);

    return fecha2;
}

HORA Obtener_Hora_Actual(void){

	RTC_TimeTypeDef hora1; // Variable tipo estructura para guardar la hora actual
	HORA hora2;

	char horas1[2];
	char horas2[3];
	char minutos1[2];
	char minutos2[3];
	char segundos1[2];
	char segundos2[3];

	HAL_RTC_GetTime(&hrtc, &hora1, RTC_FORMAT_BCD); // Obtenemos la hora actual (en formato binario)

    utoa((hora1.Hours & 0x0F),horas1,10); // Dígito menos significativo de las horas
    utoa(((hora1.Hours & 0xF0)>> 4),horas2,10); // Dígito más significativo de las horas
    strcat(horas2,horas1);
    utoa((hora1.Minutes & 0x0F),minutos1,10); // Dígito menos significativo de los minutos
    utoa(((hora1.Minutes & 0xF0)>>4),minutos2,10); // Dígito más significativo de los minutos
    strcat(minutos2,minutos1);
    utoa((hora1.Seconds & 0x0F),segundos1,10); // Dígito menos significativo de los segundos
    utoa(((hora1.Seconds & 0xF0)>>4),segundos2,10); // Dígito más significativo de los segundos
    strcat(segundos2,segundos1);

    strcpy(hora2.horas,horas2);
    strcpy(hora2.minutos,minutos2);
    strcpy(hora2.segundos,segundos2);

    return hora2;
}

void Enviar_Fecha_Actual(FECHA *fecha,char rxchar){

	uint8_t buf[20];
	char str1[20] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,"%s/%s/%s");
	strcat(str1,str2);

	sprintf((char*)buf,str1,fecha->dia,fecha->mes,fecha->ano);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

void Enviar_Hora_Actual(HORA *hora, char rxchar){

	uint8_t buf[20];
	char str1[20] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,"%s:%s");
	strcat(str1,str2);

	sprintf((char*)buf,str1,hora->horas,hora->minutos);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}
