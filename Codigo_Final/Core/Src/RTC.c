/*
 * RTC.c
 *
 *  Created on: Oct 26, 2022
 *      Author: User
 */

#include "RTC.h"

/* Función para obtener la fecha actual mediante una lectura del RTC. La fecha se guarda en una
 * estructura */
FECHA Obtener_Fecha_Actual(void){

	RTC_DateTypeDef fecha1; // Variable tipo estructura para recibir la fecha actual del RTC
	FECHA fecha2; // Variable para guardar la fecha actual en formato de string

	char dia1[2];
	char dia2[3];
	char mes1[2];
	char mes2[3];
	char ano1[2];
	char ano2[3];

	HAL_RTC_GetDate(&hrtc, &fecha1, RTC_FORMAT_BCD); // Obtenemos la fecha actual (en formato binario)

    utoa((fecha1.Date & 0x0F),dia1,10); // Dígito menos significativo del día
    utoa(((fecha1.Date & 0xF0)>>4),dia2,10); // Dígito más significativo del día
    strcat(dia2,dia1); // Concatenamos los dos dígitos del día
    utoa((fecha1.Month & 0x0F),mes1,10); // Dígito menos significativo del mes
    utoa(((fecha1.Month & 0xF0)>>4),mes2,10); // Dígito más significativo del mes
    strcat(mes2,mes1); // Concatenamos los dos dígitos del mes
    utoa((fecha1.Year & 0x0F),ano1,10); // Dígito menos significativo del año
    utoa(((fecha1.Year & 0xF0)>>4),ano2,10); // Dígito más significativo del año
    strcat(ano2,ano1); // Concatenamos los dos dígitos del año

    /* Copiamos el día, mes y año a la variable tipo FECHA */
    strcpy(fecha2.dia,dia2);
    strcpy(fecha2.mes,mes2);
    strcpy(fecha2.ano,ano2);

    return fecha2;
}

/* Rutina para obtener la hora actual mediante una lectura del RTC */
HORA Obtener_Hora_Actual(void){

	RTC_TimeTypeDef hora1; // Variable tipo estructura para guardar la hora actual leída del RTC
	HORA hora2; // Variable para guardar la hora actual en formato string

	char horas1[2];
	char horas2[3];
	char minutos1[2];
	char minutos2[3];
	char segundos1[2];
	char segundos2[3];

	HAL_RTC_GetTime(&hrtc, &hora1, RTC_FORMAT_BCD); // Obtenemos la hora actual (en formato binario)

    utoa((hora1.Hours & 0x0F),horas1,10); // Dígito menos significativo de las horas
    utoa(((hora1.Hours & 0xF0)>> 4),horas2,10); // Dígito más significativo de las horas
    strcat(horas2,horas1); // Concatenamos los dos dígitos de las horas
    utoa((hora1.Minutes & 0x0F),minutos1,10); // Dígito menos significativo de los minutos
    utoa(((hora1.Minutes & 0xF0)>>4),minutos2,10); // Dígito más significativo de los minutos
    strcat(minutos2,minutos1); // Concatenamos los dígitos de los minutos
    utoa((hora1.Seconds & 0x0F),segundos1,10); // Dígito menos significativo de los segundos
    utoa(((hora1.Seconds & 0xF0)>>4),segundos2,10); // Dígito más significativo de los segundos
    strcat(segundos2,segundos1); // Concatenamos los dos dígitos de los segundos

    strcpy(hora2.horas,horas2);
    strcpy(hora2.minutos,minutos2);
    strcpy(hora2.segundos,segundos2);

    return hora2;
}

/* Envía la fecha actual al módulo Bluetooth a través del puerto serie */
void Enviar_Fecha_Actual(FECHA *fecha,char rxchar){

	uint8_t buf[20];
	char str1[20] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,"%s/%s/%s");
	strcat(str1,str2);

	sprintf((char*)buf,str1,fecha->dia,fecha->mes,fecha->ano);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

/* Envía la hora actual al módulo Bluetooth a través del puerto serie */
void Enviar_Hora_Actual(HORA *hora, char rxchar){

	uint8_t buf[20];
	char str1[20] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,"%s:%s");
	strcat(str1,str2);

	sprintf((char*)buf,str1,hora->horas,hora->minutos);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

/* Rutina que permite guardar la fecha actual en la memoria SD */
void Escribir_Fecha_Actual(FECHA *fecha){

	uint8_t buf[40];
	uint32_t output;

	sprintf((char*)buf,"%s/%s/%s ",fecha->dia,fecha->mes,fecha->ano);
	if (f_write(&USERFile,buf,strlen((const char*)buf),(void*)&output)==FR_OK){
		Serial_Print("R0G255B0",'C'); // Encendemos un indicador verde si el archivo se escribió correctamente
	}
	else {
		Serial_Print("R255G0B0",'C'); // Encendemos un indicador rojo si el archivo no se pudo escribir
	}
}

/* Función que guarda la hora actual en la memoria SD */
void Escribir_Hora_Actual(HORA *hora){

	uint8_t buf[40];
	uint32_t output;

	sprintf((char*)buf,"%s:%s ",hora->horas,hora->minutos);
	  if (f_write(&USERFile,buf,strlen((const char*)buf),(void*)&output)==FR_OK){
		  Serial_Print("R0G255B0",'C'); // Encendemos un indicador verde si el archivo se escribió correctamente
	  }
	  else {
		  Serial_Print("R255G0B0",'C'); // Encendemos un indicador rojo si el archivo no se pudo escribir
	  }
}

/* Rutina que permite al usuario ingresar la hora actual a través de la aplicación móvil para el ajuste del RTC */
void Ajustar_Hora_Actual(void){

	uint8_t buffer[5]; // Variable para recibir la hora actual (5 caracteres)
	uint8_t h,m; // Variables para guardar las horas y minutos recibidos en formato decimal
	RTC_TimeTypeDef sTime = {0}; // Variable para el ajuste del RTC

	/* Solicitamos al usuario que ingrese la hora actual. El programa esperará indefinidamente hasta que el usuario
	 * ingrese la hora actual en formato correcto (HH:MM) */
	Serial_Println("Ingrese la hora actual",'A');
	while(HAL_UART_Receive(&huart1, buffer, 5, 500) != HAL_OK){
	}

	/*Convertimos los caracteres recibidos de ASCII a decimal para obtener la hora ingresada */
	h=10*(buffer[0]-48)+(buffer[1]-48);
	m=10*(buffer[3]-48)+(buffer[4]-48);

	/* Si la hora ingresada no es válida, debemos solicitar al usuario que vuelva a ingresar la hora actual. Esto
	 * ocurrirá hasta que el usuario ingrese la hora correctamente */
	while(h>23 || m>59){
		Serial_Println("Hora no válida",'A');
		Serial_Println("Ingrese la hora nuevamente",'A');
		while(HAL_UART_Receive(&huart1, buffer, 5, 500) != HAL_OK){
		}
		h=10*(buffer[0]-48)+(buffer[1]-48);
	    m=10*(buffer[3]-48)+(buffer[4]-48);
	}

	/* Debemos convertir las horas de decimal a hexadecimal, ya que en el campo Hours de la estructura RTC_TimeTypeDef
	 * las horas se guarda en hexadecimal */
	if(h<10){
		sTime.Hours = h;
	}
	else if(h>=10 && h<20){
		sTime.Hours = h+6;
	}
	else {
		sTime.Hours = h+12;
	}

	/* Debemos convertir los minutos de decimal a hexadecimal, ya que en el campo Minutes de la estructura
	 * RTC_TimeTypeDef los minutos se guardan en hexadecimal */
	if(m<10){
		sTime.Minutes = m;
	}
	else if(m>=10 && m<20){
		sTime.Minutes = m+6;
	}
	else if(m>=20 && m<30){
		sTime.Minutes = m+12;
	}
	else if(m>=30 && m<40){
		sTime.Minutes = m+18;
	}
	else if(m>=40 && m<50){
		sTime.Minutes = m+24;
	}
	else {
		sTime.Minutes = m+30;
	}

	sTime.Seconds = 0;

	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD); // Ajustamos la hora en el RTC

	Serial_Println("Hora ajustada",'A');
}

/* Función que permite al usuario ingresar la fecha actual de la app del teléfono para el ajuste del RTC */
void Ajustar_Fecha_Actual(void){

	uint8_t buffer[8]; // Vector para recibir la fecha actual (8 caracteres)
	uint8_t buffer2[3]; // Vector para recibir el día de la semana (3 caracteres)
    uint8_t d,m,a; // Variables para guardar el día, mes y año en formato decimal
    uint8_t wd1,wd2,wd3; // Variables para guardar el equivalente en ASCII de las letras recibidas de los días de la semana
    uint8_t wd; // Variable para guardar el día de la semana como un número del 0 al 6
    _Bool flag_date=0; // Bandera que se mantiene en 0 cuando el día de la semana ingresado es inválido

	RTC_DateTypeDef DateToUpdate = {0}; // Variable para el ajuste del RTC

	/* Solicitamos al usuario que ingrese la fecha actual. El programa se quedará esperando a que el usuario ingrese la
	 * fecha actual */
	Serial_Print("Ingrese la fecha actual",'A');
	while(HAL_UART_Receive(&huart1, buffer, 8, 50) != HAL_OK){
	}

	/*Convertimos los caracteres recibidos de ASCII a decimal para obtener la fecha ingresada */
	d=10*(buffer[0]-48)+(buffer[1]-48);
	m=10*(buffer[3]-48)+(buffer[4]-48);
	a=10*(buffer[6]-48)+(buffer[7]-48);

	/* Si la fecha ingresada no es válida, solicitamos al usuario que vuelva a ingresar la fecha actual. Esto ocurrirá
	 * hasta que el usuario ingrese correctamente la fecha actual */
	while(d>31 || m>12 || a>99){
		Serial_Println("Fecha no válida",'A');
		Serial_Println("Ingrese la fecha nuevamente",'A');
		while(HAL_UART_Receive(&huart1, buffer, 8, 50) != HAL_OK){
		}
		d=10*(buffer[0]-48)+(buffer[1]-48);
		m=10*(buffer[3]-48)+(buffer[4]-48);
		a=10*(buffer[6]-48)+(buffer[7]-48);
	}

	/* Convertimos el día ingresados de decimal a hexadecimal para configurar el RTC */
	if(d<10){
		DateToUpdate.Date = d;
	}
	else if(d>=10 && d<20){
		DateToUpdate.Date = d+6;
	}
	else if(d>=20 && d<30){
		DateToUpdate.Date = d+12;
	}
	else {
		DateToUpdate.Date = d+18;
	}

	/* Convertimos el mes ingresado de decimal a hexadecimal para configurar el RTC */
	if(m<10){
		DateToUpdate.Month = m;
	}
	else {
		DateToUpdate.Month = m+6;
	}

	/* Convertimos el año ingresado de decimal a hexadecimal para configurar el RTC */
	if(a<10){
		DateToUpdate.Year = a;
	}
	else if(a>=10 && a<20){
		DateToUpdate.Year = a+6;
	}
	else if(a>=20 && a<30){
		DateToUpdate.Year = a+12;
	}
	else if(a>=30 && a<40){
		DateToUpdate.Year = a+18;
	}
	else if(a>=40 && a<50){
		DateToUpdate.Year = a+24;
	}
	else if(a>=50 && a<60){
		DateToUpdate.Year = a+30;
	}
	else if(a>=60 && a<70){
		DateToUpdate.Year = a+36;
	}
	else if(a>=70 && a<80){
		DateToUpdate.Year = a+42;
	}
	else if(a>=80 && a<90){
		DateToUpdate.Year = a+48;
	}
	else {
		DateToUpdate.Year = a+54;
	}

	/* Solicitamos al usuario que ingrese el día de la semana. El programa se quedará esperando hasta que el usuario
	 * ingrese el día de la semana */
	Serial_Println("Ingrese el dia de la semana",'A');
	while(HAL_UART_Receive(&huart1, buffer2, 3, 50) != HAL_OK){
	}
	wd1=buffer2[0]; // Primera letra del día de la semana (en ASCII)
	wd2=buffer2[1]; // Segunda letra del día de la semana (en ASCII)
	wd3=buffer2[2]; // Tercera letra del día de la semana (en ASCII)

	/* LUN - wd1=76 (L), wd2=85 (U), wd3=78 (N)
	 * MAR - wd1=77 (M), wd2=65 (A), wd3=82 (R)
	 * MIE - wd1=77 (M), wd2=73 (I), wd3=69 (E)
	 * JUE - wd1=74 (J), wd2=85 (U), wd3=69 (E)
	 * VIE - wd1=86 (V), wd2=73 (I), wd3=69 (E)
	 * SAB - wd1=83 (S), wd2=65 (A), wd3=66 (B)
	 * DOM - wd1=68 (D), wd2=79 (O), wd3=77 (M)
	 */

	/* Asignamos al día de la semana ingresado un número del 0 al 6. Si el día de la semana ingresado es incorrecto,
	 * volvemos a solicitar al usuario que lo vuelva a ingresar */
	while(flag_date==0){
		if(wd1==76 && wd2==85 && wd3==78){
			wd=RTC_WEEKDAY_MONDAY; // Día lunes (LUN)
			flag_date=1;
		}
		else if(wd1==77 && wd2==65 && wd3==82){
			wd=RTC_WEEKDAY_TUESDAY; // Día martes (MAR)
			flag_date=1;
		}
		else if(wd1==77 && wd2==73 && wd3==69){
			wd=RTC_WEEKDAY_WEDNESDAY; // Día miércoles (MIE)
			flag_date=1;
		}
		else if(wd1==74 && wd2==85 && wd3==69){
			wd=RTC_WEEKDAY_THURSDAY; // Día jueves (JUE)
			flag_date=1;
		}
		else if(wd1==86 && wd2==73 && wd3==69){
			wd=RTC_WEEKDAY_FRIDAY; // Día viernes (VIE)
			flag_date=1;
		}
		else if(wd1==83 && wd2==65 && wd3==66){
			wd=RTC_WEEKDAY_SATURDAY; // Día sábado (SAB)
			flag_date=1;
		}
		else if(wd1==68 && wd2==79 && wd3==77){
			wd=RTC_WEEKDAY_SUNDAY; // Día domingo (DOM)
			flag_date=1;
		}
		else {
			Serial_Println("Día de la semana no válido",'A');
			Serial_Println("Ingrese el día de semana nuevamente",'A');
			flag_date=0;
			while(HAL_UART_Receive(&huart1, buffer, 3, 1) != HAL_OK){
			}
			wd1=buffer[0]; // Primera letra del día de la semana (en ASCII)
			wd2=buffer[1]; // Segunda letra del día de la semana (en ASCII)
			wd3=buffer[2]; // Tercera letra del día de la semana (en ASCII)
		}
	}

	DateToUpdate.WeekDay = wd;

	HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD); // Ajuste de la fecha actual en el RTC

	Serial_Print("Fecha ajustada",'A');
}
