/*
 * serial.c
 *
 *  Created on: Nov 1, 2022
 *      Author: User
 */

#include "serial.h"

/* Función que envía un mensaje a través del puerto serie (UART) para que pueda visualizarse en el monitor serie de
 * la aplicación móvil */
void Serial_Print(char msj[], char rxchar){

	uint8_t buf[40];
	char str1[40] = {'*',rxchar};
	char str2[2] = {'*'};

	/* El mensaje a enviar deberá tener el formato: *<Caracter de recepción><Mensaje a transmitir>*. Por ejemplo:
	 * *AIngrese la hora actual* */
	strcat(str1,msj); // Concatenamos el mensaje a enviar con el asterisco inicial más el caracter de recepción
	strcat(str1,str2); // Concatenamos el mensaje con el asterisco final

	sprintf((char*)buf,str1);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

/* Función que también envía un mensaje a través del puerto serie, pero con un salto de línea */
void Serial_Println(char msj[], char rxchar){

	uint8_t buf[40];
	char str1[40] = {'*',rxchar};
	char str2[3] = "\n*"; // Agregamos el caracter de salto de línea \n

	strcat(str1,msj);
	strcat(str1,str2);

	sprintf((char*)buf,str1);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

/* Envía un dato tipo float al módulo Bluetooth, para que pueda visualizarse en el panel de la app. Es utilizada
 * para mostrar por pantalla todos los datos meteorológicos, excepto la dirección del viento */
void Enviar_Dato_Float(float data, char rxchar, uint8_t digitos){

	uint8_t buf[20];
	char str1[20] = {'*',rxchar};
	char str2[2] = {'*'};

	/* Definimos el "formateador" de la función sprintf dependiendo de la cantidad de dígitos a mostrar en pantalla */
	switch(digitos){
	case 1:{
		strcat(str1,"%1.1f");
	}
	break;
	case 2:{
		strcat(str1,"%2.1f");
	}
	break;
	case 3:{
		strcat(str1,"%3.1f");
	}
	break;
	case 4:{
		strcat(str1,"%4.1f");
	}
	break;
	}
	strcat(str1,str2);

	sprintf((char*)buf,str1,data);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

/* Rutina que permite enviar una cadena de caracteres corta. Es utilizada para mostrar en pantalla la dirección
 * del viento */
void Enviar_Caracter(char car[],char rxchar){

	uint8_t buf[10];
	char str1[10] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,"%s");
	strcat(str1,str2);

	sprintf((char*)buf,str1,car);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

/* Rutina que escribe un tipo de dato flotante en la memoria SD */
void Escribir_Dato_Float(float data, uint8_t cantdigitos, char carfinal[]){

	uint8_t buffer[10];
	uint32_t output;
	char str[10];

	/* Definimos el "formateador" de la función sprintf dependiendo de la cantidad de dígitos a mostrar en pantalla */
	switch(cantdigitos){
	case 1:{
		strcpy(str,"%1.1f");
		strcat(str,carfinal);
	}
	break;
	case 2:{
		strcpy(str,"%2.1f");
		strcat(str,carfinal);
	}
	break;
	case 3:{
		strcpy(str,"%3.1f");
		strcat(str,carfinal);
	}
	break;
	case 4:{
		strcpy(str,"%4.1f");
		strcat(str,carfinal);
	}
	break;
	}

	sprintf((char*)buffer,str,data);

	if (f_write(&USERFile,buffer,strlen((const char*)buffer),(void*)&output)==FR_OK){
		Serial_Print("R0G255B0",'C'); // Encendemos un indicador verde si el archivo se escribió correctamente
		if(strcmp(carfinal,"\n")==0){
			 if (f_sync(&USERFile)==FR_OK){
				 f_close(&USERFile);
			 }
		}
	}
	else {
		Serial_Print("R255G0B0",'C'); // Encendemos un indicador rojo si el archivo no se pudo escribir
	}

}

/* Función que permite escribir un caracter en la memoria SD */
void Escribir_Caracter(char car[],char carfinal[]){

	uint8_t buffer[5];
	uint32_t output;

	strcat(car,carfinal);

	sprintf((char*)buffer,"%s",car);

	if (f_write(&USERFile,buffer,strlen((const char*)buffer),(void*)&output)==FR_OK){
		Serial_Print("R0G255B0",'C'); // Encendemos un indicador verde si el archivo se escribió correctamente
		if(strcmp(carfinal,"\n")==0){
			if (f_sync(&USERFile)==FR_OK){
				f_close(&USERFile);
			}
	    }
	}
	else {
		Serial_Print("R255G0B0",'C'); // Encendemos un indicador rojo si el archivo no se pudo escribir
	}
}
