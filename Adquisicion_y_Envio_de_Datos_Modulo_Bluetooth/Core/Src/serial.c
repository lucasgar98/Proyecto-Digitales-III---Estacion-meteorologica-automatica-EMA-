/*
 * serial.c
 *
 *  Created on: Nov 1, 2022
 *      Author: User
 */

#include "serial.h"


void Serial_Println(char msj[], char rxchar){

	uint8_t buf[40];
	char str1[40] = {'*',rxchar};
	char str2[3] = "\n*";

	strcat(str1,msj);
	strcat(str1,str2);

	sprintf((char*)buf,str1);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

void Serial_Println_Float(char msj[], char rxchar, float dat){

	uint8_t buf[40];
	char str1[40] = {'*',rxchar};
	char str2[3] = "\n*";

	strcat(str1,msj);
	strcat(str1,str2);

	sprintf((char*)buf,str1,dat);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}

void Enviar_Dato_Float(float data, char rxchar, uint8_t digitos){

	uint8_t buf[20];
	char str1[20] = {'*',rxchar};
	char str2[2] = {'*'};

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

void Enviar_Caracter(char car[],char rxchar){

	uint8_t buf[10];
	char str1[10] = {'*',rxchar};
	char str2[2] = {'*'};

	strcat(str1,"%s");
	strcat(str1,str2);

	sprintf((char*)buf,str1,car);
	HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
}
