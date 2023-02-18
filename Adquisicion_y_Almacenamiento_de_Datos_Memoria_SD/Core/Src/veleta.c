/*
 * veleta.c
 *
 *  Created on: Oct 25, 2022
 *      Author: User
 */

#include "veleta.h"

/* Esta función toma muestras de tensión analógica cada 1 ms, y luego calcula el promedio */
float Prom_Muestras_Veleta(uint8_t cantmuestras){

	  uint16_t vol[cantmuestras]; // Tensión analógica muestreada en mV
	  uint32_t sum=0;
	  float prom;
	  uint8_t cant=0;

	  //HAL_TIM_Base_Start_IT(&htim3);
	  HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
	  while(cant<cantmuestras){
		  if(flag_muestras==1){
			  flag_muestras=0;
			  HAL_ADC_Start_IT(&hadc1);
		  }
		  else {
			  if(flag_conversion==1){
				  flag_conversion=0;
			      vol[cant]=HAL_ADC_GetValue(&hadc1)*3300/4095;
			      cant++;
			  }
		  }
	  }
	  HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
	  //HAL_TIM_Base_Stop_IT(&htim3);
	  for(int i=0;i<cantmuestras;i++){
		  sum=sum+vol[i];
	  }
	  prom=sum/cantmuestras;

	  return prom;
}

uint8_t Detectar_Dir_Viento(float volprom){

	uint8_t dirv;

	if(volprom<100){
		dirv=E;
	}
	else if(110<volprom && volprom<200){
		dirv=SE;
	}
	else if(210<volprom && volprom<350){
		dirv=S;
	}
	else if(400<volprom && volprom<700){
		dirv=NE;
	}
	else if(800<volprom && volprom<1100){
		dirv=SW;
	}
	else if(1200<volprom && volprom<1800){
		dirv=N;
	}
	else if(2000<volprom && volprom<2500){
		dirv=NW;
	}
	else if(3000<volprom){
		dirv=W;
	}

	return dirv;
}

void Enviar_Dir_Viento(uint8_t dir){

	//uint8_t buf[30];

    //sprintf((char*)buffer,"*ATensión analógica: %4.1f mV \n*",vprom);
	//HAL_UART_Transmit(&huart1, buffer, strlen((const char*)buffer),100);

	switch(dir){
	case N:{
		Enviar_Caracter("N",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: N\n*");
		//HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case NE:{
		Enviar_Caracter("NE",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: NE\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case E:{
		Enviar_Caracter("E",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: E\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case SE:{
		Enviar_Caracter("SE",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: SE\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case S:{
		Enviar_Caracter("S",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: S\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case SW:{
		Enviar_Caracter("SW",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: SW\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case W:{
		Enviar_Caracter("W",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: W\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	case NW:{
		Enviar_Caracter("NW",'D');
	    //sprintf((char*)buf,"*ADireccion del viento: NW\n*");
	    //HAL_UART_Transmit(&huart1, buf, strlen((const char*)buf),100);
	}
	break;
	}
}

void Escribir_Dir_Viento(uint8_t dir, char carfinal[]){

	uint8_t buffer[5];
	uint32_t output;
	char str[5];

	switch(dir){
	case N:{
		strcpy(str,"N");
		strcat(str,carfinal);
	}
	break;
	case NE:{
		strcpy(str,"NE");
		strcat(str,carfinal);
	}
	break;
	case E:{
		strcpy(str,"E");
		strcat(str,carfinal);
	}
	break;
	case SE:{
		strcpy(str,"SE");
		strcat(str,carfinal);
	}
	break;
	case S:{
		strcpy(str,"S");
		strcat(str,carfinal);
	}
	break;
	case SW:{
		strcpy(str,"SW");
		strcat(str,carfinal);
	}
	break;
	case W:{
		strcpy(str,"W");
		strcat(str,carfinal);
	}
	break;
	case NW:{
		strcpy(str,"NW");
		strcat(str,carfinal);
	}
	break;
	}

	sprintf((char*)buffer,"%s",str);

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
